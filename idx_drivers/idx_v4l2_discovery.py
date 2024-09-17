#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
# ROS namespace setup
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import subprocess
import time
import rospy

from nepi_edge_sdk_base import nepi_nex

PKG_NAME = 'IDX_V4L2' # Use in display menus
FILE_TYPE = 'DISCOVERY'
CLASS_NAME = 'V4L2CamDiscovery' # Should Match Class Name
PROCESS = 'LAUNCH' # 'LAUNCH', 'RUN', or 'CALL'


TEST_NEX_DICT = {
    'group': 'IDX',
    'group_id': 'V4L2',
    'node_file_name': 'idx_v4l2_node.py',
    'node_file_path': '/opt/nepi/ros/lib/nepi_drivers',
    'node_module_name': 'idx_v4l2_node',
    'node_class_name': 'V4l2CamNode',
    'driver_name': 'IDX_V4L2',
    'driver_file_name': 'idx_v4l2_driver.py' ,
    'driver_file_path':  '/opt/nepi/ros/lib/nepi_drivers',
    'driver_module_name': 'idx_v4l2_driver' ,
    'driver_class_name':  'V4l2CamDriver',
    'driver_interfaces': ['USB'],
    'driver_options_1': [],
    'driver_default_option_1': 'None',
    'driver_set_option_1': 'None',
    'driver_options_2': [],
    'driver_default_option_2': 'None',
    'driver_set_option_2': 'None',
    'discovery_name': 'IDX_V4L2', 
    'discovery_file_name': 'idx_v4l2_discovery.py',
    'discovery_file_path': '/opt/nepi/ros/lib/nepi_drivers',
    'discovery_module_name': 'idx_v4l2_discovery',
    'discovery_class_name': 'V4L2CamDiscovery',
    'discovery_method': 'AUTO', 
    'discovery_ids': [],
    'discovery_ignore_ids': ['msm_vidc_vdec','ZED 2','ZED 2i','ZED-M'],
    'device_dict': {'device_path': '/dev/video0'},
    'order': 1,
    'active': True,
    'msg': ""
    }
                             
class V4L2CamDiscovery:


  NEPI_DEFAULT_CFG_PATH = '/opt/nepi/ros/etc/'
  CHECK_INTERVAL_S = 3.0


  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"    
  nex_dict = dict()
                               
  def __init__(self):
    # Launch the ROS node
    rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
    self.node_name = rospy.get_name().split('/')[-1]
    rospy.loginfo("Starting " + self.node_name)
    # Get nex_dict from param servers
    self.nex_dict = rospy.get_param('~nex_dict',TEST_NEX_DICT) 
    ################################################
    # Start discovery process

    self.deviceList = []
    self.includeDevices = rospy.get_param('~included_devices', self.nex_dict['discovery_ids'])
    self.excludedDevices = rospy.get_param('~excluded_devices', self.nex_dict['discovery_ignore_ids'])

    rospy.Timer(rospy.Duration(self.CHECK_INTERVAL_S), self.detectAndManageDevices)

    # Now start the node
    rospy.spin()



  def detectAndManageDevices(self, timer): # Extra arg since this is a rospy Timer callback
    # First grab the current list of known V4L2 devices
    sub_process = subprocess.Popen(['v4l2-ctl', '--list-devices'],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    stdout,_ = sub_process.communicate()
    #if sub_process.returncode != 0:  # This can happen because there are no longer any video devices connected, so v4l2-ctl returns error
      #raise Exception("Failed to list v4l2 devices: " + stdout)
    out = stdout.splitlines()

    device_type = None
    device_path = None
    active_paths = list()
    nLines = len(out)
    for i in range(0, nLines):
      line = out[i].strip()
      if line.endswith(':'):
        device_type = line.split('(')[0].strip()
        # Some v4l2-ctl outputs have an additional ':'
        device_type = device_type.split(':')[0]
        
        # Honor the exclusion list
        if device_type in self.excludedDevices:
          device_type = None
          #continue

      elif line.startswith('/dev/video'):
        device_path = line
        

        # Make sure this is a legitimate Video Capture device, not a Metadata Capture device, etc.
        is_video_cap_device = False
        sub_process = subprocess.Popen(['v4l2-ctl', '-d', device_path, '--all'],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        stdout,_ = sub_process.communicate()
        all_out = stdout.splitlines()
        in_device_caps = False
        usbBus = None
        for all_line in all_out:
          #rospy.loginfo(all_line)
          if ('Bus info' in all_line):
            usbBus = all_line.rsplit("-",1)[1].replace(".","")
          if ('Device Caps' in all_line):
            in_device_caps = True
          elif in_device_caps:
            if ('Video Capture' in all_line):
              is_video_cap_device = True
          elif ':' in all_line:
            in_device_caps = False
        
        if not is_video_cap_device:
          continue

        active_paths.append(device_path) # To check later that the device list has no entries for paths that have disappeared
        known_device = False
        # Check if this device is already known and launched
        for device in self.deviceList:
          #rospy.loginfo("device exists check")
          #rospy.loginfo(device)
          #rospy.loginfo(active_paths)
          if device['device_class'] != 'v4l2':
            continue

          if device['device_path'] == device_path:
            known_device = True
            if device['device_type'] != device_type:
              # Uh oh -- device has switched on us!
              # Kill previous and start new?
              rospy.logwarn(self.node_name + ": detected V4L2 device type change (" + device['device_type'] + "-->" + 
                            device_type + ") for device at " + device_path)
              self.stopAndPurgeDeviceNode(device['node_namespace'])
              self.startDeviceNode(dtype = device_type, path = device_path, bus = usbBus)
            elif not self.deviceNodeIsRunning(device['node_namespace']):
              rospy.logwarn(self.node_name + ": node " + device['node_name'] + " is not running. Restarting")
              self.stopAndPurgeDeviceNode(device['node_namespace'])
              self.startDeviceNode(dtype = device_type, path = device_path, bus = usbBus)
            break

        if not known_device:
          self.startDeviceNode(dtype = device_type, path = device_path, bus = usbBus)

    # Handle devices which no longer have a valid active V4L2 device path

    for device in self.deviceList:
      if device['device_class'] != 'v4l2':
        continue

      if device['device_path'] not in active_paths:
        rospy.logwarn(self.node_name + ': ' + device['node_namespace'] + ' path ' + device['device_path'] + ' no longer exists... device disconnected?')
        self.stopAndPurgeDeviceNode(device['node_namespace'])

  def startDeviceNode(self, dtype, path, bus):
    # First, get a unique name
    if dtype is not None:
      if dtype not in self.excludedDevices:
        root_name = dtype.replace(' ','_').lower()

        same_type_count = 0
        for device in self.deviceList:
          if device['device_type'] == dtype:
            same_type_count += 1

        device_node_name = self.short_name(root_name)
        if bus is not None:
          id = bus
        else:
          id = str(same_type_count)
        device_node_name += '_' + id

        device_exists = False
        for device in self.deviceList:
          if device['node_name'] == device_node_name:
            device_exists = True

        if device_exists is False:
          device_node_namespace = rospy.get_namespace() + device_node_name
          rospy.loginfo(self.node_name + ": Initiating new V4L2 node " + device_node_namespace)

          self.checkLoadConfigFile(node_name = device_node_name)

          # Now start the node via rosrun
          # rosrun nepi_drivers_idx v4l2_camera_node.py __name:=usb_cam_1 _device_path:=/dev/video0
          rospy.loginfo(self.node_name + ": " + "Launching node " + device_node_name)
          if dtype not in self.nex_dict['discovery_ignore_ids']:
            #Setup required param server nex_dict for discovery node
            self.nex_dict['device_dict'] = {'device_path': path}
            dict_param_name = device_node_name + "/nex_dict"
            rospy.set_param(dict_param_name,self.nex_dict)
            file_name = self.nex_dict['node_file_name']
            #Try and launch node
            [success, msg, sub_process] = nepi_nex.launchDriverNode(file_name, device_node_name)
            if success:
              self.deviceList.append({'device_class': 'v4l2', 'device_path': path, 'device_type': dtype, 
                                      'node_name': device_node_name, 'node_namespace': device_node_namespace,
                                      'node_subprocess': sub_process})
            else:
              rospy.loginfo(msg)

  def stopAndPurgeDeviceNode(self, node_namespace):
    rospy.loginfo(self.node_name + ": stopping " + node_namespace)
    for i, device in enumerate(self.deviceList):
      if device['node_namespace'] == node_namespace:
        node_name = device['node_namespace'].split("/")[-1]
        sub_process = device['node_subprocess']
        success = nepi_nex.killDriverNode(node_name,sub_process)
        # And remove it from the list
        self.deviceList.pop(i)  
    if success == False:
      rospy.logwarn(self.node_name + ": Unable to stop unknown node " + node_namespace)

  def deviceNodeIsRunning(self, node_namespace):
    for device in self.deviceList:
      if device['node_namespace'] == node_namespace:
        if device['node_subprocess'].poll() is not None:
          return False
        else:
          return True
    # If we get here, didn't find the node in our list    
    rospy.logwarn(self.node_name + ": cannot check run status of unknown node " + node_namespace)
    return False
  
  def checkLoadConfigFile(self, node_name):
    folder_name = "drivers/" + node_name 
    config_folder = os.path.join(self.NEPI_DEFAULT_CFG_PATH, folder_name)
    if not os.path.isdir(config_folder):
      rospy.logwarn(self.node_name + ': No config folder found for %s... creating one at %s', node_name, config_folder)
      os.makedirs(name = config_folder, mode = 0o775)
      return
    
    config_file = os.path.join(config_folder, node_name + ".yaml")
    node_namespace = rospy.get_namespace() + node_name
    if os.path.exists(config_file):
      rospy.loginfo(self.node_name + ": Loading parameters from " + config_file + " to " + node_namespace)
      #rosparam.load_file(filename = config_file, default_namespace = node_namespace)
      #rosparam.load_file(filename = config_file, default_namespace = node_name)
      # Seems programmatic rosparam.load_file is not working at all, so use the command-line version instead
      rosparam_load_cmd = ['rosparam', 'load', config_file, node_namespace]
      subprocess.run(rosparam_load_cmd)
    else:
      rospy.logwarn(self.node_name + ": No config file found for " + node_name + " in " + self.NEPI_DEFAULT_CFG_PATH)

  def short_name(self,name):
    split = name.split("_")
    if len(split) > 3:
      short_name = (split[0] + "_" + split[1] + "_" + split[2])
    else:
      short_name = name
    return short_name
    
if __name__ == '__main__':
  node = V4L2CamDiscovery()            

        
      

 
