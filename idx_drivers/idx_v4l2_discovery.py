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

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg

PKG_NAME = 'IDX_V4L2' # Use in display menus
FILE_TYPE = 'DISCOVERY'
DISCOVERY_DICT = dict(
  class_name = 'V4L2CamDiscovery',
  process = 'LAUNCH', # 'LAUNCH', 'RUN', or 'CALL'
  method = 'AUTO',  # 'AUTO', 'MANUAL', or 'OTHER' if managed by seperate application
  include_ids = [],  # List of string identifiers for discovery process
  exclude_ids = ['msm_vidc_vdec','ZED 2','ZED 2i','ZED-M'], # List of string identifiers for discovery process
  interfaces = ['USB'], # 'USB','IP','SERIALUSB','SERIAL','CANBUS'
  option_1_dict = dict(
    name = 'None',
    options = [], # List of string options. Selected option passed to driver
    default_val = 'None'
  ),
  option_2_dict = dict(
    name = 'None',
    options = [], # List of string options. Selected option passed to driver
    default_val = 'None'
  )
)


TEST_NEX_DICT = {
'group': 'IDX',
'group_id': 'V4L2',
'pkg_name': 'IDX_V4L2',
'NODE_DICT': {
    'file_name': 'idx_v4l2_node.py',
    'module_name': 'idx_v4l2_node',
    'class_name': 'V4l2CamNode',
},
'DRIVER_DICT': {
    'file_name': 'idx_v4l2_driver.py' ,
    'module_name': 'idx_v4l2_driver' ,
    'class_name':  'V4l2CamDriver'
},
'DISCOVERY_DICT': {
    'file_name': 'idx_v4l2_discovery.py',
    'module_name': 'idx_v4l2_discovery',
    'class_name': 'V4L2CamDiscovery',
    'interfaces': ['USB'],
    'options_1_dict': {
        'default_option': 'None',
        'set_option': 'None'
    },
    'options_2_dict': {
        'default_option': 'None',
        'set_option': 'None'
    },
    'method': 'AUTO', 
    'include_ids': [],
    'exclude_ids': ['msm_vidc_vdec','ZED 2','ZED 2i','ZED-M']
},
'DEVICE_DICT': {'device_path': '/dev/video0'},
'path': '/opt/nepi/ros/lib/nepi_drivers',
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
  deviceList = []                
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    # Get required nex driver dict info
    self.nex_dict = nepi_ros.get_param(self,'~nex_dict',TEST_NEX_DICT) 
    #nepi_msg.publishMsgWarn(self,"Nex_Dict: " + str(self.nex_dict))
    self.includeDevices = self.nex_dict['DISCOVERY_DICT']['include_ids']
    self.excludedDevices = self.nex_dict['DISCOVERY_DICT']['exclude_ids']

    nepi_ros.start_timer_process(nepi_ros.duration(1), self.detectAndManageDevices, oneshot = True)

    # Now start the node
    nepi_ros.spin()



  def detectAndManageDevices(self, timer): # Extra arg since this is a Timer callback
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
          if device['device_class'] != 'v4l2':
            continue

          if device['device_path'] == device_path:
            known_device = True
            if device['device_type'] != device_type:
              # Uh oh -- device has switched on us!
              # Kill previous and start new?
              nepi_msg.publishMsgWarn(self,"detected V4L2 device type change (" + device['device_type'] + "-->" + 
                            device_type + ") for device at " + device_path)
              self.stopAndPurgeDeviceNode(device['node_namespace'])
              self.startDeviceNode(dtype = device_type, path = device_path, bus = usbBus)
            elif not self.deviceNodeIsRunning(device['node_namespace']):
              nepi_msg.publishMsgWarn(self,"node " + device['node_name'] + " is not running. Restarting")
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
        nepi_msg.publishMsgWarn(self,device['node_namespace'] + ' path ' + device['device_path'] + ' no longer exists... device disconnected?')
        self.stopAndPurgeDeviceNode(device['node_namespace'])
    nepi_ros.sleep(self.CHECK_INTERVAL_S,100)
    nepi_ros.start_timer_process(nepi_ros.duration(1), self.detectAndManageDevices, oneshot = True)

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
          device_node_namespace = nepi_ros.get_base_namespace() + device_node_name
          nepi_msg.publishMsgInfo(self,"Initiating new V4L2 node " + device_node_namespace)

          self.checkLoadConfigFile(node_name = device_node_name)

          # Now start the node via rosrun
          # rosrun nepi_drivers_idx v4l2_camera_node.py __name:=usb_cam_1 _device_path:=/dev/video0
          nepi_msg.publishMsgInfo(self,"" + "Launching node " + device_node_name)
          if dtype not in self.excludedDevices:
            #Setup required param server nex_dict for discovery node
            self.nex_dict['DEVICE_DICT'] = {'device_path': path}
            dict_param_name = device_node_name + "/nex_dict"
            nepi_ros.set_param(self,dict_param_name,self.nex_dict)
            file_name = self.nex_dict['NODE_DICT']['file_name']
            #Try and launch node
            [success, msg, sub_process] = nepi_nex.launchDriverNode(file_name, device_node_name)
            if success:
              self.deviceList.append({'device_class': 'v4l2', 'device_path': path, 'device_type': dtype, 
                                      'node_name': device_node_name, 'node_namespace': device_node_namespace,
                                      'node_subprocess': sub_process})
            else:
              nepi_msg.publishMsgInfo(self,msg)

  def stopAndPurgeDeviceNode(self, node_namespace):
    nepi_msg.publishMsgInfo(self,"stopping " + node_namespace)
    for i, device in enumerate(self.deviceList):
      if device['node_namespace'] == node_namespace:
        node_name = device['node_namespace'].split("/")[-1]
        sub_process = device['node_subprocess']
        success = nepi_nex.killDriverNode(node_name,sub_process)
        # And remove it from the list
        self.deviceList.pop(i)  
    if success == False:
      nepi_msg.publishMsgWarn(self,"Unable to stop unknown node " + node_namespace)

  def deviceNodeIsRunning(self, node_namespace):
    for device in self.deviceList:
      if device['node_namespace'] == node_namespace:
        if device['node_subprocess'].poll() is not None:
          return False
        else:
          return True
    # If we get here, didn't find the node in our list    
    nepi_msg.publishMsgWarn(self,"cannot check run status of unknown node " + node_namespace)
    return False
  
  def checkLoadConfigFile(self, node_name):
    folder_name = "drivers/" + node_name 
    config_folder = os.path.join(self.NEPI_DEFAULT_CFG_PATH, folder_name)
    if not os.path.isdir(config_folder):
      nepi_msg.publishMsgWarn(self,'No config folder found for %s... creating one at %s', node_name, config_folder)
      os.makedirs(name = config_folder, mode = 0o775)
      return
    
    config_file = os.path.join(config_folder, node_name + ".yaml")
    node_namespace = nepi_ros.get_base_namespace() + node_name
    if os.path.exists(config_file):
      nepi_msg.publishMsgInfo(self,"Loading parameters from " + config_file + " to " + node_namespace)
      #rosparam.load_file(filename = config_file, default_namespace = node_namespace)
      #rosparam.load_file(filename = config_file, default_namespace = node_name)
      # Seems programmatic rosparam.load_file is not working at all, so use the command-line version instead
      rosparam_load_cmd = ['rosparam', 'load', config_file, node_namespace]
      subprocess.run(rosparam_load_cmd)
    else:
      nepi_msg.publishMsgWarn(self,"No config file found for " + node_name + " in " + self.NEPI_DEFAULT_CFG_PATH)

  def short_name(self,name):
    split = name.split("_")
    if len(split) > 3:
      short_name = (split[0] + "_" + split[1] + "_" + split[2])
    else:
      short_name = name
    return short_name
    
if __name__ == '__main__':
  node = V4L2CamDiscovery()            

        
      

 
