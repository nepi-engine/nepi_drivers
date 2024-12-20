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
from nepi_edge_sdk_base import nepi_drv

PKG_NAME = 'IDX_ZED' # Use in display menus
FILE_TYPE = 'DISCOVERY'


TEST_NEX_DICT = {
'group': 'IDX',
'group_id': 'ZED',
'pkg_name': 'IDX_ZED',
'NODE_DICT': {
    'file_name': 'idx_zed_node.py',
    'module_name': 'idx_zed_node',
    'class_name': 'ZedCamNode',
},
'DRIVER_DICT': {
    'file_name': '' ,
    'module_name': '' ,
    'class_name':  ''
},
'DISCOVERY_DICT': {
    'file_name': 'idx_zed_discovery.py',
    'module_name': 'idx_zed_discovery',
    'class_name': 'ZEDCamDiscovery',
    'interfaces': ['USB'],
    'options_1_dict': {
        'default_val': 'HD720',
        'set_val': 'HD720'
    },
    'options_2_dict': {
        'default_val': '5',
        'set_val': '5'
    },
    'method': 'AUTO', 
    'include_ids': ['ZED 2','ZED 2i','ZED-M'],
    'exclude_ids': []
},
'DEVICE_DICT': {'zed_type': 'zed2','res_val': 3},
'path': '/opt/nepi/ros/lib/nepi_drivers',
'order': 1,
'active': True,
'msg': ""
}

class ZedCamDiscovery:
  NEPI_DEFAULT_CFG_PATH = '/opt/nepi/ros/etc/'
  CHECK_INTERVAL_S = 5.0
  DEVICE_DICT = dict()

  RES_DICT = dict(
    HD2K = 0,
    HD1080 = 1,
    HD720 = 3,
    VGA = 5
  )
  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"                                   
  deviceList = []                
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgWarn(self,"Starting Initialization Processes")
    ##############################
    # Get required drv driver dict info
    self.drv_dict = nepi_ros.get_param(self,'~drv_dict',TEST_NEX_DICT) 
    #nepi_msg.publishMsgWarn(self,"Drv_Dict: " + str(self.drv_dict))
    self.includeDevices = self.drv_dict['DISCOVERY_DICT']['include_ids']
    self.excludedDevices = self.drv_dict['DISCOVERY_DICT']['exclude_ids']
    res_str = self.drv_dict['DISCOVERY_DICT']['option_1_dict']['set_val']
    if res_str in self.RES_DICT.keys():
      self.res_val = self.RES_DICT[res_str]
    else:
      self.res_val = 3

    fr_str = self.drv_dict['DISCOVERY_DICT']['option_2_dict']['set_val']
    try:
      self.fr_val = int(fr_str)
    except Exception as e:
      self.fr_val = 5
    
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
        
      # Honor the inclusion list
      if line.startswith('/dev/video'):
        device_path = line
        if device_type in self.includeDevices:
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
          
          if is_video_cap_device:
            active_paths.append(device_path) # To check later that the device list has no entries for paths that have disappeared
            known_device = False
            # Check if this device is already known and launched
            for path in self.DEVICE_DICT.keys():
              device = self.DEVICE_DICT[path]
              if device['device_path'] == device_path:
                known_device = True
                if device['device_type'] != device_type:
                  # Uh oh -- device has switched on us!
                  # Kill previous and start new?
                  nepi_msg.publishMsgWarn(self,"detected Zed device type change (" + device['device_type'] + "-->" + 
                                device_type + ") for device at " + device_path)
                  self.stopAndPurgeDeviceNode(device['node_namespace'])
                  break
                '''
                elif not self.deviceNodeIsRunning(device['node_namespace']):
                  nepi_msg.publishMsgWarn(self,"node " + device['node_name'] + " is not running. Restarting")
                  self.stopAndPurgeDeviceNode(device['node_namespace'])
                  self.startDeviceNode(dtype = device_type, path = device_path, bus = usbBus)
                break
                '''

            if known_device == False:
              nepi_msg.publishMsgWarn(self,"Starting zed on path" + device_path)
              success = self.startDeviceNode(dtype = device_type, path = device_path, bus = usbBus)
              if success:
                nepi_msg.publishMsgInfo(self,"Started new node for path: " + device_path)
              else:
                nepi_msg.publishMsgInfo(self,"Failed to start new node for path: " + device_path)

    # Check that device path still active
    #nepi_msg.publishMsgWarn(self,"Active paths " + str(active_paths))
    purge_list = []
    for path in self.DEVICE_DICT.keys():
      if path not in active_paths:
        purge_list.append(path)
    for path in purge_list:
        device_dict = self.DEVICE_DICT[path]
        node_name = device_dict['node_namespace']
        nepi_msg.publishMsgInfo(self,"Device no longer present. Stopping node " + node_name)
        self.stopAndPurgeDeviceNode(node_name)           
        
    nepi_ros.sleep(self.CHECK_INTERVAL_S,100)
    nepi_ros.start_timer_process(nepi_ros.duration(1), self.detectAndManageDevices, oneshot = True)

  def startDeviceNode(self, dtype, path, bus):
    success = False
    # First, get a unique name
    if dtype is not None:
      if dtype in self.includeDevices:
        root_name = dtype.replace('-','').replace(' ','').lower() 

        same_type_count = 0
        for path in self.DEVICE_DICT.keys():
          device = self.DEVICE_DICT[path]
          if device['device_type'] == dtype:
            same_type_count += 1

        device_node_name = self.short_name(root_name)
        if bus is not None:
          id = bus
        else:
          id = str(same_type_count)
        device_node_name += '_' + id

        device_exists = False
        for path in self.DEVICE_DICT.keys():
          device = self.DEVICE_DICT[path]
          if device['node_name'] == device_node_name:
            device_exists = True

        if device_exists is False:
          device_node_namespace = nepi_ros.get_base_namespace() + device_node_name
          nepi_msg.publishMsgInfo(self,"Initiating new Zed node " + device_node_namespace)

          self.checkLoadConfigFile(node_name = device_node_name)

          # Now start the node via rosrun
          # rosrun nepi_drivers_idx zed_camera_node.py __name:=usb_cam_1 _device_path:=/dev/video0
          nepi_msg.publishMsgInfo(self,"Launching node " + device_node_name)
          if dtype in self.includeDevices:
            #Setup required param server drv_dict for discovery node
            self.drv_dict['DEVICE_DICT']={'zed_type': root_name, 'res_val': self.res_val, 'fr_val': self.fr_val}
            dict_param_name = device_node_name + "/drv_dict"
            nepi_ros.set_param(self,dict_param_name,self.drv_dict)
            file_name = self.drv_dict['NODE_DICT']['file_name']
            #Try and launch node
            self.DEVICE_DICT[path] = {'device_class': root_name, 'device_path': path, 'device_type': dtype, 
                                      'node_name': device_node_name, 'node_namespace': device_node_namespace}
            [success, msg, sub_process] = nepi_drv.launchDriverNode(file_name, device_node_name)
            self.DEVICE_DICT[path]['node_subprocess'] = sub_process
            self.DEVICE_DICT[path]['zed_type'] = root_name
            nepi_msg.publishMsgInfo(self,msg)
      return success

  def stopAndPurgeDeviceNode(self, node_namespace):
    nepi_msg.publishMsgWarn(self,"stopping " + node_namespace)
    purge_path = None
    for path in self.DEVICE_DICT.keys():
      device = self.DEVICE_DICT[path]
      if device['node_namespace'] == node_namespace:
        node_name = device['node_namespace'].split("/")[-1]
        sub_process = device['node_subprocess']
        success = nepi_drv.killDriverNode(node_name,sub_process)
        if success:
          purge_path = path
        else:
          nepi_msg.publishMsgWarn(self,"Unable to stop unknown node " + node_namespace)

    if purge_path is not None:
      # Try and kill the zed_node
      try:
        zed_type = self.DEVICE_DICT[purge_path]['zed_type']
        zed_node_namespace = os.path.join(self.base_namespace,zed_type,'zed_node')
        nepi_ros.kill_node_namespace(zed_node_namespace)
      except Exception as e:
        nepi_msg.publishMsgWarn(self,"Failed to kill zed node namespace " + zed_node_namespace + " " + str(e))
      try:
        zed_type = self.DEVICE_DICT[purge_path]['zed_type']
        zed_node_namespace = os.path.join(self.base_namespace,zed_type,zed_type + '_state_publisher')
        nepi_ros.kill_node_namespace(zed_node_namespace)
      except Exception as e:
        nepi_msg.publishMsgWarn(self,"Failed to kill zed node namespace " + zed_node_namespace + " " + str(e))
      # delete device dict entry
      del self.DEVICE_DICT[purge_path]



  def deviceNodeIsRunning(self, node_namespace):
    for path in self.DEVICE_DICT.keys():
      device = self.DEVICE_DICT[path]
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
      nepi_msg.publishMsgWarn(self,'No config folder found for ' + node_name + '... creating one at ' + config_folder)
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
  node = ZedCamDiscovery()            

        
      

 
