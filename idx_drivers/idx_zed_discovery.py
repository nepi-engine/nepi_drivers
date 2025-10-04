#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#


import os
import rospy
import subprocess
import time


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs

from nepi_api.messages_if import MsgIF

PKG_NAME = 'IDX_ZED' # Use in display menus
FILE_TYPE = 'DISCOVERY'


class ZedCamDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  MAX_FRAMERATE = 15
  
  settings_if = None

  CHECK_INTERVAL_S = 3.0
  DEVICE_DICT = dict()
  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"                                   
  deviceList = []       

       
  RES_DICT = dict(
    HD2K = 0,
    HD1080 = 1,
    HD720 = 3,
    VGA = 5
  )

  includeDevices = ['ZED 2','ZED 2i','ZED-M']
  excludedDevices = []         

  retry = True
  failed_node_list = []

  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################  
    # Create Msg Class
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting Node Initialization Processes")

    ########################
    # Get discovery options
    try:
      self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
      self.msg_if.pub_warn("Initial Driver Dict: " + str(self.drv_dict))
      res_val = 3
      if 'resolution' in self.drv_dict['DISCOVERY_DICT']['OPTIONS']:
        res_str = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['resolution']['value']
        if res_str in self.RES_DICT.keys():
          self.res_val = self.RES_DICT[res_str]
      if 'framerate' in self.drv_dict['DISCOVERY_DICT']['OPTIONS']:
        fr_str = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['framerate']['value']
        fr_val=int(fr_str)
        if fr_val < 1:
          fr_val = 1
        if fr_val > self.MAX_FRAMERATE:
          fr_val = self.MAX_FRAMERATE
        self.fr_val = fr_val

    except Exception as e:
      self.msg_if.pub_warn("Failed to load options " + str(e))#
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because failed to get Driver Dict")
      return None
    if 'data_products' in self.drv_dict['DISCOVERY_DICT']['OPTIONS'].keys():
      data_products = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['data_products']['value'].split(',')
    else:
      data_products = ['color_image','depth_map','pointcloud']
    self.data_products = data_products
    if 'retry' in self.drv_dict['DISCOVERY_DICT']['OPTIONS'].keys():
      self.retry = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['retry']['value']
    else:
      self.retry = True
    ########################

    nepi_sdk.start_timer_process((1), self.detectAndManageDevices, oneshot = True)

    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.spin()

  #**********************
  # Discovery functions


  def detectAndManageDevices(self, timer): # Extra arg since this is a Timer callback
    # First grab the current list of known V4L2 devices
    sub_process = subprocess.Popen(['v4l2-ctl', '--list-devices'],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    stdout,_ = sub_process.communicate()
    #if sub_process.returncode != 0:  # This can happen because there are no longer any video devices connected, so v4l2-ctl returns error
      #raise Exception("Failed to list v4l2 devices: " + stdout)
    out = stdout.splitlines()

    device_type = None
    path_str = None
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
        path_str = line
        if device_type in self.includeDevices:
          # Make sure this is a legitimate Video Capture device, not a Metadata Capture device, etc.
          is_video_cap_device = False
          sub_process = subprocess.Popen(['v4l2-ctl', '-d', path_str, '--all'],
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
            active_paths.append(path_str) # To check later that the device list has no entries for paths that have disappeared
            known_device = False
            # Check if this device is already known and launched
            for path in self.DEVICE_DICT.keys():
              device = self.DEVICE_DICT[path]
              if device['device_path'] == path_str:
                known_device = True
                if device['device_type'] != device_type:
                  # Uh oh -- device has switched on us!
                  # Kill previous and start new?
                  self.msg_if.pub_warn("detected Zed device type change (" + device['device_type'] + "-->" + 
                                device_type + ") for device at " + path_str)
                  self.stopAndPurgeDeviceNode(device['node_namespace'])

                  # Remove from dont_retry_list
                  launch_id = path_str
                  if launch_id in self.dont_retry_list:
                    self.dont_retry_list.remove(launch_id)

                  break
                
                elif not self.deviceNodeIsRunning(device['node_namespace']):
                  self.stopAndPurgeDeviceNode(device['node_namespace'])
                  
                  ### DON'T REMOVE FROM dont_retry_list ###
                  launch_id = path_str
                  if launch_id in self.dont_retry_list:
                    self.msg_if.pub_warn("node " + device['node_name'] + " is not running. WILL NOT RESTART")
                  else:
                    self.msg_if.pub_warn("node " + device['node_name'] + " is not running. RESTARTING")
                    
                  time.sleep(1)

                  self.startDeviceNode(dtype = device_type, path_str= path_str, bus = usbBus)
                  break
              

            if known_device == False:
              self.msg_if.pub_warn("Starting zed on path" + path_str)
              success = self.startDeviceNode(dtype = device_type, path_str= path_str, bus = usbBus)
              if success:
                self.msg_if.pub_info("Started new node for path: " + path_str)
              else:
                self.msg_if.pub_info("Failed to start new node for path: " + path_str)

    # Check that device path still active
    #self.msg_if.pub_warn("Active paths " + str(active_paths))
    purge_list = []
    for path_str in self.DEVICE_DICT.keys():
      if path_str not in active_paths:
        purge_list.append(path_str)
    for path_str in purge_list:
        device_dict = self.DEVICE_DICT[path_str]
        node_name = device_dict['node_namespace']
        self.msg_if.pub_info("Device path: " + path_str + " no longer present. Stopping node " + node_name)
        self.stopAndPurgeDeviceNode(node_name)  

        # Remove from dont_retry_list
        launch_id = path_str
        if launch_id in self.dont_retry_list:
          self.dont_retry_list.remove(launch_id)        
        
    nepi_sdk.sleep(self.CHECK_INTERVAL_S,100)
    nepi_sdk.start_timer_process((1), self.detectAndManageDevices, oneshot = True)

  def startDeviceNode(self, dtype, path_str, bus):
    success = False 
    launch_id = path_str

    # Check if should try to launch
    launch_check = True
    if launch_id in self.launch_time_dict.keys():
      launch_time = self.launch_time_dict[launch_id]
      cur_time = nepi_utils.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LOAD_TIME_SEC
    if launch_check == False:
      return False  ###

    ### Start Node Luanch Process
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
          device_node_namespace = os.path.join(self.base_namespace, device_node_name)
          self.msg_if.pub_info("Initiating new Zed node " + device_node_namespace)

          # Now start the node via rosrun
          # rosrun nepi_drivers_idx zed_camera_node.py __name:=usb_cam_1 _path_str:=/dev/video0
          self.msg_if.pub_info("Launching node " + device_node_name)
          if dtype in self.includeDevices:
            #Setup required param server drv_dict for discovery node
            self.drv_dict['DEVICE_DICT']={'zed_type': root_name, 'res_val': self.res_val, 'framerate': self.fr_val, 'data_products': self.data_products}
            dict_param_name = device_node_name + "/drv_dict"
            nepi_sdk.set_param(dict_param_name,self.drv_dict)
            
            # Try and load save node params
            nepi_drvs.checkLoadConfigFile(device_node_name)

            file_name = self.drv_dict['NODE_DICT']['file_name']
            #Try and launch node
            self.DEVICE_DICT[path_str] = {'device_class': root_name, 'device_path': path_str, 'device_type': dtype, 
                                      'node_name': device_node_name, 'node_namespace': device_node_namespace}
            [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, device_node_name)
            if success:
              self.DEVICE_DICT[path_str]['node_subprocess'] = sub_process
              self.DEVICE_DICT[path_str]['zed_type'] = root_name
              self.msg_if.pub_info(msg)

            # Process luanch results
            self.launch_time_dict[launch_id] = nepi_utils.get_time()
            if success:
              self.msg_if.pub_info(" Launched node: " + device_node_name)
            else:
              self.msg_if.pub_info(" Failed to lauch node: " + device_node_name + " with msg: " + msg)
              if self.retry == False:
                self.msg_if.pub_info(" Will not try relaunch for node: " + device_node_name)
                self.dont_retry_list.append(launch_id)
              else:
                self.msg_if.pub_info(" Will attemp relaunch for node: " + device_node_name + " in " + self.NODE_LOAD_TIME_SEC + " secs")
    return success

  def stopAndPurgeDeviceNode(self, node_namespace):
    self.msg_if.pub_warn("stopping " + node_namespace)
    purge_path = None
    for path in self.DEVICE_DICT.keys():
      device = self.DEVICE_DICT[path]
      if device['node_namespace'] == node_namespace:
        node_name = device['node_namespace'].split("/")[-1]
        sub_process = device['node_subprocess']
        success = nepi_drvs.killDriverNode(node_name,sub_process)
        if success:
          purge_path = path
        else:
          self.msg_if.pub_warn("Unable to stop unknown node " + node_namespace)

    if purge_path is not None:
      # Try and kill the zed_node
      try:
        zed_type = self.DEVICE_DICT[purge_path]['zed_type']
        zed_node_namespace = os.path.join(self.base_namespace,zed_type,'zed_node')
        nepi_sdk.kill_node_namespace(zed_node_namespace)
      except Exception as e:
        self.msg_if.pub_warn("Failed to kill zed node namespace " + zed_node_namespace + " " + str(e))
      try:
        zed_type = self.DEVICE_DICT[purge_path]['zed_type']
        zed_node_namespace = os.path.join(self.base_namespace,zed_type,zed_type + '_state_publisher')
        nepi_sdk.kill_node_namespace(zed_node_namespace)
      except Exception as e:
        self.msg_if.pub_warn("Failed to kill zed node namespace " + zed_node_namespace + " " + str(e))
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
    self.msg_if.pub_warn("cannot check run status of unknown node " + node_namespace)
    return False
  
  def short_name(self,name):
    split = name.split("_")
    if len(split) > 3:
      short_name = (split[0] + "_" + split[1] + "_" + split[2])
    else:
      short_name = name
    return short_name
    
if __name__ == '__main__':
  node = ZedCamDiscovery()            

        
      

 
