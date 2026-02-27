#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


import os
import subprocess
import time
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

from nepi_api.messages_if import MsgIF

PKG_NAME = 'IDX_V4L2' # Use in display menus
FILE_TYPE = 'DISCOVERY'

class V4L2CamDiscovery:

  NODE_LOAD_TIME_SEC = 10
  NODE_LAUNCH_DELAY_SEC = 2

  CHECK_INTERVAL_S = 2.0

  INCLUDE_DEVICES = []
  EXCLUDE_DEVICES = ['msm_vidc_vdec','ZED 2','ZED 2i','ZED-M','ZED-X']  

  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  launch_delay_sec = NODE_LAUNCH_DELAY_SEC

  check_for_devices = True

  drv_dict = dict()                        
  deviceList = []      

  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"    

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
    # Update discovery options
    success = self.updateDiscoveryOptions()
    if success == False:
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because failed to get Driver Dict")
        return
    
    ########################
    # Start node processes
    nepi_sdk.start_timer_process((1), self.detectAndManageDevices, oneshot = True)
    nepi_sdk.start_timer_process((1), self.updateDriverDictCb, oneshot = True)
    nepi_sdk.on_shutdown(self.cleanup_actions)

    ########################
    # Now start the node
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.spin()




  #**********************
  # Discovery functions
  #**********************

  def updateDriverDictCb(self,timer):
    updated = self.updateDiscoveryOptions()
    nepi_sdk.start_timer_process((1), self.updateDriverDictCb, oneshot = True)

  def updateDiscoveryOptions(self):
    ########################
    # Get discovery options
    success = False
    last_drv_dict = copy.deepcopy(self.drv_dict)
    self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
    if len(list(self.drv_dict.keys())) == 0:
      self.msg_if.pub_warn("Failed to load Driver dict " + str(e))#
      return success    
    if 'DISCOVERY_DICT' not in self.drv_dict.keys():
      self.msg_if.pub_warn("Discovery dict missing in Drvier dict discovery dict ")#
      return success
    if last_drv_dict != self.drv_dict:
      self.msg_if.pub_warn("Updated Driver Dict: " + str(self.drv_dict))
    success = True


    if 'retry' in self.drv_dict['DISCOVERY_DICT']['OPTIONS'].keys():
      self.retry = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['retry']['value']
    else:
      self.retry = True


    launch_delay_sec = self.NODE_LAUNCH_DELAY_SEC
    if 'launch_delay_sec' in self.drv_dict['DISCOVERY_DICT']['OPTIONS'].keys():
      delay_str = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['launch_delay_sec']['value']
      try:
        delay_sec = float(delay_str)
        launch_delay_sec = delay_sec
      except:
        pass
    self.launch_delay_sec = launch_delay_sec
      
    return success



  def detectAndManageDevices(self, timer): # Extra arg since this is a Timer callback
    success = False
    if self.check_for_devices == False:
      self.msg_if.pub_warn("Stopping device discovery process") 
      return
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
        
      # Honor the exclusion list
      if line.startswith('/dev/video'):
        path_str = line
        # Check if this device is already known and launched  
        if device_type not in self.EXCLUDE_DEVICES:  
          # self.msg_if.pub_warn("Found device type: " + device_type + " on path " + path_str) 
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
            for device in self.deviceList:
              ####################
              ## Check on known devices using this path
              ####################
              if device['device_path'] == path_str:
                known_device = True
                ####################
                # Check if device type has changed
                if device['device_type'] != device_type:
                  # Uh oh -- device has switched on us!
                  # Kill previous and start new?
                  self.msg_if.pub_warn("detected V4L2 device type change (" + device['device_type'] + "-->" + 
                                device_type + ") for device at " + path_str)
                  self.stopAndPurgeDeviceNode(device['node_namespace'])
                  # Remove from dont_retry_list
                  launch_id = path_str
                  if launch_id in self.dont_retry_list:
                    self.dont_retry_list.remove(launch_id)

              ####################
              # Check if running devices still running
              ####################
              elif not self.deviceNodeIsRunning(device['node_namespace']):
                  self.stopAndPurgeDeviceNode(device['node_namespace'])
                
                  ### DON'T REMOVE FROM dont_retry_list ###
                  launch_id = path_str
                  if launch_id in self.dont_retry_list:
                    self.msg_if.pub_warn("node " + device['node_name'] + " is not running. WILL NOT RESTART")
                  else:
                    self.msg_if.pub_warn("node " + device['node_name'] + " is not running. RESTARTING")

                    # Only start one device per check 
                    if success == False:
                      # Restart Device
                      success = self.startDeviceNode(dtype = device_type, path_str= path_str, bus = usbBus)
                      if success == True:
                        self.msg_if.pub_warn("Started new node on: " + path_str)


            ####################
            ## Start device on this path if not known
            ####################        
            if known_device == False:
              
              # Only start one device per check 
              if success == False:
                self.msg_if.pub_info("Found new device on path: " + path_str)
                # Restart Device
                success = self.startDeviceNode(dtype = device_type, path_str= path_str, bus = usbBus)
                if success == True:
                  self.msg_if.pub_warn("Started new node on: " + path_str)
                
              else:
                pass
                #self.msg_if.pub_info("Failed to start new node for path: " + path_str)
            
    ######################
    # Check that device paths are still active
    ######################
    #self.msg_if.pub_warn("Active paths " + str(active_paths))
    purge_list = []
    for device in self.deviceList:
      path_str = device['device_path']
      if path_str not in active_paths:
        purge_list.append(device['node_namespace'])
    for node_name in purge_list:
        self.msg_if.pub_info("Device path: " + path_str + " no longer present. Stopping node " + device['node_name'])
        self.stopAndPurgeDeviceNode(node_name)  

        # Remove from dont_retry_list
        launch_id = path_str
        if launch_id in self.dont_retry_list:
          self.dont_retry_list.remove(launch_id)      

    #########################
    # Setup next check process
    if success == True:
      self.msg_if.pub_warn("Delaying next check for " + str(self.launch_delay_sec) + " seconds")
      nepi_sdk.sleep(self.launch_delay_sec)

    nepi_sdk.start_timer_process(self.CHECK_INTERVAL_S, self.detectAndManageDevices, oneshot = True)



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
    # Try and Connect
    # First, get a unique name
    if dtype is not None:
      if dtype not in self.EXCLUDE_DEVICES:
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

        node_name = nepi_system.get_node_name(device_node_name)
        node_namespace = os.path.join(self.base_namespace, node_name)

        device_exists = False
        for device in self.deviceList:
          if device['node_name'] == node_name:
            device_exists = True

        if device_exists is False:
          self.msg_if.pub_info("Initiating new V4L2 node " + node_namespace)
          # Now start the node via rosrun
          # rosrun nepi_drivers_idx v4l2_camera_node.py __name:=usb_cam_1 _device_path:=/dev/video0
          self.msg_if.pub_info("" + "Launching node " + node_name)
          if dtype not in self.EXCLUDE_DEVICES:


            # Try and load saved node params if file exists
            nepi_sdk.load_node_config(device_node_name, node_name)


            #Setup required param server drv_dict for discovery node
            self.drv_dict['DEVICE_DICT'] = {'device_path': path_str}
            dict_param_name = os.path.join(self.base_namespace,node_name + "/drv_dict")
            nepi_sdk.set_param(dict_param_name,self.drv_dict)

           
            
            file_name = self.drv_dict['NODE_DICT']['file_name']
            self.msg_if.pub_info("Starting new V4L2 node with drv_dict " +str(self.drv_dict))
            #Try and launch node
            [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name)
            if success:
              self.deviceList.append({'device_class': 'v4l2', 'device_path': path_str, 'device_type': dtype, 
                                      'node_name': node_name, 'node_namespace': node_namespace,
                                      'node_subprocess': sub_process})

            self.msg_if.pub_warn("Added device to active list " + node_name)
            self.msg_if.pub_warn("Updated Active Device List " + str(self.deviceList))

            # Process luanch results
            self.launch_time_dict[launch_id] = nepi_utils.get_time()
            if success:
              self.msg_if.pub_info(" Launched node: " + node_name)
            else:
              self.msg_if.pub_info(" Failed to lauch node: " + node_name + " with msg: " + msg)
              if self.retry == False:
                self.msg_if.pub_info(" Will not try relaunch for node: " + node_name)
                self.dont_retry_list.append(launch_id)
              else:
                self.msg_if.pub_info(" Will attemp relaunch for node: " + node_name + " in " + self.NODE_LOAD_TIME_SEC + " secs")
    return success

  def stopAndPurgeDeviceNode(self, node_namespace = 'All'):
    success = False
    
    if len(self.deviceList) > 0:
      if node_namespace == 'All':
        self.check_for_devices = False
      for i, device in enumerate(self.deviceList):
        if device['node_namespace'] == node_namespace or node_namespace == 'All':
          node_name = device['node_namespace'].split("/")[-1]
          sub_process = device['node_subprocess']
          self.msg_if.pub_info("Killing device node: " + node_namespace)
          success = nepi_drvs.killDriverNode(node_name,sub_process)
          if success == False:
            self.msg_if.pub_warn("Unable to kill device node " + node_name)
          else:
            self.msg_if.pub_warn("Node killed. Removed device from active list " + node_name)

      self.deviceList = []
      self.msg_if.pub_warn("Updated Active Device List " + str(self.deviceList))


  def deviceNodeIsRunning(self, node_namespace):
    for device in self.deviceList:
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
    #if len(split) > 3:
    #  short_name = (split[0] + "_" + split[1] + "_" + split[2])
    #else:
    #  short_name = name
    short_name = split[0]
    return short_name
    

  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    self.stopAndPurgeDeviceNode('All')

if __name__ == '__main__':
  node = V4L2CamDiscovery()            

        
      

 
