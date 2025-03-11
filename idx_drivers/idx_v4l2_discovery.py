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
# ROS namespace setup
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import subprocess
import time

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_drv
from nepi_sdk import nepi_save

PKG_NAME = 'IDX_V4L2' # Use in display menus
FILE_TYPE = 'DISCOVERY'

class V4L2CamDiscovery:
  includeDevices = []
  excludedDevices = ['msm_vidc_vdec','ZED 2','ZED 2i','ZED-M','ZED-X']     

  NEPI_DEFAULT_CFG_PATH = '/opt/nepi/ros/etc/nepi_drivers'
  NEPI_DEFAULT_USER_CFG_PATH = 'mnt/nepi_storage/user_cfg/ros'
  CHECK_INTERVAL_S = 3.0

  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"    
  drv_dict = dict()                        
  deviceList = []         
  
  def __init__(self):
    #### NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################

    ########################
    # Get discovery options
    try:
      self.drv_dict = nepi_ros.get_param(self,'~drv_dict',dict())
      nepi_msg.publishMsgWarn(self,"Initial Driver Dict: " + str(self.drv_dict))
    except Exception as e:
      nepi_msg.publishMsgWarn(self, "Failed to load options " + str(e))#
      nepi_ros.signal_shutdown(self.node_name + ": Shutting down because failed to get Driver Dict")
      return
    ########################

    nepi_ros.start_timer_process(nepi_ros.ros_ros_ros_duration(1), self.detectAndManageDevices, oneshot = True)
    # Now start the node
    nepi_msg.publishMsgInfo(self,"Initialization Complete")
    nepi_ros.spin()

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
      if line.startswith('/dev/video'):
        device_path = line
        # Check if this device is already known and launched  
        if device_type not in self.excludedDevices:  
          # nepi_msg.publishMsgWarn(self,"Found device type: " + device_type + " on path " + device_path) 
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
            for device in self.deviceList:
              if device['device_path'] == device_path:
                known_device = True
                if device['device_type'] != device_type:
                  # Uh oh -- device has switched on us!
                  # Kill previous and start new?
                  nepi_msg.publishMsgWarn(self,"detected V4L2 device type change (" + device['device_type'] + "-->" + 
                                device_type + ") for device at " + device_path)
                  self.stopAndPurgeDeviceNode(device['node_namespace'])
                break
            '''
            elif not self.deviceNodeIsRunning(device['node_namespace']):
              nepi_msg.publishMsgWarn(self,"node " + device['node_name'] + " is not running. Restarting")
              self.stopAndPurgeDeviceNode(device['node_namespace'])
              time.sleep(1)
              self.startDeviceNode(dtype = device_type, path = device_path, bus = usbBus)
            '''
            if known_device == False:
              #nepi_msg.publishMsgInfo(self,"Found new V4L2 device on path: " + device_path)
              success = self.startDeviceNode(dtype = device_type, path = device_path, bus = usbBus)
              if success:
                nepi_msg.publishMsgInfo(self,"Started new node for path: " + device_path)
              else:
                pass
                #nepi_msg.publishMsgInfo(self,"Failed to start new node for path: " + device_path)

    # Check that device path still active
    #nepi_msg.publishMsgWarn(self,"Active paths " + str(active_paths))
    purge_list = []
    for device in self.deviceList:
      if device['device_path'] not in active_paths:
        purge_list.append(device['node_namespace'])
    for node_name in purge_list:
        nepi_msg.publishMsgInfo(self,"Device no longer present. Stopping node " + device['node_name'])
        self.stopAndPurgeDeviceNode(node_name)  

    nepi_ros.sleep(self.CHECK_INTERVAL_S,100)
    nepi_ros.start_timer_process(nepi_ros.ros_ros_ros_duration(1), self.detectAndManageDevices, oneshot = True)

  def startDeviceNode(self, dtype, path, bus):
    # First, get a unique name
    success = False
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


          # Now start the node via rosrun
          # rosrun nepi_drivers_idx v4l2_camera_node.py __name:=usb_cam_1 _device_path:=/dev/video0
          nepi_msg.publishMsgInfo(self,"" + "Launching node " + device_node_name)
          if dtype not in self.excludedDevices:
            #Setup required param server drv_dict for discovery node
            self.drv_dict['DEVICE_DICT'] = {'device_path': path}
            dict_param_name = os.path.join(self.base_namespace,device_node_name + "/drv_dict")
            nepi_ros.set_param(self,dict_param_name,self.drv_dict)
            # Try and load save node params
            nepi_drv.checkLoadConfigFile(device_node_name)
            
            file_name = self.drv_dict['NODE_DICT']['file_name']
            nepi_msg.publishMsgInfo(self,"Starting new V4L2 node with drv_dict " +str(self.drv_dict))
            #Try and launch node
            [success, msg, sub_process] = nepi_drv.launchDriverNode(file_name, device_node_name)
            if success:
              self.deviceList.append({'device_class': 'v4l2', 'device_path': path, 'device_type': dtype, 
                                      'node_name': device_node_name, 'node_namespace': device_node_namespace,
                                      'node_subprocess': sub_process})
            else:
              nepi_msg.publishMsgInfo(self,msg)
    return success    

  def stopAndPurgeDeviceNode(self, node_namespace):
    nepi_msg.publishMsgInfo(self,"stopping " + node_namespace)
    for i, device in enumerate(self.deviceList):
      if device['node_namespace'] == node_namespace:
        node_name = device['node_namespace'].split("/")[-1]
        sub_process = device['node_subprocess']
        success = nepi_drv.killDriverNode(node_name,sub_process)
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
  




  def short_name(self,name):
    split = name.split("_")
    #if len(split) > 3:
    #  short_name = (split[0] + "_" + split[1] + "_" + split[2])
    #else:
    #  short_name = name
    short_name = split[0]
    return short_name
    
if __name__ == '__main__':
  node = V4L2CamDiscovery()            

        
      

 
