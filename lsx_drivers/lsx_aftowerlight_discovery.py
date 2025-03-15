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
import subprocess
import time
import serial
import serial.tools.list_ports

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_drvs

PKG_NAME = 'LSX_AFTOWERLIGHT'
FILE_TYPE = 'DISCOVERY'

class AfTowerLightDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  active_devices_dict = dict()
  node_launch_name = "af_tower_light"

  includeDevices = ['29987']
  excludedDevices = []
  ################################################          
  def __init__(self):
    self.log_name = PKG_NAME.lower() + "_discovery" 
    nepi_msg.createMsgPublishers(self)
    time.sleep(1)
    nepi_msg.publishMsgInfo(self, ":" + self.log_name + ": Starting Initialization")
    nepi_msg.publishMsgInfo(self, ":" + self.log_name + ": Initialization Complete")

  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace, drv_dict):
    self.drv_dict = drv_dict
    #nepi_msg.publishMsgInfo(self, ":  " + self.log_name + "Got drv_dict : " + str(self.drv_dict))
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    
    ########################
    # Get discovery options
    try:
      #Snepi_msg.publishMsgWarn(self, ": " + self.log_name + ": Starting discovery with drv_dict " + str(drv_dict))#
      baudrate_options = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['options']
      self.baud_rate = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['value']
    except Exception as e:
      nepi_msg.publishMsgWarn(self, ":" + self.log_name + ": Failed to load options " + str(e))#
      return None

    if 'retry' in self.drv_dict['DISCOVERY_DICT']['OPTIONS'].keys():
      self.retry = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['retry']['value']
    else:
      self.retry = True
    ########################

    # Create path search options    
    self.path_list = []
    ports = serial.tools.list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      self.path_list.append(loc)

    ### Purge Unresponsive Connections
    path_purge_list = []
    for path_str in self.active_devices_dict:
        success = self.checkOnDevice(path_str)
        if success == False:
          path_purge_list.append(path_str) 
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del  self.active_devices_dict[path_str]
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)

    ### Checking for devices on available paths
    for path_str in self.path_list:
      if path_str not in self.active_paths_list:
        found = self.checkForDevice(path_str)
        if found:
          success = self.launchDeviceNode(path_str)
          if success:
            self.active_paths_list.append(path_str)
    return self.active_paths_list
  ################################################

  ##########  Device specific calls

  def checkForDevice(self,path_str):
    #nepi_msg.publishMsgWarn(self, log_name + " checkForDevice")###
    found_device = False
    #nepi_msg.printMsgWarn(self.log_name + "path_str " + path_str)
    if path_str.find('ttyUSB') != -1:
      usb_dict = nepi_drvs.getSerialPortDict()
      #nepi_msg.printMsgWarn(self.log_name + "serial_port_dict " + str(usb_dict))
      if path_str in usb_dict.keys():
        #nepi_msg.printMsgWarn(self.log_name + "search ids " + str(self.search_ids))
        #nepi_msg.printMsgWarn(self.log_name + "serial_port product id " + str(usb_dict[path_str]['product_id']))
        if str(usb_dict[path_str]['product_id']) in self.includeDevices:
          #nepi_msg.printMsgWarn(self.log_name + "found device on path: " + path_str)
          found_device = True
    return found_device



  def checkOnDevice(self,path_str):
    active = True
    if path_str not in self.available_paths_list:
      active = False
    elif self.checkForDevice(path_str) == False:
      active = False
    if active == False:
      nepi_msg.publishMsgInfo(self, ":" + self.log_name + ": No longer detecting device on : " + path_str)
      if path_str in self.active_devices_dict.keys():
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        success = nepi_drvs.killDriverNode(node_name,sub_process)

        # Remove from dont_retry_list
        launch_id = path_str
        if launch_id in self.dont_retry_list:
          self.dont_retry_list.remove(launch_id)

    return active


  def launchDeviceNode(self, path_str):
    success = False
    launch_id = path_str

    # Check if should try to launch
    launch_check = True
    if launch_id in self.launch_time_dict.keys():
      launch_time = self.launch_time_dict[launch_id]
      cur_time = nepi_ros.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LAUNCH_TIME_SEC
    if launch_check == False:
      return False   ###

    ### Start Node Luanch Process
    file_name = self.drv_dict['NODE_DICT']['file_name']
    node_name = self.node_launch_name + "_" + path_str.split('/')[-1]

    nepi_msg.printMsgInfo(self.log_name + "launching node: " + node_name)
    #Setup required param server drv_dict for discovery node
    dict_param_name = self.base_namespace + node_name + "/drv_dict"
    nepi_msg.publishMsgInfo(self, ":  " + self.log_name + "launching node: " + str(self.drv_dict))
    self.drv_dict['DEVICE_DICT'] = dict()
    self.drv_dict['DEVICE_DICT']['device_path'] = path_str
    self.drv_dict['DEVICE_DICT']['baud_rate_str'] = self.baud_rate
    nepi_ros.set_param(self,dict_param_name,self.drv_dict)
    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = path_str)
    if success == True:
      self.active_devices_dict[path_str] = {'node_name': device_node_name, 'sub_process': sub_process}

    # Process luanch results
    self.launch_time_dict[launch_id] = nepi_ros.get_time()
    if success:
      nepi_msg.publishMsgInfo(self," Launched node: " + device_node_name)
    else:
      nepi_msg.publishMsgInfo(self," Failed to lauch node: " + device_node_name + " with msg: " + msg)
      if self.retry == False:
        nepi_msg.publishMsgInfo(self," Will not try relaunch for node: " + device_node_name)
        self.dont_retry_list.append(launch_id)
      else:
        nepi_msg.publishMsgInfo(self," Will attemp relaunch for node: " + device_node_name + " in " + self.NODE_LAUNCH_TIME_SEC + " secs")
    return success


if __name__ == '__main__':
    AfTowerLightDiscovery()

    


        
      

 
