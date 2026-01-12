#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import subprocess
import time
import serial
import serial


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_serial

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "iqr_pan_tilt"
logger = Logger(log_name = log_name)

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
    ############
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_sdk.logger(log_name = self.log_name)
    time.sleep(1)
    self.logger.log_info("Starting Initialization")
    self.logger.log_info("Initialization Complete")

  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace, drv_dict):
    self.drv_dict = drv_dict
    #self.logger.log_warn("Got drv_dict : " + str(self.drv_dict))
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    
    ########################
    # Get discovery options
    try:
      #Sself.logger.log_warn("": " + self.log_name + ": Starting discovery with drv_dict " + str(drv_dict))#
      baudrate_options = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['options']
      self.baud_rate = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['value']
    except Exception as e:
      self.logger.log_warn("Failed to load options " + str(e))#
      return None

    if 'retry' in self.drv_dict['DISCOVERY_DICT']['OPTIONS'].keys():
      self.retry = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['retry']['value']
    else:
      self.retry = True
    ########################

    # Create path search options
    self.path_list = nepi_serial.get_serial_ports_list()
    #self.logger.log_warn("ports list: " + str(self.path_list))###

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
    #self.logger.log_warn("log_name + " checkForDevice")###
    found_device = False
    self.logger.log_warn("path_str " + path_str)
    if path_str.find('ttyUSB') != -1:
      ports_dict = nepi_serial.get_serial_ports_dict_list()
      self.logger.log_warn("serial_port_dict " + str(ports_dict))
      if path_str in ports_dict.keys():
        self.logger.log_warn("search ids " + str(self.search_ids))
        self.logger.log_warn("serial_port product id " + str(ports_dict[path_str]['product_id']))
        if str(ports_dict[path_str]['product_id']) in self.includeDevices:
          #self.logger.log_warn("found device on path: " + path_str)
          found_device = True
    return found_device



  def checkOnDevice(self,path_str):
    active = True
    if path_str not in self.available_paths_list:
      active = False
    elif self.checkForDevice(path_str) == False:
      active = False
    if active == False:
      self.logger.log_info("No longer detecting device on : " + path_str)
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
      cur_time = nepi_sdk.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LOAD_TIME_SEC
    if launch_check == False:
      return False   ###

    ### Start Node Luanch Process
    file_name = self.drv_dict['NODE_DICT']['file_name']
    node_name = self.node_launch_name + "_" + path_str.split('/')[-1]

    self.logger.log_warn("launching node: " + node_name)
    #Setup required param server drv_dict for discovery node
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace,node_name + "/drv_dict")
    self.logger.log_warn("launching node: " + str(self.drv_dict))
    self.drv_dict['DEVICE_DICT'] = dict()
    self.drv_dict['DEVICE_DICT']['device_path'] = path_str
    self.drv_dict['DEVICE_DICT']['baud_rate_str'] = self.baud_rate
    nepi_sdk.set_param(dict_param_name,self.drv_dict)
    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = path_str)
    if success == True:
      self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}

    # Process luanch results
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    if success:
      self.logger.log_warn("Launched node: " + node_name)
    else:
      self.logger.log_warn("Failed to lauch node: " + node_name + " with msg: " + msg)
      if self.retry == False:
        self.logger.log_warn("Will not try relaunch for node: " + node_name)
        self.dont_retry_list.append(launch_id)
      else:
        self.logger.log_warn("Will attemp relaunch for node: " + node_name + " in " + self.NODE_LOAD_TIME_SEC + " secs")
    return success


if __name__ == '__main__':
    AfTowerLightDiscovery()

    


        
      

 
