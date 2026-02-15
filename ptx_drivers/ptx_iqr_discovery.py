#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


import os
import subprocess
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'PTX_IQR' 
FILE_TYPE = 'DISCOVERY'

class IqrPanTiltDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []


  active_devices_dict = dict()
  node_launch_name = "iqr_pan_tilt"

  includeDevices = ['iqr_pan_tilt']
  excludedDevices = ['ttyACM']


  baud_str = '115200'
  addr_str = '1'

  source_path = 'None'
  ################################################          
  def __init__(self):
    ############
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_sdk.logger(log_name = self.log_name)
    time.sleep(1)
    self.logger.log_info("Starting Initialization")
    self.logger.log_info("Initialization Complete")



 
  ##########  DRV Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace, drv_dict):
    self.drv_dict = drv_dict
    #self.logger.log_warn("Got drv_dict : " + str(self.drv_dict))
    #self.logger.log_warn("Got available paths list : " + str(available_paths_list))
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    
    ##################################
    # Get required data from drv_dict
    if 'retry' in self.drv_dict['DISCOVERY_DICT']['OPTIONS'].keys():
      self.retry = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['retry']['value']
    else:
      self.retry = True
    ###################################

    ### Purge Unresponsive Connections
    path_purge_list = []
    for path_str in self.active_devices_dict.keys():
        success = self.checkOnDevice(path_str)
        if success == False:
          path_purge_list.append(path_str) 
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del  self.active_devices_dict[path_str]
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)

    ### Checking for devices on available paths
    for path_str in self.available_paths_list:
      if path_str not in self.active_paths_list and path_str not in self.excludedDevices:
        found_device = self.checkForDevice(path_str)
        if found_device == True:
          self.logger.log_info("Found device on path: " + path_str)
          success = self.launchDeviceNode(path_str)
          if success == True:
            self.active_paths_list.append(path_str)
            self.active_paths_list.append(self.source_path)
    return self.active_paths_list
  ################################################

  ##########  Device specific calls

  def checkForDevice(self,path_str):
    for included_device in self.includeDevices:
      found_device = path_str.find(included_device) != -1 
      if found_device:
        return True
    return False


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

    node_launch_name = 'iqr_pan_tilt_'
    self.logger.log_warn("Entering launch device function for path: " + str(path_str) )###
    file_name = self.drv_dict['NODE_DICT']['file_name']
    device_node_name = self.node_launch_name
    node_name = nepi_system.get_node_name(device_node_name)
    self.logger.log_warn(" launching node: " + node_name)


    # Try and load saved node params if file exists
    nepi_sdk.load_node_config(device_node_name, node_name)

    #Setup required param server drv_dict for discovery node
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace,node_name + "/drv_dict")
    self.drv_dict['DEVICE_DICT'] = dict()
    self.source_path = '/dev/' + os.readlink(path_str)
    self.drv_dict['DEVICE_DICT']['device_path'] = self.source_path
    self.drv_dict['DEVICE_DICT']['baud_str'] = self.baud_str
    self.drv_dict['DEVICE_DICT']['addr_str'] = self.addr_str
    nepi_sdk.set_param(dict_param_name,self.drv_dict)

    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name)

    # Process luanch results
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    if success:
      self.logger.log_info("Launched node: " + node_name)
      self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}
    else:
      self.logger.log_info("Failed to lauch node: " + node_name + " with msg: " + msg)
      if self.retry == False:
        self.logger.log_info("Will not try relaunch for node: " + node_name)
        self.dont_retry_list.append(launch_id)
      else:
        self.logger.log_info("Will attemp relaunch for node: " + node_name + " in " + self.NODE_LAUNCH_TIME_SEC + " secs")
    return success

  def killAllDevices(self,active_paths_list):
    #self.logger.log_warn("Entering Kill All Devices function for path: " + str(path_str))###
    path_purge_list = []
    for key in self.active_devices_dict.keys():
      path_purge_list.append(key)
    #self.logger.log_warn("Killing Devices: " + str(path_purge_list))
    for path_str in path_purge_list:
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        success = nepi_drvs.killDriverNode(node_name,sub_process)
        if path_str in active_paths_list:
          active_paths_list.remove(path_str)
        if self.source_path in active_paths_list:
          active_paths_list.remove(self.source_path)
        self.source_path = 'None'
    for path_str in path_purge_list:
        del  self.active_devices_dict[path_str]
    nepi_sdk.sleep(1)
    return active_paths_list


if __name__ == '__main__':
    IqrPanTiltDiscovery()

    


        
      

 
