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
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs

from nepi_sdk.nepi_ros import logger as Logger
log_name = "iqr_pan_tilt"
logger = Logger(log_name = log_name)

PKG_NAME = 'LSX_DEEPSEA_SEALITE'
FILE_TYPE = 'DISCOVERY'

#########################################
# Discover Method
#########################################


### Function to try and connect to device and also monitor and clean up previously connected devices
class SealiteDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  active_devices_dict = dict()
  node_launch_name = "sealite"
  baudrate_list = []
  baud_str = '9600'
  baud_int = 9600
  addr_str = "001"

  includeDevices = []
  excludedDevices = ['ttyACM']

  ################################################          
  def __init__(self):
    ############
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_ros.logger(log_name = self.log_name)
    time.sleep(1)
    self.logger.log_info("Starting Initialization")
    self.logger.log_info("Initialization Complete")


  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace,drv_dict):
    self.drv_dict = drv_dict
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace

    ########################
    # Get discovery options
    try:
      #self.logger.log_warn("Starting discovery with drv_dict " + str(drv_dict))#
      baudrate_options = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['options']
      baudrate_sel = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['value']
      baudrate_list = []
      if baudrate_sel != "All":
        baudrate_list.append(baudrate_sel)
      else:
        for baudrate in baudrate_options:
          if baudrate != "All":
            baudrate_list.append(baudrate)
      self.baudrate_list = baudrate_list

      start_addr = int(drv_dict['DISCOVERY_DICT']['OPTIONS']['start_addr']['value'])
      stop_addr = int(drv_dict['DISCOVERY_DICT']['OPTIONS']['stop_addr']['value'])
      addr_range = stop_addr - start_addr
      if addr_range > 0:
        self.addr_search_list = list(range(start_addr,stop_addr+1))
      else:
        self.addr_search_list = [start_addr]
    except Exception as e:
      self.logger.log_warn("Failed to load options " + str(e))#
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
        if self.retry == False:
          if success == False:
            path_purge_list.append(path_str) 
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del  self.active_devices_dict[path_str]
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)

    ### Checking for devices on available paths
    for path_str in self.path_list:
      valid_path = True
      for id in self.excludedDevices:
        if path_str.find(id) != -1 or path_str in self.active_paths_list:
          valid_path = False
      if valid_path:
        #self.logger.log_warn("Looking for path: " + path_str)
        #self.logger.log_warn("In path_list: " + str(self.active_paths_list))
        found = self.checkForDevice(path_str)
        if found:
          success = self.launchDeviceNode(path_str)
          if success:
            self.active_paths_list.append(path_str)
    return self.active_paths_list
  ################################################

  ##########  Device specific calls
  def checkForDevice(self,path_str):
    found_device = False
    #self.logger.log_warn("Running device search with path: " + path_str + " and buadlist " + str(self.baudrate_list))
    if path_str not in self.active_paths_list:
      for baud_str in self.baudrate_list:
        self.baud_str = baud_str
        self.baud_int = int(baud_str)
        try:
          # Try and open serial port
          serial_port = serial.Serial(path_str,self.baud_int,timeout = 1)
        except Exception as e:
          self.logger.log_warn("Unable to open serial port " + path_str + " with baudrate: " + baud_str + "(" + str(e) + ")")
          continue
        for addr in self.addr_search_list:
          addr_str = str(addr)
          zero_prefix_len = 3-len(addr_str)
          for z in range(zero_prefix_len):
            addr_str = ('0' + addr_str)
          # Create message string
          ser_msg= ('!' + addr_str + ':INFO?')
          ser_str = (ser_msg + '\r\n')
          # Send Serial String
          #print("")
          #print("Sending serial message: " + ser_msg)
          b=bytearray()
          b.extend(map(ord, ser_str))
          try:
            serial_port.write(b)
            #print("Waiting for response")
            nepi_ros.sleep(.005)
            bs = serial_port.readline()
            response = bs.decode()
          except Exception as e:
            #self.logger.log_warn("Got a serial read/write error: " + str(e))
            continue
          if len(response) > 2:
            self.logger.log_warn("Got response: " + response)
            if response[3] == ',':
              self.addr_str = response[0:3]
              try:
                addr_int = int(addr)
                self.logger.log_warn("Found device at path: " + path_str)
                self.logger.log_warn("Found device at baudrate: " + baud_str)
                self.logger.log_warn("Found device at address: " + self.addr_str)
                found_device = True
                return found_device
              except Exception as a:
                self.logger.log_warn("Returned device message not valid (" + str(a) + ")")
        # Clean up the serial port
        serial_port.close()
    return found_device


  def checkOnDevice(self,path_str):
    active = True
    if path_str not in self.available_paths_list:
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
      cur_time = nepi_ros.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LAUNCH_TIME_SEC
    if launch_check == False:
      return False   ###

    ### Start Node Luanch Process
    file_name = self.drv_dict['NODE_DICT']['file_name']
    node_name = self.node_launch_name + "_" + path_str.split('/')[-1] + "_" + str(self.addr_str)
    self.logger.log_warn(" launching node: " + node_name)
    #Setup required param server drv_dict for discovery node
    dict_param_name = self.base_namespace + node_name + "/drv_dict"
    # Try to load node saved device config
    nepi_drvs.checkLoadConfigFile(node_name)
    # Store drv info for node to use
    self.drv_dict['DEVICE_DICT'] = dict()
    self.drv_dict['DEVICE_DICT']['device_path'] = path_str
    self.drv_dict['DEVICE_DICT']['baud_str'] = self.baud_str
    self.drv_dict['DEVICE_DICT']['addr_str'] = self.addr_str
    #self.logger.log_info(" launching node: " + str(self.drv_dict))
    self.logger.log_warn("Launching node  with path: " + path_str + " baudrate: " + self.baud_str + " addr: " + self.addr_str)
    #self.logger.log_warn(" launching node: " + str(self.drv_dict))
    self.launch_time_dict[path_str] = nepi_ros.get_time()
    nepi_ros.set_param(dict_param_name,self.drv_dict)
    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = path_str)
    if success == True:
      self.active_devices_dict[path_str] = {'node_name': device_node_name, 'sub_process': sub_process}

    # Process luanch results
    self.launch_time_dict[launch_id] = nepi_ros.get_time()
    if success:
      self.logger.log_warn("Launched node: " + device_node_name)
    else:
      self.logger.log_warn("Failed to lauch node: " + device_node_name + " with msg: " + msg)
      if self.retry == False:
        self.logger.log_warn("Will not try relaunch for node: " + device_node_name)
        self.dont_retry_list.append(launch_id)
      else:
        self.logger.log_warn("Will attemp relaunch for node: " + device_node_name + " in " + self.NODE_LAUNCH_TIME_SEC + " secs")
    return success


    
