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
import string

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs

PKG_NAME = 'PTX_SIDUS_SS182_SERIAL' # Use in display menus
FILE_TYPE = 'DISCOVERY'

 
#########################################
# PTX Discover Method
#########################################


### Function to try and connect to device and also monitor and clean up previously connected devices
class SidusSS182SerialDiscovery:

  DATA_LENGTH_DICT = {
      'Standard' : 4
  }

  data_len = 4

  active_paths_list = []

  active_devices_dict = dict()
  node_launch_name = "SS182"
  baudrate_list = []
  baud_str = '9600'
  baud_int = 9600
  addr_str = "001"

  includeDevices = []
  excludedDevices = ['ttyACM']

  ################################################          
  def __init__(self):
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_sdk.logger(log_name = self.log_name)
    time.sleep(1)
    self.logger.log_info("Starting Initialization")
    self.letters_list = list(string.ascii_uppercase)
    self.logger.log_info("Initialization Complete")

  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace,drv_dict):
    self.logger.log_debug("Entering discovery function with drv_dict: " + str(drv_dict))###
    self.drv_dict = drv_dict
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    ########################
    # Get discovery options
    try:
      baudrate_dict = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']
      self.logger.log_debug("Baud Rate dict: " + str(baudrate_dict))###
      #self.logger.log_debug("Starting discovery with drv_dict " + str(drv_dict))#
      baudrate_options = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['options']
      self.logger.log_debug("Baud Rate options: " + str(baudrate_options))###

      baudrate_sel = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['value']
      self.logger.log_debug("Baud Rate selected: " + str(baudrate_sel))###
      baudrate_list = []
      if baudrate_sel != "All":
        baudrate_list.append(baudrate_sel)
      else:
          for baudrate in baudrate_options:
            if baudrate != 'All':
              baudrate_list.append(baudrate)
      self.baudrate_list = baudrate_list
      self.logger.log_debug("Baud Rate list: " + str(baudrate_list))###

      start_addr = drv_dict['DISCOVERY_DICT']['OPTIONS']['start_addr']['value']
      try:
        start_ind = self.letters_list.index(start_addr)
      except:
        start_ind = 0
      self.logger.log_debug("Starting discovery addr: " + str(self.letters_list[start_ind]))

      stop_addr = drv_dict['DISCOVERY_DICT']['OPTIONS']['stop_addr']['value']
      try:
        stop_ind = self.letters_list.index(stop_addr)
      except:
        stop_ind = 0
      self.logger.log_debug("Ending discovery addr: " + str(self.letters_list[stop_ind]))

      self.addr_search_list = list(self.letters_list[start_ind:stop_ind + 1])
      self.logger.log_debug("Disc Addr List: " + str(self.addr_search_list))

    except Exception as e:
      self.logger.log_debug("" + self.log_name + ": Failed to setupoptions " + str(e))#
      return self.active_paths_list

    system_config = drv_dict['DISCOVERY_DICT']['OPTIONS']['system_config']['value']
    self.logger.log_debug("Got system config type: " + str(system_config))
    try:
      self.data_len = self.DATA_LENGTH_DICT[system_config]
      self.logger.log_debug("Got data length: " + str(self.data_len))
    except:
      self.data_len = 4
      self.logger.log_debug("Using default data length: " + str(self.data_len))
      return self.active_paths_list
    

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
      valid_path = True
      for id in self.excludedDevices:
        if path_str.find(id) != -1 or path_str in self.active_paths_list:
          valid_path = False
      if valid_path:
        self.logger.log_debug("Looking for path: " + path_str)
        self.logger.log_debug("In path_list: " + str(self.active_paths_list))
        found = self.checkForDevice(path_str)
        if found:
          success = self.launchDeviceNode(path_str)
          if success:
            self.active_paths_list.append(path_str)
    return self.active_paths_list
  ################################################

  ##########  Device specific calls
  def checkForDevice(self,path_str):
    self.logger.log_debug("Entering check for device function for path: " + str(path_str))###
    found_device = False
    if path_str not in self.active_paths_list:
      for baud_str in self.baudrate_list:
        self.baud_str = baud_str
        self.baud_int = int(baud_str)
        try:
          # Try and open serial port
          serial_port = serial.Serial(path_str,self.baud_int,timeout = 1)
        except Exception as e:
          self.logger.log_info("Unable to open serial port " + path_str + " with baudrate: " + baud_str + "(" + str(e) + ")")
          continue
        #################################################
        for addr_str in self.addr_search_list:
          data_str = ''
          for i in range(self.data_len):
            data_str = data_str + '0'
          ser_msg= ('&' + addr_str + 'DSN' + data_str + 'R')
          ser_str = (ser_msg + '\r\n')
          ################################################  
          # Send Serial String
          self.logger.log_debug("")
          self.logger.log_debug("Sending serial message: " + ser_msg)
          b=bytearray()
          b.extend(map(ord, ser_str))
          try:
            serial_port.write(b)
            self.logger.log_debug("Waiting for response")
            nepi_sdk.sleep(.010)
            bs = serial_port.readline()
            response = bs.decode()
          except Exception as e:
            self.logger.log_info("Got a serial read/write error: " + str(e))
            break
          self.logger.log_debug("Got response: " + response)
          if len(response) > 5:
            if response[0:5] == ser_msg[0:5]:
              self.addr_str = response[1]
              self.logger.log_info("Found device at path: " + path_str)
              self.logger.log_info("Found device at address: " + self.addr_str)
              found_device = True
              break # Don't check any more addresses
        # Clean up the serial port
        serial_port.close()
    return found_device


  def checkOnDevice(self,path_str):
    self.logger.log_debug("Entering check for device function for path: " + str(path_str))###
    active = True
    if path_str not in self.available_paths_list:
      active = False
    if active == False:
      self.logger.log_info(":  " +self.log_name + "No longer detecting device on : " + path_str)
      if path_str in self.active_devices_dict.keys():
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        success = nepi_drvs.killDriverNode(node_name,sub_process)
    return active


  def launchDeviceNode(self, path_str):
    self.logger.log_warn("Entering launch device function for path: " + str(path_str) + ' addr: ' + str(self.addr_str))###
    file_name = self.drv_dict['NODE_DICT']['file_name']
    node_name = self.node_launch_name + "_" + path_str.split('/')[-1] + "_" + str(self.addr_str)
    self.logger.log_info(" launching node: " + node_name)
    #Setup required param server drv_dict for discovery node
    dict_param_name = self.base_namespace + node_name + "/drv_dict"
    # Try and load save node params
    nepi_drvs.checkLoadConfigFile(node_name)
    self.logger.log_warn(" launching node: " + str(self.drv_dict))
    self.drv_dict['DEVICE_DICT'] = dict()
    self.drv_dict['DEVICE_DICT']['device_path'] = path_str
    self.drv_dict['DEVICE_DICT']['baud_str'] = self.baud_str
    self.drv_dict['DEVICE_DICT']['addr_str'] = self.addr_str
    nepi_sdk.set_param(dict_param_name,self.drv_dict)
    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = path_str)
    if success:
      self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}
      self.active_paths_list.append(path_str)
