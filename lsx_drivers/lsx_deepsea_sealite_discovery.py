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
from nepi_sdk import nepi_drv
from nepi_sdk import nepi_msg

PKG_NAME = 'LSX_DEEPSEA_SEALITE'
FILE_TYPE = 'DISCOVERY'

#########################################
# Discover Method
#########################################


### Function to try and connect to device and also monitor and clean up previously connected devices
class SealiteDiscovery:

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
    self.log_name = PKG_NAME.lower() + "_discovery" 
    nepi_msg.createMsgPublishers(self)
    time.sleep(1)
    nepi_msg.publishMsgInfo(self, ":" + self.log_name + ": Starting Initialization")
    nepi_msg.publishMsgInfo(self, ":" + self.log_name + ": Initialization Complete")


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
      nepi_msg.publishMsgWarn(self, ": " + self.log_name + ": Starting discovery with drv_dict " + str(drv_dict))#
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
      nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Failed to load options " + str(e))#
      return None
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
        #nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ": Looking for path: " + path_str)
        #nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ": In path_list: " + str(self.active_paths_list))
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
    #nepi_msg.publishMsgWarn(self, ": " + self.log_name + ": Running device search with path: " + path_str + " and buadlist " + str(self.baudrate_list))
    if path_str not in self.active_paths_list:
      for baud_str in self.baudrate_list:
        self.baud_str = baud_str
        self.baud_int = int(baud_str)
        try:
          # Try and open serial port
          serial_port = serial.Serial(path_str,self.baud_int,timeout = 1)
        except Exception as e:
          nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ": Unable to open serial port " + path_str + " with baudrate: " + baud_str + "(" + str(e) + ")")
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
            #nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ": Got a serial read/write error: " + str(e))
            continue
          if len(response) > 2:
            nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ": Got response: " + response)
            if response[3] == ',':
              self.addr_str = response[0:3]
              try:
                addr_int = int(addr)
                nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ": Found device at path: " + path_str)
                nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ": Found device at baudrate: " + baud_str)
                nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ": Found device at address: " + self.addr_str)
                found_device = True
                return found_device
              except Exception as a:
                nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ": Returned device message not valid (" + str(a) + ")")
        # Clean up the serial port
        serial_port.close()
    return found_device


  def checkOnDevice(self,path_str):
    active = True
    if path_str not in self.available_paths_list:
      active = False
    if active == False:
      nepi_msg.publishMsgInfo(self, ":" + self.log_name + ": No longer detecting device on : " + path_str)
      if path_str in self.active_devices_dict.keys():
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        success = nepi_drv.killDriverNode(node_name,sub_process)
    return active


  def launchDeviceNode(self, path_str):
    file_name = self.drv_dict['NODE_DICT']['file_name']
    node_name = self.node_launch_name + "_" + path_str.split('/')[-1] + "_" + str(self.addr_str)
    nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ":  launching node: " + node_name)
    #Setup required param server drv_dict for discovery node
    dict_param_name = self.base_namespace + node_name + "/drv_dict"
    # Try to load node saved device config
    nepi_drv.checkLoadConfigFile(node_name)
    # Store drv info for node to use
    self.drv_dict['DEVICE_DICT'] = dict()
    self.drv_dict['DEVICE_DICT']['device_path'] = path_str
    self.drv_dict['DEVICE_DICT']['baud_str'] = self.baud_str
    self.drv_dict['DEVICE_DICT']['addr_str'] = self.addr_str
    #nepi_msg.publishMsgInfo(self, ":" + self.log_name + ":  launching node: " + str(self.drv_dict))
    nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ": Launching node  with path: " + path_str + " baudrate: " + self.baud_str + " addr: " + self.addr_str)
    #nepi_msg.publishMsgInfo(self,  ":" + self.log_name + ":  launching node: " + str(self.drv_dict))
    nepi_ros.set_param(self,dict_param_name,self.drv_dict)
    [success, msg, sub_process] = nepi_drv.launchDriverNode(file_name, node_name, device_path = path_str)
    if success:
      self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}
      self.active_paths_list.append(path_str)
