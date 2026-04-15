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


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system
from nepi_sdk import nepi_serial


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

  dont_retry_list = []

  includeDevices = []
  excludedDevices = ['ttyACM', 'ttyTCU', 'ttyTHS']

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
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace,drv_dict, retry_enabled = True):
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

    # Retry behavior
    self.retry = retry_enabled
    if self.retry == True:
        self.dont_retry_list = []
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
          if self.retry == False:
            self.dont_retry_list.append(path_str)
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
        if found and path_str not in self.dont_retry_list:
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
            nepi_sdk.sleep(.005)
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
    elif path_str in self.active_devices_dict:
      sub_process = self.active_devices_dict[path_str].get('sub_process')
      if sub_process is not None and sub_process.poll() is not None:
        active = False  # node process has exited
    if active == False:
      self.logger.log_info("No longer detecting device on : " + path_str)
      if path_str in self.active_devices_dict.keys():
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        self.dont_retry_list.append(path_str)
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
    device_name = self.node_launch_name + "_" + path_str.split('/')[-1] + "_" + str(self.addr_str)
    node_name = nepi_system.get_device_alias(device_name)
    self.logger.log_warn(" launching node: " + node_name)

    
    #Setup required param server drv_dict for discovery node
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace,node_name + "/drv_dict")
    self.drv_dict['DEVICE_DICT']={'device_name': device_name,
                                  'device_path': path_str}
    self.drv_dict['DEVICE_DICT']['baud_str'] = self.baud_str
    self.drv_dict['DEVICE_DICT']['addr_str'] = self.addr_str
    #self.logger.log_info(" launching node: " + str(self.drv_dict))
    self.logger.log_warn("Launching node  with path: " + path_str + " baudrate: " + self.baud_str + " addr: " + self.addr_str)
    #self.logger.log_warn(" launching node: " + str(self.drv_dict))
    self.launch_time_dict[path_str] = nepi_sdk.get_time()
    nepi_sdk.set_param(dict_param_name,self.drv_dict)



    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = path_str)


    if success == True:
      # Process luanch results
      self.launch_time_dict[launch_id] = nepi_sdk.get_time()
      self.logger.log_warn("Launched node: " + node_name)
      self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}
    else:
      self.logger.log_info("Failed to lauch node: " + node_name + " with msg: " + msg)
      if self.retry == False:
        self.logger.log_info("Will not try relaunch for node: " + node_name)
        self.dont_retry_list.append(path_str)
    return success


    
  def killAllDevices(self,active_paths_list):
    #self.logger.log_warn("Entering Kill All Devices function for path: " + str(path_str))###
    path_purge_list = []
    for key in self.active_devices_dict.keys():
      path_purge_list.append(key)
    self.logger.log_warn("Killing Devices: " + str(path_purge_list))
    for path_str in path_purge_list:
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        if self.retry == False:
          self.logger.log_warn("Will not try relaunch for node: " + node_name)
          self.dont_retry_list.append(path_str)
        success = nepi_drvs.killDriverNode(node_name,sub_process)

    for path_str in path_purge_list:
        try:
          del  self.active_devices_dict[path_str]
        except Exception as e:
          self.logger.log_warn("Failed to remove driver from active paths dict: " + str(e))

        try:
          self.logger.log_warn("Removing path from active paths list: " + str(path_str))
          self.active_paths_list.remove(path_str)
          active_paths_list.remove(path_str)
          self.logger.log_warn("Updated active paths list: " + str(active_paths_list))
        except Exception as e:
          self.logger.log_warn("Failed to remove driver from active paths list: " + str(e))


        try:
          self.logger.log_warn("Removing path from class active paths list: " + str(path_str))
          self.active_paths_list.remove(path_str)
        except Exception as e:
          #self.logger.log_warn("Failed to remove driver from class active paths list: " + str(e))
          pass


    nepi_sdk.sleep(1)
    return active_paths_list