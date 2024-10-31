#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# NEPI Auto Discovery Script for Sealite devices

import os
import subprocess
import time
import serial
import serial.tools.list_ports

from nepi_ros_interfaces.srv import LSXCapabilitiesQuery

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_drv
from nepi_edge_sdk_base import nepi_msg

PKG_NAME = 'LSX_SEALITE' # Use in display menus
FILE_TYPE = 'DISCOVERY'


TEST_NEX_DICT = {
'group': 'LSX',
'group_id': 'SEALITE',
'pkg_name': 'LSX_SEALITE',
'NODE_DICT': {
    'file_name': 'lsx_sealite_node.py',
    'module_name': 'lsx_sealite_node',
    'class_name': 'SealiteNode',
},
'DRIVER_DICT': {
    'file_name': '' ,
    'module_name': '' ,
    'class_name':  ''
},
'DISCOVERY_DICT': {
    'file_name': 'lsx_sealite_discovery.py',
    'module_name': 'lsx_sealite_discovery',
    'class_name': 'SealiteDiscovery',
    'interfaces': ['SERIAL','USBSERIAL'],
    'options_1_dict': {
        'default_val': '57600',
        'set_val': '57600'
    },
    'options_2_dict': {
        'default_val': '10',
        'set_val': '10'
    },
    'method': 'AUTO', 
    'include_ids': [],
    'exclude_ids': ['ttyACM']
},
'DEVICE_DICT': {'device_path': '/dev/ttyUSB0','baud_int':57600, 'addr': '001'},
'path': '/opt/nepi/ros/lib/nepi_drivers',
'order': 1,
'active': True,
'msg': ""
}


#########################################
# Sealite Discover Method
#########################################

### Function to try and connect to device and also monitor and clean up previously connected devices
class SealiteDiscovery:
  active_devices_dict = dict()
  node_launch_name = "sealite"
  baud_int = 57600
  addr_str = "001"
  ################################################          
  def __init__(self):
    self.log_name = PKG_NAME.lower() + "_discovery" 
    nepi_msg.createMsgPublishers(self)

  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace,drv_dict):
    self.drv_dict = drv_dict
    #nepi_msg.publishMsgInfo(self, ":  " + self.log_name + "Got drv_dict : " + str(self.drv_dict))
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    
    # Get required data from drv_dict
    self.includeDevices = self.drv_dict['DISCOVERY_DICT']['include_ids']
    self.excludedDevices = self.drv_dict['DISCOVERY_DICT']['exclude_ids']

    baudrate_list = []
    baudrate_options = self.drv_dict["DISCOVERY_DICT"]['option_1_dict']['options']
    baudrate_sel = self.drv_dict["DISCOVERY_DICT"]['option_1_dict']['set_val']
    if baudrate_sel != "All":
      baudrate_list.append(int(baudrate_sel))
    else:
      for baudrate in baudrate_options:
        if baudrate != "All":
          baudrate_list.append(int(baudrate))
    self.baudrate_list = baudrate_list

    addr_search_range = int(self.drv_dict["DISCOVERY_DICT"]['option_2_dict']['set_val'])
    self.addr_search_list = list(range(1,addr_search_range+1))


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
        #nepi_msg.publishMsgInfo(self, ":  " + self.log_name + ": Looking for path: " + path_str)
        #nepi_msg.publishMsgInfo(self, ":  " + self.log_name + ": In path_list: " + str(self.active_paths_list))
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
    #nepi_msg.publishMsgInfo(self, ":  " + self.log_name + ":  path_str " + path_str)
    if path_str not in self.active_paths_list:
      for baud_int in self.baudrate_list:
        self.baud_int = baud_int
        try:
          # Try and open serial port
          serial_port = serial.Serial(path_str,baud_int,timeout = 1)
        except Exception as e:
          nepi_msg.publishMsgInfo(self, ":  " + self.log_name + ": Unable to open serial port " + path_str + " with baudrate: " + str(baud_int) + "(" + str(e) + ")")
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
            #nepi_ros.logerr('%s: Got a serial read/write error (%s)', self.log_name, str(e))
            break
          if len(response) > 2:
            nepi_msg.publishMsgInfo(self, ":  " + self.log_name + ": Got response: " + response)
            if response[3] == ',':
              self.addr_str = response[0:3]
              try:
                addr_int = int(addr)
                nepi_msg.publishMsgInfo(self, ":  " + self.log_name + ": Found device at address: " + self.addr_str)
                found_device = True
                break # Don't check any more addresses
              except Exception as a:
                nepi_msg.publishMsgInfo(self, ":  " + self.log_name + ": Returned device message not valid (" + str(a) + ")")
        # Clean up the serial port
        serial_port.close()
    return found_device


  def checkOnDevice(self,path_str):
    active = True
    if path_str not in self.available_paths_list:
      active = False
    if active == False:
      nepi_msg.publishMsgInfo(self, ":  " +self.log_name + "No longer detecting device on : " + path_str)
      if path_str in self.active_devices_dict.keys():
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        success = nepi_drv.killDriverNode(node_name,sub_process)
    return active


  def launchDeviceNode(self, path_str):
    file_name = self.drv_dict['NODE_DICT']['file_name']
    node_name = self.node_launch_name + "_" + path_str.split('/')[-1] + "_" + addr_str
    nepi_msg.publishMsgInfo(self, ":  " + self.log_name + ":  launching node: " + node_name)
    #Setup required param server drv_dict for discovery node
    dict_param_name = self.base_namespace + node_name + "/drv_dict"
    #nepi_msg.publishMsgInfo(self, ":  " + self.log_name + ":  launching node: " + str(self.drv_dict))
    self.drv_dict['DEVICE_DICT']['device_path'] = path_str
    self.drv_dict['DEVICE_DICT']['baud_int'] = self.baud_int
    self.drv_dict['DEVICE_DICT']['addr_str'] = self.addr_str
    nepi_ros.set_param(self,dict_param_name,self.drv_dict)
    [success, msg, sub_process] = nepi_drv.launchDriverNode(file_name, node_name, device_path = path_str)
    if success:
      self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}
      self.active_paths_list.append(path_str)
