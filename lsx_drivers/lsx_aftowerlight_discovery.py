#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
import subprocess
import time
import serial
import serial.tools.list_ports

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_drv

PKG_NAME = 'LSX_AFTOWER' # Use in display menus
FILE_TYPE = 'DISCOVERY'


TEST_NEX_DICT = {
'group': 'LSX',
'group_id': 'AFTOWER',
'pkg_name': 'LSX_AFTOWER',
'NODE_DICT': {
    'file_name': 'lsx_aftowerlight_node.py',
    'module_name': 'lsx_aftowerlight_node',
    'class_name': 'AfTowerLightNode',
},
'DRIVER_DICT': {
    'file_name': '' ,
    'module_name': '' ,
    'class_name':  ''
},
'DISCOVERY_DICT': {
    'file_name': 'lsx_aftowerlight_discovery.py',
    'module_name': 'lsx_aftowerlight_discovery',
    'class_name': 'AfTowerLightDiscovery',
    'interfaces': ['USB'],
    'options_1_dict': {
        'default_val': '9600',
        'set_val': '9600'
    },
    'options_2_dict': {
        'default_val': 'None',
        'set_val': 'None'
    },
    'method': 'AUTO', 
    'include_ids': ['29987'],
    'exclude_ids': []
},
'DEVICE_DICT': {'device_path': '/dev/ttyUSB0'},
'path': '/opt/nepi/ros/lib/nepi_drivers',
'order': 1,
'active': True,
'msg': ""
}

class AfTowerLightDiscovery:
  active_devices_dict = dict()
  node_launch_name = "af_tower_light"
  ################################################          
  def __init__(self):
    self.log_name = PKG_NAME.lower() + "_discovery" 
    nepi_msg.createMsgPublishers(self)


  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace, drv_dict = TEST_NEX_DICT):
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
    found_device = False
    #nepi_msg.printMsgWarn(self.log_name + "path_str " + path_str)
    if path_str.find('ttyUSB') != -1:
      usb_dict = nepi_drv.getSerialPortDict()
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
      nepi_msg.publishMsgInfo(self, ":  " +self.log_name + "No longer detecting device on : " + path_str)
      if path_str in self.active_devices_dict.keys():
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        success = nepi_drv.killDriverNode(node_name,sub_process)
    return active


  def launchDeviceNode(self, path_str):
    file_name = self.drv_dict['NODE_DICT']['file_name']
    node_name = self.node_launch_name + "_" + path_str.split('/')[-1]
    nepi_msg.printMsgInfo(self.log_name + "launching node: " + node_name)
    #Setup required param server drv_dict for discovery node
    dict_param_name = self.base_namespace + node_name + "/drv_dict"
    nepi_msg.publishMsgInfo(self, ":  " + self.log_name + "launching node: " + str(self.drv_dict))
    self.drv_dict['DEVICE_DICT']['device_path'] = path_str
    nepi_ros.set_param(self,dict_param_name,self.drv_dict)
    [success, msg, sub_process] = nepi_drv.launchDriverNode(file_name, node_name, device_path = path_str)
    if success:
      self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}
    return success


if __name__ == '__main__':
    AfTowerLightDiscovery()

    


        
      

 
