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
import rospy
import serial
import serial.tools.list_ports

from nepi_edge_sdk_base import nepi_nex

PKG_NAME = 'LSX_AFTOWER' # Use in display menus
FILE_TYPE = 'DISCOVERY'
CLASS_NAME = 'AfTowerLightDiscovery' # Should Match Class Name
PROCESS = 'CALL' # 'LAUNCH', 'RUN', or 'CALL'

TEST_NEX_DICT = {
    'group': 'LSX',
    'group_id': 'AFTOWER',
    'node_file_name': 'lsx_aftowerlight_node.py',
    'node_file_path': '/opt/nepi/ros/lib/nep_drivers',
    'node_module_name': 'lsx_aftowerlight_node',
    'node_class_name': 'AfTowerLightNode',
    'driver_name': "None",
    'driver_file_name': "None" ,
    'driver_file_path':  "None",
    'driver_module_name': "None" ,
    'driver_class_name':  "None",
    'driver_interfaces': ['USBSERIAL'],
    'driver_options_1': ['9600'],
    'driver_default_option_1': '9600',
    'driver_set_option_1': '9600',
    'driver_options_2': [],
    'driver_default_option_2': 'None',
    'driver_set_option_2': 'None',
    'discovery_name': 'LSX_AFTOWER', 
    'discovery_file_name': "lsx_aftowerlight_discovery.py",
    'discovery_file_path': "/opt/nepi/ros/lib/nep_drivers",
    'discovery_module_name': "lsx_aftowerlight_discovery",
    'discovery_class_name': "AfTowerLightDiscovery",
    'discovery_method': 'AUTO', 
    'discovery_ids': ['29987'],
    'discovery_ignore_ids': [],
    'device_path': '/dev/ttyUSB0',
    'order': 1,
    'active': True,
    'msg': ""
    }

class AfTowerLightDiscovery:
  active_devices_dict = dict()
  node_name = "af_tower_light"
  ################################################          
  def __init__(self, nex_dict = TEST_NEX_DICT):
    self.log_name = PKG_NAME.lower() + "_discovery" 
    self.nex_dict = nex_dict
    #rospy.logwarn(self.log_name + ":  Discovery class instantiated with nex_dict " + str(self.nex_dict))
    self.search_ids = nex_dict['discovery_ids']

  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace):
    self.base_namespace = base_namespace
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    #rospy.logwarn(self.log_name + ":  active path list: " + str(self.active_paths_list))
    self.base_namespace = base_namespace
    self.path_list = []
    ports = serial.tools.list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      rospy.logdebug(self.node_name + ": Found serial_port at: " + loc)
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
    #rospy.logwarn(self.log_name + ":  path_str " + path_str)
    if path_str.find('ttyUSB') != -1:
      usb_dict = nepi_nex.getSerialPortDict()
      #rospy.logwarn(self.log_name + ":  serial_port_dict " + str(usb_dict))
      if path_str in usb_dict.keys():
        #rospy.logwarn(self.log_name + ":  search ids " + str(self.search_ids))
        #rospy.logwarn(self.log_name + ":  serial_port product id " + str(usb_dict[path_str]['product_id']))
        if str(usb_dict[path_str]['product_id']) in self.search_ids:
          #rospy.logwarn(self.log_name + ":  found device on path: " + path_str)
          found_device = True
    return found_device


  def checkOnDevice(self,path_str):
    active = self.checkForDevice(path_str)
    return active


  def launchDeviceNode(self, path_str):
    file_name = self.nex_dict['node_file_name']
    ros_node_name = self.node_name + "_" + path_str.split('/')[-1]
    rospy.loginfo(self.log_name + ":  launching node: " + ros_node_name)
    #Setup required param server nex_dict for discovery node
    dict_param_name = self.base_namespace + ros_node_name + "/nex_dict"
    #rospy.logwarn(self.log_name + ":  launching node: " + str(self.nex_dict))
    self.nex_dict['device_path'] = path_str
    rospy.set_param(dict_param_name,self.nex_dict)
    [success, msg, sub_process] = nepi_nex.launchDriverNode(file_name, ros_node_name, device_path = path_str)
    if success:
      self.active_devices_dict[path_str] = {'ros_node_name': ros_node_name, 'sub_process': sub_process}
      self.active_paths_list.append(path_str)


if __name__ == '__main__':
    AfTowerLightDiscovery()

    


        
      

 
