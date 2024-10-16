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

from nepi_edge_sdk_base import nepi_drv
from nepi_edge_sdk_base import nepi_msg

PKG_NAME = 'PTX_IQR' # Use in display menus
FILE_TYPE = 'DISCOVERY'
DISCOVERY_DICT = dict(
  class_name = 'IqrPanTiltDiscovery',
  process = 'CALL', # 'LAUNCH', 'RUN', or 'CALL'
  method = 'AUTO',  # 'AUTO', 'MANUAL', or 'OTHER' if managed by seperate application
  include_ids =  ['iqr_pan_tilt'],  # List of string identifiers for discovery process
  exclude_ids = [], # List of string identifiers for discovery process
  interfaces = ['USB'], # 'USB','IP','SERIALUSB','SERIAL','CANBUS'
  option_1_dict = dict(
    name = 'None',
    options = [], # List of string options. Selected option passed to driver
    default_val = 'None'
  ),
  option_2_dict = dict(
    name = 'None',
    options = [], # List of string options. Selected option passed to driver
    default_val = 'None'
  )
)

TEST_DRV_DICT = {
'group': 'PTX',
'group_id': 'IQR',
'pkg_name': 'PTX_IQR',
'NODE_DICT': {
    'file_name': 'ptx_iqr_node.py',
    'module_name': 'ptx_iqr_node',
    'class_name': 'IqrPanTiltNode',
},
'DRIVER_DICT': {
    'file_name': '' ,
    'module_name': '' ,
    'class_name':  ''
},
'DISCOVERY_DICT': {
    'file_name': 'ptx_iqr_discovery.py',
    'module_name': 'ptx_iqr_discovery',
    'class_name': 'IqrPanTiltDiscovery',
    'interfaces': ['USB'],
    'options_1_dict': {
        'default_val': 'None',
        'set_val': 'None'
    },
    'options_2_dict': {
        'default_val': 'None',
        'set_val': 'None'
    },
    'method': 'AUTO', 
    'include_ids': ['iqr_pan_tilt'],
    'exclude_ids': []
},
'DEVICE_DICT': {},
'path': '/opt/nepi/ros/lib/nepi_drivers',
'order': 1,
'active': True,
'msg': ""
}

class IqrPanTiltDiscovery:
  log_name = PKG_NAME.lower() + "_discovery" + " : "
  active_devices_dict = dict()
  node_launch_name = "iqr_pan_tilt"
  ################################################          
  def __init__(self):
    pass 

  ##########  DRV Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace, drv_dict = TEST_DRV_DICT):
    self.drv_dict = drv_dict
    #nepi_msg.printMsg(self.log_name + "Got drv_dict : " + str(self.drv_dict))
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    
    # Get required data from drv_dict
    self.includeDevices = self.drv_dict['DISCOVERY_DICT']['include_ids']
    self.excludedDevices = self.drv_dict['DISCOVERY_DICT']['exclude_ids']

    ### Purge Unresponsive Connections
    path_purge_list = []
    for path_str in self.active_devices_dict.keys():
        success = self.checkOnDevice(path_str)
        if success:
          path_purge_list.append(path_str) 
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del  self.active_devices_dict[path_str]

    ### Checking for devices on available paths
    for path_str in self.available_paths_list:
      if path_str not in self.active_paths_list:
        found_device = self.checkForDevice(path_str)
        if found_device == True:
          nepi_msg.printMsg(self.log_name + "Found device on path: " + path_str)
          success = self.launchDeviceNode(path_str)
          if success == True:
            self.active_paths_list.append(path_str)
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
      path_entry = self.active_devices_dict[path_str]
      node_launch_namespace = path_entry['node_launch_namespace']
      sub_process = path_entry['sub_process']
      success = nepi_drv.killDriverNode(node_launch_namespace,sub_process)
      if success:
        active = False
    return active


  def launchDeviceNode(self, path_str):
    file_name = 'iqr_ros_pan_tilt_node'
    ros_node_launch_name = self.node_launch_name + "_" + path_str.split('/')[-1]
    nepi_msg.printMsg(self.log_name + "***Launching node with name: " + ros_node_launch_name)
    [success, msg, sub_process] = nepi_drv.launchDriverNode(file_name, ros_node_launch_name, device_path = path_str)
    if success:
      self.active_devices_dict[path_str] = {'ros_node_launch_name': ros_node_launch_name, 'sub_process': sub_process}
    return success
    


if __name__ == '__main__':
    IqrPanTiltDiscovery()

    


        
      

 
