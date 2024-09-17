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

from nepi_edge_sdk_base import nepi_nex

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

PKG_NAME = 'PTX_IQR' # Use in display menus
FILE_TYPE = 'DISCOVERY'
CLASS_NAME = 'IqrPanTiltDiscovery' # Should Match Class Name
PROCESS = 'CALL' # 'LAUNCH', 'RUN', or 'CALL'



TEST_NEX_DICT = {
    'group': 'PTX',
    'group_id': 'IQR',
    'node_file_name': 'ptx_iqr_node.py',
    'node_file_path': '/opt/nepi/ros/lib/nep_drivers',
    'node_module_name': 'ptx_iqr_node',
    'node_class_name': 'IqrPanTiltNode',
    'driver_name': "None",
    'driver_file_name': "None" ,
    'driver_file_path':  "None",
    'driver_module_name': "None" ,
    'driver_class_name':  "None",
    'driver_interfaces': [],
    'driver_options_1': [],
    'driver_default_option_1': 'None',
    'driver_set_option_1': 'None',
    'driver_options_2': [],
    'driver_default_option_2': 'None',
    'driver_set_option_2': 'None',
    'discovery_name': 'PTX_IQR', 
    'discovery_file_name': 'ptx_iqr_discovery.py',
    'discovery_file_path': '/opt/nepi/ros/lib/nepi_drivers',
    'discovery_module_name': 'ptx_iqr_discovery',
    'discovery_class_name': 'IqrPanTiltDiscovery',
    'discovery_method': 'AUTO', 
    'discovery_ids': ['iqr_pan_tilt'],
    'discovery_ignore_ids': [],
    'device_dict': {},
    'order': 1,
    'active': True,
    'msg': ""
    }

class IqrPanTiltDiscovery:
  active_devices_dict = dict()
  node_name = "iqr_pan_tilt"
  ################################################          
  def __init__(self, nex_dict = TEST_NEX_DICT):
    self.log_name = PKG_NAME.lower() + "_discovery" 
    self.nex_dict = nex_dict
    self.search_id = nex_dict['discovery_ids'][0]

  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace):
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
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
    if path_str.find(self.search_id) != -1:
      found_device = True
    return found_device


  def checkOnDevice(self,path_str):
    active = True
    if path_str not in self.available_paths_list:
      path_entry = self.active_devices_dict[path_str]
      node_namespace = path_entry['node_namespace']
      sub_process = path_entry['sub_process']
      success = nepi_nex.killDriverNode(node_namespace,sub_process)
      if success:
        active = False
    return active


  def launchDeviceNode(self, path_str):
    file_name = 'iqr_ros_pan_tilt_node'
    ros_node_name = self.node_name + "_" + path_str.split('/')[-1]
    [success, msg, sub_process] = nepi_nex.launchDriverNode(file_name, ros_node_name, device_path = path_str)
    if success:
      self.active_devices_dict[path_str] = {'ros_node_name': ros_node_name, 'sub_process': sub_process}
      self.active_paths_list.append(path_str)


if __name__ == '__main__':
    IqrPanTiltDiscovery()

    


        
      

 
