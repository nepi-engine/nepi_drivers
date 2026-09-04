#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

# TEMPORARY TEST FIXTURE. Delete with the two sibling lsx_settings_test_* files.
# Reports exactly one virtual device. Probes no bus, opens no port, touches no
# hardware. The device path is a fixed label, not a /dev/ node.

import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system


PKG_NAME = 'LSX_SETTINGS_TEST'
FILE_TYPE = 'DISCOVERY'

#########################################
# Discover Method
#########################################


class SettingsTestDiscovery:

  NODE_LOAD_TIME_SEC = 10

  # The one virtual device this fixture reports. Not a real path -- nothing
  # opens it. It only has to be a stable key in active_paths_list.
  VIRTUAL_PATH = 'settings_test'

  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  active_devices_dict = dict()
  node_launch_name = "settings_test"

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
  def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled = True):
    self.drv_dict = drv_dict
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace

    # Retry behavior
    self.retry = retry_enabled
    if self.retry == True:
      self.dont_retry_list = []

    path_str = self.VIRTUAL_PATH

    ### Purge a node that has exited so it gets relaunched
    if path_str in self.active_devices_dict.keys():
      if self.checkOnDevice(path_str) == False:
        del self.active_devices_dict[path_str]
        if path_str in self.active_paths_list:
          self.active_paths_list.remove(path_str)

    ### The device is always "found" -- that is the whole point of the fixture
    if path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
      success = self.launchDeviceNode(path_str)
      if success:
        self.active_paths_list.append(path_str)
    return self.active_paths_list
  ################################################

  def checkOnDevice(self, path_str):
    active = True
    if path_str in self.active_devices_dict.keys():
      sub_process = self.active_devices_dict[path_str].get('sub_process')
      if sub_process is not None and sub_process.poll() is not None:
        active = False  # node process has exited
    if active == False:
      self.logger.log_info("Virtual device node has exited: " + path_str)
      path_entry = self.active_devices_dict[path_str]
      nepi_drvs.killDriverNode(path_entry['node_name'], path_entry['sub_process'])
    return active


  def launchDeviceNode(self, path_str):
    success = False
    launch_id = path_str

    # Back off so a node that dies at startup does not relaunch in a tight loop
    launch_check = True
    if launch_id in self.launch_time_dict.keys():
      launch_time = self.launch_time_dict[launch_id]
      cur_time = nepi_sdk.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LOAD_TIME_SEC
    if launch_check == False:
      return False

    ### Start Node Launch Process
    file_name = self.drv_dict['NODE_DICT']['file_name']
    device_name = self.node_launch_name
    node_name = nepi_system.get_device_alias(device_name)

    # Setup required param server drv_dict for the node. The node reads its own
    # page port back out of DISCOVERY_DICT OPTIONS, so the whole drv_dict rides
    # along unchanged.
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
    self.drv_dict['DEVICE_DICT'] = {'device_name': device_name,
                                    'device_path': path_str}
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    nepi_sdk.set_param(dict_param_name, self.drv_dict)

    self.logger.log_info("Launching virtual settings test node: " + node_name)
    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = path_str)

    if success == True:
      self.launch_time_dict[launch_id] = nepi_sdk.get_time()
      self.logger.log_info("Launched node: " + node_name)
      self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}
    else:
      self.logger.log_info("Failed to launch node: " + node_name + " with msg: " + str(msg))
      if self.retry == False:
        self.logger.log_info("Will not try relaunch for node: " + node_name)
        self.dont_retry_list.append(path_str)
    return success


  def killAllDevices(self, active_paths_list):
    path_purge_list = list(self.active_devices_dict.keys())
    for path_str in path_purge_list:
      path_entry = self.active_devices_dict[path_str]
      node_name = path_entry['node_name']
      sub_process = path_entry['sub_process']
      if self.retry == False:
        self.dont_retry_list.append(path_str)
      self.logger.log_info("Killing node: " + node_name)
      nepi_drvs.killDriverNode(node_name, sub_process)

    for path_str in path_purge_list:
      try:
        del self.active_devices_dict[path_str]
      except Exception as e:
        self.logger.log_warn("Failed to remove device from active devices dict: " + str(e))
      try:
        self.active_paths_list.remove(path_str)
      except Exception as e:
        pass
      try:
        active_paths_list.remove(path_str)
      except Exception as e:
        pass

    nepi_sdk.sleep(1)
    return active_paths_list
