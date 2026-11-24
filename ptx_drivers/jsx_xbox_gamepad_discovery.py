#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi drivers (nepi_drivers) repo
# (see https://github.com/nepi-engine/nepi_drivers)
#
# License: NEPI Drivers repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
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
import re
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

from nepi_api.messages_if import MsgIF

PKG_NAME = 'JSX_XBOX_GAMEPAD' # Use in display menus
FILE_TYPE = 'DISCOVERY'

class XboxGamepadDiscovery:

  NODE_LOAD_TIME_SEC = 10
  CHECK_INTERVAL_S = 2.0

  INPUT_FOLDER = '/dev/input'
  BY_ID_FOLDER = '/dev/input/by-id'

  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  check_for_devices = True

  drv_dict = dict()
  deviceList = []


  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"

  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF()
    self.msg_if.pub_info("Starting Node Initialization Processes")

    # Instance copies of the mutable class state
    self.launch_time_dict = dict()
    self.dont_retry_list = []
    self.deviceList = []

    ########################
    # Update discovery options
    success = self.updateDiscoveryOptions()
    if success == False:
        self.msg_if.pub_debug("Shutting down because failed to get Driver Dict")
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because failed to get Driver Dict")
        return

    ########################
    # Start node processes
    nepi_sdk.start_timer_process((1), self.detectAndManageDevices, oneshot = True)
    nepi_sdk.start_timer_process((1), self.updateDriverDictCb, oneshot = True)
    nepi_sdk.on_shutdown(self.cleanup_actions)

    ########################
    # Now start the node
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.spin()




  #**********************
  # Discovery functions
  #**********************

  def updateDriverDictCb(self,timer):
    updated = self.updateDiscoveryOptions()
    nepi_sdk.start_timer_process((1), self.updateDriverDictCb, oneshot = True)

  def updateDiscoveryOptions(self):
    success = False
    last_drv_dict = copy.deepcopy(self.drv_dict)
    self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
    if len(list(self.drv_dict.keys())) == 0:
      self.msg_if.pub_warn("Failed to load Driver dict ")
      return success
    if 'DISCOVERY_DICT' not in self.drv_dict.keys():
      self.msg_if.pub_warn("Discovery dict missing in Driver dict ")
      return success
    if last_drv_dict != self.drv_dict:
      self.msg_if.pub_debug("Updated Driver Dict: " + str(self.drv_dict))
    success = True

    if 'retry_enabled' in self.drv_dict.keys():
      self.retry = self.drv_dict['retry_enabled']
    else:
      self.retry = True
    if self.retry == True:
      self.dont_retry_list = []
    return success


  def detectAndManageDevices(self, timer): # Extra arg since this is a Timer callback
    if self.check_for_devices == False:
      self.msg_if.pub_warn("Stopping device discovery process")
      return
    success = False
    try:
      found_devices = self.getJoystickDevices()
    except Exception as e:
      self.msg_if.pub_warn("Failed to list joystick devices: " + str(e), throttle_s = 10.0)
      found_devices = []
    active_paths = [device['device_path'] for device in found_devices]

    for found in found_devices:
      path_str = found['device_path']
      known_device = False
      for device in list(self.deviceList):
        if device['device_path'] != path_str:
          continue
        known_device = True
        node_namespace = device['node_namespace']
        ####################
        # Restart a known device whose node stopped (it exits after a read failure)
        if not self.deviceNodeIsRunning(node_namespace) and device['node_name'] not in self.dont_retry_list:
          self.stopAndPurgeDeviceNode(node_namespace)
          if device['node_name'] in self.dont_retry_list:
            self.msg_if.pub_debug("node " + device['node_name'] + " is not running. WILL NOT RESTART")
          elif success == False: # Only start one device per check
            self.msg_if.pub_info("Restarting controller node on path: " + path_str)
            success = self.startDeviceNode(found)

      ####################
      # Start device on this path if not known
      if known_device == False and success == False: # Only start one device per check
        self.msg_if.pub_debug("Found new controller on path: " + path_str, throttle_s = -1)
        success = self.startDeviceNode(found)

    ######################
    # Stop nodes whose device is gone (unplugged)
    for device in list(self.deviceList):
      if device['device_path'] not in active_paths:
        self.msg_if.pub_info("Device path: " + device['device_path'] + " no longer present. Stopping node " + device['node_name'])
        self.stopAndPurgeDeviceNode(device['node_namespace'])

    #########################
    # Setup next check process
    nepi_sdk.start_timer_process(self.CHECK_INTERVAL_S, self.detectAndManageDevices, oneshot = True)


  def getJoystickDevices(self):
    # js numbers follow plug order, so a controller is named and opened by its
    # /dev/input/by-id/*-joystick link when udev made one
    js_paths = []
    if os.path.isdir(self.INPUT_FOLDER):
      for entry in sorted(os.listdir(self.INPUT_FOLDER)):
        if re.match(r'^js[0-9]+$', entry):
          js_paths.append(os.path.join(self.INPUT_FOLDER, entry))
    by_id_dict = dict()
    if os.path.isdir(self.BY_ID_FOLDER):
      for entry in sorted(os.listdir(self.BY_ID_FOLDER)):
        if entry.endswith('-joystick') and not entry.endswith('-event-joystick'):
          link_path = os.path.join(self.BY_ID_FOLDER, entry)
          by_id_dict[os.path.realpath(link_path)] = link_path
    devices = []
    for js_path in js_paths:
      device_path = by_id_dict.get(os.path.realpath(js_path), js_path)
      devices.append({'device_path': device_path,
                      'device_name': self.getDeviceName(device_path)})
    return devices


  def getDeviceName(self, device_path):
    name = os.path.basename(device_path)
    if name.startswith('usb-'):
      name = name[len('usb-'):]
    if name.endswith('-joystick'):
      name = name[:-len('-joystick')]
    # ROS names allow letters, digits and underscores only
    name = re.sub(r'[^A-Za-z0-9_]', '_', name).lower()
    name = re.sub(r'_+', '_', name).strip('_')
    return 'jsx_' + name


  def startDeviceNode(self, found):
    success = False
    path_str = found['device_path']
    launch_id = path_str

    # Check if should try to launch
    if launch_id in self.launch_time_dict.keys():
      launch_time = self.launch_time_dict[launch_id]
      cur_time = nepi_utils.get_time()
      if (cur_time - launch_time) < self.NODE_LOAD_TIME_SEC:
        return False

    device_name = nepi_utils.get_clean_name(found['device_name'])
    node_name = nepi_system.get_device_alias(device_name)
    node_namespace = os.path.join(self.base_namespace, node_name)

    for device in self.deviceList:
      if device['node_name'] == node_name:
        return False
    if node_name in self.dont_retry_list:
      return False

    self.msg_if.pub_info("Launching controller node " + node_namespace + " for " + path_str)
    #Setup required param server drv_dict for the device node
    self.drv_dict['DEVICE_DICT']={'device_name': device_name,
                                  'device_path': path_str
                                  }
    dict_param_name = os.path.join(self.base_namespace,node_name + "/drv_dict")
    nepi_sdk.set_param(dict_param_name,self.drv_dict)

    file_name = self.drv_dict['NODE_DICT']['file_name']
    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name)
    if success:
      self.deviceList.append({'device_class': 'jsx', 'device_path': path_str,
                              'node_name': node_name, 'node_namespace': node_namespace,
                              'node_subprocess': sub_process})
      self.msg_if.pub_info(" Launched node: " + node_name)
    else:
      self.msg_if.pub_info(" Failed to launch node: " + node_name + " with msg: " + msg)
      if self.retry == False:
        self.msg_if.pub_info(" Will not try relaunch for node: " + node_name)
        self.dont_retry_list.append(node_name)
      else:
        self.msg_if.pub_info(" Will attempt relaunch for node: " + node_name + " in " + str(self.NODE_LOAD_TIME_SEC) + " secs")
    self.launch_time_dict[launch_id] = nepi_utils.get_time()
    return success

  def stopAndPurgeDeviceNode(self, node_namespace = 'All'):
    if node_namespace == 'All':
      self.check_for_devices = False
    for device in list(self.deviceList):
      namespace = device['node_namespace']
      node_name = os.path.basename(namespace)
      if namespace == node_namespace or node_namespace == 'All':
        sub_process = device['node_subprocess']
        self.msg_if.pub_info("Killing device node: " + node_name)
        self.deviceList.remove(device)
        if self.retry == False:
          self.msg_if.pub_info("Adding device node to don't retry list: " + node_name)
          self.dont_retry_list.append(node_name)
        success = nepi_drvs.killDriverNode(node_name,sub_process)
        if success == False:
          self.msg_if.pub_warn("Unable to kill device node " + node_name)
        else:
          self.msg_if.pub_debug("Node killed. Removed device from active list " + node_name)


  def deviceNodeIsRunning(self, node_namespace):
    for device in self.deviceList:
      if device['node_namespace'] == node_namespace:
        if device['node_subprocess'].poll() is not None:
          return False
        else:
          return True
    # If we get here, didn't find the node in our list
    self.msg_if.pub_debug("cannot check run status of unknown node " + node_namespace)
    return False


  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    self.stopAndPurgeDeviceNode('All')

if __name__ == '__main__':
  node = XboxGamepadDiscovery()
