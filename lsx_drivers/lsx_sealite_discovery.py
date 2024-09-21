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
import rospy
import serial
import serial.tools.list_ports

from nepi_ros_interfaces.srv import LSXCapabilitiesQuery

from nepi_edge_sdk_base import nepi_nex

PKG_NAME = 'LSX_SEALITE' # Use in display menus
FILE_TYPE = 'DISCOVERY'
CLASS_NAME = 'SealiteDiscovery' # Should Match Class Name
PROCESS = 'CALL' # 'LAUNCH', 'RUN', or 'CALL'

TEST_NEX_DICT = {
    'group': 'LSX',
    'group_id': 'SEALITE',
    'node_file_name': 'lsx_sealite_node.py',
    'node_file_path': '/opt/nepi/ros/lib/nep_drivers',
    'node_module_name': 'lsx_sealite_node',
    'node_class_name': 'SealiteNode',
    'driver_name': "None",
    'driver_file_name': "None" ,
    'driver_file_path':  "None",
    'driver_module_name': "None" ,
    'driver_class_name':  "None",
    'driver_interfaces': ['USBSERIAL'],
    'driver_options_1': ['All','9600','19200','57600'],
    'driver_default_option_1': '57600',
    'driver_set_option_1': '57600',
    'driver_options_2': [],
    'driver_default_option_2': 'None',
    'driver_set_option_2': 'None',
    'discovery_name': 'LSX_SEALITE', 
    'discovery_file_name': "lsx_sealite_discovery.py",
    'discovery_file_path': "/opt/nepi/ros/lib/nep_drivers",
    'discovery_module_name': "lsx_sealite_discovery",
    'discovery_class_name': "SealiteDiscovery",
    'discovery_method': 'AUTO', 
    'discovery_ids': [],
    'discovery_ignore_ids': ['ttyACM'],
    'device_path': '/dev/ttyUSB0',
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
  node_name = "sealite"
  baud_int = 57600
  addr_str = "001"
  ################################################          
  def __init__(self):
    self.log_name = PKG_NAME.lower() + "_discovery" 

  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace,nex_dict):
    self.nex_dict = nex_dict
    #rospy.logwarn(self.log_name + ":  Discovery class instantiated with nex_dict " + str(self.nex_dict))
    baudrate_list = []
    baudrate_sel = self.nex_dict['driver_set_option_1']
    if baudrate_sel != "All":
      baudrate_list.append(int(baudrate_sel))
    else:
      for baudrate in baudrate_list:
        if baudrate != "All":
          baudrate_list.append(int(baudrate))
    self.baudrate_list = baudrate_list
    addr_range = int(self.nex_dict['driver_set_option_2'])
    self.addr_list = list(range(1,addr_range+1))
    self.ignore_id_list = self.nex_dict['discovery_ignore_ids']
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
      valid_path = True
      for id in self.ignore_id_list:
        if path_str.find(id) != -1 or path_str in self.active_paths_list:
          valid_path = False
      if valid_path:
        #rospy.logwarn(self.node_name + ": Looking for path: " + path_str)
        #rospy.logwarn(self.node_name + ": In path_list: " + str(self.active_paths_list))
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
    if path_str not in self.active_paths_list:
      for baud_int in self.baudrate_list:
        self.baud_int = baud_int
        rospy.logdebug(self.node_name + ": Connecting to serial port " + path_str + " with baudrate: " + str(baud_int))
        try:
          # Try and open serial port
          rospy.logdebug(self.node_name + ": Opening serial port " + path_str + " with baudrate: " + str(baud_int))
          serial_port = serial.Serial(path_str,baud_int,timeout = 1)
        except Exception as e:
          rospy.logwarn(self.node_name + ": Unable to open serial port " + path_str + " with baudrate: " + str(baud_int) + "(" + str(e) + ")")
          continue
        for addr in self.addr_list:
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
            rospy.sleep(.005)
            bs = serial_port.readline()
            response = bs.decode()
          except Exception as e:
            #rospy.logerr('%s: Got a serial read/write error (%s)', self.node_name, str(e))
            break
          if len(response) > 2:
            rospy.loginfo(self.node_name + ": Got response: " + response)
            if response[3] == ',':
              self.addr_str = response[0:3]
              try:
                addr_int = int(addr)
                rospy.loginfo(self.node_name + ": Found device at address: " + self.addr_str)
                found_device = True
                break # Don't check any more addresses
              except Exception as a:
                rospy.logwarn(self.node_name + ": Returned device message not valid (" + str(a) + ")")
        # Clean up the serial port
        rospy.logdebug(self.node_name + ": Closing serial port " + path_str)
        serial_port.close()
    return found_device


  def checkOnDevice(self,path_str):
    active = path_str in self.path_list
    return active


  def launchDeviceNode(self, path_str):
    file_name = self.nex_dict['node_file_name']
    ros_node_name = self.node_name + "_" + path_str.split('/')[-1] + "_" + addr_str
    rospy.loginfo(self.log_name + ":  launching node: " + ros_node_name)
    #Setup required param server nex_dict for discovery node
    dict_param_name = self.base_namespace + ros_node_name + "/nex_dict"
    #rospy.logwarn(self.log_name + ":  launching node: " + str(self.nex_dict))
    self.nex_dict['device_path'] = path_str
    self.nex_dict['baud_int'] = self.baud_int
    self.nex_dict['addr_str'] = self.addr_str
    rospy.set_param(dict_param_name,self.nex_dict)
    [success, msg, sub_process] = nepi_nex.launchDriverNode(file_name, ros_node_name, device_path = path_str)
    if success:
      self.active_devices_dict[path_str] = {'ros_node_name': ros_node_name, 'sub_process': sub_process}
      self.active_paths_list.append(path_str)
