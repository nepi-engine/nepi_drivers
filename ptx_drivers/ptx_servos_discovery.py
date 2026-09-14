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


import os
import subprocess
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'PTX_SERVOS' 
FILE_TYPE = 'DISCOVERY'

class ServosPanTiltDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []


  active_devices_dict = dict()
  node_launch_name = "servos_pan_tilt"
  node_launched = False
  node_process = None
  node_name = "servos_pan_tilt"

  dont_retry_list = []

  includeDevices = []
  excludedDevices = []


  source_path = 'None'
  ################################################          
  def __init__(self):
    ############
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_sdk.logger(log_name = self.log_name)
    time.sleep(1)
    self.logger.log_info("Starting Initialization")
    self.logger.log_info("Initialization Complete")



 
  ##########  DRV Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace, drv_dict, retry_enabled = True):
    self.drv_dict = drv_dict
    #self.logger.log_warn("Got drv_dict : " + str(self.drv_dict))
    #self.logger.log_warn("Got available paths list : " + str(available_paths_list))
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    
    ##################################
    # Get required data from drv_dict

    ###################################

    if self.node_launched == False:
      success = self.launchDeviceNode()
    return self.active_paths_list
  ################################################

  

  def launchDeviceNode(self):
    success = False
    
    node_launch_name = 'servos_pan_tilt_'
    self.logger.log_warn("Entering launch device function")###
    file_name = self.drv_dict['NODE_DICT']['file_name']
    device_name = self.node_launch_name
    node_name = nepi_system.get_device_alias(device_name)
    self.logger.log_warn(" launching node: " + node_name)


    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name)
    if success == True:
      self.node_name = node_name
      self.node_process = sub_process
      self.node_launched = True
    else:
      self.logger.log_warn("Node Launch Failed " + str(msg))###
    return success

  def killAllDevices(self,active_paths_list):
    if self.node_process is not None and self.node_name is not None:
        success = nepi_drvs.killDriverNode(self.node_name,self.node_process)
        self.node_name = None
        self.node_process = None
    return active_paths_list


if __name__ == '__main__':
    ServosPanTiltDiscovery()

    


        
      

 
