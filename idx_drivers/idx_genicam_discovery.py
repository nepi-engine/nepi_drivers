#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License", 
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
# ROS namespace setup
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import subprocess
import time

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_save

# Needed for GenICam auto-detect
from harvesters.core import Harvester

from nepi_api.sys_if_msg import MsgIF


PKG_NAME = 'IDX_GENICAM' # Use in display menus
FILE_TYPE = 'DISCOVERY'

class GenicamCamDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []
 
  includeDevices = []
  excludedDevices = []     

  settings_if = None

  NEPI_DEFAULT_CFG_PATH = '/opt/nepi/ros/etc/nepi_drivers'
  NEPI_DEFAULT_USER_CFG_PATH = 'mnt/nepi_storage/user_cfg/ros'

  CHECK_INTERVAL_S = 3.0


  DEFAULT_GENTL_PRODUCER_USB =  '/opt/baumer/gentl_producers/libbgapi2_usb.cti.2.14.1'
  DEFAULT_GENTL_PRODUCER_GIGE = '/opt/baumer/gentl_producers/libbgapi2_gige.cti.2.14.1'


   ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"    
  drv_dict = dict()
  deviceList = []          


  def __init__(self):
    ####  NODE Initialization ####
    self.class_name = type(self).__name__
    self.base_namespace = nepi_ros.get_base_namespace()
    self.node_name = nepi_ros.get_node_name()
    self.node_namespace = nepi_ros.get_node_namespace()

    ##############################  
    # Create Msg Class
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting Node Initialization Processes")


    ########################
    # Get discovery options
    try:
      self.drv_dict = nepi_ros.get_param(self,'~drv_dict',dict())
      self.logger.log_msg_info("Initial Driver Dict: " + str(self.drv_dict))
    except Exception as e:
      self.logger.log_msg_warn("Failed to load options " + str(e))#
      nepi_ros.signal_shutdown(self.node_name + ": Shutting down because failed to get Driver Dict")
      return
      
    if 'retry' in self.drv_dict['DISCOVERY_DICT']['OPTIONS'].keys():
      self.retry = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['retry']['value']
    else:
      self.retry = True
    ########################

    self.genicam_harvester = Harvester()
    
    self.genicam_harvester.add_file(self.DEFAULT_GENTL_PRODUCER_USB)    
    self.genicam_harvester.add_file(self.DEFAULT_GENTL_PRODUCER_GIGE)


    nepi_ros.start_timer_process(nepi_ros.ros_duration(1), self.detectAndManageDevices, oneshot = True)

    self.logger.log_msg_info("Initialization Complete")
    nepi_ros.spin()

  #**********************
  # Discovery functions

  def detectAndManageDevices(self, timer):
    #self.logger.log_msg_warn("Starting detection process")
    # Make sure our genicam harvesters context is up to date.
    self.genicam_harvester.update()
    #self.logger.log_msg_info("str(genicam_harvester.device_info_list))
    # Take note of any genicam nodes currently running. If they are not found
    # in the current genicam harvesters context, we must assume that they have
    # been disconnected and stop the corresponding node(s).
    active_devices = {d["node_namespace"]: False for d in self.deviceList\
                                                 if d["device_class"] == "genicam"}

    # Iterate through each device in the current context.
       
    
    for device in self.genicam_harvester.device_info_list:
      #self.logger.log_msg_info(device)
      model = device.model
      sn = device.serial_number
      vendor = device.vendor
      device_is_known = False

      # Look to see if this device has already been launched as a node. If it
      # has, do nothing. If it hasn't, spin up a new node.
      for known_device in self.deviceList:
        node_namespace = known_device["node_namespace"]

        if known_device["device_class"] != "genicam":
          continue
        try:
          # The call to communicate() will timeout if the node is still running.
          # If the node has exited, we log the corresponding stdout and stderr
          # and allow it to be restarted.
          stdo, stde = known_device["node_subprocess"].communicate(timeout=0.1)
          
          self.stopAndPurgeDeviceNode(node_namespace)

          ### DON'T REMOVE FROM dont_retry_list ###
          launch_id = node_namespace
          if launch_id in self.dont_retry_list:
            self.logger.log_msg_warn("node " + node_namespace + " is not running. WILL NOT RESTART")
          else:
            self.logger.log_msg_warn("node " + node_namespace + " is not running. RESTARTING")



          continue
        except subprocess.TimeoutExpired:
          pass
        device_is_known = (device_is_known or (known_device["model"] == device.model and\
                known_device["serial_number"] == device.serial_number))
        if device_is_known:
          active_devices[known_device["node_namespace"]] = True
          break
      if not device_is_known:
        self.startDeviceNode(vendor=vendor, model=model, serial_number=sn)

        # Remove from dont_retry_list
        #launch_id = node_namespace
        #if launch_id in self.dont_retry_list:
          #self.dont_retry_list.remove(launch_id) 

    # Stop any nodes associated with devices that have disappeared.
    for node_namespace, running in active_devices.items():
      if not running:
        self.logger.log_msg_info("Device no longer present. Stopping node " + node_namespace)
        self.stopAndPurgeDeviceNode(node_namespace)
          
        # Remove from dont_retry_list
        launch_id = node_namespace
        if launch_id in self.dont_retry_list:
          self.dont_retry_list.remove(launch_id) 

    nepi_ros.sleep(self.CHECK_INTERVAL_S,100)
    nepi_ros.start_timer_process(nepi_ros.ros_duration(1), self.detectAndManageDevices, oneshot = True)


  def startDeviceNode(self, vendor, model, serial_number):
    success = False 

    # Get Node Namespace
    vendor_ros = vendor.split()[0].replace('-', '_').lower() # Some vendors have really long strings, so just use the part to the first space
    model_ros = model.replace('-', '_').replace(' ', '_').lower()
    serial_number_ros = serial_number.replace('-', '_').replace(' ', '_').lower()
    # TODO: Validate that the resulting rootname is a legal ROS identifier
    root_name = f'{vendor_ros}_{model_ros}'
    root_name = self.short_name(root_name)
    unique_root_name = root_name + '_' + serial_number
    node_needs_serial_number = True
    for device in self.deviceList:
      if device['device_type'] == model:
        node_needs_serial_number = True
        break
    device_node_name = root_name if not node_needs_serial_number else unique_root_name
    device_node_namespace = nepi_ros.get_base_namespace() + device_node_name




    launch_id = device_node_namespace

    # Check if should try to launch
    launch_check = True
    if launch_id in self.launch_time_dict.keys():
      launch_time = self.launch_time_dict[launch_id]
      cur_time = nepi_ros.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LAUNCH_TIME_SEC
    if launch_check == False:
      return False  ###

    ### Start Node Luanch Process
    # TODO: fair to assume uniqueness of device serial numbers?
 

    self.logger.log_msg_warn("Initiating new Genicam node " + device_node_namespace)

    self.logger.log_msg_warn("Starting node " + device_node_name + " via rosrun")

    # NOTE: have to make serial_number look like a string by prefixing with "sn", otherwise ROS
    #       treats it as an int param and it causes an overflow. Better way to handle this?
    #Setup required param server drv_dict for discovery node
    self.drv_dict['DEVICE_DICT']={'model': model}
    self.drv_dict['DEVICE_DICT']['serial_number'] = serial_number
    dict_param_name = device_node_name + "/drv_dict"
    nepi_ros.set_param(self,dict_param_name,self.drv_dict)
    # Try and load save node params
    nepi_drvs.checkLoadConfigFile(device_node_name)

    file_name = self.drv_dict['NODE_DICT']['file_name']
    #Try and launch node
    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, device_node_name)
    time.sleep(1)
    if sub_process.poll() is not None:
      success = False
      msg = "Node " + device_node_name + " did not responde after launching"
    else:
      self.deviceList.append({"device_class": "genicam",
                              "model": model,
                              "serial_number": serial_number,
                              "device_type": model,
                              "node_name": device_node_name,
                              "node_namespace": device_node_namespace,
                              "node_subprocess": sub_process})

    # Process luanch results
    self.launch_time_dict[launch_id] = nepi_ros.get_time()
    if success:
      self.logger.log_msg_info(" Launched node: " + device_node_name)
    else:
      self.logger.log_msg_info(" Failed to lauch node: " + device_node_name + " with msg: " + msg)
      if self.retry == False:
        self.logger.log_msg_info(" Will not try relaunch for node: " + device_node_name)
        self.dont_retry_list.append(launch_id)
      else:
        self.logger.log_msg_info(" Will attemp relaunch for node: " + device_node_name + " in " + self.NODE_LAUNCH_TIME_SEC + " secs")
    return success

  def stopAndPurgeDeviceNode(self, node_namespace):
    self.logger.log_msg_info("stopping " + node_namespace)
    for i, device in enumerate(self.deviceList):
      if device['node_namespace'] == node_namespace:
        node_name = device['node_namespace'].split("/")[-1]
        sub_process = device['node_subprocess']
        success = nepi_drvs.killDriverNode(node_name,sub_process)
        # And remove it from the list
        self.deviceList.pop(i)  
    if success == False:
      self.logger.log_msg_warn("Unable to stop unknown node " + node_namespace)

  def deviceNodeIsRunning(self, node_namespace):
    for device in self.deviceList:
      if device['node_namespace'] == node_namespace:
        if device['node_subprocess'].poll() is not None:
          return False
        else:
          return True
    # If we get here, didn't find the node in our list    
    self.logger.log_msg_warn("cannot check run status of unknown node " + node_namespace)
    return False
  

  def short_name(self,name):
    split = name.split("_")
    if len(split) > 3:
      short_name = (split[0] + "_" + split[1] + "_" + split[2])
    else:
      short_name = name
    return short_name
    
if __name__ == '__main__':
  node = GenicamCamDiscovery()            

        
      

 
