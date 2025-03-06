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
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_drv
from nepi_sdk import nepi_save

# Needed for GenICam auto-detect
from harvesters.core import Harvester


PKG_NAME = 'IDX_GENICAM' # Use in display menus
FILE_TYPE = 'DISCOVERY'

class GenicamCamDiscovery:

 
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
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################


    ########################
    # Get discovery options
    try:
      self.drv_dict = nepi_ros.get_param(self,'~drv_dict',dict())
      nepi_msg.publishMsgInfo(self,"Initial Driver Dict: " + str(self.drv_dict))
    except Exception as e:
      nepi_msg.publishMsgWarn(self, ":  " + self.log_name + ": Failed to load options " + str(e))#
      nepi_ros.signal_shutdown(self.node_name + ": Shutting down because failed to get Driver Dict")
      return
    ########################

    self.genicam_harvester = Harvester()
    
    self.genicam_harvester.add_file(self.DEFAULT_GENTL_PRODUCER_USB)    
    self.genicam_harvester.add_file(self.DEFAULT_GENTL_PRODUCER_GIGE)


    nepi_ros.start_timer_process(nepi_ros.duration(1), self.detectAndManageDevices, oneshot = True)
    
    nepi_ros.spin()

  #**********************
  # Discovery functions

  def detectAndManageDevices(self, timer):
    #nepi_msg.publishMsgWarn(self,"Starting detection process")
    # Make sure our genicam harvesters context is up to date.
    self.genicam_harvester.update()
    #nepi_msg.publishMsgInfo(self,str(genicam_harvester.device_info_list))
    # Take note of any genicam nodes currently running. If they are not found
    # in the current genicam harvesters context, we must assume that they have
    # been disconnected and stop the corresponding node(s).
    active_devices = {d["node_namespace"]: False for d in self.deviceList\
                                                 if d["device_class"] == "genicam"}

    # Iterate through each device in the current context.
       
    
    for device in self.genicam_harvester.device_info_list:
      #nepi_msg.publishMsgInfo(self,device)
      model = device.model
      sn = device.serial_number
      vendor = device.vendor
      device_is_known = False

      # Look to see if this device has already been launched as a node. If it
      # has, do nothing. If it hasn't, spin up a new node.
      for known_device in self.deviceList:
        if known_device["device_class"] != "genicam":
          continue
        try:
          # The call to communicate() will timeout if the node is still running.
          # If the node has exited, we log the corresponding stdout and stderr
          # and allow it to be restarted.
          stdo, stde = known_device["node_subprocess"].communicate(timeout=0.1)
          self.stopAndPurgeDeviceNode(known_device["node_namespace"])
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

    # Stop any nodes associated with devices that have disappeared.
    for node_namespace, running in active_devices.items():
      if not running:
        nepi_msg.publishMsgWarn(self,node_namespace + ' is no longer responding to discovery')
        self.stopAndPurgeDeviceNode(node_namespace)
    nepi_ros.sleep(self.CHECK_INTERVAL_S,100)
    nepi_ros.start_timer_process(nepi_ros.duration(1), self.detectAndManageDevices, oneshot = True)

  def startDeviceNode(self, vendor, model, serial_number):
    # TODO: fair to assume uniqueness of device serial numbers?
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
    nepi_msg.publishMsgWarn(self,"Initiating new Genicam node " + device_node_namespace)

    nepi_msg.publishMsgWarn(self,"Starting node " + device_node_name + " via rosrun")

    # NOTE: have to make serial_number look like a string by prefixing with "sn", otherwise ROS
    #       treats it as an int param and it causes an overflow. Better way to handle this?
    #Setup required param server drv_dict for discovery node
    self.drv_dict['DEVICE_DICT']={'model': model}
    self.drv_dict['DEVICE_DICT']['serial_number'] = serial_number
    dict_param_name = device_node_name + "/drv_dict"
    nepi_ros.set_param(self,dict_param_name,self.drv_dict)
    # Try and load save node params
    nepi_drv.checkLoadConfigFile(device_node_name)

    file_name = self.drv_dict['NODE_DICT']['file_name']
    #Try and launch node
    [success, msg, sub_process] = nepi_drv.launchDriverNode(file_name, device_node_name)
    if sub_process.poll() is not None:
      nepi_msg.publishMsgErr(self, 'Failed to start ' + device_node_name)
    else:
      self.deviceList.append({"device_class": "genicam",
                              "model": model,
                              "serial_number": serial_number,
                              "device_type": model,
                              "node_name": device_node_name,
                              "node_namespace": device_node_namespace,
                              "node_subprocess": sub_process})

  def stopAndPurgeDeviceNode(self, node_namespace):
    nepi_msg.publishMsgInfo(self,"stopping " + node_namespace)
    for i, device in enumerate(self.deviceList):
      if device['node_namespace'] == node_namespace:
        node_name = device['node_namespace'].split("/")[-1]
        sub_process = device['node_subprocess']
        success = nepi_drv.killDriverNode(node_name,sub_process)
        # And remove it from the list
        self.deviceList.pop(i)  
    if success == False:
      nepi_msg.publishMsgWarn(self,"Unable to stop unknown node " + node_namespace)

  def deviceNodeIsRunning(self, node_namespace):
    for device in self.deviceList:
      if device['node_namespace'] == node_namespace:
        if device['node_subprocess'].poll() is not None:
          return False
        else:
          return True
    # If we get here, didn't find the node in our list    
    nepi_msg.publishMsgWarn(self,"cannot check run status of unknown node " + node_namespace)
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

        
      

 
