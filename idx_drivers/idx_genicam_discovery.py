#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


import os
# ROS namespace setup
#NEPI_BASE_NAMESPACE = '/nepi/device1/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import subprocess
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system


# Needed for GenICam auto-detect
from harvesters.core import Harvester

from nepi_api.messages_if import MsgIF


PKG_NAME = 'IDX_GENICAM' # Use in display menus
FILE_TYPE = 'DISCOVERY'

class GenicamCamDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []
  test_val = 2
   
  includeDevices = []
  excludedDevices = []     

  settings_if = None

  CHECK_INTERVAL_S = 3.0


  DEFAULT_GENTL_PRODUCER_USB =  '/opt/baumer/gentl_producers/libbgapi2_usb.cti.2.14.1'
  DEFAULT_GENTL_PRODUCER_GIGE = '/opt/baumer/gentl_producers/libbgapi2_gige.cti.2.14.1'


   ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"    
  drv_dict = dict()
  deviceList = []          


  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################  
    # Create Msg Class
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting Node Initialization Processes")


    ########################
    # Get discovery options
    try:
      self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
      self.msg_if.pub_info("Initial Driver Dict: " + str(self.drv_dict))
    except Exception as e:
      self.msg_if.pub_warn("Failed to load options " + str(e))#
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because failed to get Driver Dict")
      return

    if 'retry' in self.drv_dict['DISCOVERY_DICT']['OPTIONS'].keys():
      self.retry = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['retry']['value']
    else:
      self.retry = True
    ########################

    self.genicam_harvester = Harvester()
    
    self.genicam_harvester.add_file(self.DEFAULT_GENTL_PRODUCER_USB)    
    self.genicam_harvester.add_file(self.DEFAULT_GENTL_PRODUCER_GIGE)


    nepi_sdk.start_timer_process((1), self.detectAndManageDevices, oneshot = True)

    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.spin()

  #**********************
  # Discovery functions

  def detectAndManageDevices(self, timer):
    #self.msg_if.pub_warn("Starting detection process")
    # Make sure our genicam harvesters context is up to date.
    self.genicam_harvester.update()
    #self.msg_if.pub_info("str(genicam_harvester.device_info_list))
    # Take note of any genicam nodes currently running. If they are not found
    # in the current genicam harvesters context, we must assume that they have
    # been disconnected and stop the corresponding node(s).
    active_devices = {d["node_namespace"]: False for d in self.deviceList\
                                                 if d["device_class"] == "genicam"}

    # Iterate through each device in the current context.
       
    
    for device in self.genicam_harvester.device_info_list:
      #self.msg_if.pub_info(device)
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
            self.msg_if.pub_warn("node " + node_namespace + " is not running. WILL NOT RESTART")
          else:
            self.msg_if.pub_warn("node " + node_namespace + " is not running. RESTARTING")



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
        self.msg_if.pub_info("Device no longer present. Stopping node " + node_namespace)
        self.stopAndPurgeDeviceNode(node_namespace)
          
        # Remove from dont_retry_list
        launch_id = node_namespace
        if launch_id in self.dont_retry_list:
          self.dont_retry_list.remove(launch_id) 

    nepi_sdk.sleep(self.CHECK_INTERVAL_S,100)
    nepi_sdk.start_timer_process((1), self.detectAndManageDevices, oneshot = True)


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

    node_name = nepi_system.get_node_name(device_node_name)
    self.logger.log_warn(" launching node: " + node_name)
    node_namespace = os.path.join(self.base_namespace, node_name)




    launch_id = node_namespace

    # Check if should try to launch
    launch_check = True
    if launch_id in self.launch_time_dict.keys():
      launch_time = self.launch_time_dict[launch_id]
      cur_time = nepi_sdk.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LAUNCH_TIME_SEC
    if launch_check == False:
      return False  ###

    ### Start Node Luanch Process
    # TODO: fair to assume uniqueness of device serial numbers?
 

    self.msg_if.pub_warn("Initiating new Genicam node " + node_namespace)

    self.msg_if.pub_warn("Starting node " + node_name + " via rosrun")

    # NOTE: have to make serial_number look like a string by prefixing with "sn", otherwise ROS
    #       treats it as an int param and it causes an overflow. Better way to handle this?

    # Try and load saved node params if file exists
    nepi_sdk.load_node_config(device_node_name, node_name)
    
    #Setup required param server drv_dict for discovery node
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace,node_name + "/drv_dict")
    self.drv_dict['DEVICE_DICT']={'model': model}
    self.drv_dict['DEVICE_DICT']['serial_number'] = serial_number
    nepi_sdk.set_param(dict_param_name,self.drv_dict)


    file_name = self.drv_dict['NODE_DICT']['file_name']
    #Try and launch node
    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name)
    time.sleep(1)
    if sub_process.poll() is not None:
      success = False
      msg = "Node " + node_name + " did not responde after launching"
    else:
      self.deviceList.append({"device_class": "genicam",
                              "model": model,
                              "serial_number": serial_number,
                              "device_type": model,
                              "node_name": node_name,
                              "node_namespace": node_namespace,
                              "node_subprocess": sub_process})

    # Process luanch results
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    if success:
      self.msg_if.pub_info(" Launched node: " + node_name)
    else:
      self.msg_if.pub_info(" Failed to lauch node: " + node_name + " with msg: " + msg)
      if self.retry == False:
        self.msg_if.pub_info(" Will not try relaunch for node: " + node_name)
        self.dont_retry_list.append(launch_id)
      else:
        self.msg_if.pub_info(" Will attemp relaunch for node: " + node_name + " in " + self.NODE_LAUNCH_TIME_SEC + " secs")
    return success

  def stopAndPurgeDeviceNode(self, node_namespace):
    self.msg_if.pub_info("stopping " + node_namespace)
    for i, device in enumerate(self.deviceList):
      if device['node_namespace'] == node_namespace:
        node_name = device['node_namespace'].split("/")[-1]
        sub_process = device['node_subprocess']
        success = nepi_drvs.killDriverNode(node_name,sub_process)
        # And remove it from the list
        self.deviceList.pop(i)  
    if success == False:
      self.msg_if.pub_warn("Unable to stop unknown node " + node_namespace)

  def deviceNodeIsRunning(self, node_namespace):
    for device in self.deviceList:
      if device['node_namespace'] == node_namespace:
        if device['node_subprocess'].poll() is not None:
          return False
        else:
          return True
    # If we get here, didn't find the node in our list    
    self.msg_if.pub_warn("cannot check run status of unknown node " + node_namespace)
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

        
      

 
