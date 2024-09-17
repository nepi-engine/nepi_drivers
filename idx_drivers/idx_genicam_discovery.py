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
# ROS namespace setup
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import subprocess
import time
import rospy

from nepi_edge_sdk_base import nepi_nex

# Needed for GenICam auto-detect
from harvesters.core import Harvester


PKG_NAME = 'IDX_GENICAM' # Use in display menus
FILE_TYPE = 'DISCOVERY'
CLASS_NAME = 'GenicamCamDiscovery'
PROCESS = 'LAUNCH' # 'LAUNCH', 'RUN', or 'CALL'

TEST_NEX_DICT = {
  'group': 'IDX',
  'group_id': 'GENICAM',
  'node_file_name': 'idx_genicam_node.py',
  'node_file_path': '/opt/nepi/ros/lib/nepi_drivers',
  'node_module_name': 'idx_genicam_node',
  'node_class_name': 'GenicamCamNode',
  'driver_name': 'IDX_GENICAM',
  'driver_file_name': 'idx_genicam_driver.py' ,
  'driver_file_path':  '/opt/nepi/ros/lib/nepi_drivers',
  'driver_module_name': 'idx_genicam_driver' ,
  'driver_class_name':  'GenicamCamDriver',
  'driver_interfaces': ['USB'],
  'driver_options_1': [],
  'driver_default_option_1': 'None',
  'driver_set_option_1': 'None',
  'driver_options_2': [],
  'driver_default_option_2': 'None',
  'driver_set_option_2': 'None',
  'discovery_name': 'IDX_GENICAM', 
  'discovery_file_name': 'idx_genicam_discovery.py',
  'discovery_file_path': '/opt/nepi/ros/lib/nepi_drivers',
  'discovery_module_name': 'idx_genicam_discovery',
  'discovery_class_name': 'GenicamCamDiscovery',
  'discovery_method': 'AUTO', 
  'discovery_ids': [],
  'discovery_ignore_ids': [],
  'device_dict': {},
  'order': 1,
  'active': True,
  'msg': ""
  }

class GenicamCamDiscovery:

  NEPI_DEFAULT_CFG_PATH = '/opt/nepi/ros/etc/'
  CHECK_INTERVAL_S = 3.0

  DEFAULT_EXCLUDED_DEVICES = []  # None at present

  DEFAULT_GENTL_PRODUCER_USB =  '/opt/baumer/gentl_producers/libbgapi2_usb.cti.2.14.1'
  DEFAULT_GENTL_PRODUCER_GIGE = '/opt/baumer/gentl_producers/libbgapi2_gige.cti.2.14.1'


   ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"    
  nex_dict = dict()
                               
  def __init__(self):
    # Launch the ROS node
    rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
    self.node_name = rospy.get_name().split('/')[-1]
    # Get nex_dict from param servers
    rospy.loginfo("Starting " + self.node_name)
    self.nex_dict = rospy.get_param('~nex_dict',TEST_NEX_DICT) 
    ################################################
    self.deviceList = []
    self.includeDevices = rospy.get_param('~included_devices', self.nex_dict['discovery_ids'])
    self.excludedDevices = rospy.get_param('~excluded_devices', self.nex_dict['discovery_ignore_ids'])

    self.genicam_harvester = Harvester()
    
    self.genicam_harvester.add_file(self.DEFAULT_GENTL_PRODUCER_USB)    
    self.genicam_harvester.add_file(self.DEFAULT_GENTL_PRODUCER_GIGE)


    rospy.Timer(rospy.Duration(self.CHECK_INTERVAL_S), self.detectAndManageDevices)
    
    rospy.spin()

  def detectAndManageDevices(self, timer):
    # Make sure our genicam harvesters context is up to date.
    self.genicam_harvester.update()
    #rospy.loginfo(self.genicam_harvester.device_info_list)
    # Take note of any genicam nodes currently running. If they are not found
    # in the current genicam harvesters context, we must assume that they have
    # been disconnected and stop the corresponding node(s).
    active_devices = {d["node_namespace"]: False for d in self.deviceList\
                                                 if d["device_class"] == "genicam"}

    # Iterate through each device in the current context.
       
    
    for device in self.genicam_harvester.device_info_list:
      #rospy.loginfo(device)
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
          rospy.logerr(f'{known_device["node_name"]} exited')
          rospy.logerr(f"stdout: {stdo}")
          rospy.logerr(f"stderr: {stde}")
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
        rospy.logwarn(f'{node_namespace} is no longer responding to discovery')
        self.stopAndPurgeDeviceNode(node_namespace)

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
    device_node_namespace = rospy.get_namespace() + device_node_name
    rospy.loginfo(f"{self.node_name}: Initiating new Genicam node {device_node_namespace}")

    self.checkLoadConfigFile(node_name=device_node_name)

    rospy.loginfo(f"{self.node_name}: Starting {device_node_name} via rosrun")

    # NOTE: have to make serial_number look like a string by prefixing with "sn", otherwise ROS
    #       treats it as an int param and it causes an overflow. Better way to handle this?
    #Setup required param server nex_dict for discovery node
    self.nex_dict['device_dict']={'model': model}
    self.nex_dict['device_dict']['serial_number'] = serial_number
    dict_param_name = device_node_name + "/nex_dict"
    rospy.set_param(dict_param_name,self.nex_dict)
    file_name = self.nex_dict['node_file_name']
    #Try and launch node
    [success, msg, sub_process] = nepi_nex.launchDriverNode(file_name, device_node_name)
    if sub_process.poll() is not None:
      rospy.logerr(f'Failed to start {device_node_name} via {" ".join(x for x in device_node_run_cmd)} (rc = {sub_process.returncode})')
    else:
      self.deviceList.append({"device_class": "genicam",
                              "model": model,
                              "serial_number": serial_number,
                              "device_type": model,
                              "node_name": device_node_name,
                              "node_namespace": device_node_namespace,
                              "node_subprocess": sub_process})

  def stopAndPurgeDeviceNode(self, node_namespace):
    rospy.loginfo(self.node_name + ": stopping " + node_namespace)
    for i, device in enumerate(self.deviceList):
      if device['node_namespace'] == node_namespace:
        node_name = device['node_namespace'].split("/")[-1]
        sub_process = device['node_subprocess']
        success = nepi_nex.killDriverNode(node_name,sub_process)
        # And remove it from the list
        self.deviceList.pop(i)  
    if success == False:
      rospy.logwarn(self.node_name + ": Unable to stop unknown node " + node_namespace)

  def deviceNodeIsRunning(self, node_namespace):
    for device in self.deviceList:
      if device['node_namespace'] == node_namespace:
        if device['node_subprocess'].poll() is not None:
          return False
        else:
          return True
    # If we get here, didn't find the node in our list    
    rospy.logwarn(self.node_name + ": cannot check run status of unknown node " + node_namespace)
    return False
  
  def checkLoadConfigFile(self, node_name):
    folder_name = "drivers/" + node_name 
    config_folder = os.path.join(self.NEPI_DEFAULT_CFG_PATH, folder_name)
    if not os.path.isdir(config_folder):
      rospy.logwarn(self.node_name + ': No config folder found for %s... creating one at %s', node_name, config_folder)
      os.makedirs(name = config_folder, mode = 0o775)
      return
    
    config_file = os.path.join(config_folder, node_name + ".yaml")
    node_namespace = rospy.get_namespace() + node_name
    if os.path.exists(config_file):
      rospy.loginfo(self.node_name + ": Loading parameters from " + config_file + " to " + node_namespace)
      #rosparam.load_file(filename = config_file, default_namespace = node_namespace)
      #rosparam.load_file(filename = config_file, default_namespace = node_name)
      # Seems programmatic rosparam.load_file is not working at all, so use the command-line version instead
      rosparam_load_cmd = ['rosparam', 'load', config_file, node_namespace]
      subprocess.run(rosparam_load_cmd)
    else:
      rospy.logwarn(self.node_name + ": No config file found for " + node_name + " in " + self.NEPI_DEFAULT_CFG_PATH)

  def short_name(self,name):
    split = name.split("_")
    if len(split) > 3:
      short_name = (split[0] + "_" + split[1] + "_" + split[2])
    else:
      short_name = name
    return short_name
    
if __name__ == '__main__':
  node = GenicamCamDiscovery()            

        
      

 
