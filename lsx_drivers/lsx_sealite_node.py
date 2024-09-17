#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# NEPI LSX Driver Script for SeaLite LED Lights


### Set the namespace before importing rospy
import os
#os.environ["ROS_NAMESPACE"] = "/nepi/s2x"
import rospy
import serial
import serial.tools.list_ports
import time
import re
import sys

from nepi_drivers.lsx_device_if import ROSLSXDeviceIF

from nepi_ros_interfaces.msg import LSXStatus

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_nex

PKG_NAME = 'LSX_SEALITE' # Use in display menus
DESCRIPTION = 'Driver package for sealite light devices'
FILE_TYPE = 'NODE'
CLASS_NAME = 'SealiteNode' # Should Match Class Name
GROUP ='LSX'
GROUP_ID = 'SEALITE' 

DRIVER_PKG_NAME = 'None' # 'Required Driver PKG_NAME or 'None'
DRIVER_INTERFACES = ['SERIALUSB'] # 'USB','IP','SERIALUSB','SERIAL','CANBUS'
DRIVER_OPTIONS_1_NAME = 'BaudRate'
DRIVER_OPTIONS_1 = ['All','9600','19200','57600'] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_1 = '57600'
DRIVER_OPTIONS_2_NAME = 'Address Search Range'
DRIVER_OPTIONS_2 = ['1','10','100','255'] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_2 = '10'

DISCOVERY_PKG_NAME = 'LSX_SEALITE' # 'Required Discovery PKG_NAME or 'None'
DISCOVERY_METHOD = 'AUTO'  # 'AUTO', 'MANUAL', or 'OTHER' if managed by seperate application
DISCOVERY_IDS = []  # List of string identifiers for discovery process
DISCOVERY_IGNORE_IDS = ['ttyACM'] # List of string identifiers for discovery process


TEST_NEX_DICT = {
    'group': 'LXS',
    'group_id': 'SEALITE',
    'node_file_name': 'lsx_aftowerlight_node.py',
    'node_file_path': '/opt/nepi/ros/lib/nep_drivers',
    'node_module_name': 'lsx_aftowerlight_node',
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
    'discovery_name': 'None', 
    'discovery_file_name': "None",
    'discovery_file_path': "None",
    'discovery_module_name': "None",
    'discovery_class_name': "None",
    'discovery_method': 'Manual', 
    'discovery_ids': [],
    'discovery_ignore_ids': ['ttyACM'],
    'device_dict': {'device_path': '/dev/ttyUSB0', 'addr': '001'},
    'order': 1,
    'active': True,
    'msg': ""
    }


#########################################
# Sealite LSX Driver Node Class
#########################################

class SealiteNode(object):
  ### LXS Driver Settings
  # Set driver capability parameters



  #######################
  DEFAULT_NODE_NAME='sealite'

  FACTORY_SETTINGS_OVERRIDES = dict( )

  #Factory Control Values 
  FACTORY_CONTROLS = dict( standby_enabled = False,
  on_off_state = False,
  intensity_ratio = 0.0,
  strobe_enabled = False,
  blink_interval_sec = 2,
  blink_enabled = False
  )

  device_info_dict = dict(node_name = "",
                        device_name = "",
                        identifier = "",
                        serial_number = "",
                        hw_version = "",
                        sw_version = "")
  
  # Initialize some parameters
  serial_num = ""
  hw_version = ""
  sw_version = ""

  serial_port = None
  serial_busy = False
  connected = False

  on_off_state = False
  standby_state = False
  intensity_ratio = 0.0
  strobe_state = False
  
  temp_c = 0.0

  self_check_count = 5
  self_check_counter = 0 # track sequencial message failures

  lsx_if = None

  ### LXS Driver NODE Initialization
  def __init__(self):
    # Launch the ROS node
    rospy.loginfo("Starting " + self.DEFAULT_NODE_NAME)
    rospy.init_node(self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
    self.node_name = rospy.get_name().split('/')[-1]
    # Get nex_dict from param servers
    self.nex_dict = rospy.get_param('~nex_dict',TEST_NEX_DICT) 
    #rospy.logwarn(self.node_name + ": ZED_NODE: " + str(self.nex_dict))
    self.ser_port_str = self.nex_dict['device_path'] 
    self.ser_buad_int = self.nex_dict['baud_int'] 
    self.addr_str = self.nex_dict['addr_str'] 
    # Address string must be three char long
    zero_prefix_len = 3-len(self.addr_str)
    for z in range(zero_prefix_len):
      self.addr_str = ('0' + self.addr_str)  
    ################################################  
    rospy.loginfo(self.node_name + ": Connecting to Device on port %s with buad %s",self.ser_port_str,self.ser_buad_int)
    ### Try and connect to device
    self.connected = self.connect()
    if self.connected:
      # Create LSX ROS node
      rospy.loginfo(self.node_name + ': Connected')
      # Initialize settings
      self.cap_settings = self.getCapSettings()
      #rospy.loginfo("CAPS SETTINGS")
      #for setting in self.cap_settings:
          #rospy.loginfo(setting)
      self.factory_settings = self.getFactorySettings()
      #rospy.loginfo("FACTORY SETTINGS")
      #for setting in self.factory_settings:
          #rospy.loginfo(setting)

      # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
      rospy.loginfo(self.node_name + ": Launching NEPI LSX (ROS) interface...")
      self.device_info_dict["node_name"] = self.node_name
      if self.node_name.find("_") != -1:
          split_name = self.node_name.rsplit('_', 1)
          self.device_info_dict["device_name"] = split_name[0]
          self.device_info_dict["identifier"] = split_name[1]
      else:
          self.device_info_dict["device_name"] = self.node_name
          self.device_info_dict["identifier"] = ""
      self.device_info_dict["serial_number"] = self.serial_num
      self.device_info_dict["hw_version"] = self.hw_version
      self.device_info_dict["sw_version"] = self.sw_version

      self.lsx_if = ROSLSXDeviceIF(
                  device_info = self.device_info_dict, 
                  getStatusFunction = self.getStatus,
                  capSettings = self.cap_settings,
                  factorySettings = self.factory_settings,
                  settingUpdateFunction=self.settingUpdateFunction,
                  getSettingsFunction=self.getSettings,
                  factoryControls = self.FACTORY_CONTROLS,
                  standbyEnableFunction = self.setStandby,
                  turnOnOffFunction = self.turnOnOff,
                  setIntensityRatioFunction = self.setIntensityRatio, 
                  blinkOnOffFunction = self.blinkOnOff,
                  enableStrobeFunction = self.setStrobeEnable,
                  reports_temp = True, 
                  reports_power = False
                 )
    
      # Start an sealite activity check process that kills node after some number of failed comms attempts
      rospy.loginfo("Starting an activity check process")
      rospy.Timer(rospy.Duration(0.2), self.check_timer_callback)
      # Initialization Complete
      lsx_if.publishMsg(" Initialization Complete")
      #Set up node shutdown
      rospy.on_shutdown(self.cleanup_actions)
      # Spin forever (until object is detected)
      rospy.spin()
    else:
      lsx_if.publishMsg(" Shutting down node")
      lsx_if.publishMsg(" Specified serial port not available")
      rospy.signal_shutdown("Serial port not available")   


  #**********************
  # Sensor setting functions

  def getSettingsOptionsDict(self):
    settings_options_dict = dict()
    return settings_options_dict


  def getCapSettings(self):
      settings = []
      settings_options_dict = self.getSettingsOptionsDict()
      if len(settings_options_dict.keys()) == 0:
        return nepi_ros.NONE_SETTINGS
      for setting_name in settings_options_dict.keys():
          info = settings_options_dict[setting_name]
          setting_type = info['type']
          setting = [setting_type,setting_name]
          if setting_type == 'Float' or setting_type == 'Int':
              setting_min = str(info['min'])
              setting_max = str(info['max'])
              setting.append(setting_min)
              setting.append(setting_max)
          elif setting_type == 'Discrete':
              options = info['options']
              for option in options:
                  setting.append(option)
          elif setting_type == 'Menu':
              legend = info['legend']
              for option_name in legend.keys():
                  option_ind = legend[option_name]
                  setting.append(option_name + ":" + str(option_ind))
          settings.append(setting)
      return settings

  def getFactorySettings(self):
      settings = []
      settings_options_dict = self.getSettingsOptionsDict()
      if len(settings_options_dict.keys()) == 0:
        return nepi_ros.NONE_SETTINGS
      for setting_name in settings_options_dict.keys():
          info = settings_options_dict[setting_name]
          setting_type = info['type']
          # Create Menu Setting
          if setting_type == 'Menu':
              setting_default = 'None'
              setting_value = info['default']
              legend = info['legend']
              for menu_name in legend.keys():
                  menu_value = legend[menu_name]
                  if menu_value == setting_value:
                      setting_default = (menu_name + ":" + str(setting_value))
          else:
              setting_default = str(info['default'])
          setting = [setting_type,setting_name,setting_default]
          settings.append(setting)
      #Apply factory setting overides
      for setting in settings:
          if setting[1] in self.FACTORY_SETTINGS_OVERRIDES:
              setting[2] = self.FACTORY_SETTINGS_OVERRIDES[setting[1]]
              settings = nepi_ros.update_setting_in_settings(setting,settings)
      return settings


  def getSettingsDict(self):
    settings_dict = dict()
    return settings_dict
          

  def getSettings(self):
      settings = []
      settings_dict = self.getSettingsDict()
      if len(settings_dict.keys()) == 0:
        return nepi_ros.NONE_SETTINGS
      #for key in settings_dict.keys():
          #string = str(settings_dict[key])
          #rospy.loginfo(key + " " + string)
      for setting_name in settings_dict.keys():
          info = settings_dict[setting_name]
          setting_type = info['type']
          # Create Current Setting
          if setting_type == 'Menu':
              setting_current = 'None'
              setting_value = info['value']
              legend = info['legend']
              for menu_name in legend.keys():
                  menu_value = legend[menu_name]
                  if menu_value == setting_value:
                      setting_current = (menu_name + ":" + str(setting_value))
          else:
              setting_current = str(info['value'])
          setting = [setting_type,setting_name,setting_current]
          settings.append(setting)
      return settings

  def setSetting(self,setting_name,setting_data):
    return True


  def settingUpdateFunction(self,setting):
      success = False
      setting_str = str(setting)
      if len(setting) == 3:
          setting_type = setting[0]
          setting_name = setting[1]
          [s_name, s_type, data] = nepi_ros.get_data_from_setting(setting)
          if data is not None:
              setting_data = data
              found_setting = False
              for cap_setting in self.cap_settings:
                  if setting_name in cap_setting:
                      found_setting = True
                      success, msg = self.setSetting(setting_name,setting_data)
                      if success:
                          msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)
              if found_setting is False:
                  msg = (self.node_name  + " Setting name" + setting_str + " is not supported")                 
          else:
              msg = (self.node_name  + " Setting data" + setting_str + " is None")
      else:
          msg = (self.node_name  + " Setting " + setting_str + " not correct length")
      return success, msg



  #######################
  ### LSX IF Get Status Function

  ### Status callback
  def getStatus(self):
    # update status values from device
    success=self.update_status_values()
    # Create LSX status message
    status_msg=LSXStatus()
    status_msg.device_name = self.device_info_dict["device_name"]
    status_msg.user_name = self.device_info_dict["device_name"]
    status_msg.identifier = self.device_info_dict["identifier"]
    status_msg.serial_num = self.device_info_dict["serial_number"]
    status_msg.hw_version = self.device_info_dict["hw_version"]
    status_msg.sw_version = self.device_info_dict["sw_version"]
    status_msg.on_off_state = self.on_off_state
    status_msg.standby_state = self.standby_state
    status_msg.intensity_ratio = self.intensity_ratio
    status_msg.strobe_state = self.strobe_state
    status_msg.temp_c = self.temp_c
    status_msg.power_w = 0
    return(status_msg)


  def update_status_values(self):
    success = True
    # Update standby status
    #lsx_if.publishMsg(" Updating standby status")
    ser_msg= ('!' + self.addr_str + ':STBY?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      if response == "0":
        self.standby_state = False
      elif response == "1":
        self.standby_state = True
      else:
        success = False
      #rospy.loginfo(self.node_name + ": Standby: " + str(self.standby_state))
    else:
      success = False
    # Update intensity_ratio status
    #rospy.loginfo(self.node_name + ": Updating intensity_ratio status")
    ser_msg= ('!' + self.addr_str + ':LOUT?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      try:
        self.intensity_ratio = float(response)/100
        #rospy.loginfo(self.node_name + ": Intensity Ratio: " + str(self.intensity_ratio))
      except Exception as i:
        self.intensity_ratio = -999
        rospy.logwarn(self.node_name + ": Level response was not valid number")
        success = False
      #rospy.loginfo(self.node_name + ": Intensity: " + str(self.intensity_ratio))
    else:
      self.intensity_ratio = -999
      success = False
    # Update strobe enable status
    #rospy.loginfo(self.node_name + ": Updating strobe enable status")
    ser_msg= ('!' + self.addr_str + ':PMOD?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      if response == "0":
        self.strobe_state = False
      elif response == "1" or response == "2":
       self.strobe_state = True
      else:
        success = False
      #rospy.loginfo(self.node_name + ": Strobe Enable: " + str(self.strobe_state))
    else:
      success = False
    # Update temp status
    #rospy.loginfo(self.node_name + ": Updating temp status")
    ser_msg= ('!' + self.addr_str + ':TEMP?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      try:
        self.temp_c = int(float(response))
        #rospy.loginfo(self.node_name + ": Temp Deg C: " + str(self.temp_c))
      except Exception as t:
        self.temp_c = 255
        rospy.logwarn(self.node_name + ": Temp response was not valid number")
        success = False
      #rospy.loginfo(self.node_name + ": Temp C: " + str(self.temp_c))
    else:
      self.temp_c = 255
      success = False
    return success

  #######################
  ### LSX IF Interface Functions
  def setStandby(self,standby_val):
    success = False
    if standby_val == True:
      ser_msg= ('!' + self.addr_str + ':STBY=1')
    else:
      ser_msg= ('!' + self.addr_str + ':STBY=0')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      success = True
    return success 

  def turnOnOff(self,turn_on_off):
    if turn_on_off == False:
      self.set_intensity(0)
    else:
      self.set_intensity(self.intensity_ratio)
    self.turn_on_off_state = turn_on_off

  def blinkOnOff(self,blink_on_off):
    if self.blink_on_off_state == True:
      if blink_on_off == False:
        self.set_intensity(0)
      else:
        self.set_intensity(self.intensity_ratio)
    else:
      self.set_intensity(0)

  def setIntensityRatio(self,intensity_ratio):
    success = False
    if intensity_ratio < 0:
      intensity_ratio = 0
    elif intensity_ratio > 1:
      intensity_ratio = 1
    success = self.setIntensityFunction(intensity_ratio)
    if success:
      self.intensity_ratio = intensity_ratio

  def setIntensityFunction(self,intensity_ratio):
    level_val = int(100*intensity_ratio)
    level_str = str(level_val)
    zero_prefix_len = 3-len(level_str)
    for z in range(zero_prefix_len):
      level_str = ('0' + level_str)
    ser_msg= ('!' + self.addr_str + ':LOUT=' +  level_str)
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      success = True
    return success 

  def setStrobeEnable(self,strobe_enable_val):
    success = False
    if strobe_enable_val == True:
      ser_msg= ('!' + self.addr_str + ':PMOD=1')
    else:
      ser_msg= ('!' + self.addr_str + ':PMOD=0')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      success = True
    return success  


  #######################
  ### Class Functions

  def check_timer_callback(self,timer):
    success = False
    ser_msg= ('!' + self.addr_str + ':INFO?')
    ser_str = (ser_msg + '\r\n')
    b=bytearray()
    b.extend(map(ord, ser_str))
    try:
      while self.serial_busy == True and not rospy.is_shutdown():
        time.sleep(0.01) # Wait for serial port to be available
      self.serial_busy = True
      self.serial_port.write(b)
    except Exception as e:
      rospy.logwarn(self.node_name + ": Failed to send message")
    time.sleep(.01)
    try:
      bs = self.serial_port.readline()
    except Exception as e:
      rospy.logwarn(self.node_name + ": Failed to receive message")
    self.serial_busy = False
    response = bs.decode()
    # Check for valid response 
    if response != None and response != "?" and len(response)>4:
      ret_addr = response[0:3]
      if ret_addr == self.addr_str:
        success = True
    # Update results and take actions
    if success:
      self.serial_busy = False # Clear the busy indicator
      self.self_check_counter = 0 # reset comms failure count
    else:
      self.serial_busy = True # Lock port until valid response
      self.self_check_counter = self.self_check_counter + 1 # increment counter
    #print("Current failed comms count: " + str(self.self_check_counter))
    if self.self_check_counter > self.self_check_count:  # Crashes node if set above limit??
      rospy.logwarn(self.node_name + ": Shutting down device: " +  self.addr_str + " on port " + self.ser_port_str)
      rospy.logwarn(self.node_name + ": Too many comm failures")
      rospy.signal_shutdown("To many comm failures")   
   
  ### Function to try and connect to device at given port and buadrate
  def connect(self):
    success = False
    port_check = self.check_port(self.ser_port_str)
    if port_check is True:
      try:
        # Try and open serial port
        rospy.loginfo(self.node_name + ": Opening serial port " + self.ser_port_str + " with buadrate: " + str(self.ser_buad_int))
        self.serial_port = serial.Serial(self.ser_port_str,self.ser_buad_int,timeout = 0.1)
        rospy.loginfo(self.node_name + ": Serial port opened")
        # Send Message
        rospy.loginfo(self.node_name + ": Requesting info for device: " + self.addr_str)
        ser_msg = ('!' + self.addr_str + ':INFO?')
        #rospy.loginfo(self.node_name + ": Sending serial string: " + ser_msg)
        response = self.send_msg(ser_msg)
        #rospy.loginfo("Got response message: " + response)
        if len(response) > 0:
          if response != None and response != "?" and response[3] == ",":
            if len(response) > 4:
              ret_addr = response[0:3]
              #rospy.loginfo(self.node_name + ": Returned address value: " + ret_addr)
              if ret_addr == self.addr_str:
                rospy.loginfo(self.node_name + ": Connected to device at address: " +  self.addr_str)
                res_split = response.split(',')
                if len(res_split) > 5:
                # Update serial, hardware, and software status values
                  self.serial_num = res_split[2]
                  self.hw_version = res_split[3]
                  self.sw_version = res_split[4]
                success = True
              else:
                rospy.logwarn(self.node_name + ": Device returned address: " + ret_addr + " does not match: " +  self.addr_str)
            else:
              rospy.logwarn(self.node_name + ": Device returned invalid response")
          else:
            rospy.logwarn(self.node_name + ": Device returned empty response")
        else:
          rospy.logwarn(self.node_name + ": Device returned invalid response")
      except Exception as e:
        rospy.logwarn(self.node_name + ": Something went wrong with connect function at serial port at: " + self.ser_port_str + "(" + str(e) + ")" )
    else:
      rospy.logwarn(self.node_name + ": serial port not active")
    return success


  def send_msg(self,ser_msg):
    response = None
    if self.serial_port is not None and not rospy.is_shutdown():
      ser_str = (ser_msg + '\r\n')
      b=bytearray()
      b.extend(map(ord, ser_str))
      
      sleep_time = .1
      timeout = 2
      timer = 0
      while self.serial_busy == True and timer < timeout and not rospy.is_shutdown():
          time.sleep(sleep_time ) # Wait for serial port to be available
          timer += sleep_time 
      if timer < timeout:
        self.serial_busy = True
        #print("Sending " + ser_msg + " message")
        try:
          self.serial_port.write(b)
          time.sleep(.01)
          try:
            bs = self.serial_port.readline()
            self.serial_busy = False
            response = bs.decode()
            #print("Send response received: " + response[0:-2])
          except Exception as e1:
            print("Failed to recieve message")
        except Exception as e2:
          print("Failed to send message")
      else:
        print("Serial port write timed out on busy state")
    else:
      print("serial port not defined, returning empty string")
    return response

  ### Function for checking if port is available
  def check_port(self,port_str):
    success = False
    ports = serial.tools.list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      if loc == port_str:
        success = True
    return success

  #######################
  ### Cleanup processes on node shutdown
  def cleanup_actions(self):
    rospy.loginfo(self.node_name + ": Shutting down: Executing script cleanup actions")
    if self.serial_port is not None:
      self.serial_port.close()
      
if __name__ == '__main__':
  SealiteNode()






