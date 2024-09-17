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


PKG_NAME = 'LSX_AFTOWER' # Use in display menus
DESCRIPTION = 'Driver package for adafruit usb tower light'
FILE_TYPE = 'NODE'
CLASS_NAME = 'AfTowerLightNode' # Should Match Class Name
GROUP ='LSX'
GROUP_ID = 'AFTOWER' 

DRIVER_PKG_NAME = 'None' # 'Required Driver PKG_NAME or 'None'
DRIVER_INTERFACES = ['USBSERIAL'] # 'USB','IP','SERIALUSB','SERIAL','CANBUS'
DRIVER_OPTIONS_1_NAME = 'BaudRate'
DRIVER_OPTIONS_1 = ['9600'] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_1 = '9600'
DRIVER_OPTIONS_2_NAME = 'None'
DRIVER_OPTIONS_2 = [] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_2 = 'None'

DISCOVERY_PKG_NAME = 'LSX_AFTOWER' # 'Required Discovery PKG_NAME or 'None'
DISCOVERY_METHOD = 'AUTO'  # 'AUTO', 'MANUAL', or 'OTHER' if managed by seperate application
DISCOVERY_IDS = ['29987']  # List of string identifiers for discovery process
DISCOVERY_IGNORE_IDS = [] # List of string identifiers for discovery process

TEST_NEX_DICT = {
    'group': 'LSX',
    'group_id': 'AFTOWER',
    'node_file_name': 'lsx_aftowerlight_node.py',
    'node_file_path': '/opt/nepi/ros/lib/nep_drivers',
    'node_module_name': 'lsx_aftowerlight_node',
    'node_class_name': 'AfTowerLightNode',
    'driver_name': "None",
    'driver_file_name': "None" ,
    'driver_file_path':  "None",
    'driver_module_name': "None" ,
    'driver_class_name':  "None",
    'driver_interfaces': ['USBSERIAL'],
    'driver_options_1': ['9600'],
    'driver_default_option_1': '9600',
    'driver_set_option_1': '9600',
    'driver_options_2': [],
    'driver_default_option_2': 'None',
    'driver_set_option_2': 'None',
    'discovery_name': 'LSX_AFTOWER', 
    'discovery_file_name': "lsx_aftowerlight_discovery.py",
    'discovery_file_path': "/opt/nepi/ros/lib/nep_drivers",
    'discovery_module_name': "lsx_aftowerlight_discovery",
    'discovery_class_name': "AfTowerLightDiscovery",
    'discovery_method': 'AUTO', 
    'discovery_ids': ['29987'],
    'discovery_ignore_ids': [],
    'device_path': '/dev/ttyUSB0',
    'order': 1,
    'active': True,
    'msg': ""
    }


#########################################
# Sealite LSX Driver Node Class
#########################################

class AfTowerLightNode(object):

  ### LXS Driver Settings
  # Set driver capability parameters

  #######################
  FACTORY_SETTINGS_OVERRIDES = dict( )

  #Factory Control Values 
  FACTORY_CONTROLS = dict( standby_enabled = False,
    on_off_state = False,
    intensity_ratio = 0.0,
    color_selection = "None",
    kelvin_val = 4000,
    strobe_enabled = False,
    blink_interval_sec = 2,
    blink_enabled = False
  )
  color_options = ['GREEN','YELLOW','RED']
  kelvin_limits = [1000,10000]

  device_info_dict = dict(node_name = "",
                        device_name = "",
                        identifier = "",
                        serial_number = "",
                        hw_version = "",
                        sw_version = "")
  
  # Initialize some parameters
  serial_num = "Unknown"
  hw_version = "Unknown"
  sw_version = "Unknown"

  serial_port = None
  serial_busy = False
  connected = False

  on_off_state = False
  blink_state = False
  standby_state = False
  strobe_state = False
  intensity_ratio = 0.0
  current_color = 'GREEN'
  last_color = 'GREEN'
  kelvin_val = 4000
  temp_c = 0.0
  power_w = 0.0

  lsx_if = None
  
  last_color = 'GREEN'
  on_off_dict = dict()
  on_off_dict['GREEN'] = [0x14,0x24] # Off,On
  on_off_dict['YELLOW'] = [0x12,0x22]
  on_off_dict['RED'] = [0x11,0x21]



  ################################################
  DEFAULT_NODE_NAME='af_tower_light'
  ### LXS Driver NODE Initialization
  def __init__(self):
      # Launch the ROS node
      rospy.loginfo("Starting " + self.DEFAULT_NODE_NAME)
      rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
      self.node_name = rospy.get_name().split('/')[-1]
      # Get nex_dict from param servers
      self.nex_dict = rospy.get_param('~nex_dict',TEST_NEX_DICT) 
      #rospy.logwarn(self.node_name + ": ZED_NODE: " + str(self.nex_dict))
      self.ser_port_str = self.nex_dict['device_path'] #rospy.get_param('~port_str') # Crash if none provided
      self.ser_buad_int = self.nex_dict[ 'driver_set_option_1'] #rospy.get_param('~baud_int') # Crash if none provided
      rospy.loginfo(self.node_name + ": Connecting to Device on port %s with buad %s",self.ser_port_str,self.ser_buad_int)
      ################################################
      ### Try and connect to device
      self.connected = self.connect()
      if self.connected:
        # Create LSX ROS node
        rospy.loginfo(self.node_name + ': Connected')
      else:
        rospy.loginfo(self.node_name + ": Shutting down node")
        rospy.loginfo(self.node_name + ": Specified serial port not available")
        rospy.signal_shutdown("Serial port not available")  


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
                  standbyEnableFunction = None, #self.setStandby,
                  turnOnOffFunction = self.turnOnOff,
                  setIntensityRatioFunction = None, #self.setIntensityRatio, 
                  color_options_list =  self.color_options, 
                  setColorFunction = self.setColorOption,
                  kelvin_limits_list = None, #self.kelvin_limits, 
                  setKelvinFunction =None, #self.setKelvinVal,
                  blinkOnOffFunction = self.blinkOnOff,
                  enableStrobeFunction = None, #self.setStrobeEnable,
                  reports_temp = False, 
                  reports_power = False
                 )
    
      # Initialization Complete
      rospy.loginfo(self.node_name + ": Initialization Complete")
      #Set up node shutdown
      rospy.on_shutdown(self.cleanup_actions)
      # Spin forever (until object is detected)
      rospy.spin() 


  #**********************
  # Device setting functions

  def getSettingsOptionsDict(self):
    settings_options_dict = dict()
    return settings_options_dict


  def getCapSettings(self):
      settings_cap_dict = self.getSettingsOptionsDict()
      return nepi_ros.NONE_SETTINGS

  def getFactorySettings(self):
      settings_options_dict = self.getSettingsOptionsDict()
      settings = nepi_ros.NONE_SETTINGS
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
      settings_dict = self.getSettingsDict()
      settings = nepi_ros.NONE_SETTINGS
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
    status_msg.standby_state = self.standby_state
    status_msg.on_off_state = self.on_off_state
    status_msg.strobe_state = self.strobe_state
    status_msg.intensity_ratio = self.intensity_ratio
    status_msg.color_setting = self.current_color
    status_msg.kelvin_setting = int(self.kelvin_val)
    status_msg.temp_c = int(self.temp_c)
    status_msg.power_w = self.power_w
    return(status_msg)


  def update_status_values(self):
    success = True
    return success

  #######################
  ### LSX IF Interface Functions


  def turnOnOff(self,turn_on_off):
    ON = self.on_off_dict[self.current_color][0]
    OFF = self.on_off_dict[self.current_color][1]
    if turn_on_off == True:
      self.send_msg(ON)
    else:
      self.send_msg(OFF)
    self.on_off_state = turn_on_off

  def blinkOnOff(self,blink_on_off):
    #rospy.logwarn(self.node_name + ": setting blink state to " + str(blink_on_off) )
    ON = self.on_off_dict[self.current_color][0]
    OFF = self.on_off_dict[self.current_color][1]
    new_state = blink_on_off
    if new_state == True:
        self.send_msg(ON)
    else:
      self.send_msg(OFF)
    self.blink_state = new_state


  def setColorOption(self,color_option):
    last_color = self.current_color
    cur_state = self.on_off_state or self.blink_state
    if color_option in self.color_options:
      if last_color != color_option:
          if cur_state == True:
            CUR_OFF = self.on_off_dict[self.current_color][1]
            self.send_msg(CUR_OFF)
          self.current_color = color_option
          if cur_state == True:
            CUR_ON = self.on_off_dict[self.current_color][0]
            self.send_msg(CUR_ON)
  '''
  ### The rest are just for testing

  def setStandby(self,standby_val):
    success = False
    self.standby_enabled = standby_val
    success = True
    return success 

  def setKelvinVal(self, kelvin_val):
    success = False
    self.kelvin_val = kelvin_val
    success = True
    return success  

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
    success = False
    self.intensity_ratio = intensity_ratio
    success = True
    return success  

  def setStrobeEnable(self,strobe_enable_val):
    success = False
    self.strobe_state = strobe_enable_val
    success = True
    return success  
  '''

  #######################
  ### Class Functions


  ### Function to try and connect to device at given port and buadrate
  def connect(self):
    success = False
    check_ser_port = self.check_port(self.ser_port_str)
    if check_ser_port == True:
      try: 
        self.serial_port = serial.Serial(self.ser_port_str,self.ser_buad_int,timeout = 0.1)
        #rospy.logwarn(self.node_name + ": Serial port open")
        success = True
      except:
        rospy.logwarn(self.node_name + ": Failed to open serial port")
      self.serial_port
    else: 
      rospy.logwarn(self.node_name + ": Failed to find serial port")
    return success


  def send_msg(self,ser_msg):
    response = None
    if self.serial_port is not None and not rospy.is_shutdown():
      sleep_time = .1
      timeout = 2
      timer = 0
      while self.serial_busy == True and timer < timeout and not rospy.is_shutdown():
          time.sleep(sleep_time ) # Wait for serial port to be available
          timer += sleep_time 
      self.serial_busy = True
      #rospy.loginfo(self.node_name + ":Sending " + str(ser_msg) + " message")
      try:
        
        self.serial_port.write(bytes([ser_msg]))
      except Exception as e:
        rospy.loginfo(self.node_name + ":Failed to send message")
    else:
      rospy.loginfo(self.node_name + ":serial port not defined, returning empty string")
    self.serial_busy = False
    return response

  ### Function for checking if port is available
  def check_port(self,port_str):
    success = False
    ports = serial.tools.list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      rospy.loginfo(self.node_name + ": Checking port: " + loc)
      rospy.loginfo(self.node_name + ": For port: " + port_str)
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
  AfTowerLightNode()






