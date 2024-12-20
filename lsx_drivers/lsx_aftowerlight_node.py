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


### Set the namespace before importing nepi_ros
import os
#os.environ["ROS_NAMESPACE"] = "/nepi/s2x"
import serial
import serial.tools.list_ports
import time
import re
import sys

from nepi_edge_sdk_base.device_if_lsx import ROSLSXDeviceIF

from nepi_ros_interfaces.msg import LSXStatus

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_drv
from nepi_edge_sdk_base import nepi_settings


PKG_NAME = 'LSX_AFTOWER' # Use in display menus
FILE_TYPE = 'NODE'


TEST_NEX_DICT = {
'group': 'LSX',
'group_id': 'AFTOWER',
'pkg_name': 'LSX_AFTOWER',
'NODE_DICT': {
    'file_name': 'lsx_aftowerlight_node.py',
    'module_name': 'lsx_aftowerlight_node',
    'class_name': 'AfTowerLightNode',
},
'DRIVER_DICT': {
    'file_name': '' ,
    'module_name': '' ,
    'class_name':  ''
},
'DISCOVERY_DICT': {
    'file_name': 'lsx_aftowerlight_discovery.py',
    'module_name': 'lsx_aftowerlight_discovery',
    'class_name': 'AfTowerLightDiscovery',
    'interfaces': ['USB'],
    'options_1_dict': {
        'default_val': '9600',
        'set_val': '9600'
    },
    'options_2_dict': {
        'default_val': 'None',
        'set_val': 'None'
    },
    'method': 'AUTO', 
    'include_ids': ['29987'],
    'exclude_ids': []
},
'DEVICE_DICT': {'device_path': '/dev/ttyUSB0'},
'path': '/opt/nepi/ros/lib/nepi_drivers',
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
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
  drv_dict = dict()   
  ### LXS Driver NODE Initialization
  def __init__(self):
      nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
      self.node_name = nepi_ros.get_node_name()
      self.base_namespace = nepi_ros.get_base_namespace()
      nepi_msg.createMsgPublishers(self)
      nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
      ##############################
      # Get required drv driver dict info
      self.drv_dict = nepi_ros.get_param(self,'~drv_dict',TEST_NEX_DICT) 
      #nepi_msg.publishMsgWarn(self,"AFTOWER_NODE: " + str(self.drv_dict))
      self.ser_port_str = self.drv_dict['DEVICE_DICT']['device_path'] 
      self.ser_buad_int = self.drv_dict['DISCOVERY_DICT']['option_1_dict']['set_val'] 
      nepi_msg.publishMsgInfo(self,"Connecting to Device on port " + self.ser_port_str + " with buad " + self.ser_buad_int)
      ################################################
      ### Try and connect to device
      self.connected = self.connect()
      if self.connected:
        # Create LSX ROS node
        nepi_msg.publishMsgInfo(self,'Connected')
      else:
        nepi_msg.publishMsgInfo(self,"Shutting down node")
        nepi_msg.publishMsgInfo(self,"Specified serial port not available")
        nepi_ros.signal_shutdown("Serial port not available")  


      # Initialize settings
      self.cap_settings = self.getCapSettings()
      self.factory_settings = self.getFactorySettings()

      # Launch the LSX interface --  this takes care of initializing all the camera settings from config. file
      nepi_msg.publishMsgInfo(self,"Launching NEPI LSX (ROS) interface...")
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
                  supportsBlinking = True,
                  enableStrobeFunction = None, #self.setStrobeEnable,
                  reports_temp = False, 
                  reports_power = False
                 )
    
      # Initialization Complete
      nepi_msg.publishMsgInfo(self,"Initialization Complete")
      #Set up node shutdown
      nepi_ros.on_shutdown(self.cleanup_actions)
      # Spin forever (until object is detected)
      nepi_ros.spin() 


  #**********************
  # Device setting functions


  def getCapSettings(self):
      return nepi_settings.NONE_CAP_SETTINGS

  def getFactorySettings(self):
      return nepi_settings.NONE_SETTINGS

  def getSettings(self):
      return nepi_settings.NONE_SETTINGS

  def setSetting(self,setting_name,setting_data):
    return True


  def settingUpdateFunction(self,setting):
      success = False
      setting_str = str(setting)
      [setting_name, s_type, data] = nepi_ros.get_data_from_setting(setting)
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
        #nepi_msg.publishMsgWarn(self,"Serial port open")
        success = True
      except:
        nepi_msg.publishMsgWarn(self,"Failed to open serial port")
      self.serial_port
    else: 
      nepi_msg.publishMsgWarn(self,"Failed to find serial port")
    return success


  def send_msg(self,ser_msg):
    response = None
    if self.serial_port is not None and not nepi_ros.is_shutdown():
      sleep_time = .1
      timeout = 2
      timer = 0
      while self.serial_busy == True and timer < timeout and not nepi_ros.is_shutdown():
          time.sleep(sleep_time ) # Wait for serial port to be available
          timer += sleep_time 
      self.serial_busy = True
      try:
        
        self.serial_port.write(bytes([ser_msg]))
      except Exception as e:
        nepi_msg.publishMsgInfo(self,"Failed to send message")
    else:
      nepi_msg.publishMsgInfo(self,"serial port not defined, returning empty string")
    self.serial_busy = False
    return response

  ### Function for checking if port is available
  def check_port(self,port_str):
    success = False
    ports = serial.tools.list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      nepi_msg.publishMsgInfo(self,"Checking port: " + loc)
      nepi_msg.publishMsgInfo(self,"For port: " + port_str)
      if loc == port_str:
        success = True
    return success

  #######################
  ### Cleanup processes on node shutdown
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")
    if self.serial_port is not None:
      self.serial_port.close()
      
if __name__ == '__main__':
  AfTowerLightNode()






