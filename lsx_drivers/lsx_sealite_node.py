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

PKG_NAME = 'LSX_SEALITE' # Use in display menus
FILE_TYPE = 'NODE'



TEST_NEX_DICT = {
'group': 'LSX',
'group_id': 'SEALITE',
'pkg_name': 'LSX_SEALITE',
'NODE_DICT': {
    'file_name': 'lsx_sealite_node.py',
    'module_name': 'lsx_sealite_node',
    'class_name': 'SealiteNode',
},
'DRIVER_DICT': {
    'file_name': '' ,
    'module_name': '' ,
    'class_name':  ''
},
'DISCOVERY_DICT': {
    'file_name': 'lsx_sealite_discovery.py',
    'module_name': 'lsx_sealite_discovery',
    'class_name': 'SealiteDiscovery',
    'interfaces': ['SERIAL','USBSERIAL'],
    'options_1_dict': {
        'default_val': '57600',
        'set_val': '57600'
    },
    'options_2_dict': {
        'default_val': '10',
        'set_val': '10'
    },
    'method': 'AUTO', 
    'include_ids': [],
    'exclude_ids': ['ttyACM']
},
'DEVICE_DICT': {'device_path': '/dev/ttyUSB0','baud_int':57600, 'addr': '001'},
'path': '/opt/nepi/ros/lib/nepi_drivers',
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
  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
  drv_dict = dict()                                                    
  def __init__(self):
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    # Get required drv driver dict info
    self.drv_dict = nepi_ros.get_param(self,'~drv_dict',TEST_NEX_DICT) 
    #nepi_msg.publishMsgWarn(self,"Nex_Dict: " + str(self.drv_dict))
    self.driver_path = self.drv_dict['path']
    self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
    self.driver_module = self.drv_dict['DRIVER_DICT']['module_name']
    self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
    self.ser_port_str = self.drv_dict['DEVICE_DICT']['device_path'] 
    self.ser_buad_int = self.drv_dict['DEVICE_DICT']['baud_int'] 
    self.addr_str = self.drv_dict['DEVICE_DICT']['addr_str'] 
    # Address string must be three char long
    zero_prefix_len = 3-len(self.addr_str)
    for z in range(zero_prefix_len):
      self.addr_str = ('0' + self.addr_str)  
    ################################################  
    nepi_msg.publishMsgInfo(self,"Connecting to Device on port " + self.ser_port_str + " with buad " + self.ser_buad_int)
    ### Try and connect to device
    self.connected = self.connect() 
    if self.connected:
      # Create LSX ROS node
      nepi_msg.publishMsgInfo(self,'Connected')
      # Initialize settings
      self.cap_settings = self.getCapSettings()
      self.factory_settings = self.getFactorySettings()

      # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
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
                  standbyEnableFunction = self.setStandby,
                  turnOnOffFunction = self.turnOnOff,
                  setIntensityRatioFunction = self.setIntensityRatio, 
                  supportsBlinking = True,
                  enableStrobeFunction = self.setStrobeEnable,
                  reports_temp = True, 
                  reports_power = False
                 )
    
      # Start an sealite activity check process that kills node after some number of failed comms attempts
      nepi_msg.publishMsgInfo(self,"Starting an activity check process")
      nepi_ros.start_timer_process(nepi_ros.duration(0.2), self.check_timer_callback)
      # Initialization Complete
      lsx_if.publishMsg(" Initialization Complete")
      #Set up node shutdown
      nepi_ros.on_shutdown(self.cleanup_actions)
      # Spin forever (until object is detected)
      nepi_ros.spin()
    else:
      lsx_if.publishMsg(" Shutting down node")
      lsx_if.publishMsg(" Specified serial port not available")
      nepi_ros.signal_shutdown("Serial port not available")   



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
      #nepi_msg.publishMsgInfo(self,"Standby: " + str(self.standby_state))
    else:
      success = False
    # Update intensity_ratio status
    #nepi_msg.publishMsgInfo(self,"Updating intensity_ratio status")
    ser_msg= ('!' + self.addr_str + ':LOUT?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      try:
        self.intensity_ratio = float(response)/100
        #nepi_msg.publishMsgInfo(self,"Intensity Ratio: " + str(self.intensity_ratio))
      except Exception as i:
        self.intensity_ratio = -999
        nepi_msg.publishMsgWarn(self,"Level response was not valid number")
        success = False
      #nepi_msg.publishMsgInfo(self,"Intensity: " + str(self.intensity_ratio))
    else:
      self.intensity_ratio = -999
      success = False
    # Update strobe enable status
    #nepi_msg.publishMsgInfo(self,"Updating strobe enable status")
    ser_msg= ('!' + self.addr_str + ':PMOD?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      if response == "0":
        self.strobe_state = False
      elif response == "1" or response == "2":
       self.strobe_state = True
      else:
        success = False
      #nepi_msg.publishMsgInfo(self,"Strobe Enable: " + str(self.strobe_state))
    else:
      success = False
    # Update temp status
    #nepi_msg.publishMsgInfo(self,"Updating temp status")
    ser_msg= ('!' + self.addr_str + ':TEMP?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      try:
        self.temp_c = int(float(response))
        #nepi_msg.publishMsgInfo(self,"Temp Deg C: " + str(self.temp_c))
      except Exception as t:
        self.temp_c = 255
        nepi_msg.publishMsgWarn(self,"Temp response was not valid number")
        success = False
      #nepi_msg.publishMsgInfo(self,"Temp C: " + str(self.temp_c))
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
      while self.serial_busy == True and not nepi_ros.is_shutdown():
        time.sleep(0.01) # Wait for serial port to be available
      self.serial_busy = True
      self.serial_port.write(b)
    except Exception as e:
      nepi_msg.publishMsgWarn(self,"Failed to send message")
    time.sleep(.01)
    try:
      bs = self.serial_port.readline()
    except Exception as e:
      nepi_msg.publishMsgWarn(self,"Failed to receive message")
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
      nepi_msg.publishMsgWarn(self,"Shutting down device: " +  self.addr_str + " on port " + self.ser_port_str)
      nepi_msg.publishMsgWarn(self,"Too many comm failures")
      nepi_ros.signal_shutdown("To many comm failures")   
   
  ### Function to try and connect to device at given port and buadrate
  def connect(self):
    success = False
    port_check = self.check_port(self.ser_port_str)
    if port_check is True:
      try:
        # Try and open serial port
        nepi_msg.publishMsgInfo(self,"Opening serial port " + self.ser_port_str + " with buadrate: " + str(self.ser_buad_int))
        self.serial_port = serial.Serial(self.ser_port_str,self.ser_buad_int,timeout = 0.1)
        nepi_msg.publishMsgInfo(self,"Serial port opened")
        # Send Message
        nepi_msg.publishMsgInfo(self,"Requesting info for device: " + self.addr_str)
        ser_msg = ('!' + self.addr_str + ':INFO?')
        #nepi_msg.publishMsgInfo(self,"Sending serial string: " + ser_msg)
        response = self.send_msg(ser_msg)
        #nepi_msg.publishMsgInfo(self,"Got response message: " + response)
        if len(response) > 0:
          if response != None and response != "?" and response[3] == ",":
            if len(response) > 4:
              ret_addr = response[0:3]
              #nepi_msg.publishMsgInfo(self,"Returned address value: " + ret_addr)
              if ret_addr == self.addr_str:
                nepi_msg.publishMsgInfo(self,"Connected to device at address: " +  self.addr_str)
                res_split = response.split(',')
                if len(res_split) > 5:
                # Update serial, hardware, and software status values
                  self.serial_num = res_split[2]
                  self.hw_version = res_split[3]
                  self.sw_version = res_split[4]
                success = True
              else:
                nepi_msg.publishMsgWarn(self,"Device returned address: " + ret_addr + " does not match: " +  self.addr_str)
            else:
              nepi_msg.publishMsgWarn(self,"Device returned invalid response")
          else:
            nepi_msg.publishMsgWarn(self,"Device returned empty response")
        else:
          nepi_msg.publishMsgWarn(self,"Device returned invalid response")
      except Exception as e:
        nepi_msg.publishMsgWarn(self,"Something went wrong with connect function at serial port at: " + self.ser_port_str + "(" + str(e) + ")" )
    else:
      nepi_msg.publishMsgWarn(self,"serial port not active")
    return success


  def send_msg(self,ser_msg):
    response = None
    if self.serial_port is not None and not nepi_ros.is_shutdown():
      ser_str = (ser_msg + '\r\n')
      b=bytearray()
      b.extend(map(ord, ser_str))
      
      sleep_time = .1
      timeout = 2
      timer = 0
      while self.serial_busy == True and timer < timeout and not nepi_ros.is_shutdown():
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
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")
    if self.serial_port is not None:
      self.serial_port.close()
      
if __name__ == '__main__':
  SealiteNode()






