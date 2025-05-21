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


### Set the namespace before importing nepi_ros
import os
import serial
import serial.tools.list_ports
import time
import re
import sys

from nepi_ros_interfaces.msg import LSXStatus

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings

from nepi_api.device_if_lsx import LSXDeviceIF
from nepi_api.messages_if import MsgIF




PKG_NAME = 'LSX_DEEPSEA_SEALITE'
FILE_TYPE = 'NODE'


DEFAULT_MIN = '1'
DEFAULT_MAX = '50'
DEFAULT_CURVE = [DEFAULT_MIN,DEFAULT_MAX,'1','5','70','95','4.5']

#########################################
# Sealite LSX Driver Node Class
#########################################

class SealiteNode(object):
  ### LXS Driver Settings
  # Set driver capability parameters

  #######################
  DEFAULT_NODE_NAME='sealite'


  CAP_SETTINGS = dict(
    min_intensity_percent = {"type":"Int","name":"min_intensity_percent","options":["1","100"]},
    max_intensity_percent =  {"type":"Int","name":"max_intensity_percent","options":["1","100"]}
  )

  FACTORY_SETTINGS = dict(
    min_intensity_percent = {"type":"Int","name":"min_intensity_percent","value":str(DEFAULT_MIN)},
    max_intensity_percent =  {"type":"Int","name":"max_intensity_percent","value": str(DEFAULT_MAX)}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  settingFunctions = dict(
    min_intensity_percent = {'get':'getMinIntensityPercent', 'set': 'setMinIntensityPercent'},
    max_intensity_percent = {'get':'getMaxIntensityPercent', 'set': 'setMaxIntensityPercent'}
  )
  
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

  settings_dict = FACTORY_SETTINGS
  
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

  addr_str = ""

  cur_curve = DEFAULT_CURVE

  ### LXS Driver NODE Initialization
  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
  drv_dict = dict()                                                    
  def __init__(self):
      ####  NODE Initialization ####
      nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
      self.class_name = type(self).__name__
      self.base_namespace = nepi_ros.get_base_namespace()
      self.node_name = nepi_ros.get_node_name()
      self.node_namespace = nepi_ros.get_node_namespace()

      ##############################  
      # Create Msg Class
      self.msg_if = MsgIF(log_name = self.class_name)
      self.msg_if.pub_info("Starting Node Initialization Processes")

      ##############################  
      # Initialize Class Variables

      # Get required drv driver dict info
      self.drv_dict = nepi_ros.get_param('~drv_dict',dict()) 
      #self.msg_if.pub_warn("Nex_Dict: " + str(self.drv_dict))
      try:
        self.port_str = self.drv_dict['DEVICE_DICT']['device_path'] 
        self.baud_str = self.drv_dict['DEVICE_DICT']['baud_str'] 
        self.baud_int = int(self.baud_str)
        self.addr_str = self.drv_dict['DEVICE_DICT']['addr_str'] 
      except Exception as e:
        self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
        nepi_ros.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
        return
      # Address string must be three char long
      zero_prefix_len = 3-len(self.addr_str)
      for z in range(zero_prefix_len):
        self.addr_str = ('0' + self.addr_str)  
      ################################################  
      self.msg_if.pub_info("Connecting to Device on port " + self.port_str + " with baud " + self.baud_str)
      ### Try and connect to device
      self.connected = self.connect() 
      if self.connected:
        # Create LSX  node
        self.msg_if.pub_info("Connected")
        # Initialize settings
        self.cap_settings = self.getCapSettings()
        self.factory_settings = self.getFactorySettings()

        # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
        self.msg_if.pub_info("Launching NEPI LSX () interface...")
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

        self.lsx_if = LSXDeviceIF(
                    device_info = self.device_info_dict, 
                    getStatusFunction = self.getStatus,
                    capSettings = self.cap_settings,
                    factorySettings = self.factory_settings,
                    settingUpdateFunction=self.settingUpdateFunction,
                    getSettingsFunction=self.getSettings,
                    factoryControls = self.FACTORY_CONTROLS,
                    standbyEnableFunction = None,
                    turnOnOffFunction = self.turnOnOff,
                    setIntensityRatioFunction = self.setIntensityRatio, 
                    blinkOnOffFunction = None,
                    reports_temp = True, 
                    reports_power = False
                    )
      
        # Start an sealite activity check process that kills node after some number of failed comms attempts
        self.msg_if.pub_info("Starting an activity check process")
        nepi_ros.start_timer_process(nepi_ros.ros_duration(0.2), self.check_timer_callback)
        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_ros.on_shutdown(self.cleanup_actions)
        # Spin forever (until object is detected)
        nepi_ros.spin()
      else:
        self.msg_if.pub_info("Shutting down node")
        self.msg_if.pub_info("Specified serial port not available")
        nepi_ros.signal_shutdown("Serial port not available")   



      #**********************
      # Device setting functions


  def getCapSettings(self):
      return self.CAP_SETTINGS

  def getFactorySettings(self):
    settings = self.getSettings()
    #Apply factory setting overides
    for setting_name in settings.keys():
      if setting_name in self.FACTORY_SETTINGS_OVERRIDES:
            settings[setting_name]['value'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
    return settings



  def getSettings(self):
      settings = dict()
      for setting_name in self.cap_settings.keys():
        cap_setting = self.cap_settings[setting_name]
        setting = dict()
        setting["name"] = setting_name
        setting["type"] = cap_setting['type']
        val = None
        if setting_name in self.settingFunctions.keys():
          function_str_name = self.settingFunctions[setting_name]['get']
          #self.msg_if.pub_info("Calling get setting function " + function_str_name)
          get_function = globals()[function_str_name]
          val = get_function(self)
        if val is not None:
            setting["value"] = str(val)
            settings[setting_name] = setting
      return settings



  def setSetting(self,setting_name,val):
    success = False
    if setting_name in self.settingFunctions.keys():
          function_str_name = self.settingFunctions[setting_name]['set']
          #self.msg_if.pub_info("Calling set setting function " + function_str_name)
          set_function = globals()[function_str_name]
          success = set_function(self,val)
    return success


  def settingUpdateFunction(self,setting):
      success = False
      setting_str = str(setting)
      [setting_name, s_type, data] = nepi_settings.get_data_from_setting(setting)
      if data is not None:
          setting_data = data
          found_setting = False
          if setting_name in self.cap_settings.keys():
              found_setting = True
              success, msg = self.setSetting(setting_name,setting_data)
              if success:
                  msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)
          if found_setting is False:
              msg = (self.node_name  + " Setting name" + setting_str + " is not supported")                 
      else:
          msg = (self.node_name  + " Setting data" + setting_str + " is None")
      return success, msg

  ##############
  ### Settings Functions

  global getMinIntensityPercent
  def getMinIntensityPercent(self):
    success = False
    val = '-999'
    ser_msg= ('!' + self.addr_str + ':curv=?')
    response = self.send_msg(ser_msg)
    response_curv_parts = response.split(',')
    if len(response_curv_parts) == 7:
      self.cur_curv = response_curv_parts
      val = response_curv_parts[0]
      success = True
    return val

  global setMinIntensityPercent
  def setMinIntensityPercent(self,val):
    cur_curv = self.cur_curve
    cur_max = cur_curv[1]
    #success = False
    if int(val) < int(cur_max):
      cur_curv[0] = val
      curv_str = ", ".join(cur_curv)
      ser_msg_start= ('!' + self.addr_str + ':CURV=')
      ser_msg = ser_msg_start + curv_str
      response = self.send_msg(ser_msg)
      if response == 'ACK':
        self.cur_curv = cur_curv
        success = True
      return success

  global getMaxIntensityPercent
  def getMaxIntensityPercent(self):
    success = False
    val = '-999'
    ser_msg= ('!' + self.addr_str + ':CURV=?')
    response = self.send_msg(ser_msg)
    response_curv_parts = response.split(',')
    if len(response_curv_parts) == 7:
      self.cur_curv = response_curv_parts
      val = response_curv_parts[0]
      success = True
    return val

  global setMaxIntensityPercent
  def setMaxIntensityPercent(self,val):
    cur_curv = self.cur_curve
    cur_min = cur_curv[0]
    success = False
    if int(val) > int(cur_min):
      cur_curv[1] = val
      curv_str = ", ".join(cur_curv)
      ser_msg_start= ('!' + self.addr_str + ':CURV=')
      ser_msg = ser_msg_start + curv_str
      response = self.send_msg(ser_msg)
      if response == 'ACK':
        self.cur_curv = cur_curv
        success = True        
    return success





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
    status_msg.blink_state = False
    status_msg.blink_interval = 0
    status_msg.temp_c = self.temp_c
    status_msg.power_w = 0
    return(status_msg)


  def update_status_values(self):
    success = True
    # Update standby status
    #self.msg_if.pub_info("Updating standby status")
    ser_msg= ('!' + self.addr_str + ':STBY?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      if response == "0":
        self.standby_state = False
      elif response == "1":
        self.standby_state = True
      else:
        success = False
      #self.msg_if.pub_info("Standby: " + str(self.standby_state))
    else:
      success = False
    # Update intensity_ratio status
    #self.msg_if.pub_info("Updating intensity_ratio status")
    ser_msg= ('!' + self.addr_str + ':LOUT?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      try:
        self.intensity_ratio = float(response)/100
        #self.msg_if.pub_info("Intensity Ratio: " + str(self.intensity_ratio))
      except Exception as i:
        self.intensity_ratio = -999
        self.msg_if.pub_warn("Level response was not valid number")
        success = False
      #self.msg_if.pub_info("Intensity: " + str(self.intensity_ratio))
    else:
      self.intensity_ratio = -999
      success = False
    # Update strobe enable status
    #self.msg_if.pub_info("Updating strobe enable status")
    ser_msg= ('!' + self.addr_str + ':PMOD?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      if response == "0":
        self.strobe_state = False
      elif response == "1" or response == "2":
       self.strobe_state = True
      else:
        success = False
      #self.msg_if.pub_info("Strobe Enable: " + str(self.strobe_state))
    else:
      success = False
    # Update temp status
    #self.msg_if.pub_info("Updating temp status")
    ser_msg= ('!' + self.addr_str + ':TEMP?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      try:
        temp_c = int(float(response))
        success = True
        #self.msg_if.pub_info("Temp Deg C: " + str(temp_c))
      except Exception as t:
        temp_c = 255
        self.msg_if.pub_warn("Temp response was not valid number")
        success = False
      #self.msg_if.pub_info("Temp C: " + str(temp_c))
    else:
      temp_c = 255
      success = False
    if temp_c < 0 or temp_c > 255:
      temp_c = 255
    self.temp_c = temp_c
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
    self.on_off_state = turn_on_off
    if turn_on_off == False:
      self.setIntensityFunction(0)
    else:
      self.setIntensityFunction(self.intensity_ratio)
    
  def blinkOnOff(self,blink_on_off):
    self.on_off_state = turn_on_off
    if turn_on_off == False:
      self.setIntensityFunction(0)
    else:
      self.setIntensityFunction(self.intensity_ratio)
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
    return success 

  def setIntensityFunction(self,intensity_ratio):
    level_val = int(100*intensity_ratio) * int(self.on_off_state)
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
      self.msg_if.pub_warn("Failed to send message")
    time.sleep(.01)
    try:
      bs = self.serial_port.readline()
    except Exception as e:
      self.msg_if.pub_warn("Failed to receive message")
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
      self.msg_if.pub_warn("Shutting down device: " +  self.addr_str + " on port " + self.port_str)
      self.msg_if.pub_warn("Too many comm failures")
      nepi_ros.signal_shutdown("To many comm failures")   
   
  ### Function to try and connect to device at given port and baudrate
  def connect(self):
    success = False
    port_check = self.check_port(self.port_str)
    if port_check is True:
      try:
        # Try and open serial port
        self.msg_if.pub_info("Opening serial port " + self.port_str + " with baudrate: " + self.baud_str)
        self.serial_port = serial.Serial(self.port_str,self.baud_int,timeout = 0.1)
        self.msg_if.pub_info("Serial port opened")
        # Send Message
        self.msg_if.pub_info("Requesting info for device: " + self.addr_str)
        ser_msg = ('!' + self.addr_str + ':INFO?')
        #self.msg_if.pub_info("Sending serial string: " + ser_msg)
        response = self.send_msg(ser_msg)
        #self.msg_if.pub_info("Got response message: " + response)
        if len(response) > 0:
          if response != None and response != "?" and response[3] == ",":
            if len(response) > 4:
              ret_addr = response[0:3]
              #self.msg_if.pub_info("Returned address value: " + ret_addr)
              if ret_addr == self.addr_str:
                self.msg_if.pub_info("Connected to device at address: " +  self.addr_str)
                res_split = response.split(',')
                if len(res_split) > 5:
                # Update serial, hardware, and software status values
                  self.serial_num = res_split[2]
                  self.hw_version = res_split[3]
                  self.sw_version = res_split[4]
                success = True
              else:
                self.msg_if.pub_warn("Device returned address: " + ret_addr + " does not match: " +  self.addr_str)
            else:
              self.msg_if.pub_warn("Device returned invalid response")
          else:
            self.msg_if.pub_warn("Device returned empty response")
        else:
          self.msg_if.pub_warn("Device returned invalid response")
      except Exception as e:
        self.msg_if.pub_warn("Something went wrong with connect function at serial port at: " + self.port_str + "(" + str(e) + ")" )
    else:
      self.msg_if.pub_warn("serial port not active")
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
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    if self.serial_port is not None:
      self.serial_port.close()
      
if __name__ == '__main__':
  SealiteNode()






