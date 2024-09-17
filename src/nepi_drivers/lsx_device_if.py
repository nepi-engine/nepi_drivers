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
import rospy
import time


from nepi_edge_sdk_base import nepi_ros

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from std_msgs.msg import UInt8, Empty, String, Bool, Float32
from nepi_ros_interfaces.msg import LSXStatus
from nepi_ros_interfaces.srv import LSXCapabilitiesQuery, LSXCapabilitiesQueryResponse

from nepi_edge_sdk_base.settings_if import SettingsIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF


NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

class ROSLSXDeviceIF:
    STATUS_UPDATE_RATE_HZ = 2


    #Factory Control Values 
    FACTORY_CONTROLS_DICT = dict( standby_enabled = False,
    on_off_state = False,
    intensity_ratio = 0.0,
    color_selection = "None",
    kelvin_val = 4000,
    strobe_enbled = False,
    blink_interval_sec = 2,
    blink_enabled = False
    )

    # Define class variables

    getStatusFunction = None
    factory_device_name = "None"
    init_device_name = "None"
  
    has_standby_mode = False
    has_on_off_control = False
    has_intensity_control = False
    has_color_control = False
    color_options_list = ["None"]
    has_kelvin_control = False
    kelvin_limits_list = [1000,10000]
    has_blink_control = False
    has_hw_strobe = False
    reports_temp = False
    reports_power = False


    blink_timer_thread = None
    last_blink_state = False

    settings_if = None
    save_cfg_if = None
    
    rbx_status_pub_interval = float(1)/float(STATUS_UPDATE_RATE_HZ)

    ## Initialize Class Variables

    def resetFactoryCb(self, msg):
        rospy.loginfo("LSX_IF: Received RBX Driver Factory Reset")
        rospy.loginfo(msg)
        self.resetFactory()

    def resetFactory(self):
        self.settings_if.resetFactorySettings()
        rospy.set_param('~lsx/device_name', self.factory_device_name)
        rospy.set_param('~lsx/standby_enabled', self.factory_controls_dict.get("standby_enabled"))
        rospy.set_param('~lsx/on_off_state', self.factory_controls_dict.get("on_off_state"))
        rospy.set_param('~lsx/intensity_ratio', self.factory_controls_dict.get("intensity_ratio"))
        rospy.set_param('~lsx/color_selection', self.factory_controls_dict.get("color_selection"))
        rospy.set_param('~lsx/kelvin_val', self.factory_controls_dict.get("kelvin_val"))
        rospy.set_param('~lsx/strobe_enbled', self.factory_controls_dict.get("strobe_enbled"))
        rospy.set_param('~lsx/blink_interval_sec', self.factory_controls_dict.get("blink_interval_sec"))
        rospy.set_param('~lsx/blink_enabled', self.factory_controls_dict.get("blink_enabled"))
        self.updateFromParamServer()
        self.publish_status()

    def resetControlsCb(self, msg):
        rospy.loginfo("LSX_IF: Resetting LSX Device Controls")
        self.publishMsg(msg)
        self.resetControls()

    def resetControls(self):
      rospy.set_param('~lsx/device_name', self.init_device_name)
      self.updateFromParamServer()
      self.publish_status()

    def resetParamServer(self,do_updates = False):
        rospy.set_param('~lsx/device_name', self.init_device_name)
        rospy.set_param('~lsx/standby_enabled', self.init_standby_enabled)
        rospy.set_param('~lsx/on_off_state', self.init_on_off_state)
        rospy.set_param('~lsx/intensity_ratio', self.init_intensity_ratio)
        rospy.set_param('~lsx/color_selection', self.init_color_selection)
        rospy.set_param('~lsx/kelvin_val', self.init_kelvin_val)
        rospy.set_param('~lsx/strobe_enbled', self.init_strobe_enbled)
        rospy.set_param('~lsx/blink_interval_sec', self.init_blink_interval_sec)
        rospy.set_param('~lsx/blink_enabled', self.init_blink_enabled)
        if do_updates:
            self.updateFromParamServer()
            self.publishStatus()

    def initializeParamServer(self,do_updates = False):
        if self.settings_if is not None:
            self.settings_if.initializeParamServer(do_updates)
        self.init_device_name = rospy.get_param('~lsx/device_name', self.factory_device_name)
        self.init_standby_enabled = rospy.get_param('~lsx/standby_enabled', self.factory_controls_dict.get("standby_enabled"))
        self.init_on_off_state = rospy.get_param('~lsx/on_off_state', self.factory_controls_dict.get("on_off_state"))
        self.init_intensity_ratio = rospy.get_param('~lsx/intensity_ratio', self.factory_controls_dict.get("intensity_ratio"))
        self.init_color_selection = rospy.get_param('~lsx/color_selection', self.factory_controls_dict.get("color_selection"))
        self.init_kelvin_val = rospy.get_param('~lsx/kelvin_val', self.factory_controls_dict.get("kelvin_val"))
        self.init_strobe_enbled = rospy.get_param('~lsx/strobe_enbled', self.factory_controls_dict.get("strobe_enbled"))
        self.init_blink_interval_sec = rospy.get_param('~lsx/blink_interval_sec', self.factory_controls_dict.get("blink_interval_sec"))
        self.init_blink_enabled = rospy.get_param('~lsx/blink_enabled', self.factory_controls_dict.get("blink_enabled"))
        self.resetParamServer(do_updates)


    def updateFromParamServer(self):
        if self.settings_if is not None:
          self.settings_if.updateFromParamServer()
        if self.standbyEnableFunction is not None:
          val = rospy.get_param('~lsx/standby_enabled', self.init_standby_enabled)
          self.standbyEnableFunction(val)
        if self.turnOnOffFunction is not None:
          val = rospy.get_param('~lsx/on_off_state', self.init_on_off_state)
          self.turnOnOffFunction(val)
        if self.setIntensityRatioFunction is not None:
          val = rospy.get_param('~lsx/intensity_ratio', self.init_intensity_ratio)
          self.setIntensityRatioFunction(val)
        if self.setColorFunction is not None:
          val = rospy.get_param('~lsx/color_selection', self.init_color_selection)
          self.setColorFunction(val)
        if self.setKelvinFunction is not None:
          val = rospy.get_param('~lsx/kelvin_val', self.init_kelvin_val)
          self.setKelvinFunction(val)
        if self.enableStrobeFunction is not None:
          val = rospy.get_param('~lsx/strobe_enbled', self.init_strobe_enbled)
          self.enableStrobeFunction(val)
        if self.blinkOnOffFunction is not None:
          self.updateBlinkThread()

        self.resetParamServer()
        

  
    def setCurrentAsDefault(self):
        if self.settings_if is not None:
          self.settings_if.initializeParamServer(do_updates = False)
        pass # We only use the param server, no member variables to apply to param server
   
    ## Callback to regulary check device comms, track failures, and kill unresponsive device connections
    def statusTimerCb(self,timer):
      #Update the status message
      self.publish_status()

    ### Status callback
    def publish_status(self):
      # update status values from device
      if self.getStatusFunction is not None:
        status_msg=self.getStatusFunction()
        status_msg.user_name = rospy.get_param('~lsx/device_name', self.init_device_name)
        status_msg.blink_state = rospy.get_param('~lsx/blink_enabled', self.init_blink_enabled)
        status_msg.blink_interval = rospy.get_param('~lsx/blink_interval_sec', self.init_blink_interval_sec)
        if not rospy.is_shutdown():
          self.status_pub.publish(status_msg)


    ### Capabilities callback
    def capabilities_query_callback(self, _):
        return self.capabilities_report

    ### Device Name callbacks
    def updateDeviceNameCb(self, msg):
        self.publishMsg("LSX_IF: Received Device Name update msg")
        self.publishMsg(msg)
        new_device_name = msg.data
        self.updateDeviceName(new_device_name)

    def updateDeviceName(self, new_device_name):
        valid_name = True
        for char in self.BAD_NAME_CHAR_LIST:
            if new_device_name.find(char) != -1:
                valid_name = False
        if valid_name is False:
            self.update_error_msg("Received invalid device name update: " + new_device_name)
        else:
            rospy.set_param('~lsx/device_name', new_device_name)
        self.device_save_config_pub.publish(Empty())
        self.publish_status()


    def resetDeviceNameCb(self,msg):
        self.publishMsg("LSX_IF: Received Device Name reset msg")
        self.publishMsg(msg)
        self.resetDeviceName()

    def resetDeviceName(self):
        rospy.set_param('~lsx/device_name', self.factory_device_name)
        self.device_save_config_pub.publish(Empty())
        self.publish_status()

    ### LSX callbacks
    def setStandbyCb(self, standby_msg):
      self.publishMsg("Recieved standby message: (" + str(standby_msg) + ")")
      standby=standby_msg.data
      self.standbyEnableFunction(standby)
      rospy.set_param('~lsx/standby_enabled', standby)
      self.publish_status()


    def turnOnOffCb(self, on_off_msg):
      self.publishMsg("Recieved on off message: (" + str(on_off_msg) + ")")
      on_off=on_off_msg.data
      self.turnOnOffFunction(on_off)
      rospy.set_param('~lsx/on_off_state', on_off)
      self.publish_status()

    ### Set intensity callback
    def setIntensityRatioCb(self, intensity_msg):
      self.publishMsg("Recieved intensity message (" + str(intensity_msg) + ")")
      intensity=intensity_msg.data
      self.setIntensityRatioFunction(intensity)
      rospy.set_param('~lsx/intensity_ratio', intensity)
      self.publish_status()

    ### Set color selection callback
    def setColorCb(self, color_msg):
      self.publishMsg("Recieved color selection message (" + str(color_msg) + ")")
      color = color_msg.data
      if color in self.color_options_list:
        self.setColorFunction(color)
        rospy.set_param('~lsx/color_selection', color)
      self.publish_status()

    ### Set kelvin value callback
    def setKelvinCb(self, kelvin_msg):
      self.publishMsg("Recieved set kelvin message (" + str(kelvin_msg) + ")")
      kelvin = kelvin_msg.data
      if kelvin >= self.kelvin_limits_list[0] and kelvin <= self.kelvin_limits_list[1]:
        self.setKelvinFunction(kelvin)
        rospy.set_param('~lsx/kelvin_val', kelvin)
      self.publish_status()

    def setStrobeEnableCb(self, strobe_enable_msg):
      self.publishMsg("Recieved strobe enable message (" + str(strobe_enable_msg) + ")")
      strobe_enable=strobe_enable_msg.data
      self.enableStrobeFunction(strobe_enable)
      rospy.set_param('~lsx/strobe_enbled', strobe_enable)
      self.publish_status()


    def blinkOnOffCb(self, on_off_msg):
      self.publishMsg("Recieved blink on off message: (" + str(on_off_msg) + ")")
      on_off=on_off_msg.data
      cur_on_off = rospy.get_param('~lsx/blink_enabled', self.init_blink_enabled)
      rospy.set_param('~lsx/blink_enabled', on_off)
      if on_off != cur_on_off:
        self.updateBlinkThread()
      self.publish_status() 

    ### Set blink interval callback
    def setBlinkIntervalCb(self, blink_int_msg):
      self.publishMsg("Recieved blink interval message (" + str(blink_int_msg) + ")")
      blink_int = blink_int_msg.data
      cur_int = rospy.get_param('~lsx/blink_interval_sec', self.init_blink_interval_sec)
      if blink_int > 0 and blink_int < 10 and blink_int != cur_int:
        rospy.set_param('~lsx/blink_interval_sec', blink_int)
        self.publish_status()
        self.updateBlinkThread()

    def updateBlinkThread(self):
        enabled = rospy.get_param('~lsx/blink_enabled', self.init_blink_enabled)
        interval = rospy.get_param('~lsx/blink_interval_sec', self.init_blink_interval_sec)
        #self.publishMsg("Got blink process with enabled: " + str(enabled) + " and interval_sec: " + str(interval))
        if enabled == True:
          if self.blink_timer_thread is not None:
            self.publishMsg("Killing blink process")
            self.blink_timer_thread.shutdown()
            self.blink_timer_thread = None
            nepi_ros.sleep(interval,100)
          self.publishMsg("Starting blink process with enabled: " + str(enabled) + " and interval_sec: " + str(interval))
          self.blink_timer_thread = rospy.Timer(rospy.Duration(interval), self.blinkTimerThread)
        if enabled == False and self.blink_timer_thread is not None:
          self.publishMsg("Killing blink process")
          self.blink_timer_thread.shutdown()
          self.blink_timer_thread = None

    def blinkTimerThread(self,timer):
        new_state = self.last_blink_state == False
        self.last_blink_state = new_state
        #self.publishMsg("Setting blink to: " + str(new_state))
        self.blinkOnOffFunction(new_state)


           
    def __init__(self, device_info, getStatusFunction, capSettings, 
                 factorySettings, settingUpdateFunction, getSettingsFunction,
                 factoryControls = None, 
                 standbyEnableFunction = None, turnOnOffFunction = None,
                 setIntensityRatioFunction = None, 
                 color_options_list =  None, setColorFunction = None,
                 kelvin_limits_list = None, setKelvinFunction = None,
                 blinkOnOffFunction = None,
                 enableStrobeFunction = None,
                 reports_temp = False, reports_power = False
                 ):
        
        node_name = rospy.get_name().split('/')[-1]
        rospy.loginfo(node_name + ": Starting IF setup")
        # Create a node msg publisher
        self.msg_pub = rospy.Publisher("~messages", String, queue_size=1)
        time.sleep(1)
        
        self.node_name = device_info["node_name"]
        self.device_name = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.factory_device_name = device_info["device_name"] + "_" + device_info["identifier"]

        # Create and update factory controls dictionary
        self.factory_controls_dict = self.FACTORY_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]

        self.initializeParamServer(do_updates = False)

        # Start LSX node control subscribers
        self.standbyEnableFunction = standbyEnableFunction
        if self.standbyEnableFunction is not None:
            self.has_standby_mode = True
            set_standby_enable_sub = rospy.Subscriber("~lsx/set_standby", Bool, self.setStandbyCb, queue_size = 1)

        self.turnOnOffFunction = turnOnOffFunction
        if self.turnOnOffFunction is not None:
            self.has_on_off_control = True
            turn_on_off_sub = rospy.Subscriber("~lsx/turn_on_off", Bool, self.turnOnOffCb, queue_size = 1)


        self.setIntensityRatioFunction = setIntensityRatioFunction
        if self.setIntensityRatioFunction is not None:
            self.has_intensity_control = True
            set_intensity_ratio_sub = rospy.Subscriber("~lsx/set_intensity_ratio", Float32, self.setIntensityRatioCb, queue_size = 1)

        self.setColorFunction = setColorFunction
        if self.setColorFunction is not None:
            self.has_color_control = True
            if color_options_list != None:
              self.color_options_list = color_options_list
            set_color_sel_sub = rospy.Subscriber("~lsx/set_color", String, self.setColorCb, queue_size = 1)


        self.setKelvinFunction = setKelvinFunction
        if self.setKelvinFunction is not None:
            self.has_kelvin_control = True
            if kelvin_limits_list != None:
              self.kelvin_limits_list = kelvin_limits_list
            set_kelvin_sub = rospy.Subscriber("~lsx/set_kelvin", Int32, self.setKelvinCb, queue_size = 1)


        self.blinkOnOffFunction = blinkOnOffFunction
        if self.blinkOnOffFunction is not None:
            self.has_blink_control = True
            blink_on_off_sub = rospy.Subscriber("~lsx/blink_on_off", Bool, self.blinkOnOffCb, queue_size = 1)
            set_blink_int_sub = rospy.Subscriber("~lsx/set_blink_interval", Float32, self.setBlinkIntervalCb, queue_size = 1)

        self.enableStrobeFunction = enableStrobeFunction
        if self.enableStrobeFunction is not None:
            self.has_hw_strobe = True
            set_strobe_enable_sub = rospy.Subscriber("~lsx/set_strobe_enable", Bool, self.setStrobeEnableCb, queue_size = 1)

        if reports_temp:
          self.reports_temp = True
    
        if reports_power:
          self.reports_power = True



        # Populate and advertise LSX node capabilities report
        self.capabilities_report = LSXCapabilitiesQueryResponse()
        self.capabilities_report.has_standby_mode = self.has_standby_mode
        self.capabilities_report.has_on_off_control = self.has_on_off_control
        self.capabilities_report.has_intensity_control = self.has_intensity_control
        self.capabilities_report.has_color_control = self.has_color_control
        self.capabilities_report.color_options_list = str(self.color_options_list)
        self.capabilities_report.has_kelvin_control = self.has_kelvin_control
        self.capabilities_report.kelvin_min = self.kelvin_limits_list[0]
        self.capabilities_report.kelvin_max = self.kelvin_limits_list[1]
        self.capabilities_report.has_blink_control = self.has_blink_control
        self.capabilities_report.has_hw_strobe = self.has_hw_strobe
        self.capabilities_report.reports_temperature = self.reports_temp
        self.capabilities_report.reports_power = self.reports_power
        rospy.Service("~lsx/capabilities_query", LSXCapabilitiesQuery, self.capabilities_query_callback)
 
 
        # Setup interface classes and update
        self.settings_if = SettingsIF(capSettings, factorySettings, settingUpdateFunction, getSettingsFunction)
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentAsDefault, paramsModifiedCallback=self.updateFromParamServer)

        time.sleep(1)


        # Start additional subscribers
        self.factory_device_name = device_info["device_name"] + "_" + device_info["identifier"]
        self.init_device_name = rospy.get_param('~lsx/device_name', self.factory_device_name)
        rospy.set_param('~lsx/device_name', self.init_device_name)
        rospy.Subscriber('~lsx/update_device_name', String, self.updateDeviceNameCb, queue_size=1) # start local callbac
        rospy.Subscriber('~lsx/reset_device_name', Empty, self.resetDeviceNameCb, queue_size=1) # start local callback

        rospy.Subscriber('~reset_factory', Empty, self.resetFactoryCb, queue_size=1) # start local callback
        rospy.Subscriber('~reset_controls', Empty, self.resetControlsCb, queue_size=1) # start local callback


        # Setup LSX status publishers
        self.getStatusFunction = getStatusFunction
        self.status_pub = rospy.Publisher("~lsx/status", LSXStatus, queue_size=1, latch=True)
        time.sleep(1)
        rospy.Timer(rospy.Duration(1), self.statusTimerCb)

        self.updateFromParamServer()
        self.publish_status()
        ## Initiation Complete
        self.publishMsg("Initialization Complete")
        
        
    def publishMsg(self,msg):
      msg_str = (self.node_name + ": " + str(msg))
      rospy.loginfo(msg_str)
      if self.msg_pub.get_num_connections() > 0:
        self.msg_pub.publish(msg_str)


    
