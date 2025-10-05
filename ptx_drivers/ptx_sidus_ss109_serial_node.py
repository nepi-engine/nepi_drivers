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
import serial
import serial.tools.list_ports
import time
import re
import sys
import inspect
import math
import glob
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_nav

from nepi_api.device_if_ptx import PTXActuatorIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'PTX_SIDUS_SS109_SERIAL' # Use in display menus
FILE_TYPE = 'NODE'


class SidusSS109SerialPTXNode:
    set_speed = False

    MAX_POSITION_UPDATE_RATE = 5
    SERIAL_RECEIVE_DELAY = 0.001

    MIN_SERIAL_SEND_DELAY = 0.01
    MAX_SERIAL_SEND_DELAY = 0.05
    serial_send_delay = 0.0 # Adjusted Atuomatically based on serial_fail_attempts

    MAX_SERIAL_ATTEMPTS=10  # Com Loss Point
    serial_fail_attempts = 0 # Current failed com attempts. Reset on success
    max_serial_fail_attempts = 0 # Tracked Max Attempts
    

    PAN_DEG_DIR = -1
    TILT_DEG_DIR = -1

    LIMITS_DICT = dict()
    LIMITS_DICT['max_pan_hardstop_deg'] = 175
    LIMITS_DICT['min_pan_hardstop_deg'] = -175
    LIMITS_DICT['max_tilt_hardstop_deg'] = 75
    LIMITS_DICT['min_tilt_hardstop_deg'] = -75
    LIMITS_DICT['max_pan_softstop_deg'] = 174
    LIMITS_DICT['min_pan_softstop_deg'] = -174
    LIMITS_DICT['max_tilt_softstop_deg'] = 74
    LIMITS_DICT['min_tilt_softstop_deg'] = -74



    CONFIGS_DICT = {
         'Standard' : {'data_len': 4, 'home':5000, 'deg_per_count':0.0879, 'degpsec_per_count': 0.5, 'max_degpsec': 20},
    }
    config_dict = CONFIGS_DICT['Standard']
    data_len = 4


    CAP_SETTINGS = {
        'None' : {"type":"None","name":"None","options":[""]}
    }

    FACTORY_SETTINGS = {
        'None' : {"type":"None","name":"None","options":[""]}
    }

    FACTORY_SETTINGS_OVERRIDES = dict()

    settingFunctions = {
        'None' : {'get':'getNone', 'set': 'setNone'}
    }


    FACTORY_SETTINGS_OVERRIDES = dict( )

    PT_DIRECTION_POSITIVE = 1
    PT_DIRECTION_NEGATIVE = -1

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
    ptx_if = None

    both_str = '!'
    pan_str = '#'
    tilt_str = '$'


    serial_port = None
    serial_busy = False
    
    connected = False


    self_check_count = 100

    current_position = [0.0,0.0]

    speed_ratio = 0.5

    drv_dict = dict()    


    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
                                                
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

        ##############################  
        # Initialize Class Variables

        # Get required drv driver dict info
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict()) 
        #self.msg_if.pub_warn("Got Drivers_Dict from param server: " + str(self.drv_dict))
        try:
            self.port_str = self.drv_dict['DEVICE_DICT']['device_path'] 
            self.baud_str = self.drv_dict['DEVICE_DICT']['baud_str'] 
            self.baud_int = int(self.baud_str)
            self.addr_str = self.drv_dict['DEVICE_DICT']['addr_str'] 

            system_config = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['system_config']['value']
            if system_config in self.CONFIGS_DICT.keys():
                self.config_dict = self.CONFIGS_DICT[system_config]
            self.data_len = self.config_dict['data_len']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        ################################################  
        self.msg_if.pub_info("Connecting to Device on port " + self.port_str + " with baud " + self.baud_str)
        ### Try and connect to device
        while self.connected == False and self.serial_fail_attempts < 5:
            self.connected = self.connect() 
            if self.connected == False:
                nepi_sdk.sleep(1)
        if self.connected == False:
            self.msg_if.pub_info("Shutting down node")
            self.msg_if.pub_info("Specified serial port not available")
            nepi_sdk.signal_shutdown("Serial port not available")   
        else:
            ################################################
            self.msg_if.pub_info("... Connected!")
            self.dev_info = self.driver_getDeviceInfo()
            self.logDeviceInfo()
            # Initialize settings
            self.cap_settings = self.getCapSettings()
            self.factory_settings = self.getFactorySettings()
              

            # Launch the PTX interface --  this takes care of initializing all the ptx settings from config. file, subscribing and advertising topics and services, etc.
            # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
            self.msg_if.pub_info("Launching NEPI PTX () interface...")
            self.device_info_dict["node_name"] = self.node_name
            if self.node_name.find("_") != -1:
                split_name = self.node_name.rsplit('_', 1)
                self.device_info_dict["device_name"] = split_name[0]
                self.device_info_dict["identifier"] = split_name[1]
            else:
                self.device_info_dict["device_name"] = self.node_name
                self.device_info_dict["identifier"] = ""

            #Factory Control Values 
            self.FACTORY_CONTROLS = {
                'frame_id' : self.node_name + '_frame',
                'pan_joint_name' : self.node_name + '_pan_joint',
                'tilt_joint_name' : self.node_name + '_tilt_joint',
                'reverse_pan_control' : False,
                'reverse_tilt_control' : False,
                'speed_ratio' : 0.5,
                'status_update_rate_hz' : 10
            }
            

            # Initialize settings
            self.cap_settings = self.getCapSettings()
            self.msg_if.pub_info("" +"CAPS SETTINGS")
            #for setting in self.cap_settings:
                #self.msg_if.pub_info("" +setting)
            self.factory_settings = self.getFactorySettings()
            self.msg_if.pub_info("" +"FACTORY SETTINGS")
            #for setting in self.factory_settings:
                #self.msg_if.pub_info("" +setting)

            self.home_pan_deg = 0.0
            self.home_tilt_deg = 0.0
            
            if self.set_speed == True:
                self.speed_control = self.setSpeedRatio
            else:
                self.speed_control = None
            


            self.ptx_if = PTXActuatorIF(device_info = self.device_info_dict, 
                                        capSettings = self.cap_settings,
                                        factorySettings = self.factory_settings,
                                        settingUpdateFunction=self.settingUpdateFunction,
                                        getSettingsFunction=self.getSettings,
                                        factoryControls = self.FACTORY_CONTROLS,
                                        factoryLimits = self.LIMITS_DICT,
                                        stopMovingCb = self.stopMoving,
                                        movePanCb = None, #self.movePan,  # Stop command not working on jog
                                        moveTiltCb = None, #self.moveTilt, # Stop command not working on jog
                                        getSoftLimitsCb = None, #self.getSoftLimits, # 109 does not return response
                                        setSoftLimitsCb = self.setSoftLimits,
                                        getSpeedRatioCb = self.getSpeedRatio,
                                        setSpeedRatioCb = self.speed_control,
                                        getPositionCb = self.getPosition,
                                        gotoPositionCb = self.gotoPosition,
                                        gotoPanPositionCb = self.gotoPanPosition,
                                        gotoTiltPositionCb = self.gotoTiltPosition,
                                        goHomeCb = self.goHome,
                                        setHomePositionCb = self.setHomePosition,
                                        setHomePositionHereCb = self.setHomePositionHere,
                                        getNavPoseCb = self.getNavPoseDict,
                                        navpose_update_rate = self.MAX_POSITION_UPDATE_RATE,
                                        deviceResetCb = self.resetDevice
                                        )
            self.msg_if.pub_info(" ... PTX interface running")

            # Start an ptx activity check process that kills node after some number of failed comms attempts
            self.msg_if.pub_info("Starting an activity check process")
            update_interval = float(1.0) / self.MAX_POSITION_UPDATE_RATE
            nepi_sdk.start_timer_process(update_interval, self.updatePositionHandler, oneshot = True)
            # Initialization Complete
            self.msg_if.pub_info("Initialization Complete")
            #Set up node shutdown
            nepi_sdk.on_shutdown(self.cleanup_actions)
            # Spin forever (until object is detected)
            nepi_sdk.spin()


    def updatePositionHandler(self,timer):
        stime=nepi_utils.get_time()
        self.current_position = self.driver_getPosition(self.current_position)
        #self.msg_if.pub_info("Got current position :" + str(self.current_position))
        gtime = nepi_utils.get_time() - stime
        wait_time = float(1.0) / self.MAX_POSITION_UPDATE_RATE - gtime
        if wait_time > 0.1:
            nepi_sdk.sleep(wait_time)
        nepi_sdk.start_timer_process(0.1, self.updatePositionHandler, oneshot = True)

    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["SerialNum"] + "\n"
        self.msg_if.pub_info(dev_info_string)

       
    def getNavPoseDict(self):
        pan_deg, tilt_deg = self.current_position
        navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
        navpose_dict['has_orientation'] = True
        navpose_dict['time_oreantation'] = nepi_utils.get_time()
        navpose_dict['roll_deg'] = 0.0
        navpose_dict['yaw_deg'] = pan_deg * self.PAN_DEG_DIR
        navpose_dict['pitch_deg'] = tilt_deg * self.TILT_DEG_DIR
        return navpose_dict


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

    global getNone
    def getNone(self):
        success = False
        val = '-999'
        success = True
        return val

    global setNone
    def setNone(self,val):
        success = False

        success = True
        return success


    #######################
    ### PTX IF Functions

    def stopMoving(self):
        self.driver_stopMotion()

    def movePan(self, direction, duration):
        pass
        '''
        axis_str = self.pan_str
        if self.ptx_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.PAN_DEG_DIR
            success = self.driver_jog(axis_str = axis_str, direction = direction)

            if success:
                if duration > 0:
                    nepi_sdk.sleep(time_s)
                    while success == False:
                        self.driver_stopAxisMotion(axis_str = axis_str, direction = direction)
                        nepi_sdk.sleep(self.serial_send_delay)
            '''


    def moveTilt(self, direction, duration):
        pass
        '''
        axis_str = self.tilt_str
        if self.ptx_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.TILT_DEG_DIR
            success = self.driver_jog(axis_str = axis_str, direction = direction)

            if success:
                if duration > 0:
                    nepi_sdk.sleep(time_s)
                    while success == False:
                        self.driver_stopAxisMotion(axis_str = axis_str, direction = direction)
                        nepi_sdk.sleep(self.serial_send_delay)
            '''


    def setSoftLimits(self, min_pan,max_pan,min_tilt,max_tilt):
        # TODO: Limits checking and driver unit conversion?
        self.driver_setSoftLimits(min_pan,max_pan,min_tilt,max_tilt)

    def getSoftLimits(self):
        # TODO: Driver unit conversion?
        [min_pan,max_pan,min_tilt,max_tilt] = self.driver_getSoftLimits()
        soft_limits = [min_pan,max_pan,min_tilt,max_tilt]
        return soft_limits




    def setSpeedRatio(self, ratio):
        # TODO: Limits checking and driver unit conversion?
        self.speed_ratio = ratio
        self.driver_setSpeedRatios(ratio)


    def getSpeedRatio(self):
        # TODO: Driver unit conversion?
        ratio = self.driver_getSpeedRatio()
        return ratio
          

    def getPosition(self):
        return self.current_position
        

    def gotoPosition(self, pan_deg, tilt_deg):
        self.driver_moveToPosition(pan_deg * self.PAN_DEG_DIR, tilt_deg * self.TILT_DEG_DIR)

    def gotoPanPosition(self, pan_deg):
        #self.msg_if.pub_warn("gotoPanPosition: " + str(pan_deg) + " direction: " + str(self.PAN_DEG_DIR))
        self.driver_moveToPanPosition(pan_deg * self.PAN_DEG_DIR)

    def gotoTiltPosition(self, tilt_deg):
        #self.msg_if.pub_warn("gotoTiltPosition: " + str(tilt_deg) + " direction: " + str(self.TILT_DEG_DIR))
        self.driver_moveToTiltPosition(tilt_deg * self.TILT_DEG_DIR)
        
    def goHome(self):
        self.driver_moveToPosition(self.home_pan_deg, self.home_tilt_deg)

    def setHomePosition(self, pan_deg, tilt_deg):
        self.home_pan_deg = pan_deg * self.PAN_DEG_DIR
        self.home_tilt_deg = tilt_deg * self.TILT_DEG_DIR

    def setHomePositionHere(self):
        if self.driver_reportsPosition() is True:
            pan_deg, tilt_deg = self.getPosition()
            self.home_pan_deg = pan_deg * self.PAN_DEG_DIR
            self.home_tilt_deg = tilt_deg * self.TILT_DEG_DIR 



    def resetDevice(self):
        self.driver_resetDevice()


   #######################
    ### Driver Interface Functions

    def driver_getDeviceInfo(self):
        method_name = sys._getframe().f_code.co_name
        dev_info = dict()
        dev_info["Manufacturer"] = 'Sidus'
        dev_info["Model"] = 'SS109'


        data_str = self.create_blank_str()
        ser_msg= (self.pan_str + self.addr_str + 'MRV' + data_str + 'R')
        [success,response] = self.send_msg(ser_msg)

        firmware = ""
        if success:
            firmware = response[5:8]
        dev_info["FirmwareVersion"] = firmware

        dev_info["SerialNum"] = ""
        return dev_info

               

    def driver_getSoftLimit(self,axis_str = '#', direction = 1):
        method_name = sys._getframe().f_code.co_name
        softLimit = -999
        success = False
        data_str = self.create_blank_str()
        if direction > 0:
            dir_str = 'MRF'
        else:
            dir_str = 'MRB'
        if axis_str == self.pan_str:
            ser_msg= (self.pan_str + self.addr_str + dir_str + data_str + 'W')
        elif axis_str == self.tilt_str:
            ser_msg= (self.tilt_str + self.addr_str + dir_str + data_str + 'W')
        elif axis_str == self.both_str:
            ser_msg= (self.both_str + self.addr_str + dir_str + data_str + 'W')
        else:
            return False
        self.msg_if.pub_warn(method_name + ": Sending Get Soft Stop serial msg: " + ser_msg)
        [success,response] = self.send_msg(ser_msg)

        if success:
            try:
                data_str = response[5:(5 + self.data_len)]
                self.msg_if.pub_warn(method_name + ": Will convert soft limit data str: " + data_str)
                pos_count = int(data_str)
                softLimit = self.pos_count2deg(pos_count)
                success = True
            except Exception as e:
                self.msg_if.pub_warn(method_name + ": Failed to convert message to int: " + data_str + " " + str(e))
        return softLimit

    def driver_getSoftLimits(self): 
        method_name = sys._getframe().f_code.co_name
        min_pan = self.driver_getSoftLimit(axis_str = self.pan_str, direction = -1)
        nepi_sdk.sleep(self.serial_send_delay)
        max_pan = self.driver_getSoftLimit(axis_str = self.pan_str, direction = 1)
        nepi_sdk.sleep(self.serial_send_delay)
        min_tilt = self.driver_getSoftLimit(axis_str = self.tilt_str, direction = -1)
        nepi_sdk.sleep(self.serial_send_delay)
        max_tilt = self.driver_getSoftLimit(axis_str = self.tilt_str, direction = 1)
        return [min_pan,max_pan,min_tilt,max_tilt] 

    def driver_setSoftLimit(self, limit_deg, axis_str = '#', direction = 1):
        method_name = sys._getframe().f_code.co_name
        success = False
        self
        pos_count = self.deg2pos_count(limit_deg)
        data_str = self.create_pos_str(pos_count)
        if direction > 0:
            dir_str = 'MLF'
        else:
            dir_str = 'MLB'
        if axis_str == self.pan_str:
            ser_msg= (self.pan_str + self.addr_str + dir_str + data_str + 'W')
        elif axis_str == self.tilt_str:
            ser_msg= (self.tilt_str + self.addr_str + dir_str + data_str + 'W')
        elif axis_str == self.both_str:
            ser_msg= (self.both_str + self.addr_str + dir_str + data_str + 'W')
        else:
            return False
        self.msg_if.pub_warn(method_name + ": Sending Set Soft Stop serial msg: " + ser_msg)
        [success,response] = self.send_msg(ser_msg)  
        return success 

    def driver_setSoftLimits(self, min_pan,max_pan,min_tilt,max_tilt): 
        method_name = sys._getframe().f_code.co_name
        success_list = []
        nepi_sdk.sleep(self.serial_send_delay)
        success_list.append(self.driver_setSoftLimit(min_pan, axis_str = self.pan_str, direction = -1))
        nepi_sdk.sleep(self.serial_send_delay)
        success_list.append(self.driver_setSoftLimit(max_pan, axis_str = self.pan_str, direction = 1))
        nepi_sdk.sleep(self.serial_send_delay)
        success_list.append(self.driver_setSoftLimit(min_tilt, axis_str = self.tilt_str, direction = -1))
        nepi_sdk.sleep(self.serial_send_delay)
        success_list.append(self.driver_setSoftLimit(max_tilt, axis_str = self.tilt_str, direction = 1))
        return False not in success_list



    def driver_getSpeedRatio(self, axis_str = '#'):
        method_name = sys._getframe().f_code.co_name
        speedRatio = 0.5
        success = False
        data_str = self.create_blank_str()
        ser_msg= (axis_str + self.addr_str + 'MRS' + data_str + 'R')
        [success,response] = self.send_msg(ser_msg)

        if success:
            try:
                data_str = response[5:(5 + self.data_len)]
                self.msg_if.pub_warn(method_name + ": Will convert speed str: " + data_str)
                speed_count = int(data_str)
                speedRatio = self.speed_count2ratio(speed_count)
            except Exception as e:
                self.msg_if.pub_warn(method_name + ": Failed to convert message to int: " + data_str + " " + str(e))

        return speedRatio

    def driver_setSpeedRatios(self,speedRatio):
        method_name = sys._getframe().f_code.co_name
        success_list = []
        success_list.append(self.driver_setSpeedRatio(speedRatio, axis_str = self.pan_str))
        nepi_sdk.sleep(self.serial_send_delay)
        success_list.append(self.driver_setSpeedRatio(speedRatio, axis_str = self.tilt_str))
        return False not in success_list

    def driver_setSpeedRatio(self,speedRatio, axis_str = '#'):
        success = False
        try:
            speed_count = self.ratio2speed_count(speedRatio)
        except Exception as e:
            self.msg_if.pub_warn(method_name + ": Failed to convert message: " + self.speedRatio + " " + str(e))
            return False

        data_str = self.create_speed_str(speed_count)
        ser_msg= (axis_str  + self.addr_str + 'MSP' + data_str + 'W')
        [success,response] = self.send_msg(ser_msg)
        return success

    def driver_resetDevice(self):
        success = False
        data_str = self.create_blank_str()
        ser_msg= (self.tilt_str  + self.addr_str + 'MFR' + data_str + 'W')
        [success,response] = self.send_msg(ser_msg)

        nepi_sdk.sleep(self.serial_send_delay)
        data_str = self.create_blank_str()
        ser_msg= (self.pan_str  + self.addr_str + 'MFR' + data_str + 'W')
        [success,response] = self.send_msg(ser_msg)

        return success




    def driver_reportsPosition(self):
        method_name = sys._getframe().f_code.co_name
        reportsPos = True
        return reportsPos

    def driver_getPosition(self, wait_on_busy = False, verbose = False):
        caller_method = inspect.currentframe().f_back.f_code.co_name
        method_name = sys._getframe().f_code.co_name
        pan_deg = self.getCurrentPanPosition()
        '''
        while pan_deg < -360:
            if verbose == True:
                self.msg_if.pub_warn(caller_method + ": " + method_name + ": Failed to get valid pan degs: " + str(pan_deg))
            nepi_sdk.sleep(self.serial_send_delay)
            pan_deg_r = self.getCurrentPanPosition(wait_on_busy = wait_on_busy, verbose = verbose)
            if pan_deg_r > -360:
                pan_deg = pan_deg_r
        '''
        if verbose == True:
            self.msg_if.pub_warn(caller_method + ": " + method_name + ": Got pan degs: " + str(pan_deg))


        nepi_sdk.sleep(self.serial_send_delay)

        tilt_deg = self.getCurrentTiltPosition()
        '''
        while tilt_deg < -360:
            if verbose == True:
                self.msg_if.pub_warn(caller_method + ": " + method_name + ": Failed to get valid tilt degs: " + str(tilt_deg))
            nepi_sdk.sleep(self.serial_send_delay)
            tilt_deg_r = self.getCurrentTiltPosition(wait_on_busy = wait_on_busy, verbose = verbose)
            if tilt_deg_r > -360:
                tilt_deg = tilt_deg_r
        '''
        if verbose == True:
            self.msg_if.pub_warn(caller_method + ": " + method_name + ": Got tilt degs: " + str(tilt_deg))
        return pan_deg, tilt_deg

        


    def getCurrentPanPosition(self, wait_on_busy = False, verbose = False):
        method_name = sys._getframe().f_code.co_name
        pan_deg = self.current_position[0]
        success = False
        data_str = self.create_blank_str()
        ser_msg= (self.pan_str + self.addr_str + 'MRL' + data_str + 'R')
        [success,response] = self.send_msg(ser_msg, wait_on_busy = wait_on_busy, verbose = verbose)

        if success == True:
            try:
                data_str = response[5:(5 + self.data_len)]
                #self.msg_if.pub_warn(method_name + ": Will convert pan position str: " + data_str)
                pan_count = int(data_str) 
                pan_deg = self.pos_count2deg(pan_count) * -1
            except Exception as e:
                self.msg_if.pub_warn(method_name + ": Failed to convert message to int: " + data_str + " " + str(e))

        return pan_deg


    def getCurrentTiltPosition(self, wait_on_busy = False, verbose = False):
        method_name = sys._getframe().f_code.co_name
        tilt_deg = self.current_position[1]
        success = False
        data_str = self.create_blank_str()
        ser_msg= (self.tilt_str + self.addr_str + 'MRL' + data_str + 'R')
        [success,response] = self.send_msg(ser_msg, wait_on_busy = wait_on_busy, verbose = verbose)

        if success:
            try:
                data_str = response[5:(5 + self.data_len)]
                #self.msg_if.pub_warn(method_name + ": Will convert tilt position str: " + data_str)
                tilt_count = int(data_str)
                tilt_deg = self.pos_count2deg(tilt_count) *-1
            except Exception as e:
                self.msg_if.pub_warn(method_name + ": Failed to convert message to int: " + data_str + " " + str(e))

        return tilt_deg


    def driver_moveToPosition(self,pan_deg, tilt_deg):
        success = self.driver_moveToPanPosition(pan_deg)
        success = self.driver_moveToTiltPosition(tilt_deg)
        self.msg_if.pub_warn("driver_moveToPosition: " + str(success))

        return success


    def driver_moveToPanPosition(self,pan_deg):

        method_name = sys._getframe().f_code.co_name
        success = False
        pos_count = self.deg2pos_count(pan_deg)
        data_str = self.create_pos_str(pos_count)
        ser_msg= (self.pan_str + self.addr_str + 'MML' + data_str + 'W')
        #self.msg_if.pub_warn(" Will send move to pan pos with pos_count: " + str(pos_count) + "data_str: " + str(data_str))
        #self.msg_if.pub_warn("ser_msg: " + str(ser_msg))
        [success,response] = self.send_msg(ser_msg)

        if self.set_speed == True:
            #self.msg_if.pub_warn("driver_setSpeedRatio called")
            #self.msg_if.pub_warn("driver_setSpeedRatio: " + str(self.speed_ratio))
            nepi_sdk.sleep(self.serial_send_delay)
            self.driver_setSpeedRatio(self.speed_ratio, axis_str = self.pan_str)

        
        return success


    def driver_moveToTiltPosition(self, tilt_deg):
        method_name = sys._getframe().f_code.co_name
        success = False
        pos_count = self.deg2pos_count(tilt_deg)
        data_str = self.create_pos_str(pos_count)
        ser_msg= (self.tilt_str + self.addr_str + 'MML' + data_str + 'W')
        self.msg_if.pub_warn("pos_count: " + str(pos_count) + "data_str: " + str(data_str))
        self.msg_if.pub_warn("ser_msg: " + str(ser_msg))
        [success,response] = self.send_msg(ser_msg)

        if self.set_speed == True:
            #self.msg_if.pub_warn("driver_setSpeedRatio called")
            #self.msg_if.pub_warn("driver_setSpeedRatio: " + str(self.speed_ratio))
            nepi_sdk.sleep(self.serial_send_delay)
            self.driver_setSpeedRatio(self.speed_ratio, axis_str = self.pan_str)


        return success


    def driver_stopMotion(self):
        method_name = sys._getframe().f_code.co_name
        self.driver_stopAxisMotion(axis_str = self.both_str)


    def driver_stopAxisMotion(self, axis_str = '#'):
        method_name = sys._getframe().f_code.co_name
        success = False
        data_str = self.create_blank_str()
        if axis_str == self.pan_str:
            ser_msg= (self.pan_str + self.addr_str + 'MST' + data_str + 'W')
            self.msg_if.pub_warn(method_name + ": Sending Stop Pan serial msg: " + ser_msg)
        elif axis_str == self.tilt_str:
            ser_msg= (self.tilt_str + self.addr_str + 'MST' + data_str + 'W')
            self.msg_if.pub_warn(method_name + ": Sending Stop Tilt serial msg: " + ser_msg)
        elif axis_str == self.both_str:
            ser_msg= (self.pan_str + self.addr_str + 'MST' + data_str + 'W')
            self.msg_if.pub_warn(method_name + ": Sending Stop Pan serial msg: " + ser_msg)

            nepi_sdk.sleep(self.serial_send_delay)

            ser_msg= (self.tilt_str + self.addr_str + 'MST' + data_str + 'W')
            self.msg_if.pub_warn(method_name + ": Sending Stop Tilt serial msg: " + ser_msg)



        else:
            return False

        [success,response] = self.send_msg(ser_msg)
        return success 


    def driver_jog(self,axis_str, direction):


        #self.driver_setSpeedRatios(self.speed_ratio)

        #nepi_sdk.sleep(self.serial_send_delay)

        method_name = sys._getframe().f_code.co_name
        success = False
        data_str = self.create_blank_str()
        if direction == 1:
            cmd_str = 'MMF'
        else:
            cmd_str = 'MMB'
        ser_msg= (axis_str + self.addr_str + cmd_str + data_str + 'W')
        [success,response] = self.send_msg(ser_msg)
        return success

    #######################
    ### Driver Util Functions



    
    ### Function to try and connect to device at given port and baudrate
    def connect(self):
        success = False
        port_check = self.check_port(self.port_str)
        if port_check is True:
            try:
                # Try and open serial port
                self.msg_if.pub_info("Opening serial port " + self.port_str + " with baudrate: " + self.baud_str)
                self.serial_port = serial.Serial(self.port_str,self.baud_int,timeout = 1)
                self.msg_if.pub_info("Serial port opened")
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Something went wrong with connecting to serial port at: " + self.port_str + "(" + str(e) + ")" )
            if success == True:
                success = False
                response = ""
                # Send Message
                self.msg_if.pub_info("Requesting info for device: " + self.addr_str)
                # Test message
                data_str = self.create_blank_str()
                ser_msg= (self.pan_str + self.addr_str + 'MRA' + data_str + 'R')
                self.msg_if.pub_warn("Sending serial string: " + ser_msg)
                [success,response] = self.send_msg(ser_msg)

                if success:
                    self.msg_if.pub_info("Connected to device at address: " +  self.addr_str)
                    # Update serial, hardware, and software status values
                    self.serial_num = "unknown"
                    self.hw_version = "unknown"
                    self.sw_version = "unknown"
                    success = True

                    # Factory Reset Device
                    nepi_sdk.sleep(self.serial_send_delay)
                    reset_success = self.driver_resetDevice()

            else:
                self.msg_if.pub_warn("serial port not active")
        return success


    def send_msg(self,ser_msg, wait_on_busy = True, verbose = True):
        caller_method = inspect.currentframe().f_back.f_code.co_name
        success = False
        response = "-999"
        if self.serial_port is not None:
            if self.serial_busy == True and wait_on_busy == True:
                while self.serial_busy == True:
                    time.sleep(self.SERIAL_RECEIVE_DELAY/4)
                time.sleep(self.SERIAL_RECEIVE_DELAY)                

            if self.serial_busy == False:
                self.serial_busy = True

                if verbose == True:
                    #self.msg_if.pub_warn(caller_method + ": send_msg: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
                    #self.msg_if.pub_warn(caller_method + ": send_msg: Locked serial with send msg: " + ser_msg)
                    pass
                ser_str = (ser_msg + '\r\n')
                b=bytearray()
                b.extend(map(ord, ser_str))
                try:
                    #self.msg_if.pub_warn(caller_method + ": send_msg: Sending message " + str(ser_str))
                    self.serial_port.write(b)
                except Exception as e:
                    self.msg_if.pub_warn(caller_method + ": send_msg: Failed to send message " + str(e))
                time.sleep(self.SERIAL_RECEIVE_DELAY)
                try:
                    bs = self.serial_port.readline()
                    response = bs.decode()
                    #if verbose == True:
                    #    self.msg_if.pub_warn(caller_method + ": send_msg: Device returned: " + str(response) + " for: " +  ser_str)
                except Exception as e:
                    self.msg_if.pub_warn(caller_method + ": send_msg: Failed to recieve message " + str(e))
                if verbose == True:
                    #self.msg_if.pub_warn(caller_method + ": send_msg: Unlocking serial")
                    #self.msg_if.pub_warn(caller_method + ": send_msg: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                    pass
                success = self.check_valid_response(ser_msg,response)
                if success == False:
                    # Try again
                    try:
                        bs = self.serial_port.readline()
                        response = bs.decode()
                        if verbose == True:
                            self.msg_if.pub_debug(caller_method + ": send_msg: Fialed - Device returned: " + str(response) + " for: " +  ser_str)
                    except Exception as e:
                        self.msg_if.pub_warn(caller_method + ": send_msg: Failed to recieve message " + str(e))
                    if verbose == True:
                        #self.msg_if.pub_warn(caller_method + ": send_msg: Unlocking serial")
                        #self.msg_if.pub_warn(caller_method + ": send_msg: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                        pass


                    self.serial_fail_attempts += 1
                    #self.msg_if.pub_warn(caller_method + ": send_msg: Updated serial_fail_attempts to " + str(self.serial_fail_attempts))
                    #self.msg_if.pub_warn("serial_fail_attempts: " +  str(self.serial_fail_attempts))
                    if self.serial_fail_attempts > self.max_serial_fail_attempts:
                        self.max_serial_fail_attempts = copy.deepcopy(self.serial_fail_attempts)
                        self.serial_send_delay = self.MIN_SERIAL_SEND_DELAY + self.MAX_SERIAL_SEND_DELAY * (self.max_serial_fail_attempts / self.MAX_SERIAL_ATTEMPTS)
                        self.msg_if.pub_warn("Serial send delay updated to : " +  str(self.serial_send_delay))
                    if self.serial_fail_attempts > self.MAX_SERIAL_ATTEMPTS:
                        nepi_sdk.signal_shutdown("Exceeded Max Serial Fail attempts in a row, Shutting Down")
                else:
                    self.serial_fail_attempts = 0


                self.serial_busy = False
            else:
                self.msg_if.pub_warn(caller_method + ": Serial port busy, can't send msg: " + ser_msg)
        
        
    
        return [success, response]


    def check_valid_response(self,ser_msg, response):
        caller_method = inspect.currentframe().f_back.f_code.co_name
        valid = False
        if len(response) >= 5 + self.data_len + 1:
            if response[0:4] == ser_msg[0:4]:
                valid = True
        if valid == False:
            pass
            #self.msg_if.pub_warn(caller_method + ": Failed to get valid response message from: " + ser_msg + " : " + str(response))
        return valid




    ### Function for checking if port is available
    def check_port(self,port_str):
        success = False
            # Try pyserial first
        ports = list(serial.tools.list_ports.comports())
        add_ports = sorted(set(
                glob.glob('/dev/ttyTHS0')
            ))
        for add_port in add_ports:
            if add_port not in ports:
                ports.append(add_port)
        self.msg_if.pub_warn("Node Port Check: " + str(ports))
        for p in ports:
            loc = getattr(p, 'device', p)
            if loc == port_str:
                success = True
        return success


    def pos_count2deg(self, count):
        dpc = self.config_dict['deg_per_count'] 
        home = self.config_dict['home'] 
        deg = float(count - home)*dpc  
        return deg

    def deg2pos_count(self, deg):
        dpc = self.config_dict['deg_per_count']
        home = self.config_dict['home'] 
        count = int(deg/dpc + home)
        return count


    def speed_count2dps(self, count):
        max_dps = self.config_dict['max_degpsec']
        dps = max
        dps = float(pos - home)*dps  
        return dps

    def dps2speed_count(self, deg):
        dps_per_count = self.config_dict['degpsec_per_count']
        count = floor(deg / dps_per_count)
        return count

    def ratio2speed_count(self,ratio):
        max_count =  math.floor(self.config_dict['max_degpsec'] / self.config_dict['degpsec_per_count'])
        count = int(ratio * max_count)
        return count

    def speed_count2ratio(self,count):
        max_count =  math.floor(self.config_dict['max_degpsec'] / self.config_dict['degpsec_per_count'])
        ratio = float(count/max_count)
        return ratio

    def create_blank_str(self):
        data_str = ""
        zero_prefix_len = self.data_len-len(data_str)
        for z in range(self.data_len):
            data_str += '0'
        return data_str

    def create_pos_str(self,count_val):
        data_str = str(count_val)
        zero_prefix_len = self.data_len-len(data_str)
        for z in range(zero_prefix_len):
            data_str = ('0' + data_str)
        return data_str

    def create_speed_str(self,count_val):
        data_str = str(count_val)
        zero_suffix_len = self.data_len-len(data_str)
        for z in range(zero_suffix_len):
            data_str = (data_str + '0')
        return data_str


    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        if self.serial_port is not None:
            self.serial_port.close()


if __name__ == '__main__':
	node = SidusSS109SerialPTXNode()
