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

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings

from nepi_api.device_if_ptx import PTXActuatorIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'PTX_KIST_KTP20' # Use in display menus
FILE_TYPE = 'NODE'


class SidusSS109SerialPTXNode:
    MAX_POSITION_UPDATE_RATE = 5
    SERIAL_RECEIVE_DELAY = 0.03
    SERIAL_SEND_DELAY = 0.5
    HEARTBEAT_CHECK_INTERVAL = 1.0
    DEG_DIR = -1

    LIMITS_DICT = dict()
    LIMITS_DICT['max_yaw_hardstop_deg'] = 175
    LIMITS_DICT['min_yaw_hardstop_deg'] = -175
    LIMITS_DICT['max_pitch_hardstop_deg'] = 75
    LIMITS_DICT['min_pitch_hardstop_deg'] = -75
    LIMITS_DICT['max_yaw_softstop_deg'] = 174
    LIMITS_DICT['min_yaw_softstop_deg'] = -174
    LIMITS_DICT['max_pitch_softstop_deg'] = 74
    LIMITS_DICT['min_pitch_softstop_deg'] = -74



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
    connect_attempts = 0
    connected = False


    self_check_count = 10
    self_check_counter = 0

    drv_dict = dict()    


    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
                                                
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
            nepi_ros.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        ################################################  
        self.msg_if.pub_info("Connecting to Device on port " + self.port_str + " with baud " + self.baud_str)
        ### Try and connect to device
        while self.connected == False and self.connect_attempts < 5:
            self.connected = self.connect() 
            if self.connected == False:
                nepi_ros.sleep(1)
        if self.connected == False:
            self.msg_if.pub_info("Shutting down node")
            self.msg_if.pub_info("Specified serial port not available")
            nepi_ros.signal_shutdown("Serial port not available")   
        else:
            ################################################
            self.msg_if.pub_info("... Connected!")
            self.dev_info = self.driver_getDeviceInfo()
            self.logDeviceInfo()
            # Initialize settings
            self.cap_settings = self.getCapSettings()
            self.factory_settings = self.getFactorySettings()


            # Must pass a capabilities structure to ptx_interface constructor
            ptx_capabilities_dict = {}
            ptx_capabilities_dict['has_absolute_positioning'] = True
            ptx_capabilities_dict['has_limit_control'] = True
            ptx_capabilities_dict['has_speed_control'] = True
            ptx_capabilities_dict['has_homing'] = True
            ptx_capabilities_dict['has_waypoints'] = True
                

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
                'yaw_joint_name' : self.node_name + '_yaw_joint',
                'pitch_joint_name' : self.node_name + '_pitch_joint',
                'reverse_yaw_control' : False,
                'reverse_pitch_control' : False,
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

            self.home_yaw_deg = 0.0
            self.home_pitch_deg = 0.0
            self.waypoints = {} # Dictionary of dictionaries with numerical key and {waypoint_pitch, waypoint_yaw} dict value


            self.ptx_if = PTXActuatorIF(device_info = self.device_info_dict, 
                                        capSettings = self.cap_settings,
                                        factorySettings = self.factory_settings,
                                        settingUpdateFunction=self.settingUpdateFunction,
                                        getSettingsFunction=self.getSettings,
                                        factoryControls = self.FACTORY_CONTROLS,
                                        factoryLimits = self.LIMITS_DICT,
                                        capabilities_dict = ptx_capabilities_dict,
                                        stopMovingCb = self.stopMoving,
                                        moveYawCb = self.moveYaw,
                                        movePitchCb = self.movePitch,
                                        setSoftLimitsCb = self.setSoftLimits,
                                        getSoftLimitsCb = None, #self.getSoftLimits, # 109 does not return response
                                        setSpeedCb = self.setSpeed,
                                        getSpeedCb = self.getSpeed,
                                        gotoPositionCb = self.gotoPosition,
                                        goHomeCb = self.goHome,
                                        setHomePositionCb = self.setHomePosition,
                                        setHomePositionHereCb = self.setHomePositionHere,
                                        gotoWaypointCb = self.gotoWaypoint,
                                        setWaypointCb = self.setWaypoint,
                                        setWaypointHereCb = self.setWaypointHere,
                                        getHeadingCb = None, getPositionCb = None, getOrientationCb = self.getOrientationCb,
                                        getLocationCb = None, getAltitudeCb = None, getDepthCb = None,
                                        max_navpose_update_rate = self.MAX_POSITION_UPDATE_RATE,
                                        deviceResetCb = self.resetDevice
                                        )
            self.msg_if.pub_info(" ... PTX interface running")

            # Start an ptx activity check process that kills node after some number of failed comms attempts
            self.msg_if.pub_info("Starting an activity check process")
            #nepi_ros.start_timer_process(self.HEARTBEAT_CHECK_INTERVAL, self.check_timer_callback)
            # Initialization Complete
            self.msg_if.pub_info("Initialization Complete")
            #Set up node shutdown
            nepi_ros.on_shutdown(self.cleanup_actions)
            # Spin forever (until object is detected)
            nepi_ros.spin()




    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["SerialNum"] + "\n"
        self.msg_if.pub_info(dev_info_string)

    def getOrientationCb(self):
        yaw_deg, pitch_deg = self.getCurrentPosition()
        orientation_dict = dict()
        orientation_dict['time_oreantation'] = nepi_utils.get_time()
        orientation_dict['roll_deg'] = 0.0
        orientation_dict['pitch_deg'] = pitch_deg * self.DEG_DIR
        orientation_dict['yaw_deg'] = yaw_deg * self.DEG_DIR
        return orientation_dict


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

    def moveYaw(self, direction, duration):
        pass
        '''
        axis_str = self.pan_str
        if self.ptx_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.DEG_DIR
            success = self.driver_jog(axis_str = axis_str, direction = direction)

            if success:
                if duration > 0:
                    nepi_ros.sleep(time_s)
                    while success == False:
                        self.driver_stopAxisMotion(axis_str = axis_str, direction = direction)
                        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
            '''


    def movePitch(self, direction, duration):
            '''
        axis_str = self.tilt_str
        if self.ptx_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.DEG_DIR
            success = self.driver_jog(axis_str = axis_str, direction = direction)

            if success:
                if duration > 0:
                    nepi_ros.sleep(time_s)
                    while success == False:
                        self.driver_stopAxisMotion(axis_str = axis_str, direction = direction)
                        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
            '''


    def setSoftLimits(self, min_yaw,max_yaw,min_pitch,max_pitch):
        # TODO: Limits checking and driver unit conversion?
        self.driver_setSoftLimits(min_yaw,max_yaw,min_pitch,max_pitch)

    def getSoftLimits(self):
        # TODO: Driver unit conversion?
        [min_yaw,max_yaw,min_pitch,max_pitch] = self.driver_getSoftLimits()
        soft_limits = [min_yaw,max_yaw,min_pitch,max_pitch]
        return soft_limits




    def setSpeed(self, ratio):
        # TODO: Limits checking and driver unit conversion?
        self.driver_setSpeedRatios(ratio)

    def getSpeed(self):
        # TODO: Driver unit conversion?
        ratio = self.driver_getSpeedRatio()
        return ratio
          

    def getCurrentPosition(self):
        yaw_deg, pitch_deg = self.driver_getCurrentPosition(wait_on_busy = False, verbose = False)
        #self.msg_if.pub_warn("Got pos degs : " + str([pan_deg, tilt_deg]))
        return yaw_deg, pitch_deg
        

    def gotoPosition(self, yaw_deg, pitch_deg):
        self.driver_moveToPosition(yaw_deg * self.DEG_DIR, pitch_deg * self.DEG_DIR)
        
    def goHome(self):
        self.driver_moveToPosition(self.home_yaw_deg, self.home_pitch_deg)

    def setHomePosition(self, yaw_deg, pitch_deg):
        self.home_yaw_deg = yaw_deg
        self.home_pitch_deg = pitch_deg

    def setHomePositionHere(self):
        if self.driver_reportsPosition() is True:
            yaw_deg, pitch_deg = self.driver_getCurrentPosition(wait_on_busy = True, verbose = True)
            self.home_yaw_deg = yaw_deg * self.DEG_DIR
            self.home_pitch_deg = pitch_deg * self.DEG_DIR 

    def gotoWaypoint(self, waypoint_index):
        if waypoint_index not in self.waypoints:
            return
        waypoint_yaw_deg = self.waypoints[waypoint_index]['yaw_deg']
        waypoint_pitch_deg = self.waypoints[waypoint_index]['pitch_deg']
        self.driver_moveToPosition(waypoint_yaw_deg, waypoint_pitch_deg)
    
    def setWaypoint(self, waypoint_index, yaw_deg, pitch_deg):
        self.waypoints[waypoint_index] = {'yaw_deg': yaw_deg * self.DEG_DIR, 'pitch_deg': pitch_deg * self.DEG_DIR}
        
    def setWaypointHere(self, waypoint_index):
        if self.driver_reportsPosition() and self.ptx_if is not None:
            yaw_deg, pitch_deg = self.driver_getCurrentPosition(wait_on_busy = True, verbose = True)
            self.waypoints[waypoint_index] = {'yaw_deg': yaw_deg, 'pitch_deg': pitch_deg}

        self.msg_if.pub_info("Waypoint set to current position")

    def setCurrentSettingsAsDefault(self):
        # Don't need to worry about any of our params in this class, just child interfaces' params
        if self.ptx_if is not None:
            self.ptx_if.initConfig()

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
        min_yaw = self.driver_getSoftLimit(axis_str = self.pan_str, direction = -1)
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
        max_yaw = self.driver_getSoftLimit(axis_str = self.pan_str, direction = 1)
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
        min_pitch = self.driver_getSoftLimit(axis_str = self.tilt_str, direction = -1)
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
        max_pitch = self.driver_getSoftLimit(axis_str = self.tilt_str, direction = 1)
        return [min_yaw,max_yaw,min_pitch,max_pitch] 

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

    def driver_setSoftLimits(self, min_yaw,max_yaw,min_pitch,max_pitch): 
        method_name = sys._getframe().f_code.co_name
        success_list = []
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
        success_list.append(self.driver_setSoftLimit(min_yaw, axis_str = self.pan_str, direction = -1))
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
        success_list.append(self.driver_setSoftLimit(max_yaw, axis_str = self.pan_str, direction = 1))
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
        success_list.append(self.driver_setSoftLimit(min_pitch, axis_str = self.tilt_str, direction = -1))
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
        success_list.append(self.driver_setSoftLimit(max_pitch, axis_str = self.tilt_str, direction = 1))
        return False not in success_list



    def driver_getSpeedRatio(self, axis_str = '#'):
        method_name = sys._getframe().f_code.co_name
        speedRatio = -999
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
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
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

        nepi_ros.sleep(self.SERIAL_SEND_DELAY)
        data_str = self.create_blank_str()
        ser_msg= (self.pan_str  + self.addr_str + 'MFR' + data_str + 'W')
        [success,response] = self.send_msg(ser_msg)

        return success




    def driver_reportsPosition(self):
        method_name = sys._getframe().f_code.co_name
        reportsPos = True
        return reportsPos

    def driver_getCurrentPosition(self, wait_on_busy = False, verbose = False):
        caller_method = inspect.currentframe().f_back.f_code.co_name
        method_name = sys._getframe().f_code.co_name
        yaw_deg = self.getCurrentPanPosition()
        while yaw_deg < -360:
            if verbose == True:
                self.msg_if.pub_warn(caller_method + ": " + method_name + ": Failed to get valid pan degs: " + str(yaw_deg))
            nepi_ros.sleep(self.SERIAL_SEND_DELAY)
            yaw_deg_r = self.getCurrentPanPosition(wait_on_busy = wait_on_busy, verbose = verbose)
            if yaw_deg_r > -360:
                yaw_deg = yaw_deg_r
        if verbose == True:
            self.msg_if.pub_warn(caller_method + ": " + method_name + ": Got pan degs: " + str(yaw_deg))
                
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)

        pitch_deg = self.getCurrentTiltPosition()
        while pitch_deg < -360:
            if verbose == True:
                self.msg_if.pub_warn(caller_method + ": " + method_name + ": Failed to get valid tilt degs: " + str(pitch_deg))
            nepi_ros.sleep(self.SERIAL_SEND_DELAY)
            pitch_deg_r = self.getCurrentTiltPosition(wait_on_busy = wait_on_busy, verbose = verbose)
            if pitch_deg_r > -360:
                pitch_deg = pitch_deg_r
        if verbose == True:
            self.msg_if.pub_warn(caller_method + ": " + method_name + ": Got tilt degs: " + str(pitch_deg))
        return yaw_deg, pitch_deg

        


    def getCurrentPanPosition(self, wait_on_busy = False, verbose = False):
        method_name = sys._getframe().f_code.co_name
        yaw_deg = -999
        success = False
        data_str = self.create_blank_str()
        ser_msg= (self.pan_str + self.addr_str + 'MRL' + data_str + 'R')
        [success,response] = self.send_msg(ser_msg, wait_on_busy = wait_on_busy, verbose = verbose)

        if success == True:
            try:
                data_str = response[5:(5 + self.data_len)]
                #self.msg_if.pub_warn(method_name + ": Will convert pan position str: " + data_str)
                yaw_count = int(data_str)
                yaw_deg = self.pos_count2deg(yaw_count)
            except Exception as e:
                self.msg_if.pub_warn(method_name + ": Failed to convert message to int: " + data_str + " " + str(e))

        return yaw_deg


    def getCurrentTiltPosition(self, wait_on_busy = False, verbose = False):
        method_name = sys._getframe().f_code.co_name
        pitch_deg = -999
        success = False
        data_str = self.create_blank_str()
        ser_msg= (self.tilt_str + self.addr_str + 'MRL' + data_str + 'R')
        [success,response] = self.send_msg(ser_msg, wait_on_busy = wait_on_busy, verbose = verbose)

        if success:
            try:
                data_str = response[5:(5 + self.data_len)]
                #self.msg_if.pub_warn(method_name + ": Will convert tilt position str: " + data_str)
                pitch_count = int(data_str)
                pitch_deg = self.pos_count2deg(pitch_count)
            except Exception as e:
                self.msg_if.pub_warn(method_name + ": Failed to convert message to int: " + data_str + " " + str(e))

        return pitch_deg


    def driver_moveToPosition(self,yaw_deg, pitch_deg):
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)

        method_name = sys._getframe().f_code.co_name
        success = False
        pos_count = self.deg2pos_count(yaw_deg)
        data_str = self.create_pos_str(pos_count)
        ser_msg= (self.pan_str + self.addr_str + 'MML' + data_str + 'W')
        [success_1,response] = self.send_msg(ser_msg)
        
        nepi_ros.sleep(self.SERIAL_SEND_DELAY)

        pos_count = self.deg2pos_count(pitch_deg)
        data_str = self.create_pos_str(pos_count)
        ser_msg= (self.tilt_str + self.addr_str + 'MML' + data_str + 'W')
        [success_2,response] = self.send_msg(ser_msg)
        success = (success_1 == True and success_2 == True) 
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

            nepi_ros.sleep(self.SERIAL_SEND_DELAY)

            ser_msg= (self.tilt_str + self.addr_str + 'MST' + data_str + 'W')
            self.msg_if.pub_warn(method_name + ": Sending Stop Tilt serial msg: " + ser_msg)



        else:
            return False

        [success,response] = self.send_msg(ser_msg)
        return success 


    def driver_jog(self,axis_str, direction):
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

    def check_timer_callback(self,timer):
        if self.serial_port is None:
            return True
        else:
            success = False
            # Test message
            data_str = self.create_blank_str()
            ser_msg= (self.pan_str + self.addr_str + 'MRA' + data_str + 'R')
            [success,response] = self.send_msg(ser_msg, wait_on_busy = False, verbose = False)
            # Update results and take actions
        if success:
            self.self_check_counter = 0 # reset comms failure count
        else:
            self.self_check_counter = self.self_check_counter + 1 # increment counter
        #self.msg_if.pub_warn("Current failed comms count: " + str(self.self_check_counter))
        if self.self_check_counter > self.self_check_count:  # Crashes node if set above limit??
            self.msg_if.pub_warn("Shutting down device: " +  self.addr_str + " on port " + self.port_str)
            self.msg_if.pub_warn("Too many comm failures")
            nepi_ros.signal_shutdown("To many comm failures")   

    
    ### Function to try and connect to device at given port and baudrate
    def connect(self):
        success = False
        self.connect_attempts += 1
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
                    nepi_ros.sleep(self.SERIAL_SEND_DELAY)
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
                    self.msg_if.pub_warn(caller_method + ": send_msg: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
                    self.msg_if.pub_warn(caller_method + ": send_msg: Locked serial with send msg: " + ser_msg)
                ser_str = (ser_msg + '\r\n')
                b=bytearray()
                b.extend(map(ord, ser_str))
                try:
                    self.serial_port.write(b)
                except Exception as e:
                    self.msg_if.pub_warn(caller_method + ": send_msg: Failed to send message " + str(e))
                time.sleep(self.SERIAL_RECEIVE_DELAY)
                try:
                    bs = self.serial_port.readline()
                    response = bs.decode()
                    if verbose == True:
                        self.msg_if.pub_debug(caller_method + ": send_msg: Device returned: " + str(response) + " for: " +  ser_str)
                except Exception as e:
                    self.msg_if.pub_warn(caller_method + ": send_msg: Failed to recieve message " + str(e))
                if verbose == True:
                    self.msg_if.pub_warn(caller_method + ": send_msg: Unlocking serial")
                    self.msg_if.pub_warn(caller_method + ": send_msg: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")

                success = self.check_valid_response(ser_msg,response)
                if success == False:
                    # Try again
                    try:
                        bs = self.serial_port.readline()
                        response = bs.decode()
                        if verbose == True:
                            self.msg_if.pub_debug(caller_method + ": send_msg: Device returned: " + str(response) + " for: " +  ser_str)
                    except Exception as e:
                        self.msg_if.pub_warn(caller_method + ": send_msg: Failed to recieve message " + str(e))
                    if verbose == True:
                        self.msg_if.pub_warn(caller_method + ": send_msg: Unlocking serial")
                        self.msg_if.pub_warn(caller_method + ": send_msg: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")



                self.serial_busy = False
        else:
            self.msg_if.pub_warn(caller_method + ": Serial port busy, can't send msg: " + ser_msg)
    
        return [success, response]


    def check_valid_response(self,ser_msg, response):
        caller_method = inspect.currentframe().f_back.f_code.co_name
        valid = False
        if len(response) >= 5 + self.data_len + 1:
            if response[0:5] == ser_msg[0:5]:
                valid = True
        if valid == False:
            self.msg_if.pub_warn(caller_method + ": Failed to get valid response message from: " + ser_msg + " : " + str(response))
        return valid




    ### Function for checking if port is available
    def check_port(self,port_str):
        success = False
        ports = serial.tools.list_ports.comports()
        for loc, desc, hwid in sorted(ports):
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
        count = float(count/max_count)
        return count

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
