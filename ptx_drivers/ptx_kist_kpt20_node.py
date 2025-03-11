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
#os.environ["ROS_NAMESPACE"] = "/nepi/s2x"
import serial
import serial.tools.list_ports
import time
import re
import sys

from nepi_sdk.device_if_ptx import ROSPTXActuatorIF

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_drv
from nepi_sdk import nepi_settings



PKG_NAME = 'PTX_KIST_KTP20' # Use in display menus
FILE_TYPE = 'NODE'


class KistKPT20Node:


    CONFIGS_DICT = {
        '1:160' : {'high_res':False, 'deg_per_step':0.0703125,'max_pos':7498,'min_pos':2502,'home':5000},
        '1:160-HR' : {'high_res':True, 'deg_per_step':0.000703125,'max_pos':749800,'min_pos':250200,'home':500000},
        '1:120' : {'high_res':False, 'deg_per_step':0.09375,'max_pos':6873,'min_pos':3127,'home':5000},
        '1:120-HR' : {'high_res':True, 'deg_per_step':0.0009375,'max_pos':687300,'min_pos':312700,'home':500000}
    }

    config_dict = dict()
    high_res = False
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

    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
    drv_dict = dict()                                                    
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")


        ##############################
        # Get required drv driver dict info
        self.drv_dict = nepi_ros.get_param(self,'~drv_dict',dict()) 
        #nepi_msg.publishMsgWarn(self,"Nex_Dict: " + str(self.drv_dict))
        try:
            self.port_str = self.drv_dict['DEVICE_DICT']['device_path'] 
            self.baud_str = self.drv_dict['DEVICE_DICT']['baud_str'] 
            self.baud_int = int(self.baud_str)
            self.addr_str = self.drv_dict['DEVICE_DICT']['addr_str'] 

            system_config = drv_dict['DISCOVERY_DICT']['OPTIONS']['system_config']['value']
            self.config_dict = self.CONFIGS_DICT[system_config]
            self.high_res = self.config_dict['high_res']
            if self.high_res == True:
                self.data_len = 6
            self.connection = drv_dict['DISCOVERY_DICT']['OPTIONS']['connection']['value']
        except Exception as e:
            nepi_msg.publishMsgWarn(self, "Failed to load Device Dict " + str(e))#
            nepi_ros.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        ################################################  
        nepi_msg.publishMsgInfo(self,"Connecting to Device on port " + self.port_str + " with baud " + self.baud_str)
        ### Try and connect to device
        self.connected = self.connect() 
        if self.connected == False:
            nepi_msg.publishMsgInfo(self,"Shutting down node")
            nepi_msg.publishMsgInfo(self,"Specified serial port not available")
            nepi_ros.signal_shutdown("Serial port not available")   
        else:
            ################################################
            nepi_msg.publishMsgInfo(self,"... Connected!")
            self.dev_info = self.driver_getDeviceInfo()
            self.logDeviceInfo()
            # Initialize settings
            self.cap_settings = self.getCapSettings()
            self.factory_settings = self.getFactorySettings()

            # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
            nepi_msg.publishMsgInfo(self,"Launching NEPI PTX (ROS) interface...")
            self.device_info_dict["node_name"] = self.node_name
            if self.node_name.find("_") != -1:
                split_name = self.node_name.rsplit('_', 1)
                self.device_info_dict["device_name"] = split_name[0]
                self.device_info_dict["identifier"] = split_name[1]
            else:
                self.device_info_dict["device_name"] = self.node_name
                self.device_info_dict["identifier"] = ""
            self.device_info_dict["serial_number"] = self.dev_info["SerialNum"]
            self.device_info_dict["hw_version"] = ""
            self.device_info_dict["sw_version"] = self.dev_info["FirmwareVersion"]



            # Must pass a capabilities structure to ptx_interface constructor
            ptx_capabilities_dict = {}

            # Now check with the driver if any of these PTX capabilities are explicitly not present
            if not self.driver_hasAdjustableSpeed():
                getSpeedCb = None # Clear the method
                self.setSpeed = None # Clear the method
                self.has_speed_control = False
            else:
                has_speed_control = True
                
            if not self.driver_reportsPosition():
                self.GetCurrentPosition = None # Clear the method
                
            if not self.driver_hasAbsolutePositioning():
                self.GotoPosition = None # Clear the method
            
            self.has_absolute_positioning_and_feedback = self.driver_hasAbsolutePositioning() and self.driver_reportsPosition()
            ptx_capabilities_dict['has_absolute_positioning'] = self.has_absolute_positioning_and_feedback
                    
            if self.has_absolute_positioning_and_feedback:
                ptx_capabilities_dict['has_homing'] = True
            
            
            if self.has_absolute_positioning_and_feedback:
                ptx_capabilities_dict['has_waypoints'] = True
                
            if not self.has_absolute_positioning_and_feedback:
                self.SetHomePosition = None    
                self.SetWaypoint = None


            # Launch the PTX interface --  this takes care of initializing all the ptx settings from config. file, subscribing and advertising topics and services, etc.
            # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
            nepi_msg.publishMsgInfo(self,"Launching NEPI PTX (ROS) interface...")
            self.device_info_dict["node_name"] = self.node_name
            if self.node_name.find("_") != -1:
                split_name = self.node_name.rsplit('_', 1)
                self.device_info_dict["device_name"] = split_name[0]
                self.device_info_dict["identifier"] = split_name[1]
            else:
                self.device_info_dict["device_name"] = self.node_name
                self.device_info_dict["identifier"] = ""
            self.device_info_dict["serial_number"] = self.driver_getDeviceSerialNumber()
            self.device_info_dict["hw_version"] = self.driver_getDeviceSerialNum()
            self.device_info_dict["sw_version"] = self.driver_getDeviceFirmwareVersion()

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
            
            # Driver can specify position limits via getPositionLimitsInDegrees. Otherwise, we hard-code them 
            # to arbitrary values here, but can be overridden in device config file (see ptx_if.py)
            self.default_settings = dict()
            if hasattr(self.driver, 'getPositionLimitsInDegrees'):
                driver_specified_limits = self.driver_getPositionLimitsInDegrees()
                self.default_settings['max_yaw_hardstop_deg'] = driver_specified_limits['max_yaw_hardstop_deg']
                self.default_settings['min_yaw_hardstop_deg'] = driver_specified_limits['min_yaw_hardstop_deg']
                self.default_settings['max_pitch_hardstop_deg'] = driver_specified_limits['max_pitch_hardstop_deg']
                self.default_settings['min_pitch_hardstop_deg'] = driver_specified_limits['min_pitch_hardstop_deg']
                self.default_settings['max_yaw_softstop_deg'] = driver_specified_limits['max_yaw_softstop_deg']
                self.default_settings['min_yaw_softstop_deg'] = driver_specified_limits['min_yaw_softstop_deg']
                self.default_settings['max_pitch_softstop_deg'] = driver_specified_limits['max_pitch_softstop_deg']
                self.default_settings['min_pitch_softstop_deg'] = driver_specified_limits['min_pitch_softstop_deg']
            else:
                self.default_settings['max_yaw_hardstop_deg'] = 60.0
                self.default_settings['min_yaw_hardstop_deg'] = -60.0
                self.default_settings['max_pitch_hardstop_deg'] = 60.0
                self.default_settings['min_pitch_hardstop_deg'] = -60.0
                self.default_settings['max_yaw_softstop_deg'] = 59.0
                self.default_settings['min_yaw_softstop_deg'] = -59.0
                self.default_settings['max_pitch_softstop_deg'] = 59.0
                self.default_settings['min_pitch_softstop_deg'] = -59.0

            # Initialize settings
            self.cap_settings = self.getCapSettings()
            nepi_msg.publishMsgInfo(self,"" +"CAPS SETTINGS")
            #for setting in self.cap_settings:
                #nepi_msg.publishMsgInfo(self,"" +setting)
            self.factory_settings = self.getFactorySettings()
            nepi_msg.publishMsgInfo(self,"" +"FACTORY SETTINGS")
            #for setting in self.factory_settings:
                #nepi_msg.publishMsgInfo(self,"" +setting)

            self.home_yaw_deg = 0.0
            self.home_pitch_deg = 0.0
            self.waypoints = {} # Dictionary of dictionaries with numerical key and {waypoint_pitch, waypoint_yaw} dict value


            self.ptx_if = ROSPTXActuatorIF(device_info = self.device_info_dict, 
                                        capSettings = self.cap_settings,
                                        factorySettings = self.factory_settings,
                                        settingUpdateFunction=self.settingUpdateFunction,
                                        getSettingsFunction=self.getSettings,
                                        factoryControls = self.FACTORY_CONTROLS,
                                        defaultSettings = self.default_settings,
                                        capabilities_dict = ptx_capabilities_dict,
                                        stopMovingCb = self.stopMoving,
                                        moveYawCb = self.moveYaw,
                                        movePitchCb = self.movePitch,
                                        setSpeedCb = self.setSpeed,
                                        getSpeedCb = self.getSpeed,
                                        getCurrentPositionCb = self.getCurrentPosition,
                                        gotoPositionCb = self.gotoPosition,
                                        goHomeCb = self.goHome,
                                        setHomePositionCb = self.setHomePosition,
                                        setHomePositionHereCb = self.setHomePositionHere,
                                        gotoWaypointCb = self.gotoWaypoint,
                                        setWaypointCb = self.setWaypoint,
                                        setWaypointHereCb = self.setWaypointHere)
            nepi_msg.publishMsgInfo(self," ... PTX interface running")

            # Start an sealite activity check process that kills node after some number of failed comms attempts
            nepi_msg.publishMsgInfo(self,"Starting an activity check process")
            nepi_ros.start_timer_process(nepi_ros.ros_duration(0.2), self.check_timer_callback)
            # Initialization Complete
            nepi_msg.publishMsgInfo(self,"Initialization Complete")
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
        nepi_msg.publishMsgInfo(self,dev_info_string)


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
                #nepi_msg.publishMsgInfo(self,"Calling get setting function " + function_str_name)
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
            #nepi_msg.publishMsgInfo(self,"Calling set setting function " + function_str_name)
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
        if self.ptx_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.PT_DIRECTION_NEGATIVE
            self.driver_jog(axis_str = self.pan_str, direction = direction, time_s = duration)

    def movePitch(self, direction, duration):
        if self.ptx_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.PT_DIRECTION_NEGATIVE
            self.driver_jog(axis_str = self.tilt_str, direction = direction, time_s = duration)

    def setSpeed(self, speed_ratio):
        # TODO: Limits checking and driver unit conversion?
        self.driver_setSpeedRatio(speed_ratio)

    def getSpeed(self):
        # TODO: Driver unit conversion?
        speed_ratio = self.driver_getSpeedRatio()
        return speed_ratio
          

    def getCurrentPosition(self):
        yaw_deg, pitch_deg = self.driver_getCurrentPosition()
        #print("Got pos degs : " + str([pan_deg, tilt_deg]))
        return yaw_deg, pitch_deg
        

    def gotoPosition(self, yaw_deg, pitch_deg):
        self.driver_moveToPosition(yaw_deg, pitch_deg)
        
    def goHome(self):
        if self.driver_hasAbsolutePositioning() is True and self.ptx_if is not None:
            self.driver_moveToPosition(self.home_yaw_deg, self.home_pitch_deg)

    def setHomePosition(self, yaw_deg, pitch_deg):
        self.home_yaw_deg = yaw_deg
        self.home_pitch_deg = pitch_deg

    def setHomePositionHere(self):
        if self.driver_reportsPosition() is True:
            yaw_deg, pitch_deg = self.driver_getCurrentPosition()
            self.home_yaw_deg = yaw_deg
            self.home_pitch_deg = pitch_deg 

    def gotoWaypoint(self, waypoint_index):
        if self.driver_hasAbsolutePositioning() is True and self.ptx_if is not None:
            if waypoint_index not in self.waypoints:
                return
            waypoint_yaw_deg = self.waypoints[waypoint_index]['yaw_deg']
            waypoint_pitch_deg = self.waypoints[waypoint_index]['pitch_deg']
            self.driver_moveToPosition(waypoint_yaw_deg, waypoint_pitch_deg)
    
    def setWaypoint(self, waypoint_index, yaw_deg, pitch_deg):
        self.waypoints[waypoint_index] = {'yaw_deg': yaw_deg, 'pitch_deg': pitch_deg}
        
    def setWaypointHere(self, waypoint_index):
        if self.driver_reportsPosition() and self.ptx_if is not None:
            yaw_deg, pitch_deg = self.driver_getCurrentPosition()
            self.waypoints[waypoint_index] = {'yaw_deg': yaw_deg, 'pitch_deg': pitch_deg}

        nepi_msg.publishMsgInfo(self,"Waypoint set to current position")

    def setCurrentSettingsAsDefault(self):
        # Don't need to worry about any of our params in this class, just child interfaces' params
        if self.ptx_if is not None:
            self.ptx_if.setCurrentSettingsToParamServer()

    def updateFromParamServer(self):
        if self.ptx_if is not None:
            self.ptx_if.updateFromParamServer()


    #######################
    ### Misc Class Functions

    def check_timer_callback(self,timer):
        success = False
        # Test message
        if self.high_res == False:
            data_str = '0000'
        else:
            data_str = '000000'
        ser_msg= ('!' + self.addr_str + 'MVR' + data_str + 'R')
        ser_str = (ser_msg + '\r\n')

        b=bytearray()
        b.extend(map(ord, ser_str))
        while self.serial_busy == True and not nepi_ros.is_shutdown():
            time.sleep(0.01) # Wait for serial port to be available
        self.serial_busy = True
        self.serial_port.write(b)
        time.sleep(.01)
        try:
            bs = self.serial_port.readline()
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to receive message")
            self.serial_busy = False
            response = bs.decode()
        # Check for valid response 
        if len(response) > 5:
            if response[0:5] == ser_msg[0:5]:
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
            nepi_msg.publishMsgWarn(self,"Shutting down device: " +  self.addr_str + " on port " + self.port_str)
            nepi_msg.publishMsgWarn(self,"Too many comm failures")
            nepi_ros.signal_shutdown("To many comm failures")   
    
    ### Function to try and connect to device at given port and baudrate
    def connect(self):
        success = False
        port_check = self.check_port(self.port_str)
        if port_check is True:
            try:
                # Try and open serial port
                nepi_msg.publishMsgInfo(self,"Opening serial port " + self.port_str + " with baudrate: " + self.baud_str)
                if self.connection == 'Serial':
                    serial_port = serial.Serial(path_str,self.baud_int,timeout = 1)
                elif self.connection == 'TCP-Server':
                    serial_port = serial.serial_for_url('socket://' + path_str, timeout=1)
                nepi_msg.publishMsgInfo(self,"Serial port opened")
                success = True
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Something went wrong with connecting to serial port at: " + self.port_str + "(" + str(e) + ")" )
            if success == True:
                success = False
                try:
                    # Send Message
                    nepi_msg.publishMsgInfo(self,"Requesting info for device: " + self.addr_str)
                    # Test message
                    if self.high_res == False:
                        data_str = '0000'
                    else:
                        data_str = '000000'
                    ser_msg= ('!' + self.addr_str + 'MVR' + data_str + 'R')
                    #nepi_msg.publishMsgInfo(self,"Sending serial string: " + ser_msg)
                    response = self.send_msg(ser_msg)
                    success = True
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Something went wrong with sending message at serial port at: " + self.port_str + "(" + str(e) + ")" )
                if success == True:
                    #nepi_msg.publishMsgInfo(self,"Got response message: " + response)
                    if len(response) > 5:
                        if response[0:5] == ser_msg[0:5]:
                            success = True
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
                nepi_msg.publishMsgWarn(self,"serial port not active")
        return success


    def send_msg(self,ser_msg):
        response = ""
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
                except Exception as e1:
                    print("Failed to send message " + str(e1))
                time.sleep(.01)
                try:
                    bs = self.serial_port.readline()
                    self.serial_busy = False
                    response = bs.decode()
                    #print("Send response received: " + response[0:-2])
                except Exception as e1:
                    print("Failed to recieve message " + str(e2))
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
    ### Driver Interface Functions

    def pos2deg(self, pos):
        hr = self.high_res
        dps = self.config_dict['deg_per_step'] 
        home = self.config_dict['home'] 
        deg = (pos - home)*dps  
        return deg

    def deg2pos(self, deg):
        hr = self.high_res
        dps = self.config_dict['deg_per_step']
        home = self.config_dict['home'] 
        pos = int(pos/dps + home)
        return pos


    def driver_getDeviceInfo(self):
        dev_info = dict()
        dev_info["Manufacturer"] = KIST
        dev_info["Model"] = KPT20

        firmware = ""
        if self.high_res == False:
            data_str = '0000'
        else:
            data_str = '000000'
        ser_msg= ('!' + self.addr_str + 'MVR' + data_str + 'R')
        if len(response) > 5:
            if response[0:5] == ser_msg[0:5]:
                try:
                    firmware = response[5:8]
                except Exception as e:
                    print("Failed to convert message to int: " + data_str + " " + str(e))
        else:
             print("Failed to get valid response message from: " + ser_msg)
        dev_info["FirmwareVersion"] = firmware

        dev_info["SerialNum"] = ""
        return dev_info

    def driver_getPositionLimitsInDegrees(self):
        limits_dict = dict()
        limits_dict['max_yaw_hardstop_deg'] = 175
        limits_dict['min_yaw_hardstop_deg'] = -175
        limits_dict['max_pitch_hardstop_deg'] = 175
        limits_dict['min_pitch_hardstop_deg'] = -175
        limits_dict['max_yaw_softstop_deg'] = 174
        limits_dict['min_yaw_softstop_deg'] = -174
        limits_dict['max_pitch_softstop_deg'] = 174
        limits_dict['min_pitch_softstop_deg'] = -174
        return limits_dict
                

    def driver_hasAdjustableSpeed(self):
        hasAdjSpeed = True
        return hasAdjSpeed

    def driver_getSpeedRatio(self):
        speedRatio = -999
        success = False
        if self.high_res == False:
            data_str = '0000'
        else:
            data_str = '000000'
        ser_msg= (self.both_str + self.addr_str + 'MVS' + data_str + 'R')
        response = self.send_msg(ser_msg)
        if len(response) > 5:
            if response[0:5] == ser_msg[0:5]:
                try:
                    data_str = response[5:-1]
                    cur_speed = int(data_str)
                    speedRatio = cur_speed/self.MAX_SPEED
                except Exception as e:
                    print("Failed to convert message to int: " + data_str + " " + str(e))
        else:
             print("Failed to get valid response message from: " + ser_msg)
        return speedRatio

    def driver_setSpeedRatio(self,speedRatio):
        success = False
        try:
            set_speed_str = str(int(speedRatio * self.MAX_SPEED))
        except Exception as e:
            print("Failed to convert message: " + self.speedRatio + " " + str(e))
        
        zero_prefix_len = self.data_length-len(set_speed_str)
        for z in range(zero_prefix_len):
            data_str = ('0' + set_speed_str)
        ser_msg= (self.both_str + self.addr_str + 'MSP' + data_str + 'W')
        response = self.send_msg(ser_msg)

        if len(response) > 5:
            if response[0:5] == ser_msg[0:5]:
                success = True
            else:
                print("Failed to get valid response message from: " + ser_msg)

        return success




    def driver_reportsPosition(self):
        reportsPos = True
        return reportsPos

    def driver_getCurrentPosition(self):
        yaw_deg = -999
        success = False
        if self.high_res == False:
            data_str = '0000'
        else:
            data_str = '000000'
        ser_msg= (self.pan_str + self.addr_str + 'MRL' + data_str + 'R')
        response = self.send_msg(ser_msg)
        if len(response) > 5:
            if response[0:5] == ser_msg[0:5]:
                try:
                    data_str = response[5:-1]
                    yaw_pos = int(data_str)
                    yaw_deg = self.pos2deg(yaw_pos)
                except Exception as e:
                    print("Failed to convert message to int: " + data_str + " " + str(e))
        else:
             print("Failed to get valid response message from: " + ser_msg)

        pitch_deg = -999
        success = False
        if self.high_res == False:
            data_str = '0000'
        else:
            data_str = '000000'
        ser_msg= (self.tilt_str + self.addr_str + 'MRL' + data_str + 'R')
        response = self.send_msg(ser_msg)
        if len(response) > 5:
            if response[0:5] == ser_msg[0:5]:
                try:
                    data_str = response[5:-1]
                    pitch_pos = int(data_str)
                    pitch_deg = self.pos2deg(pitch_pos)
                except Exception as e:
                    print("Failed to convert message to int: " + data_str + " " + str(e))
        else:
             print("Failed to get valid response message from: " + ser_msg)
        return yaw_deg, pitch_deg


    def driver_hasAbsolutePositioning(self):
        hasAbsPos = True
        return hasAbsPos

    def driver_moveToPosition(self,yaw_deg, pitch_deg):
        success = False
        data_str = str(deg2pos(self, yaw_deg))
        ser_msg= (self.pan_str + self.addr_str + 'MML' + data_str + 'W')
        response = self.send_msg(ser_msg)  
        check_1 = data_str == ser_msg
        
        data_str = str(deg2pos(self, pitch_deg))
        ser_msg= (self.tilt_str + self.addr_str + 'MML' + data_str + 'W')
        response = self.send_msg(ser_msg)  
        check_2 = data_str == ser_msg
        success = (check_1 == True and check_2 == True) 
        return success


    def driver_stopMotion(self):
        success = False
        data_str = str(deg2pos(self, yaw_deg))
        ser_msg= (self.both_str + self.addr_str + 'MST' + data_str + 'W')
        response = self.send_msg(ser_msg)  
        success = data_str == ser_msg
        return success


    def driver_jog(self,axis_str, direction, time_s):
        success = False
        if self.high_res == False:
            data_str = '0000'
        else:
            data_str = '000000'
        if direction == 1:
            cmd_str = 'MMF'
        else:
            cmd_str = 'MMB'
        ser_msg= (axis_str + self.addr_str + cmd_str + data_str + 'W')
        response = self.send_msg(ser_msg)  
        success = data_str == ser_msg
        if success:
            nepi_ros.sleep(time_s)
    
            ser_msg= (axis_str + self.addr_str + 'MST' + data_str + 'W')
            response = self.send_msg(ser_msg)  
            success = data_str == ser_msg
        return success



    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")
        if self.serial_port is not None:
            self.serial_port.close()


if __name__ == '__main__':
	node = KistKPT20Node()
