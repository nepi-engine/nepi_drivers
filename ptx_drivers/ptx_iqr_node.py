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


from dataclasses import dataclass
from threading import Thread, Lock
from ctypes import c_int16

from nepi_sdk.nepi_modbus import ModbusRTUMaster

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings



from nepi_api.device_if_ptx import PTXActuatorIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'PTX_IQR' # Use in display menus
FILE_TYPE = 'NODE'


@dataclass
class PanTiltStatus:
    # offect
    id: int = 0  # 0
    serial_num: str = ""  # 1
    hw_version: str = ""  # 2
    bd_version: str = ""  # 3
    sw_version: str = ""  # 4
    set_zero: int = 0  # 5
    speed: int = 0  # 6
    yaw_goal: float = 0.0  # 7
    pitch_goal: float = 0.0  # 8
    reserved: int = 0  # 9
    driver_ec: int = 0  # 10
    encoder_ec: int = 0  # 11
    yaw_now: float = 0.0  # 12
    pitch_now: float = 0.0  # 13
    yaw_temp: float = 0.0  # 14
    pitch_temp: float = 0.0  # 15
    yaw_raw: int = 0  # 16
    pitch_raw: int = 0  # 17
    loop_ec: int = 0  # 18
    loop_time: int = 0  # 19


class IqrPanTiltNode:
    MAX_POSITION_UPDATE_RATE = 10
    HEARTBEAT_CHECK_INTERVAL = 1.0

    LIMITS_DICT = dict()
    LIMITS_DICT['max_yaw_hardstop_deg'] = 60
    LIMITS_DICT['min_yaw_hardstop_deg'] = -60
    LIMITS_DICT['max_pitch_hardstop_deg'] = 60
    LIMITS_DICT['min_pitch_hardstop_deg'] = -60
    LIMITS_DICT['max_yaw_softstop_deg'] = 59
    LIMITS_DICT['min_yaw_softstop_deg'] = -59
    LIMITS_DICT['max_pitch_softstop_deg'] = 59
    LIMITS_DICT['min_pitch_softstop_deg'] = -59

    DEG_DIR = 1

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
    fw_version = "Unknown"
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

    speed_ratio = 0.5

    drv_dict = dict()    



    baud_str = '115200'
    addr_str = '1'
    pt_status_lock = Lock()


    modbus_master = None

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
            self.addr_int = int(self.addr_str)

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
            nepi_ros.signal_shutdown("Unable to connect to Pan Tilt device")   
        else:
            ################################################
            self.msg_if.pub_info("... Connected!")
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

            self.device_info_dict["serial_number"] = self.serial_num
            self.device_info_dict["hw_version"] = self.hw_version
            self.device_info_dict["sw_version"] = self.sw_version

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
                                        moveYawCb = None, #self.moveYaw,
                                        movePitchCb = None, #self.movePitch,
                                        setSpeedRatioCb = self.setSpeedRatio,
                                        getSpeedRatioCb = self.getSpeedRatio,
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


    def movePitch(self, direction, duration):
        pass


    def setSpeedRatio(self, ratio):
        # TODO: Limits checking and driver unit conversion?
        self.speed_ratio = ratio


    def getSpeedRatio(self):
        # TODO: Driver unit conversion?
        ratio = self.driver_getSpeedRatio()
        return ratio
          

    def getCurrentPosition(self):
        yaw_deg, pitch_deg = self.driver_getCurrentPosition()
        #self.msg_if.pub_warn("Got pos degs : " + str([yaw_deg, pitch_deg]))
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
            yaw_deg, pitch_deg = self.driver_getCurrentPosition()
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
            yaw_deg, pitch_deg = self.driver_getCurrentPosition()
            self.waypoints[waypoint_index] = {'yaw_deg': yaw_deg, 'pitch_deg': pitch_deg}

        self.msg_if.pub_info("Waypoint set to current position")

    def setCurrentSettingsAsDefault(self):
        # Don't need to worry about any of our params in this class, just child interfaces' params
        if self.ptx_if is not None:
            self.ptx_if.initConfig()

    def resetDevice(self):
        self.speed_ratio = 0.5



   #######################
    ### Driver Interface Functions
    def driver_getPtStatus(self):
            pt_status = None
            success = False
            if self.modbus_master is not None:
                with self.pt_status_lock:
                    try:
                        rcvdBuf = self.modbus_master.get_multiple_registers(
                            self.addr_int, 0x0000, 20)
                        success = True
                    except Exception as e:
                        self.msg_if.pub_info("Failed to get receive buffer: " + str(e))
                    if success == True:
                        success = False
                        pt_status = PanTiltStatus()
                        pt_status.id = rcvdBuf[0]
                        pt_status.serial_num = f"SN{int(rcvdBuf[1])}"
                        pt_status.hw_version = f"v{int((rcvdBuf[2] & 0xff00) >> 8)}.{int(rcvdBuf[2] & 0x00ff)}"
                        pt_status.bd_version = f"v{int((rcvdBuf[3] & 0xff00) >> 8)}.{int(rcvdBuf[3] & 0x00ff)}"
                        pt_status.sw_version = (f"v{int((rcvdBuf[4] & 0xf000) >> 12)}."
                                            f"{int((rcvdBuf[4] & 0x0f00) >> 8)}.{int(rcvdBuf[4] & 0x00ff)}")
                        pt_status.set_zero = rcvdBuf[5]
                        pt_status.speed = rcvdBuf[6]
                        pt_status.yaw_goal = c_int16(rcvdBuf[7]).value / 100.0
                        pt_status.pitch_goal = c_int16(rcvdBuf[8]).value / 100.0
                        pt_status.reserved = rcvdBuf[9]
                        pt_status.driver_ec = rcvdBuf[10]
                        pt_status.encoder_ec = rcvdBuf[11]
                        pt_status.yaw_now = c_int16(rcvdBuf[12]).value / 100.0
                        pt_status.pitch_now = c_int16(rcvdBuf[13]).value / 100.0
                        pt_status.yaw_temp = c_int16(rcvdBuf[14]).value / 10.0
                        pt_status.pitch_temp = c_int16(rcvdBuf[15]).value / 10.0
                        pt_status.yaw_raw = c_int16(rcvdBuf[16]).value
                        pt_status.pitch_raw = c_int16(rcvdBuf[17]).value
                        pt_status.loop_ec = rcvdBuf[18]
                        pt_status.loop_time = rcvdBuf[19]
                        success = True
            return success,pt_status
                

    def driver_stopMotion(self):
        success = False
        [success,pt_status] = self.driver_getPtStatus()
        if pt_status is not None:
            yaw = pt_status.yaw_now
            pitch = pt_status.pitch_now
            self.driver_setPosition(yaw,pitch)
            success = True
        return success


    def driver_getCurrentPosition(self):
        yaw = -999
        pitch = -999
        [success,pt_status] = self.driver_getPtStatus()
        if pt_status is not None:
            yaw = pt_status.yaw_now
            pitch = pt_status.pitch_now
        return yaw, pitch

    def driver_moveToPosition(self,yaw, pitch):
        if yaw < -60.0 or yaw > 60.0:
            return
        if pitch < -60.0 or pitch > 60.0:
            return
        speed_count = self.ratio2speed_count(self.speed_ratio)
        with self.pt_status_lock:
            if self.modbus_master is not None:
                sendBuf = [speed_count, round(yaw*100.0), round(pitch*100)]
                self.modbus_master.set_multiple_registers(self.addr_int, 0x0006, sendBuf)
                nepi_ros.sleep(0.01)

    def driver_getSpeedRatio(self, axis_str = '#'):
        method_name = sys._getframe().f_code.co_name
        ratio = -999
        [success,pt_status] = self.driver_getPtStatus()
        if success:
            ratio = self.speed_count2ratio(pt_status.speed)
        return ratio
        

      



    #######################
    ### Driver Util Functions

    def check_timer_callback(self,timer):
        if self.modbus_master is None:
            return True
        else:
            success = False
            try:
                # Try to request pt_status
                self.msg_if.pub_info("Connection Check requesting info for device on port: " + self.port_str)
                [success,pt_status] = self.driver_getPtStatus()
            except Exception as e:
                self.msg_if.pub_warn("Something went wrong with connecting to serial port at: " + self.port_str + "(" + str(e) + ")" )
            if success == True:
                success = False
                # Send Message
                self.msg_if.pub_info("Requesting info for device on poret: " + self.port_str)

                [success,pt_status] = self.driver_getPtStatus()
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
        pt_status = None
        if port_check is True:
            try:
                # Try and open serial port
                self.msg_if.pub_info("Opening mod_bus port " + self.port_str + " with baudrate: " + self.baud_str + " with addr: " + self.addr_str)
                self.modbus_master = ModbusRTUMaster(self.port_str, self.baud_int)
                self.msg_if.pub_info("Modbus port opened")
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Something went wrong with connecting to serial port at: " + self.port_str + "(" + str(e) + ")" )
            if success == True:
                nepi_ros.sleep(0.1)
                success = False
                # Send Message
                self.msg_if.pub_info("Connect process requesting info for device on port: " + self.port_str)

                [success,pt_status] = self.driver_getPtStatus()

                if success:
                    self.msg_if.pub_info("Connected to device on port: " +  self.port_str)
                    # Update serial, hardware, and software status values
                    self.serial_num = pt_status.serial_num
                    self.hw_version = pt_status.hw_version
                    self.sw_version = pt_status.sw_version
                    self.fw_version = pt_status.sw_version
                    success = True
                    # Factory Reset Device
    

                self.driver_moveToPosition(0.0, 0.0)

        return success


 



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
        count = int(ratio * 30)
        if count < 1:
            count = 1
        if count > 30:
            count = 30
        return count

    def speed_count2ratio(self,count):
        ratio = float(count/30)
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
	node = IqrPanTiltNode()





