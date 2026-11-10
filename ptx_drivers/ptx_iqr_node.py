#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

import os
import copy
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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_nav



from nepi_api.device_if_ptx import PTXActuatorIF
from nepi_api.device_if_npx import NPXDeviceIF
from nepi_api.device_if_svx import DeviceSVXIF
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
    pan_goal: float = 0.0  # 7
    tilt_goal: float = 0.0  # 8
    reserved: int = 0  # 9
    driver_ec: int = 0  # 10
    encoder_ec: int = 0  # 11
    pan_now: float = 0.0  # 12
    tilt_now: float = 0.0  # 13
    pan_temp: float = 0.0  # 14
    tilt_temp: float = 0.0  # 15
    pan_raw: int = 0  # 16
    tilt_raw: int = 0  # 17
    loop_ec: int = 0  # 18
    loop_time: int = 0  # 19


class IqrPanTiltNode:
    MAX_POSITION_UPDATE_RATE = 10
    HEARTBEAT_CHECK_INTERVAL = 1.0

    FACTORY_LIMITS_DICT = dict()
    FACTORY_LIMITS_DICT['min_pan_hardstop_deg'] = -60
    FACTORY_LIMITS_DICT['max_pan_hardstop_deg'] = 60

    FACTORY_LIMITS_DICT['min_tilt_hardstop_deg'] = -60
    FACTORY_LIMITS_DICT['max_tilt_hardstop_deg'] = 60
    
    FACTORY_LIMITS_DICT['min_pan_softstop_deg'] = -60
    FACTORY_LIMITS_DICT['max_pan_softstop_deg'] = 60
    
    FACTORY_LIMITS_DICT['min_tilt_softstop_deg'] = -60
    FACTORY_LIMITS_DICT['max_tilt_softstop_deg'] = 60
    

    # A copy: as the same dict, every soft-limit change also rewrote the factory
    # limits that setSoftLimits clamps against, so a narrowed limit could not be
    # widened again until restart
    limits_dict = copy.deepcopy(FACTORY_LIMITS_DICT)

    PAN_DEG_DIR = 1
    TILT_DEG_DIR = 1

    CONFIGS_DICT = {
         'Standard' : {'data_len': 4, 'home':5000, 'deg_per_count':0.0879, 'degpsec_per_count': 0.5, 'max_degpsec': 20},
    }
    config_dict = CONFIGS_DICT['Standard']
    data_len = 4



    PT_DIRECTION_POSITIVE = 1
    PT_DIRECTION_NEGATIVE = -1

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")
    
    # Initialize some parameters
    serial_num = "Unknown"
    hw_version = "Unknown"
    sw_version = "Unknown"
    fw_version = "Unknown"
    ptx_if = None
    svx_pan_if = None
    svx_tilt_if = None

    both_str = '!'
    pan_str = '#'
    tilt_str = '$'


    serial_port = None
    serial_busy = False
    connect_attempts = 0
    connected = False


    self_check_count = 10
    self_check_counter = 0

    current_position = [0.0,0.0]
    last_position = current_position

    speed_ratio = 0.5

    drv_dict = dict()    



    baud_str = '115200'
    addr_str = '1'
    pt_status_lock = Lock()


    modbus_master = None

    pan_stop_trig = False
    tilt_stop_trig = False

    # Last goto target commanded on each axis, in the node frame, from any of the
    # PTX, pan SVX or tilt SVX interfaces. None means never commanded, or invalidated
    # by a stop or jog (which command both axes to where they are). The IQR takes
    # both axes in one write, so a single-axis goto resends the other axis's target
    # (or its current position) to leave that axis alone. goto_lock makes reading the
    # other target, recording this one and sending them one step, so a pan goto and a
    # tilt goto arriving together cannot undo each other. Serial access itself is
    # serialized separately by pt_status_lock.
    last_goto_pan_deg = None
    last_goto_tilt_deg = None
    goto_lock = Lock()

    pt_status = PanTiltStatus()


    msg_if = None
    node_if = None
    settings_if = None
    save_data_if = None
    transform_if = None
    navpose_if = None
    connect_navpose_if = None

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
        self.msg_if = MsgIF()
        self.msg_if.pub_info("Starting Node Initialization Processes")

        ##############################  
        # Initialize Class Variables

        # Get required drv driver dict info
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict()) 
        #self.msg_if.pub_debug("Got Drivers_Dict from param server: " + str(self.drv_dict))
        try:
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.baud_str = self.drv_dict['DEVICE_DICT']['baud_str'] 
            self.baud_int = int(self.baud_str)
            self.addr_str = self.drv_dict['DEVICE_DICT']['addr_str'] 
            self.addr_int = int(self.addr_str)

        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        ################################################  
        self.msg_if.pub_info("Connecting to Device on port " + self.device_path + " with baud " + self.baud_str)
        ### Try and connect to device
        # while self.connected == False and self.connect_attempts < 5:
        #     self.connected = self.connect() 
        #     if self.connected == False:
        #         nepi_sdk.sleep(1)
        self.connected = self.connect() 
        if self.connected == False:
            self.msg_if.pub_info("Shutting down node, Unable to connect to Pan Tilt device")
            nepi_sdk.signal_shutdown("Unable to connect to Pan Tilt device")   
        else:
            ################################################
            self.msg_if.pub_info("... Connected!")

            # Launch the PTX interface --  this takes care of initializing all the ptx settings from config. file, subscribing and advertising topics and services, etc.
            # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
            self.msg_if.pub_info("Launching NEPI PTX () interface...")
            self.device_info_dict["device_name"] = self.device_name
            self.device_info_dict["path"] = self.device_path

            self.device_info_dict["serial_number"] = self.serial_num
            self.device_info_dict["hw_version"] = self.hw_version
            self.device_info_dict["sw_version"] = self.sw_version

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
            

            # This device exposes no device settings, so it registers no
            # getSettingsFunction/setSettingFunction with PTXActuatorIF. Its
            # adjustable values (speed ratios, soft limits, home position) are
            # PTX controls, not settings. PTXActuatorIF also accepts a
            # getCapSettingsFunction argument, but that argument is inert --
            # it is never forwarded to SettingsIF, which has no such parameter;
            # a control's type, bounds and options now ride in the controls
            # dict itself.

            self.home_pan_deg = 0.0
            self.home_tilt_deg = 0.0

            self.ptx_if = PTXActuatorIF(device_info = self.device_info_dict, 
                                        factoryControls = self.FACTORY_CONTROLS,
                                        factoryLimits = self.FACTORY_LIMITS_DICT,
                                        stopMovingCb = self.stopMoving,
                                        movePanCb = self.movePan,
                                        moveTiltCb = self.moveTilt,
                                        setSoftLimitsCb=self.setSoftLimits,
                                        getSoftLimitsCb=self.getSoftLimits,                                        
                                        getSpeedRatioCb = self.getSpeedRatio,
                                        setSpeedRatioCb = self.setSpeedRatio,
                                        getPositionCb = self.getPosition,
                                        gotoPositionCb = self.gotoPosition,
                                        gotoPanPositionCb = self.gotoPanPosition,
                                        gotoTiltPositionCb = self.gotoTiltPosition,
                                        goHomeCb = self.goHome,
                                        setHomePositionCb = self.setHomePosition,
                                        setHomePositionHereCb = self.setHomePositionHere,
                                        deviceResetCb = self.resetDevice,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if,
                                        settings_if = self.settings_if,
                                        save_data_if = self.save_data_if,
                                        connect_navpose_if = self.connect_navpose_if
                                        )
            self.msg_if.pub_info(" ... PTX interface running")


            self.node_if = self.ptx_if.node_if
            self.settings_if = self.ptx_if.settings_if
            self.save_data_if = self.ptx_if.save_data_if
            self.transform_if = self.ptx_if.transform_if
            self.connect_navpose_if = self.ptx_if.connect_navpose_if
            self.navpose_if = self.ptx_if.navpose_if   

            ##################################
            # Setup NPX IF Class ####################
            # The camera's navpose is published by the NPX device at <node>/npx/navpose.
            # Compute it deterministically so the image data products can advertise it in
            # ImageStatus.navpose_topic (the NPX device is created after the data threads start).

            self.data_source_description = 'Pan Tilt'
            self.data_ref_description  =   'Pan Tilt Center Axis'

            if self.getNavPoseCb is not None:

                self.msg_if.pub_debug("Starting NPX Device IF Initialization")
                self.npx_if = NPXDeviceIF(device_info = self.device_info_dict,
                    data_source_description = self.data_source_description,
                    data_ref_description = self.data_ref_description,
                    getNavPoseCb = self.getNavPoseCb,
                    max_navpose_update_rate = self.MAX_POSITION_UPDATE_RATE,
                    msg_if = self.msg_if,
                    node_if = self.node_if,
                    settings_if = self.settings_if,
                    save_data_if = self.save_data_if,
                    connect_navpose_if = self.connect_navpose_if

                    )


            ##################################
            # Setup Pan and Tilt SVX IF Classes ####################
            # Each axis is also exposed as its own servo device, so it can be driven
            # from the Servo devices page. Both share the PTX interface's node_if,
            # settings_if, save_data_if and connect_navpose_if, so no second SaveDataIF
            # or ConnectNavPoseIF is created at the node namespace. The _pan/_tilt
            # device names keep their namespaces (<node>/<device_name>/svx) and their
            # node_if registry keys apart. Stop and speed stay shared across both
            # axes: the IQR has one stop command and one speed ratio. Neither axis
            # publishes navpose of its own; the NPX device above does.
            self.msg_if.pub_info("Launching NEPI SVX pan and tilt axis interfaces...")
            pan_device_info = copy.deepcopy(self.device_info_dict)
            pan_device_info["device_name"] = self.device_info_dict["device_name"] + "_pan"
            self.svx_pan_if = DeviceSVXIF(device_info = pan_device_info,
                                        is_rotate = True,
                                        has_pan_control = True,
                                        # Both axes are built into the unit, so they are
                                        # always connected and get no Servo Connected toggle
                                        always_connected = True,
                                        # DeviceSVXIF is unitless: its limit keys carry no _deg
                                        factoryLimits = {
                                            'min_hardstop': self.FACTORY_LIMITS_DICT['min_pan_hardstop_deg'],
                                            'max_hardstop': self.FACTORY_LIMITS_DICT['max_pan_hardstop_deg'],
                                            'min_softstop': self.FACTORY_LIMITS_DICT['min_pan_softstop_deg'],
                                            'max_softstop': self.FACTORY_LIMITS_DICT['max_pan_softstop_deg']},
                                        stopMovingCb = self.stopMoving,
                                        moveCb = self.movePan,
                                        gotoPositionCb = self.gotoPanPosition,
                                        getPositionCb = self.getPanPosition,
                                        setSoftLimitsCb = self.setPanSoftLimits,
                                        getSoftLimitsCb = self.getPanSoftLimits,
                                        setSpeedRatioCb = self.setSpeedRatio,
                                        getSpeedRatioCb = self.getSpeedRatio,
                                        goHomeCb = self.goPanHome,
                                        setHomePositionCb = self.setPanHomePosition,
                                        setHomePositionHereCb = self.setPanHomePositionHere,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if,
                                        settings_if = self.settings_if,
                                        save_data_if = self.save_data_if,
                                        connect_navpose_if = self.connect_navpose_if
                                        )

            tilt_device_info = copy.deepcopy(self.device_info_dict)
            tilt_device_info["device_name"] = self.device_info_dict["device_name"] + "_tilt"
            self.svx_tilt_if = DeviceSVXIF(device_info = tilt_device_info,
                                        is_rotate = True,
                                        has_tilt_control = True,
                                        always_connected = True,
                                        # DeviceSVXIF is unitless: its limit keys carry no _deg
                                        factoryLimits = {
                                            'min_hardstop': self.FACTORY_LIMITS_DICT['min_tilt_hardstop_deg'],
                                            'max_hardstop': self.FACTORY_LIMITS_DICT['max_tilt_hardstop_deg'],
                                            'min_softstop': self.FACTORY_LIMITS_DICT['min_tilt_softstop_deg'],
                                            'max_softstop': self.FACTORY_LIMITS_DICT['max_tilt_softstop_deg']},
                                        stopMovingCb = self.stopMoving,
                                        moveCb = self.moveTilt,
                                        gotoPositionCb = self.gotoTiltPosition,
                                        getPositionCb = self.getTiltPosition,
                                        setSoftLimitsCb = self.setTiltSoftLimits,
                                        getSoftLimitsCb = self.getTiltSoftLimits,
                                        setSpeedRatioCb = self.setSpeedRatio,
                                        getSpeedRatioCb = self.getSpeedRatio,
                                        goHomeCb = self.goTiltHome,
                                        setHomePositionCb = self.setTiltHomePosition,
                                        setHomePositionHereCb = self.setTiltHomePositionHere,
                                        msg_if = self.msg_if,
                                        node_if = self.node_if,
                                        settings_if = self.settings_if,
                                        save_data_if = self.save_data_if,
                                        connect_navpose_if = self.connect_navpose_if
                                        )
            self.msg_if.pub_info(" ... SVX pan and tilt axis interfaces running")


            # Start an ptx activity check process that kills node after some number of failed comms attempts
            self.msg_if.pub_info("Starting an activity check process")
            nepi_sdk.start_timer_process(self.HEARTBEAT_CHECK_INTERVAL, self.check_timer_callback)
            self.msg_if.pub_info("Starting an update position process")
            update_interval = float(1.0) / self.MAX_POSITION_UPDATE_RATE
            nepi_sdk.start_timer_process(update_interval, self.updateStatusHandler)
            # Initialization Complete
            self.msg_if.pub_info("Initialization Complete")
            #Set up node shutdown
            nepi_sdk.on_shutdown(self.cleanup_actions)
            # Spin forever (until object is detected)
            nepi_sdk.spin()


    def updateStatusHandler(self,timer):
        [success,self.pt_status] = self.driver_getPtStatus()
        if success == True:
            self.current_position = [self.pt_status.pan_now * self.PAN_DEG_DIR, self.pt_status.tilt_now * self.TILT_DEG_DIR]


    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Device Name: " + str(self.device_info_dict["device_name"]) + "\n"
        dev_info_string += "Path: " + str(self.device_info_dict["path"]) + "\n"
        dev_info_string += "Hardware Version: " + str(self.device_info_dict["hw_version"]) + "\n"
        dev_info_string += "Software Version: " + str(self.device_info_dict["sw_version"]) + "\n"
        dev_info_string += "Serial Number: " + str(self.device_info_dict["serial_number"]) + "\n"
        self.msg_if.pub_info(dev_info_string)

    def getNavPoseCb(self):
        pan_deg, tilt_deg = self.current_position
        navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        navpose_dict['has_orientation'] = True
        navpose_dict['time_orientation'] = nepi_utils.get_time()
        navpose_dict['roll_deg'] = 0.0
        navpose_dict['yaw_deg'] = pan_deg * self.PAN_DEG_DIR
        navpose_dict['pitch_deg'] = tilt_deg * self.TILT_DEG_DIR
        return navpose_dict

    #**********************
    # Device setting functions


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
        self.clearGotoTargets()
        self.driver_stopMotion()

    def clearGotoTargets(self):
        # Stop and jog command both axes to where they are now, so neither axis's
        # last goto target is still meaningful to resend.
        with self.goto_lock:
            self.last_goto_pan_deg = None
            self.last_goto_tilt_deg = None

    def checkResetPanStopTrigger(self):
        stop_trig = copy.deepcopy(self.pan_stop_trig)
        self.pan_stop_trig = False
        return stop_trig

    def checkResetTiltStopTrigger(self):
        stop_trig = copy.deepcopy(self.tilt_stop_trig)
        self.tilt_stop_trig = False
        return stop_trig

    def movePan(self, direction, duration):
        self.clearGotoTargets()
        success = self.driver_jogPan(direction, duration)



    def moveTilt(self, direction, duration):
            self.clearGotoTargets()
            success = self.driver_jogTilt(direction, duration)

    def setSoftLimits(self,pan_min,pan_max,tilt_min,tilt_max):
        pfl_min = self.FACTORY_LIMITS_DICT['min_pan_softstop_deg']
        pfl_max = self.FACTORY_LIMITS_DICT['max_pan_softstop_deg']
        tfl_min = self.FACTORY_LIMITS_DICT['min_tilt_softstop_deg']
        tfl_max = self.FACTORY_LIMITS_DICT['max_tilt_softstop_deg']

        if pan_max > pan_min:
            if pan_min < pfl_min:
                pan_min = pfl_min
            if pan_max > pfl_max:
                pan_max = pfl_max


        if tilt_max > tilt_min:
            if tilt_min < tfl_min:
                tilt_min = tfl_min
            if tilt_max > tfl_max:
                tilt_max = tfl_max  

        self.limits_dict['min_pan_softstop_deg'] = pan_min
        self.limits_dict['max_pan_softstop_deg'] = pan_max
        self.limits_dict['min_tilt_softstop_deg'] = tilt_min
        self.limits_dict['max_tilt_softstop_deg'] = tilt_max
              
    def getSoftLimits(self):
        pan_min = self.limits_dict['min_pan_softstop_deg']
        pan_max = self.limits_dict['max_pan_softstop_deg']
        tilt_min = self.limits_dict['min_tilt_softstop_deg']
        tilt_max = self.limits_dict['max_tilt_softstop_deg']
        return pan_min,pan_max,tilt_min,tilt_max


    def setSpeedRatio(self, ratio):
        # TODO: Limits checking and driver unit conversion?
        self.speed_ratio = ratio


    def getSpeedRatio(self):
        # TODO: Driver unit conversion?
        ratio = self.driver_getSpeedRatio()
        return ratio
          

    def getPosition(self):
        return self.current_position
        

    def gotoPosition(self, pan_deg, tilt_deg):
        #self.msg_if.pub_debug("Got GoTo Position: " + str([pan_deg,tilt_deg]), throttle_s = 1)
        with self.goto_lock:
            self.last_goto_pan_deg = pan_deg
            self.last_goto_tilt_deg = tilt_deg
            self.driver_moveToPosition(pan_deg * self.PAN_DEG_DIR, tilt_deg * self.TILT_DEG_DIR)

    def gotoPanPosition(self, pan_deg):
        # The IQR commands both axes in one write: hold tilt at its last goto
        # target, or where it is now if it has none, so a pan goto leaves tilt alone.
        with self.goto_lock:
            tilt_deg = self.last_goto_tilt_deg
            if tilt_deg is None:
                tilt_deg = self.current_position[1]
            self.last_goto_pan_deg = pan_deg
            self.driver_moveToPosition(pan_deg * self.PAN_DEG_DIR, tilt_deg * self.TILT_DEG_DIR)

    def gotoTiltPosition(self, tilt_deg):
        with self.goto_lock:
            pan_deg = self.last_goto_pan_deg
            if pan_deg is None:
                pan_deg = self.current_position[0]
            self.last_goto_tilt_deg = tilt_deg
            self.driver_moveToPosition(pan_deg * self.PAN_DEG_DIR, tilt_deg * self.TILT_DEG_DIR)

      
    def goHome(self):
        # Through gotoPosition so the home targets are recorded as both axes' last
        # goto targets, like any other goto.
        self.gotoPosition(self.home_pan_deg, self.home_tilt_deg)

    def setHomePosition(self, pan_deg, tilt_deg):
        self.home_pan_deg = pan_deg
        self.home_tilt_deg = tilt_deg

    def setHomePositionHere(self):
        pan_deg, tilt_deg = self.getPosition()
        self.home_pan_deg = pan_deg * self.PAN_DEG_DIR
        self.home_tilt_deg = tilt_deg * self.TILT_DEG_DIR


    def resetDevice(self):
        self.speed_ratio = 0.5
        self.limits_dict['min_pan_softstop_deg'] = self.FACTORY_LIMITS_DICT['min_pan_softstop_deg']
        self.limits_dict['max_pan_softstop_deg'] = self.FACTORY_LIMITS_DICT['max_pan_softstop_deg']
        self.limits_dict['min_tilt_softstop_deg'] = self.FACTORY_LIMITS_DICT['min_tilt_softstop_deg']
        self.limits_dict['max_tilt_softstop_deg'] = self.FACTORY_LIMITS_DICT['max_tilt_softstop_deg']

    def driver_reportsPosition(self):
        return True

    #######################
    ### SVX IF Functions (one servo device per axis)
    # Stop, jog, speed and the per-axis gotos are shared with the PTX IF functions
    # above. These adapt the node's two-axis state to the single-axis SVX callbacks.

    def getPanPosition(self):
        return self.current_position[0]

    def getTiltPosition(self):
        return self.current_position[1]

    def setPanSoftLimits(self, min_deg, max_deg):
        # Through setSoftLimits so both paths share one clamp; tilt is resent as is.
        pan_min, pan_max, tilt_min, tilt_max = self.getSoftLimits()
        self.setSoftLimits(min_deg, max_deg, tilt_min, tilt_max)

    def getPanSoftLimits(self):
        pan_min, pan_max, tilt_min, tilt_max = self.getSoftLimits()
        return [pan_min, pan_max]

    def setTiltSoftLimits(self, min_deg, max_deg):
        pan_min, pan_max, tilt_min, tilt_max = self.getSoftLimits()
        self.setSoftLimits(pan_min, pan_max, min_deg, max_deg)

    def getTiltSoftLimits(self):
        pan_min, pan_max, tilt_min, tilt_max = self.getSoftLimits()
        return [tilt_min, tilt_max]

    def goPanHome(self):
        self.gotoPanPosition(self.home_pan_deg)

    def goTiltHome(self):
        self.gotoTiltPosition(self.home_tilt_deg)

    def setPanHomePosition(self, pan_deg):
        self.home_pan_deg = pan_deg

    def setTiltHomePosition(self, tilt_deg):
        self.home_tilt_deg = tilt_deg

    def setPanHomePositionHere(self):
        self.home_pan_deg = self.current_position[0]

    def setTiltHomePositionHere(self):
        self.home_tilt_deg = self.current_position[1]

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
                        if rcvdBuf is not None:
                            if len(rcvdBuf) >= 19:
                                pt_status.id = rcvdBuf[0]
                                pt_status.serial_num = f"SN{int(rcvdBuf[1])}"
                                pt_status.hw_version = f"v{int((rcvdBuf[2] & 0xff00) >> 8)}.{int(rcvdBuf[2] & 0x00ff)}"
                                pt_status.bd_version = f"v{int((rcvdBuf[3] & 0xff00) >> 8)}.{int(rcvdBuf[3] & 0x00ff)}"
                                pt_status.sw_version = (f"v{int((rcvdBuf[4] & 0xf000) >> 12)}."
                                                    f"{int((rcvdBuf[4] & 0x0f00) >> 8)}.{int(rcvdBuf[4] & 0x00ff)}")
                                pt_status.set_zero = rcvdBuf[5]
                                pt_status.speed = rcvdBuf[6]
                                pt_status.pan_goal = c_int16(rcvdBuf[7]).value / 100
                                pt_status.tilt_goal = c_int16(rcvdBuf[8]).value / 100
                                pt_status.reserved = rcvdBuf[9]
                                pt_status.driver_ec = rcvdBuf[10]
                                pt_status.encoder_ec = rcvdBuf[11]
                                pt_status.pan_now = c_int16(rcvdBuf[12]).value / 100
                                pt_status.tilt_now = c_int16(rcvdBuf[13]).value / 100
                                pt_status.pan_temp = c_int16(rcvdBuf[14]).value / 10.0
                                pt_status.tilt_temp = c_int16(rcvdBuf[15]).value / 10.0
                                pt_status.pan_raw = c_int16(rcvdBuf[16]).value
                                pt_status.tilt_raw = c_int16(rcvdBuf[17]).value
                                pt_status.loop_ec = rcvdBuf[18]
                                pt_status.loop_time = rcvdBuf[19]
                                success = True
            else:
                self.msg_if.pub_info("Modebus Master is None. Failed to connect")
            return success,pt_status
                

    def driver_stopMotion(self):
        pan = self.current_position[0]
        tilt = self.current_position[1]
        try:
            self.driver_moveToPosition(pan, tilt)
            return True
        except Exception as e:
            self.msg_if.pub_warn("Failed to stop motion: " + str(e))
            return False


    def driver_getPosition(self):
        pan = self.current_position[0]
        tilt = self.current_position[1]
        return pan, tilt

    def driver_moveToPosition(self,pan, tilt):
        # Returns True once the command is sent. The jog functions run their timed
        # stop only on success, so returning None sent every timed jog on to the soft limit.
        if pan < -60.0 or pan > 60.0:
            return False
        if tilt < -60.0 or tilt > 60.0:
            return False
        speed_count = self.ratio2speed_count(self.speed_ratio)
        with self.pt_status_lock:
            if self.modbus_master is None:
                return False
            sendBuf = [speed_count, round(pan*100), round(tilt*100)]
            self.modbus_master.set_multiple_registers(self.addr_int, 0x0006, sendBuf)
            nepi_sdk.sleep(0.01)
        return True

    def driver_getSpeedRatio(self, axis_str = '#'):
        method_name = sys._getframe().f_code.co_name
        self.msg_if.pub_debug("Got speed count: " + str(self.pt_status.speed)) 
        ratio = self.speed_count2ratio(self.pt_status.speed)
        self.msg_if.pub_debug("Returning speed ratio: " + str(ratio)) 
        return ratio
        

      
    def driver_jogPan(self, direction, duration):
        pan_min = self.limits_dict['min_pan_softstop_deg']
        pan_max = self.limits_dict['max_pan_softstop_deg']
        direction = direction * self.PAN_DEG_DIR
        if direction < 0:
            pan_deg = pan_min
        else:
            pan_deg = pan_max

        tilt_deg = self.current_position[1]
        success = self.driver_moveToPosition(pan_deg, tilt_deg)

        if success:
            if duration > 0:
                start_time = nepi_utils.get_time()
                stop = False
                while stop == False:
                    time_check = (nepi_utils.get_time() - start_time) > duration
                    stop = time_check == True or self.checkResetPanStopTrigger() == True
                    nepi_sdk.sleep(0.1)
                success = self.driver_stopMotion()


    def driver_jogTilt(self, direction, duration):
        tilt_min = self.limits_dict['min_tilt_softstop_deg']
        tilt_max = self.limits_dict['max_tilt_softstop_deg']
        direction = direction * self.TILT_DEG_DIR
        if direction < 0:
            tilt_deg = tilt_min
        else:
            tilt_deg = tilt_max

        pan_deg = self.current_position[0]
        success = self.driver_moveToPosition(pan_deg, tilt_deg)

        if success:
            if duration > 0:
                start_time = nepi_utils.get_time()
                stop = False
                while stop == False:
                    time_check = (nepi_utils.get_time() - start_time) > duration
                    stop = time_check == True or self.checkResetTiltStopTrigger() == True
                    nepi_sdk.sleep(0.1)
                success = self.driver_stopMotion()


    #######################
    ### Driver Util Functions

    def check_timer_callback(self,timer):
        if self.modbus_master is None:
            return
        else:
            success = False
            try:
                # Try to request pt_status
                #self.msg_if.pub_info("Connection Check requesting info for device on port: " + self.device_path)
                [success,pt_status] = self.driver_getPtStatus()
                #self.msg_if.pub_info("Heartbeat check got status: " + str(pt_status))
            except Exception as e:
                self.msg_if.pub_warn("Something went wrong with connecting to serial port at: " + self.device_path + "(" + str(e) + ")" )
        if success:
            self.self_check_counter = 0 # reset comms failure count
        else:
            self.self_check_counter = self.self_check_counter + 1 # increment counter

        #self.msg_if.pub_debug("Current failed comms count: " + str(self.self_check_counter))
        if self.self_check_counter > self.self_check_count:  # Crashes node if set above limit??
            self.msg_if.pub_warn("Shutting down device: " +  self.addr_str + " on port " + self.device_path)
            self.msg_if.pub_warn("Too many comm failures")
            nepi_sdk.signal_shutdown("To many comm failures")   

    
    ### Function to try and connect to device at given port and baudrate
    def connect(self):
        success = False
        self.connect_attempts += 1
        port_check = True #self.check_port(self.device_path)
        pt_status = None
        if port_check is True:
            try:
                # Try and open serial port
                self.msg_if.pub_info("Opening mod_bus port " + self.device_path + " with baudrate: " + self.baud_str + " with addr: " + self.addr_str)
                self.modbus_master = ModbusRTUMaster(self.device_path, self.baud_int)
                self.msg_if.pub_info("Modbus port opened")
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Something went wrong with connecting to serial port at: " + self.device_path + "(" + str(e) + ")" )
            if success == True:
                nepi_sdk.sleep(0.1)
                success = False
                # Send Message
                self.msg_if.pub_info("Connect process requesting info for device on port: " + self.device_path)

                [success,pt_status] = self.driver_getPtStatus()
                if success:
                    self.msg_if.pub_info("Connected to device on port: " +  self.device_path)
                    self.msg_if.pub_debug("Got status on connection: " + str(pt_status))
                    self.pt_status = pt_status
                    # Update serial, hardware, and software status values
                    self.serial_num = pt_status.serial_num
                    self.hw_version = pt_status.hw_version
                    self.sw_version = pt_status.sw_version
                    self.fw_version = pt_status.sw_version
                    success = True
                    # Factory Reset Device
                else:
                    self.msg_if.pub_info("Failed to get status from device on port: " +  self.device_path)
    

                self.driver_moveToPosition(0.0, 0.0)

        return success


 



    ### Function for checking if port is available
    def check_port(self,device_path):
        success = False
        ports = serial.tools.list_ports.comports()
        for loc, desc, hwid in sorted(ports):
            if loc == device_path:
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





