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
import time
import sys
import inspect
import math
import glob
import copy

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_nav

from nepi_api.connect_device_if_svx import ConnectSVXDeviceIF
from nepi_api.device_if_ptx import PTXActuatorIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'PTX_SERVOS' # Use in display menus
FILE_TYPE = 'NODE'


class ServosPTXNode:
    MAX_POSITION_UPDATE_RATE = 5

    #CAP_SETTINGS = nepi_settings.NONE_CAP_SETTINGS
    CAP_SETTINGS = dict(
      pan_servo = {"type":"Discrete","name":"pan_servo","options":["None"]},
      pan_connected = {"type":"Bool","name":"pan_connected","disabled":True},
      tilt_servo = {"type":"Discrete","name":"tilt_servo","options":["None"]},
      tilt_connected = {"type":"Bool","name":"tilt_connected","disabled":True},
    )
    # cap_settings is a per-instance deep copy made in __init__. Binding it to
    # CAP_SETTINGS here would alias the two names to one dict, so every options
    # update would mutate the factory table it is supposed to be derived from.
    cap_settings = None


    pan_connect_if = None
    pan_options = ['None']
    pan_selected = 'None'
    pan_connected = False

    tilt_connect_if = None
    tilt_options = ['None']
    tilt_selected = 'None'
    tilt_connected = False

    PAN_DEG_DIR = -1
    TILT_DEG_DIR = -1

    LIMITS_DICT = dict()
    LIMITS_DICT['max_pan_hardstop_deg'] = 175
    LIMITS_DICT['min_pan_hardstop_deg'] = -175
    LIMITS_DICT['max_tilt_hardstop_deg'] = 175
    LIMITS_DICT['min_tilt_hardstop_deg'] = -175
    LIMITS_DICT['max_pan_softstop_deg'] = 165
    LIMITS_DICT['min_pan_softstop_deg'] = -165
    LIMITS_DICT['max_tilt_softstop_deg'] = 74
    LIMITS_DICT['min_tilt_softstop_deg'] = -74


    PT_DIRECTION_POSITIVE = 1
    PT_DIRECTION_NEGATIVE = -1

    device_info_dict = dict(device_name = "Servos Pan Tilt",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")
    
    # Initialize some parameters
    serial_num = "Unknown"
    hw_version = "Unknown"
    sw_version = "Unknown"
    ptx_if = None

    
    connected = False

    current_position = [0.0,0.0]
    position_times = [0.0,0.0]

    speed_ratio = 0.5

    # Placeholder reported only before either axis has ever connected. It is deliberately
    # a poor speed rather than a plausible one: a wrong-but-believable max would scale
    # every downstream dps value silently, where a visibly slow axis gets noticed.
    UNKNOWN_SPEED_MAX_DPS = 1
    # Bounds on an operator-supplied max. A standard hobby servo runs a few hundred dps
    # (3001HB is roughly 315-400), so the range has to cover that, not stop below it.
    MIN_SPEED_MAX_DPS = 1
    MAX_SPEED_MAX_DPS = 1000

    pan_speed_max_dps = UNKNOWN_SPEED_MAX_DPS
    tilt_speed_max_dps = UNKNOWN_SPEED_MAX_DPS

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

        self.cap_settings = copy.deepcopy(self.CAP_SETTINGS)

        # Softstop limits are tracked here, not on the servo. The SVX device has no
        # soft-limit command -- SVXCapabilitiesQuery.has_limit_control is always False --
        # so this node owns the pan/tilt softstops and seeds them from LIMITS_DICT.
        self.soft_limits = [self.LIMITS_DICT['min_pan_softstop_deg'],
                            self.LIMITS_DICT['max_pan_softstop_deg'],
                            self.LIMITS_DICT['min_tilt_softstop_deg'],
                            self.LIMITS_DICT['max_tilt_softstop_deg']]

        # auto_select_enabled = False on both axes. Auto-select picks the first
        # discovered SVX device, and both interfaces discover the same list, so with
        # it on pan and tilt both land on the same servo channel -- and an operator's
        # saved channel is overwritten as soon as one axis' choice is discovered a
        # cycle later than the other's. A pan/tilt pairing is a choice only the
        # operator can make, so nothing is selected until they make it.
        self.pan_connect_if =  ConnectSVXDeviceIF(connect_name = 'pan_servo',
                namespace = None,
                statusCb = None,
                auto_select_enabled = False,
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = self.msg_if,
                node_if = None
        )

        self.tilt_connect_if =  ConnectSVXDeviceIF(connect_name = 'tilt_servo',
                namespace = None,
                statusCb = None,
                auto_select_enabled = False,
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = self.msg_if,
                node_if = None
        )

        ################################################

        # Adopt whatever selection each interface restored from its config file
        # BEFORE building the cap settings. updateConnectsHandler does not run until
        # a second after the PTX interface is up, so without this the settings the
        # PTX interface is constructed with -- and the Discrete options a restored
        # setting is validated against -- are both still 'None', and SettingsIF.init()
        # rejects the operator's saved selection as an invalid value on every boot.
        # The restored topic is added to the options list because it has not been
        # discovered yet either; updateConnectsHandler replaces both lists with the
        # discovered ones on its first cycle.
        self.pan_selected = self.pan_connect_if.get_selected_topic()
        self.tilt_selected = self.tilt_connect_if.get_selected_topic()
        self.pan_options = ['None']
        if self.pan_selected != 'None':
            self.pan_options.append(self.pan_selected)
        self.tilt_options = ['None']
        if self.tilt_selected != 'None':
            self.tilt_options.append(self.tilt_selected)

        # Initialize settings
        cap_settings = self.getCapSettings()
        factory_settings = self.getFactorySettings()
            

        # Launch the PTX interface --  this takes care of initializing all the ptx settings from config. file, subscribing and advertising topics and services, etc.
        # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
        self.msg_if.pub_info("Launching NEPI PTX () interface...")

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

        self.home_pan_deg = 0.0
        self.home_tilt_deg = 0.0

        self.device_info_dict = self.getDeviceInfo()
        
        self.ptx_if = PTXActuatorIF(device_info = self.device_info_dict,
                                    capSettings = cap_settings,
                                    factorySettings = factory_settings,
                                    getCapSettingsFunction= self.getCapSettings,
                                    settingUpdateFunction=self.settingUpdateFunction,
                                    getSettingsFunction=self.getSettings,
                                    factoryControls = self.FACTORY_CONTROLS,
                                    factoryLimits = self.LIMITS_DICT,
                                    stopMovingCb = self.stopMoving,
                                    movePanCb = self.movePan,
                                    moveTiltCb = self.moveTilt,
                                    movePanSpeedRatioCb = self.movePanSpeedRatio,
                                    moveTiltSpeedRatioCb = self.moveTiltSpeedRatio,
                                    getSoftLimitsCb = self.getSoftLimits,
                                    setSoftLimitsCb = self.setSoftLimits,
                                    getSpeedMaxCb = self.getSpeedMax,
                                    setSpeedMaxCb = None, #self.setSpeedMax,
                                    getSpeedRatioCb = self.getSpeedRatio,
                                    setSpeedRatioCb = self.setSpeedRatio,
                                    getPanSpeedRatioCb = self.getPanSpeedRatio,
                                    setPanSpeedRatioCb = self.setPanSpeedRatio,
                                    getTiltSpeedRatioCb = self.getTiltSpeedRatio,
                                    setTiltSpeedRatioCb = self.setTiltSpeedRatio,
                                    getPositionCb = self.getPosition,
                                    getPositionTimesCb = self.getPositionTimes,
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
        nepi_sdk.start_timer_process(1, self.updateConnectsHandler, oneshot = True)
        update_interval = float(1.0) / self.MAX_POSITION_UPDATE_RATE
        nepi_sdk.start_timer_process(update_interval, self.updatePositionHandler, oneshot = True)
        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever (until object is detected)
        nepi_sdk.spin()


    def getDeviceInfo(self):
        # These five keys are the contract: device_if_ptx.py reads device_name,
        # path, serial_number, hw_version and sw_version straight off this dict
        # with no .get() and no guard, so a differently-keyed dict is a KeyError
        # at IF construction, not a missing status field. Keep them in sync with
        # the device_info_dict class attribute above.
        #
        # There is no hardware to interrogate here. This node is a composite of
        # two SVX servo devices, each of which reports its own serial/versions
        # through its own SVX interface, so identity comes from the node name
        # discovery assigned (via get_device_alias) and the rest stay Unknown.
        # The other PTX drivers read device_name from drv_dict['DEVICE_DICT'],
        # which the servos discovery does not build -- it launches one node
        # unconditionally rather than one per discovered hardware path.
        dev_info = dict()
        dev_info["device_name"] = self.node_name
        dev_info["path"] = ""
        dev_info["serial_number"] = self.serial_num
        dev_info["hw_version"] = self.hw_version
        dev_info["sw_version"] = self.sw_version
        return dev_info
    
    def updateConnectsHandler(self,timer):

        # The interface owns the selection: it persists it and rejects a topic that has
        # not been discovered yet. Adopt what it reports rather than pushing the local
        # copy back in. Pushing back here re-sent the startup default 'None' on the first
        # cycle, which the interface then persisted and save_config()'d.
        # settingUpdateFunction is the only place a new selection is pushed in.
        if self.pan_connect_if is not None:
            # get_available_topics returns the interface's live internal list, so copy
            # before filtering. Editing it in place corrupts the selector state.
            pan_options = list(self.pan_connect_if.get_available_topics())
            self.pan_selected = self.pan_connect_if.get_selected_topic()
            if self.tilt_selected != None and self.tilt_selected in pan_options:
                pan_options.remove(self.tilt_selected)
            self.pan_options = ['None'] + pan_options
            self.pan_connected = self.pan_connect_if.check_connection()
            if self.pan_connected == True:
                # Keep the last known max if the device reports nothing usable, and do not
                # wipe it back to the placeholder on a disconnect -- see getPanSpeedMax.
                pan_speed_max_dps = self.pan_connect_if.get_max_speed_dps()
                if pan_speed_max_dps is not None and pan_speed_max_dps > 0:
                    self.pan_speed_max_dps = pan_speed_max_dps

        if self.tilt_connect_if is not None:
            tilt_options = list(self.tilt_connect_if.get_available_topics())
            self.tilt_selected = self.tilt_connect_if.get_selected_topic()
            if self.pan_selected != None and self.pan_selected in tilt_options:
                tilt_options.remove(self.pan_selected)
            self.tilt_options = ['None'] + tilt_options
            self.tilt_connected = self.tilt_connect_if.check_connection()
            if self.tilt_connected == True:
                tilt_speed_max_dps = self.tilt_connect_if.get_max_speed_dps()
                if tilt_speed_max_dps is not None and tilt_speed_max_dps > 0:
                    self.tilt_speed_max_dps = tilt_speed_max_dps

        nepi_sdk.start_timer_process(1, self.updateConnectsHandler, oneshot = True)


    def updatePositionHandler(self,timer):
        stime=nepi_utils.get_time()
        # Positions come from the two servo interfaces, one axis each, and are stored in
        # the node frame -- the same frame gotoPosition() takes and getPosition() reports.
        # The interfaces report in the device frame, so apply the axis direction here.
        if self.pan_connect_if is not None and self.pan_connected == True:
            pan_deg = self.pan_connect_if.get_servo_position()
            if pan_deg is not None:
                self.current_position[0] = pan_deg * self.PAN_DEG_DIR
                self.position_times[0] = nepi_utils.get_time()
        if self.tilt_connect_if is not None and self.tilt_connected == True:
            tilt_deg = self.tilt_connect_if.get_servo_position()
            if tilt_deg is not None:
                self.current_position[1] = tilt_deg * self.TILT_DEG_DIR
                self.position_times[1] = nepi_utils.get_time()
        #self.msg_if.pub_info("Got current position :" + str(self.current_position))
        #self.msg_if.pub_info("Got position times :" + str(self.position_times))
        gtime = nepi_utils.get_time() - stime
        next_delay = max(0.01, float(1.0) / self.MAX_POSITION_UPDATE_RATE - gtime)
        nepi_sdk.start_timer_process(next_delay, self.updatePositionHandler, oneshot = True)


       
    def getNavPoseDict(self):
        pan_deg, tilt_deg = self.current_position
        navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
        navpose_dict['has_orientation'] = True
        navpose_dict['time_oreantation'] = nepi_utils.get_time()
        navpose_dict['roll_deg'] = 0.0
        # current_position is already in the node frame; the axis direction was applied
        # when it was sampled. Applying it again here inverted both axes.
        navpose_dict['yaw_deg'] = pan_deg
        navpose_dict['pitch_deg'] = tilt_deg
        return navpose_dict


    #**********************
    # Device setting functions


    def getCapSettings(self):
        self.cap_settings['pan_servo']['options'] = self.pan_options
        self.cap_settings['tilt_servo']['options'] = self.tilt_options
        return self.cap_settings

    def getFactorySettings(self):
        settings = self.getSettings()
        return settings



    def getSettings(self):
        settings = dict()
        for setting_name in self.cap_settings.keys():
            cap_setting = self.cap_settings[setting_name]
            setting = dict()
            setting["name"] = setting_name
            setting["type"] = cap_setting['type']
            val = None
            if setting_name == 'pan_servo':
                val = self.pan_selected
            if setting_name == 'pan_connected':
                val = self.pan_connected
            if setting_name == 'tilt_servo':
                val = self.tilt_selected
            if setting_name == 'tilt_connected':
                val = self.tilt_connected
            setting['value'] = str(val)
            settings[setting_name] = setting
        return settings


    def settingUpdateFunction(self,setting):
        success = False
        setting_str = str(setting)
        [setting_name, s_type, data] = nepi_settings.get_data_from_setting(setting)
        setting_updated = False
        msg = "Success"
        # This is the only place a selection is pushed into a connect interface.
        # The interface may reject an undiscovered topic, so adopt what it returns.
        if setting_name in self.cap_settings.keys():
            if setting_name == 'pan_servo':
                if self.pan_connect_if is not None:
                    self.pan_selected = self.pan_connect_if.set_selected_topic(data)
                else:
                    self.pan_selected = data
                setting_updated = True
            if setting_name == 'tilt_servo':
                if self.tilt_connect_if is not None:
                    self.tilt_selected = self.tilt_connect_if.set_selected_topic(data)
                else:
                    self.tilt_selected = data
                setting_updated = True
        if setting_updated is False:
            msg = (self.node_name  + " Setting name" + setting_str + " is not supported")
        else:
            success = True
        return success, msg



    #######################
    ### PTX IF Functions

    def stopMoving(self):
        success = False
        if self.pan_connect_if is not None:
            success = self.pan_connect_if.stop_moving()
        if self.tilt_connect_if is not None:
            success = self.tilt_connect_if.stop_moving() or success
        return success

    def movePan(self, direction, duration):
        if self.pan_connect_if is not None:
            # Normalize and apply the axis direction the same way movePanSpeedRatio does.
            # Passing the caller's direction straight through made the two jog paths
            # disagree on sign, so a jog reversed when a speed was supplied.
            direction = self.PT_DIRECTION_POSITIVE if direction == 1 else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.PAN_DEG_DIR
            success = self.pan_connect_if.move_direction(direction)
            if success:
                if duration > 0:
                    nepi_sdk.sleep(duration)
                    self.pan_connect_if.stop_moving()


    def moveTilt(self, direction, duration):
        if self.tilt_connect_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == 1 else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.TILT_DEG_DIR
            success = self.tilt_connect_if.move_direction(direction)
            if success:
                if duration > 0:
                    nepi_sdk.sleep(duration)
                    self.tilt_connect_if.stop_moving()


    def movePanSpeedRatio(self, direction, speed_ratio, duration):
        if self.pan_connect_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == 1 else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.PAN_DEG_DIR
            speed_dps = speed_ratio * self.pan_speed_max_dps
            success = self.pan_connect_if.move_direction_speed(direction,speed_dps)
            if success:
                if duration > 0:
                    nepi_sdk.sleep(duration)
                    self.pan_connect_if.stop_moving()


    def moveTiltSpeedRatio(self, direction, speed_ratio, duration):
        if self.tilt_connect_if is not None:
            direction = self.PT_DIRECTION_POSITIVE if direction == 1 else self.PT_DIRECTION_NEGATIVE
            direction = direction * self.TILT_DEG_DIR
            speed_dps = speed_ratio * self.tilt_speed_max_dps
            success = self.tilt_connect_if.move_direction_speed(direction,speed_dps)
            if success:
                if duration > 0:
                    nepi_sdk.sleep(duration)
                    self.tilt_connect_if.stop_moving()


    def setSoftLimits(self, min_pan,max_pan,min_tilt,max_tilt):
        # Stored in the node frame, the frame the caller and getSoftLimits() both use.
        # The SVX device has no soft-limit command, so there is nothing to push down and
        # nothing to convert; the earlier axis-direction swap here only mattered for a
        # driver-side write that never existed.
        if (min_pan < max_pan) and (min_tilt < max_tilt):
            self.soft_limits = [min_pan, max_pan, min_tilt, max_tilt]

    def getSoftLimits(self):
        return list(self.soft_limits)



    # These report the LAST KNOWN max while an axis is disconnected rather than the
    # UNKNOWN_SPEED_MAX_DPS placeholder. PTXActuatorIF reads getSpeedMax() to fill
    # speed_max_dps, and every dps value downstream is scaled by it, so handing back the
    # placeholder during a reconnect blip silently rescales the whole speed axis.
    def getPanSpeedMax(self):
        if self.pan_connect_if is not None and self.pan_connected == True:
            max_speed_dps = self.pan_connect_if.get_max_speed_dps()
            if max_speed_dps is not None and max_speed_dps > 0:
                self.pan_speed_max_dps = max_speed_dps
        return self.pan_speed_max_dps

    def getTiltSpeedMax(self):
        if self.tilt_connect_if is not None and self.tilt_connected == True:
            max_speed_dps = self.tilt_connect_if.get_max_speed_dps()
            if max_speed_dps is not None and max_speed_dps > 0:
                self.tilt_speed_max_dps = max_speed_dps
        return self.tilt_speed_max_dps


    def getSpeedMax(self):
        max_speed_dps = self.UNKNOWN_SPEED_MAX_DPS
        if self.pan_connect_if is not None:
            max_speed_dps = max(max_speed_dps, self.getPanSpeedMax())
        if self.tilt_connect_if is not None:
            max_speed_dps = max(max_speed_dps, self.getTiltSpeedMax())
        return max_speed_dps

    def setSpeedMax(self, speed):
        # The old guard silently dropped anything outside 10-40 dps, which is well under
        # what a standard hobby servo does (a 3001HB is roughly 315-400 dps), so a correct
        # max could never be set and the axis was capped at a fraction of the hardware.
        # Out-of-range values are now reported instead of vanishing.
        if speed < self.MIN_SPEED_MAX_DPS or speed > self.MAX_SPEED_MAX_DPS:
            self.msg_if.pub_warn("Max speed " + str(speed) + " dps outside supported range " +
                                 str(self.MIN_SPEED_MAX_DPS) + "-" + str(self.MAX_SPEED_MAX_DPS) +
                                 " dps... ignoring")
            return
        if self.pan_connect_if is not None and self.pan_connected == True:
            self.pan_connect_if.set_max_speed_dps(speed)
            self.pan_speed_max_dps = speed
        if self.tilt_connect_if is not None and self.tilt_connected == True:
            self.tilt_connect_if.set_max_speed_dps(speed)
            self.tilt_speed_max_dps = speed




    def setPanSpeedRatio(self, ratio):
        # A 0-1 ratio, not degrees per second. Sending it to set_max_speed_dps() set the
        # servo maximum to a fraction of a degree per second instead of scaling the speed.
        if self.pan_connect_if is not None and self.pan_connected == True:
            self.pan_connect_if.set_max_speed_ratio(ratio)
        self.speed_ratio = ratio

    def setTiltSpeedRatio(self, ratio):
        if self.tilt_connect_if is not None and self.tilt_connected == True:
            self.tilt_connect_if.set_max_speed_ratio(ratio)
        self.speed_ratio = ratio

    def setSpeedRatio(self, ratio):
        self.setPanSpeedRatio(ratio)
        self.setTiltSpeedRatio(ratio)



    def getPanSpeedRatio(self):
        # Never returns None: PTXActuatorIF applies math.floor() to this directly.
        ratio = None
        if self.pan_connect_if is not None and self.pan_connected == True:
            ratio = self.pan_connect_if.get_speed_ratio()
        if ratio is None:
            ratio = self.speed_ratio
        return ratio

    def getTiltSpeedRatio(self):
        ratio = None
        if self.tilt_connect_if is not None and self.tilt_connected == True:
            ratio = self.tilt_connect_if.get_speed_ratio()
        if ratio is None:
            ratio = self.speed_ratio
        return ratio

    def getSpeedRatio(self):
        return max(self.getPanSpeedRatio(), self.getTiltSpeedRatio())

    def getPosition(self):
        return list(self.current_position)

    def getPositionTimes(self):
        return self.position_times


    def gotoPosition(self, pan_deg, tilt_deg):
        self.pan_connect_if.goto_position(pan_deg * self.PAN_DEG_DIR)
        self.tilt_connect_if.goto_position(tilt_deg * self.TILT_DEG_DIR)

    def gotoPanPosition(self, pan_deg):
        self.pan_connect_if.goto_position(pan_deg * self.PAN_DEG_DIR)

    def gotoTiltPosition(self, tilt_deg):
        self.tilt_connect_if.goto_position(tilt_deg * self.TILT_DEG_DIR)
        
    def goHome(self):
        self.gotoPanPosition(self.home_pan_deg)
        self.gotoTiltPosition(self.home_tilt_deg)

    def setHomePosition(self, pan_deg, tilt_deg):
        self.home_pan_deg = pan_deg * self.PAN_DEG_DIR
        self.home_tilt_deg = tilt_deg * self.TILT_DEG_DIR

    def setHomePositionHere(self):
        pan_deg, tilt_deg = self.getPosition()
        self.home_pan_deg = pan_deg * self.PAN_DEG_DIR
        self.home_tilt_deg = tilt_deg * self.TILT_DEG_DIR 

    def resetDevice(self):
        pass






    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        # This node owns no serial port; the servo drivers do. Leaving the axes stopped
        # is the only cleanup it can do, and it matters because a jog runs until stopped.
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        self.stopMoving()


if __name__ == '__main__':
	node = ServosPTXNode()
