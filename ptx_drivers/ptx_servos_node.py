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
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_nav

from nepi_interfaces.msg import DeviceSVXStatus

from nepi_api.connect_device_if_svx import ConnectSVXDeviceIF
from nepi_api.device_if_ptx import PTXActuatorIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'PTX_SERVOS' # Use in display menus
FILE_TYPE = 'NODE'


class ServosPTXNode:
    MAX_POSITION_UPDATE_RATE = 5

    # The nepi_controls settings dict, built by initSettingsDict in __init__ and
    # rebuilt from the selector state below by refreshSettingsDict.
    settings_dict = None
    # The selector state last pushed into the PTX interface's SettingsIF, so
    # updateConnectsHandler only pushes when it changes. None until the first push.
    pushed_settings_state = None


    pan_connect_if = None
    pan_options = ['None']
    pan_selected = 'None'
    pan_connected = False

    tilt_connect_if = None
    tilt_options = ['None']
    tilt_selected = 'None'
    tilt_connected = False

    # Each discovered servo's servo_connected flag, keyed by its SVX namespace (the
    # form get_available_topics returns). The connect interfaces only subscribe to the
    # one servo selected on their axis, so the flags of the other candidates come from
    # one status subscriber per available servo, kept in step by updateServoStatusSubs.
    # Per-instance dicts made in __init__: a dict bound here would be one object
    # shared by every instance of the class.
    servo_status_subs_dict = None
    servo_connected_dict = None

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
        self.msg_if = MsgIF()
        self.msg_if.pub_info("Starting Node Initialization Processes")

        ##############################
        # Initialize Class Variables

        self.servo_status_subs_dict = dict()
        self.servo_connected_dict = dict()

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
        # BEFORE building the settings dict. updateConnectsHandler does not run until
        # a second after the PTX interface is up, so without this the settings the
        # PTX interface is constructed with -- and the Selection options a restored
        # setting is validated against -- are both still 'None'. SettingsIF.init()
        # then replaces the operator's saved selection with 'None' and pushes that
        # back through setSettingFunction, deselecting the servo on every boot.
        # The restored topic is added to the options list because it has not been
        # discovered yet either; refreshSettingsDict replaces both lists with the
        # discovered ones the first time it runs.
        self.pan_selected = self.pan_connect_if.get_selected_topic()
        self.tilt_selected = self.tilt_connect_if.get_selected_topic()
        self.pan_options = ['None']
        if self.pan_selected != 'None':
            self.pan_options.append(self.pan_selected)
        self.tilt_options = ['None']
        if self.tilt_selected != 'None':
            self.tilt_options.append(self.tilt_selected)

        # Initialize settings
        self.settings_dict = self.initSettingsDict()


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
                                    getSettingsFunction=self.getSettingsFunction,
                                    setSettingFunction=self.setSettingFunction,
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
        # setSettingFunction is the only place a new selection is pushed in.
        available_topics = []
        for connect_if in [self.pan_connect_if, self.tilt_connect_if]:
            if connect_if is not None:
                for topic in connect_if.get_available_topics():
                    if topic not in available_topics:
                        available_topics.append(topic)
        self.updateServoStatusSubs(available_topics)

        # SettingsIF keeps its own copy of the settings dict and re-reads this node's
        # only in init(), so a changed servo list, selection or connected flag has to
        # be pushed. refreshSettingsDict re-reads all three from the connect interfaces
        # every cycle; push on change only, since init() also re-reads and re-applies the
        # saved params. That re-apply is safe for the selections because SettingsIF
        # saves after every accepted change, and the connected flags are never saved.
        # The try keeps a failed refresh or push from skipping the re-arm below, which
        # would freeze the selectors for the life of the node.
        if self.settings_dict is not None:
            try:
                settings_dict = self.refreshSettingsDict()
                settings_state = [list(self.pan_options), self.pan_selected, self.pan_connected,
                                  list(self.tilt_options), self.tilt_selected, self.tilt_connected]
                if settings_state != self.pushed_settings_state:
                    self.settings_dict = settings_dict
                    settings_if = None
                    if self.ptx_if is not None:
                        settings_if = self.ptx_if.settings_if
                    if settings_if is not None:
                        settings_if.init(do_updates = False)
                        self.pushed_settings_state = settings_state
            except Exception as e:
                self.msg_if.pub_warn("Failed to update servo selector settings: " + str(e), throttle_s = 5)

        if self.pan_connect_if is not None and self.pan_connected == True:
            # Keep the last known max if the device reports nothing usable, and do not
            # wipe it back to the placeholder on a disconnect -- see getPanSpeedMax.
            pan_speed_max_dps = self.pan_connect_if.get_max_speed_dps()
            if pan_speed_max_dps is not None and pan_speed_max_dps > 0:
                self.pan_speed_max_dps = pan_speed_max_dps

        if self.tilt_connect_if is not None and self.tilt_connected == True:
            tilt_speed_max_dps = self.tilt_connect_if.get_max_speed_dps()
            if tilt_speed_max_dps is not None and tilt_speed_max_dps > 0:
                self.tilt_speed_max_dps = tilt_speed_max_dps

        nepi_sdk.start_timer_process(1, self.updateConnectsHandler, oneshot = True)


    def updateServoStatusSubs(self, available_topics):
        for svx_namespace in available_topics:
            # A failed create_subscriber returns None; retried on the next cycle.
            if self.servo_status_subs_dict.get(svx_namespace) is None:
                # rospy hands callback_args to the callback as the one tuple, unpacked
                # by servoStatusCb.
                self.servo_status_subs_dict[svx_namespace] = nepi_sdk.create_subscriber(
                    svx_namespace + '/status', DeviceSVXStatus, self.servoStatusCb,
                    queue_size = 1, callback_args = (svx_namespace,))
        for svx_namespace in list(self.servo_status_subs_dict.keys()):
            if svx_namespace not in available_topics:
                sub = self.servo_status_subs_dict.pop(svx_namespace)
                if sub is not None:
                    sub.unregister()
                self.servo_connected_dict.pop(svx_namespace, None)

    def servoStatusCb(self, msg, args):
        svx_namespace = args[0]
        # A message already in flight when its servo vanished must not re-add it.
        if svx_namespace in self.servo_status_subs_dict:
            self.servo_connected_dict[svx_namespace] = msg.servo_connected

    def filterConnectedServos(self, options, selected):
        # Only servos marked Servo Connected on the servo device page are offered. An
        # axis' current selection stays listed even when unmarked, so a saved pairing
        # is never pulled out from under the operator -- the dropdown would otherwise
        # show a value it does not contain. It leaves the list once deselected.
        return [topic for topic in options
                if self.servo_connected_dict.get(topic, False) == True or topic == selected]


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

    def initSettingsDict(self):
        init_settings_dict = dict()
        init_settings_dict['pan_servo'] = {
            'type': 'Selection',
            'options': self.getSelectorOptions(self.pan_options, self.pan_selected),
            'default': self.getServoName(self.pan_selected)
        }
        # Read-only indicators of the selected servo's connection. 'param': False
        # keeps them out of the saved settings, so SettingsIF.init() never
        # re-applies a stale saved value over the live state.
        init_settings_dict['pan_connected'] = {
            'type': 'Toggle',
            'default': (self.pan_connected == True),
            'display_disabled': True,
            'param': False
        }
        init_settings_dict['tilt_servo'] = {
            'type': 'Selection',
            'options': self.getSelectorOptions(self.tilt_options, self.tilt_selected),
            'default': self.getServoName(self.tilt_selected)
        }
        init_settings_dict['tilt_connected'] = {
            'type': 'Toggle',
            'default': (self.tilt_connected == True),
            'display_disabled': True,
            'param': False
        }

        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        settings_dict_values = nepi_controls.get_values_dict(settings_dict)
        self.msg_if.pub_info("Initialized Settings: " + str(settings_dict_values))
        return settings_dict

    def refreshSettingsDict(self):
        # The servo lists change at runtime as SVX devices come and go, and each axis'
        # list excludes the other axis' selection, so this re-reads the selections,
        # lists and connected flags from the connect interfaces and rebuilds the options
        # as well as the values. Reading them only on the 1 s connects cycle left the
        # dict setSettingFunction returns offering the servo just taken by one axis to
        # the other. Both selections are read before either list is built, so each list
        # excludes the other axis' current selection rather than last cycle's.
        if self.pan_connect_if is not None:
            self.pan_selected = self.pan_connect_if.get_selected_topic()
        if self.tilt_connect_if is not None:
            self.tilt_selected = self.tilt_connect_if.get_selected_topic()

        if self.pan_connect_if is not None:
            # get_available_topics returns the interface's live internal list, so copy
            # before filtering. Editing it in place corrupts the selector state.
            pan_options = list(self.pan_connect_if.get_available_topics())
            if self.tilt_selected != None and self.tilt_selected in pan_options:
                pan_options.remove(self.tilt_selected)
            self.pan_options = ['None'] + self.filterConnectedServos(pan_options, self.pan_selected)
            self.pan_connected = self.pan_connect_if.check_connection()

        if self.tilt_connect_if is not None:
            tilt_options = list(self.tilt_connect_if.get_available_topics())
            if self.pan_selected != None and self.pan_selected in tilt_options:
                tilt_options.remove(self.pan_selected)
            self.tilt_options = ['None'] + self.filterConnectedServos(tilt_options, self.tilt_selected)
            self.tilt_connected = self.tilt_connect_if.check_connection()

        # Options go first: set_value validates a Selection against the options
        # already in the dict.
        settings_dict = copy.deepcopy(self.settings_dict)
        settings_dict = nepi_controls.set_options(settings_dict, 'pan_servo',
                            self.getSelectorOptions(self.pan_options, self.pan_selected))
        settings_dict = nepi_controls.set_value(settings_dict, 'pan_servo', self.getServoName(self.pan_selected))
        settings_dict = nepi_controls.set_value(settings_dict, 'pan_connected', (self.pan_connected == True))
        settings_dict = nepi_controls.set_options(settings_dict, 'tilt_servo',
                            self.getSelectorOptions(self.tilt_options, self.tilt_selected))
        settings_dict = nepi_controls.set_value(settings_dict, 'tilt_servo', self.getServoName(self.tilt_selected))
        settings_dict = nepi_controls.set_value(settings_dict, 'tilt_connected', (self.tilt_connected == True))
        return settings_dict

    def getSelectorOptions(self, options, selected):
        # An axis' selection always appears in its own option list. Left out --
        # which happens while the selected servo is undiscovered, or excluded as
        # the other axis' selection -- set_value falls back to another option, so
        # the setting reports a servo the interface does not hold, and the next
        # save persists that substitute, which SettingsIF.init() then pushes back
        # through setSettingFunction on the following boot.
        options = list(options)
        selected = str(selected)
        if selected not in options:
            options.append(selected)
        return [self.getServoName(topic) for topic in options]

    def getServoName(self, topic):
        # The servo selectors show each servo by its node name (maestro_00512566_ch4),
        # the label the Servo Device page uses, rather than its full SVX topic, which
        # is too wide for the settings panel. Selections are still held and pushed as
        # topics; getServoTopic maps a name coming back from the settings panel.
        name = str(topic)
        if name.endswith('/svx'):
            name = name[:-len('/svx')]
        return name.split('/')[-1]

    def getServoTopic(self, name):
        # A full topic is accepted as-is: selections saved before the selectors
        # switched to names were saved as topics, and SettingsIF.init() re-applies them.
        if name == 'None' or name.startswith('/'):
            return name
        topics = [self.pan_selected, self.tilt_selected]
        for connect_if in [self.pan_connect_if, self.tilt_connect_if]:
            if connect_if is not None:
                topics += list(connect_if.get_available_topics())
        for topic in topics:
            if topic not in [None, 'None'] and self.getServoName(topic) == name:
                return topic
        return None

    def getSettingsFunction(self):
        return self.settings_dict

    def setSettingFunction(self, setting_name, setting_value):
        setting_str = setting_name + ":" + str(setting_value)
        success = False
        msg = ""
        if setting_name not in self.settings_dict.keys():
            msg = (self.node_name + " Setting name " + setting_str + " is not supported")
            return False, msg, self.settings_dict
        if setting_name == 'pan_connected' or setting_name == 'tilt_connected':
            msg = (self.node_name + " Setting " + setting_name + " is read-only. It reports " +
                   "whether the selected servo is connected and is updated by the node")
            return False, msg, self.settings_dict

        # This is the only place a selection is pushed into a connect interface.
        # The interface ignores an undiscovered topic, so adopt what it returns.
        requested = self.getServoTopic(str(setting_value))
        if requested is None:
            msg = (self.node_name + " Servo " + str(setting_value) + " is not available for " + setting_name +
                   ". Check that the servo's driver is running")
            self.settings_dict = self.refreshSettingsDict()
            return False, msg, self.settings_dict
        if setting_name == 'pan_servo':
            self.pan_selected = self.selectServo(self.pan_connect_if, requested)
            selected = self.pan_selected
        else:
            self.tilt_selected = self.selectServo(self.tilt_connect_if, requested)
            selected = self.tilt_selected
        success = (selected == requested)
        if success:
            msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
        else:
            msg = (self.node_name + " Servo " + requested + " is not available for " + setting_name +
                   ", so it stays on " + str(selected) + ". Check that the servo's driver is running")

        self.settings_dict = self.refreshSettingsDict()
        return success, msg, self.settings_dict

    def selectServo(self, connect_if, topic):
        # Returns the selection in effect afterwards. An unchanged selection is not
        # re-sent: set_selected_topic persists and saves config on every accepted call,
        # and SettingsIF.init() re-applies every saved setting on each startup.
        if connect_if is None:
            return topic
        if topic == connect_if.get_selected_topic():
            return topic
        return connect_if.set_selected_topic(topic)



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
