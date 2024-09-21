#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_nex

from nepi_edge_sdk_base.device_if_ptx import ROSPTXActuatorIF

PKG_NAME = 'PTX_ONVIF_GENERIC' # Use in display menus
DESCRIPTION = 'Driver package for generic ONVIF pan tilt devices'
FILE_TYPE = 'NODE'
CLASS_NAME = 'OnvifPanTiltNode' # Should Match Class Name
GROUP ='PTX'
GROUP_ID = 'ONVIF' 


DRIVER_PKG_NAME = 'PTX_ONVIF_GENERIC' # 'Required Driver PKG_NAME or 'None'
DRIVER_INTERFACES = ['IP'] # 'USB','IP','SERIALUSB','SERIAL','CANBUS'
DRIVER_OPTIONS_1_NAME = 'None'
DRIVER_OPTIONS_1 = [] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_1 = 'None'
DRIVER_OPTIONS_2_NAME = 'None'
DRIVER_OPTIONS_2 = [] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_2 = 'None'

DISCOVERY_PKG_NAME = 'None' # 'Required Driver PKG_NAME or 'None'
DISCOVERY_METHOD = 'OTHER'  # 'AUTO', 'MANUAL', or 'OTHER' if managed by seperate application
DISCOVERY_IDS = []  # List of string identifiers for discovery process
DISCOVERY_IGNORE_IDS = [] # List of string identifiers for discovery process

class OnvifPanTiltNode:
    DEFAULT_DRIVER_PATHS = ["/opt/nepi/ros/lib/nepi_drivers/"]

    FACTORY_SETTINGS_OVERRIDES = dict( )


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

    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
    nex_dict = dict()                             
    def __init__(self):
        # Launch the ROS node
        rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
        self.node_name = rospy.get_name().split('/')[-1]
        # Get nex_dict from param servers
        rospy.loginfo("Starting " + self.node_name)
        try:
            self.nex_dict = rospy.get_param('~nex_dict') # Crash if not provide
        except Exception as e:
            rospy.signal_shutdown("Failed to read nex_dict from param server for node " + self.node_name + " with exception: " + str(e))
        self.driver_name = self.nex_dict['driver_name']
        self.driver_file = self.nex_dict['driver_file_name']
        self.driver_path = self.nex_dict['driver_file_path']
        self.driver_module = self.nex_dict['driver_module_name']
        self.driver_class_name = self.nex_dict['driver_class_name']
        self.driver_interfaces = self.nex_dict['driver_interfaces']
        self.driver_options_1 = self.nex_dict['driver_options_1']
        self.driver_option_1 = self.nex_dict['driver_set_option_1']
        # import driver class fromn driver module


        # Require the camera connection parameters to have been set
        if not rospy.has_param('~credentials/username'):
            rospy.logerr(self.node_name + ": Missing credentials/username parameter... cannot start")
            return
        if not rospy.has_param('~credentials/password'):
            rospy.logerr(self.node_name + ": Missing credentials/password parameter... cannot start")
            return
        if not rospy.has_param('~network/host'):
            rospy.logerr(self.node_name + ": Missing network/host parameter... cannot start")
            return
        if not rospy.has_param('~driver_id'):
            rospy.logerr(self.node_name + ": Missing driver_id parameter... cannot start")
            return
                
        username = str(rospy.get_param('~credentials/username'))
        password = str(rospy.get_param('~credentials/password'))
        host = str(rospy.get_param('~network/host'))
        
        # Allow a default for the port, since it is part of onvif spec.
        onvif_port = rospy.get_param('~network/port', 80)
        rospy.set_param('~/network/port', onvif_port)


        rospy.loginfo(self.node_name + ": Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_nex.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        
        if success:
            driver_constructed = False
            attempts = 0
            while not rospy.is_shutdown() and driver_constructed == False and attempts < 5 and not rospy.is_shutdown():
                try:
                    self.driver = self.driver_class(username, password, host, onvif_port)
                    driver_constructed = True
                    rospy.loginfo("ONVIF_NODE: Driver constructed")
                except Exception as e:
                    rospy.loginfo("ONVIF_NODE: Failed to construct driver: " + self.driver_id + "with exception: " + str(e))
                    rospy.sleep(1)
                attempts += 1 
        if driver_constructed == False:
            rospy.signal_shutdown("Shutting down Onvif node " + self.node_name + ", unable to connect to driver")
        else:
            ################################################
            rospy.loginfo(self.node_name + ": ... Connected!")
            self.dev_info = self.driver.getDeviceInfo()
            self.logDeviceInfo()

                    
            ptx_callback_names = {
                # PTX Standard
                "StopMoving": self.stopMoving,
                "MoveYaw": self.moveYaw,
                "MovePitch": self.movePitch,
                "SetSpeed": self.setSpeed,
                "GetSpeed": self.getSpeed,
                "GetCurrentPosition": self.getCurrentPosition,
                "GotoPosition": self.gotoPosition,
                "GoHome": self.goHome,
                "SetHomePosition": self.setHomePosition,
                "SetHomePositionHere": self.setHomePositionHere,
                "GotoWaypoint": self.gotoWaypoint,
                "SetWaypoint": self.setWaypoint,
                "SetWaypointHere": self.setWaypointHere
            }

            # Must pass a capabilities structure to ptx_interface constructor
            ptx_capabilities_dict = {}

            # Now check with the driver if any of these PTX capabilities are explicitly not present
            if not self.driver.hasAdjustableSpeed():
                ptx_callback_names["GetSpeed"] = None # Clear the method
                ptx_callback_names["SetSpeed"] = None # Clear the method
                ptx_capabilities_dict['has_speed_control'] = False
            else:
                ptx_capabilities_dict['has_speed_control'] = True
                
            if not self.driver.reportsPosition():
                ptx_callback_names["GetCurrentPosition"] = None # Clear the method
                
            if not self.driver.hasAbsolutePositioning():
                rospy.logerr("Debug: Clearing GotoPosition")
                ptx_callback_names["GotoPosition"] = None # Clear the method
            
            self.has_absolute_positioning_and_feedback = self.driver.hasAbsolutePositioning() and self.driver.reportsPosition()
            ptx_capabilities_dict['has_absolute_positioning'] = self.has_absolute_positioning_and_feedback
                    
            if not self.driver.canHome() and not self.has_absolute_positioning_and_feedback:
                ptx_callback_names["GoHome"] = None
                ptx_capabilities_dict['has_homing'] = False
            else:
                ptx_capabilities_dict['has_homing'] = True
            
                
            if not self.driver.homePositionAdjustable() and not self.has_absolute_positioning_and_feedback:
                ptx_callback_names["SetHomePositionHere"] = None
            
            if not self.driver.hasWaypoints() and not self.has_absolute_positioning_and_feedback:
                ptx_callback_names["GotoWaypoint"] = None
                ptx_callback_names["SetWaypointHere"] = None
                ptx_capabilities_dict['has_waypoints'] = False
            else:
                ptx_capabilities_dict['has_waypoints'] = True
            
            # Following are implemented entirely in software because ONVIF has no support for setting HOME or PRESET by position
            if not self.has_absolute_positioning_and_feedback:
                ptx_callback_names["SetHomePosition"] = None    
                ptx_callback_names["SetWaypoint"] = None

            # Now that we've updated the callbacks table, can apply the remappings... this is particularly useful since
            # many ONVIF devices report capabilities that they don't actually have, so need a user-override mechanism. In
            # that case, assign these to null in the config file
            # TODO: Not sure we actually need remappings for PTX: Makes sense for IDX because there are lots of controllable params.
            ptx_remappings = rospy.get_param('~ptx_remappings', {})
            rospy.loginfo(self.node_name + ': Establishing PTX remappings')
            for from_name in ptx_remappings:
                to_name = ptx_remappings[from_name]
                if from_name not in ptx_callback_names or (to_name not in ptx_callback_names and to_name != None and to_name != 'None'):
                    rospy.logwarn('\tInvalid PTX remapping: ' + from_name + '-->' + to_name)
                elif to_name is None or to_name == 'None':
                    ptx_callback_names[from_name] = None
                    rospy.loginfo('\tRemapping %s to non-existence to remove capability', from_name)
                elif ptx_callback_names[to_name] is None:
                    rospy.logwarn('\tRemapping ' + from_name + ' to an unavailable adjustment (' + to_name + ')')
                else:
                    ptx_callback_names[from_name] = ptx_callback_names[to_name]
                    rospy.loginfo('\t' + from_name + '-->' + to_name)



            # Launch the PTX interface --  this takes care of initializing all the ptx settings from config. file, subscribing and advertising topics and services, etc.
            # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
            rospy.loginfo(self.node_name + ": Launching NEPI PTX (ROS) interface...")
            self.device_info_dict["node_name"] = self.node_name
            if self.node_name.find("_") != -1:
                split_name = self.node_name.rsplit('_', 1)
                self.device_info_dict["device_name"] = split_name[0]
                self.device_info_dict["identifier"] = split_name[1]
            else:
                self.device_info_dict["device_name"] = self.node_name
                self.device_info_dict["identifier"] = ""
            self.device_info_dict["serial_number"] = self.driver.getDeviceSerialNumber()
            self.device_info_dict["hw_version"] = self.driver.getDeviceHardwareId()
            self.device_info_dict["sw_version"] = self.driver.getDeviceFirmwareVersion()

            #Factory Control Values 
            self.FACTORY_CONTROLS = {
                'frame_id' : self.node_name + '_frame',
                'yaw_joint_name' : self.node_name + '_yaw_joint',
                'pitch_joint_name' : self.node_name + '_pitch_joint',
                'reverse_yaw_control' : False,
                'reverse_pitch_control' : False,
                'speed_ratio' : 1.0,
                'status_update_rate_hz' : 1.0
            }
            
            # Driver can specify position limits via getPositionLimitsInDegrees. Otherwise, we hard-code them 
            # to arbitrary values here, but can be overridden in device config file (see ptx_if.py)
            default_settings = dict()
            if hasattr(self.driver, 'getPositionLimitsInDegrees'):
                driver_specified_limits = self.driver.getPositionLimitsInDegrees()
                default_settings['min_yaw_hardstop_deg'] = driver_specified_limits['min_yaw_hardstop_deg']
                default_settings['max_pitch_hardstop_deg'] = driver_specified_limits['max_pitch_hardstop_deg']
                default_settings['min_pitch_hardstop_deg'] = driver_specified_limits['min_pitch_hardstop_deg']
                default_settings['max_yaw_softstop_deg'] = driver_specified_limits['max_yaw_softstop_deg']
                default_settings['min_yaw_softstop_deg'] = driver_specified_limits['min_yaw_softstop_deg']
                default_settings['max_pitch_softstop_deg'] = driver_specified_limits['max_pitch_softstop_deg']
                default_settings['min_pitch_softstop_deg'] = driver_specified_limits['min_pitch_softstop_deg']
            else:
                default_settings['max_yaw_hardstop_deg'] = 60.0
                default_settings['min_yaw_hardstop_deg'] = -60.0
                default_settings['max_pitch_hardstop_deg'] = 60.0
                default_settings['min_pitch_hardstop_deg'] = -60.0
                default_settings['max_yaw_softstop_deg'] = 59.0
                default_settings['min_yaw_softstop_deg'] = -59.0
                default_settings['max_pitch_softstop_deg'] = 59.0
                default_settings['min_pitch_softstop_deg'] = -59.0

            # Initialize settings
            self.cap_settings = self.getCapSettings()
            rospy.loginfo(self.node_name + ": " +"CAPS SETTINGS")
            #for setting in self.cap_settings:
                #rospy.loginfo(self.node_name + ": " +setting)
            self.factory_settings = self.getFactorySettings()
            rospy.loginfo(self.node_name + ": " +"FACTORY SETTINGS")
            #for setting in self.factory_settings:
                #rospy.loginfo(self.node_name + ": " +setting)

            self.ptx_if = ROSPTXActuatorIF(device_info = self.device_info_dict, 
                                        capSettings = self.cap_settings,
                                        factorySettings = self.factory_settings,
                                        settingUpdateFunction=self.settingUpdateFunction,
                                        getSettingsFunction=self.getSettings,
                                        factoryControls = self.FACTORY_CONTROLS,
                                        defaultSettings = default_settings,
                                        capabilities_dict = ptx_capabilities_dict,
                                        stopMovingCb = ptx_callback_names["StopMoving"],
                                        moveYawCb = ptx_callback_names["MoveYaw"],
                                        movePitchCb = ptx_callback_names["MovePitch"],
                                        setSpeedCb = ptx_callback_names["SetSpeed"],
                                        getSpeedCb = ptx_callback_names["GetSpeed"],
                                        getCurrentPositionCb = ptx_callback_names["GetCurrentPosition"],
                                        gotoPositionCb = ptx_callback_names["GotoPosition"],
                                        goHomeCb = ptx_callback_names["GoHome"],
                                        setHomePositionCb = ptx_callback_names["SetHomePosition"],
                                        setHomePositionHereCb = ptx_callback_names["SetHomePositionHere"],
                                        gotoWaypointCb = ptx_callback_names["GotoWaypoint"],
                                        setWaypointCb = ptx_callback_names["SetWaypoint"],
                                        setWaypointHereCb = ptx_callback_names["SetWaypointHere"])
            rospy.loginfo(self.node_name + " ... PTX interface running")

            self.speed_ratio = 1.0
            self.home_yaw_deg = 0.0
            self.home_pitch_deg = 0.0
            self.waypoints = {} # Dictionary of dictionaries with numerical key and {waypoint_pitch, waypoint_yaw} dict value

            rospy.spin()



    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["HardwareId"] + "\n"
        rospy.loginfo(dev_info_string)

    #**********************
    # Device setting functions

    def getSettingsOptionsDict(self):
        settings_options_dict = dict()
        return settings_options_dict


    def getCapSettings(self):
        settings_cap_dict = self.getSettingsOptionsDict()
        return nepi_ros.NONE_SETTINGS

    def getFactorySettings(self):
        settings_options_dict = self.getSettingsOptionsDict()
        settings = nepi_ros.NONE_SETTINGS
        #Apply factory setting overides
        for setting in settings:
            if setting[1] in self.FACTORY_SETTINGS_OVERRIDES:
                setting[2] = self.FACTORY_SETTINGS_OVERRIDES[setting[1]]
                settings = nepi_ros.update_setting_in_settings(setting,settings)
        return settings


    def getSettingsDict(self):
        settings_dict = dict()
        return settings_dict
            

    def getSettings(self):
        settings_dict = self.getSettingsDict()
        settings = nepi_ros.NONE_SETTINGS
        return settings

    def setSetting(self,setting_name,setting_data):
        return True


    def settingUpdateFunction(self,setting):
        success = False
        setting_str = str(setting)
        if len(setting) == 3:
            setting_type = setting[0]
            setting_name = setting[1]
            [s_name, s_type, data] = nepi_ros.get_data_from_setting(setting)
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
        else:
            msg = (self.node_name  + " Setting " + setting_str + " not correct length")
        return success, msg


    #######################
    ### PTX IF Functions

    def stopMoving(self):
        self.driver.stopMotion()

    def moveYaw(self, direction, duration):
        driver_direction = self.driver.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.driver.PT_DIRECTION_NEGATIVE
        self.driver.jog(pan_direction = driver_direction, tilt_direction = self.driver.PT_DIRECTION_NONE, speed_ratio = self.speed_ratio, time_s = duration)

    def movePitch(self, direction, duration):
        driver_direction = self.driver.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.driver.PT_DIRECTION_NEGATIVE
        self.driver.jog(pan_direction = self.driver.PT_DIRECTION_NONE, tilt_direction = driver_direction, speed_ratio = self.speed_ratio, time_s = duration)

    def setSpeed(self, speed_ratio):
        # TODO: Limits checking and driver unit conversion?
        self.speed_ratio = speed_ratio

    def getSpeed(self):
        # TODO: Driver unit conversion?
        return self.speed_ratio
            
    def getCurrentPosition(self):
        pan_ratio_onvif, tilt_ratio_onvif = self.driver.getCurrentPosition()
        # Recenter the -1.0,1.0 ratio from Onvif to the 0.0,1.0 used throughout the rest of NEPI
        pan_ratio = (0.5 * pan_ratio_onvif) + 0.5
        tilt_ratio = (0.5 * tilt_ratio_onvif) + 0.5
        #rospy.logwarn(f"Debug: Current position ratio = {pan_ratio},{tilt_ratio}")
        pan_deg = self.ptx_if.yawRatioToDeg(pan_ratio)
        tilt_deg = self.ptx_if.pitchRatioToDeg(tilt_ratio)
        return pan_deg, tilt_deg

    def gotoPosition(self, yaw_deg, pitch_deg):
        yaw_ratio = self.ptx_if.yawDegToRatio(yaw_deg)
        pitch_ratio = self.ptx_if.pitchDegToRatio(pitch_deg)
        # Recenter the ratios to the ONVIF -1.0,1.0 range
        yaw_ratio_onvif = 2 * (yaw_ratio - 0.5) 
        pitch_ratio_onvif = 2 * (pitch_ratio - 0.5)
        self.driver.moveToPosition(yaw_ratio_onvif, pitch_ratio_onvif, self.speed_ratio)
        
    def goHome(self):
        if self.driver.canHome() is True:
            self.driver.goHome(self.speed_ratio)
        elif self.driver.hasAbsolutePositioning() is True:
            home_yaw_ratio = self.ptx_if.yawDegToRatio(self.home_yaw_deg)
            home_pitch_ratio = self.ptx_if.pitchDegToRatio(self.home_pitch_deg)
            home_yaw_ratio_onvif = 2 * (home_yaw_ratio - 0.5)
            home_pitch_ratio_onvif = 2 * (home_pitch_ratio - 0.5)
            self.driver.moveToPosition(home_yaw_ratio_onvif, home_pitch_ratio_onvif, self.speed_ratio)
        else:
            rospy.logwarn("Homing not supported by this PTX device... ignoring")

    def setHomePosition(self, yaw_deg, pitch_deg):
        # Have to implement these fully in s/w since ONVIF doesn't support absolute home position setting
        self.home_yaw_deg = yaw_deg
        self.home_pitch_deg = pitch_deg

    def setHomePositionHere(self):
        if self.driver.reportsPosition() is True:
            curr_yaw_ratio_onvif, curr_pitch_ratio_onvif = self.driver.getCurrentPosition()
            curr_yaw_ratio = (curr_yaw_ratio_onvif + 1.0) * 0.5
            curr_pitch_ratio = (curr_pitch_ratio_onvif + 1.0) * 0.5
            self.home_yaw_deg = self.ptx_if.yawRatioToDeg(curr_yaw_ratio)
            self.home_pitch_deg = self.ptx_if.pitchRatioToDeg(curr_pitch_ratio) 

        if self.driver.homePositionAdjustable is True:
            self.driver.setHomeHere()

    def gotoWaypoint(self, waypoint_index):
        if self.driver.hasWaypoints() is True:
           self.driver.gotoPreset(waypoint_index, self.speed_ratio)
        elif self.driver.hasAbsolutePositioning() is True:
            if waypoint_index not in self.waypoints:
                rospy.logwarn("Requested waypoint index %u is invalid/unset... ignoring", waypoint_index)
                return
            
            waypoint_yaw_deg = self.waypoints[waypoint_index]['yaw_deg']
            waypoint_yaw_ratio = self.ptx_if.yawDegToRatio(waypoint_yaw_deg)
            waypoint_pitch_deg = self.waypoints[waypoint_index]['pitch_deg']
            waypoint_pitch_ratio = self.ptx_if.pitchDegToRatio(waypoint_pitch_deg)

            waypoint_yaw_ratio_onvif = 2 * (waypoint_yaw_ratio - 0.5)
            waypoint_pitch_ratio_onvif = 2 * (waypoint_pitch_ratio - 0.5)
            self.driver.moveToPosition(waypoint_yaw_ratio_onvif, waypoint_pitch_ratio_onvif, self.speed_ratio)
    
    def setWaypoint(self, waypoint_index, yaw_deg, pitch_deg):
        # Have to implement these fully in s/w since ONVIF doesn't support absolute home position setting
        self.waypoints[waypoint_index] = {'yaw_deg': yaw_deg, 'pitch_deg': pitch_deg}
        
    def setWaypointHere(self, waypoint_index):
        if self.driver.reportsPosition():
            yaw_ratio_onvif, pitch_ratio_onvif = self.driver.getCurrentPosition()
            yaw_ratio = (yaw_ratio_onvif + 1.0) * 0.5
            pitch_ratio = (pitch_ratio_onvif + 1.0) * 0.5
            current_yaw_deg = self.ptx_if.yawRatioToDeg(yaw_ratio)
            current_pitch_deg = self.ptx_if.pitchRatioToDeg(pitch_ratio)
            self.waypoints[waypoint_index] = {'yaw_deg': current_yaw_deg, 'pitch_deg': current_pitch_deg}

        if self.driver.hasWaypoints():
            self.driver.setPresetHere(waypoint_index)

        rospy.loginfo("Waypoint set to current position")

    def setCurrentSettingsAsDefault(self):
        # Don't need to worry about any of our params in this class, just child interfaces' params
        self.ptx_if.setCurrentSettingsToParamServer()

    def updateFromParamServer(self):
        self.ptx_if.updateFromParamServer()

if __name__ == '__main__':
	node = OnvifPanTiltNode()
