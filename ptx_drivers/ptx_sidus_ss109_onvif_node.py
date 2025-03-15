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

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_settings

from nepi_sdk.device_if_ptx import ROSPTXActuatorIF

PKG_NAME = 'PTX_ONVIF_GENERIC' # Use in display menus
FILE_TYPE = 'NODE'



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

    last_pan_ratio_onvif = 0
    last_tilt_ratio_onvif = 0

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
        try:
            self.drv_dict = nepi_ros.get_param(self,'~drv_dict') # Crash if not provide
        except Exception as e:
            nepi_ros.signal_shutdown("Failed to read drv_dict from param server for node " + self.node_name + " with exception: " + str(e))
        self.driver_path = self.drv_dict['path']
        self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
        self.driver_module = self.driver_file.split('.')[0]
        self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']


        # Require the camera connection parameters to have been set
        if not nepi_ros.has_param(self,'~credentials/username'):
            nepi_msg.publishMsgErr(self,"Missing credentials/username parameter... cannot start")
            return
        if not nepi_ros.has_param(self,'~credentials/password'):
            nepi_msg.publishMsgErr(self,"Missing credentials/password parameter... cannot start")
            return
        if not nepi_ros.has_param(self,'~network/host'):
            nepi_msg.publishMsgErr(self,"Missing network/host parameter... cannot start")
            return
                
        username = str(nepi_ros.get_param(self,'~credentials/username'))
        password = str(nepi_ros.get_param(self,'~credentials/password'))
        host = str(nepi_ros.get_param(self,'~network/host'))
        
        # Allow a default for the port, since it is part of onvif spec.
        onvif_port = nepi_ros.get_param(self,'~network/port', 80)
        nepi_ros.set_param(self,'~/network/port', onvif_port)


        nepi_msg.publishMsgInfo(self,"Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drv.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        
        driver_constructed = False
        if success:
            attempts = 0
            while not nepi_ros.is_shutdown() and driver_constructed == False and attempts < 5:
                try:
                    self.driver = self.driver_class(username, password, host, onvif_port)
                    driver_constructed = True
                    nepi_msg.publishMsgInfo(self,"Driver constructed")
                except Exception as e:
                    nepi_msg.publishMsgInfo(self,"Failed to construct driver: " + self.driver_module + "with exception: " + str(e))
                    nepi_ros.sleep(1)
                attempts += 1 
        if driver_constructed == False:
            nepi_ros.signal_shutdown("Shutting down Onvif node " + self.node_name + ", unable to connect to driver")
        else:
            ################################################
            nepi_msg.publishMsgInfo(self,"... Connected!")
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
            ptx_remappings = nepi_ros.get_param(self,'~ptx_remappings', {})
            nepi_msg.publishMsgInfo(self,'Establishing PTX remappings')
            for from_name in ptx_remappings:
                to_name = ptx_remappings[from_name]
                if from_name not in ptx_callback_names or (to_name not in ptx_callback_names and to_name != None and to_name != 'None'):
                    pass
                elif to_name is None or to_name == 'None':
                    ptx_callback_names[from_name] = None
                elif ptx_callback_names[to_name] is None:
                    pass
                else:
                    ptx_callback_names[from_name] = ptx_callback_names[to_name]

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
                'speed_ratio' : 0.5,
                'status_update_rate_hz' : 10
            }
            
            # Driver can specify position limits via getPositionLimitsInDegrees. Otherwise, we hard-code them 
            # to arbitrary values here, but can be overridden in device config file (see ptx_if.py)
            self.default_settings = dict()
            if hasattr(self.driver, 'getPositionLimitsInDegrees'):
                driver_specified_limits = self.driver.getPositionLimitsInDegrees()
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

            self.speed_ratio = 0.5
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
            nepi_msg.publishMsgInfo(self," ... PTX interface running")


            nepi_ros.spin()



    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["HardwareId"] + "\n"
        nepi_msg.publishMsgInfo(self,dev_info_string)


    #**********************
    # Device setting functions


    def getCapSettings(self):
        return nepi_settings.NONE_CAP_SETTINGS

    def getFactorySettings(self):
        return nepi_settings.NONE_SETTINGS

    def getSettings(self):
        return nepi_settings.NONE_SETTINGS

    def setSetting(self,setting_name,setting_data):
        return True


    def settingUpdateFunction(self,setting):
        success = False
        setting_str = str(setting)
        [setting_name, s_type, data] = nepi_ros.get_data_from_setting(setting)
        print("Onvif PT updating setting: " + str(setting_str))
        if data is not None and data != "None":
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
        return success, msg


    #######################
    ### PTX IF Functions

    def stopMoving(self):
        self.driver.stopMotion()

    def moveYaw(self, direction, duration):
        if self.ptx_if is not None:
            driver_direction = self.driver.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.driver.PT_DIRECTION_NEGATIVE
            self.driver.jog(pan_direction = driver_direction, tilt_direction = self.driver.PT_DIRECTION_NONE, speed_ratio = self.speed_ratio, time_s = duration)

    def movePitch(self, direction, duration):
        if self.ptx_if is not None:
            driver_direction = self.driver.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.driver.PT_DIRECTION_NEGATIVE
            self.driver.jog(pan_direction = self.driver.PT_DIRECTION_NONE, tilt_direction = driver_direction, speed_ratio = self.speed_ratio, time_s = duration)

    def setSpeed(self, speed_ratio):
        # TODO: Limits checking and driver unit conversion?
        self.speed_ratio = speed_ratio

    def getSpeed(self):
        # TODO: Driver unit conversion?
        return self.speed_ratio
            
     
    def pitchDegToOvRatio(self, deg):
        ratio = 0.5
        max_ph = nepi_ros.get_param(self,'~ptx/limits/max_pitch_hardstop_deg', self.default_settings['max_pitch_hardstop_deg'])
        min_ph = nepi_ros.get_param(self,'~ptx/limits/min_pitch_hardstop_deg', self.default_settings['min_pitch_hardstop_deg'])
        reverse_pitch = False
        if self.ptx_if is not None:
            reverse_pitch = self.ptx_if.reverse_pitch_control
        if reverse_pitch == False:
          ratio = 1 - (deg - min_ph) / (max_ph - min_ph)
        else:
          ratio = (deg - min_ph) / (max_ph - min_ph)
        ovRatio = 2 * (ratio - 0.5)
        return ovRatio

    def yawDegToOvRatio(self, deg):
        ratio = 0.5
        max_yh = nepi_ros.get_param(self,'~ptx/limits/max_yaw_hardstop_deg', self.default_settings['max_yaw_hardstop_deg'])
        min_yh = nepi_ros.get_param(self,'~ptx/limits/min_yaw_hardstop_deg', self.default_settings['min_yaw_hardstop_deg'])
        reverse_yaw = False
        if self.ptx_if is not None:
            reverse_yaw = self.ptx_if.reverse_yaw_control 
        if reverse_yaw == False:
          ratio = 1 - (deg - min_yh) / (max_yh - min_yh)
        else:
          ratio = (deg - min_yh) / (max_yh - min_yh)
        ovRatio = 2 * (ratio - 0.5)
        return (ovRatio)     

    def yawOvRatioToDeg(self, ovRatio):
        yaw_deg = 0
        max_yh = nepi_ros.get_param(self,'~ptx/limits/max_yaw_hardstop_deg', self.default_settings['max_yaw_hardstop_deg'])
        min_yh = nepi_ros.get_param(self,'~ptx/limits/min_yaw_hardstop_deg', self.default_settings['min_yaw_hardstop_deg'])
        reverse_yaw = False
        if self.ptx_if is not None:
            reverse_yaw = self.ptx_if.reverse_yaw_control 
        if reverse_yaw == False:
           yaw_deg =  min_yh + (1-ovRatio) * (max_yh - min_yh)
        else:
           yaw_deg =  max_yh - (1-ovRatio)  * (max_yh - min_yh)
        return  yaw_deg
    
    def pitchOvRatioToDeg(self, ovRatio):
        pitch_deg = 0
        max_ph = nepi_ros.get_param(self,'~ptx/limits/max_pitch_hardstop_deg', self.default_settings['max_pitch_hardstop_deg'])
        min_ph = nepi_ros.get_param(self,'~ptx/limits/min_pitch_hardstop_deg', self.default_settings['min_pitch_hardstop_deg'])
        reverse_pitch = False
        if self.ptx_if is not None:
            reverse_pitch = self.ptx_if.reverse_pitch_control
        if reverse_pitch == False:
           pitch_deg =  min_ph + (1-ovRatio) * (max_ph - min_ph)
        else:
           pitch_deg =  max_ph - (1-ovRatio) * (max_ph - min_ph)
        return  pitch_deg



    def getCurrentPosition(self):
        pan_ratio_onvif, tilt_ratio_onvif = self.driver.getCurrentPosition()
        if pan_ratio_onvif != -999 and tilt_ratio_onvif != -999:
            self.last_pan_ratio_onvif = pan_ratio_onvif
            self.last_tilt_ratio_onvif = tilt_ratio_onvif
        #print("Got pos_ratio_onvif from driver: " + str([pan_ratio_onvif, tilt_ratio_onvif]))
        # Recenter the -1.0,1.0 ratio from Onvif to the 0.0,1.0 used throughout the rest of NEPI
        pan_ratio_onvif = (0.5 * self.last_pan_ratio_onvif) + 0.5
        tilt_ratio_onvif = (0.5 * self.last_tilt_ratio_onvif) + 0.5
        #print("Got pos_ratio adj: " + str([pan_ratio_onvif, tilt_ratio_onvif]))
        if self.ptx_if is None:
            pan_deg = 0
            tilt_deg = 0
        else:
            pan_deg = self.yawOvRatioToDeg(pan_ratio_onvif)
            tilt_deg = self.pitchOvRatioToDeg(tilt_ratio_onvif)
        #print("Got pos degs : " + str([pan_deg, tilt_deg]))
        return pan_deg, tilt_deg
        

    def gotoPosition(self, yaw_deg, pitch_deg):
        yaw_ratio_onvif = self.yawDegToOvRatio(yaw_deg)
        pitch_ratio_onvif = self.pitchDegToOvRatio(pitch_deg)
        # Recenter the ratios to the ONVIF -1.0,1.0 range
        self.driver.moveToPosition(yaw_ratio_onvif, pitch_ratio_onvif, self.speed_ratio)
        
    def goHome(self):
        if self.driver.canHome() is True:
            self.driver.goHome(self.speed_ratio)
        elif self.driver.hasAbsolutePositioning() is True and self.ptx_if is not None:
            home_yaw_ratio_onvif = self.yawDegToOvRatio(self.home_yaw_deg)
            home_pitch_ratio_onvif = self.pitchDegToOvRatio(self.home_pitch_deg)
            self.driver.moveToPosition(home_yaw_ratio_onvif, home_pitch_ratio_onvif, self.speed_ratio)

    def setHomePosition(self, yaw_deg, pitch_deg):
        # Have to implement these fully in s/w since ONVIF doesn't support absolute home position setting
        self.home_yaw_deg = yaw_deg
        self.home_pitch_deg = pitch_deg

    def setHomePositionHere(self):
        if self.driver.reportsPosition() is True:
            curr_yaw_ratio_onvif, curr_pitch_ratio_onvif = self.driver.getCurrentPosition()
            if self.ptx_if is not None:
                self.home_yaw_deg = self.yawOvRatioToDeg(curr_yaw_ratio_onvif)
                self.home_pitch_deg = self.pitchOvRatioToDeg(curr_pitch_ratio_onvif) 

        if self.driver.homePositionAdjustable is True:
            self.driver.setHomeHere()

    def gotoWaypoint(self, waypoint_index):
        if self.driver.hasWaypoints() is True:
           self.driver.gotoPreset(waypoint_index, self.speed_ratio)
        elif self.driver.hasAbsolutePositioning() is True and self.ptx_if is not None:
            if waypoint_index not in self.waypoints:
                return
            
            waypoint_yaw_deg = self.waypoints[waypoint_index]['yaw_deg']
            waypoint_yaw_ratio_onvif = self.yawDegToOvRatio(waypoint_yaw_deg)
            waypoint_pitch_deg = self.waypoints[waypoint_index]['pitch_deg']
            waypoint_pitch_ratio_onvif = self.pitchDegToOvRatio(waypoint_pitch_deg)
            self.driver.moveToPosition(waypoint_yaw_ratio_onvif, waypoint_pitch_ratio_onvif, self.speed_ratio)
    
    def setWaypoint(self, waypoint_index, yaw_deg, pitch_deg):
        # Have to implement these fully in s/w since ONVIF doesn't support absolute home position setting
        self.waypoints[waypoint_index] = {'yaw_deg': yaw_deg, 'pitch_deg': pitch_deg}
        
    def setWaypointHere(self, waypoint_index):
        if self.driver.reportsPosition() and self.ptx_if is not None:
            yaw_ratio_onvif, pitch_ratio_onvif = self.driver.getCurrentPosition()
            current_yaw_deg = self.yawOvRatioToDeg(yaw_ratio_onvif)
            current_pitch_deg = self.pitchOvRatioToDeg(pitch_ratio_onvif)
            self.waypoints[waypoint_index] = {'yaw_deg': current_yaw_deg, 'pitch_deg': current_pitch_deg}

        if self.driver.hasWaypoints():
            self.driver.setPresetHere(waypoint_index)

        nepi_msg.publishMsgInfo(self,"Waypoint set to current position")

    def setCurrentSettingsAsDefault(self):
        # Don't need to worry about any of our params in this class, just child interfaces' params
        if self.ptx_if is not None:
            self.ptx_if.setCurrentSettingsToParamServer()

    def updateFromParamServer(self):
        if self.ptx_if is not None:
            self.ptx_if.updateFromParamServer()



if __name__ == '__main__':
	node = OnvifPanTiltNode()
