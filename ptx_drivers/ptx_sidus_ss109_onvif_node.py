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
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings

from nepi_api.device_if_ptx import PTXActuatorIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'PTX_ONVIF_GENERIC' # Use in display menus
FILE_TYPE = 'NODE'



class SidusSs109OnvifNode:
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

    both_str = '!'
    pan_str = '#'
    tilt_str = '$'

    max_speed = 40

    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
    drv_dict = dict()                                                    
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
        self.msg_if.pub_info("Starting IF Initialization Processes")
        ##############################
        # Get required drv driver dict info
        try:
            self.drv_dict = nepi_ros.get_param('~drv_dict') # Crash if not provide
        except Exception as e:
            nepi_ros.signal_shutdown("Failed to read drv_dict from param server for node " + self.node_name + " with exception: " + str(e))
        self.driver_path = self.drv_dict['path']
        self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
        self.driver_module = self.driver_file.split('.')[0]
        self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']


        # Require the camera connection parameters to have been set
        if not nepi_ros.has_param('~credentials/username'):
            self.msg_if.pub_warn("Missing credentials/username parameter... cannot start")
            return
        if not nepi_ros.has_param('~credentials/password'):
            self.msg_if.pub_warn("Missing credentials/password parameter... cannot start")
            return
        if not nepi_ros.has_param('~network/host'):
            self.msg_if.pub_warn("Missing network/host parameter... cannot start")
            return
                
        username = str(nepi_ros.get_param('~credentials/username'))
        password = str(nepi_ros.get_param('~credentials/password'))
        host = str(nepi_ros.get_param('~network/host'))
        
        # Allow a default for the port, since it is part of onvif spec.
        onvif_port = nepi_ros.get_param('~network/port', 80)
        nepi_ros.set_param('~/network/port', onvif_port)


        #self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drv.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        
        driver_constructed = False
        if success:
            attempts = 0
            while not nepi_ros.is_shutdown() and driver_constructed == False and attempts < 5:
                try:
                    self.driver = self.driver_class(username, password, host, onvif_port)
                    driver_constructed = True
                    #self.msg_if.pub_info("Driver constructed")
                except Exception as e:
                    #self.msg_if.pub_info("Failed to construct driver: " + self.driver_module + "with exception: " + str(e))
                    nepi_ros.sleep(1)
                attempts += 1 
        if driver_constructed == False:
            nepi_ros.signal_shutdown("Shutting down Onvif node " + self.node_name + ", unable to connect to driver")
        else:
            ################################################
            #self.msg_if.pub_info("... Connected!")
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
            ptx_remappings = nepi_ros.get_param('~ptx_remappings', {})
            self.msg_if.pub_info("Establishing PTX remappings")
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
            #self.msg_if.pub_info("Launching NEPI PTX (ROS) interface...")
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
            #self.msg_if.pub_info("" +"CAPS SETTINGS")
            #for setting in self.cap_settings:
                ##self.msg_if.pub_info("" +setting)
            self.factory_settings = self.getFactorySettings()
            #self.msg_if.pub_info("" +"FACTORY SETTINGS")
            #for setting in self.factory_settings:
                ##self.msg_if.pub_info("" +setting)

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
            #self.msg_if.pub_info(" ... PTX interface running")


            nepi_ros.spin()



    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["HardwareId"] + "\n"
        #self.msg_if.pub_info(dev_info_string)


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
        if speed_ratio > 1:
            self.driver_setSpeedRatio(speed_ratio)

    def getSpeed(self):
        # TODO: Driver unit conversion?
        speed_ratio = self.driver_getSpeedRatio()
        return speed_ratio
          

    def getCurrentPosition(self):
        yaw_deg, pitch_deg = self.driver_getCurrentPosition()
        #print("Got pos degs : " + str([pan_deg, tilt_deg]))
        return yaw_deg, pitch_deg

    def pitchDegToOvRatio(self, deg):
        ratio = 0.5
        max_ph = nepi_ros.get_param('~ptx/limits/max_pitch_hardstop_deg', self.default_settings['max_pitch_hardstop_deg'])
        min_ph = nepi_ros.get_param('~ptx/limits/min_pitch_hardstop_deg', self.default_settings['min_pitch_hardstop_deg'])
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
        max_yh = nepi_ros.get_param('~ptx/limits/max_yaw_hardstop_deg', self.default_settings['max_yaw_hardstop_deg'])
        min_yh = nepi_ros.get_param('~ptx/limits/min_yaw_hardstop_deg', self.default_settings['min_yaw_hardstop_deg'])
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
        max_yh = nepi_ros.get_param('~ptx/limits/max_yaw_hardstop_deg', self.default_settings['max_yaw_hardstop_deg'])
        min_yh = nepi_ros.get_param('~ptx/limits/min_yaw_hardstop_deg', self.default_settings['min_yaw_hardstop_deg'])
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
        max_ph = nepi_ros.get_param('~ptx/limits/max_pitch_hardstop_deg', self.default_settings['max_pitch_hardstop_deg'])
        min_ph = nepi_ros.get_param('~ptx/limits/min_pitch_hardstop_deg', self.default_settings['min_pitch_hardstop_deg'])
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
        if self.driver_hasAbsolutePositioning() is True and self.ptx_if is not None:
            self.driver_moveToPosition(self.home_yaw_deg, self.home_pitch_deg)

    def setHomePosition(self, yaw_deg, pitch_deg):
        # Have to implement these fully in s/w since ONVIF doesn't support absolute home position setting
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

        #self.msg_if.pub_info("Waypoint set to current position")

    def setCurrentSettingsAsDefault(self):
        # Don't need to worry about any of our params in this class, just child interfaces' params
        if self.ptx_if is not None:
            self.ptx_if.setCurrentSettingsToParamServer()

    def updateFromParamServer(self):
        if self.ptx_if is not None:
            self.ptx_if.updateFromParamServer()

#######################
### Driver Interface Functions

    def degToCount(self, deg):
        encoder_scale_facter = .0879
        count = int(deg / encoder_scale_facter + 5000)
        return count

    def countToDeg(self, deg):
        encoder_scale_facter = .0879
        deg = int((count -5000)* encoder_scale_facter)
        return count


    def driver_getDeviceInfo(self):
        dev_info = dict()
        dev_info["Manufacturer"] = SIDUS
        dev_info["Model"] = SS109

        firmware = ""
        data_str = '0000'
        ser_msg= ('!' + self.addr_str + 'MRV' + data_str + 'R')

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
        data_str = '0000'
        ser_msg= (self.both_str + self.addr_str + 'MS' + data_str + 'R')
        response = self.send_msg(ser_msg)
        if len(response) > 5:
            if response[0:5] == ser_msg[0:5]:
                try:
                    data_str = response[5:-1]
                    cur_speed = int(data_str)
                    speedRatio = cur_speed/self.max_speed
                except Exception as e:
                    print("Failed to convert message to int: " + data_str + " " + str(e))
        else:
             print("Failed to get valid response message from: " + ser_msg)
        return speedRatio

    def driver_setSpeedRatio(self,speedRatio):
        success = False
        try:
            set_speed_str = str(int(speedRatio * self.max_speed))
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
        data_str = '0000'
        ser_msg= (self.pan_str + self.addr_str + 'MRL' + data_str + 'R')
        response = self.send_msg(ser_msg)
        if len(response) > 5:
            if response[0:5] == ser_msg[0:5]:
                try:
                    data_str = response[5:-1]
                    yaw_pos = int(data_str)
                    yaw_deg = self.countToDeg(yaw_pos)
                except Exception as e:
                    print("Failed to convert message to int: " + data_str + " " + str(e))
        else:
             print("Failed to get valid response message from: " + ser_msg)

        pitch_deg = -999
        success = False
        data_str = '0000'
        ser_msg= (self.tilt_str + self.addr_str + 'MRL' + data_str + 'R')
        response = self.send_msg(ser_msg)
        if len(response) > 5:
            if response[0:5] == ser_msg[0:5]:
                try:
                    data_str = response[5:-1]
                    pitch_pos = int(data_str)
                    pitch_deg = self.countToDeg(pitch_pos)
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
        data_str = str(countToDeg(self, yaw_deg))
        ser_msg= (self.pan_str + self.addr_str + 'MML' + data_str + 'W')
        response = self.send_msg(ser_msg)  
        check_1 = data_str == ser_msg
        
        data_str = str(countToDeg(self, pitch_deg))
        ser_msg= (self.tilt_str + self.addr_str + 'MML' + data_str + 'W')
        response = self.send_msg(ser_msg)  
        check_2 = data_str == ser_msg
        success = (check_1 == True and check_2 == True) 
        return success


    def driver_stopMotion(self):
        success = False
        data_str = '0000'
        ser_msg= (self.both_str + self.addr_str + 'MST' + data_str + 'W')
        response = self.send_msg(ser_msg)  
        success = data_str == ser_msg
        return success


    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        #self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        if self.serial_port is not None:
            self.serial_port.close()


if __name__ == '__main__':
	node = SidusSs109OnvifNode()
