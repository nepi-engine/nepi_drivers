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
from std_msgs.msg import UInt8, Float32, Empty, Bool
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nepi_ros_interfaces.msg import PanTiltStatus, PanTiltLimits, PanTiltPosition, SingleAxisTimedMove, AbsolutePanTiltWaypoint
from nepi_ros_interfaces.srv import PTXCapabilitiesQuery, PTXCapabilitiesQueryResponse

from tf.transformations import quaternion_from_euler

from nepi_edge_sdk_base.settings_if import SettingsIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

class ROSPTXActuatorIF:
    PTX_DIRECTION_POSITIVE = 1
    PTX_DIRECTION_NEGATIVE = -1
    WAYPOINT_COUNT = 256

    # Backup Factory Control Values 
    FACTORY_CONTROLS_DICT = {
                'frame_id' : 'ptx_frame',
                'yaw_joint_name' : 'ptx_yaw_joint',
                'pitch_joint_name' : 'ptx_pitch_joint',
                'reverse_yaw_control' : False,
                'reverse_pitch_control' : False,
                'speed_ratio' : 1.0,
                'status_update_rate_hz' : 1.0
    }



    def __init__(self,  device_info, capSettings, 
                 factorySettings, settingUpdateFunction, getSettingsFunction,
                 factoryControls , # Dictionary to be supplied by parent, specific key set is required
                 defaultSettings,
                 capabilities_dict, # Dictionary to be supplied by parent, specific key set is required
                 stopMovingCb, # Required; no args
                 moveYawCb, # Required; direction and time args
                 movePitchCb, # Required; direction and time args
                 setSpeedCb=None, # None ==> No speed adjustment capability; Speed ratio arg
                 getSpeedCb=None, # None ==> No speed adjustment capabilitiy; Returns speed ratio
                 gotoPositionCb=None, # None ==> No absolute positioning capability (yaw_deg, pitch_deg, speed, float move_timeout_s) 
                 getCurrentPositionCb=None, # None ==> no positional feedback; 
                 goHomeCb=None, # None ==> No native driver homing capability, can still use homing if absolute positioning is supported
                 setHomePositionCb=None, # None ==> No native driver home absolute setting capability, can still use it if absolute positioning is supported
                 setHomePositionHereCb=None, # None ==> No native driver home instant capture capability, can still use it if absolute positioning is supported
                 gotoWaypointCb=None, # None ==> No native driver support for waypoints, can still use if absolute positioning is supported
                 setWaypointCb=None, # None ==> No native driver support for absolute waypoints, can still use if absolute positioning is supported
                 setWaypointHereCb=None, # None ==> No native driver support for instant waypoints, can still use if absolute positioning is supported
                ):

        self.node_name = device_info["node_name"]
        self.device_name = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.factory_device_name = device_info["device_name"] + "_" + device_info["identifier"]


        # Create and update factory controls dictionary
        self.factory_controls_dict = self.FACTORY_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]

        self.frame_id = rospy.get_param('~ptx/frame_id', self.factory_controls_dict['frame_id'])
        self.yaw_joint_name = rospy.get_param("~ptx/yaw_joint_name", self.factory_controls_dict['yaw_joint_name'])
        self.pitch_joint_name = rospy.get_param("~ptx/pitch_joint_name", self.factory_controls_dict['pitch_joint_name'])
        self.reverse_yaw_control = rospy.get_param("~ptx/reverse_yaw_control", self.factory_controls_dict['reverse_yaw_control'])
        self.reverse_pitch_control = rospy.get_param("~ptx/reverse_pitch_control", self.factory_controls_dict['reverse_pitch_control'])

        
        # Set up status message static values
        self.status_msg = PanTiltStatus()
        self.status_msg.header.frame_id = self.frame_id
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

        # And joint state status static values
        self.joint_state_msg = JointState()
        # Skip the header -- we just copy it from the status message each time
        self.joint_state_msg.name = (self.yaw_joint_name, self.pitch_joint_name)
        self.joint_state_msg.position.append(0.0) # Yaw
        self.joint_state_msg.position.append(0.0) # Pitch

        # And odom static values
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.device_name + '_fixed_frame'
        self.odom_msg.child_frame_id = self.device_name + '_rotating_frame'
        
        # Gather capabilities - Config file takes precedence over parent-supplied defaults
        self.capabilities_report = PTXCapabilitiesQueryResponse()
        self.capabilities_report.absolute_positioning = rospy.get_param('~ptx/capabilities/has_absolute_positioning', capabilities_dict['has_absolute_positioning'])
        self.capabilities_report.adjustable_speed = rospy.get_param('~ptx/capabilities/has_speed_control', capabilities_dict['has_speed_control'])
        self.capabilities_report.homing = rospy.get_param('ptx/capabilities/has_homing', capabilities_dict['has_homing'])
        self.capabilities_report.waypoints = rospy.get_param('ptx/capabilities/has_waypoints', capabilities_dict['has_waypoints'])
        
        # Define some member variables
        self.yaw_goal_deg = 0.0
        self.yaw_home_pos_deg = 0.0
        self.min_yaw_softstop_deg = 0.0
        self.max_yaw_softstop_deg = 0.0
        self.pitch_goal_deg = 0.0
        self.pitch_home_pos_deg = 0.0
        self.min_pitch_softstop_deg = 0.0
        self.max_pitch_softstop_deg = 0.0

        # Stop motion setup
        self.stopMovingCb = stopMovingCb
        rospy.Subscriber('~ptx/stop_moving', Empty, self.stopMovingHandler, queue_size=1)

        # Timed jog setup
        self.moveYawCb = moveYawCb
        rospy.Subscriber('~ptx/jog_timed_yaw', SingleAxisTimedMove, self.jogTimedYawHandler, queue_size=1)
        self.movePitchCb = movePitchCb
        rospy.Subscriber('~ptx/jog_timed_pitch', SingleAxisTimedMove, self.jogTimedPitchHandler, queue_size=1)

        # Reverse controls setup
        rospy.Subscriber('~ptx/reverse_yaw_control', Bool, self.setReverseYawControl, queue_size=1)
        rospy.Subscriber('~ptx/reverse_pitch_control', Bool, self.setReversePitchControl, queue_size=1)

        # Speed setup if available
        if self.capabilities_report.adjustable_speed is True:
            self.setSpeedCb = setSpeedCb
            self.getSpeedCb = getSpeedCb
            if self.setSpeedCb is not None and self.getSpeedCb is not None:
                rospy.Subscriber('~ptx/set_speed_ratio', Float32, self.setSpeedRatioHandler, queue_size=1)

                speed_ratio = rospy.get_param('~ptx/speed_ratio', self.factory_controls_dict['speed_ratio'])
                self.setSpeedCb(speed_ratio)
                self.capabilities_report.adjustable_speed = True
            else:
                rospy.logwarn('Inconsistent capabilities: adjustable speed reports true, but no callback provided')
                self.capabilities_report.adjustable_speed = False
        
        # Positioning and soft limits setup if available
        if self.capabilities_report.absolute_positioning is True:
            self.gotoPositionCb = gotoPositionCb
            self.getCurrentPositionCb = getCurrentPositionCb
            if (self.gotoPositionCb is None) or (self.getCurrentPositionCb is None):
                rospy.logwarn('Inconsistent capabilities: absolute positioning reports true, but no callback provided')
                self.capabilities_report.adjustable_speed = False
                # We require both command and feedback reporting to support absolute positioning
                self.capabilities_report.absolute_positioning = False
                
        self.defaultSettings = defaultSettings
        if self.capabilities_report.absolute_positioning is True:
            # Hard limits
            self.max_yaw_hardstop_deg = rospy.get_param('~ptx/limits/max_yaw_hardstop_deg', self.defaultSettings['max_yaw_hardstop_deg'])
            self.min_yaw_hardstop_deg = rospy.get_param('~ptx/limits/min_yaw_hardstop_deg', self.defaultSettings['min_yaw_hardstop_deg'])
            self.max_pitch_hardstop_deg = rospy.get_param('~ptx/limits/max_pitch_hardstop_deg', self.defaultSettings['max_pitch_hardstop_deg'])
            self.min_pitch_hardstop_deg = rospy.get_param('~ptx/limits/min_pitch_hardstop_deg', self.defaultSettings['min_pitch_hardstop_deg'])
                        
            # Soft limits
            self.max_yaw_softstop_deg = rospy.get_param('~ptx/limits/max_yaw_softstop_deg', self.defaultSettings['max_yaw_softstop_deg'])
            self.min_yaw_softstop_deg = rospy.get_param('~ptx/limits/min_yaw_softstop_deg', self.defaultSettings['min_yaw_softstop_deg'])
            self.max_pitch_softstop_deg = rospy.get_param('~ptx/limits/max_pitch_softstop_deg', self.defaultSettings['max_pitch_softstop_deg'])
            self.min_pitch_softstop_deg = rospy.get_param('~ptx/limits/min_pitch_softstop_deg', self.defaultSettings['min_pitch_softstop_deg'])

            # Jog to position
            rospy.Subscriber('~ptx/jog_to_position', PanTiltPosition, self.jogToPositionHandler, queue_size=1)

            # Jog to yaw ratio
            rospy.Subscriber('~ptx/jog_to_yaw_ratio', Float32, self.jogToYawRatioHandler, queue_size=1)

            # Jog to pitch ratio
            rospy.Subscriber('~ptx/jog_to_pitch_ratio', Float32, self.jogToPitchRatioHandler, queue_size=1)

            # Joint state publisher
            self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

            # Odometry publisher
            self.odom_pub = rospy.Publisher('~ptx/odometry', Odometry, queue_size=10)
        
        else:
            self.max_yaw_hardstop_deg = 0.0
            self.min_yaw_hardstop_deg = 0.0
            self.max_pitch_hardstop_deg = 0.0
            self.min_pitch_hardstop_deg = 0.0
                        
            # Soft limits
            self.max_yaw_softstop_deg = 0.0
            self.min_yaw_softstop_deg = 0.0
            self.max_pitch_softstop_deg = 0.0
            self.min_pitch_softstop_deg = 0.0

            self.joint_pub = None
            self.odom_pub = None

        # Hardstop is not adjustable, so just set the status message values once here
        self.status_msg.yaw_min_hardstop_deg = self.min_yaw_hardstop_deg
        self.status_msg.yaw_max_hardstop_deg = self.max_yaw_hardstop_deg
        self.status_msg.pitch_min_hardstop_deg = self.min_pitch_hardstop_deg
        self.status_msg.pitch_max_hardstop_deg = self.max_pitch_hardstop_deg

        # Homing setup
        if self.capabilities_report.homing is True:
            self.goHomeCb = goHomeCb
            self.setHomePositionCb = setHomePositionCb
            self.setHomePositionHereCb = setHomePositionHereCb
        
            if self.goHomeCb is None and self.capabilities_report.absolute_positiong is False:
                rospy.logwarn('Inconsistent capabilities: homing reports true, but no goHome callback provided and no absolute positioning')
                self.capabilities_report.homing = False
                
        if self.capabilities_report.homing is True:
            rospy.Subscriber('~ptx/go_home', Empty, self.goHomeHandler, queue_size=1)
            
            if self.setHomePositionCb is not None:
                rospy.Subscriber('~ptx/set_home_position', PanTiltPosition, self.setHomePositionHandler,  queue_size=1)

            if self.setHomePositionHereCb is not None:
                rospy.Subscriber('~ptx/set_home_position_here', Empty, self.setHomePositionHereHandler, queue_size=1)
            
            self.home_yaw_deg = rospy.get_param('~ptx/home_position/yaw_deg', 0.0)
            self.home_pitch_deg = rospy.get_param('~ptx/home_position/pitch_deg', 0.0)
                
        # Waypoint setup
        if self.capabilities_report.waypoints is True:
            self.gotoWaypointCb = gotoWaypointCb
            self.setWaypointCb = setWaypointCb
            self.setWaypointHereCb = setWaypointHereCb

            if self.gotoWaypointCb is None:
                rospy.logwarn('Inconsistent capabilities: waypoints reports true, but no gotoWaypoint callback provided')
                self.capabilities_report.waypoints = False
                
        if self.capabilities_report.waypoints is True:
            rospy.Subscriber('~ptx/goto_waypoint', UInt8, self.gotoWaypointHandler, queue_size=1)

            if self.setWaypointCb is not None:
                rospy.Subscriber('~ptx/set_waypoint', AbsolutePanTiltWaypoint, self.setWaypointHandler, queue_size=1)

            if self.setWaypointHereCb is not None:
                rospy.Subscriber('~ptx/set_waypoint_here', UInt8, self.setWaypointHereHandler, queue_size=1)
            
        # TODO: Read waypoints in from a config. file? 

        # Set up publishers        
        self.status_pub = rospy.Publisher('~ptx/status', PanTiltStatus, queue_size=10, latch=True)

        # Periodic publishing
        self.status_update_rate = rospy.get_param('~status_update_rate_hz', self.factory_controls_dict['status_update_rate_hz'])       
        status_joint_state_pub_period = rospy.Duration(1.0 / self.status_update_rate)
        rospy.Timer(status_joint_state_pub_period, self.publishJointStateAndStatus)
        
        # Set up service providers
        rospy.Service('~ptx/capabilities_query', PTXCapabilitiesQuery, self.provideCapabilities)


        self.settings_if = SettingsIF(capSettings, factorySettings, settingUpdateFunction, getSettingsFunction)
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initializeParamServer, paramsModifiedCallback=self.updateFromParamServer)


        # Update and Publish Status Message
        self.publishStatus()
        ## Initiation Complete
        rospy.loginfo(" PTX_IF: Initialization Complete")

    def yawRatioToDeg(self, ratio):
        return  ratio * (self.max_yaw_softstop_deg - self.min_yaw_softstop_deg) + self.min_yaw_softstop_deg
    
    def yawDegToRatio(self, deg):
        return (deg - self.min_yaw_softstop_deg) / (self.max_yaw_softstop_deg - self.min_yaw_softstop_deg)
    
    def pitchDegToRatio(self, deg):
        return (deg - self.min_pitch_softstop_deg) / (self.max_pitch_softstop_deg - self.min_pitch_softstop_deg)
    
    def pitchRatioToDeg(self, ratio):
        return ratio * (self.max_pitch_softstop_deg - self.min_pitch_softstop_deg) + self.min_pitch_softstop_deg

    def publishJointStateAndStatus(self, _):
            self.publishStatus()

    def publishStatus(self):
        self.status_msg.header.seq += 1
        self.status_msg.header.stamp = rospy.Time.now()
        
        self.status_msg.reverse_yaw_control = self.reverse_yaw_control
        self.status_msg.reverse_pitch_control = self.reverse_pitch_control
        self.status_msg.yaw_goal_deg = self.yaw_goal_deg
        self.status_msg.yaw_home_pos_deg = self.yaw_home_pos_deg
        self.status_msg.yaw_min_softstop_deg = self.min_yaw_softstop_deg
        self.status_msg.yaw_max_softstop_deg = self.max_yaw_softstop_deg
        self.status_msg.pitch_goal_deg = self.pitch_goal_deg
        self.status_msg.pitch_home_pos_deg = self.pitch_home_pos_deg
        self.status_msg.pitch_min_softstop_deg = self.min_pitch_softstop_deg
        self.status_msg.pitch_max_softstop_deg = self.max_pitch_softstop_deg

        if self.capabilities_report.absolute_positioning is True:
            self.status_msg.yaw_now_deg, self.status_msg.pitch_now_deg = self.getCurrentPositionCb()
        
        if self.capabilities_report.adjustable_speed is True:
            self.status_msg.speed_ratio = self.getSpeedCb()

        #self.status_msg.error_msgs = ??? # TODO
        self.status_pub.publish(self.status_msg)

        yaw_rad = 0.01745329 * self.status_msg.yaw_now_deg
        pitch_rad = 0.01745329 * self.status_msg.pitch_now_deg

        # And joint state if appropriate
        if self.joint_pub is not None:
            self.joint_state_msg.header = self.status_msg.header
            self.joint_state_msg.position[0] = yaw_rad
            self.joint_state_msg.position[1] = pitch_rad
            self.joint_pub.publish(self.joint_state_msg)

        if self.odom_pub is not None:
            self.odom_msg.header.seq = self.status_msg.header.seq
            self.odom_msg.header.stamp = self.status_msg.header.stamp
            self.odom_msg.pose.pose.orientation = quaternion_from_euler(0.0, pitch_rad, yaw_rad)
            self.odom_pub.publish(self.odom_msg)

    def positionWithinSoftLimits(self, yaw_deg, pitch_deg):
        if (yaw_deg < self.min_yaw_softstop_deg) or (yaw_deg > self.max_yaw_softstop_deg) or \
           (pitch_deg < self.min_pitch_softstop_deg) or (pitch_deg > self.max_pitch_softstop_deg):
            return False
        
        return True

    def setSpeedRatioHandler(self, msg):
        speed_ratio = msg.data
        if (speed_ratio < 0.0) or (speed_ratio > 1.0):
            rospy.logwarn(" PTX_IF: Invalid speed ratio requested (%.2f)... ignoring", speed_ratio)

        self.setSpeedCb(speed_ratio)
        rospy.loginfo(" PTX_IF: Updated speed ratio to " + str(speed_ratio))

    def setHomePositionHandler(self, msg):
        if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
            rospy.logwarn(" PTX_IF: Requested home position is invalid... ignoring")
            return

        if self.setHomePositionCb is not None:
            # Driver supports absolute positioning, so just let it operate
            self.home_yaw_deg = msg.yaw_deg
            self.home_pitch_deg = msg.pitch_deg
            self.setHomePositionCb(self.home_yaw_deg, self.home_pitch_deg)
        else:
            rospy.logwarn(" PTX_IF: Absolution position home setpoints not available... ignoring")
            return
        
        rospy.loginfo(" PTX_IF: Updated home position to (%.2f,%.2f)",  self.home_yaw_deg, self.home_pitch_deg)
    
    def setSoftLimitsHandler(self, msg):
        if (msg.min_yaw_softstop_deg < self.min_yaw_hardstop_deg) or \
           (msg.max_yaw_softstop_deg < self.max_yaw_hardstop_deg) or \
           (msg.min_pitch_softstop_deg < self.min_pitch_hardstop_deg) or \
           (msg.max_pitch_softstop_deg < self.max_pitch_hardstop_deg):
            rospy.logwarn(" PTX_IF: Soft limits cannot exceed hard limits... ignoring");
            return
        
        self.min_yaw_softstop_deg = msg.min_yaw_softstop_deg
        self.max_yaw_softstop_deg = msg.max_yaw_softstop_deg
        self.min_pitch_softstop_deg = msg.min_pitch_softstop_deg
        self.max_pitch_softstop_deg = msg.max_pitch_softstop_deg
        rospy.loginfo(" PTX_IF: Updated softstop limits")

    def goHomeHandler(self, _):
        if self.goHomeCb is not None:
            self.goHomeCb()

    def jogToPositionHandler(self, msg):
        if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
            rospy.logwarn(" PTX_IF: Requested jog position is invalid... ignoring")
            return

        self.yaw_goal_deg = msg.yaw_deg
        self.pitch_goal_deg = msg.pitch_deg
        self.gotoPositionCb(yaw_deg = msg.yaw_deg, pitch_deg = msg.pitch_deg,)
        rospy.loginfo(" PTX_IF: Driving to (%0.2f, %0.2f) by request", msg.yaw_deg, msg.pitch_deg)
            
    def jogToYawRatioHandler(self, msg):
        ratio = msg.data if self.reverse_yaw_control is False else (1.0 - msg.data)
        if (ratio < 0.0 or ratio > 1.0):
            rospy.logwarn(" PTX_IF: Invalid yaw position ratio %0.2f... ignoring", ratio)
            return
        
        self.yaw_goal_deg = self.yawRatioToDeg(ratio)
        _, pitch_now_deg = self.getCurrentPositionCb()
        self.gotoPositionCb(yaw_deg = self.yaw_goal_deg, pitch_deg = pitch_now_deg)

    def jogToPitchRatioHandler(self, msg):
        ratio = msg.data if self.reverse_pitch_control is False else (1.0 - msg.data)
        if (ratio < 0.0 or ratio > 1.0):
            rospy.logwarn(" PTX_IF: Invalid pitch position ratio %0.2f... ignoring", ratio)
            return
        
        self.pitch_goal_deg = self.pitchRatioToDeg(ratio)
        yaw_now_deg, _ = self.getCurrentPositionCb()
        self.gotoPositionCb(yaw_deg = yaw_now_deg, pitch_deg = self.pitch_goal_deg)

    def stopMovingHandler(self, _):
        self.stopMovingCb()
        rospy.loginfo(" PTX_IF: Stopping motion by request")

    def jogTimedYawHandler(self, msg):
        direction = msg.direction if self.reverse_yaw_control is False else (-1 * msg.direction)
        duration = 1000000.0 if (msg.duration_s < 0.0) else msg.duration_s

        self.moveYawCb(direction,  duration)
        rospy.loginfo(" PTX_IF: Jogging yaw")

    def jogTimedPitchHandler(self, msg):
        direction = msg.direction if self.reverse_pitch_control is False else (-1 * msg.direction)
        duration = 1000000.0 if (msg.duration_s < 0.0) else msg.duration_s

        self.movePitchCb(direction, duration)
        rospy.loginfo(" PTX_IF: Jogging pitch")

    def setReverseYawControl(self, msg):
        self.reverse_yaw_control = msg.data
        rospy.loginfo(" PTX_IF: Set yaw control to reverse=" + str(self.reverse_yaw_control))

    def setReversePitchControl(self, msg):
        self.reverse_pitch_control = msg.data
        rospy.loginfo(" PTX_IF: Set pitch control to reverse=" + str(self.reverse_pitch_control))

    def setHomePositionHereHandler(self, _):
        if self.setHomePositionHereCb is not None:
            # Driver supports it directly
            # Capture home position if possible
            if self.getCurrentPositionCb is not None:
                self.home_yaw_deg, self.home_pitch_deg = self.getCurrentPositionCb()
            self.setHomePositionHereCb()
        else:
            rospy.logwarn(" PTX_IF: Instant home position not available for this device")
            return
        
        rospy.loginfo(" PTX_IF: Updated home position to current position")

    def setWaypointHandler(self, msg):
        yaw_deg = msg.yaw_deg
        pitch_deg = msg.pitch_deg
        waypoint_index = msg.waypoint_index

        if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
            rospy.logwarn(" PTX_IF: Requested waypoint position is invalid... ignoring")
            return

        if self.setWaypointCb is not None:
            self.setWaypointCb(waypoint_index, yaw_deg, pitch_deg)

        rospy.loginfo(" PTX_IF: Waypoint %u set to [%.2f, %.2f]", waypoint_index, yaw_deg, pitch_deg)

    def setWaypointHereHandler(self, msg):
        waypoint_index = msg.data
        if self.setWaypointHereCb is not None:
            self.setWaypointHereCb(waypoint_index)
    
    def gotoWaypointHandler(self, msg):
        waypoint_index = msg.data

        if self.gotoWaypointCb is not None:
            self.gotoWaypointCb(waypoint_index)
            rospy.loginfo(" PTX_IF: Going to waypoint %u by command", waypoint_index)
    
    def provideCapabilities(self, _):
        return self.capabilities_report
    
    def initializeParamServer(self):
        rospy.set_param('~status_update_rate_hz', self.status_update_rate)

        rospy.set_param('~ptx/frame_id', self.frame_id)
        rospy.set_param("~ptx/yaw_joint_name", self.yaw_joint_name)
        rospy.set_param("~ptx/pitch_joint_name", self.pitch_joint_name)
        rospy.set_param("~ptx/reverse_yaw_control", self.reverse_yaw_control)
        rospy.set_param("~ptx/reverse_pitch_control", self.reverse_pitch_control)

        rospy.set_param('~ptx/capabilities/has_speed_control', self.capabilities_report.adjustable_speed)
        rospy.set_param('~ptx/capabilities/has_absolute_positioning', self.capabilities_report.absolute_positioning)
        rospy.set_param('~ptx/capabilities/has_homing', self.capabilities_report.homing)
        rospy.set_param('~ptx/capabilities/has_waypoints', self.capabilities_report.waypoints)

        if (self.capabilities_report.adjustable_speed is True):
            rospy.set_param("~ptx/speed_ratio", self.getSpeedCb()) # This one comes from the parent
        
        if (self.capabilities_report.absolute_positioning is True):
            rospy.set_param('~ptx/limits/max_yaw_hardstop_deg', self.max_yaw_hardstop_deg)
            rospy.set_param('~ptx/limits/min_yaw_hardstop_deg', self.min_yaw_hardstop_deg)
            rospy.set_param('~ptx/limits/max_pitch_hardstop_deg', self.max_pitch_hardstop_deg)
            rospy.set_param('~ptx/limits/min_pitch_hardstop_deg', self.min_pitch_hardstop_deg)
            rospy.set_param('~ptx/limits/max_yaw_softstop_deg', self.max_yaw_softstop_deg)
            rospy.set_param('~ptx/limits/min_yaw_softstop_deg', self.min_yaw_softstop_deg)
            rospy.set_param('~ptx/limits/max_pitch_softstop_deg', self.max_pitch_softstop_deg)
            rospy.set_param('~ptx/limits/min_pitch_softstop_deg', self.min_pitch_softstop_deg)

        if (self.capabilities_report.homing is True):
            rospy.set_param('~ptx/home_position/yaw_deg', self.home_yaw_deg)
            rospy.set_param('~ptx/home_position/pitch_deg', self.home_pitch_deg)

    def updateFromParamServer(self):
        self.status_update_rate = rospy.get_param('~status_update_rate_hz', self.status_update_rate) 

        self.frame_id = rospy.get_param('~ptx/frame_id', self.frame_id)
        self.yaw_joint_name = rospy.get_param("~ptx/yaw_joint_name", self.yaw_joint_name)
        self.pitch_joint_name = rospy.get_param("~ptx/pitch_joint_name", self.pitch_joint_name)
        self.reverse_yaw_control = rospy.get_param("~ptx/reverse_yaw_control", self.reverse_yaw_control)
        self.reverse_pitch_control = rospy.get_param("~ptx/reverse_pitch_control", self.reverse_pitch_control)

        # Do caps next, since others rely on them
        self.capabilities_report.adjustable_speed = rospy.get_param('~ptx/capabilities/has_speed_control', self.capabilities_report.adjustable_speed)
        self.capabilities_report.absolute_positioning = rospy.get_param('~ptx/capabilities/has_absolute_positioning', self.capabilities_report.absolute_positioning)
        self.capabilities_report.homing = rospy.get_param('~ptx/capabilities/has_homing', self.capabilities_report.homing)
        self.capabilities_report.waypoints = rospy.get_param('~ptx/capabilities/has_waypoints', self.capabilities_report.waypoints)

        if (self.capabilities_report.adjustable_speed is True):
            speed_ratio = rospy.get_param('~ptx/speed_ratio', self.getSpeedCb)
            self.setSpeedCb(speed_ratio)

        if self.capabilities_report.absolute_positioning is True:
            # Hard limits
            self.max_yaw_hardstop_deg = rospy.get_param('~ptx/limits/max_yaw_hardstop_deg', self.max_yaw_hardstop_deg)
            self.min_yaw_hardstop_deg = rospy.get_param('~ptx/limits/min_yaw_hardstop_deg', self.min_yaw_hardstop_deg)
            self.max_pitch_hardstop_deg = rospy.get_param('~ptx/limits/max_pitch_hardstop_deg', self.max_pitch_hardstop_deg)
            self.min_pitch_hardstop_deg = rospy.get_param('~ptx/limits/min_pitch_hardstop_deg', self.min_pitch_hardstop_deg)
                        
            # Soft limits
            self.max_yaw_softstop_deg = rospy.get_param('~ptx/limits/max_yaw_softstop_deg', self.max_yaw_softstop_deg)
            self.min_yaw_softstop_deg = rospy.get_param('~ptx/limits/min_yaw_softstop_deg', self.min_yaw_softstop_deg)
            self.max_pitch_softstop_deg = rospy.get_param('~ptx/limits/max_pitch_softstop_deg', self.max_pitch_softstop_deg)
            self.min_pitch_softstop_deg = rospy.get_param('~ptx/limits/min_pitch_softstop_deg', self.min_pitch_softstop_deg)

        if (self.capabilities_report.homing is True):
            self.home_yaw_deg = rospy.get_param('~ptx/home_position/yaw_deg', self.home_yaw_deg)
            self.home_pitch_deg = rospy.set_param('~ptx/home_position/pitch_deg', self.home_pitch_deg)

        
        
