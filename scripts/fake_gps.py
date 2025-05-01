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


# Sample NEPI Config Script.
# 1) Publishes a fake GPS MAVLink Message 
# Provides two ROS control topics
# a) goto_geopoint_wgs84 - Simulates move to new geopoint
# b) reset_geopoint_wgs84 - Resets GPS and global x,y NED home position at new geopoint
# c) reset_current - Resets GPS and global x,y NED home position at current geopoint
# d) subscribe to RBX command topics and apply position moves
# Fake GPS control messages take a GeoPoint with WGS84 Height for Atlitude

# Requires the following additional scripts are running
# a) None
# These scripts are available for download at:
# [link text](https://github.com/numurus-nepi/nepi_sample_auto_scripts)

###################################################
### For Ardupilot Mavlink Support
### These Ardupilot Parameters Must Be Configured First to allow MAVLINK GPS Override
### There maybe better configuration options, lots of settings to play with
#GPS_TYPE = 14
#GPS_DELAY_MS = 1
#EK3_POS_I_GATE = 300
#EK3_POSNE_M_NSE = 5
#EK3_SRC_OPTIONS = 0
#EK3_SRC1_POSXY = 3
#EK3_SRC1_POSZ = 3
#EK3_SRC1_VELXY = 3
#EK3_SRC1_VELZ = 3
#EK3_SRC1_YAW = 1
#BARO_OPTION = 1  (This was required for proper barometer reading on Pixhawk)
#####################################################


import os
import rospy

import rosnode
import time
import sys
import numpy as np
import math
import random
import copy
from nepi_sdk import nepi_ros 
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_rbx

from std_msgs.msg import Empty, Bool, UInt8, Int8, Float32, Float64, String, Header
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from nepi_ros_interfaces.msg import RBXGotoPose, RBXGotoPosition, RBXGotoLocation
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest

from mavros_msgs.msg import HilGPS, State

MAX_MOVE_TIME_S = 20

#########################################
# Node Class
#########################################

class RBXFakeGPS:

  DEFAULT_NODE_NAME = "fake_gps" # connection port added once discovered

  # ROS namespace setup
  NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

  #Homeup Location
  # [Lat, Long, Altitude_WGS84]
  FAKE_GPS_START_GEOPOINT_WGS84 = [46.6540828,-122.3187578,0.0]
  ZERO_POINT = Point()
  ZERO_POINT.x = 0
  ZERO_POINT.y = 0
  ZERO_POINT.z = 0
  

  #GPS Setup
  SAT_COUNT = 20
  GPS_PUB_RATE_HZ = 50

  #GPS Simulation Position Update Controls
  #Adjust these settings for smoother xyz movements
  MOVE_UPDATE_TIME_SEC_PER_METER=1

  # Create shared class variables and thread locks
  fake_gps_enabled = False
  stop_triggered = False
  
  reset_point = copy.deepcopy(ZERO_POINT)
  current_point = copy.deepcopy(ZERO_POINT)
  new_point = copy.deepcopy(ZERO_POINT)



  ###################################################
  # Init Fake GPS Node
  def __init__(self):
    ####  IF INIT SETUP ####
    rospy.init_node(self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting IF Initialization Processes")
    ##############################

    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ## Initialize Class Variables
    # RBX State and Mode Dictionaries
    self.rbx_cap_modes = []
    self.rbx_cap_actions = []
    self.current_location_wgs84_geo = None
    self.current_heading_deg = 0
    self.current_yaw_enu_deg = 0
    self.current_home_wgs84_geo = None
    self.navpose_update_interval = 0.1
    self.fake_gps_ready = True
    self.gps_publish_interval_sec=1.0/self.GPS_PUB_RATE_HZ
    self.takeoff_m = 0
    # Initialize Current Location
    cur_loc_str = str(self.FAKE_GPS_START_GEOPOINT_WGS84)
    nepi_msg.publishMsgInfo(self,"Start Home GEO Location: " + cur_loc_str)
    self.current_location_wgs84_geo=GeoPoint()
    self.current_location_wgs84_geo.latitude = self.FAKE_GPS_START_GEOPOINT_WGS84[0]
    self.current_location_wgs84_geo.longitude = self.FAKE_GPS_START_GEOPOINT_WGS84[1]
    self.current_location_wgs84_geo.altitude = self.FAKE_GPS_START_GEOPOINT_WGS84[2]


    # Start navpose callbacks
    self.nepi_nav_service_name = nepi_ros.get_base_namespace() + "nav_pose_query"
    nepi_msg.publishMsgInfo(self,"will call NEPI navpose service for current heading at: " + self.nepi_nav_service_name)
    rospy.Timer(rospy.Duration(self.navpose_update_interval), self.update_current_heading_callback)
    
    #Start Mavlink Fake GPS publisher if needed
    # Check if need to send Mavlink message
    mav_node_name = self.node_name.replace("fake_gps","mavlink")
    nepi_msg.publishMsgInfo(self,"checking for mavlink node that includes: " + mav_node_name)
    mav_node_name = nepi_ros.find_node(mav_node_name)
    if mav_node_name != "":
      MAVLINK_NAMESPACE = (mav_node_name + '/')
      nepi_msg.publishMsgInfo(self,"Found mavlink namespace: " + MAVLINK_NAMESPACE)
      # MAVLINK Fake GPS Publish Topic
      MAVLINK_HILGPS_TOPIC = MAVLINK_NAMESPACE + "hil/gps"
      nepi_msg.publishMsgInfo(self,"Will publish fake gps on mavlink topic: " + MAVLINK_HILGPS_TOPIC)
      self.mavlink_pub = rospy.Publisher(MAVLINK_HILGPS_TOPIC, HilGPS, queue_size=1)
      nepi_msg.publishMsgInfo(self,"Fake gps publishing to " + MAVLINK_HILGPS_TOPIC)
      self.send_mavlink_gps_msg = True
    else:
      self.send_mavlink_gps_msg = False
    
    # Start fake gps publishing
    self.odom_msg = Odometry()
    #self.odom_msg.header.frame_id = self.name + '_fixed_frame'
    #self.odom_msg.child_frame_id = self.name + '_rotating_frame'
    self.fake_odom_pub = rospy.Publisher("~odom", Odometry, queue_size=1)
    self.fake_gps_pub = rospy.Publisher("~gps_fix", NavSatFix, queue_size=1)
    time.sleep(1)
    rospy.Timer(rospy.Duration(self.gps_publish_interval_sec), self.fake_gps_pub_callback)

   # Setup RBX driver interfaces
    nepi_msg.publishMsgInfo(self,"Got fake gps node name: " + self.node_name)
    robot_namespace = self.node_name.replace("fake_gps","ardupilot")
    nepi_msg.publishMsgInfo(self,"Waiting for RBX node that includes string: " + robot_namespace)
    robot_namespace = nepi_ros.wait_for_node(robot_namespace)
    robot_namespace = robot_namespace.split("/rbx")[0] + "/"
    rbx_namespace = (robot_namespace + 'rbx/')
    nepi_msg.publishMsgInfo(self,"Found rbx namespace: " + rbx_namespace)
        
    # Create Fake GPS controls subscribers
    rospy.Subscriber("~enable", Bool, self.fakeGPSEnableCb)
    rospy.Subscriber("~reset", GeoPoint, self.fakeGPSResetLocCb)
    rospy.Subscriber("~go_stop", Empty, self.fakeGPSGoStopCb)
    rospy.Subscriber("~goto_position", Point, self.fakeGPSGoPosCb)
    rospy.Subscriber("~goto_location", GeoPoint, self.fakeGPSGoLocCb)


    self.status_pub = rospy.Publisher("~status", Bool, queue_size=1, latch = True)
    time.sleep(1)
    self.status_pub.publish(self.fake_gps_enabled)
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self,"Initialization Complete")

    #Set up node shutdown
    #rospy.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    rospy.spin()

  #######################
  ### Node Methods


  ### Setup a regular Send Fake GPS callback using current geo point value
  def fake_gps_pub_callback(self,timer):
    if self.fake_gps_ready:
      self.publishFakeGPS()

  def publishFakeGPS(self):
    if self.fake_gps_enabled:
      # Publish a fake gps message
      navsatfix = NavSatFix()
      navsatfix.header.stamp = rospy.Time.now()
      navsatfix.latitude = self.current_location_wgs84_geo.latitude
      navsatfix.longitude = self.current_location_wgs84_geo.longitude
      navsatfix.altitude = self.current_location_wgs84_geo.altitude
      #nepi_msg.publishMsgInfo(self,navsatfix)
      # Calculate position change from reset position
      self.odom_msg.header.stamp = rospy.Time.now()
      self.odom_msg.pose.pose.position.x = self.current_point.x
      self.odom_msg.pose.pose.position.y = self.current_point.y
      self.odom_msg.pose.pose.position.z = self.current_point.z

      if not rospy.is_shutdown():
        self.fake_gps_pub.publish(navsatfix)
        self.fake_odom_pub.publish(self.odom_msg)
      # Send Mavlink GPS Override if needed
      if self.send_mavlink_gps_msg:
        hilgps=HilGPS()
        hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavlink_fake_gps")
        hilgps.fix_type=3
        hilgps.geo.latitude=self.current_location_wgs84_geo.latitude
        hilgps.geo.longitude=self.current_location_wgs84_geo.longitude
        hilgps.geo.altitude=self.current_location_wgs84_geo.altitude
        hilgps.satellites_visible=self.SAT_COUNT
        #nepi_msg.publishMsgInfo(self,"Created new HilGPS message")
        #nepi_msg.publishMsgInfo(self,hilgps)
        # Create and publish Fake GPS Publisher
        hilgps.header = Header(stamp=rospy.Time.now(), frame_id="mavlink_fake_gps")
        if not rospy.is_shutdown():
          self.mavlink_pub.publish(hilgps)





  #######################
  # Node Process Functions
  ### function to simulate move to new global geo position
  def move(self,geopoint_msg):
    nepi_msg.publishMsgInfo(self,"")
    nepi_msg.publishMsgInfo(self,'***********************')
    loc = self.current_location_wgs84_geo
    nepi_msg.publishMsgInfo(self,"Fake GPS Moving FROM: " + str(loc.latitude) + ", " + str(loc.longitude) + ", " + str(loc.altitude)) 
    loc = geopoint_msg
    nepi_msg.publishMsgInfo(self,"T0: " + str(loc.latitude) + ", " + str(loc.longitude) + ", " + str(loc.altitude)) 

    org_geo=np.array([self.current_location_wgs84_geo.latitude, \
                      self.current_location_wgs84_geo.longitude, self.current_location_wgs84_geo.altitude])
    cur_geo = copy.deepcopy(org_geo)
    new_geo=np.array([geopoint_msg.latitude, geopoint_msg.longitude, geopoint_msg.altitude])
    for ind, val in enumerate(new_geo):
      if new_geo[ind] == -999.0: # Use current
        new_geo[ind]=org_geo[ind]
    delta_geo = new_geo - org_geo
    move_dist_m = nepi_nav.distance_geopoints(org_geo,new_geo)

    org_point = np.array([self.current_point.x,self.current_point.y,self.current_point.z])
    cur_point = copy.deepcopy(org_point)
    nepi_msg.publishMsgInfo(self,"cur point movement: " + str(org_point))
    new_point = np.array([org_point[0] + self.new_point.x, org_point[1] + self.new_point.y, org_point[2] + self.new_point.z]) 
    nepi_msg.publishMsgInfo(self,"new point movement: " + str(new_point))
    delta_point = new_point - org_point
    nepi_msg.publishMsgInfo(self,"delta point movement: " + str(delta_point))

    #nepi_msg.publishMsgInfo(self,"TO:")
    #nepi_msg.publishMsgInfo(self,delta_geo)
    if move_dist_m > 0 and self.checkStopTrigger() == False:
      move_time = self.MOVE_UPDATE_TIME_SEC_PER_METER * move_dist_m
      if move_time > MAX_MOVE_TIME_S:
        move_time = MAX_MOVE_TIME_S
      move_steps = move_time * self.GPS_PUB_RATE_HZ
      stp_interval_sec = float(move_time)/float(move_steps)
      nepi_msg.publishMsgInfo(self,"Moving " + "%.2f" % move_dist_m + " meters in " + "%.2f" % move_time + " seconds")
      nepi_msg.publishMsgInfo(self,"with " + "%.2f" % move_steps + " steps")

      ramp=np.hanning(move_steps)
      ramp=ramp**2
      ramp_norm=ramp/np.sum(ramp)
      step_norm=np.zeros(len(ramp_norm))
      for ind, val in enumerate(ramp_norm):
        step_norm[ind]=np.sum(ramp_norm[0:ind])
      
      rospy.loginfo_timer = 0
      for ind, val in enumerate(step_norm):
        time.sleep(stp_interval_sec)
        #rospy.loginfo_timer = rospy.loginfo_timer + stp_interval_sec
        cur_geo_step = delta_geo * val
        cur_geo = org_geo + cur_geo_step
        self.current_location_wgs84_geo.latitude = cur_geo[0]
        self.current_location_wgs84_geo.longitude = cur_geo[1]
        self.current_location_wgs84_geo.altitude = cur_geo[2]


        cur_point_step = [delta_point[0] * val, delta_point[1] * val, delta_point[2] * val]
        cur_point = org_point + cur_point_step

        self.current_point.x = cur_point[0]
        self.current_point.y = cur_point[1]
        self.current_point.z = cur_point[2]

          #nepi_msg.publishMsgInfo(self,"")
          #nepi_msg.publishMsgInfo(self,"Updated to")
          #nepi_msg.publishMsgInfo(self,self.current_location_wgs84_geo)
          #current_error_m = nepi_nav.distance_geopoints(cur_geo,new_geo)
          #nepi_msg.publishMsgInfo(self,"Current move error : " + "%.2f" % (current_error_m) + " meters")
          #rospy.loginfo_timer=0
    current_error_m = nepi_nav.distance_geopoints(cur_geo,new_geo)
    nepi_msg.publishMsgInfo(self,"Move Error: " + "%.2f" % (current_error_m) + " meters")

    nepi_msg.publishMsgInfo(self,"FAKE GPS Move Complete")
    nepi_msg.publishMsgInfo(self,'***********************')


  def checkStopTrigger(self):
    triggered = self.stop_triggered
    self.stop_triggered = False # Reset Stop Trigger
    return triggered

  #######################
  # NEPI NavPose Interfaces

  ### Setup a regular background navpose get and update heading data
  def update_current_heading_callback(self,timer):
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
    if not rospy.is_shutdown():
      try:
        get_navpose_service = rospy.ServiceProxy(self.nepi_nav_service_name, NavPoseQuery)
        nav_pose_response = get_navpose_service(NavPoseQueryRequest())
        self.current_heading_deg = nav_pose_response.nav_pose.heading.heading
        self.current_yaw_enu_deg = nepi_nav.get_navpose_orientation_enu_degs(nav_pose_response)[2]
        #nepi_msg.publishMsgInfo(self,'')
        #nepi_msg.publishMsgInfo(self,"Update current heading to: " + "%.2f" % (self.current_heading_deg))
      except Exception as e:
        pass
        #nepi_msg.publishMsgInfo(self,"navpose service call failed: " + str(e))


  #######################
  # NEPI Fake GPS Interfaces


  #######################
  # NEPI Fake GPS Interfaces

    ### Callback to set fake gps enable
  def fakeGPSEnableCb(self,msg):
    nepi_msg.publishMsgInfo(self,"Received set fake gps enable message: " + str(msg.data))
    self.fake_gps_enabled = msg.data
    self.status_pub.publish(self.fake_gps_enabled)

  def fakeGPSResetLocCb(self,geo_msg):
    geo_str = str([geo_msg.latitude,geo_msg.longitude,geo_msg.altitude])
    nepi_msg.publishMsgInfo(self,"Received Fake GPS Reset to Location Msg: " + geo_str)
    success = self.reset_gps_loc(geo_msg)
    if success:
      self.reset_point = self.ZERO_POINT
      nepi_msg.publishMsgInfo(self,"Reset Complete")
    return success

  ### Function to reset gps and wait for position ned x,y to reset
  def reset_gps_loc(self, geo_msg):
    self.fake_gps_ready = False
    self.stop_triggered = True
    nepi_ros.sleep(5,50)
    self.current_location_wgs84_geo = geo_msg
    #nepi_msg.publishMsgInfo(self,"Waiting for GPS to reset") 
    nepi_ros.sleep(5,50)
    self.stop_triggered = False
    self.fake_gps_ready = True
    return True


  ### Callback to stop
  def fakeGPSGoStopCb(self,empty_msg):
    if self.fake_gps_enabled:
      nepi_msg.publishMsgInfo(self,"Received go stop message")
      self.stop_triggered = True
      time.sleep(1)
      self.stop_triggered = False

  ### Function to monitor RBX GoTo Position Command Topics
  def fakeGPSGoPosCb(self,enu_point_msg):
    if self.fake_gps_enabled:
      self.checkStopTrigger() # Clear stop trigger
      point_str = str(enu_point_msg)
      nepi_msg.publishMsgInfo(self,"Recieved GoTo Position Message: " + point_str)
      geo_str = str (self.current_location_wgs84_geo)
      nepi_msg.publishMsgInfo(self,"At Geo Location Message: " + geo_str)
      self.new_point = enu_point_msg

      new_enu_position = [enu_point_msg.x,enu_point_msg.y,enu_point_msg.z]
      nepi_msg.publishMsgInfo(self,"Sending Fake GPS Setpoint Position Update")
      new_geopoint_wgs84=nepi_nav.get_geopoint_at_enu_point(self.current_location_wgs84_geo, new_enu_position) 
      geo_str = str (self.current_location_wgs84_geo)
      nepi_msg.publishMsgInfo(self,"Current Geo Location Message: " + geo_str)
      geo_str = str (new_geopoint_wgs84)
      nepi_msg.publishMsgInfo(self,"New Geo Location Message: " + geo_str)
      self.move(new_geopoint_wgs84)


  ### Function to monitor RBX GoTo Location Command Topics
  def fakeGPSGoLocCb(self,geo_msg):
    if self.fake_gps_enabled:
      self.checkStopTrigger() # Clear stop trigger
      geo_str = str(geo_msg)
      nepi_msg.publishMsgInfo(self,"Recieved GoTo Location Message: " + geo_str)
      self.move(geo_msg)

  
    #######################
    # Node Cleanup Function
    
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  RBXFakeGPS()
  
