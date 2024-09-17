#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import sys
import time
import math
import rospy
import ros_numpy
import threading
import cv2
import copy


import subprocess
import dynamic_reconfigure.client
import numpy as np
import tf

from nepi_drivers.idx_device_if import ROSIDXSensorIF

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_nav
from nepi_edge_sdk_base import nepi_img
from nepi_edge_sdk_base import nepi_pc
from nepi_edge_sdk_base import nepi_nex

from datetime import datetime
from std_msgs.msg import UInt8, Empty, String, Bool, Float32
from sensor_msgs.msg import Image, PointCloud2
from nepi_ros_interfaces.msg import IDXStatus, RangeWindow, SaveDataStatus, SaveData, SaveDataRate
from nepi_ros_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryResponse
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from dynamic_reconfigure.msg import Config
from rospy.numpy_msg import numpy_msg

PKG_NAME = 'IDX_ZED' # Use in display menus
DESCRIPTION = 'Driver package for StereoLab Zed stereo camera devices'
FILE_TYPE = 'NODE'
CLASS_NAME = 'ZedCamNode' # Should Match Class Name
GROUP ='IDX'
GROUP_ID = 'ZED' 

DRIVER_PKG_NAME = 'None' # 'Required Driver PKG_NAME or 'None'
DRIVER_INTERFACES = ['USB'] # 'USB','IP','SERIALUSB','SERIAL','CANBUS'
DRIVER_OPTIONS_1_NAME = 'None'
DRIVER_OPTIONS_1 = [] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_1 = 'None'
DRIVER_OPTIONS_2_NAME = 'None'
DRIVER_OPTIONS_2 = [] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_2 = 'None'

DISCOVERY_PKG_NAME = 'IDX_ZED' # 'Required Discovery PKG_NAME or 'None'
DISCOVERY_METHOD = 'AUTO'  # 'AUTO', 'MANUAL', or 'OTHER' if managed by seperate application
DISCOVERY_IDS = ['ZED 2','ZED 2i','ZED-M']  # List of string identifiers for discovery process
DISCOVERY_IGNORE_IDS = [] # List of string identifiers for discovery process

TEST_NEX_DICT = {
    'group': 'IDX',
    'group_id': 'ZED',
    'node_file_name': 'idx_zed_node.py',
    'node_file_path': '/opt/nepi/ros/lib/nep_drivers',
    'node_module_name': 'idx_zed_node',
    'node_class_name': 'ZedCamNode',
    'driver_name': "None",
    'driver_file_name': "None" ,
    'driver_file_path':  "None",
    'driver_module_name': "None" ,
    'driver_class_name':  "None",
    'driver_interfaces': [],
    'driver_options_1': [],
    'driver_default_option_1': 'None',
    'driver_set_option_1': 'None',
    'driver_options_2': [],
    'driver_default_option_2': 'None',
    'driver_set_option_2': 'None',
    'discovery_name': 'IDX_ZED', 
    'discovery_file_name': 'idx_zed_discovery.py',
    'discovery_file_path': '/opt/nepi/ros/lib/nepi_drivers',
    'discovery_module_name': 'idx_zed_discovery',
    'discovery_class_name': 'ZedCamDiscovery',
    'discovery_method': 'AUTO', 
    'discovery_ids': ['ZED 2','ZED 2i','ZED-M'],
    'discovery_ignore_ids': ['msm_vidc_vdec'],
    'device_dict': {'zed_type': 'zed2'},
    'order': 1,
    'active': True,
    'msg': ""
    }



class ZedCamNode(object):

    CAL_SRC_PATH = "/usr/local/zed/settings"
    USER_CFG_PATH = "/mnt/nepi_storage/user_cfg"
    CAL_BACKUP_PATH = USER_CFG_PATH + "/zed_cals"

    CAP_SETTINGS = [["Float","pub_frame_rate","0.1","100.0"],
                    ["Int","depth_confidence","0","100"],
                    ["depth_texture_conf","0","100"],
                    ["Int","point_cloud_freq","0.1","100"],
                    ["Int","brightness","0","8"],
                    ["Int","contrast","0","8"],
                    ["Int","hue","0","11"],
                    ["Int","saturation","0","8"],
                    ["Int","sharpness","0","8"],
                    ["Int","gamma","1","9"],
                    ["Bool","auto_exposure_gain"],
                    ["Int","gain","0","100"],
                    ["Int","exposure","0","100"],
                    ["Bool","auto_whitebalance"],
                    ["Int","whitebalance_temperature","1","100"]]

    FACTORY_SETTINGS_OVERRIDES = dict( )
    
    
    #Factory Control Values 
    FACTORY_CONTROLS = dict( controls_enable = True,
    auto_adjust = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    resolution_mode = 3, # LOW, MED, HIGH, MAX
    framerate_mode = 3, # LOW, MED, HIGH, MAX
    start_range_ratio = 0.0, 
    stop_range_ratio = 1.0,
    min_range_m = 0.0,
    max_range_m = 20.0,
    frame_id = 'sensor_frame' 
    )

    DEFAULT_CURRENT_FPS = 20 # Will be update later with actual

    ZED_MIN_RANGE_M_OVERRIDES = { 'zed': .2, 'zedm': .15, 'zed2': .2, 'zedx': .2} 
    ZED_MAX_RANGE_M_OVERRIDES = { 'zed':  15, 'zedm': 15, 'zed2': 20, 'zedx': 15} 

    zed_type = 'zed'

    # Create shared class variables and thread locks 
    
    device_info_dict = dict(node_name = "",
                            sensor_name = "",
                            identifier = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")
    color_img_acquire = False
    color_img_msg = None
    color_img_last_stamp = None
    color_img_lock = threading.Lock()
    bw_img_acquire = False
    bw_img_msg = None
    bw_img_last_stamp = None
    bw_img_lock = threading.Lock()
    depth_map_acquire = False
    depth_map_msg = None
    depth_map_last_stamp = None
    depth_map_lock = threading.Lock() 
    depth_img_acquire = False   
    depth_img_msg = None
    depth_img_last_stamp = None
    depth_img_lock = threading.Lock() 
    pc_acquire = False   
    pc_msg = None
    pc_last_stamp = None
    pc_lock = threading.Lock()
    pc_img_acquire = False
    pc_img_msg = None
    pc_img_last_stamp = None
    pc_img_lock = threading.Lock()

    gps_msg = None
    odom_msg = None
    heading_msg = None

    # Rendering Initialization Values
    render_img_width = 1280
    render_img_height = 720
    render_background = [0, 0, 0, 0] # background color rgba
    render_fov = 60 # camera field of view in degrees
    render_center = [3, 0, 0]  # look_at target
    render_eye = [-5, -2.5, 0]  # camera position
    render_up = [0, 0, 1]  # camera orientation


    img_renderer = None
    img_renderer_mtl = None
    

    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"                                   
    def __init__(self):
        # Launch the ROS node
        rospy.loginfo("Starting " + self.DEFAULT_NODE_NAME)
        rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
        self.node_name = rospy.get_name().split('/')[-1]
        # Get nex_dict from param servers
        self.nex_dict = rospy.get_param('~nex_dict',TEST_NEX_DICT) 
        #rospy.logwarn("ZED_NODE: " + str(self.nex_dict))

        ################################################
        # Try to restore camera calibration files from
        [success,files_copied,files_not_copied] = nepi_ros.copy_files_from_folder(self.CAL_BACKUP_PATH,self.CAL_SRC_PATH)
        if success:
          if len(files_copied) > 0:
            strList = str(files_copied)
            rospy.loginfo(self.node_name + ": " +"Restored zed cal files: " + strList)
        else:
          rospy.loginfo(self.node_name + ": " +"Failed to restore zed cal files")
        # This parameter should be automatically set by idx_sensor_mgr
        self.zed_type = self.nex_dict['device_dict']['zed_type']
        #rospy.logwarn(self.node_name + ": " + self.zed_type)

        # Run the correct zed_ros_wrapper launch file
        zed_launchfile = self.zed_type + '.launch'
        zed_ros_wrapper_run_cmd = ['roslaunch', 'zed_wrapper', zed_launchfile]
        # TODO: Some process management for the Zed ROS wrapper
        self.zed_ros_wrapper_proc = subprocess.Popen(zed_ros_wrapper_run_cmd)

        # Connect to Zed node
        self.zed_type = self.nex_dict['device_dict']['zed_type']
        ZED_BASE_NAMESPACE = nepi_ros.get_base_namespace() + self.zed_type + "/zed_node/"

        # Now that Zed SDK is started, we can set up the reconfig client
        nepi_ros.sleep(2,10)
        success = False
        while success == False and not rospy.is_shutdown():
          try:
            self.zed_dynamic_reconfig_client = dynamic_reconfigure.client.Client(ZED_BASE_NAMESPACE, timeout=30)
            success = True
          except Exception as e:
            rospy.loginfo(self.node_name + ": " +str(e))
            nepi_ros.sleep(1,10)


        # Zed control topics
        # ZED_PARAMETER_UPDATES_TOPIC = ZED_BASE_NAMESPACE + "parameter_updates"
        # Zed data stream topics
        self.color_img_topic = ZED_BASE_NAMESPACE + "left/image_rect_color"
        self.bw_img_topic = ZED_BASE_NAMESPACE + "left/image_rect_gray"
        self.depth_map_topic = ZED_BASE_NAMESPACE + "depth/depth_registered"
        self.pc_topic = ZED_BASE_NAMESPACE + "point_cloud/cloud_registered"
        ZED_ODOM_TOPIC = ZED_BASE_NAMESPACE + "odom"
        ZED_MIN_RANGE_PARAM = ZED_BASE_NAMESPACE + "depth/min_depth"
        ZED_MAX_RANGE_PARAM = ZED_BASE_NAMESPACE + "depth/max_depth"




        # Wait for zed camera topic to publish, then subscribeCAPS SETTINGS
        rospy.loginfo(self.node_name + ": " +"Waiting for topic: " + self.color_img_topic)
        nepi_ros.wait_for_topic(self.color_img_topic)

        rospy.loginfo(self.node_name + ": " +"Starting Zed IDX subscribers and publishers")
        self.color_img_sub = None
        self.bw_img_sub = None
        self.depth_map_sub = None
        self.depth_img_sub = None
        self.pc_sub = None
        self.pc_img_sub = None
        rospy.Subscriber(ZED_ODOM_TOPIC, Odometry, self.idx_odom_topic_callback)

        # Launch the ROS node
        rospy.loginfo(self.node_name + ": ... Connected!")


        idx_callback_names = {
            "Controls" : {
                # IDX Standard
                "Controls_Enable":  self.setControlsEnable,
                "Auto_Adjust":  self.setAutoAdjust,
                "Brightness": self.setBrightness,
                "Contrast":  self.setContrast,
                "Thresholding": self.setThresholding,
                "Resolution": self.setResolutionMode,
                "Framerate":  self.setFramerateMode,
                "Range": self.setRange
            },
            

            "Data" : {
                # Data callbacks
                "Color2DImg": self.getColorImg,
                "StopColor2DImg": self.stopColorImg,
                "BW2DImg": self.getBWImg,
                "StopBW2DImg": self.stopBWImg,
                "DepthMap": self.getDepthMap, 
                "StopDepthMap":  self.stopDepthMap,
                "DepthImg": self.getDepthImg,
                "StopDepthImg": self.stopDepthImg,
                "Pointcloud":  self.getPointcloud, 
                "StopPointcloud":  self.stopPointcloud,
                "PointcloudImg":  self.getPointcloudImg, 
                "StopPointcloudImg":  self.stopPointcloudImg,
                "GPS": None,
                "Odom": self.getOdom,
                "Heading": None
            }
        }

        # Initialize controls
        self.factory_controls = self.FACTORY_CONTROLS
        # Apply OVERRIDES
        if self.zed_type in self.ZED_MIN_RANGE_M_OVERRIDES:
          self.factory_controls['min_range_m'] = self.ZED_MIN_RANGE_M_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_MAX_RANGE_M_OVERRIDES:
          self.factory_controls['max_range_m'] = self.ZED_MAX_RANGE_M_OVERRIDES[self.zed_type]
        
        self.current_controls = self.factory_controls # Updateded during initialization
        self.current_fps = self.DEFAULT_CURRENT_FPS # Should be updateded when settings read

        # Initialize settings
        self.cap_settings = self.getCapSettings()
        rospy.loginfo(self.node_name + ": " +"CAPS SETTINGS")
        #for setting in self.cap_settings:
            #rospy.loginfo(self.node_name + ": " +setting)
        self.factory_settings = self.getFactorySettings()
        rospy.loginfo(self.node_name + ": " +"FACTORY SETTINGS")
        #for setting in self.factory_settings:
            #rospy.loginfo(self.node_name + ": " +setting)
             

        # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
        rospy.loginfo(self.node_name + ": Launching NEPI IDX (ROS) interface...")
        self.device_info_dict["node_name"] = self.node_name
        if self.node_name.find("_") != -1:
            split_name = self.node_name.rsplit('_', 1)
            self.device_info_dict["sensor_name"] = split_name[0]
            self.device_info_dict["identifier"] = split_name[1]
        else:
            self.device_info_dict["sensor_name"] = self.node_name
        self.idx_if = ROSIDXSensorIF(device_info = self.device_info_dict,
                                     capSettings = self.cap_settings,
                                     factorySettings = self.factory_settings,
                                     settingUpdateFunction=self.settingUpdateFunction,
                                     getSettingsFunction=self.getSettings,
                                     factoryControls = self.FACTORY_CONTROLS,
                                     setControlsEnable = idx_callback_names["Controls"]["Controls_Enable"],
                                     setAutoAdjust= idx_callback_names["Controls"]["Auto_Adjust"],
                                     setResolutionMode=idx_callback_names["Controls"]["Resolution"], 
                                     setFramerateMode=idx_callback_names["Controls"]["Framerate"], 
                                     setContrast=idx_callback_names["Controls"]["Contrast"], 
                                     setBrightness=idx_callback_names["Controls"]["Brightness"], 
                                     setThresholding=idx_callback_names["Controls"]["Thresholding"], 
                                     setRange=idx_callback_names["Controls"]["Range"], 
                                     getColor2DImg=idx_callback_names["Data"]["Color2DImg"], 
                                     stopColor2DImgAcquisition=idx_callback_names["Data"]["StopColor2DImg"],
                                     getBW2DImg=idx_callback_names["Data"]["BW2DImg"], 
                                     stopBW2DImgAcquisition=idx_callback_names["Data"]["StopBW2DImg"],
                                     getDepthMap=idx_callback_names["Data"]["DepthMap"], 
                                     stopDepthMapAcquisition=idx_callback_names["Data"]["StopDepthMap"],
                                     getDepthImg=idx_callback_names["Data"]["DepthImg"], 
                                     stopDepthImgAcquisition=idx_callback_names["Data"]["StopDepthImg"],
                                     getPointcloud=idx_callback_names["Data"]["Pointcloud"], 
                                     stopPointcloudAcquisition=idx_callback_names["Data"]["StopPointcloud"],
                                     getPointcloudImg=idx_callback_names["Data"]["PointcloudImg"], 
                                     stopPointcloudImgAcquisition=idx_callback_names["Data"]["StopPointcloudImg"],
                                     getGPSMsg=idx_callback_names["Data"]["GPS"],
                                     getOdomMsg=idx_callback_names["Data"]["Odom"],
                                     getHeadingMsg=idx_callback_names["Data"]["Heading"])
        rospy.loginfo(self.node_name + ": " + " ... IDX interface running")

        # Update available IDX callbacks based on capabilities that the driver reports
        self.logDeviceInfo()

        # Configure pointcloud processing Verbosity
        #pc_verbosity = nepi_pc.set_verbosity_level("Error")
        #rospy.loginfo(self.node_name + ": " +pc_verbosity)

        # Now that all camera start-up stuff is processed, we can update the camera from the parameters that have been established
        self.idx_if.updateFromParamServer()

        # Try to backup camera calibration files
        [success,files_copied,files_not_copied] = nepi_ros.copy_files_from_folder(self.CAL_SRC_PATH,self.CAL_BACKUP_PATH)
        if success:
           if len(files_copied) > 0:
            strList = str(files_copied)
            rospy.loginfo(self.node_name + ": " +"Backed up zed cal files: " + strList)
        else:
          rospy.loginfo(self.node_name + ": " +"Failed to back up up zed cal files")

        ## Initiation Complete
        rospy.loginfo(self.node_name + ": " + "Initialization Complete")
        # Now start the node
        rospy.spin()

    #**********************
    # Sensor setting functions
    def getCapSettings(self):
      return self.CAP_SETTINGS

    def getFactorySettings(self):
      settings = self.getSettings()
      #Apply factory setting overides
      for setting in settings:
        if setting[1] in self.FACTORY_SETTINGS_OVERRIDES:
              setting[2] = self.FACTORY_SETTINGS_OVERRIDES[setting[1]]
              settings = nepi_ros.update_setting_in_settings(setting,settings)
      return settings


    def getSettings(self):
      settings = []
      config_dict = self.zed_dynamic_reconfig_client.get_configuration()
      if config_dict is not None:
        for cap_setting in self.cap_settings:
          setting_type = cap_setting[0]
          setting_name = cap_setting[1]
          if setting_name in config_dict.keys():
            setting_value = config_dict[setting_name]
            setting = [setting_type,setting_name,str(setting_value)]
            settings.append(setting)
      return settings

    def settingUpdateFunction(self,setting):
      success = False
      setting_str = str(setting)
      if len(setting) == 3:
        setting_type = setting[0]
        setting_name = setting[1]
        [s_name, s_type, data] = nepi_ros.get_data_from_setting(setting)
        #rospy.loginfo(self.node_name + ": " +type(data))
        if data is not None:
          setting_data = data
          config_dict = self.zed_dynamic_reconfig_client.get_configuration()
          if setting_name in config_dict.keys():
            self.zed_dynamic_reconfig_client.update_configuration({setting_name:setting_data})
            success = True
            msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)
          else:
            msg = (self.node_name  + " Setting name" + setting_str + " is not supported")                   
        else:
          msg = (self.node_name  + " Setting data" + setting_str + " is None")
      else:
        msg = (self.node_name  + " Setting " + setting_str + " not correct length")
      return success, msg


    #**********************
    # Zed camera data callbacks

    # callback to get color 2d image data
    def color_2d_image_callback(self, image_msg):
      self.color_img_lock.acquire()
      self.color_img_msg = image_msg
      self.color_img_lock.release()

    # callback to get 2d image data
    def bw_2d_image_callback(self, image_msg):
      self.bw_img_lock.acquire()
      self.bw_img_msg = image_msg
      self.bw_img_lock.release()


    # callback to get depthmap
    def depth_map_callback(self, image_msg):
      image_msg.header.stamp = rospy.Time.now()
      self.depth_map_lock.acquire()
      self.depth_map_msg = image_msg
      self.depth_map_lock.release()

    # callback to get depthmap
    def depth_image_callback(self, image_msg):
      image_msg.header.stamp = rospy.Time.now()
      self.depth_img_lock.acquire()
      self.depth_img_msg = image_msg
      self.depth_img_lock.release()

    # callback to get and republish point_cloud
    def pointcloud_callback(self, pointcloud_msg):
      self.pc_lock.acquire()
      self.pc_msg = pointcloud_msg
      self.pc_lock.release()

    # callback to get and process point_cloud image
    def pointcloud_image_callback(self, pointcloud_msg):
      self.pc_img_lock.acquire()
      self.pc_img_msg = pointcloud_msg
      self.pc_img_lock.release()



    # Callback to get odom data
    def idx_odom_topic_callback(self, odom_msg):
      self.odom_msg = odom_msg


    #**********************
    # IDX driver functions

    def logDeviceInfo(self):
        device_info_str = self.node_name + " info:\n"
        rospy.loginfo(self.node_name + ": " + device_info_str)
        rospy.loginfo(self.node_name + ": " + str(self.device_info_dict))

    def setControlsEnable(self, enable):
        self.current_controls["controls_enable"] = enable
        status = True
        err_str = ""
        return status, err_str
        
    def setAutoAdjust(self, enable):
        ret = self.current_controls["auto_adjust"] = enable
        status = True
        err_str = ""
        return status, err_str

    def setBrightness(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < 0:
            ratio = 0
        self.current_controls["brightness_ratio"] = ratio
        status = True
        err_str = ""
        return status, err_str

    def setContrast(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < 0:
            ratio = 0
        self.current_controls["contrast_ratio"] = ratio
        status = True
        err_str = ""
        return status, err_str

    def setThresholding(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < 0:
            ratio = 0
        self.current_controls["threshold_ratio"] = ratio
        status = True
        err_str = ""
        return status, err_str

    def setResolutionMode(self, mode):
        if (mode > self.idx_if.RESOLUTION_MODE_MAX):
            return False, "Invalid mode value"
        self.current_controls["resolution_mode"] = mode
        status = True
        err_str = ""
        return status, err_str
    
    def setFramerateMode(self, mode):
        if (mode > self.idx_if.FRAMERATE_MODE_MAX):
            return False, "Invalid mode value"
        self.current_controls["framerate_mode"] = mode
        status = True
        err_str = ""
        return status, err_str

    def setRange(self, min_ratio, max_ratio):
        if min_ratio > 1:
            min_ratio = 1
        elif min_ratio < 0:
            min_ratio = 0
        self.current_controls["start_range_ratio"] = min_ratio
        if max_ratio > 1:
            max_ratio = 1
        elif max_ratio < 0:
            max_ratio = 0
        if min_ratio < max_ratio:
          self.current_controls["stop_range_ratio"] = max_ratio
          status = True
          err_str = ""
        else:
          status = False
          err_str = "Invalid Range Window"
        return status, err_str

    
 

    # Good base class candidate - Shared with ONVIF
    def getColorImg(self):
        if self.color_img_sub == None:
          self.color_img_sub = rospy.Subscriber(self.color_img_topic, Image, self.color_2d_image_callback, queue_size = 1)
          time.sleep(0.1)
        # Set process input variables
        data_product = "color_2d_image"
        self.color_img_lock.acquire()
        img_msg = None
        if self.color_img_msg != None:
          if self.color_img_msg.header.stamp != self.color_img_last_stamp:
            img_msg = copy.deepcopy(self.color_img_msg)          
        self.color_img_lock.release()
        encoding = 'bgr8'

        # Run get process
        # Initialize some process return variables
        status = False
        msg = ""
        cv2_img = None
        ros_timestamp = None
        if img_msg is not None:
          if img_msg.header.stamp != self.color_img_last_stamp:
            ros_timestamp = img_msg.header.stamp
            status = True
            msg = ""
            ros_timestamp = img_msg.header.stamp
            if self.current_controls.get("controls_enable"):
              cv2_img =  nepi_img.rosimg_to_cv2img(img_msg, encoding = encoding)
              cv2_img = nepi_nex.applyIDXControls2Image(cv2_img,self.current_controls,self.current_fps)
              #img_msg = nepi_img.cv2img_to_rosimg(cv2_img, encoding = encoding)
            self.color_img_last_stamp = ros_timestamp
          else:
            msg = "No new data for " + data_product + " available"
        else:
          msg = "Received None type data for " + data_product + " process"
        if cv2_img is not None:
          return status, msg, cv2_img, ros_timestamp, encoding
        else: 
          return status, msg, img_msg, ros_timestamp, encoding
    
    # Good base class candidate - Shared with ONVIF
    def stopColorImg(self):
        self.color_img_lock.acquire()
        self.color_img_sub.unregister()
        self.color_img_sub = None
        self.color_img_msg = None
        self.color_img_lock.release()
        ret = True
        msg = "Success"
        return ret,msg
    
    # Good base class candidate - Shared with ONVIF
    def getBWImg(self):
        if self.bw_img_sub == None:
          self.bw_img_sub =rospy.Subscriber(self.bw_img_topic, Image, self.bw_2d_image_callback, queue_size = 1)
          time.sleep(0.1)
        # Set process input variables
        data_product = "bw_2d_image"
        self.bw_img_lock.acquire()
        img_msg = None
        if self.bw_img_msg != None:
          if self.bw_img_msg.header.stamp != self.bw_img_last_stamp:
            img_msg = copy.deepcopy(self.bw_img_msg)
        self.bw_img_lock.release()
        encoding = "mono8"

        # Run get process
        # Initialize some process return variables
        status = False
        msg = ""
        cv2_img = None
        ros_timestamp = None
        if img_msg is not None:
          if img_msg.header.stamp != self.bw_img_last_stamp:
            ros_timestamp = img_msg.header.stamp
            status = True
            msg = ""
            ros_timestamp = img_msg.header.stamp
            if self.current_controls.get("controls_enable"):
              cv2_img =  nepi_img.rosimg_to_cv2img(img_msg, encoding = encoding)
              cv2_img = nepi_nex.applyIDXControls2Image(cv2_img,self.current_controls,self.current_fps)
              #img_msg = nepi_img.cv2img_to_rosimg(cv2_img, encoding = encoding)
            self.bw_img_last_stamp = ros_timestamp
          else:
            msg = "No new data for " + data_product + " available"
        else:
          msg = "Received None type data for " + data_product + " process"
        if cv2_img is not None:
          return status, msg, cv2_img, ros_timestamp, encoding
        else: 
          return status, msg, img_msg, ros_timestamp, encoding
    
    # Good base class candidate - Shared with ONVIF
    def stopBWImg(self):
      self.bw_img_lock.acquire()

      self.bw_img_sub.unregister()
      self.bw_img_sub = None

      self.bw_img_msg = None
      self.bw_img_lock.release()
      ret = True
      msg = "Success"
      return ret,msg

    def getDepthMap(self):
        if self.depth_map_sub == None:
          self.depth_map_sub =rospy.Subscriber(self.depth_map_topic, Image, self.depth_map_callback, queue_size = 1)
          time.sleep(0.1)
        # Set process input variables
        data_product = "depth_map"
        self.depth_map_lock.acquire()
        img_msg = None
        if self.depth_map_msg != None:
          if self.depth_map_msg.header.stamp != self.depth_map_last_stamp:
            img_msg = copy.deepcopy(self.depth_map_msg)
        self.depth_map_lock.release()
        encoding = '32FC1'
        # Run get process
        # Initialize some process return variables
        status = False
        msg = ""
        cv2_img = None
        ros_timestamp = None
        if img_msg is not None:
          if img_msg.header.stamp != self.depth_map_last_stamp:
            ros_timestamp = img_msg.header.stamp
            self.depth_map_last_stamp = ros_timestamp
            status = True
            msg = ""
            # Adjust range Limits if IDX Controls enabled and range ratios are not min/max
            start_range_ratio = self.current_controls.get("start_range_ratio")
            stop_range_ratio = self.current_controls.get("stop_range_ratio")
            if self.current_controls.get("controls_enable") and (start_range_ratio > 0 or stop_range_ratio < 1):
              # Convert ros depth_map to cv2_img and numpy depth data
              cv2_depth_map = nepi_img.rosimg_to_cv2img(img_msg, encoding="passthrough")
              depth_data = (np.array(cv2_depth_map, dtype=np.float32)) # replace nan values
              # Get range data
              min_range_m = self.current_controls.get("min_range_m")
              max_range_m = self.current_controls.get("max_range_m")
              #Update range limits and crop depth map
              delta_range_m = max_range_m - min_range_m
              max_range_m = min_range_m + stop_range_ratio * delta_range_m
              min_range_m = min_range_m + start_range_ratio * delta_range_m
              delta_range_m = max_range_m - min_range_m
              # Filter depth_data in range
              depth_data[np.isnan(depth_data)] = float('nan')  # set to NaN
              depth_data[depth_data <= min_range_m] = float('nan')  # set to NaN
              depth_data[depth_data >= max_range_m] = float('nan')  # set to NaN
              cv2_img = depth_data
              #img_msg = nepi_img.cv2img_to_rosimg(cv2_depth_image,encoding)
           
          else:
            msg = "No new data for " + data_product + " available"
        else:
          msg = "Received None type data for " + data_product + " process"
        if cv2_img is not None:
          return status, msg, cv2_img, ros_timestamp, encoding
        else: 
          return status, msg, img_msg, ros_timestamp, encoding
    
    def stopDepthMap(self):
        self.depth_map_lock.acquire()

        self.depth_map_sub.unregister()
        self.depth_map_sub = None

        self.depth_map_msg = None
        self.depth_map_lock.release()
        ret = True
        msg = "Success"
        return ret,msg

    def getDepthImg(self):
        if self.depth_img_sub == None:
          self.depth_img_sub =rospy.Subscriber(self.depth_map_topic, Image, self.depth_image_callback, queue_size = 1)
          time.sleep(0.1)
        # Set process input variables
        data_product = "depth_image"
        self.depth_img_lock.acquire()
        img_msg = None
        if self.depth_img_msg != None:
          if self.depth_img_msg.header.stamp != self.depth_img_last_stamp:
            img_msg = copy.deepcopy(self.depth_img_msg)
        self.depth_img_lock.release()
        encoding = 'bgr8'
         # Run get process
        # Initialize some process return variables
        status = False
        msg = ""
        cv2_img = None
        ros_timestamp = None
        if img_msg is not None:
          if img_msg.header.stamp != self.depth_img_last_stamp:
            ros_timestamp = img_msg.header.stamp
            self.depth_img_last_stamp = ros_timestamp
            status = True
            msg = ""
            # Convert ros depth_map to cv2_img and numpy depth data
            cv2_depth_map = nepi_img.rosimg_to_cv2img(img_msg, encoding="passthrough")
            depth_data = (np.array(cv2_depth_map, dtype=np.float32)) # replace nan values
            # Get range data
            start_range_ratio = self.current_controls.get("start_range_ratio")
            stop_range_ratio = self.current_controls.get("stop_range_ratio")
            min_range_m = self.current_controls.get("min_range_m")
            max_range_m = self.current_controls.get("max_range_m")
            delta_range_m = max_range_m - min_range_m
            # Adjust range Limits if IDX Controls enabled and range ratios are not min/max
            if self.current_controls.get("controls_enable") and (start_range_ratio > 0 or stop_range_ratio < 1):
              max_range_m = min_range_m + stop_range_ratio * delta_range_m
              min_range_m = min_range_m + start_range_ratio * delta_range_m
              delta_range_m = max_range_m - min_range_m
            # Filter depth_data in range
            depth_data[np.isnan(depth_data)] = max_range_m 
            depth_data[depth_data <= min_range_m] = max_range_m # set to max
            depth_data[depth_data >= max_range_m] = max_range_m # set to max
            # Create colored cv2 depth image
            depth_data = depth_data - min_range_m # Shift down 
            depth_data = np.abs(depth_data - max_range_m) # Reverse for colormap
            depth_data = np.array(255*depth_data/delta_range_m,np.uint8) # Scale for bgr colormap
            cv2_img = cv2.applyColorMap(depth_data, cv2.COLORMAP_JET)
            #ros_img = nepi_img.cv2img_to_rosimg(cv2_depth_image,encoding)
          else:
            msg = "No new data for " + data_product + " available"
        else:
          msg = "Received None type data for " + data_product + " process"
        return status, msg, cv2_img, ros_timestamp, encoding

    
    def stopDepthImg(self):
        self.depth_img_lock.acquire()

        self.depth_img_sub.unregister()
        self.depth_img_sub = None

        self.depth_img_msg = None
        self.depth_img_lock.release()
        ret = True
        msg = "Success"
        return ret,msg



    def getPointcloud(self):     
        if self.pc_sub == None:
          self.pc_sub =rospy.Subscriber(self.pc_topic, PointCloud2, self.pointcloud_callback, queue_size = 1)
          time.sleep(0.1)
        # Set process input variables
        data_product = "pointcloud"
        self.pc_lock.acquire()
        pc_msg = None
        if self.pc_msg != None:
          if self.pc_msg.header.stamp != self.pc_last_stamp:
           pc_msg = copy.deepcopy(self.pc_msg)
        self.pc_lock.release()
        # Run get process
        # Initialize some process return variables
        status = False
        msg = ""
        o3d_pc = None
        ros_timestamp = None
        ros_frame = None
        if pc_msg is not None:
          if pc_msg.header.stamp != self.pc_last_stamp:
            ros_timestamp = pc_msg.header.stamp
            ros_frame = pc_msg.header.frame_id
            status = True
            msg = ""
            self.pc_last_stamp = ros_timestamp
            if self.current_controls.get("controls_enable"):
              start_range_ratio = self.current_controls.get("start_range_ratio")
              stop_range_ratio = self.current_controls.get("stop_range_ratio")
              min_range_m = self.current_controls.get("min_range_m")
              max_range_m = self.current_controls.get("max_range_m")
              delta_range_m = max_range_m - min_range_m
              range_clip_min_range_m = min_range_m + start_range_ratio  * delta_range_m
              range_clip_max_range_m = min_range_m + stop_range_ratio  * delta_range_m
              if start_range_ratio > 0 or stop_range_ratio < 1:
                o3d_pc = nepi_pc.rospc_to_o3dpc(pc_msg, remove_nans=False)
                o3d_pc = nepi_pc.range_clip_spherical( o3d_pc, range_clip_min_range_m, range_clip_max_range_m)
          else:
            msg = "No new data for " + data_product + " available"
        else:
          msg = "Received None type data for " + data_product + " process"
        if o3d_pc is not None:
          return status, msg, o3d_pc, ros_timestamp, ros_frame
        else: 
          return status, msg, pc_msg, ros_timestamp, ros_frame

    
    def stopPointcloud(self):
      self.pc_lock.acquire()

      self.pc_sub.unregister()
      self.pc_sub = None

      self.pc_msg = None
      self.pc_lock.release()
      ret = True
      msg = "Success"
      return ret,msg

    def getPointcloudImg(self,render_controls=[0.5,0.5,0.5]):     
        if self.pc_img_sub == None:
          self.pc_img_sub =rospy.Subscriber(self.pc_topic, PointCloud2, self.pointcloud_image_callback, queue_size = 1) 
          time.sleep(0.1)
        # Set process input variables
        data_product = "pointcloud_image"
        self.pc_img_lock.acquire()
        pc_msg = None
        encoding = 'passthrough'
        if self.pc_img_msg != None:
          if self.pc_img_msg.header.stamp != self.pc_img_last_stamp:
            pc_msg = copy.deepcopy(self.pc_img_msg)
        self.pc_img_lock.release()
        # Run get process
        # Initialize some process return variables
        status = False
        msg = ""
        ros_img = None
        ros_timestamp = None
        ros_frame = None
        if pc_msg is not None:
          if pc_msg.header.stamp != self.pc_img_last_stamp:
            ros_timestamp = pc_msg.header.stamp
            ros_frame = pc_msg.header.frame_id
            status = True
            msg = ""
            self.pc_img_last_stamp = ros_timestamp
            o3d_pc = nepi_pc.rospc_to_o3dpc(pc_msg, remove_nans=True)

            img_width = self.render_img_width
            img_height = self.render_img_height
            render_eye = self.render_eye
            render_center = self.render_center
            render_up = self.render_up

            if self.current_controls.get("controls_enable"):
              start_range_ratio = self.current_controls.get("start_range_ratio")
              stop_range_ratio = self.current_controls.get("stop_range_ratio")
              min_range_m = self.current_controls.get("min_range_m")
              max_range_m = self.current_controls.get("max_range_m")
              delta_range_m = max_range_m - min_range_m
              range_clip_min_range_m = min_range_m + start_range_ratio  * delta_range_m
              range_clip_max_range_m = min_range_m + stop_range_ratio  * delta_range_m
              if start_range_ratio > 0 or stop_range_ratio < 1:
                o3d_pc = nepi_pc.rospc_to_o3dpc(pc_msg, remove_nans=False)
                o3d_pc = nepi_pc.range_clip_spherical( o3d_pc, range_clip_min_range_m, range_clip_max_range_m)
       
              zoom_ratio = render_controls[0]
              zoom_scaler = 1 - zoom_ratio
              render_eye = [number*zoom_scaler for number in self.render_eye] # Apply IDX zoom control
              
              rotate_ratio = render_controls[1]
              rotate_angle = (0.5 - rotate_ratio) * 2 * 180
              rotate_vector = [0, 0, rotate_angle]
              o3d_pc = nepi_pc.rotate_pc(o3d_pc, rotate_vector)
             
              tilt_ratio = render_controls[2]
              tilt_angle = (0.5 - tilt_ratio) * 2 * 180
              tilt_vector = [0, tilt_angle, 0]
              o3d_pc = nepi_pc.rotate_pc(o3d_pc, tilt_vector)
            
            if self.img_renderer is not None and self.img_renderer_mtl is not None:
              self.img_renderer = nepi_pc.remove_img_renderer_geometry(self.img_renderer)
              self.img_renderer = nepi_pc.add_img_renderer_geometry(o3d_pc,self.img_renderer, self.img_renderer_mtl)
              o3d_img = nepi_pc.render_img(self.img_renderer,render_center,render_eye,render_up)
              ros_img = nepi_pc.o3dimg_to_rosimg(o3d_img, stamp=ros_timestamp, frame_id=ros_frame)
            else:
              # Create point cloud renderer
              self.img_renderer = nepi_pc.create_img_renderer(img_width=img_width,img_height=img_height, fov=self.render_fov, background = self.render_background)
              self.img_renderer_mtl = nepi_pc.create_img_renderer_mtl()

          else:
            msg = "No new data for " + data_product + " available"
        else:
          msg = "Received None type data for " + data_product + " process"
        return status, msg, ros_img, ros_timestamp,  encoding

    
    def stopPointcloudImg(self):
        self.pc_img_lock.acquire()

        self.pc_img_sub.unregister()
        self.pc_img_sub = None

        self.pc_img_msg = None
        self.pc_img_lock.release()
        ret = True
        msg = "Success"
        return ret,msg

    def getOdom(self):
        return self.odom_msg
        
if __name__ == '__main__':
    node = ZedCamNode()