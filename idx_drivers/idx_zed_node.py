#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import sys
import time
import math
import ros_numpy
import threading
import cv2
import copy
import yaml

import rospy


import subprocess
import dynamic_reconfigure.client
import numpy as np
import tf

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_settings

from datetime import datetime
from std_msgs.msg import UInt8, Empty, String, Bool, Float32
from sensor_msgs.msg import Image, PointCloud2
from nepi_interfaces.msg import RangeWindow, SaveDataStatus, SaveDataRate
from nepi_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryResponse
from nav_msgs.msg import Odometry

from dynamic_reconfigure.msg import Config
from rospy.numpy_msg import numpy_msg

from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'IDX_ZED' # Use in display menus
FILE_TYPE = 'NODE'

TEST_DRV_DICT = {
'type': 'IDX',
'group_id': 'None',
'usr_cfg_path': '/mnt/nepi_storage/user_cfg',
'NODE_DICT': {
    'file_name': 'idx_zed_node.py',
    'class_name': 'ZedCamNode',
},
'DRIVER_DICT': {
    'file_name': 'idx_zed_driver.py' ,
    'class_name':  'ZedCamDriver'
},
'DEVICE_DICT': {'zed_type': 'zed2','res_val': 3},
}



class ZedCamNode(object):
    CHECK_INTERVAL_S = 3.0
    CAL_SRC_PATH = "/usr/local/zed/settings"
    USER_CFG_PATH = "/mnt/nepi_storage/user_cfg"
    CAL_BACKUP_PATH = USER_CFG_PATH + "/zed_cals"
    ZED_PARAMS_PATH = '/opt/nepi/nepi_engine/share/zed_wrapper/params/'

    CAP_SETTINGS = dict(
      pub_frame_rate = {"type":"Float","name":"pub_frame_rate","options":["0.1","15"]},
      depth_confidence = {"type":"Int","name":"depth_confidence","options":["0","100"]},
      depth_texture_conf = {"type":"Int","name":"depth_texture_conf","options":["0","100"]},
      point_cloud_freq = {"type":"Int","name":"point_cloud_freq","options":["0.1","100"]},
      brightness = {"type":"Int","name":"brightness","options":["0","8"]},
      contrast ={"type":"Int","name":"contrast","options":["0","8"]},
      hue = {"type":"Int","name":"hue","options":["0","11"]},
      saturation ={"type":"Int","name":"saturation","options":["0","8"]},
      sharpness ={"type":"Int","name":"sharpness","options":["0","8"]},
      gamma ={"type":"Int","name":"gamma","options":["1","9"]},
      auto_exposure_gain = {"type":"Bool","name":"auto_exposure_gain"},
      gain = {"type":"Int","name":"gain","options":["0","100"]},
      exposure = {"type":"Int","name":"exposure","options":["0","100"]},
      auto_whitebalance = {"type":"Bool","name":"auto_whitebalance"},
      whitebalance_temperature = {"type":"Int","name":"whitebalance_temperature","options":["1","100"]}
    )

    FACTORY_SETTINGS_OVERRIDES = dict( )
    
    #Factory Control Values 
    FACTORY_CONTROLS = dict( 
    start_range_ratio = 0.0, 
    stop_range_ratio = 1.0,
    min_range_m = 0.0,
    max_range_m = 20.0,
    frame_id = 'sensor_frame' 
    )

    ZED_MIN_RANGE_M_OVERRIDES = { 'zed': .2, 'zedm': .15, 'zed2': .2, 'zedx': .2} 
    ZED_MAX_RANGE_M_OVERRIDES = { 'zed':  15, 'zedm': 15, 'zed2': 20, 'zedx': 15} 

    zed_type = 'zed'

    # Create shared class variables and thread locks 
    
    device_info_dict = dict(node_name = "",
                            device_name = "",
                            identifier = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")
    color_img_acquire = False
    color_img_msg = None
    color_img_last_stamp = None
    color_img_lock = threading.Lock()
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
    
    idx_if = None

    current_fps = 100
    cl_img_last_time = None
    dm_img_last_time = None
    di_img_last_time = None
    pc_img_last_time = None
    pc_last_time = None

    max_framerate = 100

    navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
 
    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"         
    drv_dict = dict()                          
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

        # Get required drv driver dict info
        self.drv_dict = nepi_sdk.get_param('~drv_dict',TEST_DRV_DICT) 

        ################################################
        # Try to restore camera calibration files from
        [success,files_copied,files_not_copied] = nepi_utils.copy_files_from_folder(self.CAL_BACKUP_PATH,self.CAL_SRC_PATH)
        if success:
          if len(files_copied) > 0:
            strList = str(files_copied)
            self.msg_if.pub_info("Restored zed cal files: " + strList)
        else:
          self.msg_if.pub_info("Failed to restore zed cal files")

        # Connect to Zed node
        self.zed_type = self.drv_dict['DEVICE_DICT']['zed_type']
        self.res_val = self.drv_dict['DEVICE_DICT']['res_val']
        self.framerate = self.drv_dict['DEVICE_DICT']['framerate']
        self.data_products = self.drv_dict['DEVICE_DICT']['data_products']
        self.msg_if.pub_warn("Got discovery data products: " + str(self.data_products))
        ZED_BASE_NAMESPACE = os.path.join(self.base_namespace, self.zed_type + "/zed_node/")


        '''
        # First check if zed wrapper already running
        zed_wrapper_not_running = True
        try:
          self.zed_dynamic_reconfig_client = dynamic_reconfigure.client.Client(ZED_BASE_NAMESPACE, timeout=3)
          zed_wrapper_not_running = nepi_drvs.killDriverNode(ZED_BASE_NAMESPACE,self.zed_ros_wrapper_proc)
          nepi_sdk.sleep(2,20)
        except Exception as e:
          pass #self.msg_if.pub_info(str(e))

        if zed_wrapper_not_running == False:
          rospy.signal_shutdown("Zed  Wrapper still running, Shutting Down")
        else:
        '''

        # Set resolution in zed wrapper param file
        zed_params_path = os.path.join(self.ZED_PARAMS_PATH,self.zed_type + ".yaml")
        if os.path.exists(zed_params_path):
          try:
            with open(zed_params_path) as f:
              cfg = yaml.load(f, Loader=yaml.FullLoader)
            #self.msg_if.pub_warn("Updating zed param config with resolution " + str(self.res_val))
            cfg['general']['resolution'] = self.res_val
            cfg['general']['grab_frame_rate'] = self.framerate
            self.msg_if.pub_warn("Updating zed param file: " + zed_params_path + " with cfg " + str(cfg))
            with open(zed_params_path, "w") as f:
                cfg = yaml.dump(
                    cfg, stream=f, default_flow_style=False, sort_keys=False
                )
          except:
            self.msg_if.pub_warn("Failed to update zed param file: " + zed_params_path + " " + str(e))
        else:
          self.msg_if.pub_warn("Failed to find zed param file: " + zed_params_path)

        # Run the correct zed_ros_wrapper launch file
        zed_launchfile = self.zed_type + '.launch'
        zed_ros_wrapper_run_cmd = ['roslaunch', 'zed_wrapper', zed_launchfile]
        # TODO: Some process management for the Zed  wrapper
        self.zed_ros_wrapper_proc = subprocess.Popen(zed_ros_wrapper_run_cmd)
        # Now that Zed SDK is started, we can set up the reconfig client
        nepi_sdk.sleep(5,10)
        success = False
        timeout = 10
        waittime = 1
        timer = 0
        self.zed_dynamic_reconfig_client = None
        while success == False and timer < timeout and not nepi_sdk.is_shutdown():
          try:
            self.zed_dynamic_reconfig_client = dynamic_reconfigure.client.Client(ZED_BASE_NAMESPACE, timeout=3)
            success = True
          except Exception as e:
            self.msg_if.pub_info(str(e))
            nepi_sdk.sleep(waittime,10)
            timer += waittime
        if timer >= timeout or self.zed_dynamic_reconfig_client is None:
          self.msg_if.pub_warn("Failed to connect to zed_node using launch process" + str(zed_ros_wrapper_run_cmd))
          self.msg_if.pub_warn("Killing node named: " + ZED_BASE_NAMESPACE)
          success = nepi_drvs.killDriverNode(ZED_BASE_NAMESPACE,self.zed_ros_wrapper_proc)
          if success:
            time.sleep(2)
          nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because Zed Node not running")
          return
        self.msg_if.pub_warn("Zed DRC: " + str(self.zed_dynamic_reconfig_client))
        time.sleep(2)



        # Zed control topics
        # ZED_PARAMETER_UPDATES_TOPIC = ZED_BASE_NAMESPACE + "parameter_updates"
        # Zed data stream topics
        self.image_topic = ZED_BASE_NAMESPACE + "left/image_rect_color"
        self.depth_map_topic = ZED_BASE_NAMESPACE + "depth/depth_registered"
        self.pc_topic = ZED_BASE_NAMESPACE + "point_cloud/cloud_registered"
        ZED_MIN_RANGE_PARAM = ZED_BASE_NAMESPACE + "depth/min_depth"
        ZED_MAX_RANGE_PARAM = ZED_BASE_NAMESPACE + "depth/max_depth"




        # Wait for zed camera topic to publish, then subscribeCAPS SETTINGS
        self.msg_if.pub_info("Waiting for topic: " + self.image_topic)
        nepi_sdk.wait_for_topic(self.image_topic)

        self.msg_if.pub_info("Starting Zed IDX subscribers and publishers")
        self.color_img_sub = None
        self.depth_map_sub = None
        self.depth_img_sub = None
        self.pc_sub = None
        self.pc_img_sub = None

        ZED_ODOM_TOPIC = ZED_BASE_NAMESPACE + "odom"
        odom_sub = rospy.Subscriber(ZED_ODOM_TOPIC, Odometry, self.odom_topic_callback)

        # Launch the  node
        self.msg_if.pub_info("... Connected!")

        # Initialize controls
        self.factory_controls = self.FACTORY_CONTROLS
        # Apply OVERRIDES
        if self.zed_type in self.ZED_MIN_RANGE_M_OVERRIDES:
          self.factory_controls['min_range_m'] = self.ZED_MIN_RANGE_M_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_MAX_RANGE_M_OVERRIDES:
          self.factory_controls['max_range_m'] = self.ZED_MAX_RANGE_M_OVERRIDES[self.zed_type]
        
        self.current_controls = self.factory_controls # Updateded during initialization
        self.current_fps = self.framerate # Should be updateded when settings read

        # Initialize settings
        self.cap_settings = self.getCapSettings()
        self.factory_settings = self.getFactorySettings()
            

        # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
        self.msg_if.pub_info("Launching NEPI IDX () interface...")
        self.device_info_dict["node_name"] = self.node_name
        if self.node_name.find("_") != -1:
            split_name = self.node_name.rsplit('_', 1)
            self.device_info_dict["device_name"] = split_name[0]
            self.device_info_dict["identifier"] = split_name[1]
        else:
            self.device_info_dict["device_name"] = self.node_name
        self.idx_if = IDXDeviceIF(device_info = self.device_info_dict,
                                    data_products =  self.data_products,
                                    data_source_description = 'stereo_camera',
                                    data_ref_description = 'left_camera_lense',
                                    capSettings = self.cap_settings,
                                    factorySettings = self.factory_settings,
                                    settingUpdateFunction=self.settingUpdateFunction,
                                    getSettingsFunction=self.getSettings,
                                    factoryControls = self.factory_controls,
                                    setMaxFramerate =self.setMaxFramerate, 
                                    getFramerate = self.driver.getFramerate,
                                    setRangeRatio = self.setRangeRatio,
                                    getColorImage = self.getColorImage, 
                                    stopColorImageAcquisition = self.stopColorImage,
                                    getDepthMap = self.getDepthMap, 
                                    stopDepthMapAcquisition = self.stopDepthMap,
                                    getPointcloud = self.getPointcloud, 
                                    stopPointcloudAcquisition = self.stopPointcloud,                                  
                                    getNavPoseCb = self.getNavPoseDict, 
                                    navpose_update_rate = 10 
                                    )
        self.msg_if.pub_info("... IDX interface running")

        # Update available IDX callbacks based on capabilities that the driver reports
        self.logDeviceInfo()

        # Now that all camera start-up stuff is processed, we can update the camera from the parameters that have been established

        # Try to backup camera calibration files
        [success,files_copied,files_not_copied] = nepi_utils.copy_files_from_folder(self.CAL_SRC_PATH,self.CAL_BACKUP_PATH)
        if success:
          if len(files_copied) > 0:
            strList = str(files_copied)
            self.msg_if.pub_info("Backed up zed cal files: " + strList)
        else:
          self.msg_if.pub_info("Failed to back up up zed cal files")


        ##########################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Now start zed node check process
        self.attempts = 0
        nepi_sdk.start_timer_process((1), self.checkZedNodeCb)
        rospy.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()

    def checkZedNodeCb(self,timer):
      poll = self.zed_ros_wrapper_proc.poll()
      running = poll is None
      if running:
        self.attempts = 0
      else:
        self.attempts += 1
      if self.attempts > 2:
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because Zed Node not running")

      



    #**********************
    # Sensor setting functions
    def getCapSettings(self):
      return self.CAP_SETTINGS

    def getFactorySettings(self):
      settings = self.getSettings()
      #Apply factory setting overides
      for setting_name in settings.keys():
        if setting_name in self.FACTORY_SETTINGS_OVERRIDES:
              setting = settings[setting_name]
              setting['value'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
              settings[setting_name] = setting
      return settings


    def getSettings(self):
      settings = dict()
      config_dict = self.zed_dynamic_reconfig_client.get_configuration()
      if config_dict is not None:
        for setting_name in self.cap_settings.keys():
          cap_setting = self.cap_settings[setting_name]
          setting = dict()
          setting["name"] = cap_setting['name']
          setting["type"] = cap_setting['type']
          if setting_name in config_dict.keys():
            setting["value"] = str(config_dict[setting_name])
            settings[setting_name] = setting
      return settings

    def settingUpdateFunction(self,setting):
      success = False
      setting_str = str(setting)
      [s_name, s_type, data] = nepi_settings.get_data_from_setting(setting)
      if data is not None:
        setting_name = setting['name']
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
      return success, msg


    #**********************
    # Zed camera data callbacks

    # callback to get color 2d image data
    def color_2d_image_callback(self, image_msg):
        # Check for control framerate adjustment
        last_time = self.cl_img_last_time
        current_time = nepi_utils.get_time()

        need_data = False
        if last_time != None and self.idx_if is not None:
          fr_delay = float(1) / self.max_framerate
          timer = current_time - last_time
          if timer > fr_delay:
            need_data = True
        else:
          need_data = True

        # Get and Process Data if Needed
        if need_data == True:
          self.cl_img_last_time = current_time

          self.color_img_lock.acquire()
          self.color_img_msg = image_msg
          self.color_img_lock.release()

    # callback to get depthmap
    def depth_map_callback(self, image_msg):
        # Check for control framerate adjustment
        last_time = self.dm_img_last_time
        current_time = nepi_utils.get_time()

        need_data = False
        if last_time != None and self.idx_if is not None:
          fr_delay = float(1) / self.max_framerate
          timer = current_time - last_time
          if timer > fr_delay:
            need_data = True
        else:
          need_data = True


        # Get and Process Data if Needed
        if need_data == True:
          self.dm_img_last_time = current_time

          image_msg.header.stamp = nepi_sdk.get_msg_stamp()
          self.depth_map_lock.acquire()
          self.depth_map_msg = image_msg
          self.depth_map_lock.release()

    # callback to get depthmap
    def depth_image_callback(self, image_msg):
        # Check for control framerate adjustment
        last_time = self.di_img_last_time
        current_time = nepi_utils.get_time()

        
        need_data = False
        if last_time != None and self.idx_if is not None:
          fr_delay = float(1) / self.max_framerate
          timer = current_time - last_time
          if timer > fr_delay:
            need_data = True
        else:
          need_data = True


        # Get and Process Data if Needed
        if need_data == True:
          self.di_img_last_time = current_time

          image_msg.header.stamp = nepi_sdk.get_msg_stamp()
          self.depth_img_lock.acquire()
          self.depth_img_msg = image_msg
          self.depth_img_lock.release()

    # callback to get and republish point_cloud
    def pointcloud_callback(self, pointcloud_msg):
        # Check for control framerate adjustment
        last_time = self.pc_last_time
        current_time = nepi_utils.get_time()


        need_data = False
        if last_time != None and self.idx_if is not None:
          fr_delay = float(1) / self.max_framerate
          timer = current_time - last_time
          if timer > fr_delay:
            need_data = True
        else:
          need_data = True


        # Get and Process Data if Needed
        if need_data == True:
          self.pc_last_time = current_time

          self.pc_lock.acquire()
          self.pc_msg = pointcloud_msg
          self.pc_lock.release()


          self.pc_img_last_time = current_time

          self.pc_img_lock.acquire()
          self.pc_img_msg = pointcloud_msg
          self.pc_img_lock.release()

        # callback to get and process point_cloud image


    def getNavPoseDict(self):
      return self.navpose_dict

    def getOrientationDict(self):
      return self.orientation_dict

        
    ### Callback to publish RBX odom topic
    def odom_topic_callback(self,odom_msg):
        # Convert quaternion to roll,pitch,yaw
        pose = odom_msg.pose.pose.orientation
        xyzw = list([pose.x,pose.y,pose.z,pose.w])
        rpy = nepi_nav.convert_quat2rpy(xyzw)

        # Convert position body to position ENU
        body = odom_msg.pose.pose.position
        xyz_body_o = list([body.x, body.y, body.z])
        xyz = nepi_nav.convert_point_body2enu(xyz_body_o,rpy[2])

        timestamp = nepi_sdk.sec_from_msg_stamp(odom_msg.header.stamp)

        self.navpose_dict['has_orientation']
        self.navpose_dict['time_oreantation'] = timestamp
        # Orientation Degrees in selected 3d frame (roll,pitch,yaw)
        self.navpose_dict['roll_deg'] = rpy[0]
        self.navpose_dict['pitch_deg'] = rpy[1]
        self.navpose_dict['yaw_deg'] = rpy[2]

        self.navpose_dict['has_position']
        self.navpose_dict['time_position'] = timestamp
        # Relative Position Meters in selected 3d frame (x,y,z) with x forward, y right/left, and z up/down
        self.navpose_dict['x_m'] = xyz[0]
        self.navpose_dict['y_m'] = xyz[1]
        self.navpose_dict['z_m'] = xyz[2]


    #**********************
    # IDX driver functions

    def logDeviceInfo(self):
        device_info_str = self.node_name + " info:\n"
        self.msg_if.pub_info(device_info_str)
        self.msg_if.pub_info(str(self.device_info_dict))

     
    def setMaxFramerate(self, rate):
        if rate is None:
            return False, 'Got None Max Framerate'
        if rate < 1:
            rate = 1
        if rate > 100:
            rate = 100
        self.max_framerate = rate
        #print('Set FR Mode: ' +  str(self.current_controls["max_framerate"]))
        status = True
        err_str = ""
        return status, err_str


    def setRangeRatio(self, min_ratio, max_ratio):
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
    def getColorImage(self):
        if self.color_img_sub == None:
          self.color_img_sub = rospy.Subscriber(self.image_topic, Image, self.color_2d_image_callback, queue_size = 1)
          time.sleep(0.1)

        

        # Set process input variables
        data_product = "color_2d_image"
        self.color_img_lock.acquire()
        img_msg = None
        if self.color_img_msg != None:
          img_msg = copy.deepcopy(self.color_img_msg)    
          self.color_img_msg = None   
        self.color_img_lock.release()
        encoding = 'bgr8'

        # Run get process
        # Initialize some process return variables
        status = False
        msg = ""
        cv2_img = None
        timestamp = None
        if img_msg is not None:
          if img_msg.header.stamp != self.color_img_last_stamp:
            timestamp = nepi_sdk.sec_from_msg_stamp(img_msg.header.stamp)
            status = True
            msg = ""
            timestamp = nepi_sdk.sec_from_msg_stamp(img_msg.header.stamp)
            cv2_img =  nepi_img.rosimg_to_cv2img(img_msg, encoding = encoding)
            self.color_img_last_stamp = timestamp
          else:
            msg = "No new data for " + data_product + " available"
        else:
          msg = "Received None type data for " + data_product + " process"
        return status, msg, cv2_img, timestamp, encoding

    
    # Good base class candidate - Shared with ONVIF
    def stopColorImage(self):
        self.color_img_lock.acquire()
        self.color_img_sub.unregister()
        self.color_img_sub = None
        self.color_img_msg = None
        self.color_img_lock.release()
        self.msg_if.pub_warn("Stoped Color Image Acquire")
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
          img_msg = copy.deepcopy(self.depth_map_msg)    
          self.depth_map_msg = None 
        self.depth_map_lock.release()
        encoding = '32FC1'
        # Run get process
        # Initialize some process return variables
        status = False
        msg = ""
        cv2_img = None
        timestamp = None
        if img_msg is not None:
          if img_msg.header.stamp != self.depth_map_last_stamp:
            timestamp = nepi_sdk.sec_from_msg_stamp(img_msg.header.stamp)
            self.depth_map_last_stamp = timestamp
            status = True
            msg = ""
            # Adjust range Limits if IDX Controls enabled and range ratios are not min/max
            start_range_ratio = self.current_controls.get("start_range_ratio")
            stop_range_ratio = self.current_controls.get("stop_range_ratio")
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
          return status, msg, cv2_img, timestamp, encoding
        else: 
          return status, msg, img_msg, timestamp, encoding
    
    def stopDepthMap(self):
        self.depth_map_lock.acquire()

        self.depth_map_sub.unregister()
        self.depth_map_sub = None

        self.depth_map_msg = None
        self.depth_map_lock.release()
        self.msg_if.pub_warn("Stoped Depthmap Acquire")
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
          pc_msg = copy.deepcopy(self.pc_msg)    
          self.pc_msg = None 
        self.pc_lock.release()
        # Run get process
        # Initialize some process return variables
        status = False
        msg = ""
        o3d_pc = None
        timestamp = None
        frame_id = None
        if pc_msg is not None:
          if pc_msg.header.stamp != self.pc_last_stamp:
            timestamp = nepi_sdk.sec_from_msg_stamp(pc_msg.header.stamp)
            frame_id = pc_msg.header.frame_id
            status = True
            msg = ""
            self.pc_last_stamp = timestamp
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
          return status, msg, o3d_pc, timestamp, frame_id
        else: 
          return status, msg, pc_msg, timestamp, frame_id

    
    def stopPointcloud(self):
      self.pc_lock.acquire()

      self.pc_sub.unregister()
      self.pc_sub = None

      self.pc_msg = None
      self.pc_lock.release()
      ret = True
      msg = "Success"
      return ret,msg


    def cleanup_actions(self):
      self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
      try:
        zed_type = self.zed_type
        zed_node_namespace = os.path.join(self.base_namespace,zed_type,'zed_node')
        nepi_sdk.kill_node_namespace(zed_node_namespace)
      except Exception as e:
        self.msg_if.pub_warn("Failed to kill zed node namespace " + zed_node_namespace + " " + str(e))
      try:
        zed_type = self.zed_type
        zed_node_namespace = os.path.join(self.base_namespace,zed_type,zed_type + '_state_publisher')
        nepi_sdk.kill_node_namespace(zed_node_namespace)
      except Exception as e:
        self.msg_if.pub_warn("Failed to kill zed node namespace " + zed_node_namespace + " " + str(e))
        
if __name__ == '__main__':
    node = ZedCamNode()
