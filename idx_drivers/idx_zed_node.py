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
import threading
import cv2
import open3d as o3d
import numpy as np
import math
import copy



pyzed_folder = '/home/nepi/.local/lib/python3.8/site-packages'
sys.path.insert(0, pyzed_folder)
import pyzed.sl as sl

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
# nepi_sdk import nepi_img
# from nepi_sdk import nepi_pc
#from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_settings


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
'DEVICE_DICT': {'zed_type': 'zed2','resolution': 'VGA'},
}



class ZedCamNode(object):
    CHECK_INTERVAL_S = 3.0
    CAL_SRC_PATH = "/usr/local/zed/settings"
    CAL_BACKUP_PATH = "/mnt/nepi_storage/user_cfg/cals"



    #CAP_SETTINGS = nepi_settings.NONE_CAP_SETTINGS
    CAP_SETTINGS = dict(
      #pub_frame_rate = {"type":"Float","name":"pub_frame_rate","options":["0.1","15"]},
      #depth_confidence = {"type":"Int","name":"depth_confidence","options":["0","100"]},
      #depth_texture_conf = {"type":"Int","name":"depth_texture_conf","options":["0","100"]},
      point_cloud_freq = {"type":"Int","name":"point_cloud_freq","options":["1","10"]},
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
      whitebalance_temperature = {"type":"Int","name":"whitebalance_temperature","options":["2800","6500"]}
    )


    CAP_ZED_DICT = dict(
      brightness = sl.VIDEO_SETTINGS.BRIGHTNESS,
      contrast = sl.VIDEO_SETTINGS.CONTRAST,
      hue = sl.VIDEO_SETTINGS.HUE,
      saturation = sl.VIDEO_SETTINGS.SATURATION,
      sharpness = sl.VIDEO_SETTINGS.SHARPNESS,
      gamma = sl.VIDEO_SETTINGS.GAMMA,
      gain = sl.VIDEO_SETTINGS.GAIN,
      exposure = sl.VIDEO_SETTINGS.EXPOSURE,
      auto_exposure_gain = sl.VIDEO_SETTINGS.AEC_AGC_ROI,
      whitebalance_temperature = sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE,
      auto_whitebalance = sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO


    )

    FACTORY_SETTINGS_OVERRIDES = dict( )
    
    #Factory Control Values 
    FACTORY_CONTROLS = dict( 
    start_range_ratio = 0.0, 
    stop_range_ratio = 1.0,
    min_range_m = 0.0,
    max_range_m = 20.0,
    width_deg = 110,
    height_deg = 70,
    frame_id = 'sensor_frame' 
    )

    data_source_description = 'stereo_camera'
    data_ref_description = 'left_camera_lense'

    ZED_MIN_RANGE_M_OVERRIDES = { 'zed': .2, 'zedm': .15, 'zed2': .2, 'zedx': .2} 
    ZED_MAX_RANGE_M_OVERRIDES = { 'zed':  15, 'zedm': 15, 'zed2': 20, 'zedx': 15} 
    ZED_WIDTH_DEG_OVERRIDES = { 'zed': 110, 'zedm': 110, 'zed2': 110, 'zedx': 110} 
    ZED_HEIGHT_DEG_OVERRIDES = { 'zed':  70, 'zedm': 70, 'zed2': 70, 'zedx': 80} 

    zed = None

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
    dm_data_last_time = None
    di_img_last_time = None
    pc_img_last_time = None
    pc_last_time = None

    zed_type = 'zed'
    resolution = 'VGA'
    framerate = 15
    data_products = []

    max_framerate = 100
    max_pointcloud_framerate = 1

    runtime_parameters = sl.RuntimeParameters()
    zed_pose = sl.Pose()

    cap_settings = CAP_SETTINGS

    navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
 
    nav_published = False
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
        self.resolution = self.drv_dict['DEVICE_DICT']['resolution']
        self.framerate = 15 # self.drv_dict['DEVICE_DICT']['framerate']
        self.data_products = self.drv_dict['DEVICE_DICT']['data_products']
        self.msg_if.pub_warn("Got discovery data products: " + str(self.data_products))


        # Create a Camera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.sensors_required = True 
        res_dict = {
            'HD2K': sl.RESOLUTION.HD2K,
            'HD1080': sl.RESOLUTION.HD1080,
            'HD720': sl.RESOLUTION.HD720,
            'VGA': sl.RESOLUTION.VGA
        }

        if self.resolution in res_dict.keys():
           resolution = res_dict[self.resolution]
        else:
           resolution = sl.RESOLUTION.AUTO

        init_params.camera_resolution = resolution #sl.RESOLUTION.AUTO # Use HD720 opr HD1200 video mode, depending on camera type.
        init_params.camera_fps = self.framerate #30  # Set fps at 30
        # Use a right-handed Y-up coordinate system
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.coordinate_units = sl.UNIT.METER  # Set units in meters



        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.msg_if.pub_warn("Camera Open : " + str(err))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because Not Able to Connect to Zed Camera")

        self.msg_if.pub_info("Zed Camera Connected!")

        self.tracking_parameters = sl.PositionalTrackingParameters()
        err = self.zed.enable_positional_tracking(self.tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            self.msg_if.pub_warn("Positional Tracking enable failed : " + str(err))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because Not Able to Connect to Zed Camera")


       # Initialize controls
        self.factory_controls = self.FACTORY_CONTROLS
        # Apply OVERRIDES
        if self.zed_type in self.ZED_MIN_RANGE_M_OVERRIDES:
          self.factory_controls['min_range_m'] = self.ZED_MIN_RANGE_M_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_MAX_RANGE_M_OVERRIDES:
          self.factory_controls['max_range_m'] = self.ZED_MAX_RANGE_M_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_WIDTH_DEG_OVERRIDES:
          self.factory_controls['width_deg'] = self.ZED_WIDTH_DEG_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_HEIGHT_DEG_OVERRIDES:
          self.factory_controls['height_deg'] = self.ZED_HEIGHT_DEG_OVERRIDES[self.zed_type]
        
        self.current_controls = self.factory_controls # Updateded during initialization
        self.current_fps = self.framerate # Should be updateded when settings read

        # Initialize settings
        # self.cap_settings = self.getCapSettings()
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
                                    data_source_description = self.data_source_description,
                                    data_ref_description = self.data_ref_description,
                                    capSettings = self.cap_settings,
                                    factorySettings = self.factory_settings,
                                    settingUpdateFunction=self.settingUpdateFunction,
                                    getSettingsFunction=self.getSettings,
                                    factoryControls = self.factory_controls,
                                    setMaxFramerate =self.setMaxFramerate, 
                                    getFramerate = self.getFramerate,
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
        #nepi_sdk.start_timer_process((1), self.checkZedStatusCb)
        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()

    
    
    
    def checkZedStatusCb(self,timer):
        timestamp = None
        try:
          timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
        except:
          pass
        
        if timestamp is None:
          self.msg_if.pub_info("Failed to get timestamp from zed camera")
          nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because Zed Node not running")

      



    #**********************
    # Sensor setting functions
    def getCapSettings(self):
      setting = getSettings()
      return self.cap_settings

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
      #settings = nepi_settings.NONE_SETTINGS
      purge_settings = []
      settings = dict()
      camera_settings = sl.VIDEO_SETTINGS
      for setting_name in self.cap_settings.keys():
        cap_setting = self.cap_settings[setting_name]
        setting = dict()
        setting["name"] = cap_setting['name']
        setting["type"] = cap_setting['type']
        if setting_name in self.CAP_ZED_DICT.keys():

          try:
            zed_name = setting_name.upper()
            zed_setting = self.CAP_ZED_DICT[setting_name]
            value = self.zed.get_camera_settings(zed_setting)[1]
            if zed_name == "WHITEBALANCE_AUTO":
              value = value == 1
            elif zed_name == "AEC_AGC_ROI":
              value = value == 1 
            
            setting["value"] = str(value)
            settings[setting_name] = setting

          except Exception as e:
              purge_settings.append(setting_name)
              self.msg_if.pub_warn("Failed to get setting: " + str(zed_name) + " : " + str(e))
        elif setting_name == "point_cloud_freq":
            value = self.max_pointcloud_framerate
            setting["value"] = str(value)
            settings[setting_name] = setting
      for setting_name in purge_settings:
         del self.cap_settings[setting_name]

      #self.msg_if.pub_warn("got settings: " + str(settings))

      # Get the pose of the left eye of the camera with reference to the world frame



      return settings

    def settingUpdateFunction(self,setting):
      success = False
      msg = ""
      setting_str = str(setting)
      [s_name, s_type, data] = nepi_settings.get_data_from_setting(setting)
      if data is not None:
        setting_name = setting['name']
        setting_data = data

        if setting_name in self.CAP_ZED_DICT.keys():
          zed_name = setting_name.upper()
          zed_setting = self.CAP_ZED_DICT[setting_name]  
          if zed_name == "WHITEBALANCE_AUTO":
              if data == True:
                data = 1
              elif data == False:
                 data = 0
          elif zed_name == "AEC_AGC_ROI":
              if data == True:
                data = 1
              elif data == False:
                 data = 0        
          try:
            self.zed.set_camera_settings(zed_setting, data)
            success = True
            msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)

          except Exception as e:
            self.msg_if.pub_warn("Failed to set setting: " + str(zed_name) + " : " + str(e))
        elif setting_name == "point_cloud_freq":
          success = self.setPointcloudFramerate(data)
        else:
          msg = (self.node_name  + " Setting name" + setting_str + " is not supported")                   
      else:
        msg = (self.node_name  + " Setting data" + setting_str + " is None")
      return success, msg


    #**********************
    # Zed camera data callbacks


    def getNavPoseDict(self):
        
        success = False
        navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        navpose_dict['has_orientation'] = True
        navpose_dict['time_oreantation'] = nepi_utils.get_time()

        rpy = self.getOrientation()
        if rpy is not None:
              
            navpose_dict['roll_deg'] = rpy[0]
            navpose_dict['pitch_deg'] = rpy[1]
            navpose_dict['yaw_deg'] = rpy[2]



            # Relative Position Meters in selected 3d frame (x,y,z) with x forward, y right/left, and z up/down
            xyz = self.getPosition()
            self.msg_if.pub_warn("p called")
            if xyz is not None:
              navpose_dict['x_m'] = xyz[0]
              navpose_dict['y_m'] = xyz[1]
              navpose_dict['z_m'] = xyz[2]
              navpose_dict['has_position'] = True
              navpose_dict['time_position'] = nepi_utils.get_time()

              success = True

              # if self.nav_published == False:
              #    self.msg_if.pub_warn("nav navpose_dict befor: " + str(navpose_dict))

              #navpose_dict = nepi_nav.convert_navpose_ned2edu(navpose_dict)
              # self.msg_if.pub_warn("roll: " + str(navpose_dict['roll_deg']) + " | pitch: " + str(navpose_dict['pitch_deg']) + " | yaw: " + str(navpose_dict['yaw_deg']))


              if self.nav_published == False:
                self.nav_published = True
                self.msg_if.pub_warn("nav navpose_dict after: " + str(navpose_dict))

        if success == False:
           navpose_dict = None
        return navpose_dict
    
    def getOrientation(self):
        rpy = None
        # return rpy
        orientation_pose = sl.Pose()
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
              # Get camera pose
              self.zed.get_position(orientation_pose, sl.REFERENCE_FRAME.WORLD)

              # Get orientation as a quaternion
              orientation = orientation_pose.get_orientation()
              # You can use the quaternion values (orientation.get()[0], etc.)
              
              # Get orientation directly as Euler angles (roll, pitch, yaw)
              # Angles are in radians by default. Use get_roll_pitch_yaw() to get them in a list/vector.
              roll_pitch_yaw = orientation_pose.get_euler_angles() 
              roll_deg = round(math.degrees(roll_pitch_yaw[2])*-1, 3)
              pitch_deg = round(math.degrees(roll_pitch_yaw[0])*-1)
              yaw_deg = round(math.degrees(roll_pitch_yaw[1])*-1)

              rpy = [roll_deg, pitch_deg, yaw_deg]
              #self.msg_if.pub_warn("roll deg: " + str(roll_deg) + " | pitch deg: " + str(pitch_deg) + " | yaw deg: " + str(yaw_deg))
        return rpy
    
    def getPosition(self):
      position = None
      position_pose = sl.Pose()

      if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # Get the pose of the left eye of the camera with reference to the world frame
        # Get the pose of the camera relative to the world frame
        state = self.zed.get_position(position_pose, sl.REFERENCE_FRAME.WORLD)
        # Display translation and timestamp
        py_translation = sl.Translation()
        tx = round(position_pose.get_translation(py_translation).get()[0], 3)
        ty = round(position_pose.get_translation(py_translation).get()[1], 3)
        tz = round(position_pose.get_translation(py_translation).get()[2], 3)

        position = [tx, ty, tz]
      return position


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
    
    def setPointcloudFramerate(self, rate):
        if rate is None:
            return False, 'Got None Max Framerate'
        if rate < 1:
            rate = 1
        if rate > 10:
            rate = 10
        self.max_pointcloud_framerate = rate
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


 
    def getFramerate(self):
       return int(self.framerate)


    # Good base class candidate - Shared with ONVIF
    def getColorImage(self):
      status = False
      msg = ""
      cv2_img = None
      timestamp = None
      encoding = 'bgr8'
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

      #need_data = True
      # Get and Process Data if Needed
      if need_data == False:
        return False, "Waiting for Timer", None, None, None  # Return None data
      else:
        # Grab an image, a RuntimeParameters object must be given to grab()
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            status = True
            zed_img = sl.Mat()
            # A new image is available if grab() returns SUCCESS
            self.zed.retrieve_image(zed_img, sl.VIEW.LEFT)
            cv2_img = cv2.cvtColor(zed_img.get_data(), cv2.COLOR_BGRA2BGR)
            timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
            self.cl_img_last_time = nepi_utils.get_time()
        return status, msg, cv2_img, timestamp, encoding

      
    # Good base class candidate - Shared with ONVIF
    def stopColorImage(self):
        self.stop_color_img = True
        # self.color_img_lock.acquire()
        # self.color_img_sub.unregister()
        # self.color_img_sub = None
        # self.color_img_msg = None
        # self.color_img_lock.release()
        self.msg_if.pub_warn("Stoped Color Image Acquire")
        ret = True
        msg = "Success"
        return ret,msg
    
    def getDepthMap(self):
      status = False
      msg = ""
      np_depth_map = None
      encoding = '32FC1'
      # Check for control framerate adjustment
      last_time = self.dm_data_last_time
      current_time = nepi_utils.get_time()
      
      need_data = False
      if last_time != None and self.idx_if is not None:
        fr_delay = float(1) / self.max_framerate
        timer = current_time - last_time
        if timer > fr_delay:
          need_data = True
      else:
        need_data = True

      #need_data = True
      # Get and Process Data if Needed
      if need_data == False:
        return False, "Waiting for Timer", None, None, None  # Return None data
      else:
        # Grab an depth_image_zed, a RuntimeParameters object must be given to grab()
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            status = True
            zed_depth = sl.Mat()
            self.zed.retrieve_measure(zed_depth, sl.MEASURE.DEPTH, sl.MEM.CPU)
            zed_depth_map = zed_depth.get_data() 
            zed_depth_map[np.isinf(zed_depth_map)] = np.nan
            #print('Zed Depth Map Min Max: ' + str([np.nanmin(zed_depth_map),np.nanmax(zed_depth_map)]) )
            timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
            np_depth_map = (np.array(zed_depth_map, dtype=np.float32)) # replace nan values
            self.dm_data_last_time = nepi_utils.get_time()
        return status, msg, np_depth_map, timestamp, encoding


    def stopDepthMap(self):
        self.stop_depth_map = True

        # self.depth_map_lock.acquire()

        # self.depth_map_sub.unregister()
        # self.depth_map_sub = None

        # self.depth_map_msg = None
        # self.depth_map_lock.release()
        self.msg_if.pub_warn("Stopped Depthmap Acquire")
        ret = True
        msg = "Success"
        return ret,msg

    def getPointcloud(self): 
        status = False
        msg = ""    
        frame_id = "sensor frame"
        o3d_pc = None
        # Check for control framerate adjustment
        last_time = self.pc_data_last_time
        current_time = nepi_utils.get_time()
        
        need_data = False
        if last_time != None and self.idx_if is not None:
          max_framerate = min(self.max_framerate, self.max_pointcloud_framerate)
          fr_delay = float(1) / max_framerate
          timer = current_time - last_time
          if timer > fr_delay:
            need_data = True
        else:
          need_data = True

        #need_data = True
        # Get and Process Data if Needed
        if need_data == False:
          return False, "Waiting for Timer", None, None, None  # Return None data
        else:
          # Initialize some process return variables
          if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
              status = True
              # Retrieve the point cloud
              point_cloud = sl.Mat()
              self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
              # Get the point cloud data as a numpy array
              zed_pc = point_cloud.get_data()
              # Extract the XYZ data
              xyz_data = zed_pc[:, :, :3]
              xyz_data = xyz_data.reshape(-1,3)
              # Create an Open3D point cloud
              o3d_pc = o3d.geometry.PointCloud()
              o3d_pc.points = o3d.utility.Vector3dVector(xyz_data)

              timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
              self.pc_data_last_time = nepi_utils.get_time()            
          return status, msg, o3d_pc, timestamp, frame_id


    
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
        self.zed.close()
        self.msg_if.pub_warn("Closed zed sdk connection")
      except Exception as e:
        self.msg_if.pub_warn("Failed to close zed sdk connection" + str(e))
        
if __name__ == '__main__':
    node = ZedCamNode()
