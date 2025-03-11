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


import os
import sys
import time
import math
import threading
import cv2

from nepi_sdk.device_if_idx import ROSIDXSensorIF

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img
from nepi_sdk import nepi_drv
from nepi_sdk import nepi_settings

PKG_NAME = 'IDX_ONVIF_GENERIC' # Use in display menus
FILE_TYPE = 'NODE'


class OnvifCamNode:

    FACTORY_SETTINGS_OVERRIDES = dict(WhiteBalance_Mode = "AUTO",
                                      Exposure_Mode = "AUTO",
                                      Resolution = "2560:1440" )

    #Factory Control Values 
    FACTORY_CONTROLS = dict( controls_enable = True,
    auto_adjust = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    resolution_mode = 1, # LOW, MED, HIGH, MAX
    framerate_mode = 1, # LOW, MED, HIGH, MAX
    start_range_ratio = None, 
    stop_range_ratio = None,
    min_range_m = None,
    max_range_m = None,
    frame_id = None
    )
 
    DEFAULT_CURRENT_FPS = 20 # Will be update later with actual
    
    device_info_dict = dict(node_name = "",
       sensor_name = "",
       identifier = "",
       serial_number = "",
       hw_version = "",
       sw_version = "")


    idx_if = None

    current_fps = 20
    cl_img_last_time = None
    bw_img_last_time = None
    dm_img_last_time = None
    di_img_last_time = None
    pc_img_last_time = None
    pc_last_time = None
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
            while not nepi_ros.is_shutdown() and driver_constructed == False and attempts < 5 and not nepi_ros.is_shutdown():
                try:
                    self.driver = self.driver_class(username, password, host, onvif_port)
                    driver_constructed = True
                    nepi_msg.publishMsgInfo(self,"ONVIF_NODE: Driver constructed")
                except Exception as e:
                    nepi_msg.publishMsgInfo(self,"ONVIF_NODE: Failed to construct driver " + self.driver_module + " with exception: " + str(e))
                    time.sleep(1)
                attempts += 1 
        if driver_constructed == False:
            nepi_ros.signal_shutdown("Shutting down Onvif node " + self.node_name + ", unable to connect to driver")
        else:
            ################################################
            nepi_msg.publishMsgInfo(self,"... Connected!")
            self.dev_info = self.driver.getDeviceInfo()
            self.logDeviceInfo()        
            # Configurable IDX parameter and data output remapping to support specific camera needs/capabilities
            # Don't edit this table directly -- do it through idx_remapping parameters
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
                    "Range":  None
                },
                "Data" : {
                    # Data callbacks
                    "Color2DImg": self.getColorImg,
                    "StopColor2DImg": self.stopColorImg,
                    "BW2DImg": self.getBWImg,
                    "StopBW2DImg": self.stopBWImg,
                    "DepthMap": None, 
                    "StopDepthMap": None,
                    "DepthImg": None, 
                    "StopDepthImg": None,
                    "Pointcloud": None, 
                    "StopPointcloud": None,
                    "PointcloudImg": None, 
                    "StopPointcloudImg": None,
                    "GPS": None,
                    "Odom": None,
                    "Heading": None,
                }
            }

            # Establish the URI indices (from ONVIF "Profiles") for the two image streams.
            # If these aren't the same, encoder param adjustments (resolution and framerate)
            # will only affect the first one.... so
            # TODO: Consider a scheme for adjusting parameters for separate streams independently
            # or in lock-step. Not sure if the uri_index and encoder_index have the same meaning
            self.img_uri_index = nepi_ros.get_param(self,'~/img_uri_index', 0)
            nepi_ros.set_param(self,'~/img_uri_index', self.img_uri_index)
            #self.bw_2d_img_uri_index = nepi_ros.get_param(self,'~/image_uris/bw_2d_img_uri_index', 0)
            #nepi_ros.set_param(self,'~/image_uris/bw_2d_img_uri_index', self.bw_2d_img_uri_index)

            # Create threading locks for each URI index (currently just 1) to provide threadsafety
            self.img_uri_lock = threading.Lock()
            self.color_image_acquisition_running = False
            self.bw_image_acquisition_running = False
            self.cached_2d_color_frame = None
            self.cached_2d_color_frame_timestamp = None


            # Initialize controls
            self.factory_controls = self.FACTORY_CONTROLS
            self.current_controls = self.factory_controls # Updateded during initialization
            self.current_fps = self.DEFAULT_CURRENT_FPS # Should be updateded when settings read

            # Initialize settings
            self.cap_settings = self.getCapSettings()
            self.factory_settings = self.getFactorySettings()

            # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
            nepi_msg.publishMsgInfo(self,"Launching NEPI IDX (ROS) interface...")
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
                                        get_rtsp_url = self.getRtspUrl,
                                        setControlsEnable = idx_callback_names["Controls"]["Controls_Enable"],
                                        setAutoAdjust= idx_callback_names["Controls"]["Auto_Adjust"],
                                        setResolutionMode=idx_callback_names["Controls"]["Resolution"], 
                                        setFramerateMode=idx_callback_names["Controls"]["Framerate"], 
                                        setContrast=idx_callback_names["Controls"]["Contrast"], 
                                        setBrightness=idx_callback_names["Controls"]["Brightness"], 
                                        setThresholding=idx_callback_names["Controls"]["Thresholding"], 
                                        setRange=idx_callback_names["Controls"]["Range"], 
                                        getFramerate = self.getFramerate,
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
            nepi_msg.publishMsgInfo(self," ... IDX interface running")
            # Now that all camera start-up stuff is processed, we can update the camera from the parameters that have been established
            self.idx_if.updateFromParamServer()

            # Now start the node
            nepi_ros.spin()


    #**********************
    # Sensor setting functions

    def getRtspUrl(self):
        onvif_port = str(nepi_ros.get_param(self,'~network/port', 80))
        onvif_address = str(nepi_ros.get_param(self,'~network/host',""))
        url = "http://" + onvif_address #+ ":" + onvif_port
        username = str(nepi_ros.get_param(self,'~credentials/username'))
        password = str(nepi_ros.get_param(self,'~credentials/password'))
        return url, username, password


    def getCapSettings(self):
        cap_settings = dict()
        controls_dict = self.driver.getCameraControls()
        for cap_setting_name in controls_dict.keys():
            if 'type' in controls_dict[cap_setting_name]:
                cap_setting = dict()
                cap_setting['name'] = cap_setting_name
                info = controls_dict[cap_setting_name]
                cap_setting_type = info['type']
                if cap_setting_type == 'int':
                    cap_setting_type = 'Int'
                elif cap_setting_type == 'float':
                    cap_setting_type = 'Float'
                elif cap_setting_type == 'bool':
                    cap_setting_type = 'Bool'
                elif cap_setting_type == 'discrete':
                    cap_setting_type = 'Discrete'
                cap_setting['type'] = cap_setting_type
                if cap_setting_type == 'Float' or cap_setting_type == 'Int':
                    cap_setting_min = str(info['min'])
                    cap_setting_max = str(info['max'])
                    cap_setting['options'] = [cap_setting_min,cap_setting_max]
                elif cap_setting_type == 'Discrete':
                    cap_setting['options'] = info['options']
                cap_settings[cap_setting_name] = cap_setting
        # Add Resolution Cap Settting
        try:
            [success,resolutions,encoder_cfg] = self.driver.getAvailableResolutions()
            nepi_msg.publishMsgInfo(self," " + "Driver returned resolution options: " + str(resolutions))
            if success:
                cap_setting = dict()
                cap_setting['name'] = 'Resolution'
                cap_setting['type'] = 'Discrete'
                options = []
                for res_dict in resolutions:
                    width = str(res_dict['Width'])
                    height = str(res_dict['Height'])
                    cap_setting_option = (width + ":" + height)
                    options.append(cap_setting_option)
                cap_setting['options'] = options
                cap_settings['Resolution'] = cap_setting
        except Exception as e:
            nepi_msg.publishMsgWarn(self," " + "Driver returned invalid resolution options: " + str(e))
        # Add Framerate Cap cap_setting
        try:
            [success,framerates,encoder_cfg] = self.driver.getFramerateRange()
            nepi_msg.publishMsgInfo(self," " + "Driver returned framerate options: " + str(framerates))
            if success:
                cap_setting = dict()
                cap_setting['name'] = 'Framerate'
                cap_setting['type'] = 'Int'
                options = [str(framerates['Min']),str(framerates['Max'])]
                cap_setting['options'] = options
                cap_settings['Framerate'] = cap_setting
        except Exception as e:
            nepi_msg.publishMsgWarn(self," " + "Driver returned invalid framerate options: " + str(e))
        return cap_settings


    def getFactorySettings(self):
        settings = self.getSettings()
        #Apply factory setting overides
        purge_list = []
        for setting_name in settings.keys():
            setting = settings[setting_name]
            if setting['value'] == 'Not Supported':
                purge_list.append(setting_name)
        for setting_name in purge_list:
            del setting[setting_name]
        for setting_name in settings.keys():
            if setting_name in self.FACTORY_SETTINGS_OVERRIDES.keys():
                settings[setting_name]['value'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
        return settings


    def getSettings(self):
        settings = dict()
        controls_dict = self.driver.getCameraControls()
        for setting_name in controls_dict.keys():
            if 'type' in controls_dict[setting_name]:
                setting = dict()
                setting['name'] = setting_name
                info = controls_dict[setting_name]
                setting_type = info['type']
                if setting_type == 'int':
                    setting_type = 'Int'
                elif setting_type == 'float':
                    setting_type = 'Float'
                elif setting_type == 'bool':
                    setting_type = 'Bool'
                elif setting_type == 'discrete':
                    setting_type = 'Discrete'
                setting['type'] = setting_type
                setting['value'] = str(info['value'])
                settings[setting_name] = setting
            elif 'value' in controls_dict[setting_name]:
                setting_value = controls_dict[setting_name]['value']
                if setting_value != "Not Supported":
                    setting['value'] = setting_value
                    setting['type'] = "String"
                    settings[setting_name] = setting
        # Add Resolution Cap Settting
        [success,res_dict] = self.driver.getResolution()
        #nepi_msg.publishMsgInfo(self,str(res_dict))
        setting = dict()
        setting['name'] = 'Resolution'
        setting['type'] = 'Discrete'
        width = str(res_dict['Width'])
        height = str(res_dict['Height'])
        setting_value = (width + ":" + height)
        setting['value'] = setting_value
        settings['Resolution'] = setting

        # Add Framerate Cap Setting
        [success,framerate] = self.driver.getFramerate()
        setting = dict()
        setting['name'] = 'Framerate'
        setting['type'] = 'Int'
        setting_value = (str(round(framerate,2)))
        setting['value'] = setting_value
        settings['Framerate'] = setting
        self.current_fps = framerate
        return settings


    def settingUpdateFunction(self,setting):
        success = False
        setting_str = str(setting)
        ret = nepi_settings.get_data_from_setting(setting)
        [setting_name, setting_type, data] = ret
        if data is not None:
            setting_data = data
            found_setting = False
            for cap_setting in self.cap_settings.keys():
                if setting_name in cap_setting:
                    found_setting = True
                    if setting_name != "Resolution" and setting_name != "Framerate":
                        success, msg = self.driver.setCameraControl(setting_name,setting_data)
                        if success:
                            msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)
                    elif setting_name == "Resolution":
                        data_split = data.split(":")
                        try:
                            width = int(data_split[0])
                            height = int(data_split[1])
                        except Exception as e:
                            nepi_msg.publishMsgInfo(self,"Resoluton setting: " + data + " could not be parsed to int " + str(e)) 
                        try:
                            res_dict = {'Width': width, 'Height': height}
                            success, msg = self.driver.setResolution(res_dict)
                        except Exception as e:
                            nepi_msg.publishMsgInfo(self,"setResolution function failed " + str(e))                            
                        break     
                    elif setting_name == "Framerate":
                        try:
                            framerate = int(data)
                            success, msg = self.driver.setFramerate(framerate)
                        except Exception as e:
                            nepi_msg.publishMsgInfo(self,"Framerate setting: " + data + " could not be parsed to float " + str(e))
                        break    
            if found_setting is False:
                msg = (self.node_name  + " Setting name" + setting_str + " is not supported")                   
        else:
            msg = (self.node_name  + " Setting data" + setting_str + " is None")
        return success, msg

    #**********************
    # Node driver functions

    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["HardwareId"] + "\n"
        nepi_msg.publishMsgInfo(self,dev_info_string)
    
        controls_dict = self.driver.getCameraControls()
        for key in controls_dict.keys():
            string = str(controls_dict[key])
            nepi_msg.publishMsgInfo(self,key + " " + string)
                
    
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

    def getFramerate(self):
        fr_mode = self.current_controls.get("framerate_mode")
        adj_fps =   nepi_img.adjust_framerate(self.current_fps,fr_mode)
        return adj_fps
    
    def getColorImg(self):
        # Check for control framerate adjustment
        last_time = self.cl_img_last_time
        current_time = nepi_ros.ros_time_now()
        controls_enabled = self.current_controls.get("controls_enable")
        fr_mode = self.current_controls.get("framerate_mode")
        need_data = False
        if fr_mode != 3 and last_time != None and self.idx_if is not None:
          adj_fr =   nepi_img.adjust_framerate(self.current_fps,fr_mode)
          fr_delay = float(1) / adj_fr
          timer =(current_time.to_sec() - last_time.to_sec())
          if timer > fr_delay:
            need_data = True
        else:
          need_data = True

        # Get and Process Data if Needed
        if need_data == False:
          return False, "Waiting for Timer", None, None, None  # Return None data
        else:
            self.cl_img_last_time = current_time

            encoding = "bgr8"
            self.img_uri_lock.acquire()
            # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
            ret, msg = self.driver.startImageAcquisition(uri_index = self.img_uri_index)
            if ret is False:
                self.img_uri_lock.release()
                return ret, msg, None, None, None
            self.color_image_acquisition_running = True
            timestamp = None
            start = time.time()
            cv2_img, timestamp, ret, msg = self.driver.getImage(uri_index = self.img_uri_index)
            stop = time.time()
            #print('GI: ', stop - start)
            if ret is False:
                self.img_uri_lock.release()
                return ret, msg, None, None, None
            if timestamp is not None:
                ros_timestamp = nepi_ros.ros_time_from_ros_stamp(timestamp)
            else:
                ros_timestamp = nepi_ros.ros_time_now()
            # Make a copy for the bw thread to use rather than grabbing a new cv2_img
            if self.bw_image_acquisition_running:
                self.cached_2d_color_frame = cv2_img
                self.cached_2d_color_frame_timestamp = ros_timestamp
            self.img_uri_lock.release()
            # Apply controls
            if self.current_controls.get("controls_enable") and cv2_img is not None and self.idx_if is not None:
                cv2_img = self.idx_if.applyIDXControls2Image(cv2_img,self.current_controls,self.current_fps)        
            return ret, msg, cv2_img, ros_timestamp, encoding
        
    def stopColorImg(self):
        self.img_uri_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        if self.bw_image_acquisition_running is False:
            ret,msg = self.driver.stopImageAcquisition(uri_index = self.img_uri_index)
        else:
            ret = True
            msg = "Success"
        self.color_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None
        self.img_uri_lock.release()
        return ret,msg
    
    def getBWImg(self):

        # Check for control framerate adjustment
        last_time = self.bw_img_last_time
        current_time = nepi_ros.ros_time_now()
        controls_enabled = self.current_controls.get("controls_enable")
        fr_mode = self.current_controls.get("framerate_mode")
        need_data = False
        if fr_mode != 3 and last_time != None and self.idx_if is not None:
          adj_fr =   nepi_img.adjust_framerate(self.current_fps,fr_mode)
          fr_delay = float(1) / adj_fr
          timer =(current_time.to_sec() - last_time.to_sec())
          if timer > fr_delay:
            need_data = True
        else:
          need_data = True
        # Get and Process Data if Needed
        if need_data == False:
          return False, "Waiting for Timer", None, None, None  # Return None data
        else:
            self.bw_img_last_time = current_time

            encoding = "mono8"
            self.img_uri_lock.acquire()
            # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
            ret, msg = self.driver.startImageAcquisition(uri_index = self.img_uri_index)
            if ret is False:
                self.img_uri_lock.release()
                return ret, msg, None, None, None
            self.bw_image_acquisition_running = True
            ros_timestamp = None
            # Only grab a frame if we don't already have a cached color frame... avoids cutting the update rate in half when
            # both image streams are running
            if self.color_image_acquisition_running is False or self.cached_2d_color_frame is None:
                cv2_img, timestamp, ret, msg = self.driver.getImage(uri_index = self.img_uri_index)
                if timestamp is not None:
                    ros_timestamp = nepi_ros.ros_time_from_ros_stamp(timestamp)
                else:
                    ros_timestamp = nepi_ros.ros_time_now()
            else:
                cv2_img = self.cached_2d_color_frame.copy()
                ros_timestamp = self.cached_2d_color_frame_timestamp
                self.cached_2d_color_frame = None # Clear it to avoid using it multiple times in the event that threads are running at different rates
                self.cached_2d_color_frame_timestamp = None
                ret = True
                msg = "Success: Reusing cached cv2_img"
            self.img_uri_lock.release()
            # Abort if there was some error or issue in acquiring the image
            if ret is False or cv2_img is None:
                return False, msg, None, None, None
            # Fix the channel count if necessary
            if cv2_img.ndim == 3:
                cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
            # Apply controls
            if self.current_controls.get("controls_enable") and cv2_img is not None and self.idx_if is not None:
                cv2_img = self.idx_if.applyIDXControls2Image(cv2_img,self.current_controls,self.current_fps)
            return ret, msg, cv2_img, ros_timestamp, encoding
    
    def stopBWImg(self):
        self.img_uri_lock.acquire()
        # Don't stop acquisition if the color image is still being requested
        if self.color_image_acquisition_running is False:
            ret,msg = self.driver.stopImageAcquisition(uri_index = self.img_uri_index)
        else:
            ret = True
            msg = "Success"
        self.bw_image_acquisition_running = False
        self.img_uri_lock.release()
        return ret, msg

if __name__ == '__main__':
	node = OnvifCamNode()

            


        

