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

import sys
import time
import math
import threading
import cv2

from nepi_sdk.device_if_idx import ROSIDXSensorIF

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_drv

PKG_NAME = 'IDX_GENICAM' # Use in display menus
FILE_TYPE = 'NODE'

TEST_DRV_DICT = {
'type': 'IDX',
'group_id': 'None',
'path': '/opt/nepi/ros/lib/nepi_drivers',
'usr_cfg_path': '/mnt/nepi_storage/user_cfg/ros',
'NODE_DICT': {
    'file_name': 'idx_genicam_node.py',
    'class_name': 'GenicamCamNode',
},
'DRIVER_DICT': {
    'file_name': 'idx_genicam_driver.py' ,
    'class_name':  'GenicamCamDriver'
},
'DEVICE_DICT': {'model':'0','serial_number': '1'},
}


class GenicamCamNode:
    FACTORY_SETTINGS_OVERRIDES = dict( BalanceWhiteAuto = 'Continuous',
                                      ColorCorrectionMode = 'Auto',
                                      ExposureAuto = 'Continuous',
                                      GainAuto = 'Continuous')

    #Factory Control Values 
    FACTORY_CONTROLS = dict( controls_enable = True,
    auto_adjust = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    resolution_mode = 3, # LOW, MED, HIGH, MAX
    framerate_mode = 2, # LOW, MED, HIGH, MAX
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
        self.drv_dict = nepi_ros.get_param(self,'~drv_dict',TEST_DRV_DICT) 
        #nepi_msg.publishMsgWarn(self,"Drv_Dict: " + str(self.drv_dict))
        self.driver_path = self.drv_dict['path']
        self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
        self.driver_module = self.drv_dict['DRIVER_DICT']['module_name']
        self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
        
        model = self.drv_dict['DEVICE_DICT']['model']
        serial_number = self.drv_dict['DEVICE_DICT']['serial_number']
        # import driver class fromn driver module
        nepi_msg.publishMsgInfo(self,model)
        nepi_msg.publishMsgInfo(self,serial_number)
        nepi_msg.publishMsgInfo(self,"Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drv.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        
        if success:
            try:
                self.driver = self.driver_class(model=model, serial_number=serial_number)
            except Exception as e:
                # Only log the error every 30 seconds -- don't want to fill up log in the case that the camera simply isn't attached.
                nepi_msg.publishMsgErr(self,"Failed to instantiate driver - " + str(e) + ")")
                sys.exit(-1)
        ################################################
        genicam_cfg_file_mappings = nepi_ros.get_param(self,"~genicam_mappings", {})
        if not self.driver.isConnected():
           nepi_msg.publishMsgErr(self,"Failed to connect to camera device")

        nepi_msg.publishMsgInfo(self,f"{self.node_name}: ... Connected!")

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

        # IDX Remappings: Not necessary since we have a separate mechanism for genicam parameter assignment

        self.img_lock = threading.Lock()
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
        '''
        nepi_msg.publishMsgInfo(self,"CAPS SETTINGS")
        for setting_name in self.cap_settings.keys():
            nepi_msg.publishMsgInfo(self,self.cap_settings[setting_name])
        '''

        self.factory_settings = self.getFactorySettings()
        '''
        nepi_msg.publishMsgInfo(self,"FACTORY SETTINGS")
        for setting_name in self.factory_settings.keys():
            nepi_msg.publishMsgInfo(self,self.factory_settings[setting_name])
        '''

          
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
                                     settingUpdateFunction= self.settingUpdateFunction,
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
        nepi_msg.publishMsgInfo(self,"... IDX interface running")
        self.logDeviceInfo()
        self.idx_if.updateFromParamServer()


        ## Initiation Complete
        nepi_msg.publishMsgInfo(self,"Initialization Complete")
        # Now start the node
        nepi_ros.spin()


    #**********************
    # Sensor setting functions

    def getCapSettings(self):
        cap_settings = dict()
        controls_dict = self.driver.getCameraControls()
        for cap_setting_name in controls_dict.keys():
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
            elif cap_setting_type == 'enum':
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
            [success,available_resolutions] = self.driver.getCurrentFormatAvailableResolutions()
            if success == True:
                nepi_msg.publishMsgInfo(self,"Driver returned Resolution options: " + str(available_resolutions))
                cap_setting = dict()
                cap_setting['name'] = 'Resolution'
                cap_setting['type'] = 'Discrete'
                options = []
                for res_dict in available_resolutions:
                    width = str(res_dict['width'])
                    height = str(res_dict['height'])
                    cap_setting_option = (width + ":" + height)
                    options.append(cap_setting_option)
                cap_setting['options'] = options
                cap_settings['Resolution'] = cap_setting
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Driver returned invalid resolution options: " + str(e))
        # Add Framerate Cap cap_setting
        try:
            [success,framerates] = self.driver.getCurrentResolutionAvailableFramerates()
            if success == True:
                nepi_msg.publishMsgInfo(self,"Driver returned framerate options: " + str(framerates))
                cap_setting = dict()
                cap_setting['name'] = 'Framerate'
                cap_setting['type'] = 'Float'
                options = []
                if len(framerates) > 0:
                    for rate in framerates:
                        cap_setting_option = (str(round(rate,2)))
                        options.append(cap_setting_option)
                    cap_setting['options'] = options
                    cap_settings['Framerate'] = cap_setting
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Driver returned invalid framerate options: " + str(e))
        return cap_settings

    def getFactorySettings(self):
        settings = self.getSettings()
        #Apply factory setting overides
        for setting_name in settings.keys():
            if setting_name in self.FACTORY_SETTINGS_OVERRIDES.keys():
                settings[setting_name]['value'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
            '''
            if setting_name == "Resolution":
                options = self.cap_settings[setting_name]['options']
                cur_val = settings[setting_name]['value']
                nepi_msg.publishMsgWarn(self,"Checking that " + cur_val + " in options " + str(options))
                if cur_val not in options:
                    settings[setting_name]['value'] = options[-1]
            '''
        return settings
            

    def getSettings(self):
        settings = dict()
        controls_dict = self.driver.getCameraControls()
        for setting_name in controls_dict.keys():
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
            elif setting_type == 'enum':
                setting_type = 'Discrete'
            setting['type'] = setting_type
            setting['value'] = str(info['value'])
            settings[setting_name] = setting
        # Add Resolution Cap Settting
        try:
            [success,res_dict] = self.driver.getCurrentResolution()
            setting = dict()
            setting['name'] = 'Resolution'
            setting['type'] = 'Discrete'
            width = str(res_dict['width'])
            height = str(res_dict['height'])
            setting_value = width + ":" + height
            setting['value'] = setting_value
            settings['Resolution'] = setting
            #nepi_msg.publishMsgWarn(self,"Driver returned res setting: " + str(settings['Resolution']) )
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to get current resolution: " + str(e))
        # Add Framerate Cap Setting
        try:
            [success,framerate] = self.driver.getFramerate()
            setting = dict()
            setting['name'] = 'Framerate'
            setting['type'] = 'Float'
            setting_value = (str(round(framerate,2)))
            setting['value'] = setting_value
            settings['Framerate'] = setting
            #nepi_msg.publishMsgWarn(self,"Driver returned fr setting: " + str(settings['Framerate']) )
            self.current_fps = framerate
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to get current framerate: " + str(e))
        return settings


    def settingUpdateFunction(self,setting):
        success = False
        setting_str = str(setting)
        [setting_name, setting_type, data] = nepi_settings.get_data_from_setting(setting)
        #nepi_msg.publishMsgWarn(self,"Updating Setting: " + setting_str)
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
                        data = data.split(":")
                        try:
                            width = int(data[0])
                            height = int(data[1])
                            res_dict = {'width': width, 'height': height}
                            nepi_msg.publishMsgWarn(self,"Updating Res with Res Dict: " + str(res_dict))
                            success, msg = self.driver.setResolution(res_dict)
                        except Exception as e:
                            nepi_msg.publishMsgWarn(self,"Resoluton setting: " + data + " could not be parsed to float " + str(e))                            
                        break     
                    elif setting_name == "Framerate":
                        try:
                            framerate = float(data)
                            success, msg = self.driver.setFramerate(framerate)
                        except Exception as e:
                            nepi_msg.publishMsgWarn(self,"Framerate setting: " + data + " could not be parsed to float " + str(e))
                        break    
            if found_setting is False:
                msg = (self.node_name  + " Setting name" + setting_str + " is not supported")                   
        else:
            msg = (self.node_name  + " Setting data" + setting_str + " is None")
        return success, msg

    #**********************
    # Node driver functions

    def logDeviceInfo(self):
        device_info_str = f"{self.node_name} info:\n"\
                + f"\tModel: {self.driver.model}\n"\
                + f"\tS/N: {self.driver.serial_number}\n"

        controls_dict = self.driver.getCameraControls()
        #for key in controls_dict.keys():
            #string = str(controls_dict[key])
            #nepi_msg.publishMsgInfo(self,key + string)
            
        _, fmt = self.driver.getCurrentFormat()
        device_info_str += f"\tCamera Output Format: {fmt}\n"

        _, res_dict = self.driver.getCurrentResolution()
        device_info_str += "\tCurrent Resolution: " + f'{res_dict["width"]}x{res_dict["height"]}' + "\n"

        if (self.driver.hasAdjustableResolution()):
            _, available_resolutions = self.driver.getCurrentFormatAvailableResolutions()
            device_info_str += "\tAvailable Resolutions:\n"
            for res in available_resolutions:
                device_info_str += "\t\t" + f'{res["width"]}x{res["height"]}' + "\n"

        if (self.driver.hasAdjustableFramerate()):
            _, available_framerates = self.driver.getCurrentResolutionAvailableFramerates()
            device_info_str += "\t" + f'Available Framerates (current resolution): {available_framerates}' + "\n"
        #nepi_msg.publishMsgInfo(self,device_info_str)

    
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
        current_time = nepi_ros.get_rostime()
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
            self.img_lock.acquire()
            # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
            ret, msg = self.driver.startImageAcquisition()
            if ret is False:
                self.img_lock.release()
                return ret, msg, None, None, None

            self.color_image_acquisition_running = True
            timestamp = None
            start = time.time()
            cv2_img, timestamp, ret, msg = self.driver.getImage()
            stop = time.time()
            #print('GI: ', stop - start)
            if ret is False:
                self.img_lock.release()
                return ret, msg, None, None, None
            if timestamp is not None:
                ros_timestamp = nepi_ros.time_from_timestamp(timestamp)
            else:
                ros_timestamp = nepi_ros.time_now()
            # Make a copy for the bw thread to use rather than grabbing a new cv2_img
            if self.bw_image_acquisition_running:
                self.cached_2d_color_frame = cv2_img
                self.cached_2d_color_frame_timestamp = ros_timestamp
            self.img_lock.release()
            #nepi_msg.publishMsgInfo(self,"About to run cl_img controls with controls: " + str(self.current_controls))
            # Apply controls
            if self.current_controls.get("controls_enable") and cv2_img is not None and self.idx_if is not None:
                cv2_img = self.idx_if.applyIDXControls2Image(cv2_img,self.current_controls,self.current_fps)
            return ret, msg, cv2_img, ros_timestamp, encoding
        
    def stopColorImg(self):
        self.img_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        if self.bw_image_acquisition_running is False:
            ret,msg = self.driver.stopImageAcquisition()
        else:
            ret = True
            msg = "Success"
        self.color_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None
        self.img_lock.release()
        return ret,msg
    
    def getBWImg(self):

        # Check for control framerate adjustment
        last_time = self.bw_img_last_time
        current_time = nepi_ros.get_rostime()
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
            self.img_lock.acquire()
            # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
            ret, msg = self.driver.startImageAcquisition()
            if ret is False:
                self.img_lock.release()
                return ret, msg, None, None
            self.bw_image_acquisition_running = True
            ros_timestamp = None
            # Only grab a frame if we don't already have a cached color frame... avoids cutting the update rate in half when
            # both image streams are running
            if self.color_image_acquisition_running is False or self.cached_2d_color_frame is None:
                #nepi_msg.publishMsgWarn(self,"Debugging: getBWImg acquiring")
                cv2_img, timestamp, ret, msg = self.driver.getImage()
                if timestamp is not None:
                    ros_timestamp = nepi_ros.time_from_timestamp(timestamp)
                else:
                    ros_timestamp = nepi_ros.time_now()
            else:
                #nepi_msg.publishMsgWarn(self,"Debugging: getBWImg reusing")
                cv2_img = self.cached_2d_color_frame.copy()
                ros_timestamp = self.cached_2d_color_frame_timestamp
                self.cached_2d_color_frame = None # Clear it to avoid using it multiple times in the event that threads are running at different rates
                self.cached_2d_color_frame_timestamp = None
                ret = True
                msg = "Success: Reusing cached cv2_img"
            self.img_lock.release()
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
        self.img_lock.acquire()
        # Don't stop acquisition if the color image is still being requested
        if self.color_image_acquisition_running is False:
            ret,msg = self.driver.stopImageAcquisition()
        else:
            ret = True
            msg = "Success"
        self.bw_image_acquisition_running = False
        self.img_lock.release()
        return ret, msg

if __name__ == '__main__':
    node = GenicamCamNode()
