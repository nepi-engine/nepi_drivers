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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_img
from nepi_sdk import nepi_settings

from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'IDX_V4L2' # Use in display menus
FILE_TYPE = 'NODE'

TEST_DRV_DICT = {
'type': 'IDX',
'group_id': 'None',
'path': '/opt/nepi/nepi_engine/lib/nepi_drivers',
'NODE_DICT': {
    'file_name': 'idx_v4l2_node.py',
    'class_name': 'V4l2CamNode',
},
'DRIVER_DICT': {
    'file_name': 'idx_v4l2_driver.py' ,
    'class_name':  'V4l2CamDriver'
},
'DEVICE_DICT': {'device_path': '/dev/video0'},
}

class V4l2CamNode:

    FACTORY_SETTINGS_OVERRIDES = dict( white_balance_temperature_auto = "True",
                                    focus_auto = "True" )
                                    
    DEFAULT_DEVICE_PATH = '/dev/video0'


    #Factory Control Values 
    FACTORY_CONTROLS = dict( 
    frame_id = 'sensor_frame' 
    )

 
    DEFAULT_CURRENT_FPS = 20 # Will be update later with actual

    device_info_dict = dict(node_name = "",
                            device_name = "",
                            identifier = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")


    # Create threading locks, controls, and status
    img_lock = threading.Lock()
    color_image_acquisition_running = False
    cached_2d_color_image = None
    cached_2d_color_image_timestamp = None
    set_framerate = 0
    idx_if = None


    current_fps = 20
    cl_img_last_time = None


    framerate_ratio = 1.0


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
        #self.msg_if.pub_warn("Drv_Dict: " + str(self.drv_dict))
        self.driver_path = self.drv_dict['path']
        self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
        self.driver_module = self.driver_file.split('.')[0]
        self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
        
        self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
        if self.device_path == "":
            self.device_path = self.DEFAULT_DEVICE_PATH
        # import driver class fromn driver module
        self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        if success:
            try:
                self.driver = self.driver_class(self.device_path)
            except Exception as e:
                # Only log the error every 30 seconds -- don't want to fill up log in the case that the camera simply isn't attached.
                self.msg_if.pub_warn("Failed to instantiate driver " + str(e) )
                sys.exit(-1)
        ################################################
        # Start node initialization
        

        if not self.driver.isConnected():
           self.msg_if.pub_warn("Failed to connect to camera device")
            
        self.msg_if.pub_info("... Connected!")


        # Initialize controls
        self.factory_controls = self.FACTORY_CONTROLS
        self.current_controls = self.factory_controls # Updateded during initialization
        self.current_fps = self.DEFAULT_CURRENT_FPS # Should be updateded when settings read


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
                                    data_source_description = 'camera',
                                    data_ref_description = 'camera_lense',
                                    capSettings = self.cap_settings,
                                    factorySettings = self.factory_settings,
                                    settingUpdateFunction= self.settingUpdateFunction,
                                    getSettingsFunction=self.getSettings,
                                    factoryControls = self.factory_controls,
                                    setFramerateRatio =self.setFramerateRatio, 
                                    getFramerate = self.getFramerate,
                                    getColorImage = self.getColorImg, 
                                    stopColorImageAcquisition = self.stopColorImg,
                                    data_products = ['color_image'])
                                    
        self.msg_if.pub_info(" " + " ... IDX interface running")

        # Update available IDX callbacks based on capabilities that the driver reports
        self.logDeviceInfo()
        time.sleep(1)
        self.getColorImg()
        # Now that all camera start-up stuff is processed, we can update the camera from the parameters that have been established
        time.sleep(1)

        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Now start the node
        nepi_sdk.spin()

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
            elif cap_setting_type == 'menu':
                cap_setting_type = 'Menu'
            cap_setting['type'] = cap_setting_type
            if cap_setting_type == 'Float' or cap_setting_type == 'Int':
                cap_setting_min = str(info['min'])
                cap_setting_max = str(info['max'])
                cap_setting['options'] = [cap_setting_min,cap_setting_max]
            elif  cap_setting_type == 'Menu':
                legend = info['legend']
                options = []
                for option_name in legend.keys():
                    option_ind = legend[option_name]
                    options.append(option_name + ":" + str(option_ind))
                cap_setting['options'] = options
            cap_settings[cap_setting_name] = cap_setting    
        # Add Resolution Cap Settting
        try:
            [success,available_resolutions] = self.driver.getCurrentFormatAvailableResolutions()
            cap_setting = dict()
            cap_setting['type'] = 'Discrete'
            options = []
            for res_dict in available_resolutions:
                width = str(res_dict['width'])
                height = str(res_dict['height'])
                cap_setting_option = (width + ":" + height)
                options.append(cap_setting_option)
            cap_setting['options'] = options
            cap_setting['name'] = 'resolution'
            cap_settings['resolution'] = cap_setting
        except Exception as e:
            self.msg_if.pub_info(" " + "Driver returned invalid resolution options: " + str(available_resolutions))
        # Add Framerate Cap cap_setting
        try:
            [success,framerates] = self.driver.getCurrentResolutionAvailableFramerates()
            self.msg_if.pub_info(" " + "Driver returned framerate options: " + str(framerates))
            cap_setting = dict()
            cap_setting['type'] = 'Descrete'
            options = []
            if len(framerates) > 0:
                for rate in framerates:
                    cap_setting_option = (str(round(rate,2)))
                    options.append(cap_setting_option)
                cap_setting['options'] = options
                cap_setting['name'] = 'framerate'
                cap_settings['framerate'] = cap_setting
        except Exception as e:
            self.msg_if.pub_info(" " + "Driver returned invalid framerate options: " + str(framerates))
        return cap_settings



    def getFactorySettings(self):
        settings = dict()
        controls_dict = self.driver.getCameraControls()
        for setting_name in controls_dict.keys():
            info = controls_dict[setting_name]
            setting_type = info['type']
            if setting_type == 'int':
                setting_type = 'Int'
            elif setting_type == 'float':
                setting_type = 'Float'
            elif setting_type == 'bool':
                setting_type = 'Bool'
            elif setting_type == 'menu':
                setting_type = 'Menu'
            # Create Menu Setting
            if setting_type == 'Menu':
                setting_default = 'None'
                setting_value = info['default']
                legend = info['legend']
                for menu_name in legend.keys():
                    menu_value = legend[menu_name]
                    if menu_value == setting_value:
                        setting_default = (menu_name + ":" + str(setting_value))
            else:
                setting_default = str(info['default'])
            setting = dict()
            setting['type'] = setting_type
            setting['name'] =setting_name
            setting['value'] =setting_default
            settings[setting_name] = setting

        #Apply factory setting overides
        for setting_name in settings.keys():
            if setting_name in self.FACTORY_SETTINGS_OVERRIDES.keys():
                settings[setting_name]['value'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
        return settings

            



    def getSettings(self):
        settings = dict()
        controls_dict = self.driver.getCameraControls()
        #for key in controls_dict.keys():
        #string = str(controls_dict[key])
        #self.msg_if.pub_info(key + " " + string)
        for setting_name in controls_dict.keys():
            info = controls_dict[setting_name]
            setting_type = info['type']
            if setting_type == 'int':
                setting_type = 'Int'
            elif setting_type == 'float':
                setting_type = 'Float'
            elif setting_type == 'bool':
                setting_type = 'Bool'
            elif setting_type == 'menu':
                setting_type = 'Menu'
            # Create Current Setting
            if setting_type == 'Menu':
                setting_current = 'None'
                setting_value = info['value']
                legend = info['legend']
                for menu_name in legend.keys():
                    menu_value = legend[menu_name]
                    if menu_value == setting_value:
                        setting_current = (menu_name + ":" + str(setting_value))
            else:
                setting_current = str(info['value'])
            setting = dict()
            setting['type'] = setting_type
            setting['name'] =setting_name
            setting['value'] =setting_current
            settings[setting_name] = setting
        # Resolution
        [success,res_dict] = self.driver.getCurrentResolution()
        setting = dict()
        setting['type'] = 'Discrete'
        width = str(res_dict['width'])
        height = str(res_dict['height'])
        setting_value = (width + ":" + height)
        setting['value'] = setting_value
        setting['name'] = 'resolution'
        settings['resolution'] = setting
        # Framerate
        [success,framerate] = self.driver.getFramerate() 
        setting = dict()
        setting['type'] = 'Discrete'
        setting['value'] = str(framerate)
        setting['name'] = 'framerate'
        settings['framerate'] = setting
        self.current_fps = framerate
        return settings



    def settingUpdateFunction(self,setting):
        success = False
        setting_str = str(setting)
        [setting_name, setting_type, data] = nepi_settings.get_data_from_setting(setting)
        if data is not None:
            setting_data = data
            found_setting = False
            for cap_setting in self.cap_settings.keys():
                if setting_name in cap_setting:
                    found_setting = True
                    if setting_name != "resolution" and setting_name != "framerate":
                        success, msg = self.driver.setCameraControl(setting_name,setting_data)
                        if success:
                            msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)
                    elif setting_name == "resolution":
                        data = data.split(":")
                        try:
                            width = int(data[0])
                            height = int(data[1])
                            res_dict = {'width': width, 'height': height}
                            success, msg = self.driver.setResolution(res_dict)
                        except Exception as e:
                            self.msg_if.pub_info("Resoluton setting: " + data + " could not be parsed to float " + str(e))                            
                        break     
                    elif setting_name == "framerate":
                        try:
                            framerate = float(data)
                            success, msg = self.driver.setFramerate(framerate)
                        except Exception as e:
                            self.msg_if.pub_info("Framerate setting: " + data + " could not be parsed to float " + str(e))
                        break    
            if found_setting is False:
                msg = (self.node_name  + " Setting name" + setting_str + " is not supported")                   
        else:
            msg = (self.node_name  + " Setting data" + setting_str + " is None")
        return success, msg


    #**********************
    # IDX driver functions

    def logDeviceInfo(self):
        device_info_str = self.node_name + " info:\n"
        device_info_str += "\tDevice Path: " + self.driver.device_path + "\n"

        controls_dict = self.driver.getCameraControls()
        for key in controls_dict.keys():
            string = str(controls_dict[key])
            self.msg_if.pub_info(key + " " + string)
        



        _, format = self.driver.getCurrentFormat()
        device_info_str += "\tCamera Output Format: " + format + "\n"

        _, resolution_dict = self.driver.getCurrentResolution()
        device_info_str += "\tCurrent Resolution: " + str(resolution_dict['width']) + 'x' + str(resolution_dict['height']) + "\n"

        if (self.driver.hasAdjustableResolution()):
            _, available_resolutions = self.driver.getCurrentFormatAvailableResolutions()
            device_info_str += "\tAvailable Resolutions:\n"
            for res in available_resolutions:
                device_info_str += "\t\t" + str(res["width"]) + 'x' + str(res["height"]) + "\n"

        if (self.driver.hasAdjustableFramerate()):
            _, available_framerates = self.driver.getCurrentResolutionAvailableFramerates()
            device_info_str += "\tAvailable Framerates (current resolution): " + str(available_framerates) + "\n"
        
        self.msg_if.pub_info(device_info_str)

        
    def setFramerateRatio(self, ratio):
        if ratio < 0.1:
            ratio = 0.1
        if ratio > .99:
            ratio = 1.0
        self.framerate_ratio = ratio
        #self.msg_if.pub_warn("Updated framerate: " + str(self.framerate_ratio))
        status = True
        err_str = ""
        return status, err_str

    def getFramerate(self):
        adj_fps =   nepi_img.adjust_framerate_ratio(self.current_fps,self.framerate_ratio)
        return adj_fps

    def setDriverCameraControl(self, control_name, value):
        return self.driver.setScaledCameraControl(control_name, value)
    
    # Good base class candidate - Shared with ONVIF
    def getColorImg(self):
        # Check for control framerate adjustment
        last_time = self.cl_img_last_time
        current_time = nepi_utils.get_time()
        
        need_data = False
        if last_time != None and self.idx_if is not None:
          adj_fr =   nepi_img.adjust_framerate_ratio(self.current_fps,self.framerate_ratio)
          #self.msg_if.pub_warn("Current HW Framerate: " + str(self.current_fps))
          #self.msg_if.pub_warn("Using Max Framerate: " + str(adj_fr))
          fr_delay = float(1) / adj_fr
          timer = current_time - last_time
          if timer > fr_delay or self.framerate_ratio > 0.95:
            need_data = True
        else:
          need_data = True

        #need_data = True
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
            if timestamp is None:
                timestamp = nepi_utils.get_time()
            self.img_lock.release()
            return ret, msg, cv2_img, timestamp, encoding
    
    # Good base class candidate - Shared with ONVIF
    def stopColorImg(self):
        self.img_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        ret,msg = self.driver.stopImageAcquisition()
        self.color_image_acquisition_running = False
        self.cached_2d_color_image = None
        self.cached_2d_color_image_timestamp = None
        self.img_lock.release()
        return ret,msg

        
if __name__ == '__main__':
    node = V4l2CamNode()
