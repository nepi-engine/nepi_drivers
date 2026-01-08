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
from nepi_sdk import nepi_img
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_drvs

from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'IDX_GENICAM' # Use in display menus
FILE_TYPE = 'NODE'

class GenicamCamNode:
    FACTORY_SETTINGS_OVERRIDES = dict( BalanceWhiteAuto = 'Continuous',
                                      ColorCorrectionMode = 'Auto',
                                      ExposureAuto = 'Continuous',
                                      GainAuto = 'Continuous')

 

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

    idx_if = None

    current_fps = 20
    cl_img_last_time = None

    max_framerate = 100
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
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict()) 
        self.msg_if.pub_warn("Drv_Dict: " + str(self.drv_dict))
        self.driver_path = self.drv_dict['path']
        self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
        self.driver_module = self.driver_file.split(".")[0]
        self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
        self.data_products = self.drv_dict['DEVICE_DICT']['data_products']
        model = self.drv_dict['DEVICE_DICT']['model']
        serial_number = self.drv_dict['DEVICE_DICT']['serial_number']
        # import driver class fromn driver module
        self.msg_if.pub_info(model)
        self.msg_if.pub_info(serial_number)
        self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        
        if success:
            try:
                self.driver = self.driver_class(model=model, serial_number=serial_number)
            except Exception as e:
                # Only log the error every 30 seconds -- don't want to fill up log in the case that the camera simply isn't attached.
                self.msg_if.pub_warn("Failed to instantiate driver - " + str(e) + ")")
                sys.exit(-1)
                return
                
        ################################################
        genicam_cfg_file_mappings = nepi_sdk.get_param("~genicam_mappings", {})
        if not self.driver.isConnected():
           self.msg_if.pub_warn("Failed to connect to camera device")

        self.msg_if.pub_info(f"{self.node_name}: ... Connected!")

        self.img_lock = threading.Lock()
        self.color_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None

        # Initialize controls
        self.factory_controls = self.FACTORY_CONTROLS
        self.current_controls = self.factory_controls # Updateded during initialization
        self.current_fps = self.DEFAULT_CURRENT_FPS # Should be updateded when settings read

        # Initialize settings
        self.cap_settings = self.getCapSettings()
        '''
        self.msg_if.pub_info("CAPS SETTINGS")
        for setting_name in self.cap_settings.keys():
            self.msg_if.pub_info(self.cap_settings[setting_name])
        '''

        self.factory_settings = self.getFactorySettings()
        '''
        self.msg_if.pub_info("FACTORY SETTINGS")
        for setting_name in self.factory_settings.keys():
            self.msg_if.pub_info(self.factory_settings[setting_name])
        '''

          
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
                                    setMaxFramerate =self.setMaxFramerate, 
                                    getFramerate = self.driver.getFramerate,
                                    getColorImage = self.getColorImg, 
                                    stopColorImageAcquisition = self.stopColorImg,
                                    data_products = self.data_products)
        self.msg_if.pub_info("... IDX interface running")
        self.logDeviceInfo()


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
                self.msg_if.pub_info("Driver returned Resolution options: " + str(available_resolutions))
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
            self.msg_if.pub_warn("Driver returned invalid resolution options: " + str(e))
        # Add Framerate Cap cap_setting
        try:
            [success,framerates] = self.driver.getCurrentResolutionAvailableFramerates()
            if success == True:
                self.msg_if.pub_info("Driver returned framerate options: " + str(framerates))
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
            self.msg_if.pub_warn("Driver returned invalid framerate options: " + str(e))
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
                self.msg_if.pub_warn("Checking that " + cur_val + " in options " + str(options))
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
            #self.msg_if.pub_warn("Driver returned res setting: " + str(settings['Resolution']) )
        except Exception as e:
            self.msg_if.pub_warn("Failed to get current resolution: " + str(e))
        # Add Framerate Cap Setting
        try:
            [success,framerate] = self.driver.getFramerate()
            setting = dict()
            setting['name'] = 'Framerate'
            setting['type'] = 'Float'
            setting_value = (str(round(framerate,2)))
            setting['value'] = setting_value
            settings['Framerate'] = setting
            #self.msg_if.pub_warn("Driver returned fr setting: " + str(settings['Framerate']) )
            self.current_fps = framerate
        except Exception as e:
            self.msg_if.pub_warn("Failed to get current framerate: " + str(e))
        return settings


    def settingUpdateFunction(self,setting):
        success = False
        setting_str = str(setting)
        [setting_name, setting_type, data] = nepi_settings.get_data_from_setting(setting)
        #self.msg_if.pub_warn("Updating Setting: " + setting_str)
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
                            [success,framerate] = self.driver.getFramerate()
                            width = int(data[0])
                            height = int(data[1])
                            res_dict = {'width': width, 'height': height}
                            self.msg_if.pub_warn("Updating Res with Res Dict: " + str(res_dict))
                            success, msg = self.driver.setResolution(res_dict)
                            # reset framerate if needed
                            if 'framerate' in self.cap_settings.keys():
                                nepi_sdk.sleep(1)
                                try:
                                    framerate = float(framerate)
                                    success, msg = self.driver.setFramerate(framerate)
                                    self.msg_if.pub_warn("Updated Framerate: " + str(framerate))
                                except Exception as e:
                                    self.msg_if.pub_warn("Framerate setting: " + data + " could not be parsed to float " + str(e))

                        except Exception as e:
                            self.msg_if.pub_warn("Resoluton setting: " + data + " could not be parsed to float " + str(e))     
                        break     
                    elif setting_name == "Framerate":
                        try:
                            framerate = float(data)
                            success, msg = self.driver.setFramerate(framerate)
                        except Exception as e:
                            self.msg_if.pub_warn("Framerate setting: " + data + " could not be parsed to float " + str(e))
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
            #self.msg_if.pub_info(key + string)
            
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
        #self.msg_if.pub_info(device_info_str)

        
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


    def getColorImg(self):
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
        
    def stopColorImg(self):
        self.img_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        ret,msg = self.driver.stopColorImageAcquisition()
        self.color_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None
        self.img_lock.release()
        return ret,msg
    

if __name__ == '__main__':
    node = GenicamCamNode()
