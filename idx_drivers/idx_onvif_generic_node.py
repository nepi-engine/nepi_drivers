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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_settings

from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'IDX_ONVIF_GENERIC' # Use in display menus
FILE_TYPE = 'NODE'


class OnvifCamNode:

    FACTORY_SETTINGS_OVERRIDES = dict(WhiteBalance_Mode = "AUTO",
                                      Exposure_Mode = "AUTO",
                                      Resolution = "2560:1440" )

 
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
        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict') # Crash if not provide
        except Exception as e:
            nepi_sdk.signal_shutdown("Failed to read drv_dict from param server for node " + self.node_name + " with exception: " + str(e))
        self.driver_path = self.drv_dict['path']
        self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
        self.driver_module = self.driver_file.split('.')[0]
        self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']

        # Require the camera connection parameters to have been set
        if not nepi_sdk.has_param('~credentials/username'):
            self.msg_if.pub_warn("Missing credentials/username parameter... cannot start")
            return
        if not nepi_sdk.has_param('~credentials/password'):
            self.msg_if.pub_warn("Missing credentials/password parameter... cannot start")
            return
        if not nepi_sdk.has_param('~network/host'):
            self.msg_if.pub_warn("Missing network/host parameter... cannot start")
            return
                
        username = str(nepi_sdk.get_param('~credentials/username'))
        password = str(nepi_sdk.get_param('~credentials/password'))
        host = str(nepi_sdk.get_param('~network/host'))
        
        # Allow a default for the port, since it is part of onvif spec.
        onvif_port = nepi_sdk.get_param('~network/port', 80)
        nepi_sdk.set_param('~/network/port', onvif_port)


        self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        driver_constructed = False
        if success:
            attempts = 0
            while not nepi_sdk.is_shutdown() and driver_constructed == False and attempts < 5 and not nepi_sdk.is_shutdown():
                try:
                    self.driver = self.driver_class(username, password, host, onvif_port)
                    driver_constructed = True
                    self.msg_if.pub_info("ONVIF_NODE: Driver constructed")
                except Exception as e:
                    self.msg_if.pub_info("ONVIF_NODE: Failed to construct driver " + self.driver_module + " with exception: " + str(e))
                    time.sleep(1)
                attempts += 1 
        if driver_constructed == False:
            nepi_sdk.signal_shutdown("Shutting down Onvif node " + self.node_name + ", unable to connect to driver")
        else:
            ################################################
            self.msg_if.pub_info("... Connected!")
            self.dev_info = self.driver.getDeviceInfo()
            self.logDeviceInfo()        


            # Establish the URI indices (from ONVIF "Profiles") for the two image streams.
            # If these aren't the same, encoder param adjustments (resolution and framerate)
            # will only affect the first one.... so
            # TODO: Consider a scheme for adjusting parameters for separate streams independently
            # or in lock-step. Not sure if the uri_index and encoder_index have the same meaning
            self.img_uri_index = nepi_sdk.get_param('~/img_uri_index', 0)
            nepi_sdk.set_param('~/img_uri_index', self.img_uri_index)

            # Create threading locks for each URI index (currently just 1) to provide threadsafety
            self.img_uri_lock = threading.Lock()
            self.color_image_acquisition_running = False
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
            self.msg_if.pub_info(" ... IDX interface running")
            # Now that all camera start-up stuff is processed, we can update the camera from the parameters that have been established

            # Now start the node
            nepi_sdk.spin()


    #**********************
    # Sensor setting functions

    def getRtspUrl(self):
        onvif_port = str(nepi_sdk.get_param('~network/port', 80))
        onvif_address = str(nepi_sdk.get_param('~network/host',""))
        url = "http://" + onvif_address #+ ":" + onvif_port
        username = str(nepi_sdk.get_param('~credentials/username'))
        password = str(nepi_sdk.get_param('~credentials/password'))
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
            self.msg_if.pub_info(" " + "Driver returned resolution options: " + str(resolutions))
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
            self.msg_if.pub_warn(" " + "Driver returned invalid resolution options: " + str(e))
        # Add Framerate Cap cap_setting
        try:
            [success,framerates,encoder_cfg] = self.driver.getFramerateRange()
            self.msg_if.pub_info(" " + "Driver returned framerate options: " + str(framerates))
            if success:
                cap_setting = dict()
                cap_setting['name'] = 'Framerate'
                cap_setting['type'] = 'Int'
                options = [str(framerates['Min']),str(framerates['Max'])]
                cap_setting['options'] = options
                cap_settings['Framerate'] = cap_setting
        except Exception as e:
            self.msg_if.pub_warn(" " + "Driver returned invalid framerate options: " + str(e))
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
        #self.msg_if.pub_info(str(res_dict))
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
                            self.msg_if.pub_info("Resoluton setting: " + data + " could not be parsed to int " + str(e)) 
                        try:
                            res_dict = {'Width': width, 'Height': height}
                            success, msg = self.driver.setResolution(res_dict)
                        except Exception as e:
                            self.msg_if.pub_info("setResolution function failed " + str(e))                            
                        break     
                    elif setting_name == "Framerate":
                        try:
                            framerate = int(data)
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
    # Node driver functions

    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["HardwareId"] + "\n"
        self.msg_if.pub_info(dev_info_string)
    
        controls_dict = self.driver.getCameraControls()
        for key in controls_dict.keys():
            string = str(controls_dict[key])
            self.msg_if.pub_info(key + " " + string)
                
        
    def setFramerateRatio(self, ratio):
        if ratio < 0.1:
            ratio = 0.1
        if ratio > .99:
            ratio = 1.0
        self.framerate_ratio = ratio
        #print('Set FR Mode: ' +  str(self.current_controls["framerate_ratio"]))
        status = True
        err_str = ""
        return status, err_str


    def getFramerate(self):
        adj_fps =   nepi_img.adjust_framerate_ratio(self.current_fps,self.framerate_ratio)
        return adj_fps
    
    def getColorImg(self):
        # Check for control framerate adjustment
        last_time = self.cl_img_last_time
        current_time = nepi_utils.get_time()

        need_data = False
        if last_time != None and self.idx_if is not None:
          adj_fr = nepi_img.adjust_framerate_ratio(self.current_fps,self.framerate_ratio)
          fr_delay = float(1) / adj_fr
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
            self.img_uri_lock.acquire()
            # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
            ret, msg = self.driver.startImageAcquisition(uri_index = self.img_uri_index)
            if ret is False:
                self.img_uri_lock.release()
                return ret, msg, None, None, None
            self.color_image_acquisition_running = True
            timestamp = None
            start = nepi_utils.get_time()
            cv2_img, timestamp, ret, msg = self.driver.getImage(uri_index = self.img_uri_index)
            stop = nepi_utils.get_time()
            #print('GI: ', stop - start)
            if ret is False:
                self.img_uri_lock.release()
                return ret, msg, None, None, None
            if timestamp is None:
                timestamp = nepi_utils.get_time()  
            self.img_uri_lock.release() 
            return ret, msg, cv2_img, timestamp, encoding
        
    def stopColorImg(self):
        self.img_uri_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        ret,msg = self.driver.stopImageAcquisition(uri_index = self.img_uri_index)
        self.color_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None
        self.img_uri_lock.release()
        return ret,msg
    
if __name__ == '__main__':
	node = OnvifCamNode()

            


        

