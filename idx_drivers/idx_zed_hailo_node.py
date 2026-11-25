#!/usr/bin/env python
#
# Copyright (c) 2026 Numurus <https://www.numurus.com>.
#
# This file is part of nepi drivers (nepi_drivers) repo
# (see https://github.com/nepi-engine/nepi_drivers)
#
# License: NEPI Drivers repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

# ZED stereo camera node for hosts without an NVIDIA GPU (e.g. Raspberry Pi 5 + Hailo-8).
# The ZED is read as a plain UVC camera through V4l2CamDriver; ZedHailoStereoDriver
# splits, rectifies and computes depth. Publishes the left image (IDX color_image),
# the right image (a second ColorImageIF under <device>/right) and a depth map.

import sys
import copy
import time
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_controls

from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.data_product_if_image import ColorImageIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'IDX_ZED_HAILO' # Use in display menus
FILE_TYPE = 'NODE'

TEST_DRV_DICT = {
'type': 'IDX',
'group_id': 'None',
'path': '/opt/nepi/nepi_engine/lib/nepi_drivers',
'usr_cfg_path': '/mnt/nepi_storage/user_cfg',
'NODE_DICT': {
    'file_name': 'idx_zed_hailo_node.py',
    'class_name': 'ZedHailoCamNode',
},
'DRIVER_DICT': {
    'file_name': 'idx_zed_hailo_driver.py' ,
    'class_name':  'ZedHailoStereoDriver'
},
'DEVICE_DICT': {'device_name': 'zed2_1-1', 'device_path': '/dev/video0', 'zed_type': 'zed2', 'resolution': 'HD720',
                'framerate': 15, 'data_products': ['color_image','depth_map'], 'download_calibration': True},
}



class ZedHailoCamNode(object):
    CAL_BACKUP_PATH = "/mnt/nepi_storage/user_cfg/cals"
    HAILO_STEREO_PATH = "/mnt/nepi_storage/ai_models/hailo_stereo"

    # V4L2 capture driver, loaded from the same driver path as DRIVER_DICT's class
    CAM_DRIVER_FILE = 'idx_v4l2_driver.py'
    CAM_DRIVER_CLASS = 'V4l2CamDriver'

    TIMING_LOG_INTERVAL_S = 10.0

    DEPTH_SETTINGS = dict(
      depth_backend = {'type': 'Selection', 'options': ['cpu_sgbm', 'hailo'], 'default': 'cpu_sgbm'},
      depth_filter = {'type': 'Selection', 'options': ['none', 'median', 'fill'], 'default': 'fill'},
      depth_downscale = {'type': 'Selection', 'options': ['auto', '1', '2', '3', '4'], 'default': 'auto'},
      sgbm_num_disparities = {'type': 'Int', 'bounds': [16, 256], 'default': 64},
      sgbm_block_size = {'type': 'Int', 'bounds': [3, 11], 'default': 5},
      hailo_swap_inputs = {'type': 'Toggle', 'default': False}
    )

    # Camera controls set at startup, before the settings defaults are read from the
    # camera. The ZED powers up with sharpness 0; the ZED SDK's default is 4.
    CAMERA_CONTROL_DEFAULTS = dict(
      sharpness = 4
    )

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
    right_data_ref_description = 'right_camera_lense'

    ZED_MIN_RANGE_M_OVERRIDES = { 'zed': .2, 'zedm': .15, 'zed2': .2, 'zedx': .2}
    ZED_MAX_RANGE_M_OVERRIDES = { 'zed':  15, 'zedm': 15, 'zed2': 20, 'zedx': 15}
    ZED_WIDTH_DEG_OVERRIDES = { 'zed': 110, 'zedm': 110, 'zed2': 110, 'zedx': 110}
    ZED_HEIGHT_DEG_OVERRIDES = { 'zed':  70, 'zedm': 70, 'zed2': 70, 'zedx': 80}

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")

    # Acquisition runs while any consumer wants frames
    acq_lock = threading.Lock()
    consumers_dict = dict(color_image = False, right_image = False, depth_map = False)

    # Last frame timestamp each consumer delivered, so a frame is published once
    color_last_ts = None
    right_last_ts = None
    depth_last_ts = None

    cl_img_last_time = None
    rt_img_last_time = None
    dm_data_last_time = None

    zed_type = 'zed'
    resolution = 'HD720'
    framerate = 15
    data_products = []
    download_calibration = True
    serial_number = None

    max_framerate = 100

    init_settings_dict = dict()
    settings_dict = dict()

    cam_driver = None
    stereo_driver = None
    idx_if = None
    right_image_if = None
    right_image_topic = None
    right_image_thread = None
    shutting_down = False

    msg_if = None
    node_if = None
    settings_if = None
    save_data_if = None

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
        self.msg_if = MsgIF()
        self.msg_if.pub_info("Starting Node Initialization Processes")

        ##############################
        # Initialize Class Variables
        self.consumers_dict = copy.deepcopy(self.consumers_dict)

        # Get required drv driver dict info
        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.zed_type = self.drv_dict['DEVICE_DICT']['zed_type']
            self.resolution = self.drv_dict['DEVICE_DICT']['resolution']
            self.framerate = self.drv_dict['DEVICE_DICT']['framerate']
            self.data_products = self.drv_dict['DEVICE_DICT']['data_products']
            self.download_calibration = self.drv_dict['DEVICE_DICT']['download_calibration']
            self.driver_path = self.drv_dict['path']
            self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
            self.driver_module = self.driver_file.split('.')[0]
            self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
        except Exception as e:
            self.msg_if.pub_debug("Failed to load Device Dict " + str(e))#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        # import driver classes from driver modules
        self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        if not success:
            self.msg_if.pub_warn("Failed to import " + self.driver_class_name + " from " + self.driver_path + ": " + str(msg))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because driver class could not be imported")
            return
        cam_module = self.CAM_DRIVER_FILE.split('.')[0]
        self.msg_if.pub_info("Importing driver class " + self.CAM_DRIVER_CLASS + " from module " + cam_module)
        [success, msg, self.cam_driver_class] = nepi_drvs.importDriverClass(self.CAM_DRIVER_FILE,self.driver_path,cam_module,self.CAM_DRIVER_CLASS)
        if not success:
            self.msg_if.pub_warn("Failed to import " + self.CAM_DRIVER_CLASS + " from " + self.driver_path + ": " + str(msg))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because camera driver class could not be imported")
            return

        # Open the camera. On a fresh hot-plug the USB device may not be fully
        # enumerated yet, so retry a few times with a short settle delay.
        # The ZED streams YUYV only, so MJPG is off.
        OPEN_MAX_TRIES = 5
        OPEN_RETRY_DELAY_SEC = 2.0
        open_msg = ""
        for attempt in range(OPEN_MAX_TRIES):
            try:
                cam_driver = self.cam_driver_class(self.device_path, False)
                if cam_driver.isConnected():
                    self.cam_driver = cam_driver
                    break
                open_msg = "device reported not connected"
            except Exception as e:
                open_msg = str(e)
            self.msg_if.pub_debug("Camera Open (attempt " + str(attempt + 1) + "/" +
                                 str(OPEN_MAX_TRIES) + ") : " + open_msg)
            if nepi_sdk.is_shutdown():
                return
            nepi_sdk.sleep(OPEN_RETRY_DELAY_SEC)
        if self.cam_driver is None:
            self.msg_if.pub_warn("Camera open failed after " + str(OPEN_MAX_TRIES) + " attempts at " + str(self.device_path) + ": " + open_msg)
            # Exit so drivers_mgr can re-discover and relaunch the node once the camera is available
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because Not Able to Connect to Zed Camera")
            return
        self.msg_if.pub_info("Zed Camera Connected at " + str(self.device_path))

        try:
            self.stereo_driver = self.driver_class(self.cam_driver, self.resolution, self.CAL_BACKUP_PATH, self.HAILO_STEREO_PATH)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create " + self.driver_class_name + ": " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because stereo driver could not be created")
            return

        # Capture mode: request the side-by-side size and check what was negotiated
        [size_ok, msg, fps] = self.stereo_driver.configure_capture(self.framerate)
        if size_ok:
            self.msg_if.pub_info(msg)
        else:
            self.msg_if.pub_error(msg)
        self.framerate = fps

        # Camera control defaults, applied before initSettingsDict reads the camera's values
        camera_controls = self.cam_driver.getCameraControls()
        for control_name in self.CAMERA_CONTROL_DEFAULTS.keys():
            if control_name in camera_controls:
                control_value = self.CAMERA_CONTROL_DEFAULTS[control_name]
                [success, msg] = self.cam_driver.setCameraControl(control_name, control_value)
                if success:
                    self.msg_if.pub_info("Camera control " + control_name + " set to " + str(control_value))
                else:
                    self.msg_if.pub_warn("Could not set camera control " + control_name + " to " + str(control_value) + ": " + str(msg))

        # Serial number and calibration
        [self.serial_number, msg] = self.stereo_driver.find_serial_number(self.device_path)
        self.msg_if.pub_info(msg)
        if self.serial_number is not None:
            self.device_info_dict["serial_number"] = self.serial_number
        download = (self.download_calibration == True or self.download_calibration == 'True' or self.download_calibration == 'true')
        [cal_success, msg, warn] = self.stereo_driver.load_calibration(self.serial_number, download)
        if warn:
            self.msg_if.pub_warn(msg)
        else:
            self.msg_if.pub_info(msg)

        # Initialize controls
        self.factory_controls = copy.deepcopy(self.FACTORY_CONTROLS)
        # Apply OVERRIDES
        if self.zed_type in self.ZED_MIN_RANGE_M_OVERRIDES:
          self.factory_controls['min_range_m'] = self.ZED_MIN_RANGE_M_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_MAX_RANGE_M_OVERRIDES:
          self.factory_controls['max_range_m'] = self.ZED_MAX_RANGE_M_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_WIDTH_DEG_OVERRIDES:
          self.factory_controls['width_deg'] = self.ZED_WIDTH_DEG_OVERRIDES[self.zed_type]
        if self.zed_type in self.ZED_HEIGHT_DEG_OVERRIDES:
          self.factory_controls['height_deg'] = self.ZED_HEIGHT_DEG_OVERRIDES[self.zed_type]
        # A calibrated camera's FOV comes from its rectified left camera
        fov_deg = self.stereo_driver.get_fov_deg()
        if fov_deg is not None:
          self.factory_controls['width_deg'] = round(fov_deg[0], 2)
          self.factory_controls['height_deg'] = round(fov_deg[1], 2)
        self.current_controls = self.factory_controls # Updated during initialization

        # Initialize settings
        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()

        # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
        self.msg_if.pub_info("Launching NEPI IDX () interface...")
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.idx_if = IDXDeviceIF(device_info = self.device_info_dict,
                                    data_products =  self.data_products,
                                    data_source_description = self.data_source_description,
                                    data_ref_description = self.data_ref_description,
                                    getSettingsFunction=self.getSettingsFunction,
                                    setSettingFunction=self.setSettingFunction,
                                    factoryControls = self.factory_controls,
                                    setMaxFramerate =self.setMaxFramerate,
                                    getFramerate = self.getFramerate,
                                    setRangeRatio = self.setRangeRatio,
                                    getColorImage = self.getColorImage,
                                    stopColorImageAcquisition = self.stopColorImage,
                                    getDepthMap = self.getDepthMap,
                                    stopDepthMapAcquisition = self.stopDepthMap,
                                    msg_if = self.msg_if,
                                    )
        self.msg_if.pub_info("... IDX interface running")

        ready = self.idx_if.wait_for_ready()

        self.node_if = self.idx_if.node_if
        self.settings_if = self.idx_if.settings_if
        self.save_data_if = self.idx_if.save_data_if

        # Right camera image: its own ColorImageIF and loop, modeled on the IDX color loop
        self.right_image_thread = threading.Thread(target=self.runRightImageThread)
        self.right_image_thread.daemon = True
        self.right_image_thread.start()

        self.logDeviceInfo()
        self.logDriverMsgs()

        ##########################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        nepi_sdk.start_timer_process(self.TIMING_LOG_INTERVAL_S, self.logTimingCb)
        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()


    #**********************
    # Sensor setting functions

    def initSettingsDict(self):
        init_settings_dict = dict()
        device_controls_dict = self.cam_driver.getCameraControls()

        ### Camera controls from the V4L2 driver
        for setting_name in device_controls_dict.keys():
            setting_dict = dict()
            setting_current = None
            setting_dict['name'] = setting_name
            info = device_controls_dict[setting_name]
            setting_type = info['type']
            if setting_type == 'int':
                setting_type = 'Int'
            elif setting_type == 'float':
                setting_type = 'Float'
            elif setting_type == 'bool':
                setting_type = 'Toggle'
            elif setting_type == 'menu':
                setting_type = 'Menu'
            setting_dict['type'] = setting_type

            setting_current = str(info['value'])
            if setting_type == 'Int':
                try:
                    setting_dict['default'] = int(setting_current)
                    setting_dict['bounds'] = [int(info['min']),int(info['max'])]
                except:
                    pass
            elif setting_type == 'Float':
                try:
                    setting_dict['default'] = float(setting_current)
                    setting_dict['bounds'] = [float(info['min']),float(info['max'])]
                except:
                    pass
            elif setting_type == 'Toggle':
                setting_dict['default'] = (setting_current == True or setting_current == 'True' or setting_current == 'true')
            elif  setting_type == 'Selection' or setting_type == 'Selections' or setting_type == 'Menu':
                try:
                    setting_dict['default'] = int(setting_current)
                    legend = info['legend']
                    setting_dict['options'] = list(legend.keys())
                except:
                    pass
            else:
                setting_dict['default'] = str(setting_current)
            init_settings_dict[setting_name] = setting_dict

        ### Depth settings
        for setting_name in self.DEPTH_SETTINGS.keys():
            setting_dict = copy.deepcopy(self.DEPTH_SETTINGS[setting_name])
            setting_dict['name'] = setting_name
            init_settings_dict[setting_name] = setting_dict
        self.applyDepthSetting('depth_filter', self.DEPTH_SETTINGS['depth_filter']['default'])
        self.applyDepthSetting('depth_downscale', self.DEPTH_SETTINGS['depth_downscale']['default'])
        self.applyDepthSetting('sgbm_num_disparities', self.DEPTH_SETTINGS['sgbm_num_disparities']['default'])
        self.applyDepthSetting('sgbm_block_size', self.DEPTH_SETTINGS['sgbm_block_size']['default'])
        self.applyDepthSetting('hailo_swap_inputs', self.DEPTH_SETTINGS['hailo_swap_inputs']['default'])

        self.init_settings_dict = init_settings_dict
        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        settings_dict_values = nepi_controls.get_values_dict(settings_dict)
        self.msg_if.pub_debug("Initialized Settings: " + str(settings_dict_values))
        return settings_dict

    def refreshSettingsDict(self):
        settings_dict = copy.deepcopy(self.settings_dict)
        device_controls_dict = self.cam_driver.getCameraControls()

        ### Camera controls
        for setting_name in device_controls_dict.keys():
            if setting_name in settings_dict.keys():
                info = device_controls_dict[setting_name]
                setting_current = str(info['value'])
                setting_type = settings_dict[setting_name]['type']
                if setting_type == 'Int':
                    try:
                        settings_dict = nepi_controls.set_value(settings_dict, setting_name, int(setting_current))
                        settings_dict = nepi_controls.set_bounds(settings_dict, setting_name, [int(info['min']),int(info['max'])])
                    except:
                        pass
                elif setting_type == 'Float':
                    try:
                        settings_dict = nepi_controls.set_value(settings_dict, setting_name, float(setting_current))
                        settings_dict = nepi_controls.set_bounds(settings_dict, setting_name, [float(info['min']),float(info['max'])])
                    except:
                        pass
                elif setting_type == 'Toggle':
                    value = (setting_current == True or setting_current == 'True' or setting_current == 'true')
                    settings_dict = nepi_controls.set_value(settings_dict, setting_name, value)
                elif  setting_type == 'Selection' or setting_type == 'Selections' or setting_type == 'Menu':
                    try:
                        settings_dict = nepi_controls.set_value(settings_dict, setting_name, int(setting_current))
                        legend = info['legend']
                        settings_dict = nepi_controls.set_options(settings_dict, setting_name, list(legend.keys()))
                    except:
                        pass
                else:
                    settings_dict = nepi_controls.set_value(settings_dict, setting_name, str(setting_current))

        ### Depth settings, read back from the stereo driver
        if 'depth_backend' in settings_dict.keys():
            settings_dict = nepi_controls.set_value(settings_dict, 'depth_backend', self.stereo_driver.requested_backend)
        if 'depth_filter' in settings_dict.keys():
            settings_dict = nepi_controls.set_value(settings_dict, 'depth_filter', self.stereo_driver.depth_filter)
        if 'depth_downscale' in settings_dict.keys():
            settings_dict = nepi_controls.set_value(settings_dict, 'depth_downscale', self.stereo_driver.depth_downscale)
        if 'sgbm_num_disparities' in settings_dict.keys():
            settings_dict = nepi_controls.set_value(settings_dict, 'sgbm_num_disparities', int(self.stereo_driver.sgbm_num_disparities))
        if 'sgbm_block_size' in settings_dict.keys():
            settings_dict = nepi_controls.set_value(settings_dict, 'sgbm_block_size', int(self.stereo_driver.sgbm_block_size))
        if 'hailo_swap_inputs' in settings_dict.keys():
            settings_dict = nepi_controls.set_value(settings_dict, 'hailo_swap_inputs', self.stereo_driver.hailo_swap_inputs == True)

        settings_dict_values = nepi_controls.get_values_dict(settings_dict)
        self.msg_if.pub_debug("Refreshed Current Settings: " + str(settings_dict_values))
        return settings_dict

    def getSettingsFunction(self):
        return self.settings_dict

    def setSettingFunction(self,setting_name, setting_value):
        setting_str = setting_name + ":" + str(setting_value)
        success = True
        msg = 'Success'
        found_setting = False
        if setting_name in self.settings_dict.keys():
            found_setting = True
            cur_val = nepi_controls.get_value(self.settings_dict, setting_name)
            # depth_backend is always applied, so selecting hailo again retries a failed load
            if str(cur_val) != str(setting_value) or setting_name == 'depth_backend':
                self.msg_if.pub_debug("Update Setting: " + setting_name + " from: " + str(cur_val) + " to: " + str(setting_value))
                if setting_name in self.DEPTH_SETTINGS.keys():
                    success, msg = self.applyDepthSetting(setting_name, setting_value)
                else:
                    success, msg = self.cam_driver.setCameraControl(setting_name,setting_value)
                self.settings_dict = self.refreshSettingsDict()
                self.logDriverMsgs()
        if found_setting is False:
            success = False
            msg = (self.node_name  + " Setting name " + setting_str + " is not supported")
        return success, msg, self.settings_dict

    def applyDepthSetting(self, setting_name, setting_value):
        try:
            if setting_name == 'depth_backend':
                return self.stereo_driver.set_depth_backend(str(setting_value))
            elif setting_name == 'depth_filter':
                return self.stereo_driver.set_depth_filter(str(setting_value))
            elif setting_name == 'depth_downscale':
                return self.stereo_driver.set_depth_downscale(str(setting_value))
            elif setting_name == 'sgbm_num_disparities':
                [nd, bs] = self.stereo_driver.set_sgbm_params(int(float(setting_value)), self.stereo_driver.sgbm_block_size)
                return True, "sgbm_num_disparities set to " + str(nd)
            elif setting_name == 'sgbm_block_size':
                [nd, bs] = self.stereo_driver.set_sgbm_params(self.stereo_driver.sgbm_num_disparities, int(float(setting_value)))
                return True, "sgbm_block_size set to " + str(bs)
            elif setting_name == 'hailo_swap_inputs':
                swap = (setting_value == True or setting_value == 'True' or setting_value == 'true')
                self.stereo_driver.set_hailo_swap_inputs(swap)
                return True, "hailo_swap_inputs set to " + str(swap)
        except Exception as e:
            return False, "Failed to apply " + setting_name + " = " + str(setting_value) + ": " + str(e)
        return False, "Unknown depth setting " + setting_name


    #**********************
    # IDX driver functions

    def logDeviceInfo(self):
        device_info_str = self.node_name + " info:\n"
        device_info_str += "\tDevice Path: " + str(self.device_path) + "\n"
        device_info_str += "\tZed Type: " + str(self.zed_type) + "\n"
        device_info_str += "\tSerial Number: " + str(self.serial_number) + "\n"
        device_info_str += "\tResolution: " + str(self.resolution) + " at " + str(self.framerate) + " fps\n"
        device_info_str += "\tCalibrated: " + str(self.stereo_driver.is_calibrated()) + "\n"
        device_info_str += "\tRectifying: " + str(self.stereo_driver.is_rectifying()) + "\n"
        device_info_str += "\tDepth Backend: " + str(self.stereo_driver.get_active_backend()) + "\n"
        device_info_str += "\tDepth Filter: " + str(self.stereo_driver.depth_filter) + "\n"
        device_info_str += "\tDepth Scale Factor: " + str(self.stereo_driver.get_depth_scale_factor()) + "\n"
        device_info_str += "\tRight Image Topic: " + str(self.right_image_topic) + "\n"
        self.msg_if.pub_info(device_info_str)
        self.msg_if.pub_info(str(self.device_info_dict))

    def logDriverMsgs(self):
        if self.stereo_driver is None:
            return
        for [level, text] in self.stereo_driver.get_messages():
            if level == 'error':
                self.msg_if.pub_error(text)
            elif level == 'warn':
                self.msg_if.pub_warn(text)
            elif level == 'info':
                self.msg_if.pub_info(text)
            else:
                self.msg_if.pub_debug(text)

    def logTimingCb(self, timer):
        if self.stereo_driver is None or self.shutting_down:
            return
        stats = self.stereo_driver.get_timing_stats()
        rect_ms = 'n/a' if stats['rectify_ms'] is None else str(round(stats['rectify_ms'], 1))
        depth_ms = 'n/a' if stats['depth_ms'] is None else str(round(stats['depth_ms'], 1))
        self.msg_if.pub_debug("Stereo timing: capture " + str(round(stats['capture_fps'], 1)) + " fps, rectify " + rect_ms +
                              " ms, depth " + depth_ms + " ms, backend " + stats['backend'])
        self.logDriverMsgs()

    def setMaxFramerate(self, rate):
        if rate is None:
            return False, 'Got None Max Framerate'
        if rate < 1:
            rate = 1
        if rate > 100:
            rate = 100
        self.max_framerate = rate
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

    def getRangeWindowM(self):
        # The active window, computed as IDXDeviceIF does for the depth product.
        # Read from the IDX IF when it exists, since it holds the restored ratios.
        src = self.idx_if if self.idx_if is not None else None
        min_m = getattr(src, 'min_range_m', self.current_controls['min_range_m'])
        max_m = getattr(src, 'max_range_m', self.current_controls['max_range_m'])
        start = getattr(src, 'start_range_ratio', self.current_controls['start_range_ratio'])
        stop = getattr(src, 'stop_range_ratio', self.current_controls['stop_range_ratio'])
        try:
            range_m = max_m - min_m
            return [min_m + start * range_m, max_m - (1 - stop) * range_m]
        except Exception:
            return [None, None]

    def needsData(self, last_time):
        if last_time is None or self.idx_if is None:
            return True
        return (nepi_utils.get_time() - last_time) > (float(1) / self.max_framerate)

    def startAcquisition(self, consumer):
        self.acq_lock.acquire()
        self.consumers_dict[consumer] = True
        # Always try to start; the driver returns quickly if already running
        ret, msg = self.cam_driver.startImageAcquisition()
        self.acq_lock.release()
        return ret, msg

    def stopAcquisition(self, consumer):
        self.acq_lock.acquire()
        self.consumers_dict[consumer] = False
        ret = True
        msg = "Success"
        if True not in self.consumers_dict.values():
            ret, msg = self.cam_driver.stopImageAcquisition()
            self.msg_if.pub_debug("Stopped camera acquisition: no image or depth consumers")
        self.acq_lock.release()
        return ret, msg

    def getColorImage(self):
        if self.needsData(self.cl_img_last_time) == False:
            return False, "Waiting for Timer", None, None, None
        ret, msg = self.startAcquisition('color_image')
        if ret is False:
            self.msg_if.pub_debug("Image acquisition failed to start for " + str(self.device_path) + ": " + str(msg), throttle_s = 5.0)
            return False, msg, None, None, None
        left, right, timestamp, success, msg = self.stereo_driver.get_stereo_pair(last_timestamp = self.color_last_ts)
        self.logDriverMsgs()
        if success is False:
            if timestamp is None:
                self.msg_if.pub_warn("No color image from " + str(self.device_path) + ": " + str(msg), throttle_s = 20.0)
            return False, msg, None, None, None
        self.color_last_ts = timestamp
        self.cl_img_last_time = nepi_utils.get_time()
        return True, "Success", left, timestamp, 'bgr8'

    def stopColorImage(self):
        ret, msg = self.stopAcquisition('color_image')
        self.color_last_ts = None
        self.msg_if.pub_debug("Stopped Color Image Acquire")
        return ret, msg

    def getRightImage(self):
        if self.needsData(self.rt_img_last_time) == False:
            return False, "Waiting for Timer", None, None, None
        ret, msg = self.startAcquisition('right_image')
        if ret is False:
            self.msg_if.pub_debug("Image acquisition failed to start for " + str(self.device_path) + ": " + str(msg), throttle_s = 5.0)
            return False, msg, None, None, None
        left, right, timestamp, success, msg = self.stereo_driver.get_stereo_pair(last_timestamp = self.right_last_ts)
        self.logDriverMsgs()
        if success is False:
            return False, msg, None, None, None
        self.right_last_ts = timestamp
        self.rt_img_last_time = nepi_utils.get_time()
        return True, "Success", right, timestamp, 'bgr8'

    def stopRightImage(self):
        ret, msg = self.stopAcquisition('right_image')
        self.right_last_ts = None
        self.msg_if.pub_debug("Stopped Right Image Acquire")
        return ret, msg

    def getDepthMap(self):
        if self.needsData(self.dm_data_last_time) == False:
            return False, "Waiting for Timer", None, None, None
        if not self.stereo_driver.is_rectifying():
            self.msg_if.pub_warn("No depth map: " + self.stereo_driver.get_depth_disabled_msg(), throttle_s = 60.0)
            return False, "Depth unavailable", None, None, None
        ret, msg = self.startAcquisition('depth_map')
        if ret is False:
            self.msg_if.pub_debug("Image acquisition failed to start for " + str(self.device_path) + ": " + str(msg), throttle_s = 5.0)
            return False, msg, None, None, None
        [min_range_m, max_range_m] = self.getRangeWindowM()
        depth_map, timestamp, success, msg = self.stereo_driver.get_depth_map(min_range_m, max_range_m, last_timestamp = self.depth_last_ts)
        self.logDriverMsgs()
        if success is False:
            if timestamp is None:
                self.msg_if.pub_warn("No depth map from " + str(self.device_path) + ": " + str(msg), throttle_s = 20.0)
            return False, msg, None, None, None
        self.depth_last_ts = timestamp
        self.dm_data_last_time = nepi_utils.get_time()
        return True, "Success", depth_map, timestamp, '32FC1'

    def stopDepthMap(self):
        ret, msg = self.stopAcquisition('depth_map')
        self.depth_last_ts = None
        self.msg_if.pub_debug("Stopped Depthmap Acquire")
        return ret, msg


    #**********************
    # Right image loop (modeled on IDXDeviceIF's color image loop)

    def runRightImageThread(self):
        right_namespace = nepi_sdk.create_namespace(self.idx_if.get_namespace(), 'right')
        caps_dict = {
            'has_stabilize': True
        }
        # Own node_if (none passed): image IFs sharing a node_if collide on registry keys.
        # No save_data_if: it keys saves by data product, which would mix right frames into
        # the left color_image saves. No transform/navpose: those describe the left lens.
        self.right_image_if = ColorImageIF(namespace = right_namespace,
                    data_source_description = self.data_source_description,
                    data_ref_description = self.right_data_ref_description,
                    perspective = 'pov',
                    caps_dict = caps_dict,
                    msg_if = self.msg_if
                    )
        ready = self.right_image_if.wait_for_ready()
        self.right_image_topic = self.right_image_if.get_namespace()
        self.msg_if.pub_info("Right camera image publishing on " + str(self.right_image_topic))

        acquiring = False
        while (not nepi_sdk.is_shutdown() and not self.shutting_down):
            get_data = self.right_image_if.needs_data_check()
            device_disabled = getattr(self.idx_if, 'device_disabled', False)
            if get_data == True and device_disabled == False:
                acquiring = True
                status, msg, cv2_img, timestamp, encoding = self.getRightImage()
                if status is True and cv2_img is not None:
                    self.right_image_if.publish_cv2_img(cv2_img, encoding = encoding,
                                                    timestamp = timestamp,
                                                    width_deg = self.current_controls['width_deg'],
                                                    height_deg = self.current_controls['height_deg']
                                                    )
            elif acquiring is True:
                self.msg_if.pub_info("Stopping right_image acquisition" )
                self.stopRightImage()
                acquiring = False
            else:
                nepi_sdk.sleep(0.25)
            nepi_sdk.sleep(0.01) # Yield


    def cleanup_actions(self):
      self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
      self.shutting_down = True
      try:
        if self.cam_driver is not None:
          self.cam_driver.stopImageAcquisition()
          self.msg_if.pub_info("Stopped zed camera acquisition")
      except Exception as e:
        self.msg_if.pub_warn("Failed to stop zed camera acquisition: " + str(e))
      try:
        if self.stereo_driver is not None:
          self.stereo_driver.cleanup()
          self.msg_if.pub_info("Released stereo driver resources")
      except Exception as e:
        self.msg_if.pub_warn("Failed to release stereo driver resources: " + str(e))

if __name__ == '__main__':
    node = ZedHailoCamNode()
