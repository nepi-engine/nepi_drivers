#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
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

# Discovery for a Blue Robotics Bar02 or Bar30 depth sensor (TE MS5837) wired
# directly to one host I2C bus, read with Blue Robotics' ms5837-python library
# (Python module ms5837).
#
# ONE BUS ONLY: probing writes to the bus (a reset command, then PROM reads at
# address 0x76), and on a Jetson the other buses carry cameras and
# power-management parts. Only /dev/i2c-<i2c_bus> is probed, and only while no
# node is running on it, because a probe resets the sensor in the middle of
# the node's conversions. While a node runs, discovery only checks that its
# bus still exists and its process is still alive.
#
# A PROBE PASS IS NOT AN IDENTIFICATION: a PROM CRC pass rules out some other
# 0x76 parts (BMP280/BME280, the MS8607 pressure half) but not an MS5611 or
# MS5637, which use the same commands and CRC. The model cannot be read from
# the chip either, so it is the depth_model option, checked by the node
# against its first reading.

import copy
import math
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

# Imported inside drivers_mgr's process, so a missing library must not stop
# the module from loading: discovery reports it once instead.
try:
    import ms5837
    MS5837_IMPORT_ERROR = None
except Exception as e:
    ms5837 = None
    MS5837_IMPORT_ERROR = str(e)


PKG_NAME = 'NPX_BLUEROBOTICS_BAR'
FILE_TYPE = 'DISCOVERY'


PARAM_FILE_PATH = '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_bluerobotics_bar_params.yaml'

# The sensor's fixed I2C address. ms5837-python hard-codes it; kept here for
# messages only.
MS5837_I2C_ADDRESS = 0x76

#########################################
# Discover Method
#########################################

class BlueRoboticsBarDiscovery:

    NODE_LOAD_TIME_SEC = 10
    # A bus whose probe found no sensor is re-probed no sooner than this, so an
    # empty bus is not reset every discovery pass.
    PROBE_RETRY_SEC = 10
    launch_time_dict = dict()
    probe_after_dict = dict()
    retry = True
    dont_retry_list = []

    # device path -> {node_name, sub_process, config}
    active_devices_dict = dict()
    # key -> last message logged, for messages logged only when they change
    last_log_dict = dict()
    node_launch_name = "bluerobotics_bar"

    # depth_model option -> its MS5837 variant. The node's BAR_MODELS has the
    # same keys and chips; keep the two in step. The chip names the library
    # constant: ms5837.MODEL_02BA, ms5837.MODEL_30BA.
    DEPTH_MODEL_CHIPS = {
        'BlueRobotics Bar02 (MS5837-02BA)': '02BA',
        'BlueRobotics Bar30 (MS5837-30BA)': '30BA',
    }
    DEFAULT_DEPTH_MODEL = 'BlueRobotics Bar02 (MS5837-02BA)'
    # The library's OSR_256 to OSR_8192
    DEPTH_OSR_VALUES = [256, 512, 1024, 2048, 4096, 8192]
    # The i2c_bus option's choices, /dev/i2c-0 to /dev/i2c-15
    I2C_BUS_RANGE = (0, 15)
    # Fresh water is 997 and salt water 1029 kg/m3; the range leaves room for
    # warm fresh water and dense brine, and rejects a value in g/cm3.
    FLUID_DENSITY_KG_M3_RANGE = (950.0, 1100.0)
    # Sea-level air pressure is 1013.25 mbar; about 700 mbar is 3000 m up.
    SURFACE_PRESSURE_MBAR_RANGE = (700.0, 1100.0)


    ################################################
    def __init__(self):
        ############
        # Create Message Logger
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(namespace = self.log_name)
        time.sleep(1)
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")

    ##########  Nex Standard Discovery Function
    def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace,drv_dict, retry_enabled = True):

        self.drv_dict = drv_dict
        self.available_paths_list = available_paths_list
        self.active_paths_list = active_paths_list
        self.base_namespace = base_namespace

        ########################
        # Get discovery options. config is None when an option is invalid
        # (already logged): nothing new is launched, and a running node is
        # left alone.
        options = dict()
        try:
            options = self.drv_dict['DISCOVERY_DICT']['OPTIONS']
        except Exception as e:
            self.logOnChange('options', "Failed to load options " + str(e))
            return self.active_paths_list
        config = self.resolveConfig(options)

        # Retry behavior
        self.retry = retry_enabled
        if self.retry == True:
            self.dont_retry_list = []
        ########################


        ### Purge nodes whose bus disappeared or whose process exited
        path_purge_list = []
        for path_str in self.active_devices_dict:
            success = self.checkOnDevice(path_str)
            if success == False:
                path_purge_list.append(path_str)
        for path_str in path_purge_list:
            del  self.active_devices_dict[path_str]
            if path_str in self.active_paths_list:
                self.active_paths_list.remove(path_str)

        if config is None:
            return self.active_paths_list

        ### Stop a node whose bus or options no longer match the options
        for path_str in list(self.active_devices_dict.keys()):
            if self.active_devices_dict[path_str]['config'] != config:
                self.logger.log_info("Options changed; stopping node " +
                                     self.active_devices_dict[path_str]['node_name'])
                self.killDevice(path_str)

        ### Check for the device on the configured bus only
        path_str = config['device_path']
        if path_str in self.active_devices_dict:
            # A node is running on this bus: never probe it again.
            return self.active_paths_list

        if ms5837 is None:
            self.logOnChange('library', "Python module ms5837 (Blue Robotics ms5837-python) is not installed in the " +
                             "NEPI container (" + str(MS5837_IMPORT_ERROR) + "); no Bar sensor node launched. " +
                             "Install it with smbus2 to use this driver")
            return self.active_paths_list
        self.logOnChange('library', None)

        if path_str not in self.available_paths_list:
            self.logOnChange('bus', "No " + path_str + " on this host (or not visible in the NEPI container); " +
                             "check the i2c_bus option and that the bus is enabled. No Bar sensor node launched")
            return self.active_paths_list
        self.logOnChange('bus', None)

        if path_str in self.active_paths_list or path_str in self.dont_retry_list:
            # Claimed by another driver, or not to be retried
            return self.active_paths_list

        if self.probeAllowed(path_str) == False:
            return self.active_paths_list

        found = self.checkForDevice(path_str, config)
        if found:
            self.active_paths_list.append(path_str)
            success = self.launchDeviceNode(path_str, config)
            if success == False:
                if path_str in self.active_paths_list:
                    self.active_paths_list.remove(path_str)
        else:
            self.probe_after_dict[path_str] = nepi_sdk.get_time() + self.PROBE_RETRY_SEC
        return self.active_paths_list
    ################################################


    ##########  Device specific calls
    def checkForDevice(self, path_str, config):
        # Probe: construct the library object for the configured model on the
        # configured bus and run init() (reset, PROM read, CRC-4). The bus
        # handle is released whatever happens.
        found_device = False
        reason = None
        sensor = None
        try:
            model = getattr(ms5837, 'MODEL_' + self.DEPTH_MODEL_CHIPS[config['depth_config']['depth_model']])
            sensor = ms5837.MS5837(model, config['i2c_bus'])
            if getattr(sensor, '_bus', 'unknown') is None:
                reason = ("ms5837 could not open " + path_str + " (no permission, or smbus2/smbus missing " +
                          "in the NEPI container)")
            elif sensor.init() != True:
                reason = ("a device answers at I2C address 0x%02X on %s but its calibration words fail the CRC " +
                          "check: not a Bar sensor (an MS8607 or BME280 also uses 0x76), or a bad connection") % \
                         (MS5837_I2C_ADDRESS, path_str)
            elif promCoefficientsValid(getattr(sensor, '_C', None)) == False:
                reason = ("the device at I2C address 0x%02X on %s returned blank calibration words") % \
                         (MS5837_I2C_ADDRESS, path_str)
            else:
                found_device = True
        except Exception as e:
            reason = ("no Bar sensor answers at I2C address 0x%02X on %s (%s); check its wiring, 3.3 V logic, " +
                      "power and pull-ups") % (MS5837_I2C_ADDRESS, path_str, str(e))
        finally:
            closeSensorBus(sensor)

        if found_device == True:
            self.logOnChange('probe', None)
            self.logger.log_info("Found a sensor with valid MS5837 calibration words at I2C address 0x%02X on %s" %
                                 (MS5837_I2C_ADDRESS, path_str))
        else:
            self.logOnChange('probe', "Bar sensor probe: " + str(reason) + ". Probing again every " +
                             str(self.PROBE_RETRY_SEC) + " s", warn = False)
        return found_device

    def checkOnDevice(self,path_str):
        active = True
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        if path_str not in self.available_paths_list:
            self.logger.log_info("No longer detecting " + path_str + "; stopping node " + node_name)
            active = False
        elif sub_process is not None and sub_process.poll() is not None:
            # The node exits on its own after sustained read failures, so the
            # bus is free to probe again.
            self.logger.log_warn("Node " + node_name + " exited; probing " + path_str + " again before relaunching")
            active = False
        if active == False:
            nepi_drvs.killDriverNode(node_name,sub_process)
        return active

    def launchDeviceNode(self, path_str, config):
        self.logger.log_debug("Entering Launch device on path: " + str(path_str))###
        success = False
        launch_id = path_str

        # Check if should try to launch
        launch_check = True
        if launch_id in self.launch_time_dict.keys():
            launch_time = self.launch_time_dict[launch_id]
            cur_time = nepi_sdk.get_time()
            launch_check = (cur_time - launch_time) > (self.NODE_LOAD_TIME_SEC)
        if launch_check == False:
            return False   ###
        ### Start Node Launch Process
        file_name = self.drv_dict['NODE_DICT']['file_name']
        device_name = self.node_launch_name + "_" + rosName(path_str.split('/')[-1])
        node_name = nepi_system.get_device_alias(device_name)
        self.logger.log_info(" launching node: " + node_name)


        #Setup required param server drv_dict for discovery node
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace,node_name + "/drv_dict")

        # Deep-copied so a launch never mutates the drv_dict drivers_mgr owns.
        node_drv_dict = copy.deepcopy(self.drv_dict)
        node_drv_dict['DEVICE_DICT'] = {'device_name': device_name,
                                        'device_path': path_str,
                                        'i2c_bus': config['i2c_bus'],
                                        'param_file': PARAM_FILE_PATH,
                                        'depth_config': copy.deepcopy(config['depth_config'])}

        self.logger.log_debug("Launching node with path: " + path_str + " config: " + str(config))

        nepi_sdk.set_param(dict_param_name, node_drv_dict)

        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path=path_str)

        self.launch_time_dict[launch_id] = nepi_sdk.get_time()
        if success:
            self.logger.log_debug("Launched node: " + node_name)
            self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process,
                                                  'config': copy.deepcopy(config)}
        else:
            self.logger.log_warn("Failed to launch node: " + node_name + " with msg: " + msg)
            if self.retry == False:
                self.logger.log_debug("Will not try relaunch for node: " + node_name)
                self.dont_retry_list.append(launch_id)
            else:
                self.logger.log_debug("Will attempt relaunch for node: " + node_name + " in " + str(self.NODE_LOAD_TIME_SEC) + " secs")
        return success

    def killDevice(self, path_str):
        if path_str not in self.active_devices_dict:
            return
        path_entry = self.active_devices_dict[path_str]
        nepi_drvs.killDriverNode(path_entry['node_name'], path_entry['sub_process'])
        del self.active_devices_dict[path_str]
        if path_str in self.active_paths_list:
            self.active_paths_list.remove(path_str)


    def killAllDevices(self,active_paths_list):
        path_purge_list = []
        for key in self.active_devices_dict.keys():
            path_purge_list.append(key)
        for path_str in path_purge_list:
            path_entry = self.active_devices_dict[path_str]
            node_name = path_entry['node_name']
            sub_process = path_entry['sub_process']
            self.dont_retry_list.append(path_str)
            success = nepi_drvs.killDriverNode(node_name,sub_process)
            if path_str in active_paths_list:
                active_paths_list.remove(path_str)
        for path_str in path_purge_list:
            del  self.active_devices_dict[path_str]
        nepi_sdk.sleep(1)
        return active_paths_list


    def probeAllowed(self, path_str):
        # Not within PROBE_RETRY_SEC of a failed probe, and not before the
        # launch backoff would let a found sensor launch: a probe that cannot
        # lead to a launch only resets the sensor for nothing.
        cur_time = nepi_sdk.get_time()
        if cur_time < self.probe_after_dict.get(path_str, 0.0):
            return False
        if path_str in self.launch_time_dict:
            if (cur_time - self.launch_time_dict[path_str]) <= self.NODE_LOAD_TIME_SEC:
                return False
        return True


    def resolveConfig(self, options):
        # The options, validated: {device_path, i2c_bus, depth_config}, or
        # None (logged once) if any is invalid.
        try:
            i2c_bus = self.intOption(options, 'i2c_bus', 1, *self.I2C_BUS_RANGE)
            depth_model = self.optionValue(options, 'depth_model', self.DEFAULT_DEPTH_MODEL)
            if depth_model not in self.DEPTH_MODEL_CHIPS:
                raise ValueError("depth_model '" + depth_model + "' must be one of " +
                                 str(sorted(self.DEPTH_MODEL_CHIPS.keys())))
            depth_osr = self.intOption(options, 'depth_osr', 1024, self.DEPTH_OSR_VALUES[0], self.DEPTH_OSR_VALUES[-1])
            if depth_osr not in self.DEPTH_OSR_VALUES:
                raise ValueError("depth_osr must be one of " + str(self.DEPTH_OSR_VALUES))
            depth_config = dict(depth_model = depth_model,
                                depth_osr = depth_osr,
                                fluid_density_kg_m3 = self.floatOption(options, 'fluid_density_kg_m3', 997.0,
                                                                       *self.FLUID_DENSITY_KG_M3_RANGE),
                                surface_pressure_mbar = self.floatOption(options, 'surface_pressure_mbar', 1013.25,
                                                                         *self.SURFACE_PRESSURE_MBAR_RANGE))
            config = dict(device_path = '/dev/i2c-' + str(i2c_bus),
                          i2c_bus = i2c_bus,
                          depth_config = depth_config)
        except Exception as e:
            self.logOnChange('config', "Invalid Bar sensor options, no node launched: " + str(e))
            return None
        self.logOnChange('config', "Bar sensor options: " + str(config), warn = False)
        return config


    def optionValue(self, options, name, default):
        value = options.get(name, {}).get('value', default)
        if value is None:
            value = default
        return str(value).strip()


    def intOption(self, options, name, default, min_val, max_val):
        text = self.optionValue(options, name, str(default))
        try:
            fval = float(text)
        except Exception:
            raise ValueError(name + " '" + text + "' is not a number")
        if math.isnan(fval) or fval != int(fval):
            raise ValueError(name + " '" + text + "' is not a whole number")
        val = int(fval)
        if val < min_val or val > max_val:
            raise ValueError(name + " " + str(val) + " is outside " + str(min_val) + "-" + str(max_val))
        return val


    def floatOption(self, options, name, default, min_val, max_val):
        text = self.optionValue(options, name, str(default))
        try:
            val = float(text)
        except Exception:
            raise ValueError(name + " '" + text + "' is not a number")
        if math.isnan(val) or val < min_val or val > max_val:
            raise ValueError(name + " " + text + " is outside " + str(min_val) + "-" + str(max_val))
        return val


    def logOnChange(self, key, msg, warn = True):
        # Discovery runs every few seconds, so a standing condition is logged once
        # when it appears or changes rather than on every pass. msg None clears
        # the condition, so it is logged again if it comes back.
        if msg is None:
            self.last_log_dict.pop(key, None)
            return
        if self.last_log_dict.get(key) == msg:
            return
        self.last_log_dict[key] = msg
        if warn == True:
            self.logger.log_warn(msg)
        else:
            self.logger.log_info(msg)


##########################
# Helpers shared with npx_bluerobotics_bar_node.py. Identical copies: drivers
# install flat into one folder and the discovery runs inside drivers_mgr, so
# the files share no module. Keep the copies in step.

def closeSensorBus(sensor):
    # ms5837-python has no close(); its SMBus handle is the private _bus.
    # Setting it to None afterwards makes the library's read() return False
    # rather than raise on a closed handle.
    if sensor is None:
        return
    bus = getattr(sensor, '_bus', None)
    if bus is not None:
        try:
            bus.close()
        except Exception:
            pass
    try:
        sensor._bus = None
    except Exception:
        pass


def promCoefficientsValid(words):
    # True unless the calibration coefficients C1-C6 are blank. An all-zero
    # PROM passes the CRC-4, and 0 or 0xFFFF coefficients mean the bus read
    # nothing real. None (a library without _C) cannot be checked and passes.
    if words is None:
        return True
    if len(words) < 7:
        return False
    for w in words[1:7]:
        if w <= 0 or w >= 0xFFFF:
            return False
    return True


def rosName(text):
    # A plain identifier for a node name: 'i2c-1' -> 'i2c_1'
    return ''.join(c if (c.isalnum() or c == '_') else '_' for c in text)
