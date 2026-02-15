#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import subprocess
import time
import serial

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system
from nepi_sdk import nepi_serial

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "microstrain_imu"
logger = Logger(log_name = log_name)

PKG_NAME = 'NPX_MICROSTRAIN_AHAR'
FILE_TYPE = 'DISCOVERY'


PARAM_FILE_PATH = '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_microstrain_params.yaml'

#########################################
# Discover Method
#########################################

class MicrostrainDiscovery:

    NODE_LOAD_TIME_SEC = 10
    launch_time_dict = dict()
    retry = True
    dont_retry_list = []

    active_devices_dict = dict()
    node_launch_name = "microstrain_imu"
    baudrate_list = []
    baud_str = '115200'
    baud_int = 115200
    addr_str = "001"

    includeDevices = []
    excludedDevices = ['ttyACM']


    ################################################          
    def __init__(self):
        ############
        # Create Message Logger
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name = self.log_name)
        time.sleep(1)
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")

    ##########  Nex Standard Discovery Function
    def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace,drv_dict):

        # Create path search options
        
        #self.logger.log_warn("Entering discovery function with available_paths_list: " + str(available_paths_list))###
        #self.logger.log_warn("Entering discovery function with active_paths_list: " + str(active_paths_list))###
        #self.logger.log_warn("Entering discovery function with drv_dict: " + str(drv_dict))###

        self.drv_dict = drv_dict
        self.available_paths_list = available_paths_list
        self.active_paths_list = active_paths_list
        self.base_namespace = base_namespace

        ########################
        # Get discovery options


        self.path_list = nepi_serial.get_serial_ports_list()
        

        try:
                
            #self.logger.log_warn("Starting discovery with drv_dict " + str(drv_dict))#
            baudrate_options = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['options']
            baudrate_sel = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['value']
            #self.logger.log_warn("Got baudrate options: " + str(baudrate_options))###
            #self.logger.log_warn("Got selected baudrate: " + str(baudrate_sel))###
            baudrate_list = []
            if baudrate_sel != "All":
                baudrate_list.append(baudrate_sel)
            else:
                for baudrate in baudrate_options:
                    if baudrate != "All":
                        baudrate_list.append(baudrate)
            self.baudrate_list = baudrate_list
            #self.logger.log_warn("Got selected baudrate list" + str(self.baudrate_list))
        except Exception as e:
            self.logger.log_warn("Failed to load options " + str(e))#
            return None

        if 'retry' in self.drv_dict['DISCOVERY_DICT']['OPTIONS'].keys():
            self.retry = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['retry']['value']
        else:
            self.retry = True
        #self.logger.log_warn("Got retry" + str(self.retry))
        ########################


        ### Purge Unresponsive Connections
        #self.logger.log_warn("Running Cleanup Check Process")
        path_purge_list = []
        for path_str in self.active_devices_dict:
            success = self.checkOnDevice(path_str)
            if self.retry == False:
                if success == False:
                    path_purge_list.append(path_str) 
        # Clean up the active_devices_dict
        for path_str in path_purge_list:
            del  self.active_devices_dict[path_str]
            if path_str in self.active_paths_list:
                self.active_paths_list.remove(path_str)

        ### Checking for devices on available paths
        #self.logger.log_warn("Starting Check for Device Process with path list: " + str(self.path_list))
        for path_str in self.path_list:
            valid_path = True
            #self.logger.log_warn("Checking path against exclude list: " + str(self.excludedDevices))
            for exclude in self.excludedDevices:
                #self.logger.log_warn("Checking exclude : " + str(exclude) + " with active list " + str(self.active_paths_list))
                if path_str.find(exclude) != -1 or path_str in self.active_paths_list:
                    valid_path = False
            #self.logger.log_warn("Got path check valid: " + str(valid_path))
            if valid_path:
                #self.logger.log_warn("Looking for path: " + path_str)
                #self.logger.log_warn("In path_list: " + str(self.active_paths_list))
                found = self.checkForDevice(path_str)
                if found:
                    success = self.launchDeviceNode(path_str)
                    if success:
                        self.active_paths_list.append(path_str)
        return self.active_paths_list
    ################################################


    # Function to verify the connection by sending a command
    def verify_microstrain_connection(self,port_str,baud_str):
        # Clear buffers

        try:
            self.baud_int = int(baud_str)
            # Open the serial port (adjust baud rate as needed for your specific device)
            # MicroStrain devices typically use 115200 baud rate.
            ser = serial.Serial(port_str, baudrate=115200, timeout=1)
            time.sleep(0.1) # Give the port a moment to open


            #Clear buffers
            ser.flushInput()
            ser.flushOutput()
            

            # Send an ping command to the device
            command = b'\x75\x65\x01\x02\x02\x01\xE0\xC6' # Placeholder: replace with actual command

            ser.write(command)
            #response = ser.readline().decode('utf-8').strip()
            response = str(ser.readline().hex()) # Read the response
            #print("Response: " + str(response))
            ser.close()

            # Check if the response is valid
            if response and '7565010404f10100d56a' in response: # Check for expected response
                #print(f"Successfully verified connection to MicroStrain device on {port_str}.")
                return True
            else:
                #print(f"Device at {port_str} did not respond as expected.")
                return False

        except serial.SerialException as e:
            #print(f"Error opening/communicating with port {port_str}: {e}")
            return False





    ##########  Device specific calls
    def checkForDevice(self, path_str):
        found_device = False
        #self.logger.log_warn("Checking for device on path: " + path_str)

        if path_str not in self.active_paths_list:
            for baud_str in self.baudrate_list:
                self.baud_str = baud_str
                found_device = self.verify_microstrain_connection(path_str,baud_str)
                if found_device == True:
                    self.logger.log_warn("Found device on path: " + path_str + " with baud rate: " + baud_str) 
                    break

        #self.logger.log_warn("Check for device on path returned: " + str(found_device))
        return found_device

    def checkOnDevice(self,path_str):
        self.logger.log_debug("Entering check for device function for path: " + str(path_str))###
        active = True
        if path_str not in self.available_paths_list:
            active = False
        if active == False:
            self.logger.log_info("No longer detecting device on : " + path_str)
            if path_str in self.active_devices_dict.keys():
                path_entry = self.active_devices_dict[path_str]
                node_name = path_entry['node_name']
                sub_process = path_entry['sub_process']
                success = nepi_drvs.killDriverNode(node_name,sub_process)
        return active

    def launchDeviceNode(self, path_str):
        self.logger.log_debug("Entering Launch device on path: " + str(path_str))###
        success = False
        launch_id = path_str

        # Check if should try to launch
        launch_check = True
        if launch_id in self.launch_time_dict.keys():
            launch_time = self.launch_time_dict[launch_id]
            cur_time = nepi_sdk.get_time()
            launch_check = (cur_time - launch_time) > str(self.NODE_LOAD_TIME_SEC)
        if launch_check == False:
            return False   ###
        #self.logger.log_debug("Starting Launch process on path: " + str(path_str))###
        ### Start Node Launch Process
        file_name = self.drv_dict['NODE_DICT']['file_name']
        device_node_name = self.node_launch_name + "_" + path_str.split('/')[-1]
        node_name = nepi_system.get_node_name(device_node_name)
        self.logger.log_warn(" launching node: " + node_name)
        
        # Try and load saved node params if file exists
        nepi_sdk.load_node_config(device_node_name, node_name)
        
        #Setup required param server drv_dict for discovery node
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace,node_name + "/drv_dict")
        
        self.drv_dict['DEVICE_DICT'] = dict()
        self.drv_dict['DEVICE_DICT']['port'] = path_str
        self.drv_dict['DEVICE_DICT']['baudrate'] = int(self.baud_str)
        self.drv_dict['DEVICE_DICT']['aux_port'] = 'None' # path_str
        self.drv_dict['DEVICE_DICT']['aux_baudrate'] = int(self.baud_str)
        self.drv_dict['DEVICE_DICT']['param_file'] = PARAM_FILE_PATH
        
        self.logger.log_warn("Launching node with path: " + path_str + " baudrate: " + self.baud_str)
        
        nepi_sdk.set_param(dict_param_name, self.drv_dict)
        
        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path=path_str)

        self.launch_time_dict[launch_id] = nepi_sdk.get_time()
        if success:
            self.logger.log_warn("Launched node: " + node_name)
            self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}
        else:
            self.logger.log_warn("Failed to launch node: " + node_name + " with msg: " + msg)
            if self.retry == False:
                self.logger.log_warn("Will not try relaunch for node: " + node_name)
                self.dont_retry_list.append(launch_id)
            else:
                self.logger.log_warn("Will attempt relaunch for node: " + node_name + " in " + str(self.NODE_LOAD_TIME_SEC) + " secs")
        return success
    


    def killAllDevices(self,active_paths_list):
        #self.logger.log_warn("Entering Kill All Devices function for path: " + str(path_str))###
        path_purge_list = []
        for key in self.active_devices_dict.keys():
            path_purge_list.append(key)
        #self.logger.log_warn("Killing Devices: " + str(path_purge_list))
        for path_str in path_purge_list:
            path_entry = self.active_devices_dict[path_str]
            node_name = path_entry['node_name']
            sub_process = path_entry['sub_process']
            success = nepi_drvs.killDriverNode(node_name,sub_process)
            if path_str in active_paths_list:
                active_paths_list.remove(path_str)
        for path_str in path_purge_list:
            del  self.active_devices_dict[path_str]
        nepi_sdk.sleep(1)
        return active_paths_list