#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system
from nepi_sdk import nepi_utils


PKG_NAME = 'NPX_NMEA_UDP'
FILE_TYPE = 'DISCOVERY'

PARAM_FILE_PATH = '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_nmea_udp_params.yaml'
DEFAULT_NODE_FILE = 'npx_nmea_udp_node.py'

class NMEAUDPDiscovery:
    NODE_LOAD_TIME_SEC = 10
    launch_time_dict = dict()
    retry = True
    dont_retry_list = []

    active_devices_dict = dict()
    node_launch_name = "nmea"

    system_config = dict()
    system_nav_ip = 'NONE'

    ################################################
    def __init__(self):
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name=self.log_name)
        time.sleep(0.2)
        self.system_config = nepi_system.load_nepi_system_config()
        self.msg_if.pub_warn("Got System Config: " + str(self.system_config))
        if self.system_config is None:
            self.system_config = dict()
        if len(self.system_config.keys()) == 0:
            self.msg_if.pub_warn("Failed to Read NEPI config file")
        else:
          if 'NEPI_NAV_IP' in self.system_config.keys():
            nav_ip = self.system_config['NEPI_NAV_IP']
            if nepi_utils.is_valid_ip(nav_ip):
              self.system_nav_ip = nav_ip
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")

    ################################################
    def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled=True):
        self.drv_dict = drv_dict or {}
        self.available_paths_list = available_paths_list or []
        self.active_paths_list = active_paths_list or []
        self.base_namespace = base_namespace

        opts = (self.drv_dict.get('DISCOVERY_DICT') or {}).get('OPTIONS', {})
        if nepi_utils.is_valid_ip(self.system_nav_ip):
            host = str(self.system_nav_ip)
        else:
            host = str(opts.get('tcp_host', {}).get('value', '127.0.0.1'))
        port = int(opts.get('tcp_port', {}).get('value', 50000))

        self.retry = retry_enabled
        if self.retry == True:
            self.dont_retry_list = []

        launch_key = f"{host}:{port}"

        purge = []
        for key in list(self.active_devices_dict.keys()):
            if key != launch_key:
                purge.append(key)
        for key in purge:
            entry = self.active_devices_dict[key]
            nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
            if key in self.active_paths_list:
                self.active_paths_list.remove(key)
            del self.active_devices_dict[key]

        if launch_key not in self.active_paths_list and launch_key not in self.dont_retry_list:
            found = self.checkForDevice(launch_key)
            if found:
                if self.launchDeviceNode(launch_key, host, port):
                    self.active_paths_list.append(launch_key)

        return self.active_paths_list

    ################################################
    def checkForDevice(self, launch_key: str) -> bool:
        # For TCP, configuration is the discovery. If user provided host/port, we consider it "found".
        return True

    def checkOnDevice(self, launch_key: str) -> bool:
        return True

    def launchDeviceNode(self, launch_key: str, host: str, port: int) -> bool:
        success = False

        launch_check = True
        if launch_key in self.launch_time_dict:
            last = float(self.launch_time_dict[launch_key])
            cur = float(nepi_sdk.get_time())
            launch_check = (cur - last) > float(self.NODE_LOAD_TIME_SEC)
        if launch_check is False:
            return False

        file_name = DEFAULT_NODE_FILE
        try:
            node_dict = self.drv_dict['NODE_DICT']
            if node_dict.get('file_name'):
                file_name = node_dict['file_name']
        except Exception:
            pass

        try:
            device_name = self.node_launch_name + "_" + str(launch_key).split(':')[0].replace('.','').replace('-','_')
        except:
            device_name = self.node_launch_name + "_" + str(launch_key).replace(':','_').replace('.','').replace('-','_')
        node_name = nepi_system.get_device_alias(device_name)

        dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")

        self.drv_dict['DEVICE_DICT'] = {
            'device_name': device_name,
            'device_path': host + ':' + str(port),
            'tcp_host': host,
            'tcp_port': int(port),
            'param_file': PARAM_FILE_PATH
        }
        self.drv_dict.setdefault('SAVE_DATA', {})
        self.drv_dict['SAVE_DATA']['save_rate_dict'] = self.drv_dict['SAVE_DATA'].get('save_rate_dict', {})
        self.drv_dict['SAVE_DATA']['save_data_enable'] = bool(self.drv_dict['SAVE_DATA'].get('save_data_enable', False))
        nepi_sdk.set_param(dict_param_name, self.drv_dict)

        success, msg, subp = nepi_drvs.launchDriverNode(file_name, node_name)
        if success == True:
            self.launch_time_dict[launch_key] = nepi_sdk.get_time()
            self.logger.log_warn("Launched node :" + str(node_name))
            self.active_devices_dict[launch_key] = {'node_name': node_name, 'sub_process': subp}
            success = True
        else:
            self.logger.log_warn("Failed to launch node :" + str(node_name) + " with msg :" + str(msg))
            if not self.retry:
                self.logger.log_warn("Will not try relaunch for node :" + str(node_name))
                self.dont_retry_list.append(launch_key)
            else:
                self.logger.log_warn("Will attempt relaunch for node: " + node_name + " in " + str(self.NODE_LOAD_TIME_SEC) + " secs")
        return success

    def killAllDevices(self, active_paths_list):
        purge = list(self.active_devices_dict.keys())
        for key in purge:
            entry = self.active_devices_dict[key]
            nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
            if key in active_paths_list:
                active_paths_list.remove(key)
        for key in purge:
            del self.active_devices_dict[key]
        nepi_sdk.sleep(1)
        return active_paths_list
