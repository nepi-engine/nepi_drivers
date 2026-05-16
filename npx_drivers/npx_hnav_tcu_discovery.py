#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
# Discovery script for HNav binary protocol TCP NavPose sensors.
#
# The device streams HNav binary packets over a persistent TCP
# connection.  There is no serial port to enumerate; "discovery" here means
# reading the configured host:port from the driver dictionary and launching
# the node once per unique endpoint.

import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'NPX_HNAV'
FILE_TYPE = 'DISCOVERY'

PARAM_FILE_PATH = '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_hnav_tcu_params.yaml'
DEFAULT_NODE_FILE = 'npx_hnav_node.py'


class HNavDiscovery:
    NODE_LOAD_TIME_SEC = 10

    launch_time_dict    = {}
    active_devices_dict = {}
    dont_retry_list     = []
    node_launch_name    = "hnav"
    retry               = True

    def __init__(self):
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name=self.log_name)
        time.sleep(0.2)
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")

    def discoveryFunction(self, available_paths_list, active_paths_list,
                          base_namespace, drv_dict, retry_enabled=True):
        self.drv_dict = drv_dict or {}
        self.available_paths_list = available_paths_list or []
        self.active_paths_list    = active_paths_list or []
        self.base_namespace       = base_namespace

        self.retry = retry_enabled
        if self.retry:
            self.dont_retry_list = []

        opts = (self.drv_dict.get('DISCOVERY_DICT') or {}).get('OPTIONS', {})
        host = str(opts.get('tcp_host', {}).get('value', '127.0.0.1'))
        port = int(opts.get('tcp_port', {}).get('value', 16718))

        launch_key = f"{host}:{port}"

        # Remove stale entries if the endpoint changed
        for key in [k for k in list(self.active_devices_dict.keys()) if k != launch_key]:
            entry = self.active_devices_dict[key]
            nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
            if key in self.active_paths_list:
                self.active_paths_list.remove(key)
            del self.active_devices_dict[key]

        if (launch_key not in self.active_paths_list
                and launch_key not in self.dont_retry_list):
            if self._check_for_device(launch_key):
                if self._launch_device_node(launch_key, host, port):
                    self.active_paths_list.append(launch_key)

        return self.active_paths_list

    def _check_for_device(self, launch_key):
        return True  # TCP endpoint – configuration IS discovery

    def _launch_device_node(self, launch_key, host, port):
        if launch_key in self.launch_time_dict:
            elapsed = float(nepi_sdk.get_time()) - float(self.launch_time_dict[launch_key])
            if elapsed < float(self.NODE_LOAD_TIME_SEC):
                return False

        file_name = DEFAULT_NODE_FILE
        try:
            node_dict = self.drv_dict['NODE_DICT']
            if node_dict.get('file_name'):
                file_name = node_dict['file_name']
        except Exception:
            pass

        try:
            safe_host   = host.replace('.', '').replace('-', '_')
            device_name = f"{self.node_launch_name}_{safe_host}"
        except Exception:
            device_name = f"{self.node_launch_name}_{launch_key.replace(':', '_').replace('.', '')}"

        node_name = nepi_system.get_device_alias(device_name)

        dict_param_name = nepi_sdk.create_namespace(
            self.base_namespace, node_name + "/drv_dict")

        self.drv_dict['DEVICE_DICT'] = {
            'device_name': device_name,
            'device_path': f"{host}:{port}",
            'tcp_host':    host,
            'tcp_port':    int(port),
            'param_file':  PARAM_FILE_PATH,
        }
        self.drv_dict.setdefault('SAVE_DATA', {})
        self.drv_dict['SAVE_DATA']['save_rate_dict'] = \
            self.drv_dict['SAVE_DATA'].get('save_rate_dict', {})
        self.drv_dict['SAVE_DATA']['save_data_enable'] = bool(
            self.drv_dict['SAVE_DATA'].get('save_data_enable', False))
        nepi_sdk.set_param(dict_param_name, self.drv_dict)

        success, msg, subp = nepi_drvs.launchDriverNode(file_name, node_name)
        if success:
            self.launch_time_dict[launch_key] = nepi_sdk.get_time()
            self.active_devices_dict[launch_key] = {
                'node_name':   node_name,
                'sub_process': subp,
            }
            self.logger.log_warn(f"Launched HNav node: {node_name}")
        else:
            self.logger.log_warn(f"Failed to launch {node_name}: {msg}")
            if not self.retry:
                self.dont_retry_list.append(launch_key)
            else:
                self.logger.log_warn(
                    f"Will retry {node_name} in {self.NODE_LOAD_TIME_SEC} s")
        return success

    def killAllDevices(self, active_paths_list):
        for key in list(self.active_devices_dict.keys()):
            entry = self.active_devices_dict[key]
            nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
            if key in active_paths_list:
                active_paths_list.remove(key)
            del self.active_devices_dict[key]
        nepi_sdk.sleep(1)
        return active_paths_list
