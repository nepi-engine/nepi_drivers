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


import os
import subprocess
import copy

import requests

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

from nepi_api.messages_if import MsgIF


PKG_NAME = 'IDX_LIMELIGHT4_USB'
FILE_TYPE = 'DISCOVERY'


class Limelight4UsbCamDiscovery:

    NODE_LOAD_TIME_SEC = 10
    CHECK_INTERVAL_S = 3.0

    launch_time_dict = dict()
    check_for_devices = True

    drv_dict = dict()
    deviceList = []

    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"

    def __init__(self):
        #### NODE Initialization ####
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        ########################
        # Update discovery options
        success = self.updateDiscoveryOptions()
        if not success:
            self.msg_if.pub_warn("Shutting down because failed to get Driver Dict")
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because failed to get Driver Dict")
            return

        ########################
        # Start node processes
        nepi_sdk.start_timer_process(1, self.detectAndManageDevices, oneshot=True)
        nepi_sdk.on_shutdown(self.cleanup_actions)

        ########################
        self.msg_if.pub_info("Initialization Complete")
        nepi_sdk.spin()

    #**********************
    # Discovery functions

    def updateDiscoveryOptions(self):
        success = False
        last_drv_dict = copy.deepcopy(self.drv_dict)
        self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
        if len(list(self.drv_dict.keys())) == 0:
            self.msg_if.pub_warn("Failed to load Driver dict")
            return success
        if 'DISCOVERY_DICT' not in self.drv_dict.keys():
            self.msg_if.pub_warn("Discovery dict missing in Driver dict")
            return success
        if last_drv_dict != self.drv_dict:
            self.msg_if.pub_warn("Updated Driver Dict: " + str(self.drv_dict))
        success = True
        return success

    def detectAndManageDevices(self, timer):
        if not self.check_for_devices:
            self.msg_if.pub_warn("Stopping device discovery process")
            return

        # Read all network interface names
        try:
            all_ifaces = os.listdir('/sys/class/net/')
        except Exception as e:
            self.msg_if.pub_warn("Failed to list /sys/class/net/: " + str(e))
            nepi_sdk.start_timer_process(self.CHECK_INTERVAL_S, self.detectAndManageDevices, oneshot=True)
            return

        enx_ifaces = [i for i in all_ifaces if i.startswith('enx')]

        # Purge entries whose interface has disappeared
        purge_list = []
        for device in list(self.deviceList):
            if device['iface'] not in all_ifaces:
                purge_list.append(device['node_namespace'])
        for ns in purge_list:
            self.msg_if.pub_info("Interface no longer present; stopping node " + ns)
            self.stopAndPurgeDeviceNode(ns)

        # Process enx interfaces
        for iface in enx_ifaces:
            # Check if already managed
            already_known = any(d['iface'] == iface for d in self.deviceList)
            if already_known:
                # If the node process has died, remove it so re-detection fires next cycle
                for device in list(self.deviceList):
                    if device['iface'] == iface:
                        if not self.deviceNodeIsRunning(device['node_namespace']):
                            self.msg_if.pub_warn("Node for " + iface + " has died; queuing for re-detection")
                            self.stopAndPurgeDeviceNode(device['node_namespace'])
                        break
                continue

            # Get gateway IP for this interface
            gateway_ip = self.getGatewayIp(iface)
            if gateway_ip is None:
                continue

            # Probe REST API to confirm this is a Limelight
            try:
                resp = requests.get("http://" + gateway_ip + ":5807/results", timeout=1)
                resp.raise_for_status()
                data = resp.json()
            except Exception:
                continue

            if 'hwtype' not in data:
                continue

            # Confirmed Limelight — apply NODE_LOAD_TIME_SEC backoff before launching
            launch_id = iface
            if launch_id in self.launch_time_dict:
                elapsed = nepi_utils.get_time() - self.launch_time_dict[launch_id]
                if elapsed < self.NODE_LOAD_TIME_SEC:
                    continue

            # Build unique node name from interface identifier
            iface_id = iface.replace('enx', '')
            device_name = 'limelight4_usb_' + iface_id
            device_name = nepi_utils.get_clean_name(device_name)
            node_name = nepi_system.get_device_alias(device_name)
            node_namespace = os.path.join(self.base_namespace, node_name)

            # Set drv_dict on param server for the node
            self.drv_dict['DEVICE_DICT'] = {'device_name': 'limelight4_usb', 'device_path': gateway_ip}
            dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
            nepi_sdk.set_param(dict_param_name, self.drv_dict)

            # Launch node
            file_name = self.drv_dict['NODE_DICT']['file_name']
            self.msg_if.pub_info(
                "Launching Limelight4 node for iface=" + iface + " gateway=" + gateway_ip
            )
            [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name)
            self.launch_time_dict[launch_id] = nepi_utils.get_time()

            if success:
                self.deviceList.append({
                    'device_class': 'limelight4_usb',
                    'device_path': gateway_ip,
                    'iface': iface,
                    'node_name': node_name,
                    'node_namespace': node_namespace,
                    'node_subprocess': sub_process,
                })
                self.msg_if.pub_warn("Started Limelight4 node: " + node_name)
                self.msg_if.pub_warn("Updated Active Device List " + str(self.deviceList))
            else:
                self.msg_if.pub_warn("Failed to launch Limelight4 node for " + iface + ": " + msg)

        # Schedule next check
        nepi_sdk.start_timer_process(self.CHECK_INTERVAL_S, self.detectAndManageDevices, oneshot=True)

    def getGatewayIp(self, iface):
        try:
            sub_process = subprocess.Popen(
                ['ip', 'addr', 'show', iface],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
            )
            stdout, _ = sub_process.communicate()
            for line in stdout.splitlines():
                line = line.strip()
                if line.startswith('inet ') and '/' in line:
                    ip = line.split()[1].split('/')[0]
                    parts = ip.split('.')
                    if len(parts) == 4:
                        parts[3] = '1'
                        return '.'.join(parts)
        except Exception as e:
            self.msg_if.pub_warn("Failed to get IP for " + iface + ": " + str(e))
        return None

    def stopAndPurgeDeviceNode(self, node_namespace='All'):
        success = False
        if len(self.deviceList) > 0:
            if node_namespace == 'All':
                self.check_for_devices = False
            for device in list(self.deviceList):
                namespace = device['node_namespace']
                node_name = os.path.basename(namespace)
                if namespace == node_namespace or node_namespace == 'All':
                    sub_process = device['node_subprocess']
                    self.msg_if.pub_info("Killing device node: " + node_name)
                    self.deviceList.remove(device)
                    success = nepi_drvs.killDriverNode(node_name, sub_process)
                    if not success:
                        self.msg_if.pub_warn("Unable to kill device node " + node_name)
                    else:
                        self.msg_if.pub_warn("Node killed. Removed device from active list " + node_name)
            self.msg_if.pub_warn("Updated Active Device List " + str(self.deviceList))
        return success

    def deviceNodeIsRunning(self, node_namespace):
        for device in self.deviceList:
            if device['node_namespace'] == node_namespace:
                if device['node_subprocess'].poll() is not None:
                    return False
                else:
                    return True
        self.msg_if.pub_warn("Cannot check run status of unknown node " + node_namespace)
        return False

    def cleanup_actions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        self.stopAndPurgeDeviceNode('All')


if __name__ == '__main__':
    node = Limelight4UsbCamDiscovery()
