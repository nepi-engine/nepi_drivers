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

# SVX (servo) discovery for the Pololu Micro Maestro USB servo controller.
#
# Structure mirrors svx_servo_generic_discovery.py (the SVX template) and the
# serial discovery pattern in ptx_sidus_ss109_serial_discovery.py. The stub
# detection in the generic discovery is replaced here with a real USB match on
# the Pololu vendor/product IDs.
#
# One SVX device = one servo = one Maestro channel. A Maestro is a single USB
# device exposing two virtual serial ports (a Command Port and a TTL Port); we
# send native commands to the Command Port. For each detected board we launch one
# SVX node per configured servo channel, all sharing that one Command Port. The
# node serializes port access with an advisory file lock, so several channel
# nodes on one board coexist safely.
#
# The Maestro is open loop -- there is no feedback wire, so the board cannot tell
# whether a servo is physically attached to a channel. Which channels carry
# servos is therefore user-declared config (the "channels" option), never
# auto-detected.
#
# Unlike the usual one-device-per-path NEPI drivers, one path here backs several
# devices, so this discovery reconciles a desired set of (path, channel) pairs
# against what is running on every pass rather than claiming a path once. That is
# what lets the "channels" option be edited live in the RUI: adding a channel
# launches just that node, removing one kills just that node, and reordering the
# option changes nothing.

import copy
import time

import serial
from serial.tools import list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'SVX_SERVO_MAESTRO'
FILE_TYPE = 'DISCOVERY'


class SvxServoMaestroDiscovery:

    # Pololu USB vendor ID and the Maestro product IDs mapped to their servo
    # channel counts. Both virtual serial ports of a board share the same
    # VID/PID; they differ by USB interface number, which is how we pick the
    # Command Port below.
    POLOLU_VENDOR_ID = 0x1FFB
    MAESTRO_CHANNEL_COUNTS = {
        0x0089: 6,   # Micro Maestro 6-Channel
        0x008A: 12,  # Mini Maestro 12-Channel
        0x008B: 18,  # Mini Maestro 18-Channel
        0x008C: 24   # Mini Maestro 24-Channel
    }
    # Used when a board reports no usable PID. The Micro is the smallest board,
    # so assuming it never invents channels that do not exist.
    DEFAULT_CHANNEL_COUNT = 6

    node_launch_name = "maestro"

    # launch_id ("<port>:ch<N>") -> {node_name, sub_process, path, channel}
    active_devices_dict = dict()
    active_paths_list = []
    dont_retry_list = []
    # path -> last resolved channel list, so the resolution is logged on change
    # rather than every pass.
    last_channels_dict = dict()

    retry = True

    # Discovery options (populated from drv_dict each pass). All board-level --
    # anything describing an individual servo (pulse endpoints, degree range,
    # acceleration) belongs to the servo node, not here.
    channels_str = 'all'
    baud_str = '9600'
    device_number = 12
    command_port_index = 0


    ################################################
    def __init__(self):
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name = self.log_name)
        time.sleep(1)
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")


    ##########  DRV Standard Discovery Function
    def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled = True):
        self.drv_dict = drv_dict
        self.available_paths_list = available_paths_list
        self.active_paths_list = active_paths_list
        self.base_namespace = base_namespace

        ##################################
        # Get discovery options
        try:
            options = self.drv_dict.get('DISCOVERY_DICT', {}).get('OPTIONS', {})
            self.channels_str = str(options.get('channels', {}).get('value', 'all'))
            self.baud_str = str(options.get('baud_rate', {}).get('value', '9600'))
            self.device_number = int(options.get('device_number', {}).get('value', 12))
            self.command_port_index = int(options.get('command_port_index', {}).get('value', 0))
        except Exception as e:
            self.logger.log_warn(self.log_name + ": Failed to setup options " + str(e))
            return self.active_paths_list

        # Retry behavior
        self.retry = retry_enabled
        if self.retry == True:
            self.dont_retry_list = []

        ### Purge nodes whose Command Port has disappeared from /dev.
        # Kept separate from the desired-set reconciliation below so a board
        # vanishing and a port-enumeration hiccup stay distinguishable.
        path_purge_list = []
        for launch_id in list(self.active_devices_dict.keys()):
            path_str = self.active_devices_dict[launch_id]['path']
            if path_str not in self.available_paths_list:
                path_purge_list.append(launch_id)
        for launch_id in path_purge_list:
            self.killDevice(launch_id)

        ### Enumerate Maestro Command Ports
        [command_ports, enumerate_ok] = self.findCommandPorts()
        if enumerate_ok == False:
            # Port enumeration failed rather than finding no boards. Reconciling
            # against an empty list here would tear down every running node and
            # respawn it on the next pass, so change nothing.
            return self.active_paths_list

        ### Build the desired set of (path, channel) pairs
        desired_dict = dict()
        for port_info in command_ports:
            path_str = port_info['path']
            for channel in self.resolveChannels(port_info):
                launch_id = path_str + ":ch" + str(channel)
                desired_dict[launch_id] = {
                    'port_info': port_info,
                    'channel': channel
                }

        ### Kill any running channel that is no longer wanted (channel removed
        ### from the option, or its board is gone).
        for launch_id in list(self.active_devices_dict.keys()):
            if launch_id not in desired_dict:
                self.killDevice(launch_id)

        ### Launch any wanted channel that is not running yet
        for launch_id in sorted(desired_dict.keys()):
            if launch_id in self.active_devices_dict:
                continue
            entry = desired_dict[launch_id]
            if entry['port_info']['path'] in self.dont_retry_list:
                continue
            self.launchDeviceNode(entry['port_info'], entry['channel'])

        ### Claim each board's path once any of its channels is live
        for port_info in command_ports:
            path_str = port_info['path']
            still_active = any(e['path'] == path_str for e in self.active_devices_dict.values())
            if still_active and path_str not in self.active_paths_list:
                self.active_paths_list.append(path_str)
        return self.active_paths_list
    ################################################


    ##########  Device specific calls

    def resolveChannels(self, port_info):
        # Turn the "channels" option into this board's channel list, validated
        # against how many channels the board actually has. A stale or nonsense
        # value must never silently narrow discovery to a subset -- anything
        # unusable falls back to every channel, loudly.
        channel_count = port_info['channel_count']
        path_str = port_info['path']

        channels_str = str(self.channels_str).strip().lower()
        if channels_str == 'all':
            resolved = list(range(0, channel_count))
        else:
            parsed = self.parseChannels(channels_str)
            resolved = []
            for channel in parsed:
                if 0 <= channel < channel_count:
                    resolved.append(channel)
                else:
                    self.logger.log_warn("Ignoring channel " + str(channel) + " for " + path_str +
                                         ": board has " + str(channel_count) + " channels (0-" +
                                         str(channel_count - 1) + ")")
            if len(resolved) == 0:
                self.logger.log_warn("No usable channels in channels option '" + str(self.channels_str) +
                                     "' for " + path_str + "; falling back to all " +
                                     str(channel_count) + " channels")
                resolved = list(range(0, channel_count))

        if self.last_channels_dict.get(path_str) != resolved:
            self.logger.log_info("Channels for " + path_str + ": " + str(resolved))
            self.last_channels_dict[path_str] = list(resolved)
        return resolved


    def parseChannels(self, channels_str):
        # "0" -> [0]; "0,1" -> [0,1]; "5,0,4" -> [5,0,4]. Written order is
        # preserved and duplicates are dropped. Returns [] when nothing parses;
        # the caller decides the fallback (never silently narrow to one channel).
        # Out-of-range values are left in place for resolveChannels() to reject
        # against the real board so the operator gets told which value was bad.
        channels = []
        for tok in str(channels_str).replace(';', ',').split(','):
            tok = tok.strip()
            if tok == '':
                continue
            try:
                channel = int(tok)
            except Exception:
                self.logger.log_warn("Ignoring unparseable channels entry: '" + tok + "'")
                continue
            if channel not in channels:
                channels.append(channel)
        return channels


    def findCommandPorts(self):
        # Enumerate USB serial ports, keep the Pololu Maestro matches, group the
        # two ports of each physical board by serial number, and return the
        # Command Port of each board (the lowest USB interface number, or the
        # command_port_index-th when a board exposes them in a fixed order).
        # Returns [ports, ok]; ok is False only when enumeration itself failed,
        # which the caller must not confuse with "no boards attached".
        matches = []
        try:
            for p in list_ports.comports():
                vid = getattr(p, 'vid', None)
                pid = getattr(p, 'pid', None)
                if vid == self.POLOLU_VENDOR_ID and (pid in self.MAESTRO_CHANNEL_COUNTS or pid is None):
                    matches.append(p)
        except Exception as e:
            self.logger.log_warn("Failed to enumerate serial ports: " + str(e))
            return [], False

        # Group by serial number (falls back to the location prefix if the board
        # reports no serial number).
        boards = dict()
        for p in matches:
            key = getattr(p, 'serial_number', None)
            if key is None:
                loc = getattr(p, 'location', None) or getattr(p, 'device', '')
                key = str(loc).split(':')[0]
            boards.setdefault(key, []).append(p)

        command_ports = []
        for key, ports in boards.items():
            ports_sorted = sorted(ports, key = self.interfaceSortKey)
            idx = self.command_port_index
            if idx < 0 or idx >= len(ports_sorted):
                idx = 0
            chosen = ports_sorted[idx]
            pid = getattr(chosen, 'pid', None)
            channel_count = self.MAESTRO_CHANNEL_COUNTS.get(pid, self.DEFAULT_CHANNEL_COUNT)
            command_ports.append({
                'path': chosen.device,
                'serial_number': getattr(chosen, 'serial_number', None) or 'Unknown',
                'channel_count': channel_count
            })
        return command_ports, True


    def interfaceSortKey(self, port):
        # Sort a board's ports by USB interface number so the Command Port
        # (interface 0) sorts first. location looks like "1-1.2:1.0"; the trailing
        # ".0" is the interface. Falls back to the device string.
        loc = getattr(port, 'location', None)
        if loc is not None and ':' in loc:
            try:
                return (0, int(loc.split(':')[-1].split('.')[-1]))
            except Exception:
                pass
        return (1, str(getattr(port, 'device', '')))


    def boardIdString(self, port_info):
        # Board identifier used in the device name. The serial number is stable
        # across reboots; the port basename ("ttyACM0") is not, and the device
        # name keys the NEPI device alias and its saved config -- so a reboot
        # that renumbers the ports must not reattach a saved config to a
        # different servo. Falls back to the port basename only when the board
        # reports no serial number.
        serial_number = port_info.get('serial_number', None)
        if serial_number is None or str(serial_number).strip() in ['', 'Unknown']:
            board_id = port_info['path'].split('/')[-1]
        else:
            board_id = str(serial_number).strip()
        # Node names must be plain identifiers
        safe_id = ''.join(c if (c.isalnum() or c == '_') else '_' for c in board_id)
        return safe_id


    def launchDeviceNode(self, port_info, channel):
        path_str = port_info['path']
        launch_id = path_str + ":ch" + str(channel)
        if launch_id in self.active_devices_dict:
            return True

        file_name = self.drv_dict['NODE_DICT']['file_name']
        device_name = self.node_launch_name + "_" + self.boardIdString(port_info) + "_ch" + str(channel)
        node_name = nepi_system.get_device_alias(device_name)
        self.logger.log_info("Launching node: " + node_name + " on channel " + str(channel))

        # Setup required param server drv_dict for the node. Deep-copied per
        # launch so the DEVICE_DICT of one channel can never leak into another's.
        #
        # Only board-level facts go in here. The servo's own calibration (pulse
        # endpoints, degree range, acceleration) is owned by the node as its own
        # settings, persisted per device -- discovery must not overwrite an
        # operator's calibration every time it relaunches a channel.
        node_drv_dict = copy.deepcopy(self.drv_dict)
        node_drv_dict['DEVICE_DICT'] = {
            'device_name': device_name,
            'device_path': path_str,
            'channel': channel,
            'baud_str': self.baud_str,
            'device_number': self.device_number,
            'serial_number': port_info['serial_number']
        }
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
        nepi_sdk.set_param(dict_param_name, node_drv_dict)

        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = path_str)
        if success:
            self.active_devices_dict[launch_id] = {
                'node_name': node_name,
                'sub_process': sub_process,
                'path': path_str,
                'channel': channel
            }
            self.logger.log_info("Launched node: " + node_name)
        else:
            self.logger.log_info("Failed to launch node: " + node_name + " with msg: " + msg)
            if self.retry == False:
                self.dont_retry_list.append(path_str)
        return success


    def killDevice(self, launch_id):
        if launch_id not in self.active_devices_dict:
            return
        entry = self.active_devices_dict[launch_id]
        node_name = entry['node_name']
        sub_process = entry['sub_process']
        path_str = entry['path']
        self.logger.log_info("Killing node: " + node_name)
        nepi_drvs.killDriverNode(node_name, sub_process)
        del self.active_devices_dict[launch_id]
        # Drop the path from active list only once none of its channels remain.
        still_active = any(e['path'] == path_str for e in self.active_devices_dict.values())
        if not still_active:
            if path_str in self.active_paths_list:
                self.active_paths_list.remove(path_str)
            if path_str in self.last_channels_dict:
                del self.last_channels_dict[path_str]


    def killAllDevices(self, active_paths_list):
        for launch_id in list(self.active_devices_dict.keys()):
            entry = self.active_devices_dict[launch_id]
            node_name = entry['node_name']
            sub_process = entry['sub_process']
            path_str = entry['path']
            if self.retry == False:
                self.dont_retry_list.append(path_str)
            nepi_drvs.killDriverNode(node_name, sub_process)
            if path_str in active_paths_list:
                active_paths_list.remove(path_str)
            if path_str in self.active_paths_list:
                self.active_paths_list.remove(path_str)
        self.active_devices_dict = dict()
        self.last_channels_dict = dict()
        nepi_sdk.sleep(1)
        return active_paths_list


if __name__ == '__main__':
    SvxServoMaestroDiscovery()
