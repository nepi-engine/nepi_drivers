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
#
# The board's serial mode is stored on the board itself, and a new Maestro ships in
# "UART, detect baud rate" mode -- in which it silently ignores every command sent
# to its USB Command Port. So discovery owns the mode: the "serial_mode" option says
# how NEPI talks to the board (USB or UART), and every board found is switched to
# the matching mode over native USB before any channel node is launched on it, then
# re-checked periodically. In UART mode the channel nodes talk through the serial
# port named by "uart_port" (wired to the Maestro's RX/TX pins), while the USB
# cable stays connected for discovery and the mode switch.

import copy
import time

import serial
from serial.tools import list_ports

try:
    import usb.core  # Native USB control transfers, only for the serial-mode switch.
    import usb.util  # Needs raw USB permission (drivers_mgr runs as root).
    HAVE_PYUSB = True
except Exception:
    HAVE_PYUSB = False

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

    # Native USB protocol for the board's stored serial mode (Pololu Usc protocol;
    # the same requests were verified by hand on a Micro Maestro 6). A mode change
    # only takes effect after a reinitialize, which restarts the servo outputs, so
    # it is written only when the board reports a different mode.
    REQUEST_GET_PARAMETER = 0x81
    REQUEST_SET_PARAMETER = 0x82
    REQUEST_REINITIALIZE = 0x90
    PARAMETER_SERIAL_MODE = 3
    # USB -> USB Dual Port: the USB Command Port takes commands.
    # UART -> UART detect baud: commands arrive on RX at whatever baud the first
    # 0xAA byte uses, so the node needs no fixed-baud setting on the board.
    SERIAL_MODE_VALUES = {'USB': 0, 'UART': 2}
    SERIAL_MODE_NAMES = {0: 'USB Dual Port', 1: 'USB Chained',
                         2: 'UART detect baud', 3: 'UART fixed baud'}
    # How often a board's mode is re-read once it is right, so a board flipped by
    # other software (or swapped for a new one on the same port) gets fixed again.
    MODE_CHECK_INTERVAL_SEC = 10
    # A channel node that exited on its own (lost serial link, failed handshake)
    # is relaunched no sooner than this, so a node that keeps failing cannot spin.
    RELAUNCH_DELAY_SEC = 10

    node_launch_name = "maestro"

    # launch_id ("<node port>:ch<N>") -> {node_name, sub_process, path, node_port, channel}.
    # path is the board's USB Command Port, which is how the board is found;
    # node_port is the port the channel node talks through -- the same port in USB
    # mode, uart_port in UART mode.
    active_devices_dict = dict()
    active_paths_list = []
    dont_retry_list = []
    # path -> last resolved channel list, so the resolution is logged on change
    # rather than every pass.
    last_channels_dict = dict()
    # "<board path>|<serial>" -> {'mode', 'time'} of the last successful mode check.
    mode_checks_dict = dict()
    # key -> last message logged, for messages logged only when they change.
    last_log_dict = dict()
    # launch_id -> time its node was found to have exited on its own.
    exit_times_dict = dict()

    retry = True

    # Discovery options (populated from drv_dict each pass). All board-level --
    # anything describing an individual servo (pulse endpoints, degree range,
    # acceleration) belongs to the servo node, not here.
    channels_str = 'all'
    baud_str = '9600'
    device_number = 12
    command_port_index = 0
    serial_mode = 'USB'
    uart_port_str = 'None'


    ################################################
    def __init__(self):
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger()
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
            serial_mode = str(options.get('serial_mode', {}).get('value', 'USB')).strip().upper()
            self.serial_mode = serial_mode if serial_mode in self.SERIAL_MODE_VALUES else 'USB'
            self.uart_port_str = str(options.get('uart_port', {}).get('value', 'None')).strip()
        except Exception as e:
            self.logger.log_warn("Failed to setup options " + str(e))
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

        ### Reap channel nodes that exited on their own. A node shuts itself down
        ### when it loses its serial link, and without this its stale entry would
        ### block the relaunch below for as long as its port stays in /dev.
        for launch_id in list(self.active_devices_dict.keys()):
            sub_process = self.active_devices_dict[launch_id]['sub_process']
            if sub_process is not None and sub_process.poll() is not None:
                self.logger.log_warn("Node " + self.active_devices_dict[launch_id]['node_name'] +
                                     " exited; relaunching in " + str(self.RELAUNCH_DELAY_SEC) +
                                     "s if its board is still present")
                self.exit_times_dict[launch_id] = time.time()
                self.killDevice(launch_id)

        ### Enumerate Maestro Command Ports
        [command_ports, enumerate_ok] = self.findCommandPorts()
        if enumerate_ok == False:
            # Port enumeration failed rather than finding no boards. Reconciling
            # against an empty list here would tear down every running node and
            # respawn it on the next pass, so change nothing.
            return self.active_paths_list

        ### Put every board in the mode the serial_mode option asks for, then
        ### build the desired set of (node port, channel) pairs. The mode goes
        ### first: a channel node launched against a board in the wrong mode fails
        ### its handshake and exits, and is only retried after RELAUNCH_DELAY_SEC.
        current_keys = [p['path'] + '|' + str(p['serial_number']) for p in command_ports]
        for check_key in list(self.mode_checks_dict.keys()):
            if check_key not in current_keys:
                del self.mode_checks_dict[check_key]
        desired_dict = dict()
        uart_port_taken = False
        for port_info in sorted(command_ports, key = lambda p: str(p['serial_number'])):
            self.checkSerialMode(port_info)
            node_port = port_info['path']
            if self.serial_mode == 'UART':
                if uart_port_taken == True:
                    # Every channel node of one board shares the one UART line; a
                    # second board would put the same channel numbers on it.
                    self.logOnChange('uart_extra:' + port_info['path'],
                                     "UART mode drives one Maestro; ignoring the board on " +
                                     port_info['path'] + " (serial " + str(port_info['serial_number']) + ")")
                    continue
                node_port = self.resolveUartPort(command_ports)
                if node_port is None:
                    continue
                uart_port_taken = True
            for channel in self.resolveChannels(port_info):
                launch_id = node_port + ":ch" + str(channel)
                desired_dict[launch_id] = {
                    'port_info': port_info,
                    'node_port': node_port,
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
            if launch_id in self.exit_times_dict:
                if time.time() - self.exit_times_dict[launch_id] < self.RELAUNCH_DELAY_SEC:
                    continue
                del self.exit_times_dict[launch_id]
            self.launchDeviceNode(entry['port_info'], entry['channel'], entry['node_port'])

        ### Claim each board's path, and the UART port its nodes talk through,
        ### once any of its channels is live. Claiming the UART port keeps other
        ### drivers' serial probes off a line that commands servos.
        for entry in self.active_devices_dict.values():
            for claim_path in [entry['path'], entry['node_port']]:
                if claim_path not in self.active_paths_list:
                    self.active_paths_list.append(claim_path)
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
                'all_paths': [p.device for p in ports_sorted],
                'serial_number': getattr(chosen, 'serial_number', None) or 'Unknown',
                'channel_count': channel_count
            })
        return command_ports, True


    def logOnChange(self, key, msg, warn = True):
        # Discovery runs every few seconds, so a standing condition is logged once
        # when it appears or changes rather than on every pass.
        if self.last_log_dict.get(key) == msg:
            return
        self.last_log_dict[key] = msg
        if warn == True:
            self.logger.log_warn(msg)
        else:
            self.logger.log_info(msg)


    def checkSerialMode(self, port_info):
        # Make sure the board is in the serial mode the serial_mode option asks for,
        # switching it if not. Re-checked every MODE_CHECK_INTERVAL_SEC, or at once
        # when the option changes. Best-effort: when the mode cannot be read the
        # nodes are still launched, which is exactly what happened before this
        # check existed, and the reason is logged.
        path_str = port_info['path']
        # Keyed by serial number too, so a different board appearing on the same
        # port is checked straight away rather than riding the old board's result.
        check_key = path_str + '|' + str(port_info['serial_number'])
        wanted_mode = self.SERIAL_MODE_VALUES[self.serial_mode]
        last_check = self.mode_checks_dict.get(check_key)
        if (last_check is not None and last_check['mode'] == wanted_mode and
                (time.time() - last_check['time']) < self.MODE_CHECK_INTERVAL_SEC):
            return True

        log_key = 'mode:' + path_str
        board_str = "Maestro " + str(port_info['serial_number']) + " on " + path_str
        wanted_name = self.SERIAL_MODE_NAMES[wanted_mode]
        if HAVE_PYUSB == False:
            self.logOnChange(log_key, "pyusb not available; cannot check or set the serial mode of " +
                             board_str + " (needs " + wanted_name + ")")
            return False
        dev = self.findUsbDevice(port_info)
        if dev is None:
            self.logOnChange(log_key, "Could not find " + board_str + " on native USB to check its serial mode")
            return False
        try:
            mode = self.getUsbParameter(dev, self.PARAMETER_SERIAL_MODE)
            if mode != wanted_mode:
                self.logger.log_warn(board_str + " is in " + self.SERIAL_MODE_NAMES.get(mode, str(mode)) +
                                     " mode; switching it to " + wanted_name)
                dev.ctrl_transfer(0x40, self.REQUEST_SET_PARAMETER, wanted_mode,
                                  (1 << 8) | self.PARAMETER_SERIAL_MODE)
                dev.ctrl_transfer(0x40, self.REQUEST_REINITIALIZE, 0, 0)
                time.sleep(0.2)
                mode = self.getUsbParameter(dev, self.PARAMETER_SERIAL_MODE)
                if mode != wanted_mode:
                    self.logOnChange(log_key, board_str + " still reports " +
                                     self.SERIAL_MODE_NAMES.get(mode, str(mode)) +
                                     " after switching to " + wanted_name)
                    return False
            self.logOnChange(log_key, board_str + " serial mode: " + wanted_name, warn = False)
            self.mode_checks_dict[check_key] = {'mode': wanted_mode, 'time': time.time()}
            return True
        except Exception as e:
            self.logOnChange(log_key, "Failed to check or set the serial mode of " + board_str + ": " + str(e))
            return False
        finally:
            try:
                usb.util.dispose_resources(dev)
            except Exception:
                pass


    def getUsbParameter(self, dev, parameter):
        # One-byte board parameter (vendor IN request to the device, so no
        # interface claim -- the kernel's serial driver stays bound throughout).
        data = dev.ctrl_transfer(0xC0, self.REQUEST_GET_PARAMETER, 0, parameter, 1)
        return int(data[0])


    def findUsbDevice(self, port_info):
        # The serial port and the native USB device are two views of one board;
        # the USB serial number ties them together. A board that reports no serial
        # number can only be matched when it is the only Maestro attached.
        serial_number = str(port_info.get('serial_number', 'Unknown'))
        maestros = []
        for dev in usb.core.find(idVendor = self.POLOLU_VENDOR_ID, find_all = True):
            if dev.idProduct not in self.MAESTRO_CHANNEL_COUNTS:
                continue
            maestros.append(dev)
            try:
                if serial_number != 'Unknown' and str(dev.serial_number) == serial_number:
                    return dev
            except Exception:
                pass
        if serial_number == 'Unknown' and len(maestros) == 1:
            return maestros[0]
        return None


    def resolveUartPort(self, command_ports):
        # The uart_port option holds a port basename ("ttyTHS0"), as listed by
        # drivers_mgr's SERIAL_DEVICES options. Returns the full path, or None when
        # no usable port is set -- the board has still been switched to UART mode,
        # but there is nothing to launch channel nodes on.
        port_str = str(self.uart_port_str).strip()
        if port_str in ['', 'None', 'SERIAL_DEVICES']:
            self.logOnChange('uart_port', "UART mode selected but no uart_port set; " +
                             "no servo nodes launched until one is chosen")
            return None
        if port_str.startswith('/') == False:
            port_str = '/dev/' + port_str
        for port_info in command_ports:
            if port_str in port_info['all_paths']:
                # In UART mode the board ignores its USB serial ports entirely.
                self.logOnChange('uart_port', "uart_port " + port_str + " is one of the Maestro's own USB " +
                                 "ports, which it ignores in UART mode; choose the port wired to its RX/TX pins")
                return None
        self.logOnChange('uart_port', "UART mode: servo nodes use " + port_str + " at " +
                         str(self.baud_str) + " baud", warn = False)
        return port_str


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


    def launchDeviceNode(self, port_info, channel, node_port):
        path_str = port_info['path']
        launch_id = node_port + ":ch" + str(channel)
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
        # The device name stays keyed by the board's serial number in either serial
        # mode, so switching modes keeps each servo's saved config.
        node_drv_dict = copy.deepcopy(self.drv_dict)
        node_drv_dict['DEVICE_DICT'] = {
            'device_name': device_name,
            'device_path': node_port,
            'serial_mode': self.serial_mode,
            'channel': channel,
            'baud_str': self.baud_str,
            'device_number': self.device_number,
            'serial_number': port_info['serial_number']
        }
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
        nepi_sdk.set_param(dict_param_name, node_drv_dict)

        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = node_port)
        if success:
            self.active_devices_dict[launch_id] = {
                'node_name': node_name,
                'sub_process': sub_process,
                'path': path_str,
                'node_port': node_port,
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
        node_port = entry['node_port']
        self.logger.log_info("Killing node: " + node_name)
        nepi_drvs.killDriverNode(node_name, sub_process)
        del self.active_devices_dict[launch_id]
        # Drop each claimed path from the active list only once no remaining
        # channel node still uses it.
        still_active = any(e['path'] == path_str for e in self.active_devices_dict.values())
        if not still_active:
            if path_str in self.active_paths_list:
                self.active_paths_list.remove(path_str)
            if path_str in self.last_channels_dict:
                del self.last_channels_dict[path_str]
        port_active = any(node_port in [e['path'], e['node_port']] for e in self.active_devices_dict.values())
        if not port_active and node_port in self.active_paths_list:
            self.active_paths_list.remove(node_port)


    def killAllDevices(self, active_paths_list):
        for launch_id in list(self.active_devices_dict.keys()):
            entry = self.active_devices_dict[launch_id]
            node_name = entry['node_name']
            sub_process = entry['sub_process']
            path_str = entry['path']
            if self.retry == False:
                self.dont_retry_list.append(path_str)
            nepi_drvs.killDriverNode(node_name, sub_process)
            for claim_path in [path_str, entry['node_port']]:
                if claim_path in active_paths_list:
                    active_paths_list.remove(claim_path)
                if claim_path in self.active_paths_list:
                    self.active_paths_list.remove(claim_path)
        self.active_devices_dict = dict()
        self.last_channels_dict = dict()
        self.exit_times_dict = dict()
        # A re-enabled driver re-checks every board's mode straight away.
        self.mode_checks_dict = dict()
        nepi_sdk.sleep(1)
        return active_paths_list


if __name__ == '__main__':
    SvxServoMaestroDiscovery()
