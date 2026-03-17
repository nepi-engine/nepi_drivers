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
#
# Simulator support:
#   When simulate_hnav is True in the OPTIONS, the discovery script starts a
#   local TCP server on the configured host:port that generates valid HNav
#   binary packets from the sim_* parameters below.  The node then connects
#   to that server exactly as it would to real hardware.

import math
import socket
import struct
import threading
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'NPX_HNAV'
FILE_TYPE = 'DISCOVERY'

PARAM_FILE_PATH = '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_hnav_tcu_params.yaml'
DEFAULT_NODE_FILE = 'npx_hnav_node.py'

# ---------------------------------------------------------------------------
# HNav packet builder helpers (mirror of node decoder constants)
# ---------------------------------------------------------------------------
_PACKET_SIZE        = 67
_START_BYTE1        = 0xAA
_START_BYTE2        = 0xBF
_PROTO_VER          = 0x00
_MESSAGE_ID_LO      = 0x00
_MESSAGE_ID_HI      = 0x00
_DATA_SIZE_LO       = 0x37   # 55
_DATA_SIZE_HI       = 0x00
_DATA_SIZE          = 55

_LAT_SCALE          = 90.0  / 2147483648.0
_LON_SCALE          = 180.0 / 2147483648.0
_DEPTH_SCALE        = 0.001
_ALT_SCALE          = 0.01
_ORIENT_SCALE       = 0.01
_VEL_SCALE          = 0.001
_SOUND_VEL_SCALE    = 0.1
_TEMP_SCALE         = 0.01

# Status: position valid, heading valid, depth valid, altitude valid,
#         velocity valid – all valid (no invalid bits set), hybrid mode on.
_DEFAULT_STATUS     = 0x0002  # hybrid mode bit only; all validity bits clear

# Data struct format (same as node decoder)
_DATA_FMT = '<BQiiiHhhHhhhhhhHhfHHH'


def _reflect(value, bit_num):
    out = 0
    bit = 1
    for i in range(bit_num - 1, -1, -1):
        if value & (1 << i):
            out |= bit
        bit <<= 1
    return out


def _crc16(data_bytes):
    """Compute the HNav CRC-16 over the provided bytes (excludes the CRC field)."""
    crc = 0xFFFF
    for byte in data_bytes:
        current = _reflect(byte, 8)
        j = 0x80
        while j:
            bit = crc & 0x8000
            crc = (crc << 1) & 0xFFFF
            if current & j:
                bit ^= 0x8000
            if bit:
                crc ^= 0x1021
            j >>= 1
    crc = _reflect(crc, 16)
    crc ^= 0xFFFF
    crc &= 0xFFFF
    return crc


def build_hnav_packet(lat_deg, lon_deg, depth_m, alt_m,
                      roll_deg, pitch_deg, heading_deg,
                      vel_fwd_ms=0.0, vel_stbd_ms=0.0, vel_down_ms=0.0,
                      ang_fwd_dps=0.0, ang_stbd_dps=0.0, ang_down_dps=0.0,
                      sound_vel_ms=1500.0, temp_c=10.0,
                      pos_qual_m=1.0, head_qual_deg=0.1, vel_qual_mms=10.0,
                      status=_DEFAULT_STATUS):
    """
    Build a valid 67-byte HNav binary packet from engineering-unit inputs.
    Returns a bytes object ready to send over TCP.
    """
    utc_us = int(time.time() * 1e6)

    lat_raw   = int(lat_deg   / _LAT_SCALE)
    lon_raw   = int(lon_deg   / _LON_SCALE)
    depth_raw = int(depth_m   / _DEPTH_SCALE)
    alt_raw   = max(0, int(alt_m / _ALT_SCALE))
    roll_raw  = int(roll_deg  / _ORIENT_SCALE)
    pitch_raw = int(pitch_deg / _ORIENT_SCALE)
    head_raw  = int((heading_deg % 360.0) / _ORIENT_SCALE)
    vfwd_raw  = int(vel_fwd_ms  / _VEL_SCALE)
    vstbd_raw = int(vel_stbd_ms / _VEL_SCALE)
    vdown_raw = int(vel_down_ms / _VEL_SCALE)
    afwd_raw  = int(ang_fwd_dps  / 0.01)
    astbd_raw = int(ang_stbd_dps / 0.01)
    adown_raw = int(ang_down_dps / 0.01)
    svel_raw  = int(sound_vel_ms / _SOUND_VEL_SCALE)
    temp_raw  = int(temp_c / _TEMP_SCALE)
    hqual_raw = int(head_qual_deg / 0.01)
    vqual_raw = int(vel_qual_mms)

    # Clamp 16-bit signed fields
    def clamp16s(v): return max(-32768, min(32767, v))
    def clamp16u(v): return max(0, min(65535, v))

    data = struct.pack(
        _DATA_FMT,
        0,                          # HNav version
        utc_us & 0xFFFFFFFFFFFFFFFF,# UTC microseconds (uint64)
        lat_raw,                    # latitude (int32)
        lon_raw,                    # longitude (int32)
        depth_raw,                  # depth (int32)
        clamp16u(alt_raw),          # altitude (uint16)
        clamp16s(roll_raw),         # roll (int16)
        clamp16s(pitch_raw),        # pitch (int16)
        clamp16u(head_raw),         # heading (uint16)
        clamp16s(vfwd_raw),         # vel fwd (int16)
        clamp16s(vstbd_raw),        # vel stbd (int16)
        clamp16s(vdown_raw),        # vel down (int16)
        clamp16s(afwd_raw),         # ang fwd (int16)
        clamp16s(astbd_raw),        # ang stbd (int16)
        clamp16s(adown_raw),        # ang down (int16)
        clamp16u(svel_raw),         # sound velocity (uint16)
        clamp16s(temp_raw),         # temperature (int16)
        float(pos_qual_m),          # position quality CEP50 (float32)
        clamp16u(hqual_raw),        # heading quality (uint16)
        clamp16u(vqual_raw),        # velocity quality (uint16)
        clamp16u(status),           # status flags (uint16)
    )

    # 10-byte header (start + proto_ver + msg_id + data_size + 3 reserved)
    header = bytes([
        _START_BYTE1, _START_BYTE2,
        _PROTO_VER,
        _MESSAGE_ID_LO, _MESSAGE_ID_HI,
        _DATA_SIZE_LO, _DATA_SIZE_HI,
        0x00, 0x00, 0x00,           # reserved bytes
    ])

    body = header + data            # 65 bytes, CRC covers all of these
    crc  = _crc16(body)
    return body + struct.pack('<H', crc)


class HNavDiscovery:
    NODE_LOAD_TIME_SEC = 10

    launch_time_dict    = {}
    active_devices_dict = {}
    dont_retry_list     = []
    node_launch_name    = "hnav"
    retry               = True

    # Simulator tracking: key = "host:port" -> {'thread': t, 'stop': Event}
    _sim_threads = {}
    _sim_cfg     = None

    def __init__(self):
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name=self.log_name)
        time.sleep(0.2)
        self.logger.log_info("Starting Initialization")
        self._saved_tcp_host = None   # preserves user's tcp_host while sim is active
        self.logger.log_info("Initialization Complete")

    # -------------------------------------------------------------------------
    def discoveryFunction(self, available_paths_list, active_paths_list,
                          base_namespace, drv_dict, retry_enabled=True):
        """
        TCP-based discovery: pull host/port from drv_dict and ensure the
        HNav node is running for that endpoint.  Optionally start a built-in
        HNav simulator server before launching the node.
        """
        self.drv_dict = drv_dict or {}
        self.available_paths_list = available_paths_list or []
        self.active_paths_list    = active_paths_list or []
        self.base_namespace       = base_namespace

        self.retry = retry_enabled
        if self.retry:
            self.dont_retry_list = []

        opts = (self.drv_dict.get('DISCOVERY_DICT') or {}).get('OPTIONS', {})
        configured_host = str(opts.get('tcp_host', {}).get('value', '127.0.0.1'))
        port = int(opts.get('tcp_port', {}).get('value', 16718))

        # Simulator config
        simulate = str(opts.get('simulate_hnav', {}).get('value', 'False')).strip().lower() in ('true', '1', 'yes')
        if simulate:
            # Save the user's configured host the first time we enter sim mode
            if self._saved_tcp_host is None:
                self._saved_tcp_host = configured_host
                self.logger.log_info(
                    f"simulate_hnav enabled: saving tcp_host '{configured_host}', "
                    f"connecting to 127.0.0.1")
            host = '127.0.0.1'
        else:
            # Restore the saved host when leaving sim mode
            if self._saved_tcp_host is not None:
                self.logger.log_info(
                    f"simulate_hnav disabled: restoring tcp_host to '{self._saved_tcp_host}'")
                try:
                    self.drv_dict['DISCOVERY_DICT']['OPTIONS']['tcp_host']['value'] = self._saved_tcp_host
                except Exception:
                    pass
                configured_host = self._saved_tcp_host
                self._saved_tcp_host = None
            host = configured_host
        sim_lat       = float(opts.get('sim_latitude',   {}).get('value', 47.6205))
        sim_lon       = float(opts.get('sim_longitude',  {}).get('value', -122.3493))
        sim_depth     = float(opts.get('sim_depth_m',    {}).get('value', 10.0))
        sim_alt       = float(opts.get('sim_altitude_m', {}).get('value', 5.0))
        sim_heading   = float(opts.get('sim_heading_deg',{}).get('value', 0.0))
        sim_rate      = int(  opts.get('sim_rate_hz',    {}).get('value', 10))
        sim_speed     = float(opts.get('sim_speed_ms',   {}).get('value', 0.5))
        sim_roll      = float(opts.get('sim_roll_deg',   {}).get('value', 0.0))
        sim_pitch     = float(opts.get('sim_pitch_deg',  {}).get('value', 0.0))

        self._sim_cfg = dict(
            simulate=simulate, host=host, port=port,
            lat=sim_lat, lon=sim_lon, depth=sim_depth, alt=sim_alt,
            heading=sim_heading, rate_hz=sim_rate, speed_ms=sim_speed,
            roll=sim_roll, pitch=sim_pitch,
        )

        launch_key = f"{host}:{port}"

        # Remove stale entries if the endpoint changed
        for key in [k for k in self.active_devices_dict if k != launch_key]:
            entry = self.active_devices_dict[key]
            nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
            if key in self.active_paths_list:
                self.active_paths_list.remove(key)
            del self.active_devices_dict[key]
            self._stop_sim_server(key)

        if (launch_key not in self.active_paths_list
                and launch_key not in self.dont_retry_list):
            if self._check_for_device(launch_key):
                # Start simulator before the node tries to connect
                if simulate:
                    self._start_sim_server(launch_key)
                if self._launch_device_node(launch_key, host, port):
                    self.active_paths_list.append(launch_key)

        return self.active_paths_list

    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    def killAllDevices(self, active_paths_list):
        for key in list(self.active_devices_dict.keys()):
            entry = self.active_devices_dict[key]
            nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
            if key in active_paths_list:
                active_paths_list.remove(key)
            del self.active_devices_dict[key]
            self._stop_sim_server(key)
        nepi_sdk.sleep(1)
        return active_paths_list

    # =========================================================================
    # Built-in HNav simulator server
    # =========================================================================

    def _start_sim_server(self, launch_key):
        if launch_key in self._sim_threads:
            return
        cfg = self._sim_cfg
        stop_evt = threading.Event()
        t = threading.Thread(
            target=self._sim_loop,
            args=(cfg['host'], cfg['port'], stop_evt,
                  cfg['lat'], cfg['lon'], cfg['depth'], cfg['alt'],
                  cfg['heading'], cfg['rate_hz'], cfg['speed_ms'],
                  cfg['roll'], cfg['pitch']),
            daemon=True,
        )
        self._sim_threads[launch_key] = {'thread': t, 'stop': stop_evt}
        t.start()
        self.logger.log_warn(f"Started HNav simulator on {launch_key}")

    def _stop_sim_server(self, key):
        info = self._sim_threads.pop(key, None)
        if not info:
            return
        info['stop'].set()
        # Wake the accept() call with a dummy connection
        try:
            host, port = key.split(':')
            with socket.create_connection((host, int(port)), timeout=0.2):
                pass
        except Exception:
            pass
        self.logger.log_warn(f"Stopped HNav simulator on {key}")

    def _sim_loop(self, host, port, stop_evt,
                  lat, lon, depth, alt,
                  heading, rate_hz, speed_ms, roll, pitch):
        """TCP server loop: accepts one client at a time and streams HNav packets."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, int(port)))
        srv.listen(1)
        srv.settimeout(0.5)
        try:
            while not stop_evt.is_set():
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                self._serve_client(conn, stop_evt,
                                   lat, lon, depth, alt,
                                   heading, rate_hz, speed_ms, roll, pitch)
        finally:
            try:
                srv.close()
            except Exception:
                pass

    def _serve_client(self, conn, stop_evt,
                      lat, lon, depth, alt,
                      heading, rate_hz, speed_ms, roll, pitch):
        """Stream HNav packets to a single connected client."""
        period = max(0.01, 1.0 / max(1, rate_hz))
        # Convert speed to deg/s for lat/lon dead-reckoning
        # 1 m/s ≈ 1/111320 deg/s latitude
        lat_rate = speed_ms / 111320.0
        cur_lat, cur_lon = lat, lon
        try:
            while not stop_evt.is_set():
                packet = build_hnav_packet(
                    lat_deg     = cur_lat,
                    lon_deg     = cur_lon,
                    depth_m     = depth,
                    alt_m       = alt,
                    roll_deg    = roll,
                    pitch_deg   = pitch,
                    heading_deg = heading,
                    vel_fwd_ms  = speed_ms,
                    sound_vel_ms= 1500.0,
                    temp_c      = 12.0,
                    pos_qual_m  = 1.0,
                )
                conn.sendall(packet)
                # Advance position in the heading direction
                head_rad = math.radians(heading)
                cur_lat += lat_rate * math.cos(head_rad) * period
                cur_lon += (lat_rate / max(math.cos(math.radians(cur_lat)), 1e-6)) \
                           * math.sin(head_rad) * period
                time.sleep(period)
        except Exception:
            pass
        finally:
            try:
                conn.close()
            except Exception:
                pass
