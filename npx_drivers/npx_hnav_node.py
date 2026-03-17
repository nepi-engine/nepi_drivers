#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
# Driver for the Sonardyne SPRINT Nav Mini (8334) via the binary HNav protocol.
#
# HNav binary protocol reference:
#   https://github.com/Sonardyne/sprint-nav-mini-hnav
#
# Packet layout (67 bytes total):
#   Bytes  0-1  : Start bytes 0xAA 0xBF
#   Byte   2    : Protocol version (0x00)
#   Bytes  3-4  : Message ID (uint16 LE, 0x0000 for HNav)
#   Bytes  5-6  : Data size  (uint16 LE, 0x0037 = 55)
#   Bytes  7-9  : Reserved (source/device ID - 3 bytes)
#   Byte  10    : HNav data version
#   Bytes 11-18 : UTC time   (uint64 LE, microseconds since epoch – verify ICD)
#   Bytes 19-22 : Latitude   (int32  LE, scaled by HNAV_LAT_SCALE)
#   Bytes 23-26 : Longitude  (int32  LE, scaled by HNAV_LON_SCALE)
#   Bytes 27-30 : Depth      (int32  LE, scaled by HNAV_DEPTH_SCALE, positive down)
#   Bytes 31-32 : Altitude   (uint16 LE, scaled by HNAV_ALT_SCALE, above seabed)
#   Bytes 33-34 : Roll       (int16  LE, scaled by HNAV_ORIENT_SCALE)
#   Bytes 35-36 : Pitch      (int16  LE, scaled by HNAV_ORIENT_SCALE)
#   Bytes 37-38 : Heading    (uint16 LE, scaled by HNAV_ORIENT_SCALE, True North)
#   Bytes 39-40 : Vel fwd    (int16  LE, scaled by HNAV_VEL_SCALE, body frame)
#   Bytes 41-42 : Vel stbd   (int16  LE, scaled by HNAV_VEL_SCALE, body frame)
#   Bytes 43-44 : Vel down   (int16  LE, scaled by HNAV_VEL_SCALE, body frame)
#   Bytes 45-46 : Ang rate fwd  (int16 LE, scaled by HNAV_ANG_VEL_SCALE)
#   Bytes 47-48 : Ang rate stbd (int16 LE, scaled by HNAV_ANG_VEL_SCALE)
#   Bytes 49-50 : Ang rate down (int16 LE, scaled by HNAV_ANG_VEL_SCALE)
#   Bytes 51-52 : Sound velocity (uint16 LE, scaled by HNAV_SOUND_VEL_SCALE)
#   Bytes 53-54 : Temperature   (int16  LE, scaled by HNAV_TEMP_SCALE)
#   Bytes 55-58 : Position quality CEP50 (float32 LE, metres)
#   Bytes 59-60 : Heading quality (uint16 LE, scaled by HNAV_HEAD_QUAL_SCALE)
#   Bytes 61-62 : Velocity quality (uint16 LE, scaled by HNAV_VEL_QUAL_SCALE, mm/s)
#   Bytes 63-64 : Status flags   (uint16 LE – see STATUS_* masks below)
#   Bytes 65-66 : CRC-16 (little-endian)
#
# IMPORTANT – Scale constants below are derived from the open-source reference decoder
# and verified against the published example data.  If Sonardyne updates the ICD,
# re-check these values against the official Interface Control Document.

import os
import socket
import struct
import threading
import time
import copy

from nepi_sdk import nepi_sdk, nepi_utils, nepi_nav
from nepi_api.messages_if import MsgIF
from nepi_api.device_if_npx import NPXDeviceIF

PKG_NAME = 'NPX_HNAV'
DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

# ---------------------------------------------------------------------------
# HNav packet constants
# ---------------------------------------------------------------------------
PACKET_SIZE = 67
DATA_OFFSET = 10          # first data byte (version field)
START_BYTE1 = 0xAA
START_BYTE2 = 0xBF
PROTOCOL_VERSION = 0x00
MESSAGE_ID = 0x0000
MESSAGE_DATA_SIZE = 55

# Struct format for the 55-byte data section (little-endian):
#   B  – HNav version    (1)
#   Q  – UTC time        (8)
#   i  – latitude        (4)
#   i  – longitude       (4)
#   i  – depth           (4)
#   H  – altitude        (2)
#   h  – roll            (2)
#   h  – pitch           (2)
#   H  – heading         (2)
#   h  – vel fwd         (2)
#   h  – vel stbd        (2)
#   h  – vel down        (2)
#   h  – ang rate fwd    (2)
#   h  – ang rate stbd   (2)
#   h  – ang rate down   (2)
#   H  – sound velocity  (2)
#   h  – temperature     (2)
#   f  – pos quality     (4)
#   H  – heading quality (2)
#   H  – vel quality     (2)
#   H  – status flags    (2)
# Struct format for the 55-byte data section (little-endian).
# Field types:  B=uint8  Q=uint64  i=int32  H=uint16  h=int16  f=float32
# Note: sound_vel is unsigned (H), all velocities/angular-rates are signed (h).
# Size check: 1+8+4+4+4+2+2+2+2+2+2+2+2+2+2+2+2+4+2+2+2 = 55 ✓
DATA_STRUCT_FMT = '<BQiiiHhhHhhhhhhHhfHHH'
DATA_STRUCT_SIZE = struct.calcsize(DATA_STRUCT_FMT)   # should be 55

# ---------------------------------------------------------------------------
# Scale constants  (VERIFY AGAINST SONARDYNE SPRINT NAV MINI ICD)
# ---------------------------------------------------------------------------
HNAV_UTC_SCALE      = 1e-6           # uint64 microseconds → seconds
HNAV_LAT_SCALE      = 90.0 / 2147483648.0   # int32 → degrees  (±90°)
HNAV_LON_SCALE      = 180.0 / 2147483648.0  # int32 → degrees  (±180°)
HNAV_DEPTH_SCALE    = 0.001          # int32 mm   → metres (positive = down)
HNAV_ALT_SCALE      = 0.01           # uint16 cm  → metres (above seabed)
HNAV_ORIENT_SCALE   = 0.01           # int16/uint16 → degrees
HNAV_VEL_SCALE      = 0.001          # int16 mm/s → m/s
HNAV_ANG_VEL_SCALE  = 0.01           # int16      → deg/s
HNAV_SOUND_VEL_SCALE= 0.1            # uint16     → m/s
HNAV_TEMP_SCALE     = 0.01           # int16      → °C
HNAV_POS_QUAL_SCALE = 1.0            # float32, already metres (CEP50)
HNAV_HEAD_QUAL_SCALE= 0.01           # uint16     → degrees
HNAV_VEL_QUAL_SCALE = 1.0            # uint16     → mm/s

# ---------------------------------------------------------------------------
# Status word bitmasks  (derived from reference C++ decoder + example packet)
# ---------------------------------------------------------------------------
STATUS_SYSTEM_ERROR    = 0x0001   # bit  0: set = system error present
STATUS_HYBRID_MODE     = 0x0002   # bit  1: set = operating in hybrid mode
STATUS_HEADING_INVALID = 0x0004   # bit  2: set = heading invalid
STATUS_DEPTH_INVALID   = 0x0008   # bit  3: set = depth invalid
STATUS_VEL_INVALID     = 0x0010   # bit  4: set = velocity invalid
STATUS_SVEL_INVALID    = 0x0020   # bit  5: set = sound velocity invalid
STATUS_TEMP_INVALID    = 0x0040   # bit  6: set = temperature invalid
STATUS_ALT_INVALID     = 0x0080   # bit  7: set = altitude invalid
STATUS_POS_INVALID     = 0x0100   # bit  8: set = position invalid


class HNavNode(object):
    navpose_update_rate = 20

    device_info_dict = dict(device_name="",
                            path="",
                            serial_number="",
                            hw_version="",
                            sw_version="")

    def __init__(self, drv_dict=None):
        nepi_sdk.init_node(name=DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.msg_if = MsgIF(log_name=self.class_name)

        self.msg_if.pub_info("Starting Node Initialization")

        # -- Pull settings from drv_dict ---------------------------------------
        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']

            dev = self.drv_dict.get('DEVICE_DICT', {}) or {}
            self.host = str(dev.get('tcp_host', '192.168.2.201'))
            self.port = int(dev.get('tcp_port', 16718))
            self.param_file = dev.get('param_file',
                '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_hnav_params.yaml')

            save_data = self.drv_dict.get('SAVE_DATA', {}) or {}
            self.debug = bool(save_data.get('debug', False)
                              or self.drv_dict.get('debug', False))
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict: " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": No valid Device Dict")
            return

        self.msg_if.pub_info(f"TCP target: {self.host}:{self.port}")
        self.msg_if.pub_info(f"Debug: {self.debug}")

        if not os.path.exists(self.param_file):
            self.msg_if.pub_warn("Param file not found: " + self.param_file)
            nepi_sdk.signal_shutdown(self.node_name + ": Missing param file")
            return

        # -- Shared navpose state ----------------------------------------------
        self._lock = threading.Lock()
        self._navpose = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        # Pre-declare the capabilities this driver supports so NPXDeviceIF
        # always creates the correct publishers regardless of whether the
        # device is reachable at startup time.
        self._navpose['has_location']    = True
        self._navpose['has_heading']     = True
        self._navpose['has_orientation'] = True
        self._navpose['has_altitude']    = True
        self._navpose['has_depth']       = True

        self._stop_evt = threading.Event()

        # -- Register with NEPI NPX manager ------------------------------------
        # TCP thread starts AFTER NPXDeviceIF so the init-time getNavPoseCb
        # snapshot always sees the pre-declared capabilities above.
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.npx_if = NPXDeviceIF(
            device_info=self.device_info_dict,
            data_source_description="sensor",
            data_ref_description="sensor_center",
            getNavPoseCb=self.getNavPoseCb,
            max_navpose_update_rate=self.navpose_update_rate,
            msg_if=self.msg_if
        )

        self.msg_if.pub_info("Initialization Complete")
        nepi_sdk.on_shutdown(self.cleanup_actions)
        self._rx_thread = threading.Thread(target=self._tcp_loop, daemon=True)
        self._rx_thread.start()
        nepi_sdk.spin()

    # -------------------------------------------------------------------------
    # TCP reader – maintains a sliding byte buffer, finds HNav sync frames
    # -------------------------------------------------------------------------
    def _clear_navpose(self):
        """Reset has_* flags so consumers know data is stale after disconnect."""
        with self._lock:
            for key in self._navpose:
                if key.startswith('has_'):
                    self._navpose[key] = False

    def _tcp_loop(self):
        while not self._stop_evt.is_set():
            try:
                with socket.create_connection((self.host, self.port),
                                              timeout=5.0) as s:
                    s.settimeout(5.0)
                    buf = bytearray()
                    while not self._stop_evt.is_set():
                        try:
                            chunk = s.recv(256)
                        except socket.timeout:
                            continue
                        if not chunk:
                            break
                        buf.extend(chunk)
                        self._process_buffer(buf)
            except Exception as e:
                self.msg_if.pub_warn(
                    f"TCP reconnect in 2 s after: {type(e).__name__}: {e}")
            self._clear_navpose()
            time.sleep(2.0)

    def _process_buffer(self, buf):
        """Consume all complete, CRC-valid HNav packets from buf in-place."""
        while True:
            # Find sync pattern
            idx = -1
            for i in range(len(buf) - 1):
                if buf[i] == START_BYTE1 and buf[i + 1] == START_BYTE2:
                    idx = i
                    break
            if idx < 0:
                # No sync – keep last byte in case it is the start of a header
                if len(buf) > 1:
                    del buf[:-1]
                return

            if idx > 0:
                del buf[:idx]   # drop pre-sync garbage

            if len(buf) < PACKET_SIZE:
                return          # wait for more data

            packet = buf[:PACKET_SIZE]

            # Validate header fields
            proto_ver = packet[2]
            msg_id = packet[3] | (packet[4] << 8)
            data_sz = packet[5] | (packet[6] << 8)

            if (proto_ver != PROTOCOL_VERSION
                    or msg_id != MESSAGE_ID
                    or data_sz != MESSAGE_DATA_SIZE):
                # Not a valid HNav frame – skip first two bytes and retry
                del buf[:2]
                continue

            if not self._check_crc(packet):
                if self.debug:
                    self.msg_if.pub_warn("HNav CRC mismatch – skipping 2 bytes")
                del buf[:2]
                continue

            # Good packet – decode and advance buffer
            self._decode_packet(packet)
            del buf[:PACKET_SIZE]

    # -------------------------------------------------------------------------
    # CRC-16 (ported from Sonardyne reference C++)
    # -------------------------------------------------------------------------
    @staticmethod
    def _reflect(crc, bit_num):
        out = 0
        bit = 1
        for i in range(bit_num - 1, -1, -1):
            if crc & (1 << i):
                out |= bit
            bit <<= 1
        return out

    def _check_crc(self, packet):
        crc = 0xFFFF
        for i in range(PACKET_SIZE - 2):
            current_byte = self._reflect(packet[i], 8)
            j = 0x80
            while j:
                bit = crc & 0x8000
                crc = (crc << 1) & 0xFFFF
                if current_byte & j:
                    bit ^= 0x8000
                if bit:
                    crc ^= 0x1021
                j >>= 1
        crc = self._reflect(crc, 16)
        crc ^= 0xFFFF
        crc &= 0xFFFF
        expected = packet[65] | (packet[66] << 8)
        return crc == expected

    # -------------------------------------------------------------------------
    # Packet decoder
    # -------------------------------------------------------------------------
    def _decode_packet(self, packet):
        data = bytes(packet[DATA_OFFSET: DATA_OFFSET + MESSAGE_DATA_SIZE])
        if len(data) < DATA_STRUCT_SIZE:
            return

        try:
            (version,
             utc_raw,
             lat_raw, lon_raw, depth_raw,
             alt_raw,
             roll_raw, pitch_raw, heading_raw,
             vel_fwd_raw, vel_stbd_raw, vel_down_raw,
             ang_fwd_raw, ang_stbd_raw, ang_down_raw,
             svel_raw, temp_raw,
             pos_qual_raw,
             head_qual_raw, vel_qual_raw,
             status) = struct.unpack(DATA_STRUCT_FMT, data)
        except struct.error as e:
            self.msg_if.pub_warn("HNav struct unpack error: " + str(e))
            return

        # -- Validity flags from status word -----------------------------------
        pos_valid   = not bool(status & STATUS_POS_INVALID)
        head_valid  = not bool(status & STATUS_HEADING_INVALID)
        alt_valid   = not bool(status & STATUS_ALT_INVALID)
        depth_valid = not bool(status & STATUS_DEPTH_INVALID)
        vel_valid   = not bool(status & STATUS_VEL_INVALID)

        if self.debug:
            hybrid = bool(status & STATUS_HYBRID_MODE)
            error  = bool(status & STATUS_SYSTEM_ERROR)
            self.msg_if.pub_info(
                f"HNav status=0x{status:04X} hybrid={hybrid} err={error} "
                f"pos={pos_valid} hdg={head_valid} alt={alt_valid} "
                f"dep={depth_valid} vel={vel_valid}"
            )

        # -- Scale raw integers → engineering units ---------------------------
        lat_deg   = lat_raw   * HNAV_LAT_SCALE
        lon_deg   = lon_raw   * HNAV_LON_SCALE
        depth_m   = depth_raw * HNAV_DEPTH_SCALE   # positive = downward
        alt_m     = alt_raw   * HNAV_ALT_SCALE
        roll_deg  = roll_raw  * HNAV_ORIENT_SCALE
        pitch_deg = pitch_raw * HNAV_ORIENT_SCALE
        head_deg  = heading_raw * HNAV_ORIENT_SCALE  # True North, 0-360

        if self.debug:
            self.msg_if.pub_info(
                f"lat={lat_deg:.6f} lon={lon_deg:.6f} "
                f"depth={depth_m:.2f}m alt={alt_m:.2f}m "
                f"roll={roll_deg:.2f}° pitch={pitch_deg:.2f}° "
                f"heading={head_deg:.2f}°"
            )

        t = nepi_utils.get_time()
        with self._lock:
            d = self._navpose
            d['timestamp'] = t

            if pos_valid:
                d['latitude']      = lat_deg
                d['longitude']     = lon_deg
                d['has_location']  = True
                d['time_location'] = t
                # has_position stays False – we have no local ENU x/y/z from this message

            if head_valid:
                d['heading_deg']   = head_deg
                d['has_heading']   = True
                d['time_heading']  = t
                # Populate orientation using roll/pitch/heading
                d['roll_deg']        = roll_deg
                d['pitch_deg']       = pitch_deg
                d['yaw_deg']         = head_deg
                d['has_orientation'] = True
                d['time_orientation']= t

            if alt_valid:
                d['altitude_m']   = alt_m
                d['has_altitude'] = True
                d['time_altitude']= t

            if depth_valid:
                d['depth_m']    = depth_m
                d['has_depth']  = True
                d['time_depth'] = t

    # -------------------------------------------------------------------------
    # NPX callback
    # -------------------------------------------------------------------------
    def getNavPoseCb(self):
        with self._lock:
            return copy.deepcopy(self._navpose)

    def cleanup_actions(self):
        self._stop_evt.set()
        self.msg_if.pub_warn("Shutdown cleanup complete")


if __name__ == '__main__':
    HNavNode()
