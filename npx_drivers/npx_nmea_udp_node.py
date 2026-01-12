#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import socket
import threading
import time
import copy

from nepi_sdk import nepi_sdk, nepi_utils, nepi_nav
from nepi_api.messages_if import MsgIF
from nepi_api.device_if_npx import NPXDeviceIF

PKG_NAME = 'NPX_NMEA_UDP'
DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"


# ------------------ Helpers ------------------
def _safe_float(x, default=0.0):
    try:
        return float(x)
    except Exception:
        return default


def _nmea_latlon_to_decimal(field: str, hemi: str, is_lat: bool):
    if not field or not hemi:
        return None
    try:
        if is_lat:
            deg = int(field[0:2])
            minutes = float(field[2:])
            sign = 1.0 if hemi.upper() == "N" else -1.0
        else:
            deg = int(field[0:3])
            minutes = float(field[3:])
            sign = 1.0 if hemi.upper() == "E" else -1.0
        return sign * (deg + minutes / 60.0)
    except Exception:
        return None


class NMEAUDPNode(object):
    navpose_update_rate = 20
    driver_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

    def __init__(self, drv_dict=None):
        # Initialize ROS/NEPI node
        nepi_sdk.init_node(name=DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.msg_if = MsgIF(log_name=self.class_name)

        self.msg_if.pub_info("Starting Node Initialization Processes")
        self.msg_if.pub_info("Gathering driver settings")

        # 1) KEEP param-loaded drv_dict; don't overwrite it with None later
        if drv_dict is None:
            drv_dict = nepi_sdk.get_param('~drv_dict', {})
        self.drv_dict = drv_dict

        if not self.drv_dict:
            self.msg_if.pub_info("Driver Dict not provided")
            nepi_sdk.signal_shutdown(self.node_name + ": Driver Dict not provided")
            return

        # 2) Read connection fields from DEVICE_DICT (as discovery writes them)
        dev = self.drv_dict.get('DEVICE_DICT', {}) or {}
        self.host = str(dev.get('tcp_host', '127.0.0.1'))
        self.port = int(dev.get('tcp_port', 50000))
        self.param_file = dev.get('param_file', '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_nmea_udp_params.yaml')

        save_data = self.drv_dict.get('SAVE_DATA', {}) or {}
        self.debug = bool(save_data.get('debug', False) or self.drv_dict.get('debug', False))
        self.heading_min_speed = float(self.drv_dict.get('heading_min_speed_knots', 0.5))

        self.msg_if.pub_info(f"Using host: {self.host}")
        self.msg_if.pub_info(f"Using port: {self.port}")
        self.msg_if.pub_info(f"Param file: {self.param_file}")
        self.msg_if.pub_info(f"Heading min speed (kt): {self.heading_min_speed}")
        self.msg_if.pub_info(f"Debug: {self.debug}")

        # 3) Param file existence/readability check (non-fatal beyond logging + shutdown)
        if not os.path.exists(self.param_file):
            self.msg_if.pub_warn("Could not find param file at: " + self.param_file)
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting Down - Could not find param file")
            return
        node_params_dict = nepi_utils.read_dict_from_file(self.param_file)
        if node_params_dict is None:
            self.msg_if.pub_warn("Could not read params from file at: " + self.param_file)
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting Down - Could not read params from file")
            return

        # 4) Prepare parsing/state
        self._lock = threading.Lock()

        # 5) Start TCP reader thread (client) that feeds NMEA lines to the parser
        self._stop_evt = threading.Event()
        self._rx_thread = threading.Thread(target=self._tcp_loop, daemon=True)
        self._rx_thread.start()

        # 6) Register NPXDeviceIF so the rest of NEPI can query navpose
        self.device_info_dict = dict(
            node_name=self.node_name,
            device_name="nmea-udp",
            identifier="udp",
            serial_number="", hw_version="", sw_version=""
        )
        self.npx_if = NPXDeviceIF(
            device_info=self.device_info_dict,
            data_source_description="sensor",
            data_ref_description="sensor_center",
            getNavPoseCb=self.getNavPoseCb,
            get3DTransformCb=None,
            max_navpose_update_rate=self.navpose_update_rate,
            msg_if=self.msg_if
        )

        self.msg_if.pub_info("Initialization Complete")
        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()

    # ---------- TCP client that reads NMEA ----------
    def _tcp_loop(self):
        while not self._stop_evt.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=3.0) as s:
                    s_file = s.makefile('r', encoding='ascii', newline='\n')
                    for line in s_file:
                        if self._stop_evt.is_set():
                            break
                        line = line.strip()
                        if not line or not line.startswith('$'):
                            continue
                        self._handle_nmeaCb(line)
            except Exception as e:
                if self.debug:
                    self.msg_if.pub_warn(f"TCP reconnect in 1s after error: {e}")
                time.sleep(1.0)

    # ---------- NMEA parsing ----------
    def _handle_nmeaCb(self, sentence: str):
        try:
            if "*" in sentence:
                body, _ = sentence[1:].split("*", 1)
            else:
                body = sentence[1:]
            p = body.split(",")
            typ = p[0]
        except Exception:
            return

        if   typ.endswith("GGA"): self._parse_gga(p)
        elif typ.endswith("RMC"): self._parse_rmc(p)
        elif typ.endswith("VTG"): self._parse_vtg(p)
        elif typ.endswith("HDT") or typ.endswith("HDG"): self._parse_hdt(p)

    def _parse_gga(self, p):
        # $GxGGA,hhmmss,lat,N,lon,W,fix,nsat,hdop,alt,M,geoid,M,...
        if len(p) < 10:
            return
        lat = p[2]; lat_hemi = p[3]
        lon = p[4]; lon_hemi = p[5]
        fix = p[6]
        alt = p[9]
        lat_dd = _nmea_latlon_to_decimal(lat, lat_hemi, True)
        lon_dd = _nmea_latlon_to_decimal(lon, lon_hemi, False)
        try:
            fix_q = int(fix)
        except Exception:
            fix_q = 0
        alt_m = _safe_float(alt, None)

        if lat_dd is None or lon_dd is None or fix_q <= 0:
            return

        t = nepi_utils.get_time()
        with self._lock:
            driver_navpose_dict = self.driver_navpose_dict
            driver_navpose_dict['latitude'] = lat_dd
            driver_navpose_dict['longitude'] = lon_dd
            driver_navpose_dict['latitude_deg'] = lat_dd
            driver_navpose_dict['longitude_deg'] = lon_dd
            driver_navpose_dict['timestamp'] = t

            driver_navpose_dict['has_location'] = True
            driver_navpose_dict['time_location'] = t
            driver_navpose_dict['has_position'] = True
            driver_navpose_dict['time_position'] = t

            if alt_m is not None:
                driver_navpose_dict['altitude_m'] = alt_m
                driver_navpose_dict['time_altitude'] = t
                driver_navpose_dict['has_altitude'] = True

        if self.debug:
            self.msg_if.pub_info(f"GGA pos: lat={lat_dd:.6f}, lon={lon_dd:.6f}, alt={alt_m}")

    def _parse_rmc(self, p):
        # $GxRMC,hhmmss,A,lat,N,lon,W,sog,cog,ddmmyy,...
        if len(p) < 9:
            return
        status = p[2]
        if status.upper() != "A":
            return
        lat_dd = _nmea_latlon_to_decimal(p[3], p[4], True)
        lon_dd = _nmea_latlon_to_decimal(p[5], p[6], False)
        sog = _safe_float(p[7], 0.0)
        cog = _safe_float(p[8], 0.0)

        t = nepi_utils.get_time()
        with self._lock:
            driver_navpose_dict = self.driver_navpose_dict
            if lat_dd is not None and lon_dd is not None:
                driver_navpose_dict['latitude'] = lat_dd
                driver_navpose_dict['longitude'] = lon_dd
                driver_navpose_dict['latitude_deg'] = lat_dd
                driver_navpose_dict['longitude_deg'] = lon_dd
                driver_navpose_dict['timestamp'] = t
                driver_navpose_dict['has_location'] = True
                driver_navpose_dict['time_location'] = t
                driver_navpose_dict['has_position'] = True
                driver_navpose_dict['time_position'] = t

            if sog >= self.heading_min_speed:
                driver_navpose_dict['heading_deg'] = cog
                driver_navpose_dict['has_heading'] = True
                driver_navpose_dict['time_heading'] = t
            driver_navpose_dict['sog_knots'] = sog

        if self.debug:
            self.msg_if.pub_info(f"RMC vel: cog={cog:.1f}deg")

    def _parse_vtg(self, p):
        # $..VTG,<cog_true>,T,<cog_mag>,M,<sog_knots>,N,<sog_kph>,K
        cog = _safe_float(p[1] if len(p) > 1 else "", 0.0)
        sog = _safe_float(p[5] if len(p) > 5 else "", 0.0)
        t = nepi_utils.get_time()
        with self._lock:
            driver_navpose_dict = self.driver_navpose_dict
            if sog >= self.heading_min_speed:
                driver_navpose_dict['heading_deg'] = cog
                driver_navpose_dict['has_heading'] = True
                driver_navpose_dict['time_heading'] = t
            driver_navpose_dict['sog_knots'] = sog
        if self.debug:
            self.msg_if.pub_info(f"VTG: cog={cog:.1f}deg, sog={sog:.2f}kt")

    def _parse_hdt(self, p):
        # $..HDT,<true_heading>,T
        cog = _safe_float(p[1] if len(p) > 1 else "", 0.0)
        t = nepi_utils.get_time()
        with self._lock:
            driver_navpose_dict = self.driver_navpose_dict
            driver_navpose_dict['heading_deg'] = cog
            driver_navpose_dict['has_heading'] = True
            driver_navpose_dict['time_heading'] = t
        if self.debug:
            self.msg_if.pub_info(f"HDT: heading_true={cog:.1f}deg")

    def getNavPoseCb(self):
        return self.driver_navpose_dict

    def cleanup_actions(self):
        self.msg_if.pub_warn("Shutdown cleanup actions complete")


if __name__ == '__main__':
    # Normal launches are handled via discovery setting ~drv_dict and starting this script.
    # Direct run is supported for sanity (will attempt to read ~drv_dict).
    NMEAUDPNode()
