#!/usr/bin/env python
# Numurus / NEPI — NMEA UDP Discovery (TCP) — matches MicroStrain discovery structure
import os
import time
import socket
import threading
import datetime

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs

PKG_NAME = 'NPX_NMEA_UDP'
FILE_TYPE = 'DISCOVERY'

PARAM_FILE_PATH = '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_nmea_udp_params.yaml'
DEFAULT_NODE_FILE = 'npx_nmea_udp_node.py'  # sensible default

class NMEAUDPDiscovery:
    NODE_LOAD_TIME_SEC = 10
    launch_time_dict = dict()
    retry = True
    dont_retry_list = []

    active_devices_dict = dict()
    node_launch_name = "nmea"

    # simulator tracking
    sim_threads = {}   # key: "host:port" -> {'thread': t, 'stop': threading.Event()}
    _sim_cfg = None

    ################################################
    def __init__(self):
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name=self.log_name)
        time.sleep(0.2)
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")

    ################################################
    # Nex Standard Discovery Function
    def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict):
        """
        For TCP NMEA we don't 'discover' hardware. We:
          - Pull host/port/options from drv_dict
          - Build a stable key like '127.0.0.1:50000'
          - Ensure we only launch once, respect retry timing, and push a full drv_dict
        """
        self.drv_dict = drv_dict or {}
        self.available_paths_list = available_paths_list or []
        self.active_paths_list = active_paths_list or []
        self.base_namespace = base_namespace

        # Load options (host/port, retry). Provide safe defaults.
        opts = (self.drv_dict.get('DISCOVERY_DICT') or {}).get('OPTIONS', {})
        host = str(opts.get('tcp_host', {}).get('value', '127.0.0.1'))
        port = int(opts.get('tcp_port', {}).get('value', 50000))

        # Retry behavior
        self.retry = bool(opts.get('retry', {}).get('value', True) if 'retry' in opts else opts.get('auto_restart', {}).get('value', True))

        # Internal simulator configuration (optional)
        simulate = bool(opts.get('simulate_nmea', {}).get('value', False))
        alt_latitude = float(opts.get('alt_latitude', {}).get('value', 47.6205))
        alt_longitude = float(opts.get('alt_longitude', {}).get('value', -122.3493))
        alt_altitude = float(opts.get('alt_altitude', {}).get('value', 10.0))
        alt_heading_deg = float(opts.get('alt_heading_deg', {}).get('value', 0.0))
        sim_rate = int(opts.get('sim_rate_hz', {}).get('value', 5))
        sim_speed = float(opts.get('sim_speed_kts', {}).get('value', 0.0))
        course_deg = opts.get('course_deg', {}).get('value', None)
        if course_deg is not None and course_deg != "None":
            try:
                course_deg = float(course_deg)
            except Exception:
                course_deg = None
        self._sim_cfg = dict(
            simulate=simulate, host=host, port=port,
            lat=alt_latitude, lon=alt_longitude, alt_m=alt_altitude,
            heading_deg=alt_heading_deg, rate_hz=sim_rate,
            sog_kts=sim_speed, cog_deg=(course_deg if course_deg is not None else alt_heading_deg)
        )

        launch_key = f"{host}:{port}"

        purge = []
        for key in list(self.active_devices_dict.keys()):
            # For TCP, if user changed host:port in YAML, treat old key as stale
            if key != launch_key:
                purge.append(key)
        for key in purge:
            entry = self.active_devices_dict[key]
            nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
            if key in self.active_paths_list:
                self.active_paths_list.remove(key)
            del self.active_devices_dict[key]
            # stop any sim server tied to the old key
            self._stop_sim_server(key)

        # If not already active, attempt to “find” (always true for TCP) and launch
        if launch_key not in self.active_paths_list:
            found = self.checkForDevice(launch_key)  # always True for configured TCP
            if found:
                if self.launchDeviceNode(launch_key, host, port):
                    self.active_paths_list.append(launch_key)

        return self.active_paths_list

    ################################################
    # Device-specific helpers (shape mirrors MicroStrain file)
    def checkForDevice(self, launch_key: str) -> bool:
        # For TCP, configuration is the discovery. If user provided host/port, we consider it “found”.
        return True

    def checkOnDevice(self, launch_key: str) -> bool:
        # For TCP, we can simply consider it active unless config changed; discovery manager will re-call us.
        return True

    def launchDeviceNode(self, launch_key: str, host: str, port: int) -> bool:
        success = False

        # Respect relaunch backoff window
        launch_check = True
        if launch_key in self.launch_time_dict:
            last = float(self.launch_time_dict[launch_key])
            cur = float(nepi_sdk.get_time())
            launch_check = (cur - last) > float(self.NODE_LOAD_TIME_SEC)
        if launch_check is False:
            return False

        # Resolve node file
        file_name = DEFAULT_NODE_FILE
        try:
            node_dict = self.drv_dict['NODE_DICT']
            if node_dict.get('file_name'):
                file_name = node_dict['file_name']
        except Exception:
            pass

        # Form node_name in the same style you already use in logs
        try:
            node_name = self.node_launch_name + "_" + str(launch_key).split(':')[0].replace('.','').replace('-','_')
        except:
            node_name = self.node_launch_name + "_" + str(launch_key).replace(':','_').replace('.','').replace('-','_')

        self.logger.log_warn("Launching node: " + node_name)

        # Load any saved config for this node (same pattern as MicroStrain)
        nepi_drvs.checkLoadConfigFile(node_name)

        # Ensure DEVICE_DICT and SAVE_DATA blocks exist (avoid NoneType in SaveDataIF)
        self.drv_dict.setdefault('DEVICE_DICT', {})
        self.drv_dict['DEVICE_DICT'].update({
            'tcp_host': host,
            'tcp_port': int(port),
            'param_file': PARAM_FILE_PATH
        })

        # Minimal SAVE_DATA section so SaveDataIF init never sees None
        self.drv_dict.setdefault('SAVE_DATA', {})
        self.drv_dict['SAVE_DATA']['save_rate_dict'] = self.drv_dict['SAVE_DATA'].get('save_rate_dict', {})
        self.drv_dict['SAVE_DATA']['save_data_enable'] = bool(self.drv_dict['SAVE_DATA'].get('save_data_enable', False))

        # Push the complete driver dict under the node namespace (same as MicroStrain flow)
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
        nepi_sdk.set_param(dict_param_name, self.drv_dict)

        # Start internal NMEA simulator if requested (before launching client node so it can connect)
        try:
            if self._sim_cfg and self._sim_cfg.get('simulate'):
                self._start_sim_server(
                    host=self._sim_cfg['host'],
                    port=self._sim_cfg['port'],
                    lat=self._sim_cfg['lat'],
                    lon=self._sim_cfg['lon'],
                    alt_m=self._sim_cfg['alt_m'],
                    heading_deg=self._sim_cfg['heading_deg'],
                    rate_hz=self._sim_cfg['rate_hz'],
                    sog_kts=self._sim_cfg['sog_kts'],
                    cog_deg=self._sim_cfg['cog_deg'],
                )
        except Exception as e:
            self.logger.log_warn(f"Failed to start internal NMEA sim: {e}")

        # Launch the node
        ok, msg, subp = nepi_drvs.launchDriverNode(file_name, node_name)
        self.launch_time_dict[launch_key] = nepi_sdk.get_time()

        if ok:
            self.logger.log_warn(f"Launched node: {node_name}")
            self.active_devices_dict[launch_key] = {'node_name': node_name, 'sub_process': subp}
            success = True
        else:
            self.logger.log_warn(f"Failed to launch node: {node_name} with msg: {msg}")
            if not self.retry:
                self.logger.log_warn(f"Will not try relaunch for node: {node_name}")
                self.dont_retry_list.append(launch_key)
            else:
                self.logger.log_warn(f"Will attempt relaunch for node: {node_name} in {self.NODE_LOAD_TIME_SEC} secs")

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
            # stop sim server as well
            self._stop_sim_server(key)
        nepi_sdk.sleep(1)
        return active_paths_list

    # ---------- internal simulator plumbing ----------
    def _start_sim_server(self, host, port, lat, lon, alt_m, heading_deg, rate_hz, sog_kts, cog_deg):
        key = f"{host}:{port}"
        if key in self.sim_threads:
            return  # already running

        stop_evt = threading.Event()
        t = threading.Thread(
            target=self._sim_loop,
            args=(host, port, stop_evt, lat, lon, alt_m, heading_deg, rate_hz, sog_kts, cog_deg),
            daemon=True
        )
        self.sim_threads[key] = {'thread': t, 'stop': stop_evt}
        t.start()
        self.logger.log_warn(f"Started internal NMEA sim on {key}")

    def _stop_sim_server(self, key):
        info = self.sim_threads.get(key)
        if not info:
            return
        try:
            info['stop'].set()
        except Exception:
            pass
        # best-effort wake the accept() by connecting once
        try:
            host, port = key.split(":")
            with socket.create_connection((host, int(port)), timeout=0.2):
                pass
        except Exception:
            pass
        self.sim_threads.pop(key, None)
        self.logger.log_warn(f"Stopped internal NMEA sim on {key}")

    def _sim_loop(self, host, port, stop_evt, lat, lon, alt_m, heading_deg, rate_hz, sog_kts, cog_deg):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, int(port)))
        srv.listen(1)
        srv.settimeout(0.5)

        try:
            while not stop_evt.is_set():
                try:
                    conn, addr = srv.accept()
                    self._serve_client(conn, stop_evt, lat, lon, alt_m, heading_deg, rate_hz, sog_kts, cog_deg)
                except socket.timeout:
                    continue
                except OSError:
                    break
        finally:
            try: srv.close()
            except: pass

    def _serve_client(self, conn, stop_evt, lat, lon, alt_m, heading_deg, rate_hz, sog_kts, cog_deg):
        try:
            while not stop_evt.is_set():
                lines = [
                    self._make_GGA(lat, lon, alt_m),
                    self._make_RMC(lat, lon, sog_kts, cog_deg),
                    self._make_VTG(cog_deg, sog_kts),
                    self._make_HDG(heading_deg),
                ]
                payload = ("\r\n".join(lines) + "\r\n").encode("ascii")
                conn.sendall(payload)
                time.sleep(max(0.01, 1.0 / max(1, rate_hz)))
        except Exception:
            pass
        finally:
            try: conn.close()
            except: pass

    # ---------- NMEA helpers ----------
    def _dd_to_nmea(self, lat, lon):
        alat, alon = abs(lat), abs(lon)
        lat_deg = int(alat); lon_deg = int(alon)
        lat_min = (alat - lat_deg) * 60.0
        lon_min = (alon - lon_deg) * 60.0
        ns = 'N' if lat >= 0 else 'S'
        ew = 'E' if lon >= 0 else 'W'
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        return lat_str, ns, lon_str, ew

    def _nmea_checksum(self, s):
        c = 0
        for ch in s:
            c ^= ord(ch)
        return f"{c:02X}"

    def _make_GGA(self, lat, lon, alt_m, fix=1, sats=8, hdop=1.0):
        now = datetime.datetime.utcnow().strftime("%H%M%S")
        lat_s, ns, lon_s, ew = self._dd_to_nmea(lat, lon)
        parts = ["GPGGA", now, lat_s, ns, lon_s, ew, str(fix), f"{sats:02d}", f"{hdop:.1f}",
                 f"{alt_m:.1f}", "M", "0.0", "M", "", ""]
        core = ",".join(parts)
        return f"${core}*{self._nmea_checksum(core)}"

    def _make_RMC(self, lat, lon, sog_kts=0.0, cog_deg=0.0):
        now = datetime.datetime.utcnow()
        tstr = now.strftime("%H%M%S"); dstr = now.strftime("%d%m%y")
        lat_s, ns, lon_s, ew = self._dd_to_nmea(lat, lon)
        parts = ["GPRMC", tstr, "A", lat_s, ns, lon_s, ew, f"{sog_kts:.1f}", f"{cog_deg:.1f}", dstr, "", ""]
        core = ",".join(parts)
        return f"${core}*{self._nmea_checksum(core)}"

    def _make_VTG(self, cog_deg=0.0, sog_kts=0.0):
        parts = ["GPVTG", f"{cog_deg:.1f}", "T", "", "M", f"{sog_kts:.1f}", "N", f"{sog_kts*1.852:.1f}", "K"]
        core = ",".join(parts)
        return f"${core}*{self._nmea_checksum(core)}"

    def _make_HDG(self, heading_deg=0.0):
        parts = ["HCHDG", f"{heading_deg:.1f}", "", "", "", ""]
        core = ",".join(parts)
        return f"${core}*{self._nmea_checksum(core)}"
