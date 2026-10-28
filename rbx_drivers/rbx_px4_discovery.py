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
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

import os
import subprocess
import time
import struct
import serial
import socket

from serial.tools import list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'RBX_PX4' # Use in display menus
FILE_TYPE = 'DISCOVERY'


#########################################
# MAVLink heartbeat helpers
#########################################
# Module level, with no ROS or NEPI dependency, so the probe logic can be
# tested offline. Only HEARTBEAT is ever decoded here: discovery needs to know
# that a PX4 autopilot is talking and which sys/comp id it uses, nothing more,
# and pymavlink is not a runtime dependency of this driver.

MAVLINK_V1_MAGIC = 0xFE
MAVLINK_V2_MAGIC = 0xFD
MAVLINK_V1_HEADER_LEN = 6   # magic, len, seq, sysid, compid, msgid
MAVLINK_V2_HEADER_LEN = 10  # magic, len, incompat, compat, seq, sysid, compid, msgid x3
MAVLINK_CRC_LEN = 2
MAVLINK_SIGNATURE_LEN = 13
MAVLINK_IFLAG_SIGNED = 0x01
# Longest possible frame: a signed v2 frame with a full 255 byte payload. Any
# frame that is still incomplete at the end of a read buffer starts within this
# many bytes of the end, so this is all of the tail that must be kept.
MAVLINK_MAX_FRAME_LEN = MAVLINK_V2_HEADER_LEN + 255 + MAVLINK_CRC_LEN + MAVLINK_SIGNATURE_LEN

HEARTBEAT_MSG_ID = 0
HEARTBEAT_CRC_EXTRA = 50
HEARTBEAT_PAYLOAD_LEN = 9
HEARTBEAT_AUTOPILOT_BYTE = 5 # custom_mode(0-3), type(4), autopilot(5), base_mode(6), system_status(7), mavlink_version(8)

MAV_AUTOPILOT_PX4 = 12
MAV_AUTOPILOT_INVALID = 8
MAV_TYPE_GCS = 6

GCS_SYS_ID = 255
GCS_COMP_ID = 190


def mavlinkCrc(data, crc = 0xFFFF):
  # X.25 / MCRF4XX checksum, the CRC every MAVLink frame carries.
  for byte in data:
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
  return crc


def findPx4Heartbeat(data):
  # Scans a raw byte stream for the first CRC-valid MAVLink 1 or 2 HEARTBEAT
  # whose autopilot byte is MAV_AUTOPILOT_PX4 and whose sys_id is 1-254.
  # Returns [sys_id, comp_id, mavlink_version] or None.
  #
  # Every magic byte is tried in turn rather than skipping ahead by a frame
  # length, so a stray 0xFE/0xFD inside noise or a truncated frame can never
  # hide a real frame that starts after it. The CRC (with HEARTBEAT's
  # CRC_EXTRA) is what rejects false starts. Heartbeats from any other
  # autopilot (ArduPilot is 3, a GCS or companion is 8) are ignored.
  data = bytes(data)
  data_len = len(data)
  ind = 0
  while ind < data_len:
    magic = data[ind]
    if magic == MAVLINK_V1_MAGIC and (ind + MAVLINK_V1_HEADER_LEN) <= data_len:
      payload_len = data[ind + 1]
      sys_id = data[ind + 3]
      comp_id = data[ind + 4]
      msg_id = data[ind + 5]
      payload_start = ind + MAVLINK_V1_HEADER_LEN
      frame_end = payload_start + payload_len + MAVLINK_CRC_LEN
      # MAVLink 1 never truncates a payload
      len_valid = (payload_len == HEARTBEAT_PAYLOAD_LEN)
      version = 1
    elif magic == MAVLINK_V2_MAGIC and (ind + MAVLINK_V2_HEADER_LEN) <= data_len:
      payload_len = data[ind + 1]
      incompat_flags = data[ind + 2]
      sys_id = data[ind + 5]
      comp_id = data[ind + 6]
      msg_id = data[ind + 7] | (data[ind + 8] << 8) | (data[ind + 9] << 16)
      payload_start = ind + MAVLINK_V2_HEADER_LEN
      frame_end = payload_start + payload_len + MAVLINK_CRC_LEN
      if incompat_flags & MAVLINK_IFLAG_SIGNED:
        frame_end += MAVLINK_SIGNATURE_LEN
      # MAVLink 2 may truncate trailing zero payload bytes. Any incompat flag
      # other than signing means a frame this parser must not interpret.
      len_valid = (payload_len >= 1 and payload_len <= HEARTBEAT_PAYLOAD_LEN
                   and (incompat_flags & ~MAVLINK_IFLAG_SIGNED) == 0)
      version = 2
    else:
      ind += 1
      continue

    if msg_id == HEARTBEAT_MSG_ID and len_valid and frame_end <= data_len:
      crc_start = payload_start + payload_len
      crc = mavlinkCrc(data[ind + 1:crc_start])
      crc = mavlinkCrc([HEARTBEAT_CRC_EXTRA], crc)
      frame_crc = data[crc_start] | (data[crc_start + 1] << 8)
      if crc == frame_crc:
        payload = data[payload_start:crc_start] + bytes(HEARTBEAT_PAYLOAD_LEN - payload_len)
        if payload[HEARTBEAT_AUTOPILOT_BYTE] == MAV_AUTOPILOT_PX4 and sys_id >= 1 and sys_id <= 254:
          return [sys_id, comp_id, version]
    ind += 1
  return None


def buildGcsHeartbeat(seq = 0):
  # One MAVLink 2 HEARTBEAT from a GCS (sysid 255, compid 190, type GCS,
  # autopilot INVALID). Sent once on a silent PX4 USB port: with
  # SYS_USB_AUTO = 1 PX4 waits to hear MAVLink from the host before it
  # starts its own USB MAVLink instance.
  payload = struct.pack('<IBBBBB',
                        0,                     # custom_mode
                        MAV_TYPE_GCS,          # type
                        MAV_AUTOPILOT_INVALID, # autopilot
                        0,                     # base_mode
                        0,                     # system_status
                        3)                     # mavlink_version
  header = bytes([MAVLINK_V2_MAGIC, len(payload), 0, 0, seq & 0xFF,
                  GCS_SYS_ID, GCS_COMP_ID, 0, 0, 0])
  crc = mavlinkCrc(header[1:] + payload)
  crc = mavlinkCrc([HEARTBEAT_CRC_EXTRA], crc)
  return header + payload + struct.pack('<H', crc)


#########################################
# PX4 Discover Method
#########################################

### Function to try and connect to device and also monitor and clean up previously connected devices
class PX4Discovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  active_devices_dict = dict()
  node_launch_name = "px4"

  # USB: the Pixhawk's own USB port enumerates as a CDC-ACM ttyACM device
  # whose USB product string starts "PX4 " on every vendor checked. Only those
  # ports are ever opened -- Pololu Maestro and Teensy devices on these
  # systems are ttyACM ports too, owned by other drivers. "BL" is the PX4
  # bootloader's product token ("PX4 BL FMU v5.x"); a board sitting in its
  # bootloader is never probed.
  USB_DEVICE_MATCH = 'ttyACM'
  USB_PRODUCT_MATCH = 'PX4'
  USB_BOOTLOADER_TOKEN = 'BL'
  USB_BAUD_STR = '115200' # CDC-ACM ignores the rate; used for the probe and the fcu_url alike
  USB_READ_TIMEOUT_SEC = 0.1
  USB_READ_CHUNK_BYTES = 512
  USB_LISTEN_SEC = 2.0        # SYS_USB_AUTO = 2: PX4 transmits first
  USB_ANSWER_LISTEN_SEC = 3.0 # SYS_USB_AUTO = 1: PX4 answers our heartbeat
  # drivers_mgr runs every driver's discoveryFunction in turn on one thread,
  # so a PX4 port that answers nothing (MAVLink disabled on USB) would cost
  # every other driver the full 5 s probe on every cycle. A silent port is
  # re-probed at this interval instead.
  USB_SILENT_RETRY_SEC = 15.0

  # SITL: PX4 SITL's offboard-API MAVLink instance sends from UDP 14580 to
  # 14540 on its target host (localhost unless the SITL was started with this
  # device's IP as its target). mavros binds the same port once launched.
  SITL_PATH_STR = 'SITL_UDP_14540'
  SITL_DEVICE_ID = 'sitl'
  SITL_UDP_PORT = 14540
  SITL_LISTEN_SEC = 2.0
  SITL_RECV_TIMEOUT_SEC = 0.25
  SITL_RECV_BYTES = 2048

  # Mavros (mavlink) launch configuration: mavros's own PX4 configs, loaded
  # into the mavros node's namespace before it starts. They ship next to the
  # apm_*.yaml files in the mavros package's launch/ folder, found through
  # rospack with the old fixed /opt/nepi path as a fallback, the same way
  # rbx_ardupilot_discovery.py finds its APM configs.
  PX4_CONFIG_FILES = ['px4_pluginlists.yaml', 'px4_config.yaml']
  LEGACY_MAVROS_LAUNCH_FOLDER = '/opt/nepi/nepi_engine/share/mavros/launch'
  mavros_launch_folder = None
  mavros_launch_folder_checked = False

  ################################################
  def __init__(self):
    ############
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_sdk.logger(namespace = self.log_name)
    time.sleep(1)
    self.logger.log_info("Starting Initialization")

    # Instance attributes, not the class-level defaults above: drivers_mgr
    # caches one discovery instance per driver entry, and class-level dicts
    # mutated in place would be shared by every instance (see the matching
    # comment in rbx_ardupilot_discovery.py).
    self.active_devices_dict = dict()
    self.launch_time_dict = dict()
    self.dont_retry_list = []

    # Probe log bookkeeping, so a PX4 port that stays silent, or a SITL port
    # some other process holds, is reported once instead of on every
    # discovery cycle.
    self.silent_ports_dict = dict() # path_str: time of the last silent probe
    self.port_errors_dict = dict()
    self.sitl_bind_error = None

    self.logger.log_info("Initialization Complete")


  ##########  Drv Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled = True):
    self.drv_dict = drv_dict
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace

    ########################
    # Get discovery options
    try:
      connection_type = drv_dict['DISCOVERY_DICT']['OPTIONS']['connection']['value']
    except Exception as e:
      self.logger.log_warn("Failed to load options " + str(e))
      # drivers_mgr assigns this return value back to the ONE active_paths_list
      # every driver shares, so it is returned unchanged, never None.
      return self.active_paths_list

    # Retry behavior
    self.retry = retry_enabled
    if self.retry == True:
      self.dont_retry_list = []
    ########################

    ### Purge Unresponsive Connections
    path_purge_list = []
    for path_str in self.active_devices_dict.keys():
      success = self.checkOnDevice(path_str)
      if success == False:
        path_purge_list.append(path_str)
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del self.active_devices_dict[path_str]
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)

    ### Checking for devices on available paths
    # RUN USB PROCESSES
    if connection_type == 'USB':
      # Create path search options
      self.path_list = self.getPx4UsbPortsList()
      for path_str in list(self.silent_ports_dict.keys()):
        if path_str not in self.path_list:
          del self.silent_ports_dict[path_str]
      for path_str in self.path_list:
        valid_path = True
        if path_str in self.active_paths_list or path_str in self.dont_retry_list:
          valid_path = False
        if valid_path and path_str in self.silent_ports_dict.keys():
          valid_path = (nepi_sdk.get_time() - self.silent_ports_dict[path_str]) > self.USB_SILENT_RETRY_SEC
        if valid_path:
          [found_device, path_str, comp_id, sys_id] = self.checkForUsbDevice(path_str)
          if found_device:
            success = self.launchUsbDeviceNode(path_str, comp_id, sys_id)
            self.logger.log_info("USB mavlink launch process returned: " + str(success))
            if success:
              self.active_paths_list.append(path_str)
    # RUN SITL PROCESS (PX4 Software-In-The-Loop over UDP)
    elif connection_type == 'SITL':
      path_str = self.SITL_PATH_STR
      if path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
        [found_device, path_str, sender_addr, comp_id, sys_id] = self.checkForSitlDevice(path_str)
        if found_device:
          success = self.launchSitlDeviceNode(path_str, sender_addr, comp_id, sys_id)
          self.logger.log_info("SITL mavlink launch process returned: " + str(success))
          if success:
            self.active_paths_list.append(path_str)
    # Wrap Up
    return self.active_paths_list


  ################################################
  ##########  Shared Processes

  def checkOnDevice(self, path_str):
    # Returns True if the device's mavlink AND RBX node processes are both
    # still alive and its path is still present. Both are checked because
    # mavros can stay healthy while the RBX node exits (see the matching
    # comment in rbx_ardupilot_discovery.py) -- and the RBX node exiting is
    # exactly how a SITL link loss is signalled (PX4Node's link watchdog).
    active = True
    if path_str not in self.active_devices_dict.keys():
      return False

    device_entry = self.active_devices_dict[path_str]
    mavlink_subproc = device_entry["mavlink_subproc"]
    rbx_subproc = device_entry.get("rbx_subproc")

    purge_node = False
    # Check that the mavlink process is still running
    if mavlink_subproc is None or mavlink_subproc.poll() is not None:
      self.logger.log_warn("Mavlink process for " + path_str + " is no longer running... purging from managed list")
      purge_node = True
    # Check that the PX4 RBX node itself is still running
    elif rbx_subproc is None or rbx_subproc.poll() is not None:
      self.logger.log_warn("PX4 RBX node for " + path_str + " is no longer running... purging from managed list")
      purge_node = True
    # For USB connections, check that the port still exists
    elif path_str.startswith('/dev/') and path_str not in self.available_paths_list:
      self.logger.log_warn("Port associated with node no longer detected " + path_str)
      purge_node = True

    if purge_node:
      self.killDeviceProcesses(device_entry)
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)
      if path_str in self.dont_retry_list:
        self.dont_retry_list.remove(path_str)
      active = False
    return active


  def killDeviceProcesses(self, device_entry):
    # Kill the RBX node and mavlink subprocesses for a device entry
    mav_node_name = device_entry.get("mav_node_name")
    rbx_node_name = device_entry.get("rbx_node_name")
    mavlink_subproc = device_entry.get("mavlink_subproc")
    rbx_subproc = device_entry.get("rbx_subproc")

    if rbx_subproc is not None:
      self.logger.log_info("Killing px4 node: " + str(rbx_node_name))
      nepi_drvs.killDriverNode(rbx_node_name, rbx_subproc)
    if mavlink_subproc is not None:
      self.logger.log_info("Killing mavlink node: " + str(mav_node_name))
      nepi_drvs.killDriverNode(mav_node_name, mavlink_subproc)


  def getMavrosLaunchFolder(self):
    # Looked up once per discovery instance -- the mavros install does not
    # move while drivers_mgr runs, and rospack is slow enough to matter on
    # every relaunch. Checking once is also what keeps the missing-configs
    # warning to a single line.
    if self.mavros_launch_folder_checked:
      return self.mavros_launch_folder
    self.mavros_launch_folder_checked = True
    candidates = []
    try:
      result = subprocess.run(['rospack', 'find', 'mavros'], capture_output = True, text = True, timeout = 10)
      if result.returncode == 0 and result.stdout.strip():
        candidates.append(os.path.join(result.stdout.strip(), 'launch'))
    except Exception as e:
      self.logger.log_warn("rospack find mavros failed: " + str(e))
    candidates.append(self.LEGACY_MAVROS_LAUNCH_FOLDER)
    for folder in candidates:
      if all(os.path.isfile(os.path.join(folder, f)) for f in self.PX4_CONFIG_FILES):
        self.mavros_launch_folder = folder
        self.logger.log_info("Using mavros PX4 configs from: " + folder)
        return folder
    self.logger.log_warn("mavros PX4 configs (" + ", ".join(self.PX4_CONFIG_FILES) +
                         ") not found in " + str(candidates) +
                         " -- mavros will load every plugin with its defaults")
    return None

  def launchDeviceNode(self, path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url, connection_str):
    success = False
    launch_id = path_str

    # Check if should try to launch (backoff to prevent rapid relaunch loops)
    launch_check = True
    if launch_id in self.launch_time_dict.keys():
      launch_time = self.launch_time_dict[launch_id]
      cur_time = nepi_sdk.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LOAD_TIME_SEC
    if launch_check == False:
      return False

    ### Start Mavlink (mavros) Node Launch Process
    # The px4_ prefix keeps this node name distinct from the ArduPilot
    # driver's mavlink_<id> nodes (mavlink_sitl in particular), since
    # nepi_sdk.wait_for_node matches by substring.
    mav_node_name = "mavlink_" + self.node_launch_name + "_" + device_id_str
    mav_node_namespace = nepi_sdk.create_namespace(self.base_namespace, mav_node_name)
    self.logger.log_info("Starting mavlink node setup: " + mav_node_name)
    # Load the proper configs for PX4
    px4_config_folder = self.getMavrosLaunchFolder()
    if px4_config_folder is not None:
      for config_file in self.PX4_CONFIG_FILES:
        subprocess.run(['rosparam', 'load', os.path.join(px4_config_folder, config_file), mav_node_namespace])
    # Adjust the timesync_rate to cut down on log noise
    nepi_sdk.set_param(mav_node_namespace + '/conn/timesync_rate', 1.0)

    # Launch Mavlink Node. The target ids come from the probed heartbeat;
    # left unset, mavros would address sys/comp 1 regardless of SYS_ID.
    self.logger.log_info("Launching mavlink node: " + mav_node_name)
    node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + mav_node_name,
                    '_fcu_url:=' + fcu_url, '_gcs_url:=' + gcs_url,
                    '_target_system_id:=' + str(mav_sys_id),
                    '_target_component_id:=' + str(mav_comp_id)]
    try:
      mav_subproc = subprocess.Popen(node_run_cmd)
    except Exception as e:
      self.logger.log_warn("Failed to launch mavlink node: " + mav_node_name + " (" + str(e) + ")")
      return False

    ### Start the PX4 RBX node for this mavlink connection
    rbx_device_name = self.node_launch_name + "_" + device_id_str
    rbx_node_name = nepi_system.get_device_alias(rbx_device_name)

    # Setup required param server drv_dict for the PX4 node
    file_name = self.drv_dict['NODE_DICT']['file_name']
    self.drv_dict['DEVICE_DICT'] = {
      'device_name': rbx_device_name,
      'device_path': path_str,
      'mavlink_node_name': mav_node_name,
      'fcu_url': fcu_url,
      'gcs_url': gcs_url,
      'mav_sys_id': mav_sys_id,
      'mav_comp_id': mav_comp_id,
      'connection': connection_str
    }
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace, rbx_node_name + "/drv_dict")
    nepi_sdk.set_param(dict_param_name, self.drv_dict)

    self.logger.log_info("Starting px4 rbx node: " + rbx_node_name)
    # LD_PRELOAD aarch64 libgomp for the RBX node launch only, exactly as
    # rbx_ardupilot_discovery.py does (see its comment for the full
    # diagnosis): the node's import chain loads cv2 before open3d, and on
    # aarch64 cv2 claims libgomp's static TLS slot first, so open3d's import
    # crashes with "cannot allocate memory in static TLS block". Preloading
    # gives libgomp the slot before the interpreter starts. Set only around
    # this spawn and restored right after, so mavros and other drivers never
    # inherit it, and only when the library exists.
    ld_preload_key = 'LD_PRELOAD'
    ld_preload_lib = '/lib/aarch64-linux-gnu/libgomp.so.1'
    prev_ld_preload = os.environ.get(ld_preload_key)
    if os.path.isfile(ld_preload_lib):
      os.environ[ld_preload_key] = ld_preload_lib
    try:
      [success, msg, rbx_subproc] = nepi_drvs.launchDriverNode(file_name, rbx_node_name)
    finally:
      if prev_ld_preload is None:
        os.environ.pop(ld_preload_key, None)
      else:
        os.environ[ld_preload_key] = prev_ld_preload

    # Process launch results
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    if success:
      self.logger.log_info("Launched node: " + rbx_node_name)
      device_entry = dict()
      device_entry["sysid"] = mav_sys_id
      device_entry["compid"] = mav_comp_id
      device_entry["mav_node_name"] = mav_node_name
      device_entry["rbx_node_name"] = rbx_node_name
      device_entry["mavlink_subproc"] = mav_subproc
      device_entry["rbx_subproc"] = rbx_subproc
      self.active_devices_dict[path_str] = device_entry
    else:
      self.logger.log_warn("Failed to launch node: " + rbx_node_name + " with msg: " + msg)
      # The PX4 node failed; tear down the mavlink process we started for it
      nepi_drvs.killDriverNode(mav_node_name, mav_subproc)
      if self.retry == False:
        self.logger.log_warn("Will not retry launch for node: " + rbx_node_name)
        self.dont_retry_list.append(launch_id)
    return success


  def logPortErrorOnce(self, path_str, err_str):
    # One warning per distinct error per port; a port that keeps failing the
    # same way is only reported again once its error changes.
    if self.port_errors_dict.get(path_str) != err_str:
      self.port_errors_dict[path_str] = err_str
      self.logger.log_warn(err_str)


  ########## USB PROCESSES ############

  def getPx4UsbPortsList(self):
    # Only ttyACM ports whose USB product string names PX4 and is not the
    # bootloader. product may be None (e.g. a port with no USB descriptor).
    ports_list = []
    try:
      comports = list_ports.comports()
    except Exception as e:
      self.logPortErrorOnce('list_ports', "Unable to list serial ports (" + str(e) + ")")
      return ports_list
    for port in comports:
      path_str = str(port.device)
      product = port.product
      if product is None:
        continue
      product = str(product)
      if path_str.find(self.USB_DEVICE_MATCH) == -1:
        continue
      if product.find(self.USB_PRODUCT_MATCH) == -1 or self.USB_BOOTLOADER_TOKEN in product.split():
        continue
      ports_list.append(path_str)
    return ports_list


  def readForPx4Heartbeat(self, serial_port, listen_sec):
    # Reads for up to listen_sec and returns [sys_id, comp_id, mavlink_version]
    # from the first PX4 heartbeat, or None. Only the tail a split frame could
    # still occupy is kept between reads, so a fast USB stream cannot grow the
    # buffer.
    read_buffer = b''
    end_time = nepi_sdk.get_time() + listen_sec
    while nepi_sdk.get_time() < end_time:
      bytes_read = serial_port.read(self.USB_READ_CHUNK_BYTES)
      if len(bytes_read) == 0:
        continue
      read_buffer = read_buffer + bytes_read
      heartbeat = findPx4Heartbeat(read_buffer)
      if heartbeat is not None:
        return heartbeat
      read_buffer = read_buffer[-MAVLINK_MAX_FRAME_LEN:]
    return None


  def checkForUsbDevice(self, path_str):
    found_device = False
    mav_comp_id = None
    mav_sys_id = None
    try:
      # Try and open serial port
      serial_port = serial.Serial(path_str, int(self.USB_BAUD_STR), timeout = self.USB_READ_TIMEOUT_SEC)
    except Exception as e:
      self.logPortErrorOnce(path_str, "Unable to open PX4 USB port " + path_str + " (" + str(e) + ")")
      return found_device, path_str, mav_comp_id, mav_sys_id

    heartbeat = None
    try:
      # SYS_USB_AUTO = 2 (current default): PX4 starts MAVLink as soon as the
      # host enumerates, so it transmits first.
      heartbeat = self.readForPx4Heartbeat(serial_port, self.USB_LISTEN_SEC)
      if heartbeat is None:
        # SYS_USB_AUTO = 1: PX4 waits for a MAVLink heartbeat from the host.
        # Exactly one is written, then PX4 gets time to start and answer.
        serial_port.write(buildGcsHeartbeat())
        serial_port.flush()
        heartbeat = self.readForPx4Heartbeat(serial_port, self.USB_ANSWER_LISTEN_SEC)
    except Exception as e:
      self.logPortErrorOnce(path_str, "PX4 USB probe failed on " + path_str + " (" + str(e) + ")")
    finally:
      # Clean up the serial port
      try:
        serial_port.close()
      except Exception:
        pass

    if heartbeat is not None:
      [mav_sys_id, mav_comp_id, mav_version] = heartbeat
      found_device = True
      self.silent_ports_dict.pop(path_str, None)
      self.port_errors_dict.pop(path_str, None)
      self.logger.log_info("Found PX4 autopilot at: " + path_str + " with sys_id " + str(mav_sys_id) +
                           " comp_id " + str(mav_comp_id) + " (MAVLink " + str(mav_version) + ")")
    else:
      # Info once, not a warning every cycle: a PX4 board that is powered but
      # has MAVLink disabled on USB is a configuration state, not a fault.
      if path_str not in self.silent_ports_dict.keys():
        self.logger.log_info("PX4 USB port " + path_str + " sent no PX4 heartbeat (check SYS_USB_AUTO)" +
                             " -- will keep probing it every " + str(int(self.USB_SILENT_RETRY_SEC)) + " s")
      self.silent_ports_dict[path_str] = nepi_sdk.get_time()
    return found_device, path_str, mav_comp_id, mav_sys_id


  def launchUsbDeviceNode(self, path_str, mav_comp_id, mav_sys_id):
    success = False
    if mav_comp_id is not None and mav_sys_id is not None:
      device_id_str = path_str.split('/')[-1]
      fcu_url = path_str + ':' + self.USB_BAUD_STR
      gcs_url = ""
      success = self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url, 'USB')
    return success


  ########## SITL PROCESS ############

  def checkForSitlDevice(self, path_str):
    found_device = False
    sender_addr = None
    mav_comp_id = None
    mav_sys_id = None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Explicit, not inherited: nepi_sdk/rospy set a process-wide 60 s
    # socket.setdefaulttimeout that every plain socket in drivers_mgr picks up.
    # No SO_REUSEADDR, so a port some other process already holds (mavros for
    # another driver, a GCS) fails the bind instead of silently sharing it.
    sock.settimeout(self.SITL_RECV_TIMEOUT_SEC)
    try:
      sock.bind(('0.0.0.0', self.SITL_UDP_PORT))
    except Exception as e:
      err_str = str(e)
      if err_str != self.sitl_bind_error:
        self.sitl_bind_error = err_str
        self.logger.log_warn("Unable to listen for PX4 SITL on UDP port " + str(self.SITL_UDP_PORT) +
                             " (" + err_str + ") -- another process holds it; skipping")
      try:
        sock.close()
      except Exception:
        pass
      return found_device, path_str, sender_addr, mav_comp_id, mav_sys_id
    self.sitl_bind_error = None

    try:
      end_time = nepi_sdk.get_time() + self.SITL_LISTEN_SEC
      while found_device == False and nepi_sdk.get_time() < end_time:
        try:
          data, addr = sock.recvfrom(self.SITL_RECV_BYTES)
        except socket.timeout:
          continue
        heartbeat = findPx4Heartbeat(data)
        if heartbeat is not None:
          [mav_sys_id, mav_comp_id, mav_version] = heartbeat
          sender_addr = addr
          found_device = True
          self.logger.log_info("Found PX4 SITL at: " + str(addr[0]) + ":" + str(addr[1]) +
                               " with sys_id " + str(mav_sys_id) + " comp_id " + str(mav_comp_id))
    except Exception as e:
      self.logger.log_debug("PX4 SITL probe failed: " + str(e), throttle_s = 60.0)
    finally:
      try:
        sock.close()
      except Exception:
        pass
    return found_device, path_str, sender_addr, mav_comp_id, mav_sys_id


  def launchSitlDeviceNode(self, path_str, sender_addr, mav_comp_id, mav_sys_id):
    success = False
    if sender_addr is not None and mav_comp_id is not None and mav_sys_id is not None:
      device_id_str = self.SITL_DEVICE_ID  # -> mavros node "mavlink_px4_sitl", rbx node "px4_sitl"
      # mavros binds the port the probe just released and sends back to the
      # exact address the SITL heartbeat came from.
      fcu_url = "udp://:" + str(self.SITL_UDP_PORT) + "@" + str(sender_addr[0]) + ":" + str(sender_addr[1])
      gcs_url = ""
      success = self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url, 'SITL')
    return success


  def killAllDevices(self, active_paths_list):
    path_purge_list = []
    for key in self.active_devices_dict.keys():
      path_purge_list.append(key)
    for path_str in path_purge_list:
      device_entry = self.active_devices_dict[path_str]
      if self.retry == False:
        self.dont_retry_list.append(path_str)
      self.killDeviceProcesses(device_entry)
      if path_str in active_paths_list:
        active_paths_list.remove(path_str)
    for path_str in path_purge_list:
      del self.active_devices_dict[path_str]
    nepi_sdk.sleep(1)
    return active_paths_list


#########################################
# Main
#########################################
if __name__ == '__main__':
  PX4Discovery()
