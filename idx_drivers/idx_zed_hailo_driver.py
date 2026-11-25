#!/usr/bin/env python
#
# Copyright (c) 2026 Numurus <https://www.numurus.com>.
#
# This file is part of nepi drivers (nepi_drivers) repo
# (see https://github.com/nepi-engine/nepi_drivers)
#
# License: NEPI Drivers repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

# Stereo processing for a ZED camera read as a plain UVC device (no ZED SDK, no CUDA).
# Frames come from a V4l2CamDriver instance passed in by the node. This class holds
# the calibration lookup and parsing, the camera serial lookup, rectification, and
# the cpu_sgbm and hailo depth backends. It is ROS-free: status comes back as
# return values, and one-off log lines are queued for the node (get_messages).

import os
import re
import glob
import math
import time
import threading
import configparser
import urllib.request

import numpy as np
import cv2

try:
  from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams, ConfigureParams,
                              InputVStreamParams, OutputVStreamParams, FormatType)
  HAILO_AVAILABLE = True
  HAILO_IMPORT_ERROR = None
  HAILO_INTERFACE = HailoStreamInterface.PCIe
except Exception as e:
  HEF = None
  VDevice = None
  InferVStreams = None
  ConfigureParams = None
  InputVStreamParams = None
  OutputVStreamParams = None
  FormatType = None
  HAILO_AVAILABLE = False
  HAILO_IMPORT_ERROR = str(e)
  HAILO_INTERFACE = None

PKG_NAME = 'IDX_ZED_HAILO' # Use in display menus
FILE_TYPE = 'DRIVER'


class ZedHailoStereoDriver(object):

  # Side-by-side frame size, per-eye size, frame rates and calibration file
  # resolution key for each ZED USB mode (Stereolabs documentation)
  RESOLUTIONS = {
    'HD2K':   {'sbs': (4416, 1242), 'eye': (2208, 1242), 'fps': [15],              'cal_key': '2K'},
    'HD1080': {'sbs': (3840, 1080), 'eye': (1920, 1080), 'fps': [15, 30],          'cal_key': 'FHD'},
    'HD720':  {'sbs': (2560, 720),  'eye': (1280, 720),  'fps': [15, 30, 60],      'cal_key': 'HD'},
    'VGA':    {'sbs': (1344, 376),  'eye': (672, 376),   'fps': [15, 30, 60, 100], 'cal_key': 'VGA'},
  }

  CAL_KEYS = ['2K', 'FHD', 'HD', 'VGA']
  CAL_CAM_KEYS = ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2', 'k3']
  CAL_URL = 'https://calib.stereolabs.com/?SN='
  CAL_DOWNLOAD_TIMEOUT_S = 10
  CAL_DOWNLOAD_MAX_BYTES = 1000000

  ZED_USB_VENDOR_ID = '2b03'

  DEPTH_BACKENDS = ['cpu_sgbm', 'hailo']
  SGBM_NUM_DISP_BOUNDS = [16, 256]
  SGBM_BLOCK_SIZE_BOUNDS = [3, 11]
  SGBM_UNIQUENESS_RATIO = 10
  SGBM_SPECKLE_WINDOW_SIZE = 100
  SGBM_SPECKLE_RANGE = 2
  SGBM_DISP12_MAX_DIFF = 1

  # Disparity post-filters. 'fill' is the median followed by an edge-aware hole
  # fill: a guided filter (He et al.) on the disparity and on its validity mask,
  # guided by the left image. Areas whose valid support stays below
  # FILL_MIN_SUPPORT are left empty, not invented.
  DEPTH_FILTERS = ['none', 'median', 'fill']
  MEDIAN_KSIZE = 5
  FILL_RADIUS = 8
  FILL_EPS = 1e-3
  FILL_MIN_SUPPORT = 0.3

  # Depth is computed on the rectified pair shrunk by an integer factor, then
  # scaled back to the full rectified size so depth stays aligned with the left
  # image. 'auto' picks the factor that brings the per-eye width nearest VGA.
  DEPTH_DOWNSCALE_OPTIONS = ['auto', '1', '2', '3', '4']
  DEPTH_AUTO_TARGET_WIDTH = 672

  def __init__(self, cam_driver, resolution, cal_folder, hailo_folder, sysfs_root = '/sys'):
    if resolution not in self.RESOLUTIONS:
      raise Exception("Unsupported ZED resolution " + str(resolution) + ", expected one of " + str(list(self.RESOLUTIONS.keys())))
    self.cam_driver = cam_driver
    self.resolution = resolution
    self.res_dict = self.RESOLUTIONS[resolution]
    self.cal_folder = cal_folder
    self.hailo_folder = hailo_folder
    self.sysfs_root = sysfs_root

    self.msg_lock = threading.Lock()
    self.msg_list = []

    # Capture size check result (configure_capture)
    self.size_ok = True
    self.size_msg = ""
    self.shape_warned = False

    # Calibration and rectification
    self.calibrated = False
    self.cal_file = None
    self.cal_dict = None
    self.uncal_msg = "No ZED calibration loaded"
    self.rect_maps = None
    self.P1 = None
    self.P2 = None
    self.Q = None
    self.fx_rect = None
    self.baseline_mm = None
    self.fov_deg = None

    # Shared split/rectified pair, one per captured frame
    self.pair_lock = threading.Lock()
    self.pair_ts = None
    self.pair_left = None
    self.pair_right = None
    self.pair_rectified = False
    self.pair_fetch_time = None
    self.frame_interval = None

    # Depth
    self.depth_lock = threading.Lock()
    self.disp_ts = None
    self.disp_cache = None
    self.requested_backend = 'cpu_sgbm'
    self.active_backend = 'cpu_sgbm'
    self.sgbm_num_disparities = 64
    self.sgbm_block_size = 5
    self.sgbm = None
    self.depth_filter = 'fill'
    self.depth_downscale = 'auto'
    self.hailo_swap_inputs = False
    self.hailo_hef_path = None
    self.hailo_device = None
    self.hailo_hef = None
    self.hailo_network_group = None
    self.hailo_network_group_params = None
    self.hailo_input_params = None
    self.hailo_output_params = None
    self.hailo_inputs = None
    self.hailo_output = None
    self.hailo_first_inference = True

    # Timing stats, read and reset by get_timing_stats
    self.stats_lock = threading.Lock()
    self.stats_start = time.time()
    self.stats_frames = 0
    self.stats_rect_s = 0.0
    self.stats_rect_n = 0
    self.stats_depth_s = 0.0
    self.stats_depth_n = 0


  ###################
  ## Messages

  def get_messages(self):
    """Return and clear the log lines queued by the driver.

    Returns:
      list: [level, text] pairs, level one of 'debug', 'info', 'warn', 'error'.
    """
    self.msg_lock.acquire()
    msgs = self.msg_list
    self.msg_list = []
    self.msg_lock.release()
    return msgs

  def queueMsg(self, level, text):
    self.msg_lock.acquire()
    self.msg_list.append([level, text])
    self.msg_lock.release()


  ###################
  ## Capture

  def configure_capture(self, framerate):
    """Request this resolution's side-by-side size and a frame rate, then check the negotiated size.

    The frame rate is clamped to the rates the ZED supports at this resolution.
    If the negotiated size does not match, rectification and depth are disabled
    for this run and the images are still split at width/2.

    Args:
      framerate (float): Requested frames per second.

    Returns:
      list: [success, msg, fps_used]. success is False when the negotiated size does not match.
    """
    sbs_w, sbs_h = self.res_dict['sbs']
    fps = self.clampFramerate(framerate)
    self.frame_interval = 1.0 / float(fps)
    [res_success, res_msg] = self.cam_driver.setResolution({'width': sbs_w, 'height': sbs_h})
    [fps_success, fps_msg] = self.cam_driver.setFramerate(fps)
    [cur_success, cur_res] = self.cam_driver.getCurrentResolution()
    if cur_success and cur_res['width'] == sbs_w and cur_res['height'] == sbs_h:
      self.size_ok = True
      self.size_msg = ""
      msg = "Capture set to " + self.resolution + " (" + str(sbs_w) + "x" + str(sbs_h) + ") at " + str(fps) + " fps"
      if not fps_success:
        msg += " (frame rate request failed: " + str(fps_msg) + ")"
      return [True, msg, fps]
    self.size_ok = False
    self.size_msg = ("Negotiated frame size " + str(cur_res.get('width', 0)) + "x" + str(cur_res.get('height', 0)) +
                     " does not match the " + self.resolution + " side-by-side size " + str(sbs_w) + "x" + str(sbs_h) +
                     " (resolution request: " + str(res_msg) + "). Publishing left and right halves split at width/2," +
                     " with rectification and depth disabled for this run")
    return [False, self.size_msg, fps]

  def clampFramerate(self, framerate):
    rates = self.res_dict['fps']
    allowed = [r for r in rates if r <= framerate]
    if len(allowed) == 0:
      return min(rates)
    return max(allowed)


  ###################
  ## Serial number

  def find_serial_number(self, device_path):
    """Find the ZED serial number in sysfs for a video device.

    Looks for a USB device with vendor id 2b03 and an all-digit serial string (the
    ZED's sensor HID interface reports the camera serial this way). On a USB 2
    link the HID sits on the same hub as the video device. On a USB 3 link the
    video device is on the SuperSpeed bus and the HID is under the device on the
    video port's USB 2 peer port (<usb device>/port/peer/device), so that subtree
    is searched too. The sysfs root is set at construction.

    Args:
      device_path (str): Video device path, e.g. /dev/video0.

    Returns:
      list: [serial or None, msg].
    """
    video_name = os.path.basename(os.path.realpath(device_path))
    v4l_dev = os.path.join(self.sysfs_root, 'class', 'video4linux', video_name, 'device')
    if not os.path.exists(v4l_dev):
      return [None, "No sysfs entry " + v4l_dev + " for " + device_path]
    usb_dir = os.path.realpath(v4l_dev)
    root_dir = os.path.realpath(self.sysfs_root)
    while not os.path.exists(os.path.join(usb_dir, 'idVendor')):
      parent = os.path.dirname(usb_dir)
      if parent == usb_dir or len(parent) <= len(root_dir):
        return [None, "No USB device found above " + os.path.realpath(v4l_dev) + " in sysfs"]
      usb_dir = parent
    hub_dir = os.path.dirname(usb_dir)
    peer_dir = None
    peer_device = os.path.join(usb_dir, 'port', 'peer', 'device')
    if os.path.exists(peer_device):
      peer_dir = os.path.realpath(peer_device)

    usb_devices_dir = os.path.join(self.sysfs_root, 'bus', 'usb', 'devices')
    try:
      names = sorted(os.listdir(usb_devices_dir))
    except Exception as e:
      return [None, "Could not list " + usb_devices_dir + ": " + str(e)]
    serials = []
    for name in names:
      dev_dir = os.path.realpath(os.path.join(usb_devices_dir, name))
      on_hub = os.path.dirname(dev_dir) == hub_dir
      under_peer = peer_dir is not None and (dev_dir == peer_dir or dev_dir.startswith(peer_dir + os.sep))
      if not (on_hub or under_peer):
        continue
      vendor = self.readSysfsAttr(dev_dir, 'idVendor')
      serial = self.readSysfsAttr(dev_dir, 'serial')
      if vendor is None or serial is None:
        continue
      if vendor.lower() == self.ZED_USB_VENDOR_ID and re.match(r'^[0-9]+$', serial) is not None:
        if serial not in serials:
          serials.append(serial)
    where = "USB hub " + hub_dir + ("" if peer_dir is None else " or under USB 2 peer " + peer_dir)
    if len(serials) == 1:
      return [serials[0], "Found ZED serial number " + serials[0] + " on " + where]
    if len(serials) == 0:
      return [None, "No USB device with vendor id " + self.ZED_USB_VENDOR_ID + " and a numeric serial number on " + where]
    return [None, "More than one ZED serial number on " + where + ": " + str(serials)]

  def readSysfsAttr(self, dev_dir, attr):
    try:
      with open(os.path.join(dev_dir, attr), 'r') as f:
        return f.read().strip()
    except Exception:
      return None


  ###################
  ## Calibration

  def load_calibration(self, serial, download_enabled):
    """Find, parse and apply the ZED factory calibration, then build the rectification maps.

    Lookup order: <cal_folder>/SN<serial>.conf when the serial is known; the only
    SN*.conf in the folder when the serial is unknown; a one-time download from
    Stereolabs when the serial is known, the file is missing and download_enabled
    is True. Otherwise the driver stays uncalibrated: raw halves and no depth.

    Args:
      serial (str or None): Camera serial number (all digits), or None if unknown.
      download_enabled (bool): Allow the one-time download.

    Returns:
      list: [success, msg, warn]. warn is True when msg should be logged as a warning.
    """
    self.calibrated = False
    self.rect_maps = None
    if serial is not None and re.match(r'^[0-9]+$', str(serial)) is None:
      serial = None
    cal_dict = None
    cal_file = None
    notes = []
    warn = False
    if serial is not None:
      expected_file = 'SN' + serial + '.conf'
      cal_path = os.path.join(self.cal_folder, expected_file)
      if os.path.isfile(cal_path):
        [success, msg, cal_dict] = self.parse_calibration_file(cal_path)
        if success:
          cal_file = cal_path
        else:
          notes.append(msg)
      elif download_enabled:
        [success, msg, cal_dict] = self.downloadCalibration(serial, cal_path)
        notes.append(msg)
        if success:
          cal_file = cal_path
      else:
        notes.append("download_calibration is off")
    else:
      expected_file = 'SN<camera serial>.conf'
      found = sorted(glob.glob(os.path.join(self.cal_folder, 'SN*.conf')))
      if len(found) == 1:
        [success, msg, cal_dict] = self.parse_calibration_file(found[0])
        if success:
          cal_file = found[0]
          warn = True
          notes.append("Camera serial number unknown; using the only calibration file in " + self.cal_folder + ": " + os.path.basename(found[0]))
        else:
          notes.append(msg)
      else:
        notes.append("camera serial number unknown and " + str(len(found)) + " SN*.conf files in the folder")

    if cal_dict is not None and cal_file is not None:
      [success, msg] = self.buildRectification(cal_dict)
      if success:
        self.cal_dict = cal_dict
        self.cal_file = cal_file
        self.calibrated = True
        notes.append(msg)
        return [True, "Loaded ZED calibration " + cal_file + ". " + ". ".join(notes), warn]
      notes.append(msg)

    self.uncal_msg = ("No usable ZED calibration: expected " + expected_file + " in " + self.cal_folder +
                      ". Copy the camera's factory calibration file there (Stereolabs serves it at " + self.CAL_URL +
                      ("<serial>" if serial is None else serial) + ") and restart the driver. Publishing unrectified" +
                      " left and right images and no depth" + (" (" + "; ".join(notes) + ")" if len(notes) > 0 else ""))
    return [False, self.uncal_msg, True]

  def parse_calibration_file(self, file_path):
    """Parse a ZED SN<serial>.conf calibration file.

    Args:
      file_path (str): Path to the file.

    Returns:
      list: [success, msg, cal_dict]. cal_dict holds 'left' and 'right' (per
      resolution key: fx, fy, cx, cy, k1, k2, p1, p2, k3), 'stereo' (baseline_mm,
      ty, tz, and per resolution key cv, rx, rz) and 'resolutions' (keys with a
      complete set of values).
    """
    try:
      with open(file_path, 'r') as f:
        text = f.read()
    except Exception as e:
      return [False, "Could not read calibration file " + file_path + ": " + str(e), None]
    [success, msg, cal_dict] = self.parseCalibrationText(text)
    if not success:
      msg = "Calibration file " + file_path + ": " + msg
    return [success, msg, cal_dict]

  def parseCalibrationText(self, text):
    parser = configparser.ConfigParser(interpolation=None, strict=False)
    try:
      parser.read_string(text)
    except Exception as e:
      return [False, "not a readable INI file (" + str(e) + ")", None]
    sections = dict()
    for name in parser.sections():
      sections[name.upper()] = dict(parser.items(name))

    cal_dict = {'left': dict(), 'right': dict(), 'stereo': dict(), 'resolutions': []}
    for key in self.CAL_KEYS:
      for side, prefix in [('left', 'LEFT_CAM_'), ('right', 'RIGHT_CAM_')]:
        sect = sections.get(prefix + key)
        if sect is None:
          continue
        vals = self.readFloats(sect, self.CAL_CAM_KEYS)
        if vals is not None:
          cal_dict[side][key] = vals

    stereo = sections.get('STEREO')
    if stereo is None:
      return [False, "no [STEREO] section", None]
    base = self.readFloats(stereo, ['baseline', 'ty', 'tz'])
    if base is None:
      return [False, "[STEREO] is missing Baseline, TY or TZ", None]
    cal_dict['stereo'] = {'baseline_mm': base['baseline'], 'ty': base['ty'], 'tz': base['tz'],
                          'cv': dict(), 'rx': dict(), 'rz': dict()}
    for key in self.CAL_KEYS:
      rot = self.readFloats(stereo, ['cv_' + key.lower(), 'rx_' + key.lower(), 'rz_' + key.lower()])
      if rot is None:
        continue
      cal_dict['stereo']['cv'][key] = rot['cv_' + key.lower()]
      cal_dict['stereo']['rx'][key] = rot['rx_' + key.lower()]
      cal_dict['stereo']['rz'][key] = rot['rz_' + key.lower()]
      if key in cal_dict['left'] and key in cal_dict['right']:
        cal_dict['resolutions'].append(key)

    if len(cal_dict['resolutions']) == 0:
      return [False, "no resolution has complete LEFT_CAM, RIGHT_CAM and STEREO rotation values", None]
    return [True, "Parsed calibration for " + str(cal_dict['resolutions']), cal_dict]

  def readFloats(self, section_dict, keys):
    vals = dict()
    for k in keys:
      if k not in section_dict:
        return None
      try:
        vals[k] = float(section_dict[k])
      except Exception:
        return None
      if not math.isfinite(vals[k]):
        return None
    return vals

  def downloadCalibration(self, serial, save_path):
    url = self.CAL_URL + serial
    try:
      response = urllib.request.urlopen(url, timeout=self.CAL_DOWNLOAD_TIMEOUT_S)
      status = response.getcode()
      body = response.read(self.CAL_DOWNLOAD_MAX_BYTES + 1)
      response.close()
    except Exception as e:
      return [False, "calibration download from " + url + " failed: " + str(e), None]
    if status != 200:
      return [False, "calibration download from " + url + " returned HTTP " + str(status), None]
    if len(body) > self.CAL_DOWNLOAD_MAX_BYTES:
      return [False, "calibration download from " + url + " was larger than " + str(self.CAL_DOWNLOAD_MAX_BYTES) + " bytes", None]
    text = body.decode('utf-8', errors='replace')
    [success, msg, cal_dict] = self.parseCalibrationText(text)
    if not success:
      return [False, "calibration downloaded from " + url + " did not parse: " + msg, None]
    try:
      if not os.path.isdir(self.cal_folder):
        os.makedirs(self.cal_folder)
      tmp_path = save_path + '.tmp'
      with open(tmp_path, 'w') as f:
        f.write(text)
      os.replace(tmp_path, save_path)
    except Exception as e:
      return [True, "downloaded calibration from " + url + " but could not save it as " + save_path + " (" + str(e) + "); using it for this run only", cal_dict]
    return [True, "downloaded calibration from " + url + " and saved it as " + save_path, cal_dict]

  def buildRectification(self, cal_dict):
    key = self.res_dict['cal_key']
    if key not in cal_dict['resolutions']:
      return [False, "calibration has no complete " + key + " values, needed for " + self.resolution]
    w, h = self.res_dict['eye']
    lc = cal_dict['left'][key]
    rc = cal_dict['right'][key]
    st = cal_dict['stereo']
    K1 = np.array([[lc['fx'], 0, lc['cx']], [0, lc['fy'], lc['cy']], [0, 0, 1]], dtype=np.float64)
    D1 = np.array([lc['k1'], lc['k2'], lc['p1'], lc['p2'], lc['k3']], dtype=np.float64)
    K2 = np.array([[rc['fx'], 0, rc['cx']], [0, rc['fy'], rc['cy']], [0, 0, 1]], dtype=np.float64)
    D2 = np.array([rc['k1'], rc['k2'], rc['p1'], rc['p2'], rc['k3']], dtype=np.float64)
    rvec = np.array([st['rx'][key], st['cv'][key], st['rz'][key]], dtype=np.float64)
    # OpenCV's T maps left-camera points into the right-camera frame; the right
    # eye sits at +Baseline on the left camera's x axis, so all three are negated
    T = np.array([-st['baseline_mm'], -st['ty'], -st['tz']], dtype=np.float64)
    try:
      R, _ = cv2.Rodrigues(rvec)
      R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K1, D1, K2, D2, (w, h), R, T,
                                                        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
      lm1, lm2 = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_16SC2)
      rm1, rm2 = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_16SC2)
    except Exception as e:
      return [False, "rectification failed for " + key + ": " + str(e)]
    fx_rect = float(P1[0, 0])
    fy_rect = float(P1[1, 1])
    baseline_mm = -float(P2[0, 3]) / float(P2[0, 0])
    if not (math.isfinite(fx_rect) and math.isfinite(baseline_mm) and fx_rect > 0 and fy_rect > 0 and baseline_mm > 0):
      return [False, "rectification gave fx " + str(fx_rect) + " and baseline " + str(baseline_mm) + " mm"]
    self.P1 = P1
    self.P2 = P2
    self.Q = Q
    self.fx_rect = fx_rect
    self.baseline_mm = baseline_mm
    self.fov_deg = [math.degrees(2 * math.atan(w / (2.0 * fx_rect))),
                    math.degrees(2 * math.atan(h / (2.0 * fy_rect)))]
    self.rect_maps = [lm1, lm2, rm1, rm2]
    return [True, ("Rectified " + key + " " + str(w) + "x" + str(h) + ": fx " + str(round(fx_rect, 2)) +
                   " px, baseline " + str(round(baseline_mm, 3)) + " mm, FOV " + str(round(self.fov_deg[0], 1)) +
                   " x " + str(round(self.fov_deg[1], 1)) + " deg")]

  def is_calibrated(self):
    """Return True when a calibration is loaded and its rectification maps are built."""
    return self.calibrated

  def is_rectifying(self):
    """Return True when frames are rectified and depth is available (calibrated and frame size matched)."""
    return self.calibrated and self.size_ok and self.rect_maps is not None

  def get_fov_deg(self):
    """Return [width_deg, height_deg] from the rectified left camera, or None when uncalibrated."""
    if not self.calibrated:
      return None
    return list(self.fov_deg)

  def get_depth_disabled_msg(self):
    """Return why depth is unavailable, naming the calibration file expected when that is the cause."""
    if not self.size_ok:
      return self.size_msg
    if not self.calibrated:
      return self.uncal_msg
    return ""


  ###################
  ## Shared stereo pair

  def get_stereo_pair(self, last_timestamp = None):
    """Return copies of the left and right images of the latest frame.

    Each captured frame is split, and rectified when calibrated, once; the left
    image, right image and depth map of one frame all reuse that pair.

    Args:
      last_timestamp (float or None): Timestamp the caller already has. If the
        latest frame has it, nothing is copied and success is False with the
        timestamp set (no new frame).

    Returns:
      tuple: (left_img, right_img, timestamp, success, msg).
    """
    left, right, ts, rectified, success, msg = self.getSharedPair()
    if not success:
      return None, None, None, False, msg
    if last_timestamp is not None and ts == last_timestamp:
      return None, None, ts, False, "No new frame"
    return left.copy(), right.copy(), ts, True, msg

  def getSharedPair(self):
    self.pair_lock.acquire()
    try:
      # At most two camera reads per frame period, shared by every consumer
      now = time.time()
      if (self.pair_left is not None and self.pair_fetch_time is not None and self.frame_interval is not None and
          (now - self.pair_fetch_time) < 0.5 * self.frame_interval):
        return self.pair_left, self.pair_right, self.pair_ts, self.pair_rectified, True, "Success"
      frame, ts, success, msg = self.cam_driver.getImage()
      if not success or frame is None:
        return None, None, None, False, False, msg
      self.pair_fetch_time = now
      if ts is not None and ts == self.pair_ts and self.pair_left is not None:
        return self.pair_left, self.pair_right, self.pair_ts, self.pair_rectified, True, "Success"
      h, w = frame.shape[:2]
      half = w // 2
      left = frame[:, :half]
      right = frame[:, half:half * 2]
      rectified = False
      if self.is_rectifying():
        if (half, h) == self.res_dict['eye']:
          start = time.time()
          [lm1, lm2, rm1, rm2] = self.rect_maps
          left = cv2.remap(left, lm1, lm2, cv2.INTER_LINEAR)
          right = cv2.remap(right, rm1, rm2, cv2.INTER_LINEAR)
          rectified = True
          self.stats_lock.acquire()
          self.stats_rect_s += time.time() - start
          self.stats_rect_n += 1
          self.stats_lock.release()
        elif not self.shape_warned:
          self.shape_warned = True
          self.queueMsg('error', "Frame size " + str(w) + "x" + str(h) + " does not match the " + self.resolution +
                        " side-by-side size; publishing unrectified halves for these frames")
      if not rectified:
        left = np.ascontiguousarray(left)
        right = np.ascontiguousarray(right)
      self.pair_left = left
      self.pair_right = right
      self.pair_ts = ts
      self.pair_rectified = rectified
      self.stats_lock.acquire()
      self.stats_frames += 1
      self.stats_lock.release()
      return left, right, ts, rectified, True, "Success"
    finally:
      self.pair_lock.release()


  ###################
  ## Depth

  def get_depth_map(self, min_range_m = None, max_range_m = None, last_timestamp = None):
    """Compute the depth map of the latest frame with the active backend.

    The rectified pair is shrunk by get_depth_scale_factor() before matching, the
    disparity is post-filtered (depth_filter) at that size, then scaled back to
    the full rectified size.

    Args:
      min_range_m (float or None): Near limit in meters; nearer pixels are NaN.
      max_range_m (float or None): Far limit in meters; farther pixels are NaN.
      last_timestamp (float or None): Timestamp the caller already has. If the
        latest frame has it, no depth is computed and success is False with the
        timestamp set (no new frame).

    Returns:
      tuple: (depth_mm, timestamp, success, msg). depth_mm is float32 at the rectified
      per-eye size, aligned with the rectified left image, NaN where invalid.
    """
    if not self.is_rectifying():
      return None, None, False, self.get_depth_disabled_msg()
    left, right, ts, rectified, success, msg = self.getSharedPair()
    if not success:
      return None, None, False, msg
    if last_timestamp is not None and ts == last_timestamp:
      return None, ts, False, "No new frame"
    if not rectified:
      return None, None, False, "Frame was not rectified (frame size mismatch); no depth for it"
    self.depth_lock.acquire()
    try:
      if ts is not None and ts == self.disp_ts and self.disp_cache is not None:
        disp = self.disp_cache
      else:
        start = time.time()
        h, w = left.shape[:2]
        factor = self.get_depth_scale_factor()
        if factor > 1:
          small_size = (max(1, w // factor), max(1, h // factor))
          left_d = cv2.resize(left, small_size, interpolation=cv2.INTER_AREA)
          right_d = cv2.resize(right, small_size, interpolation=cv2.INTER_AREA)
        else:
          left_d = left
          right_d = right
        disp = None
        if self.active_backend == 'hailo':
          [success, msg, disp] = self.computeDisparityHailo(left_d, right_d)
          if not success:
            disp = None
            self.releaseHailo()
            self.queueMsg('warn', "Hailo depth backend failed, using cpu_sgbm until hailo is selected again: " + msg)
            self.setActiveBackend('cpu_sgbm', "hailo inference failed")
        if disp is None:
          disp = self.computeDisparitySgbm(left_d, right_d)
        disp = self.filter_disparity(disp, left_d)
        disp = self.upsampleDisparity(disp, w, h)
        self.stats_lock.acquire()
        self.stats_depth_s += time.time() - start
        self.stats_depth_n += 1
        self.stats_lock.release()
        self.disp_ts = ts
        self.disp_cache = disp
    finally:
      self.depth_lock.release()
    depth = self.disparity_to_depth(disp, min_range_m, max_range_m)
    return depth, ts, True, "Success"

  def disparity_to_depth(self, disparity, min_range_m = None, max_range_m = None):
    """Convert disparity (rectified pixels) to depth in millimeters.

    depth_mm = fx_rect * baseline_mm / disparity. Pixels with disparity <= 0, a
    non-finite disparity, or a depth outside the min/max range window are NaN.

    Args:
      disparity (numpy.ndarray): Disparity in rectified pixels.
      min_range_m (float or None): Near limit in meters.
      max_range_m (float or None): Far limit in meters.

    Returns:
      numpy.ndarray: float32 depth in millimeters.
    """
    disp = np.asarray(disparity, dtype=np.float32)
    depth = np.full(disp.shape, np.nan, dtype=np.float32)
    valid = np.isfinite(disp) & (disp > 0)
    depth[valid] = (self.fx_rect * self.baseline_mm) / disp[valid]
    with np.errstate(invalid='ignore'):
      if min_range_m is not None:
        depth[depth < float(min_range_m) * 1000.0] = np.nan
      if max_range_m is not None:
        depth[depth > float(max_range_m) * 1000.0] = np.nan
    return depth

  def set_depth_backend(self, backend):
    """Select the depth backend: 'cpu_sgbm' or 'hailo'.

    Selecting hailo opens the Hailo device and loads the stereo HEF; any failure
    falls back to cpu_sgbm, queued once with its cause. Selecting cpu_sgbm
    releases the Hailo device so another process can open it.

    Args:
      backend (str): 'cpu_sgbm' or 'hailo'.

    Returns:
      list: [success, msg]. success is False when hailo was requested and is unavailable.
    """
    if backend not in self.DEPTH_BACKENDS:
      return [False, "Unknown depth backend " + str(backend) + ", expected one of " + str(self.DEPTH_BACKENDS)]
    self.depth_lock.acquire()
    try:
      self.requested_backend = backend
      if backend == 'cpu_sgbm':
        self.releaseHailo()
        self.setActiveBackend('cpu_sgbm', "selected")
        return [True, "Depth backend set to cpu_sgbm"]
      if self.active_backend == 'hailo' and self.hailo_network_group is not None:
        return [True, "Depth backend hailo already active"]
      [success, msg] = self.loadHailo()
      if success:
        self.setActiveBackend('hailo', "selected")
        return [True, msg]
      self.releaseHailo()
      fallback_msg = "Hailo depth backend unavailable, using cpu_sgbm: " + msg
      self.queueMsg('warn', fallback_msg)
      self.setActiveBackend('cpu_sgbm', "hailo unavailable")
      return [False, fallback_msg]
    finally:
      self.depth_lock.release()

  def get_active_backend(self):
    """Return the depth backend in use: 'cpu_sgbm' or 'hailo'."""
    return self.active_backend

  def setActiveBackend(self, backend, cause):
    if backend != self.active_backend:
      self.queueMsg('info', "Depth backend changed from " + self.active_backend + " to " + backend + " (" + cause + ")")
    self.active_backend = backend
    self.disp_ts = None
    self.disp_cache = None

  def set_sgbm_params(self, num_disparities, block_size):
    """Set the cpu_sgbm disparity range and block size.

    num_disparities is rounded to a multiple of 16 within 16..256; block_size is
    clamped to 3..11 and forced odd.

    Args:
      num_disparities (int): Disparity search range in pixels.
      block_size (int): Matching block size in pixels.

    Returns:
      list: [num_disparities, block_size] as applied.
    """
    nd = int(round(float(num_disparities) / 16.0)) * 16
    nd = max(self.SGBM_NUM_DISP_BOUNDS[0], min(self.SGBM_NUM_DISP_BOUNDS[1], nd))
    bs = int(round(float(block_size)))
    bs = max(self.SGBM_BLOCK_SIZE_BOUNDS[0], min(self.SGBM_BLOCK_SIZE_BOUNDS[1], bs))
    if bs % 2 == 0:
      bs = bs + 1 if bs < self.SGBM_BLOCK_SIZE_BOUNDS[1] else bs - 1
    self.depth_lock.acquire()
    self.sgbm_num_disparities = nd
    self.sgbm_block_size = bs
    self.sgbm = None
    self.disp_ts = None
    self.disp_cache = None
    self.depth_lock.release()
    return [nd, bs]

  def set_hailo_swap_inputs(self, swap):
    """Swap which rectified image feeds the hailo model's first input (default: left to the first input by sorted name).

    Args:
      swap (bool): True feeds the right image to the first input.
    """
    self.depth_lock.acquire()
    self.hailo_swap_inputs = bool(swap)
    self.disp_ts = None
    self.disp_cache = None
    self.depth_lock.release()

  def set_depth_filter(self, depth_filter):
    """Select the disparity post-filter: 'none', 'median' or 'fill'.

    'median' is a 5x5 median. 'fill' is the median followed by an edge-aware hole
    fill guided by the left image; it leaves areas with too little valid support
    empty rather than inventing depth there.

    Args:
      depth_filter (str): One of DEPTH_FILTERS.

    Returns:
      list: [success, msg].
    """
    depth_filter = str(depth_filter)
    if depth_filter not in self.DEPTH_FILTERS:
      return [False, "Unknown depth filter " + depth_filter + ", expected one of " + str(self.DEPTH_FILTERS)]
    self.depth_lock.acquire()
    self.depth_filter = depth_filter
    self.disp_ts = None
    self.disp_cache = None
    self.depth_lock.release()
    return [True, "Depth filter set to " + depth_filter]

  def set_depth_downscale(self, downscale):
    """Select how much the rectified pair is shrunk before depth is computed.

    'auto' brings the per-eye width nearest VGA (factor 1 at VGA, 2 at HD720,
    3 at HD1080 and HD2K). The depth map is always published at the full
    rectified size. sgbm_num_disparities applies at the shrunk size.

    Args:
      downscale (str or int): One of DEPTH_DOWNSCALE_OPTIONS.

    Returns:
      list: [success, msg].
    """
    downscale = str(downscale)
    if downscale not in self.DEPTH_DOWNSCALE_OPTIONS:
      return [False, "Unknown depth downscale " + downscale + ", expected one of " + str(self.DEPTH_DOWNSCALE_OPTIONS)]
    self.depth_lock.acquire()
    self.depth_downscale = downscale
    self.disp_ts = None
    self.disp_cache = None
    self.depth_lock.release()
    return [True, "Depth downscale set to " + downscale + " (factor " + str(self.get_depth_scale_factor()) + ")"]

  def get_depth_scale_factor(self):
    """Return the integer factor the rectified pair is shrunk by before depth is computed."""
    if self.depth_downscale == 'auto':
      eye_w = self.res_dict['eye'][0]
      return max(1, int(round(float(eye_w) / float(self.DEPTH_AUTO_TARGET_WIDTH))))
    try:
      return max(1, int(self.depth_downscale))
    except Exception:
      return 1

  def filter_disparity(self, disparity, left_rect):
    """Apply the selected depth_filter to a disparity map.

    Args:
      disparity (numpy.ndarray): Disparity in pixels, <= 0 or non-finite where invalid.
      left_rect (numpy.ndarray): Rectified left image at the same size, BGR or gray (the fill guide).

    Returns:
      numpy.ndarray: float32 disparity, 0 where invalid.
    """
    disp = np.array(disparity, dtype=np.float32)
    disp[~np.isfinite(disp)] = 0
    disp[disp < 0] = 0
    if self.depth_filter == 'none':
      return disp
    disp = cv2.medianBlur(disp, self.MEDIAN_KSIZE)
    if self.depth_filter == 'median':
      return disp
    gray = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY) if left_rect.ndim == 3 else left_rect
    return self.fillDisparity(disp, gray)

  def fillDisparity(self, disp, gray):
    ksize = (2 * self.FILL_RADIUS + 1, 2 * self.FILL_RADIUS + 1)
    guide = gray.astype(np.float32) / 255.0
    valid = (disp > 0).astype(np.float32)
    mean_i = cv2.boxFilter(guide, -1, ksize)
    var_i = cv2.boxFilter(guide * guide, -1, ksize) - mean_i * mean_i
    num = self.guidedFilter(guide, mean_i, var_i, disp * valid, ksize)
    den = self.guidedFilter(guide, mean_i, var_i, valid, ksize)
    out = np.zeros_like(disp)
    ok = den > self.FILL_MIN_SUPPORT
    out[ok] = num[ok] / den[ok]
    out[out < 0] = 0
    return out

  def guidedFilter(self, guide, mean_i, var_i, src, ksize):
    mean_p = cv2.boxFilter(src, -1, ksize)
    cov_ip = cv2.boxFilter(guide * src, -1, ksize) - mean_i * mean_p
    a = cov_ip / (var_i + self.FILL_EPS)
    b = mean_p - a * mean_i
    return cv2.boxFilter(a, -1, ksize) * guide + cv2.boxFilter(b, -1, ksize)

  def upsampleDisparity(self, disp, width, height):
    # Interpolates only between valid pixels (num/den), so a hole's edge is not
    # pulled toward 0; values are scaled into full-size pixels
    h, w = disp.shape[:2]
    if w == width and h == height:
      return disp
    valid = (disp > 0).astype(np.float32)
    num = cv2.resize(disp * valid, (width, height), interpolation=cv2.INTER_LINEAR)
    den = cv2.resize(valid, (width, height), interpolation=cv2.INTER_LINEAR)
    valid_up = cv2.resize(valid, (width, height), interpolation=cv2.INTER_NEAREST) > 0
    out = np.zeros((height, width), dtype=np.float32)
    ok = valid_up & (den > 1e-3)
    out[ok] = num[ok] / den[ok]
    return out * (float(width) / float(w))

  def computeDisparitySgbm(self, left, right):
    nd = self.sgbm_num_disparities
    if self.sgbm is None:
      bs = self.sgbm_block_size
      self.sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=nd, blockSize=bs,
                                        P1=8 * bs * bs, P2=32 * bs * bs,
                                        disp12MaxDiff=self.SGBM_DISP12_MAX_DIFF,
                                        uniquenessRatio=self.SGBM_UNIQUENESS_RATIO,
                                        speckleWindowSize=self.SGBM_SPECKLE_WINDOW_SIZE,
                                        speckleRange=self.SGBM_SPECKLE_RANGE,
                                        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY) if left.ndim == 3 else left
    right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY) if right.ndim == 3 else right
    # Pad on the left so SGBM's invalid band (the first numDisparities columns) falls in the padding
    left_pad = cv2.copyMakeBorder(left_gray, 0, 0, nd, 0, cv2.BORDER_CONSTANT, value=0)
    right_pad = cv2.copyMakeBorder(right_gray, 0, 0, nd, 0, cv2.BORDER_CONSTANT, value=0)
    disp16 = self.sgbm.compute(left_pad, right_pad)
    return disp16[:, nd:].astype(np.float32) / 16.0


  ###################
  ## Hailo backend

  def findHailoModel(self):
    hef_files = sorted(glob.glob(os.path.join(self.hailo_folder, '*.hef')))
    if len(hef_files) == 0:
      return None
    return hef_files[0]

  def loadHailo(self):
    if not HAILO_AVAILABLE:
      return [False, "hailo_platform did not import (" + str(HAILO_IMPORT_ERROR) + ")"]
    hef_path = self.findHailoModel()
    if hef_path is None:
      return [False, "no .hef file found in " + self.hailo_folder]
    try:
      self.hailo_device = VDevice()
      self.hailo_hef = HEF(hef_path)
      configure_params = ConfigureParams.create_from_hef(self.hailo_hef, interface=HAILO_INTERFACE)
      self.hailo_network_group = self.hailo_device.configure(self.hailo_hef, configure_params)[0]
      self.hailo_network_group_params = self.hailo_network_group.create_params()
      input_infos = list(self.hailo_hef.get_input_vstream_infos())
      output_infos = list(self.hailo_hef.get_output_vstream_infos())
    except Exception as e:
      self.releaseHailo()
      return [False, ("loading " + hef_path + " on the Hailo device raised: " + str(e) +
                      " (the device may be held by another process such as a running Hailo detector," +
                      " or the HEF was compiled for another chip or a newer HailoRT)")]
    [success, msg] = self.setHailoStreamInfo(input_infos, output_infos)
    if not success:
      self.releaseHailo()
      return [False, hef_path + " does not match the stereo contract (two inputs, one output): " + msg]
    try:
      self.hailo_input_params = InputVStreamParams.make_from_network_group(self.hailo_network_group, quantized=False, format_type=FormatType.UINT8)
      self.hailo_output_params = OutputVStreamParams.make_from_network_group(self.hailo_network_group, quantized=False, format_type=FormatType.FLOAT32)
    except Exception as e:
      self.releaseHailo()
      return [False, "building vstream params for " + hef_path + " raised: " + str(e)]
    self.hailo_hef_path = hef_path
    self.hailo_first_inference = True
    left_idx = 1 if self.hailo_swap_inputs else 0
    in_desc = []
    for i, inp in enumerate(self.hailo_inputs):
      in_desc.append(inp[0] + " " + str(inp[1]) + " " + inp[2] + (" (left)" if i == left_idx else " (right)"))
    out = self.hailo_output
    self.queueMsg('info', "Loaded Hailo stereo model " + hef_path + ": inputs " + ", ".join(in_desc) +
                  "; output " + out[0] + " " + str(out[1]) + " " + out[2])
    return [True, "Loaded Hailo stereo model " + hef_path]

  def setHailoStreamInfo(self, input_infos, output_infos):
    if len(input_infos) != 2 or len(output_infos) != 1:
      return [False, "found " + str(len(input_infos)) + " inputs and " + str(len(output_infos)) + " outputs"]
    inputs = []
    for info in sorted(input_infos, key=lambda i: i.name):
      shape = tuple(int(s) for s in info.shape)
      if len(shape) != 3 or shape[2] != 3:
        return [False, "input " + info.name + " has shape " + str(shape) + ", expected (height, width, 3)"]
      inputs.append([info.name, shape, self.hailoFormatStr(info)])
    if inputs[0][1] != inputs[1][1]:
      return [False, "input shapes differ: " + str(inputs[0][1]) + " and " + str(inputs[1][1])]
    out_info = output_infos[0]
    out_shape = tuple(int(s) for s in out_info.shape)
    if not (len(out_shape) == 2 or (len(out_shape) == 3 and out_shape[2] == 1)):
      return [False, "output " + out_info.name + " has shape " + str(out_shape) + ", expected (height, width) or (height, width, 1)"]
    self.hailo_inputs = inputs
    self.hailo_output = [out_info.name, out_shape, self.hailoFormatStr(out_info)]
    return [True, "Success"]

  def hailoFormatStr(self, info):
    fmt = getattr(info, 'format', None)
    if fmt is None:
      return "format unknown"
    fmt_type = getattr(fmt, 'type', None)
    fmt_order = getattr(fmt, 'order', None)
    if fmt_type is None and fmt_order is None:
      return str(fmt)
    return str(fmt_type) + " " + str(fmt_order)

  def hailo_preprocess(self, left_rect, right_rect):
    """Build the hailo model's input dict from a rectified BGR pair.

    Each image is converted BGR to RGB and resized to the model input size, and
    passed as raw uint8 with a batch axis (normalization is inside the network).
    The left image feeds the first input by sorted vstream name, unless
    hailo_swap_inputs is on.

    Args:
      left_rect (numpy.ndarray): Rectified left image, BGR or gray.
      right_rect (numpy.ndarray): Rectified right image, BGR or gray.

    Returns:
      dict: {input vstream name: uint8 array (1, height, width, 3)}.
    """
    in_h, in_w = self.hailo_inputs[0][1][0], self.hailo_inputs[0][1][1]
    if self.hailo_swap_inputs:
      left_name, right_name = self.hailo_inputs[1][0], self.hailo_inputs[0][0]
    else:
      left_name, right_name = self.hailo_inputs[0][0], self.hailo_inputs[1][0]
    input_data = dict()
    for name, img in [(left_name, left_rect), (right_name, right_rect)]:
      if img.ndim == 2:
        img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
      else:
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
      resized = cv2.resize(img_rgb, (in_w, in_h), interpolation=cv2.INTER_LINEAR)
      input_data[name] = np.expand_dims(resized, axis=0).astype(np.uint8)
    return input_data

  def hailo_postprocess(self, results, rect_width, rect_height):
    """Turn the hailo model output into disparity at the rectified size.

    The output is read as float32 disparity in model-input pixels, resized to the
    rectified size and scaled by rect_width / input_width. On the first good
    inference after each load, its min, median and max (before scaling) are queued
    as an info message.

    Args:
      results (dict): {output vstream name: array} from the inference call.
      rect_width (int): Rectified image width.
      rect_height (int): Rectified image height.

    Returns:
      list: [success, msg, disparity] with disparity float32 (rect_height, rect_width).
    """
    out_name = self.hailo_output[0]
    if results is None or out_name not in results:
      return [False, "inference returned no output " + out_name, None]
    out = np.squeeze(np.asarray(results[out_name], dtype=np.float32))
    if out.ndim != 2:
      return [False, "output " + out_name + " has shape " + str(np.shape(results[out_name])) + " after squeeze, expected 2-D", None]
    if self.hailo_first_inference:
      self.hailo_first_inference = False
      self.queueMsg('info', "First Hailo stereo output (before scaling): min " + str(round(float(np.nanmin(out)), 3)) +
                    " median " + str(round(float(np.nanmedian(out)), 3)) + " max " + str(round(float(np.nanmax(out)), 3)) +
                    " for input width " + str(self.hailo_inputs[0][1][1]))
    in_w = self.hailo_inputs[0][1][1]
    disp = cv2.resize(out, (int(rect_width), int(rect_height)), interpolation=cv2.INTER_LINEAR)
    disp = disp * (float(rect_width) / float(in_w))
    return [True, "Success", disp.astype(np.float32)]

  def runHailoInference(self, input_data):
    with InferVStreams(self.hailo_network_group, self.hailo_input_params, self.hailo_output_params) as infer_pipeline:
      with self.hailo_network_group.activate(self.hailo_network_group_params):
        results = infer_pipeline.infer(input_data)
    return results

  def computeDisparityHailo(self, left, right):
    if self.hailo_network_group is None:
      return [False, "Hailo model not loaded", None]
    try:
      input_data = self.hailo_preprocess(left, right)
      results = self.runHailoInference(input_data)
    except Exception as e:
      return [False, "inference raised: " + str(e), None]
    h, w = left.shape[:2]
    return self.hailo_postprocess(results, w, h)

  def releaseHailo(self):
    if self.hailo_device is not None:
      try:
        self.hailo_device.release()
      except Exception:
        pass
    self.hailo_network_group = None
    self.hailo_network_group_params = None
    self.hailo_input_params = None
    self.hailo_output_params = None
    self.hailo_hef = None
    self.hailo_device = None


  ###################
  ## Timing and cleanup

  def get_timing_stats(self):
    """Return timing since the last call, and reset the window.

    Returns:
      dict: capture_fps (new frames split per second), rectify_ms and depth_ms
      (mean per frame, None if none ran), backend (active depth backend).
    """
    self.stats_lock.acquire()
    elapsed = max(time.time() - self.stats_start, 1e-6)
    stats = {'capture_fps': self.stats_frames / elapsed,
             'rectify_ms': (1000.0 * self.stats_rect_s / self.stats_rect_n) if self.stats_rect_n > 0 else None,
             'depth_ms': (1000.0 * self.stats_depth_s / self.stats_depth_n) if self.stats_depth_n > 0 else None,
             'backend': self.active_backend}
    self.stats_start = time.time()
    self.stats_frames = 0
    self.stats_rect_s = 0.0
    self.stats_rect_n = 0
    self.stats_depth_s = 0.0
    self.stats_depth_n = 0
    self.stats_lock.release()
    return stats

  def cleanup(self):
    """Release the Hailo device and network group, if held."""
    self.depth_lock.acquire()
    try:
      self.releaseHailo()
    finally:
      self.depth_lock.release()
