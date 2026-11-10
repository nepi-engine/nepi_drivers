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

# RBX driver node for a Gazebo simulated robot, following
# rbx_ardupilot_node.py's RBXRobotIF integration pattern. All Gazebo knowledge
# lives in this file and its discovery script -- nothing else in NEPI knows this
# is Gazebo.
#
# A Gazebo instance runs its own ROS master, so its ROS graph is invisible from
# this device. This node therefore holds a persistent TCP connection to the
# Gazebo-side bridge script (host and port from DEVICE_DICT) speaking
# newline-delimited JSON: velocity commands out, odometry telemetry and relayed
# camera frames in.
#
# Unlike the ardupilot driver there is no onboard autopilot to delegate goto
# setpoints to. The robot only understands instantaneous velocity, so this node
# runs its own closed-loop 2D controller (gotoControlCb) that drives toward the
# RBXRobotIF-supplied position and yaw target until within the RBX error bounds.
#
# The robot has no arm/disarm or flight-mode equivalent, no battery, and no
# geographic (WGS84) location, so states, modes, gotoLocation, and the battery
# callback are legitimately empty or None -- RBXRobotIF reports the matching
# has_* capabilities False, which is exactly how the capability model is meant to
# be used. Home and set-home ARE wired, reusing RBXRobotIF's GeoPoint-shaped home
# plumbing with its three floats reinterpreted as local ENU x/y/z meters, the
# only home concept a robot with no WGS84 reference has.
#
# ############################################################################
# Two-camera convention. This driver defines exactly two cameras and populates
# the generic sensor topic list with those two named entries:
#
#   scene_camera  Third-person view. Reference frame is the robot's body frame;
#                 default pose is an offset relative to it (up and back, angled
#                 down slightly). Fully modifiable in principle.
#   robot_camera  Onboard/FPV view. Reference frame is base_link, this robot's
#                 main reference frame -- the root link every other link and
#                 both camera links are jointed to. Default pose is coincident
#                 with that frame unless configured otherwise.
#
# Live camera pose adjustment (offset and angle) is deliberately NOT given a wire
# shape here. The generic contract's camera control surface is a single view-mode
# string, not a structured multi-axis pose, and whether that gets extended,
# replaced, or paired with a new pose control is a decision for when the
# contract's camera surface is settled -- guessing a wire shape now would
# hard-code the wrong one. What this driver exposes instead this pass: the
# camera_view_mode setting, whose two discrete options name the two cameras, and
# which of them is relayed as the live image data product. The reference frames
# and default offsets above are declarative, not commandable.
# ############################################################################

import base64
import copy
import json
import math
import os
import socket
import threading
import time

import numpy as np
import cv2

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_img

from std_msgs.msg import UInt32, String
from sensor_msgs.msg import Image

from geographic_msgs.msg import GeoPoint

from nepi_api.device_if_rbx import RBXRobotIF
from nepi_api.device_if_npx import NPXDeviceIF
from nepi_api.device_if_svx import DeviceSVXIF
from nepi_api.messages_if import MsgIF
from nepi_api.data_product_if_image import ColorImageIF

PKG_NAME = 'RBX_GAZEBO' # Use in display menus
FILE_TYPE = 'NODE'


#########################################
# Node Class
#########################################

class GazeboNode:

  # The two cameras this driver defines. SCENE_CAMERA is the third-person view,
  # ROBOT_CAMERA the onboard/FPV view. These names are the camera_view_mode
  # setting's options AND the names the generic sensor topic list is populated
  # with, so a client sees one consistent pair of names either way.
  SCENE_CAMERA_NAME = "scene_camera"
  ROBOT_CAMERA_NAME = "robot_camera"
  CAMERA_VIEW_MODES = [SCENE_CAMERA_NAME.upper(), ROBOT_CAMERA_NAME.upper()]

  # Reference frames the two cameras are defined against. Declarative for this
  # pass -- reported, not commandable. base_link is this robot's main reference
  # frame: the root link both camera links are fixed-jointed to.
  ROBOT_MAIN_REFERENCE_FRAME = "base_link"
  # Default scene_camera offset from the body frame, in meters, plus a downward
  # tilt in degrees. Placeholder values pending real tuning against a running
  # Gazebo instance; not commandable this pass.
  SCENE_CAMERA_DEFAULT_OFFSET_M = [-2.0, 0.0, 3.0]
  SCENE_CAMERA_DEFAULT_TILT_DEG = -20.0

  # Environment options this driver's bridge supports. Reported as the generic
  # environment option list rather than hardcoded into a control.
  ENVIRONMENT_OPTIONS = ["FLAT_GROUND", "OBSTACLE_COURSE"]
  OBSTACLE_COURSE_OPTION = "OBSTACLE_COURSE"

  CAMERA_SETTING_NAMES = ("camera_view_mode",)
  ENVIRONMENT_SETTING_NAMES = ("environment",)

  CAP_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","options":["0.05","5.0"]},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","options":["5.0","180.0"]},
    camera_view_mode = {"type":"Selection","name":"camera_view_mode","options":CAMERA_VIEW_MODES},
    environment = {"type":"Selection","name":"environment","options":ENVIRONMENT_OPTIONS}
  )

  FACTORY_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","value":"0.5"},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","value":"45.0"},
    camera_view_mode = {"type":"Selection","name":"camera_view_mode","value":ROBOT_CAMERA_NAME.upper()},
    environment = {"type":"Selection","name":"environment","value":ENVIRONMENT_OPTIONS[0]}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  # A differential-drive ground robot has no arm/disarm or flight-mode
  # machinery. RBXRobotIF handles empty lists correctly (bounds checks reject any
  # set_state/set_mode index, status shows "Not Set"), so no placeholder entries
  # are invented. The get*Ind functions must still be real callables, since
  # RBXRobotIF calls them unconditionally.
  RBX_STATES = []
  RBX_MODES = []
  # Two one-shot setup commands: an instant physics teleport back to the spawn
  # pose, and driving back to the user-settable home position under closed-loop
  # control. RETURN_HOME is also wired as the goHomeFunction so the standard Go
  # Home control works too.
  RBX_SETUP_ACTIONS = ["RESET_SIM", "RETURN_HOME"]
  RBX_GO_ACTIONS = []

  # RETURN_HOME polls the same goto_target the goto setpoint functions drive
  # (gotoControlCb clears it on arrival) rather than returning immediately,
  # because RBXRobotIF treats this function's return value as the command's
  # success or failure, not merely as "accepted".
  GO_HOME_TIMEOUT_SEC = 60.0
  GO_HOME_POLL_INTERVAL_SEC = 0.2

  RECONNECT_INTERVAL_SEC = 3.0
  SOCKET_TIMEOUT_SEC = 5.0

  # At the top of the settable max_linear_speed_mps range a 10 Hz control tick
  # would let the robot travel up to 0.5 m between heading corrections, coarse
  # enough to overshoot and oscillate. Finer ticks cut that per-tick travel.
  CONTROLLER_RATE_HZ = 20
  NAVPOSE_UPDATE_RATE = 10
  TELEMETRY_FRESH_SEC = 2.0

  # Manual motor-ratio tank-drive conversion: motor 0 = left, motor 1 = right,
  # converted to the same linear/angular velocity pair gotoControlCb already
  # sends via standard differential-drive kinematics -- so no new bridge message
  # or Gazebo plugin is needed, since the control loop already owns sending
  # velocity every tick. Per MotorControl.msg, speed_ratio is a 0-1 magnitude
  # with no direction bit, so this can drive straight and steer differentially
  # but cannot reverse or spin in place. That is an honest limitation of the wire
  # format, not of this conversion.
  MOTOR_MAX_LINEAR_MPS = 0.5
  MOTOR_WHEEL_BASE_M = 0.4

  # Closed-loop goto controller: proportional gains plus a turn-in-place gate so
  # the robot rotates toward the target bearing before driving.
  GOTO_KP_LIN = 0.5       # m/s per m of distance error
  GOTO_KP_ANG = 1.5       # rad/s per rad of heading error
  GOTO_TURN_GATE_RAD = math.radians(30.0)
  # Stop inside half the RBX error bounds so the interface's own convergence
  # check (error below bound, sustained for the stabilize window) passes cleanly
  # instead of hovering at the edge.
  GOTO_TOL_FRACTION = 0.5
  # Fallbacks if the RBX interface is not up yet, matching GOTO_TOL_FRACTION of
  # RBXRobotIF's factory 2.0 m / 2.0 deg error bounds.
  FACTORY_GOTO_TOL_M = 1.0
  FACTORY_GOTO_TOL_RAD = math.radians(1.0)

  # RBXRobotIF's blocking setpoint wait uses one fixed cmd_timeout for both the
  # drive and the final-yaw phase. A non-holonomic differential-drive robot can
  # legitimately need close to a full 360 degrees of cumulative turning for a
  # single goto (turn to face the bearing, drive, then turn again to the
  # requested final yaw) plus the full commanded travel distance at the capped
  # max_linear_speed_mps. The RBXRobotIF factory default of 25 s was measured too
  # short for that: a 6 m body-frame goto needing two roughly 180 degree turns
  # timed out with cmd_success False even though the controller went on to
  # converge about 2.5 s later by its own independently correct tolerance check.
  # Raised here to cover a full reorientation (360 deg at the factory 45 deg/s =
  # 8 s) plus a generous single-command travel distance for a simulated world
  # (20 m at the factory 0.5 m/s = 40 s) with margin. A heuristic sized to these
  # factory speed defaults and a reasonably-sized world, not a guarantee for
  # arbitrarily long commands or a user who raises the speed caps. The broader
  # design question -- RBXRobotIF's fixed-timeout model does not scale with
  # maneuver complexity for non-holonomic ground vehicles in general -- is
  # deferred, not solved here.
  GOTO_CMD_TIMEOUT_SEC = 60

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

  # A ground robot moves in the plane: x, y and yaw are commandable, z, roll
  # and pitch are not.

  caps_dict = dict()
  caps_dict['has_x_control'] = True
  caps_dict['has_y_control'] = True
  caps_dict['has_z_control'] = False

  caps_dict['has_roll_control'] = False
  caps_dict['has_pitch_control'] = False
  caps_dict['has_yaw_control'] = True


  drv_dict = dict()

  rbx_if = None

  # NPX device that publishes this robot's navpose (see selectOwnNavPoseCb)
  npx_if = None
  node_if = None
  settings_if = None
  save_data_if = None
  connect_navpose_if = None
  OWN_NAVPOSE_MAX_TRIES = 60 # 1 s apart
  own_navpose_topic = None
  own_navpose_tries = 0

  # Main camera published as a NEPI color image data product
  # (see selectOwnImageCb)
  robot_color_if = None
  OWN_IMAGE_MAX_TRIES = 60 # 1 s apart
  own_image_tries = 0

  # Every motor is its own DeviceSVXIF, named <device name>_motor_<n> (n from
  # 1), keyed by that name in motor_svx_if_dict, which RBXRobotIF gets as its
  # servo_if_dict. See createMotorSvxIfs.
  motor_svx_if_dict = dict()
  motor_svx_dirs = []
  motor_svx_speed_ratios = []
  motor_svx_stop_triggers = []
  motor_svx_cmds_enabled = False

  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF()
    self.msg_if.pub_info("Starting Node Initialization Processes")

    ##############################
    # Gather Driver Settings from param server drv_dict
    self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
    try:
      self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
      self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
      self.sim_host = self.drv_dict['DEVICE_DICT']['host']
      self.bridge_port = self.drv_dict['DEVICE_DICT']['bridge_port']
    except Exception as e:
      self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
      return

    ##############################
    # Bridge connection and telemetry state
    self.sock = None
    self.sock_lock = threading.Lock()
    self.last_telemetry_time = 0.0
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

    ##############################
    # Two-camera image relay. Frames arrive over the bridge already selected by
    # the Gazebo side according to the current camera_view_mode, and are
    # republished on this instance's own image topic.
    #
    # Deliberately a BARE relative name rather than one built from
    # self.node_namespace: RBXRobotIF's own image subscribe is also a bare
    # relative subscribe (its find_topic returns the search string rather than a
    # matched full path), and a bare relative name resolves against the namespace
    # shared by every driver node on this device, not this node's own -- the
    # driver launch helper remaps the node name, never the namespace. The
    # device_name is baked into the string so a second instance cannot resolve to
    # the same shared topic and bleed its frames into this one's data product.
    self.image_topic_name = self.device_name + "/color_2d_image"
    self.image_pub = nepi_sdk.create_publisher(self.image_topic_name, Image, queue_size = 1)

    ##############################
    # The generic sensor topic list this driver populates, as exactly two named
    # entries per the two-camera convention. Both surface through the same
    # generic mechanism; which one is streaming is the camera_view_mode setting.
    self.sensor_topics = [
      (self.image_topic_name + "/" + self.SCENE_CAMERA_NAME, 'sensor_msgs/Image'),
      (self.image_topic_name + "/" + self.ROBOT_CAMERA_NAME, 'sensor_msgs/Image'),
    ]

    ##############################
    # Goto controller state
    self.goto_target = None       # dict(x_m, y_m, yaw_deg or None) in sim ENU world frame
    self.goto_target_lock = threading.Lock()
    self.stop_triggered = False

    ##############################
    # Manual motor-ratio state: motor 0 = left, motor 1 = right
    self.motor_ratios = [0.0, 0.0]
    # One servo (SVX) device per motor, named <device name>_motor_<n>. The
    # motor count is fixed here at construction (motor_ratios above); any
    # change to it needs a node restart. The names are entered now so
    # RBXRobotIF sees its manual (servo) controls at construction; the
    # DeviceSVXIF instances that share its IFs fill them in afterwards
    # (createMotorSvxIfs).
    motor_count = len(self.motor_ratios)
    self.motor_svx_if_dict = dict()
    for motor_ind in range(motor_count):
      self.motor_svx_if_dict[self.device_name + "_motor_" + str(motor_ind + 1)] = None
    self.motor_svx_dirs = [1] * motor_count
    self.motor_svx_speed_ratios = [0.0] * motor_count
    self.motor_svx_stop_triggers = [False] * motor_count

    ##############################
    # Home position state: local ENU x/y/z meters, carried over RBXRobotIF's
    # existing GeoPoint-based home plumbing (see getHome/setHome).
    self.home_x_m = 0.0
    self.home_y_m = 0.0
    self.home_z_m = 0.0

    ##############################
    # Initialize RBX Settings
    self.settings_dict = self.initSettingsDict()
    self.settings_dict = self.refreshSettingsDict()

 

    ##############################
    # Bridge client thread: connects, reads telemetry, reconnects on failure
    self.bridge_thread = threading.Thread(target = self.bridgeLoop)
    self.bridge_thread.daemon = True
    self.bridge_thread.start()

    ##############################
    # Launch the NEPI RBX interface. Every callback the Gazebo robot does not
    # support is passed None, so the matching capability flags fall out of
    # construction rather than being declared anywhere.
    #
    # data_source_description is what makes this device discoverable as a
    # simulator by any simulator-aware consumer: it is a self-declaration of
    # kind, published in the device's own status, and it is deliberately generic
    # rather than naming this simulator.
    self.msg_if.pub_info("Launching NEPI RBX interface...")
    self.device_info_dict = dict(device_name = self.device_name,
                                 path = self.device_path,
                                 serial_number = "",
                                 hw_version = "",
                                 sw_version = "")
    self.msg_if.pub_info(str(self.device_info_dict))

    self.rbx_if = RBXRobotIF(device_info = self.device_info_dict,
                             getSettingsFunction = self.getSettingsFunction,
                             setSettingFunction = self.setSettingFunction,
                             axisCapabilities = self.caps_dict,
                             getBatteryPercentFunction = None,
                             states = self.RBX_STATES,
                             getStateIndFunction = self.getStateInd,
                             setStateIndFunction = self.setStateInd,
                             modes = self.RBX_MODES,
                             getModeIndFunction = self.getModeInd,
                             setModeIndFunction = self.setModeInd,
                             checkStopFunction = self.checkStopFunction,
                             setup_actions = self.RBX_SETUP_ACTIONS,
                             setSetupActionIndFunction = self.setSetupActionInd,
                             go_actions = self.RBX_GO_ACTIONS,
                             setGoActionIndFunction = self.setGoActionInd,
                             manualControlsReadyFunction = self.manualControlsReady,
                             # One DeviceSVXIF per motor, see createMotorSvxIfs
                             servo_if_dict = self.motor_svx_if_dict,
                             autonomousControlsReadyFunction = self.autonomousControlsReady,
                             getHomeFunction = self.getHome,
                             setHomeFunction = self.setHome,
                             goHomeFunction = self.returnHomeAction,
                             goStopFunction = self.goStop,
                             gotoPoseFunction = self.gotoPose,
                             gotoPositionFunction = self.gotoPosition,
                             gotoLocationFunction = None,
                             data_source_description = 'simulator',
                             data_ref_description = 'simulator',
                             msg_if = self.msg_if
                            )

    self.msg_if.pub_info("... RBX interface running")
    ready = self.rbx_if.wait_for_ready()

    self.node_if = self.rbx_if.node_if
    self.settings_if = self.rbx_if.settings_if
    self.save_data_if = self.rbx_if.save_data_if
    self.connect_navpose_if = self.rbx_if.connect_navpose_if
    ##################################
    # Setup NPX IF Class ####################
    # The robot's navpose is published by an NPX device at
    # <node>/<device_name>/npx/navpose, from getNavPoseCb, and selected as the
    # RBX navpose source so RBXRobotIF's current_* control state tracks it.
    self.msg_if.pub_debug("Starting NPX Device IF Initialization")
    self.npx_if = NPXDeviceIF(device_info = self.device_info_dict,
        data_source_description = 'simulator',
        getNavPoseCb = self.getNavPoseCb,
        max_navpose_update_rate = self.NAVPOSE_UPDATE_RATE,
        msg_if = self.msg_if,
        node_if = self.node_if,
        settings_if = self.settings_if,
        save_data_if = self.save_data_if,
        connect_navpose_if = self.connect_navpose_if
        )
    # Make the robot's own navpose the RBX navpose source.
    self.own_navpose_topic = self.npx_if.navpose_if.get_namespace()
    nepi_sdk.start_timer_process(1.0, self.selectOwnNavPoseCb, oneshot = True)

    ##################################
    # Setup Motor SVX IF Classes ####################
    self.createMotorSvxIfs()

    ##################################
    # Setup robot_color Image IF Class ####################
    # The relayed camera frames (whichever camera camera_view_mode selects)
    # are also published as a NEPI color image data product at
    # <rbx namespace>/robot_color, which RBXRobotIF's image-source connect
    # can discover and select. Constructed the way IDXDeviceIF builds its
    # ColorImageIF, sharing the RBX interface's IFs.
    self.msg_if.pub_debug("Starting robot_color Image IF Initialization")
    self.robot_color_if = ColorImageIF(name = 'robot_color',
                    namespace = self.rbx_if.get_namespace(),
                    data_source_description = 'simulator',
                    data_ref_description = 'robot camera',
                    msg_if = self.msg_if,
                    node_if = self.node_if,
                    save_data_if = self.save_data_if,
                    transform_if = self.rbx_if.transform_if,
                    navpose_if = self.rbx_if.navpose_if
                    )
    self.robot_color_if.wait_for_ready()
    time.sleep(1)

    ## Raise the interface's setpoint-wait cmd_timeout above its factory default
    ## -- see GOTO_CMD_TIMEOUT_SEC above for why a non-holonomic ground robot
    ## needs more headroom than the factory value provides.
    self.rbx_if.setCmdTimeoutCb(UInt32(data = self.GOTO_CMD_TIMEOUT_SEC))

    ## Point the interface's image-source search at this instance's own
    ## device_name-qualified topic, overriding the factory default and any stale
    ## persisted config on every startup, for deterministic per-instance behavior.
    ## 3.4: RBXRobotIF has no setImageTopicCb; its image source is the
    ## connect_img_if (ConnectColorImageIF) selection. set_selected_topic
    ## quietly rejects a topic the connect has not discovered yet, so the
    ## robot_color image IF's topic is selected by selectOwnImageCb once the
    ## connect has discovered it. The raw sensor_msgs/Image topic is not a
    ## NEPI image data product, so it is no longer selected.
    nepi_sdk.start_timer_process(1.0, self.selectOwnImageCb, oneshot = True)

    ## Start the closed-loop goto controller
    controller_interval = float(1) / self.CONTROLLER_RATE_HZ
    nepi_sdk.start_timer_process(controller_interval, self.gotoControlCb)

    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()


  #**********************
  # Two-camera reporting

  def get_sensor_topics(self):
    """Returns this driver's two named camera entries as typed topic pairs.

    Both cameras surface through the same generic sensor-topic mechanism, as the
    two named entries scene_camera and robot_camera. Which one is streaming is
    the camera_view_mode setting, not a separate mechanism.

    Returns:
        list: [(topic_name, msg_type), ...] with exactly two entries, both
            'sensor_msgs/Image'.
    """
    return list(self.sensor_topics)

  def get_camera_reference_frames(self):
    """Returns each camera's reference frame and default pose.

    Declarative for this pass: reported, not commandable. No wire shape for live
    camera pose adjustment is defined until the generic contract's camera control
    surface is settled.

    Returns:
        dict: Per-camera dicts with 'reference_frame', 'offset_m', and
            'tilt_deg'. robot_camera is coincident with the robot's main
            reference frame; scene_camera carries a default offset from it.
    """
    return {
      self.SCENE_CAMERA_NAME: {
        'reference_frame': self.ROBOT_MAIN_REFERENCE_FRAME,
        'offset_m': list(self.SCENE_CAMERA_DEFAULT_OFFSET_M),
        'tilt_deg': self.SCENE_CAMERA_DEFAULT_TILT_DEG,
      },
      self.ROBOT_CAMERA_NAME: {
        'reference_frame': self.ROBOT_MAIN_REFERENCE_FRAME,
        'offset_m': [0.0, 0.0, 0.0],
        'tilt_deg': 0.0,
      },
    }


  #**********************
  # Setting functions

  def initSettingsDict(self):
    init_settings_dict = dict()
    for setting_name in self.CAP_SETTINGS.keys():
      cap_setting = self.CAP_SETTINGS[setting_name]
      setting_type = cap_setting['type']
      setting_dict = dict()
      setting_dict['type'] = setting_type
      if 'options' in cap_setting.keys():
        try:
          if setting_type == 'Int':
            # An Int/Float control's min and max used to ride in the same
            # 'options' pair. The controls contract calls that 'bounds'.
            setting_dict['bounds'] = [int(cap_setting['options'][0]),int(cap_setting['options'][1])]
          elif setting_type == 'Float':
            setting_dict['bounds'] = [float(cap_setting['options'][0]),float(cap_setting['options'][1])]
          else:
            setting_dict['options'] = [str(option) for option in cap_setting['options']]
        except Exception as e:
          self.msg_if.pub_warn("Invalid options for setting: " + setting_name + " : " + str(e))

      default = None
      if setting_name in self.FACTORY_SETTINGS.keys():
        default = self.FACTORY_SETTINGS[setting_name]['value']
      if setting_name in self.FACTORY_SETTINGS_OVERRIDES.keys():
        default = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
      if default is None:
        continue
      try:
        if setting_type == 'Int':
          default = int(float(default))
        elif setting_type == 'Float':
          default = float(default)
        elif setting_type == 'Toggle':
          default = (str(default) == 'True' or str(default) == 'true')
        else:
          default = str(default)
      except Exception as e:
        self.msg_if.pub_warn("Invalid factory value for setting: " + setting_name + " : " + str(e))
        continue
      setting_dict['default'] = default
      init_settings_dict[setting_name] = setting_dict

    self.init_settings_dict = init_settings_dict
    settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
    settings_dict_values = nepi_controls.get_values_dict(settings_dict)
    self.msg_if.pub_info("Initialized Settings: " + str(settings_dict_values))
    return settings_dict

  def refreshSettingsDict(self):
    # These are software settings held by this node and pushed to the Gazebo
    # bridge; the simulator reports no capability set to read them back from.
    # Kept for contract consistency with the other drivers.
    return copy.deepcopy(self.settings_dict)

  def getSettingsFunction(self):
    return self.settings_dict

  def setSettingFunction(self, setting_name, setting_value):
    setting_str = setting_name + ":" + str(setting_value)
    if setting_name not in self.settings_dict.keys():
      msg = (self.node_name + " Setting name " + setting_str + " is not supported")
      return False, msg, self.settings_dict
    # check_valid_value() was removed from nepi_controls; get_clean_value()
    # returns None for an invalid value. Test 'is None', not falsiness -- a
    # Toggle cleans to a legitimate False.
    if nepi_controls.get_clean_value(self.settings_dict, setting_name, setting_value) is None:
      msg = (self.node_name + " Setting data " + setting_str + " is not valid")
      return False, msg, self.settings_dict

    self.settings_dict = nepi_controls.set_value(self.settings_dict, setting_name, setting_value)
    msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
    if setting_name in self.CAMERA_SETTING_NAMES:
      self.sendCameraSettings()
    if setting_name in self.ENVIRONMENT_SETTING_NAMES:
      self.setEnvironmentAction(nepi_controls.get_value(self.settings_dict, setting_name))
    return True, msg, self.settings_dict


  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    # No robot states (empty RBX_STATES). RBXRobotIF still calls this
    # unconditionally and displays "Not Set" for the empty list.
    return 0

  def setStateInd(self, state_ind):
    # Unreachable with empty RBX_STATES (RBXRobotIF bounds-checks first)
    return False

  def getModeInd(self):
    return 0

  def setModeInd(self, mode_ind):
    # Unreachable with empty RBX_MODES
    return False

  def checkStopFunction(self):
    triggered = self.stop_triggered
    self.stop_triggered = False # Reset Stop Trigger
    return triggered

  def manualControlsReady(self):
    # Gates manual motor-ratio commands on a live bridge connection. Fresh
    # telemetry is not required here (unlike goto), since a direct motor command
    # does not depend on knowing the current position or heading.
    with self.sock_lock:
      return self.sock is not None

  def setMotorControlRatio(self, motor_ind, speed_ratio):
    # Only updates local state. gotoControlCb, already running continuously at
    # CONTROLLER_RATE_HZ, is the single authoritative sender of velocity commands
    # to the bridge: a one-shot send from here would be immediately overwritten
    # by that loop's next (0,0)-when-idle tick, since it sends every tick
    # regardless of whether a goto or manual command is active. See
    # motorControlToVelocity.
    if motor_ind < 0 or motor_ind >= len(self.motor_ratios):
      self.msg_if.pub_warn("Motor control ignored: motor index " + str(motor_ind) + " out of range")
      return
    self.motor_ratios[motor_ind] = max(0.0, min(1.0, speed_ratio))

  def getMotorControlRatios(self):
    return self.motor_ratios

  ##########################
  # Motor Servo (SVX) Functions
  # Every motor is its own DeviceSVXIF (a spin servo) on the RUI Servo page.
  # Each callback acts on one motor through setMotorControlRatio above, so its
  # checks (index range, 0-1 clamp) still apply. A negative (reverse) command
  # clamps to 0 there: this bridge's wheels cannot reverse.

  def createMotorSvxIfs(self):
    # One DeviceSVXIF per motor, shaped like ptx_iqr's per-axis servos: each
    # shares the RBX interface's msg_if, node_if, settings_if, save_data_if and
    # connect_navpose_if, and its <device name>_motor_<n> device name gives it
    # its own <node>/<name>/svx namespace, status topic and node_if keys.
    # Built after RBXRobotIF because they share its IFs; RBXRobotIF holds the
    # same motor_svx_if_dict object, so it sees them once they are filled in.
    self.msg_if.pub_info("Launching NEPI SVX motor interfaces...")
    for motor_ind, motor_name in enumerate(list(self.motor_svx_if_dict.keys())):
      motor_device_info = copy.deepcopy(self.device_info_dict)
      motor_device_info["device_name"] = motor_name
      self.motor_svx_if_dict[motor_name] = DeviceSVXIF(device_info = motor_device_info,
                                        is_spin = True,
                                        # Built into the robot, so always connected,
                                        # with no Servo Connected toggle
                                        always_connected = True,
                                        stopMovingCb = lambda motor_ind = motor_ind: self.stopMotorSvx(motor_ind),
                                        moveCb = lambda direction, time_s = None, motor_ind = motor_ind: self.moveMotorSvx(motor_ind, direction, time_s),
                                        moveSpeedRatioCb = lambda direction, speed_ratio, time_s = None, motor_ind = motor_ind: self.moveMotorSvxSpeedRatio(motor_ind, direction, speed_ratio, time_s),
                                        # The Servo page drives a spin servo with set_direction +
                                        # set_speed_ratio, and its STOP button sends go_home
                                        set_direction = lambda direction, motor_ind = motor_ind: self.setMotorSvxDirection(motor_ind, direction),
                                        get_direction = lambda motor_ind = motor_ind: self.motor_svx_dirs[motor_ind],
                                        setSpeedRatioCb = lambda speed_ratio, motor_ind = motor_ind: self.setMotorSvxSpeedRatio(motor_ind, speed_ratio),
                                        getSpeedRatioCb = lambda motor_ind = motor_ind: self.motor_svx_speed_ratios[motor_ind],
                                        goHomeCb = lambda motor_ind = motor_ind: self.stopMotorSvx(motor_ind),
                                        msg_if = self.msg_if,
                                        node_if = self.node_if,
                                        settings_if = self.settings_if,
                                        save_data_if = self.save_data_if,
                                        connect_navpose_if = self.connect_navpose_if
                                        )
    # DeviceSVXIF's init restores its saved speed ratio through setSpeedRatioCb
    # during construction. Run commands are ignored until here, so starting the
    # node never spins a motor.
    self.motor_svx_cmds_enabled = True
    self.msg_if.pub_info(" ... SVX motor interfaces running: " + str(list(self.motor_svx_if_dict.keys())))

  def stopMotorSvx(self, motor_ind):
    self.motor_svx_stop_triggers[motor_ind] = True
    self.setMotorControlRatio(motor_ind, 0.0)

  def runMotorSvx(self, motor_ind, signed_ratio):
    # Same gate the RBX interface applied to set_motor_control before 3.4
    if self.motor_svx_cmds_enabled == False:
      return False
    if self.manualControlsReady() != True:
      self.msg_if.pub_warn("Motor " + str(motor_ind + 1) + " command ignored: manual controls not ready")
      return False
    self.motor_svx_stop_triggers[motor_ind] = False
    self.setMotorControlRatio(motor_ind, signed_ratio)
    return True

  def moveMotorSvx(self, motor_ind, direction, time_s = None):
    # moveCb (jog_timed): direction and optional time, at this motor's speed ratio
    self.moveMotorSvxSpeedRatio(motor_ind, direction, self.motor_svx_speed_ratios[motor_ind], time_s)

  def moveMotorSvxSpeedRatio(self, motor_ind, direction, speed_ratio, time_s = None):
    # moveSpeedRatioCb (jog_timed_speed_ratio): the signed ratio is the
    # direction's sign times the speed ratio. As in ptx_iqr's timed jog, a
    # positive time stops the motor once it elapses or a stop arrives; no time
    # (or 0) runs until stopped.
    sign = 1.0 if direction >= 0 else -1.0
    if self.runMotorSvx(motor_ind, sign * float(speed_ratio)) == False:
      return
    if time_s is not None and time_s > 0:
      start_time = nepi_utils.get_time()
      while (nepi_utils.get_time() - start_time) < time_s and self.motor_svx_stop_triggers[motor_ind] == False and not nepi_sdk.is_shutdown():
        nepi_sdk.sleep(0.1)
      if self.motor_svx_stop_triggers[motor_ind] == False:
        self.stopMotorSvx(motor_ind)

  def setMotorSvxDirection(self, motor_ind, direction):
    self.motor_svx_dirs[motor_ind] = 1 if direction >= 0 else -1
    # Re-command a running motor so a direction change at an unchanged speed
    # ratio takes effect (DeviceSVXIF skips an unchanged set_speed_ratio)
    ratios = self.getMotorControlRatios()
    if motor_ind < len(ratios) and ratios[motor_ind] != 0:
      self.runMotorSvx(motor_ind, self.motor_svx_dirs[motor_ind] * self.motor_svx_speed_ratios[motor_ind])

  def setMotorSvxSpeedRatio(self, motor_ind, speed_ratio):
    # For a spin servo the speed ratio is the run command: the Servo page's
    # speed/direction slider sends set_direction, then set_speed_ratio.
    self.motor_svx_speed_ratios[motor_ind] = float(speed_ratio)
    self.runMotorSvx(motor_ind, self.motor_svx_dirs[motor_ind] * self.motor_svx_speed_ratios[motor_ind])

  def autonomousControlsReady(self):
    # Gates all goto commands: a live bridge connection AND fresh telemetry, so
    # goto targets are computed from a real current position.
    with self.sock_lock:
      connected = self.sock is not None
    fresh = (nepi_utils.get_time() - self.last_telemetry_time) < self.TELEMETRY_FRESH_SEC
    return connected and fresh

  def goStop(self):
    self.stop_triggered = True
    self.clearGotoTarget()
    self.sendVelocityCmd(0.0, 0.0)
    return True

  def gotoPose(self, attitude_enu_degs):
    # RBXRobotIF passes target attitude ENU [roll, pitch, yaw] in degrees. Only
    # yaw is achievable on a ground robot: roll and pitch stay near 0 on flat
    # ground, so the interface's roll/pitch error checks converge. Hold position,
    # turn in place.
    self.msg_if.pub_info("Received Pose setpoint command: " + str(attitude_enu_degs))
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'],
                          'y_m': self.navpose_dict['y_m'],
                          'yaw_deg': attitude_enu_degs[2]}

  def gotoPosition(self, point_enu_m, orientation_enu_deg):
    # RBXRobotIF passes the goal as an ENU offset point from the current position
    # plus an absolute target orientation (its own convergence check uses current
    # + offset), so the controller target is computed the same way from the same
    # navpose source. z is ignored: the robot moves in the ground plane.
    self.msg_if.pub_info("Received Position setpoint command: " + str(point_enu_m))
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'] + point_enu_m.x,
                          'y_m': self.navpose_dict['y_m'] + point_enu_m.y,
                          'yaw_deg': orientation_enu_deg[2]}

  ### Callback returning the full navpose dict to the NPX interface
  def getNavPoseCb(self):
    # Returns None until telemetry has arrived: NPXDeviceIF derives its has_*
    # capabilities once, from the first dict it gets, so a blank dict returned
    # before telemetry would fix every capability at False for the life of
    # the node.
    if self.navpose_dict['has_orientation'] == False or self.navpose_dict['has_position'] == False:
      return None
    return copy.deepcopy(self.navpose_dict)

  def selectOwnNavPoseCb(self, timer):
    # Select the robot's own NPX navpose as the RBX navpose source once the
    # connect has discovered it. Only a startup default is replaced ('None',
    # '', or the node namespace that older saved configs hold), never a
    # source the operator picked, and it gives up after a bounded number of
    # tries so a navpose that never appears costs nothing.
    connect_if = self.connect_navpose_if
    if connect_if is None or self.own_navpose_topic is None or nepi_sdk.is_shutdown():
      return
    selected = connect_if.get_selected_topic()
    if selected == self.own_navpose_topic:
      return
    if selected not in ['None', '', None, self.node_namespace]:
      self.msg_if.pub_info("Keeping operator navpose source: " + str(selected))
      return
    if self.own_navpose_topic in connect_if.get_available_topics():
      connect_if.set_selected_topic(self.own_navpose_topic)
      self.msg_if.pub_info("Selected own navpose source: " + self.own_navpose_topic)
      return
    self.own_navpose_tries += 1
    if self.own_navpose_tries < self.OWN_NAVPOSE_MAX_TRIES:
      nepi_sdk.start_timer_process(1.0, self.selectOwnNavPoseCb, oneshot = True)
    else:
      self.msg_if.pub_warn("Own navpose source never appeared, leaving navpose unselected: " + self.own_navpose_topic)

  def selectOwnImageCb(self, timer):
    # Select the robot_color image IF as the RBX image source once the
    # connect has discovered it. Like the raw-topic selection this replaces,
    # it overrides any persisted selection every startup; the operator can
    # pick another source afterwards. Gives up after a bounded number of tries.
    if self.robot_color_if is None or self.rbx_if is None or nepi_sdk.is_shutdown():
      return
    connect_if = self.rbx_if.connect_img_if
    own_image_topic = self.robot_color_if.get_namespace()
    if connect_if.get_selected_topic() == own_image_topic:
      return
    if own_image_topic in connect_if.get_available_topics():
      connect_if.set_selected_topic(own_image_topic)
      self.msg_if.pub_info("Selected own image source: " + own_image_topic)
      return
    self.own_image_tries += 1
    if self.own_image_tries < self.OWN_IMAGE_MAX_TRIES:
      nepi_sdk.start_timer_process(1.0, self.selectOwnImageCb, oneshot = True)
    else:
      self.msg_if.pub_warn("Own image source never appeared, leaving image source unchanged: " + own_image_topic)

  #######################
  ### Setup-Action Functions

  def setSetupActionInd(self, action_ind):
    # action_ind is already bounds-checked against RBX_SETUP_ACTIONS by
    # RBXRobotIF before this is called.
    action = self.RBX_SETUP_ACTIONS[action_ind]
    if action == "RESET_SIM":
      return self.resetSimAction()
    elif action == "RETURN_HOME":
      return self.returnHomeAction()
    return False

  #######################
  ### Go-Action Functions

  def setGoActionInd(self, action_ind):
    # Unreachable with empty RBX_GO_ACTIONS (RBXRobotIF bounds-checks first)
    return False

  #######################
  ### Home Functions

  def getHome(self):
    # No GPS or WGS84 reference on a ground robot. RBXRobotIF's home mechanism
    # only carries a plain three-float GeoPoint, so this reuses that same
    # plumbing (set_home / get_home / set_home_current, and RBXRobotIF's own
    # home_location param persistence) with the three floats reinterpreted as
    # local ENU x/y/z meters. See processTelemetryLine's matching latitude /
    # longitude / altitude_m reinterpretation, which is what makes "use current
    # position as home" capture the real position instead of always reading back
    # (0, 0, 0), with zero shared-API changes.
    home = GeoPoint()
    home.latitude = self.home_x_m
    home.longitude = self.home_y_m
    home.altitude = self.home_z_m
    return home

  def setHome(self, geo_point):
    # With no location fix the IF passes -999 latitude/longitude (NavPose
    # gates them on has_location), so each -999 element is taken from this
    # robot's own current x/y/z instead. A real fix is stored as given.
    if geo_point.latitude == -999 or geo_point.longitude == -999:
      own_xyz = [self.navpose_dict['x_m'], self.navpose_dict['y_m'], self.navpose_dict['z_m']]
      home_xyz = [geo_point.latitude, geo_point.longitude, geo_point.altitude]
      for i in range(3):
        if home_xyz[i] == -999:
          home_xyz[i] = own_xyz[i]
      [self.home_x_m, self.home_y_m, self.home_z_m] = home_xyz
      return True
    self.home_x_m = geo_point.latitude
    self.home_y_m = geo_point.longitude
    self.home_z_m = geo_point.altitude
    return True

  def returnHomeAction(self):
    # Drives to the stored home position the same way a gotoPosition setpoint
    # does: set the goto_target and let gotoControlCb's closed-loop controller
    # take it from there. No forced final yaw -- home is a position here, not a
    # position plus heading. Blocks until arrival or timeout, since both the
    # setup-action dispatch and RBXRobotIF's own goHomeFunction call treat this
    # call's return value as success or failure, not just as "accepted".
    if not self.autonomousControlsReady():
      return False
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.home_x_m, 'y_m': self.home_y_m, 'yaw_deg': None}
    start_time = nepi_utils.get_time()
    while (nepi_utils.get_time() - start_time) < self.GO_HOME_TIMEOUT_SEC:
      with self.goto_target_lock:
        reached = self.goto_target is None
      if reached:
        return True
      time.sleep(self.GO_HOME_POLL_INTERVAL_SEC)
    return False

  def resetSimAction(self):
    # Unlike RETURN_HOME (drive there under the closed-loop controller), this is
    # an instant physics teleport back to the spawn pose. The Gazebo side owns
    # the actual model-state call; this side just needs a live connection.
    self.clearGotoTarget()
    self.sendVelocityCmd(0.0, 0.0)
    with self.sock_lock:
      connected = self.sock is not None
    if not connected:
      return False
    self.sendLineToBridge({'type': 'reset'}, "Reset sim")
    return True

  def setEnvironmentAction(self, environment_value):
    # Fire-and-forget over the bridge, same pattern as resetSimAction: the Gazebo
    # side owns the actual model spawn and delete calls and its own
    # already-spawned bookkeeping, so this side only needs a live connection.
    # Sent as a named environment option rather than as a dedicated per-option
    # message type, so adding an environment option needs no change here.
    with self.sock_lock:
      connected = self.sock is not None
    if not connected:
      return False
    enabled = (environment_value == self.OBSTACLE_COURSE_OPTION)
    self.sendLineToBridge({'type': 'environment_option',
                           'option': self.OBSTACLE_COURSE_OPTION,
                           'enabled': enabled},
                          "Environment " + str(environment_value))
    return True

  #######################
  ### Goto Controller Processes

  def clearGotoTarget(self):
    with self.goto_target_lock:
      self.goto_target = None

  def gotoControlCb(self, timer):
    # Closed-loop differential-drive controller: turn toward the target bearing,
    # drive when roughly aligned, then rotate to the final yaw goal.
    #
    # A velocity command is sent on EVERY tick, active goto or not, defaulting to
    # (0, 0) when idle. Never rely on a single one-shot zero command to stop the
    # robot: the bridge is a plain TCP link with no delivery acknowledgement, and
    # if a stop packet -- or the last drive-phase command right at convergence --
    # is ever dropped, Gazebo's drive plugin latches whatever velocity it last
    # received and the robot keeps drifting indefinitely, since nothing would
    # re-send the correction. Re-asserting every tick makes that self-healing.
    with self.goto_target_lock:
      target = self.goto_target

    lin = 0.0
    ang = 0.0
    if target is not None:
      cur_x = self.navpose_dict['x_m']
      cur_y = self.navpose_dict['y_m']
      cur_yaw_rad = math.radians(self.navpose_dict['yaw_deg'])

      max_lin = float(nepi_controls.get_value(self.settings_dict,'max_linear_speed_mps'))
      max_ang = math.radians(float(nepi_controls.get_value(self.settings_dict,'max_angular_rate_dps')))
      tol_m = self.FACTORY_GOTO_TOL_M
      tol_rad = self.FACTORY_GOTO_TOL_RAD
      if self.rbx_if is not None:
        tol_m = self.rbx_if.status_msg.error_bounds.max_distance_error_m * self.GOTO_TOL_FRACTION
        tol_rad = (math.radians(self.rbx_if.status_msg.error_bounds.max_rotation_error_deg)
                   * self.GOTO_TOL_FRACTION)

      dx = target['x_m'] - cur_x
      dy = target['y_m'] - cur_y
      dist = math.hypot(dx, dy)

      if dist > tol_m:
        # Drive phase: point at the target, drive when roughly aligned
        bearing_err = self.normalizeAngle(math.atan2(dy, dx) - cur_yaw_rad)
        ang = max(-max_ang, min(max_ang, self.GOTO_KP_ANG * bearing_err))
        if abs(bearing_err) < self.GOTO_TURN_GATE_RAD:
          lin = max(0.0, min(max_lin, self.GOTO_KP_LIN * dist))
      else:
        # Final yaw phase (skipped if no yaw goal)
        yaw_err = 0.0
        if target['yaw_deg'] is not None:
          yaw_err = self.normalizeAngle(math.radians(target['yaw_deg']) - cur_yaw_rad)
        if abs(yaw_err) > tol_rad:
          ang = max(-max_ang, min(max_ang, self.GOTO_KP_ANG * yaw_err))
        else:
          # Target reached: clear it (lin/ang stay 0.0, so the robot stops)
          self.clearGotoTarget()
          self.msg_if.pub_info("Goto target reached")
    elif any(self.motor_ratios):
      # No active goto -- an active manual motor command takes over this same
      # tick, so there is exactly one authoritative sender rather than a race
      # between this loop and a separate one-shot command.
      lin, ang = self.motorControlToVelocity()
    self.sendVelocityCmd(lin, ang)

  def motorControlToVelocity(self):
    left = self.motor_ratios[0]
    right = self.motor_ratios[1]
    lin = (left + right) / 2.0 * self.MOTOR_MAX_LINEAR_MPS
    ang = (right - left) / self.MOTOR_WHEEL_BASE_M * self.MOTOR_MAX_LINEAR_MPS
    return lin, ang

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  #######################
  ### Bridge Processes

  def bridgeLoop(self):
    # Persistent client to the Gazebo-side bridge server. The Gazebo stack can
    # restart independently of this node -- any failure tears the socket down and
    # retries the connect on a fixed interval.
    while not nepi_sdk.is_shutdown():
      sock = None
      try:
        sock = socket.create_connection((self.sim_host, int(self.bridge_port)),
                                        timeout = self.SOCKET_TIMEOUT_SEC)
        sock.settimeout(self.SOCKET_TIMEOUT_SEC)
      except Exception as e:
        self.msg_if.pub_warn("Bridge connect to " + str(self.sim_host) + ":" +
                             str(self.bridge_port) + " failed: " + str(e), throttle_s = 10.0)
        time.sleep(self.RECONNECT_INTERVAL_SEC)
        continue
      with self.sock_lock:
        self.sock = sock
      self.msg_if.pub_info("Connected to sim bridge at " + str(self.sim_host) +
                           ":" + str(self.bridge_port))
      # Push this node's actual current camera and environment settings on every
      # reconnect. A bare restart of this node resets settings_dict to factory,
      # while the Gazebo side keeps whatever it last had, so an explicit push
      # avoids relying on both sides coincidentally matching factory defaults.
      self.sendCameraSettings()
      self.setEnvironmentAction(nepi_controls.get_value(self.settings_dict,'environment'))
      buf = b''
      while not nepi_sdk.is_shutdown():
        try:
          data = sock.recv(4096)
        except socket.timeout:
          # The server pushes telemetry continuously, so a quiet-but-open socket
          # past the timeout means the far side is gone (e.g. a half-open
          # forwarded connection).
          data = b''
        except Exception:
          data = b''
        if not data:
          break
        buf += data
        while b'\n' in buf:
          line, buf = buf.split(b'\n', 1)
          if line.strip():
            self.processBridgeLine(line)
      with self.sock_lock:
        self.sock = None
      try:
        sock.close()
      except Exception:
        pass
      self.msg_if.pub_warn("Sim bridge connection lost -- retrying in " +
                           str(self.RECONNECT_INTERVAL_SEC) + "s")
      time.sleep(self.RECONNECT_INTERVAL_SEC)

  def processBridgeLine(self, line):
    # Single entry point for every line off the bridge socket: parse once, then
    # dispatch by key presence. Image frames carry "type":"image"; a line with no
    # type key is telemetry.
    try:
      msg = json.loads(line)
    except Exception as e:
      self.msg_if.pub_warn("Bad line from bridge: " + str(e), throttle_s = 5.0)
      return
    if not isinstance(msg, dict):
      return
    if msg.get('type') == 'image':
      self.processImageLine(msg)
    else:
      self.processTelemetryLine(msg)

  def processImageLine(self, msg):
    # Decode the relayed JPEG and republish it as a raw Image on this instance's
    # own image topic. The Gazebo side has already selected which of the two
    # cameras this frame came from, per the current camera_view_mode.
    try:
      jpeg_bytes = base64.b64decode(msg['data'])
      arr = np.frombuffer(jpeg_bytes, dtype = np.uint8)
      cv2_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
      if cv2_img is None:
        raise ValueError("cv2.imdecode returned None")
      ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding = "bgr8")
      # The raw sensor_msgs/Image topic stays: the sim connector's common view
      # mirror subscribes to it by path.
      self.image_pub.publish(ros_img)
      # Also published as a NEPI color image data product, the RBX image
      # source (see robot_color_if in __init__)
      if self.robot_color_if is not None:
        self.robot_color_if.publish_cv2_img(cv2_img, encoding = "bgr8")
    except Exception as e:
      self.msg_if.pub_warn("Failed to process camera image frame: " + str(e), throttle_s = 5.0)

  def processTelemetryLine(self, telem):
    # Bridge telemetry -> the navpose dict consumed by getNavPoseCb, published as
    # the standard NEPI navpose by RBXRobotIF's NPXDeviceIF and bridged to the
    # current_* attributes its goto convergence checks use.
    now = nepi_utils.get_time()
    x_m = float(telem.get('x', 0.0))
    y_m = float(telem.get('y', 0.0))
    yaw_rad = float(telem.get('yaw', 0.0))
    lin_mps = float(telem.get('linear_x', 0.0))
    ang_radps = float(telem.get('angular_z', 0.0))

    # Position: the sim world frame is ENU, and this is a ground robot, so z is 0
    self.navpose_dict['has_position'] = True
    self.navpose_dict['time_position'] = now
    self.navpose_dict['x_m'] = x_m
    self.navpose_dict['y_m'] = y_m
    self.navpose_dict['z_m'] = 0.0
    # No WGS84 fix on this robot, so has_location stays False, and NavPose gates
    # the latitude / longitude mirrored below on has_location: "use current
    # position as home" reaches setHome as -999, which takes this robot's own
    # x/y/z instead (see setHome above).
    self.navpose_dict['latitude'] = x_m
    self.navpose_dict['longitude'] = y_m
    self.navpose_dict['altitude_m'] = 0.0
    # Body-forward speed decomposed into the nav frame
    self.navpose_dict['x_m_per_sec'] = lin_mps * math.cos(yaw_rad)
    self.navpose_dict['y_m_per_sec'] = lin_mps * math.sin(yaw_rad)
    self.navpose_dict['z_m_per_sec'] = 0.0
    # A real velocity from the sim, so say so: NavPose.has_velocity defaults
    # False and gates x/y/z_m_per_sec on its own.
    self.navpose_dict['has_velocity'] = True
    self.navpose_dict['time_velocity'] = now

    # Orientation: flat-ground robot, only yaw is meaningful
    self.navpose_dict['has_orientation'] = True
    self.navpose_dict['time_orientation'] = now
    self.navpose_dict['roll_deg'] = 0.0
    self.navpose_dict['pitch_deg'] = 0.0
    self.navpose_dict['yaw_deg'] = math.degrees(yaw_rad)
    self.navpose_dict['yaw_deg_per_sec'] = math.degrees(ang_radps)

    self.last_telemetry_time = now

  def sendVelocityCmd(self, linear_x, angular_z):
    cmd = {'linear_x': linear_x, 'angular_z': angular_z}
    self.sendLineToBridge(cmd, "Velocity command")

  def sendCameraSettings(self):
    # Which of the two cameras is relayed. The Gazebo side reads view_mode and
    # relays the matching camera; this is the only camera control this driver
    # exposes for now (see the two-camera note at the top of this file).
    cmd = {
      'type': 'camera_settings',
      'view_mode': nepi_controls.get_value(self.settings_dict,'camera_view_mode'),
    }
    self.sendLineToBridge(cmd, "Camera settings")

  def sendLineToBridge(self, line_dict, description):
    with self.sock_lock:
      sock = self.sock
      if sock is None:
        self.msg_if.pub_warn(description + " dropped -- sim bridge not connected",
                             throttle_s = 5.0)
        return
      # The send happens under the same lock as the socket read: this is a single
      # TCP stream written from the control-loop timer thread and from setting
      # callbacks, and two unsynchronized sendall calls can interleave their
      # bytes and corrupt the newline-delimited JSON the far side parses.
      try:
        sock.sendall((json.dumps(line_dict) + '\n').encode())
      except Exception as e:
        # bridgeLoop's recv will fail on the same dead socket and reconnect
        self.msg_if.pub_warn("Failed to send " + description.lower() + " to bridge: " + str(e))

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    """Stops the robot on node shutdown by sending a zero velocity command."""
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    self.sendVelocityCmd(0.0, 0.0)


#########################################
# Main
#########################################
if __name__ == '__main__':
  GazeboNode()
