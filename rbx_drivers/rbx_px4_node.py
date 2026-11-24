#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
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

import time
import math
import copy
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls

from std_msgs.msg import Float64, Bool, Empty
from geometry_msgs.msg import PoseStamped, Point
from geographic_msgs.msg import GeoPoint
from mavros_msgs.msg import State, ExtendedState, PositionTarget, StatusText, GPSINPUT, HilGPS
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandHome, CommandHomeRequest, CommandLong, CommandLongRequest, ParamGet, ParamGetRequest

from sensor_msgs.msg import NavSatFix, BatteryState

from nepi_api.device_if_rbx import RBXRobotIF
from nepi_api.device_if_npx import NPXDeviceIF
from nepi_api.device_if_svx import DeviceSVXIF
from nepi_api.messages_if import MsgIF

# The Nav Sim app's GPS instance status names the mavros node each instance
# injects GPS_INPUT into, which is how this driver finds its own fake GPS.
# Optional: a deployment without the app only loses the fake GPS commands.
try:
  from nepi_app_nav_sim.msg import NepiAppGpsSimStatus
except ImportError:
  NepiAppGpsSimStatus = None

PKG_NAME = 'RBX_PX4' # Use in display menus
FILE_TYPE = 'NODE'

# Nav Sim GPS instance command topics, relative to the instance namespace
# (<base>/app_nav_sim/gps_instances/<name>), keyed by this driver's command
# names. They are nav_sim's programmatic API for the GPS kind, the same set
# rbx_ardupilot_node.py uses.
FAKE_GPS_STATUS_MSG = 'NepiAppGpsSimStatus'
FAKE_GPS_CMD_TOPICS = {
  'enable':        ('set_gps_enabled', Bool),
  'reset':         ('reset_location',  GeoPoint),
  'go_stop':       ('go_stop',         Empty),
  'goto_position': ('goto_position',   Point),
  'goto_location': ('goto_location',   GeoPoint),
}
FAKE_GPS_TARGET_UPDATE_SEC = 2.0

# PX4 has no GPS_INPUT handler in any release (checked 1.14.3, 1.16.0, 1.17.0
# and main, mavlink_receiver.cpp). Its only MAVLink GPS input is HIL_GPS, taken
# only while MAV_USEHILGPS = 1 and from a sender whose system ID equals PX4's
# own MAV_SYS_ID. Nav Sim injects GPS_INPUT, which is what ArduPilot reads
# with GPS_TYPE 14, so this driver resends it as HIL_GPS (fakeGpsInputCb),
# capped at a real receiver's rate.
FAKE_GPS_HIL_MAX_RATE_HZ = 10.0
GPS_INPUT_IGNORE_VEL_HORIZ = 8
GPS_INPUT_IGNORE_VEL_VERT = 16
GPS_INPUT_IGNORE_HORIZ_ACCURACY = 64
GPS_INPUT_IGNORE_VERT_ACCURACY = 128
# mavros's hil plugin (1.20.1, hil.cpp gps_cb) multiplies HilGPS eph, epv,
# vel, vn, ve, vd and cog by 100 on the way to HIL_GPS, so the integer HilGPS
# fields carry whole metres, m/s and degrees. These caps keep the x100 values
# inside HIL_GPS's uint16/int16 fields; an accuracy at the cap (655 m) fails
# every EKF2 check, which is what an ignored GPS_INPUT accuracy should do.
HIL_GPS_MAX_ACCURACY_M = 655
HIL_GPS_MAX_SPEED_MPS = 327


#########################################
# Frame helpers
#########################################
# Module level, with no ROS dependency, so the frame and geodetic arithmetic
# can be checked offline.

def rotateBodyToEnu(body_x, body_y, yaw_enu_rad):
  # Body x forward, y LEFT (ROS REP-103), rotated by the ENU yaw (0 = facing
  # east, positive counter-clockwise) into ENU east/north.
  enu_x = body_x * math.cos(yaw_enu_rad) - body_y * math.sin(yaw_enu_rad)
  enu_y = body_x * math.sin(yaw_enu_rad) + body_y * math.cos(yaw_enu_rad)
  return [enu_x, enu_y]

def limitHorizontal(x, y, max_speed):
  # Scales the x/y vector down to max_speed, keeping its direction. Clipping
  # each axis on its own would let a diagonal reach sqrt(2) times the cap and
  # bend the direction of travel.
  speed = math.hypot(x, y)
  if speed > max_speed and speed > 0.0:
    scale = max_speed / speed
    return [x * scale, y * scale]
  return [x, y]

def clampValue(value, min_value, max_value):
  return max(min_value, min(max_value, value))

def getGeoOffsetEnuM(lat_deg, lon_deg, alt_m, target_lat_deg, target_lon_deg, target_alt_m):
  # Local [east, north, up] metres from one geopoint to another, using the
  # WGS84 metres-per-degree series at the start latitude: a flat-earth step,
  # meant for the short hops an outer loop commands. It does not handle
  # crossing the +-180 deg meridian. Both altitudes must be in the same frame.
  lat_rad = math.radians(lat_deg)
  m_per_deg_lat = (111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
                   - 0.0023 * math.cos(6 * lat_rad))
  m_per_deg_lon = (111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3 * lat_rad)
                   + 0.118 * math.cos(5 * lat_rad))
  north_m = (target_lat_deg - lat_deg) * m_per_deg_lat
  east_m = (target_lon_deg - lon_deg) * m_per_deg_lon
  up_m = target_alt_m - alt_m
  return [east_m, north_m, up_m]


#########################################
# PX4 RBX Node
#########################################
# PX4 runs the inner loops; this node is the outer loop. Everything it
# commands goes through ONE offboard setpoint stream (setpoint_raw/local,
# see sendSetpointStreamCb), which runs for the life of the node: PX4 only
# enters OFFBOARD while setpoints are already streaming, and leaves it (and
# fails safe) as soon as they stop. Goto, teleop, stop and takeoff calls only
# change the target that stream carries.

class PX4Node:
  DEFAULT_NODE_NAME = "px4" # connection port added once discovered

  # RBX State and Mode Dictionaries
  RBX_STATES = ["DISARM","ARM"]
  # PX4's own mode names exactly as mavros reports them in State.mode, so
  # get_state_callback can match them directly. The manual modes are listed so
  # an RC safety pilot taking over shows as what it is; PX4 refuses them
  # itself when there is no RC.
  RBX_MODES = ["MANUAL","STABILIZED","ALTCTL","POSCTL","OFFBOARD",
               "AUTO.TAKEOFF","AUTO.LOITER","AUTO.RTL","AUTO.LAND","RESUME"]
  # TAKEOFF and LAUNCH climb with an offboard position setpoint, not
  # MAV_CMD_NAV_TAKEOFF: PX4 reads that command's altitude as AMSL, so the
  # relative height the ArduPilot driver sends would mean metres above sea
  # level. FORCE_DISARM is MAV_CMD_COMPONENT_ARM_DISARM with the force magic
  # value, which disarms even in flight.
  RBX_SETUP_ACTIONS = ["TAKEOFF","LAUNCH","FORCE_DISARM"]
  RBX_GO_ACTIONS = []

  RBX_STATE_FUNCTIONS = ["disarm","arm"]
  RBX_MODE_FUNCTIONS = ["manual","stabilize","altctl","posctl","offboard",
                        "auto_takeoff","loiter","rtl","land","resume"]
  RBX_SETUP_ACTION_FUNCTIONS = ["takeoff","launch","force_disarm"]
  RBX_GO_ACTION_FUNCTIONS = []

  # The speed caps apply to teleop and goto_velocity. PX4's own limits
  # (MPC_XY_VEL_MAX and friends) still apply on top of them, so raising these
  # past the airframe's parameters has no effect.
  CAP_SETTINGS = dict(
    takeoff_height_m = {"type":"Float","name":"takeoff_height_m","options":["0.5","100.0"]},
    max_horizontal_speed_mps = {"type":"Float","name":"max_horizontal_speed_mps","options":["0.1","50.0"]},
    max_vertical_speed_mps = {"type":"Float","name":"max_vertical_speed_mps","options":["0.1","10.0"]},
    max_yaw_rate_dps = {"type":"Float","name":"max_yaw_rate_dps","options":["1.0","360.0"]},
    motor_count = {"type":"Int","name":"motor_count","options":["1","12"]},
    motor_test_max_throttle_percent = {"type":"Float","name":"motor_test_max_throttle_percent","options":["0.0","100.0"]},
    motor_test_timeout_s = {"type":"Float","name":"motor_test_timeout_s","options":["1.0","60.0"]},
    # The RUI shows the Teleop control type only when teleop_movement_enabled
    # exists and is not "FALSE", and hides Autonomous when
    # autonomous_movement_enabled is "FALSE".
    autonomous_movement_enabled = {"type":"Selection","name":"autonomous_movement_enabled","options":["TRUE","FALSE"]},
    teleop_movement_enabled = {"type":"Selection","name":"teleop_movement_enabled","options":["TRUE","FALSE"]}
  )

  FACTORY_SETTINGS = dict(
    takeoff_height_m = {"type":"Float","name":"takeoff_height_m","value":"3.0"},
    max_horizontal_speed_mps = {"type":"Float","name":"max_horizontal_speed_mps","value":"2.0"},
    max_vertical_speed_mps = {"type":"Float","name":"max_vertical_speed_mps","value":"1.0"},
    max_yaw_rate_dps = {"type":"Float","name":"max_yaw_rate_dps","value":"30.0"},
    motor_count = {"type":"Int","name":"motor_count","value":"4"},
    motor_test_max_throttle_percent = {"type":"Float","name":"motor_test_max_throttle_percent","value":"15.0"},
    motor_test_timeout_s = {"type":"Float","name":"motor_test_timeout_s","value":"5.0"},
    autonomous_movement_enabled = {"type":"Selection","name":"autonomous_movement_enabled","value":"TRUE"},
    teleop_movement_enabled = {"type":"Selection","name":"teleop_movement_enabled","value":"TRUE"}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  POSITION_UPDATE_RATE = 10

  # Offboard setpoint stream. PX4 will not switch to OFFBOARD unless
  # setpoints are already arriving faster than 2 Hz, so LAUNCH waits for
  # about a second of stream first.
  SETPOINT_PUBLISH_RATE_HZ = 30
  SETPOINT_STREAM_MIN_SEC = 1.0
  SETPOINT_STREAM_WAIT_SEC = 3.0
  SETPOINT_HOLD = "HOLD"
  SETPOINT_VELOCITY = "VELOCITY"
  # HOLD: position ENU plus yaw. VELOCITY: velocity ENU plus yaw rate.
  # mavros converts both from ENU/FLU to the NED that PX4 receives.
  HOLD_TYPE_MASK = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                    PositionTarget.IGNORE_YAW_RATE)
  VELOCITY_TYPE_MASK = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
                        PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                        PositionTarget.IGNORE_YAW)

  LOCAL_POSE_STALE_SEC = 1.0
  EXTENDED_STATE_STALE_SEC = 2.0

  # A teleop command is held this long after it arrives; the RUI resends a
  # held key every 200 ms. When commands stop, the stream turns the expired
  # velocity target into a position hold.
  TELEOP_CMD_TIMEOUT_SEC = 0.75
  GOTO_VELOCITY_CHECK_INTERVAL_SEC = 0.05

  # MAV_CMD_ACTUATOR_TEST: PX4's commander handles this and not
  # MAV_CMD_DO_MOTOR_TEST. param1 is the output (motor: 0 = minimum thrust,
  # expected to spin; 1 = full; NaN = stop), param2 a timeout PX4 clamps to
  # 3 s (0 or less releases control at once), param5 the output function
  # (ACTUATOR_OUTPUT_FUNCTION_MOTORn = n). PX4 answers DENIED while armed or
  # with the safety switch engaged. Because of the 3 s cap, a motor left
  # running from the Servo page is resent every MOTOR_TEST_REFRESH_SEC.
  MAV_CMD_ACTUATOR_TEST = 310
  ACTUATOR_TEST_TIMEOUT_SEC = 3.0
  MOTOR_TEST_REFRESH_SEC = 1.0
  # MAV_CMD_COMPONENT_ARM_DISARM with the param2 force magic value bypasses
  # the in-flight disarm interlock, on PX4 as on ArduPilot.
  MAV_CMD_COMPONENT_ARM_DISARM = 400
  MAV_CMD_FORCE_MAGIC = 21196.0

  # Link watchdog (SITL and SERIAL): a simulator that goes away, or a
  # Pixhawk that powers off on a UART that stays present, leaves mavros
  # running with connected False. The node shuts itself down so discovery
  # purges the device and probes for it again. A USB link needs no watchdog:
  # discovery purges it when the /dev path disappears.
  LINK_LOST_SEC = 15.0
  LINK_WATCHDOG_INTERVAL_SEC = 1.0

  device_info_dict = dict(device_name = "",
                          path = "",
                          serial_number = "",
                          hw_version = "",
                          sw_version = "")

  # NavPose data provided as a single nepi_nav.BLANK_NAVPOSE_DICT, returned by getNavPoseCb
  navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)


  init_settings_dict = dict()
  settings_dict = dict()

  caps_dict = dict()
  caps_dict['has_x_control'] = True
  caps_dict['has_y_control'] = True
  caps_dict['has_z_control'] = True

  caps_dict['has_roll_control'] = True
  caps_dict['has_pitch_control'] = True
  caps_dict['has_yaw_control'] = True

  OWN_NAVPOSE_RETRY_SEC = 2.0
  own_navpose_topic = None
  own_navpose_wait_logged = False

  # Every motor is its own DeviceSVXIF, named <device name>_motor_<n> (n from
  # 1), keyed by that name in motor_svx_if_dict, which RBXRobotIF gets as its
  # servo_if_dict. See createMotorSvxIfs.
  motor_svx_if_dict = dict()
  motor_svx_dirs = []
  motor_svx_speed_ratios = []
  motor_svx_stop_triggers = []
  motor_svx_cmds_enabled = False

  state_ind = 0
  state_current = "None"
  state_last = "None"

  mode_ind = 0
  mode_current = "None"
  mode_last = "None"

  battery_percent = 0

  mavlink_state = None
  armed_last = None

  rbx_if = None

  takeoff_complete = False
  # Local ENU z goal of the last takeoff, so isAirborne() can still recognise
  # a climb that finished after takeoff_action()'s own wait gave up. Cleared
  # with takeoff_complete on every disarm.
  takeoff_goal_alt = None
  takeoff_error_bound_m = None

  home_loc = GeoPoint()
  home_loc.latitude = -999
  home_loc.longitude = -999
  home_loc.altitude = -999
  home_location = home_loc

  stop_triggered = False

  # Set from the fake_gps discovery option (see reconcileFakeGpsApp and
  # fakeGpsInputCb)
  has_fake_gps = False

  # GPS_INPUT to HIL_GPS relay (see fakeGpsInputCb)
  fake_gps_hil_pub = None
  fake_gps_hil_last_time = 0.0
  fake_gps_relay_started = False
  fake_gps_relay_off_logged = False
  # MAV_USEHILGPS / MAV_SYS_ID check (see checkHilGpsParamsCb). mavros pulls
  # the parameter list after connecting, which takes several seconds on USB.
  HIL_GPS_PARAM_RETRY_SEC = 5.0
  HIL_GPS_PARAM_MAX_TRIES = 24
  hil_gps_param_tries = 0
  param_get_client = None

  # Wall-clock receipt times, for staleness checks
  local_pose_time = None
  extended_state = None
  extended_state_time = None

  setpoint_stream_start_time = None
  link_lost_time = None
  link_lost_signalled = False
  gotopose_rp_logged = False

  # Most recent flight-controller STATUSTEXT (e.g. pre-arm/arm rejection reasons).
  # Recorded from the mavros statustext/recv topic and surfaced to the RBX status
  # so operators see the FCU's reason without digging through the mavros log.
  FCU_TEXT_REMIND_S = 10.0   # re-surface a persisting FCU message at most this often
  FCU_TEXT_RECENT_S = 15.0   # treat FCU text within this window as a command's failure reason
  last_fcu_text = ""
  last_fcu_severity = None
  last_fcu_text_time = 0.0
  _last_surfaced_fcu_text = ""
  _last_surfaced_fcu_time = 0.0

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  drv_dict = dict()
  ### RBX Driver NODE Initialization
  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
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
        self.mav_node_name = self.drv_dict['DEVICE_DICT']['mavlink_node_name']
        self.connection = self.drv_dict['DEVICE_DICT']['connection']
        # Gates the data_source_description reported to RBXRobotIF (the Sim
        # Connector app discovers simulators by it).
        self.is_sitl = (str(self.connection).upper() == 'SITL')
        # Gates the link watchdog -- see LINK_LOST_SEC
        self.watch_link = self.is_sitl or (str(self.connection).upper() == 'SERIAL')
        # Absent from a DEVICE_DICT written by a discovery without the option
        self.has_fake_gps = (str(self.drv_dict['DEVICE_DICT'].get('fake_gps', False)).lower() == 'true')
    except Exception as e:
        self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
        return

    # Offboard target and motor-test state. RLocks, so the stream callback
    # and the motor helpers can call the locked setters while holding them.
    self.setpoint_lock = threading.RLock()
    self.setpoint_kind = None
    self.setpoint_position_enu = [0.0, 0.0, 0.0]
    self.setpoint_yaw_rad = 0.0
    self.setpoint_velocity_enu = [0.0, 0.0, 0.0]
    self.setpoint_yaw_rate_rads = 0.0
    self.setpoint_expiry_time = None
    self.motor_lock = threading.RLock()

    ##############################
    # Get Mavlink NameSpace
    self.msg_if.pub_info("Waiting for mavlink node that includes: " + self.mav_node_name)
    mav_node_name = nepi_sdk.wait_for_node(self.mav_node_name)
    if mav_node_name == "":
      # wait_for_node gives up after 60 s. Exiting lets discovery purge the
      # device (it watches this process) and relaunch both nodes.
      self.msg_if.pub_warn("Mavlink node " + self.mav_node_name + " never appeared")
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no mavlink node was found")
      return
    MAVLINK_NAMESPACE = (mav_node_name + '/')
    self.msg_if.pub_info("Using mavlink namespace: " + MAVLINK_NAMESPACE)
    # Start Mavlink State Subscriber
    MAVLINK_STATE_TOPIC = MAVLINK_NAMESPACE + "state"
    # Wait for MAVLink State topic to publish then subscribe
    self.msg_if.pub_info("Waiting for topic: " + MAVLINK_STATE_TOPIC)
    nepi_sdk.wait_for_topic(MAVLINK_STATE_TOPIC)
    self.msg_if.pub_info("Starting state scubscriber callback")
    nepi_sdk.create_subscriber(MAVLINK_STATE_TOPIC, State, self.get_state_callback, queue_size = 1)
    while self.state_current == "None" and not nepi_sdk.is_shutdown():
      self.msg_if.pub_info("Waiting for mavlink state status to set")
      time.sleep(0.1)
    while self.mode_current == "None" and not nepi_sdk.is_shutdown():
      self.msg_if.pub_info("Waiting for mavlink mode status to set")
      time.sleep(0.1)
    self.msg_if.pub_info("Starting State: " + self.state_current)
    self.msg_if.pub_info("Starting Mode: " + self.mode_current)

    # MAVLINK Required Services
    self.msg_if.pub_info("Configuring interfaces for mavlink namespace: " + MAVLINK_NAMESPACE)
    ## Define Mavlink Services Calls
    MAVLINK_SET_HOME_SERVICE = MAVLINK_NAMESPACE + "cmd/set_home"
    MAVLINK_SET_MODE_SERVICE = MAVLINK_NAMESPACE + "set_mode"
    MAVLINK_ARMING_SERVICE = MAVLINK_NAMESPACE + "cmd/arming"
    MAVLINK_COMMAND_SERVICE = MAVLINK_NAMESPACE + "cmd/command"

    # Wait for the services to be advertised before reporting Initialization
    # Complete: mavros's state topic flows before its service servers
    # register, and a call in that gap fails silently (see the matching
    # comment in rbx_ardupilot_node.py).
    self.msg_if.pub_info("Waiting for mavlink services under: " + MAVLINK_NAMESPACE)
    nepi_sdk.wait_for_service(MAVLINK_SET_MODE_SERVICE)
    nepi_sdk.wait_for_service(MAVLINK_ARMING_SERVICE)

    # Remembered so a failed call can name the service it was trying to reach.
    self.mode_client_name = MAVLINK_SET_MODE_SERVICE
    self.arming_client_name = MAVLINK_ARMING_SERVICE
    self.mavlink_namespace = MAVLINK_NAMESPACE

    self.set_home_client = nepi_sdk.connect_service(MAVLINK_SET_HOME_SERVICE, CommandHome)
    self.mode_client = nepi_sdk.connect_service(MAVLINK_SET_MODE_SERVICE, SetMode)
    self.arming_client = nepi_sdk.connect_service(MAVLINK_ARMING_SERVICE, CommandBool)
    self.command_client = nepi_sdk.connect_service(MAVLINK_COMMAND_SERVICE, CommandLong)

    # Subscribe to MAVLink topics
    MAVLINK_EXTENDED_STATE_TOPIC = MAVLINK_NAMESPACE + "extended_state"
    MAVLINK_BATTERY_TOPIC = MAVLINK_NAMESPACE + "battery"
    nepi_sdk.create_subscriber(MAVLINK_EXTENDED_STATE_TOPIC, ExtendedState, self.extendedStateTopicCb, queue_size = 1)
    nepi_sdk.create_subscriber(MAVLINK_BATTERY_TOPIC, BatteryState, self.get_mavlink_battery_callback, queue_size = 1)

    # Position and orientation come from local_position/pose (ENU, relative
    # to the EKF origin), the frame the offboard setpoints are written in.
    # Location comes from global_position/global.
    MAVLINK_SOURCE_POSE_TOPIC = MAVLINK_NAMESPACE + "local_position/pose"
    MAVLINK_SOURCE_GPS_TOPIC = MAVLINK_NAMESPACE + "global_position/global"
    MAVLINK_SOURCE_HEADING_TOPIC = MAVLINK_NAMESPACE + "global_position/compass_hdg"

    nepi_sdk.create_subscriber(MAVLINK_SOURCE_POSE_TOPIC, PoseStamped, self.localPoseTopicCb, queue_size = 1)
    nepi_sdk.create_subscriber(MAVLINK_SOURCE_GPS_TOPIC, NavSatFix, self.gps_topic_callback, queue_size = 1)
    nepi_sdk.create_subscriber(MAVLINK_SOURCE_HEADING_TOPIC, Float64, self.heading_topic_callback, queue_size = 1)

    # FCU status text (pre-arm/arm rejections, EKF messages, failsafes, etc.)
    MAVLINK_STATUSTEXT_TOPIC = MAVLINK_NAMESPACE + "statustext/recv"
    nepi_sdk.create_subscriber(MAVLINK_STATUSTEXT_TOPIC, StatusText, self.get_statustext_callback, queue_size = 10)

    ## Define Mavlink Publishers
    # The only setpoint publisher -- see sendSetpointStreamCb.
    MAVLINK_SETPOINT_RAW_LOCAL_TOPIC = MAVLINK_NAMESPACE + "setpoint_raw/local"
    self.setpoint_raw_local_pub = nepi_sdk.create_publisher(MAVLINK_SETPOINT_RAW_LOCAL_TOPIC, PositionTarget, queue_size=1)

    self.msg_if.pub_info("... Connected to Mavlink!")

    # Fake GPS commands (takeoff climb, gotos, stop, land, home) go to the Nav
    # Sim app's GPS instance that injects GPS_INPUT into THIS vehicle's mavros
    # node. Each nav_sim GPS instance reports the mavros node it is bound to on
    # its own status topic, so updateFakeGpsTargetCb follows those and points
    # the command publishers at the instance bound to this vehicle, never at
    # one feeding another vehicle. With no instance bound here, pubFakeGps is
    # a no-op.
    self.fake_gps_lock = threading.Lock()
    self.fake_gps_status_subs = dict()   # instance namespace -> status subscriber
    self.fake_gps_bound_mavros = dict()  # instance namespace -> its selected_mavros_node
    self.fake_gps_target_ns = None
    # Replaced whole on every target change, never mutated, so pubFakeGps can
    # read it without the lock
    self.fake_gps_pubs = dict()          # command name -> publisher on the target
    if NepiAppGpsSimStatus is None:
      self.msg_if.pub_warn("nepi_app_nav_sim is not installed -- fake GPS commands are disabled")
    else:
      nepi_sdk.start_timer_process(1.0, self.updateFakeGpsTargetCb, oneshot = True)
    # The fake fix itself reaches PX4 through the GPS_INPUT to HIL_GPS relay
    # (see FAKE_GPS_HIL_MAX_RATE_HZ). Not on SITL, which has its own GPS.
    if self.is_sitl == False:
      self.fake_gps_hil_pub = nepi_sdk.create_publisher(MAVLINK_NAMESPACE + "hil/gps", HilGPS, queue_size = 1)
      nepi_sdk.create_subscriber(MAVLINK_NAMESPACE + "gps_input/gps_input", GPSINPUT, self.fakeGpsInputCb, queue_size = 1)
      if self.has_fake_gps:
        self.param_get_client = nepi_sdk.connect_service(MAVLINK_NAMESPACE + "param/get", ParamGet)
        nepi_sdk.start_timer_process(self.HIL_GPS_PARAM_RETRY_SEC, self.checkHilGpsParamsCb, oneshot = True)

    ##############################
    # Settings
    self.settings_dict = self.initSettingsDict()
    self.settings_dict = self.refreshSettingsDict()

    # Per-motor commanded test ratios (0-1) and the time of each motor's last
    # Servo page command, which motor_test_timeout_s is measured from.
    motor_count = int(nepi_controls.get_value(self.settings_dict, 'motor_count'))
    self.motor_ratios = [0.0] * motor_count
    self.motor_change_times = [0.0] * motor_count
    # One servo (SVX) device per motor, named <device name>_motor_<n>. The
    # motor count is read from the motor_count setting HERE, once: changing
    # motor_count later resizes motor_ratios but adds or removes no motor
    # servo devices. This is the factory value, read before RBXRobotIF's
    # SettingsIF restores saved settings, so a saved motor_count does not
    # change the device count even after a restart. The names are entered now so
    # RBXRobotIF sees its manual (servo) controls at construction; the
    # DeviceSVXIF instances that share its IFs fill them in afterwards
    # (createMotorSvxIfs).
    self.motor_svx_if_dict = dict()
    for motor_ind in range(motor_count):
      self.motor_svx_if_dict[self.device_name + "_motor_" + str(motor_ind + 1)] = None
    self.motor_svx_dirs = [1] * motor_count
    self.motor_svx_speed_ratios = [0.0] * motor_count
    self.motor_svx_stop_triggers = [False] * motor_count


    # Launch the NEPI RBX interface -- this takes care of initializing all the rbx
    # settings from config, subscribing/advertising topics and services, etc.
    self.msg_if.pub_info("Launching NEPI RBX interface...")
    self.device_info_dict["device_name"] = self.device_name
    self.device_info_dict["path"] = self.device_path
    self.device_info_dict["serial_number"] = ""
    self.device_info_dict["hw_version"] = ""
    self.device_info_dict["sw_version"] = ""
    self.msg_if.pub_info(str(self.device_info_dict))


    self.rbx_if = RBXRobotIF(device_info = self.device_info_dict,
                                  # 'simulator' is what the Sim Connector app's
                                  # device discovery matches on, so a PX4 SITL
                                  # instance appears in its selector. A real
                                  # airframe reports the factory 'control_system'.
                                  data_source_description = ('simulator' if self.is_sitl else 'control_system'),
                                  getSettingsFunction = self.getSettingsFunction,
                                  setSettingFunction = self.setSettingFunction,
                                  axisCapabilities = self.caps_dict,
                                  getBatteryPercentFunction = self.getBatteryPercent,
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
                                  # Motor test: one DeviceSVXIF per motor, see createMotorSvxIfs
                                  servo_if_dict = self.motor_svx_if_dict,
                                  teleopControlsReadyFunction = self.teleopControlsReady,
                                  setTeleopVelocityFunction = self.setTeleopVelocity,
                                  autonomousControlsReadyFunction = self.autonomousControlsReady,
                                  getHomeFunction = self.getHomeLocation,
                                  setHomeFunction = self.setHomeLocation,
                                  goHomeFunction = self.goHome,
                                  goStopFunction = self.goStop,
                                  gotoPoseFunction = self.gotoPose,
                                  gotoPositionFunction = self.gotoPosition,
                                  gotoLocationFunction = self.gotoLocation,
                                  gotoVelocityFunction = self.gotoVelocity,
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
    # The autopilot's navpose is published by an NPX device at
    # <node>/<device_name>/npx/navpose, from getNavPoseCb, and selected as the
    # RBX navpose source so RBXRobotIF's current_* control state tracks it.
    self.msg_if.pub_debug("Starting NPX Device IF Initialization")
    self.npx_if = NPXDeviceIF(device_info = self.device_info_dict,
        data_source_description = ('simulator' if self.is_sitl else 'control_system'),
        getNavPoseCb = self.getNavPoseCb,
        max_navpose_update_rate = self.POSITION_UPDATE_RATE,
        msg_if = self.msg_if,
        node_if = self.node_if,
        settings_if = self.settings_if,
        save_data_if = self.save_data_if,
        connect_navpose_if = self.connect_navpose_if
        )
    # Make the autopilot's own navpose the RBX navpose source (see selectOwnNavPoseCb).
    nepi_sdk.start_timer_process(1.0, self.selectOwnNavPoseCb, oneshot = True)

    ##################################
    # Setup Motor SVX IF Classes ####################
    self.createMotorSvxIfs()

    ## Start the offboard setpoint stream. It publishes nothing until the
    ## first local pose arrives, then runs for the life of the node.
    setpoint_pub_interval = float(1) / self.SETPOINT_PUBLISH_RATE_HZ
    nepi_sdk.start_timer_process(setpoint_pub_interval, self.sendSetpointStreamCb)
    ## Start the motor test refresh loop -- see MOTOR_TEST_REFRESH_SEC
    nepi_sdk.start_timer_process(self.MOTOR_TEST_REFRESH_SEC, self.motorTestRefreshCb)
    ## Start the link watchdog -- see LINK_LOST_SEC
    if self.watch_link:
      nepi_sdk.start_timer_process(self.LINK_WATCHDOG_INTERVAL_SEC, self.linkWatchdogCb)
    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    #Set up node shutdown
    nepi_sdk.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    nepi_sdk.spin()


  #**********************
  # Setting functions

  def initSettingsDict(self):
    init_settings_dict = dict()
    for setting_name in self.CAP_SETTINGS.keys():
      cap_setting = self.CAP_SETTINGS[setting_name]
      setting_type = cap_setting['type']
      setting_dict = dict()
      setting_dict['type'] = setting_type
      # The retired cap-settings form carried an Int/Float control's min and
      # max in an 'options' pair; a Selection/Discrete/String control's
      # actual option list also rode in 'options' -- the controls contract
      # splits these into 'bounds' (numeric) vs 'options' (named choices).
      if 'options' in cap_setting.keys():
        try:
          if setting_type == 'Int':
            setting_dict['bounds'] = [int(cap_setting['options'][0]), int(cap_setting['options'][1])]
          elif setting_type == 'Float':
            setting_dict['bounds'] = [float(cap_setting['options'][0]), float(cap_setting['options'][1])]
          else:
            setting_dict['options'] = [str(option) for option in cap_setting['options']]
        except Exception as e:
          self.msg_if.pub_warn("Invalid bounds/options for setting: " + setting_name + " : " + str(e))

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
    # Nothing live to read back: every setting here is a driver-side value,
    # none is read from the FCU.
    return copy.deepcopy(self.settings_dict)

  def getSettingSelection(self, setting_name):
    # nepi_controls.get_value() hands a Selection back as a one-element list
    # (['TRUE']), so comparing it against 'TRUE' is never equal. This returns
    # the selected option itself.
    value = nepi_controls.get_value(self.settings_dict, setting_name)
    if isinstance(value, list):
      value = value[0] if len(value) > 0 else None
    return value

  def getSettingsFunction(self):
    # Deep copy, not a bare reference: SettingsIF keeps whatever this returns
    # and mutates its entries in place, which would otherwise land on this
    # node's own live settings_dict (see the matching comment in
    # rbx_ardupilot_node.py).
    return copy.deepcopy(self.settings_dict)

  def setSettingFunction(self, setting_name, setting_value):
    success = False
    setting_str = setting_name + ":" + str(setting_value)
    if setting_name not in self.settings_dict.keys():
      msg = (self.node_name + " Setting name " + setting_str + " is not supported")
      return success, msg, copy.deepcopy(self.settings_dict)
    # get_clean_value() returns None for an invalid value -- test 'is None',
    # not falsiness, since a Toggle can legitimately clean to False.
    if nepi_controls.get_clean_value(self.settings_dict, setting_name, setting_value) is None:
      msg = (self.node_name + " Setting data " + setting_str + " is not valid")
      return success, msg, copy.deepcopy(self.settings_dict)

    self.settings_dict = nepi_controls.set_value(self.settings_dict, setting_name, setting_value)
    success = True
    msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
    if setting_name == 'motor_count':
      # motor_ratios is sized from motor_count, so it has to be resized with
      # it. Any motor still under test is released first, so a dropped index
      # cannot keep a motor running.
      with self.motor_lock:
        self.releaseAllMotors()
        motor_count = int(nepi_controls.get_value(self.settings_dict, 'motor_count'))
        self.motor_ratios = [0.0] * motor_count
        self.motor_change_times = [0.0] * motor_count
    # Deep copy, matching getSettingsFunction()'s own reasoning.
    return success, msg, copy.deepcopy(self.settings_dict)

  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    return self.state_ind

  def setStateInd(self,state_ind):
    state_last = self.state_current
    set_state_function = globals()[self.RBX_STATE_FUNCTIONS[state_ind]]
    success = set_state_function(self)
    if success:
      self.state_ind = state_ind
      self.state_current = self.RBX_STATES[state_ind]
      self.state_last = state_last
    return success

  def getModeInd(self):
    return self.mode_ind

  def setModeInd(self,mode_ind):
    mode_on_entry = self.mode_current
    set_mode_function = globals()[self.RBX_MODE_FUNCTIONS[mode_ind]]
    success = set_mode_function(self)
    if success:
      if self.RBX_MODES[mode_ind] == "RESUME":
        if self.mode_last != "RESUME" and self.mode_last in self.RBX_MODES:
          self.mode_current = self.mode_last
          self.mode_ind = self.RBX_MODES.index(self.mode_last)
          self.mode_last = mode_on_entry # Don't update last on resume
      else:
        if (mode_ind >= 0 and mode_ind <= (len(self.RBX_MODES)-1)):
          self.mode_ind = mode_ind
          self.mode_current = self.RBX_MODES[mode_ind]
          self.mode_last = mode_on_entry # Don't update last on resume
    return success

  def checkStopFunction(self):
    triggered = self.stop_triggered
    self.stop_triggered = False # Reset Stop Trigger
    return triggered

  def getBatteryPercent(self):
    return self.battery_percent

  def setHomeLocation(self,geo_point):
    self.set_home_location(geo_point)

  def getHomeLocation(self):
    return self.home_location

  def getMotorControlRatios(self):
    return self.motor_ratios

  def setSetupActionInd(self,action_ind):
    set_action_function = globals()[self.RBX_SETUP_ACTION_FUNCTIONS[action_ind]]
    success = set_action_function(self)
    return success

  def setGoActionInd(self,action_ind):
    set_action_function = globals()[self.RBX_GO_ACTION_FUNCTIONS[action_ind]]
    success = set_action_function(self)
    return success


  ##############################
  # Motor Test (MAV_CMD_ACTUATOR_TEST)

  def setMotorControlRatio(self,motor_ind,speed_ratio):
    speed_ratio = max(0.0, min(1.0, speed_ratio))
    # Held across the service call so the refresh loop can never resend a
    # spin for a motor whose release is in flight.
    with self.motor_lock:
      if motor_ind < 0 or motor_ind >= len(self.motor_ratios):
        self.msg_if.pub_warn("Motor test ignored: motor index " + str(motor_ind + 1) + " out of range")
        return
      self.motor_change_times[motor_ind] = nepi_utils.get_time()
      if speed_ratio > 0.0:
        success = self.sendActuatorTest(motor_ind, self.getMotorTestOutput(speed_ratio))
      else:
        success = self.sendActuatorTest(motor_ind, None)
      # Zeroed on any failure, so the refresh loop never resends a command
      # PX4 has already refused.
      self.motor_ratios[motor_ind] = speed_ratio if success else 0.0
    if success == False:
      self.reportMotorTestRejected(motor_ind)

  def getMotorTestOutput(self, speed_ratio):
    # Scale the motor's 0-1 Servo page speed ratio onto
    # [0, motor_test_max_throttle_percent], so the cap integrates with the
    # Servo page's speed control instead of overriding it.
    max_throttle_percent = float(nepi_controls.get_value(self.settings_dict,'motor_test_max_throttle_percent'))
    return speed_ratio * max_throttle_percent / 100.0

  def sendActuatorTest(self, motor_ind, output_value):
    # output_value None releases the motor. See MAV_CMD_ACTUATOR_TEST above.
    test_cmd = CommandLongRequest()
    test_cmd.broadcast = False
    test_cmd.command = self.MAV_CMD_ACTUATOR_TEST
    test_cmd.confirmation = 0
    if output_value is None:
      test_cmd.param1 = float('nan')   # NaN = stop, whatever PX4 does with the action
      test_cmd.param2 = 0.0            # timeout <= 0 releases control at once
    else:
      test_cmd.param1 = float(output_value)
      test_cmd.param2 = self.ACTUATOR_TEST_TIMEOUT_SEC
    test_cmd.param3 = 0.0
    test_cmd.param4 = 0.0
    test_cmd.param5 = float(motor_ind + 1)  # ACTUATOR_OUTPUT_FUNCTION_MOTORn = n, 1-based
    test_cmd.param6 = 0.0
    test_cmd.param7 = 0.0
    response = nepi_sdk.call_service(self.command_client, test_cmd)
    return (response is not None and response.success)

  def reportMotorTestRejected(self, motor_ind):
    fail_msg = ("Motor " + str(motor_ind + 1) + " test rejected -- PX4 refuses actuator tests"
                + " while armed or with the safety switch engaged")
    reason = self.get_recent_fcu_reason()
    if reason != "":
      fail_msg = fail_msg + " (FCU: " + reason + ")"
    self.msg_if.pub_warn(fail_msg)
    if self.rbx_if is not None:
      self.rbx_if.update_error_msg(fail_msg)

  def releaseAllMotors(self):
    # Returns the 1-based numbers of the motors it released
    released_list = []
    with self.motor_lock:
      for motor_ind in range(len(self.motor_ratios)):
        if self.motor_ratios[motor_ind] > 0.0:
          self.sendActuatorTest(motor_ind, None)
          self.motor_ratios[motor_ind] = 0.0
          released_list.append(motor_ind + 1)
    return released_list

  def motorTestRefreshCb(self, timer):
    # PX4 ends any actuator test after 3 s, so a motor still running from the
    # Servo page is resent every MOTOR_TEST_REFRESH_SEC, until
    # motor_test_timeout_s has passed since that motor's last Servo page
    # command; then it is released.
    rejected_list = []
    with self.motor_lock:
      if len(self.motor_ratios) == 0 or max(self.motor_ratios) <= 0.0:
        return
      timeout_s = float(nepi_controls.get_value(self.settings_dict,'motor_test_timeout_s'))
      now = nepi_utils.get_time()
      for motor_ind in range(len(self.motor_ratios)):
        speed_ratio = self.motor_ratios[motor_ind]
        if speed_ratio <= 0.0:
          continue
        if (now - self.motor_change_times[motor_ind]) >= timeout_s:
          self.sendActuatorTest(motor_ind, None)
          self.motor_ratios[motor_ind] = 0.0
          self.msg_if.pub_info("Motor " + str(motor_ind + 1) + " test released after " + str(timeout_s) +
                               " s without a slider change")
        elif self.sendActuatorTest(motor_ind, self.getMotorTestOutput(speed_ratio)) == False:
          self.motor_ratios[motor_ind] = 0.0
          rejected_list.append(motor_ind)
    for motor_ind in rejected_list:
      self.reportMotorTestRejected(motor_ind)


  ##########################
  # Motor Servo (SVX) Functions
  # Every motor is its own DeviceSVXIF (a spin servo) on the RUI Servo page.
  # Each callback acts on one motor through setMotorControlRatio above, so the
  # PX4 actuator test path is shared: its checks (index range, 0-1 clamp,
  # motor_test_max_throttle_percent), the motor_lock, the MOTOR_TEST_REFRESH_SEC
  # resend and motor_test_timeout_s release in motorTestRefreshCb, and the
  # rejection report all still apply. PX4 refuses an actuator test while armed
  # or with the safety switch engaged. A negative (reverse) command clamps to 0
  # there, which stops the motor: a PX4 actuator test cannot spin backwards.

  def createMotorSvxIfs(self):
    # One DeviceSVXIF per motor, shaped like ptx_iqr's per-axis servos: each
    # shares the RBX interface's msg_if, node_if, settings_if, save_data_if and
    # connect_navpose_if, and passes its own <node>/motor_<n> namespace, so it
    # lands at <node>/motor_<n>/svx with its own topics, params and node_if keys.
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
                                        connect_navpose_if = self.connect_navpose_if,
                                        namespace = nepi_sdk.create_namespace(self.node_namespace, 'motor_' + str(motor_ind + 1))
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


  ##############################
  # Goto Functions
  # Each one only changes the offboard target; sendSetpointStreamCb sends it.

  def goStop(self):
    self.stop_triggered = True
    self.setHoldAtCurrent()
    self.pubFakeGps('go_stop', Empty())
    return True

  def goHome(self):
    home_loc = self.home_location
    if home_loc.latitude == -999 or home_loc.longitude == -999:
      fail_msg = "Go Home ignored: no home location yet (it is captured when the vehicle arms with a global position)"
      self.msg_if.pub_warn(fail_msg)
      self.rbx_if.update_error_msg(fail_msg)
      return False
    self.stop_triggered = True
    nepi_sdk.sleep(1,10)
    self.stop_triggered = False
    nepi_sdk.sleep(3,30)
    # -999 altitude holds the current altitude: home's altitude was captured
    # on the ground at arming, so flying to it would mean flying into the
    # ground on the way back.
    setpoint_location = [home_loc.latitude,home_loc.longitude,-999,-999]
    return self.rbx_if.setpoint_location_global_wgs84(setpoint_location)

  def gotoPose(self,attitude_enu_degs):
    att_str = str(attitude_enu_degs)
    self.msg_if.pub_info("Recieved Pose setpoint command: " + att_str)
    # A multicopter cannot hold a commanded roll or pitch while it
    # station-keeps -- tilting is how it moves. Only the yaw is applied.
    if self.gotopose_rp_logged == False:
      self.gotopose_rp_logged = True
      self.msg_if.pub_info("GoTo Pose: this multicopter applies only the yaw; roll and pitch are ignored")
    [x_m, y_m, z_m, yaw_rad] = self.getCurrentPose()
    self.setHoldTarget(x_m, y_m, z_m, math.radians(attitude_enu_degs[2]))

  def gotoPosition(self,point_enu_m,orientation_enu_deg):
    pos_str = str(point_enu_m)
    self.msg_if.pub_info("Recieved Position setpoint command: " + pos_str)
    # RBXRobotIF hands drivers an ENU OFFSET from the current position (see
    # rbx_ardupilot_node.py's gotoPosition); the driver adds its own position.
    [x_m, y_m, z_m, yaw_rad] = self.getCurrentPose()
    self.setHoldTarget(x_m + point_enu_m.x, y_m + point_enu_m.y, z_m + point_enu_m.z,
                       math.radians(orientation_enu_deg[2]))
    # The fake GPS gets the offset as received: nav_sim's goto_position is
    # itself a relative move from the simulated fix's current spot.
    self.pubFakeGps('goto_position', Point(x = point_enu_m.x, y = point_enu_m.y, z = point_enu_m.z))

  def gotoLocation(self,geopoint_amsl,orientation_enu_deg):
    loc_str = str(geopoint_amsl)
    self.msg_if.pub_info("Recieved Location setpoint command: " + loc_str)
    if self.navpose_dict['has_location'] != True:
      fail_msg = "GoTo Location ignored: the FCU is not reporting a global position"
      self.msg_if.pub_warn(fail_msg)
      self.rbx_if.update_error_msg(fail_msg)
      return False
    # Converted to a local offset and flown as an ordinary local HOLD target,
    # so there is still only one setpoint channel. geoid_height_meters is 0
    # in this driver, so the target altitude and the navpose altitude are in
    # the same frame and any ellipsoid/AMSL offset cancels in the difference.
    [east_m, north_m, up_m] = getGeoOffsetEnuM(self.navpose_dict['latitude'], self.navpose_dict['longitude'],
                                               self.navpose_dict['altitude_m'],
                                               geopoint_amsl.latitude, geopoint_amsl.longitude,
                                               geopoint_amsl.altitude)
    [x_m, y_m, z_m, yaw_rad] = self.getCurrentPose()
    # RBXRobotIF passes ENU orientation, so the yaw is used as is.
    yaw_enu_deg = orientation_enu_deg[2]
    self.setHoldTarget(x_m + east_m, y_m + north_m, z_m + up_m, math.radians(yaw_enu_deg))
    # With a navpose geoid height of 0 this goal's altitude is the ellipsoid
    # height, so it goes through the same AMSL conversion as every other
    # location sent to the fake GPS (see pubFakeGpsLocation).
    self.pubFakeGpsLocation('goto_location', geopoint_amsl)
    return True

  def gotoVelocity(self, x_mps, y_mps, z_mps, yaw_degps, duration_s):
    # A timed, open-loop BODY-frame command (x forward, y LEFT, z up, yaw
    # rate positive to port). RBXRobotIF expects this to block for duration_s
    # and stop the vehicle itself, and rejects other goto commands until it
    # returns. The body vector is rotated into ENU once, with the yaw at the
    # start of the command.
    self.msg_if.pub_info("Recieved Velocity setpoint command: " + str([x_mps, y_mps, z_mps, yaw_degps, duration_s]))
    max_horizontal_mps = float(nepi_controls.get_value(self.settings_dict,'max_horizontal_speed_mps'))
    max_vertical_mps = float(nepi_controls.get_value(self.settings_dict,'max_vertical_speed_mps'))
    max_yaw_dps = float(nepi_controls.get_value(self.settings_dict,'max_yaw_rate_dps'))
    [body_x, body_y] = limitHorizontal(float(x_mps), float(y_mps), max_horizontal_mps)
    vel_z = clampValue(float(z_mps), -max_vertical_mps, max_vertical_mps)
    yaw_rate_dps = clampValue(float(yaw_degps), -max_yaw_dps, max_yaw_dps)
    [x_m, y_m, z_m, yaw_rad] = self.getCurrentPose()
    [enu_x, enu_y] = rotateBodyToEnu(body_x, body_y, yaw_rad)
    end_time = nepi_utils.get_time() + max(0.0, float(duration_s))
    # A stop sent before this command began was meant for an earlier one
    self.stop_triggered = False
    self.setVelocityTarget(enu_x, enu_y, vel_z, math.radians(yaw_rate_dps), end_time)
    completed = False
    while not nepi_sdk.is_shutdown():
      if nepi_utils.get_time() >= end_time:
        completed = True
        break
      if self.stop_triggered:
        self.stop_triggered = False
        self.msg_if.pub_info("GoTo Velocity stopped by a Stop command")
        break
      if self.isOffboardArmed() == False:
        self.msg_if.pub_warn("GoTo Velocity ended early: the vehicle left OFFBOARD")
        break
      time.sleep(self.GOTO_VELOCITY_CHECK_INTERVAL_SEC)
    self.setHoldAtCurrent()
    return completed


  ##############################
  # Offboard Setpoint Stream

  def getCurrentPose(self):
    # [x_m, y_m, z_m, yaw_rad], all local ENU
    return [self.navpose_dict['x_m'], self.navpose_dict['y_m'], self.navpose_dict['z_m'],
            math.radians(self.navpose_dict['yaw_deg'])]

  def localPoseFresh(self):
    if self.local_pose_time is None:
      return False
    return (nepi_utils.get_time() - self.local_pose_time) <= self.LOCAL_POSE_STALE_SEC

  def isOffboardArmed(self):
    return self.mode_current == "OFFBOARD" and self.state_current == "ARM"

  def setHoldTarget(self, x_m, y_m, z_m, yaw_rad):
    with self.setpoint_lock:
      self.setpoint_kind = self.SETPOINT_HOLD
      self.setpoint_position_enu = [float(x_m), float(y_m), float(z_m)]
      self.setpoint_yaw_rad = float(yaw_rad)
      self.setpoint_velocity_enu = [0.0, 0.0, 0.0]
      self.setpoint_yaw_rate_rads = 0.0
      self.setpoint_expiry_time = None

  def setHoldAtCurrent(self):
    [x_m, y_m, z_m, yaw_rad] = self.getCurrentPose()
    self.setHoldTarget(x_m, y_m, z_m, yaw_rad)

  def setVelocityTarget(self, vx_mps, vy_mps, vz_mps, yaw_rate_rads, expiry_time):
    with self.setpoint_lock:
      self.setpoint_kind = self.SETPOINT_VELOCITY
      self.setpoint_velocity_enu = [float(vx_mps), float(vy_mps), float(vz_mps)]
      self.setpoint_yaw_rate_rads = float(yaw_rate_rads)
      self.setpoint_expiry_time = float(expiry_time)

  def waitForSetpointStream(self):
    # True once the stream has been publishing for SETPOINT_STREAM_MIN_SEC,
    # waiting up to SETPOINT_STREAM_WAIT_SEC for that.
    waited = 0.0
    while waited <= self.SETPOINT_STREAM_WAIT_SEC and not nepi_sdk.is_shutdown():
      start_time = self.setpoint_stream_start_time
      if start_time is not None and (nepi_utils.get_time() - start_time) >= self.SETPOINT_STREAM_MIN_SEC:
        return True
      time.sleep(0.1)
      waited += 0.1
    return False

  def sendSetpointStreamCb(self, timer):
    # The one setpoint publisher in this driver. Every tick:
    # - Not OFFBOARD (or not armed): HOLD at the current position and yaw,
    #   refreshed every tick, so any switch into OFFBOARD -- from the RUI,
    #   the RC or LAUNCH -- holds where the vehicle is instead of flying back
    #   to a stale target. Refreshing while disarmed as well means arming in
    #   OFFBOARD also holds where the vehicle sits.
    # - OFFBOARD with an expired VELOCITY target: HOLD at the position and
    #   yaw of that moment. This is how teleop and goto_velocity stop when
    #   their commands stop.
    # Otherwise the current target is sent unchanged.
    if self.local_pose_time is None or nepi_sdk.is_shutdown():
      return
    now = nepi_utils.get_time()
    [x_m, y_m, z_m, yaw_rad] = self.getCurrentPose()
    setpoint_msg = PositionTarget()
    setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    with self.setpoint_lock:
      if self.isOffboardArmed() == False or self.setpoint_kind is None:
        self.setHoldTarget(x_m, y_m, z_m, yaw_rad)
      elif self.setpoint_kind == self.SETPOINT_VELOCITY and now >= self.setpoint_expiry_time:
        self.setHoldTarget(x_m, y_m, z_m, yaw_rad)
      if self.setpoint_kind == self.SETPOINT_VELOCITY:
        setpoint_msg.type_mask = self.VELOCITY_TYPE_MASK
        setpoint_msg.velocity.x = self.setpoint_velocity_enu[0]
        setpoint_msg.velocity.y = self.setpoint_velocity_enu[1]
        setpoint_msg.velocity.z = self.setpoint_velocity_enu[2]
        setpoint_msg.yaw_rate = self.setpoint_yaw_rate_rads
      else:
        setpoint_msg.type_mask = self.HOLD_TYPE_MASK
        setpoint_msg.position.x = self.setpoint_position_enu[0]
        setpoint_msg.position.y = self.setpoint_position_enu[1]
        setpoint_msg.position.z = self.setpoint_position_enu[2]
        setpoint_msg.yaw = self.setpoint_yaw_rad
    setpoint_msg.header.stamp = nepi_sdk.get_msg_stamp()
    # publish_pub catches the error: shutdown closes the publisher before this
    # 30 Hz timer stops, so a tick can still land after the check above.
    if nepi_sdk.publish_pub(self.setpoint_raw_local_pub, setpoint_msg) == False:
      return
    if self.setpoint_stream_start_time is None:
      self.setpoint_stream_start_time = now
      self.msg_if.pub_info("Offboard setpoint stream started")


  ##########################
  # Control Ready Check Funcitons

  def manualControlsReady(self):
    # Manual controls are the motor test, and PX4 denies actuator tests while
    # armed.
    return self.state_current == "DISARM"

  def isAirborne(self):
    # PX4's land detector, through mavros extended_state, when it is current.
    # Otherwise fall back to the takeoff bookkeeping, rechecking the real
    # local altitude against the last takeoff goal on every query (see
    # rbx_ardupilot_node.py's isAirborne for why the flag alone is not
    # trusted).
    if self.extended_state is not None and self.extended_state_time is not None:
      if (nepi_utils.get_time() - self.extended_state_time) <= self.EXTENDED_STATE_STALE_SEC:
        return self.extended_state.landed_state == ExtendedState.LANDED_STATE_IN_AIR
    if self.takeoff_complete:
      return True
    if self.takeoff_goal_alt is None or self.takeoff_error_bound_m is None:
      return False
    alt_error = self.takeoff_goal_alt - self.navpose_dict['z_m']
    if abs(alt_error) <= self.takeoff_error_bound_m:
      self.takeoff_complete = True
      return True
    return False

  def autonomousControlsReady(self):
    # Checked here, not just in the RUI (which hides the controls), so
    # disabling it blocks the command for any client.
    if self.getSettingSelection('autonomous_movement_enabled') != 'TRUE':
      return False
    # mode_current/state_current, not RBX_MODES[self.mode_ind]: a PX4 mode
    # this driver does not list leaves mode_ind at -999.
    return self.isOffboardArmed() and self.isAirborne()

  def teleopControlsReady(self):
    if self.getSettingSelection('teleop_movement_enabled') != 'TRUE':
      return False
    return self.isOffboardArmed() and self.isAirborne()

  def setTeleopVelocity(self, linear_x, linear_y, linear_z, angular_z):
    # The channel an outer-loop app streams on: non-blocking, latest wins.
    # Inputs are body-frame ratios in [-1,1] -- x forward, y LEFT, z up, and
    # yaw rate positive counter-clockwise -- the ROS REP-103 convention
    # device_if_rbx.py documents. No sign is flipped here (the RUI's
    # TeleopKeymap.js currently sends the opposite y and yaw signs). Each
    # command expires TELEOP_CMD_TIMEOUT_SEC after it arrives, after which
    # the setpoint stream holds position, so there is no separate stop
    # stream.
    max_horizontal_mps = float(nepi_controls.get_value(self.settings_dict,'max_horizontal_speed_mps'))
    max_vertical_mps = float(nepi_controls.get_value(self.settings_dict,'max_vertical_speed_mps'))
    max_yaw_rads = math.radians(float(nepi_controls.get_value(self.settings_dict,'max_yaw_rate_dps')))
    body_x = clampValue(float(linear_x), -1.0, 1.0) * max_horizontal_mps
    body_y = clampValue(float(linear_y), -1.0, 1.0) * max_horizontal_mps
    [body_x, body_y] = limitHorizontal(body_x, body_y, max_horizontal_mps)
    vel_z = clampValue(float(linear_z), -1.0, 1.0) * max_vertical_mps
    yaw_rate_rads = clampValue(float(angular_z), -1.0, 1.0) * max_yaw_rads
    [x_m, y_m, z_m, yaw_rad] = self.getCurrentPose()
    [enu_x, enu_y] = rotateBodyToEnu(body_x, body_y, yaw_rad)
    self.setVelocityTarget(enu_x, enu_y, vel_z, yaw_rate_rads, nepi_utils.get_time() + self.TELEOP_CMD_TIMEOUT_SEC)


  ##############################
  # RBX NavPose Topic Publishers
  ### Callback to publish RBX navpose data

  def gps_topic_callback(self,navsatfix_msg):
      # No geoid separation source exists in this pipeline, so this is an
      # explicit 0: altitude_m is then the mavros global_position/global
      # altitude itself, and gotoLocation's altitude arithmetic stays in that
      # one frame.
      geoid_height_m = 0
      altitude_wgs84 = navsatfix_msg.altitude - geoid_height_m
      time_ns = nepi_sdk.sec_from_msg_stamp(navsatfix_msg.header.stamp)
      # Location Lat,Long
      self.navpose_dict['has_location'] = True
      self.navpose_dict['time_location'] = time_ns
      self.navpose_dict['latitude'] = navsatfix_msg.latitude
      self.navpose_dict['longitude'] = navsatfix_msg.longitude
      self.navpose_dict['geoid_height_meters'] = geoid_height_m
      # Altitude positive meters WGS84
      self.navpose_dict['has_altitude'] = True
      self.navpose_dict['time_altitude'] = time_ns
      self.navpose_dict['altitude_m'] = altitude_wgs84


  ### Callback to update RBX local pose (orientation + position) navpose data
  def localPoseTopicCb(self, pose_msg):
      or_msg = pose_msg.pose.orientation
      or_list = [or_msg.x, or_msg.y, or_msg.z, or_msg.w]
      pos_msg = pose_msg.pose.position
      rpy = nepi_nav.convert_quat2rpy(or_list)
      time_ns = nepi_sdk.sec_from_msg_stamp(pose_msg.header.stamp)

      # Orientation Degrees in selected 3d frame (roll,pitch,yaw)
      self.navpose_dict['has_orientation'] = True
      self.navpose_dict['time_orientation'] = time_ns
      self.navpose_dict['roll_deg'] = rpy[0]
      self.navpose_dict['pitch_deg'] = rpy[1]
      # rpy[2] directly, no convert_yaw_ned2enu: mavros publishes this
      # quaternion already in ENU, so reflecting it as a NED heading would put
      # yaw and x/y in different frames (see rbx_ardupilot_node.py's
      # odom_topic_callback). It is also the frame the
      # setpoint stream's yaw is written in.
      self.navpose_dict['yaw_deg'] = rpy[2]

      # Position meters local ENU (x east, y north, z up) from the EKF origin
      self.navpose_dict['has_position'] = True
      self.navpose_dict['time_position'] = time_ns
      self.navpose_dict['x_m'] = pos_msg.x
      self.navpose_dict['y_m'] = pos_msg.y
      self.navpose_dict['z_m'] = pos_msg.z
      self.local_pose_time = nepi_utils.get_time()


  ### Callback to update RBX heading navpose data
  def heading_topic_callback(self,heading_msg):
      # Heading in Degrees True North
      self.navpose_dict['has_heading'] = True
      self.navpose_dict['time_heading'] = nepi_utils.get_time()
      self.navpose_dict['heading_deg'] = heading_msg.data


  ### Callback for PX4's land detector state
  def extendedStateTopicCb(self, extended_state_msg):
      self.extended_state = extended_state_msg
      self.extended_state_time = nepi_utils.get_time()


  ### Callback for flight-controller status text (pre-arm/arm rejections, EKF, failsafes).
  ### The FCU reports why a command is refused (e.g. "Arm: Compass not healthy") only as
  ### STATUSTEXT, which otherwise lands in the mavros log. Surface warning-or-worse text to
  ### the RBX status so it shows in the UI, and record it for command handlers to report.
  def get_statustext_callback(self, statustext_msg):
      text = statustext_msg.text.strip()
      if text == "":
          return
      severity = statustext_msg.severity
      now = nepi_utils.get_time()
      # Record the latest so command handlers (e.g. arm) can report the FCU reason
      self.last_fcu_text = text
      self.last_fcu_severity = severity
      self.last_fcu_text_time = now
      # StatusText.WARNING == 4; lower severity value == more severe (MAV_SEVERITY).
      # Skip NOTICE/INFO/DEBUG to avoid log noise. Dedupe identical consecutive
      # messages, but re-remind every FCU_TEXT_REMIND_S while a condition persists.
      if severity <= StatusText.WARNING:
          is_new = (text != self._last_surfaced_fcu_text)
          is_stale = (now - self._last_surfaced_fcu_time) > self.FCU_TEXT_REMIND_S
          if is_new or is_stale:
              self._last_surfaced_fcu_text = text
              self._last_surfaced_fcu_time = now
              fcu_msg = "FCU: " + text
              self.msg_if.pub_warn(fcu_msg)
              if self.rbx_if is not None:
                  self.rbx_if.update_error_msg(fcu_msg)

  ### Returns the most recent FCU status text when it is a recent warning-or-worse
  ### message, used to annotate a failed command with the flight-controller's reason.
  def get_recent_fcu_reason(self):
      if self.last_fcu_text == "":
          return ""
      age = nepi_utils.get_time() - self.last_fcu_text_time
      recent = age <= self.FCU_TEXT_RECENT_S
      severe = (self.last_fcu_severity is None or self.last_fcu_severity <= StatusText.WARNING)
      if recent and severe:
          return self.last_fcu_text
      return ""


  ### Callback returning the full navpose dict to the NPX interface
  def getNavPoseCb(self):
    # NavPose convention: ENU orientation and position from the EKF origin
    # (mavros local_position/pose), compass heading_deg (compass_hdg), WGS84
    # location with altitude_m the mavros global_position/global altitude and
    # geoid_height_meters 0 (global_position/global).
    # Returns None until the local pose and the global fix have both arrived:
    # NPXDeviceIF derives its has_* capabilities once, from the first dict it
    # gets, so a blank dict returned before telemetry would fix every
    # capability at False for the life of the node.
    if self.navpose_dict['has_orientation'] == False or self.navpose_dict['has_location'] == False:
      return None
    return copy.deepcopy(self.navpose_dict)

  def selectOwnNavPoseCb(self, timer):
    # Make the autopilot's own NPX navpose the RBX navpose source. RBXRobotIF
    # takes the current_* state that goto and takeoff completion check against
    # from that source, so it has to be this autopilot.
    #
    # A robot's navpose selector lists only NavPose Manager frames by default
    # (ConnectNavPoseIF filter ['navposes']) and auto-selects the first one, so
    # the NPX topic is added to this robot's selector and an auto-selected frame
    # is replaced. An operator's pick is the SAVED selection: auto-select never
    # saves, set_selected_topic always does. A saved topic inside this node's
    # own namespace is a stale name for this NPX, not an operator's pick.
    #
    # The NPX creates its navpose publisher (navpose_pub_if) only once
    # getNavPoseCb returns data, after a local pose and a GPS fix, which can
    # take minutes, so this keeps checking until then. npx_if.navpose_if is not
    # that topic: it is the node-level navpose this robot republishes, which
    # the selector excludes.
    connect_if = self.connect_navpose_if
    if connect_if is None or self.npx_if is None or nepi_sdk.is_shutdown():
      return
    if self.own_navpose_topic is None:
      navpose_pub_if = getattr(self.npx_if, 'navpose_pub_if', None)
      if navpose_pub_if is not None:
        self.own_navpose_topic = navpose_pub_if.get_namespace()
        # An empty filter list already offers every topic. Replaced, not
        # appended to: the default list is one module-level object that every
        # ConnectNavPoseIF in this process shares.
        if len(connect_if.filter_topics_list) > 0:
          connect_if.filter_topics_list = list(connect_if.filter_topics_list) + [self.own_navpose_topic]
    if self.own_navpose_topic is not None:
      if connect_if.get_selected_topic() == self.own_navpose_topic:
        return
      if self.own_navpose_topic in connect_if.get_available_topics():
        saved = connect_if.node_if.get_param(connect_if.selected_topic_param)
        # NodeIF lowercases topic namespaces but not the node namespace
        # (px4_ttyacm1 vs px4_ttyACM1), so they are compared lowercased.
        saved_lower = str(saved).lower()
        node_lower = self.node_namespace.lower()
        if saved not in ['None', '', None] and saved_lower != node_lower and not saved_lower.startswith(node_lower + '/'):
          self.msg_if.pub_info("Keeping operator navpose source: " + str(saved))
          return
        connect_if.set_selected_topic(self.own_navpose_topic)
        self.msg_if.pub_info("Selected own navpose source: " + self.own_navpose_topic)
        return
    if self.own_navpose_wait_logged == False:
      self.own_navpose_wait_logged = True
      self.msg_if.pub_info("Own navpose source can be selected once the autopilot has a local position and a GPS fix")
    nepi_sdk.start_timer_process(self.OWN_NAVPOSE_RETRY_SEC, self.selectOwnNavPoseCb, oneshot = True)

  #######################
  # Fake GPS (Nav Sim app GPS instance)
  # The same binding as rbx_ardupilot_node.py. Every command the fake GPS gets
  # moves the simulated fix the way the matching offboard target moves the
  # vehicle, so on a bench the FCU's position follows the command. Velocity
  # commands (teleop, goto_velocity) have no nav_sim equivalent and do not
  # move the fix.

  def waitForPubConnection(self, pub, timeout_sec = 10.0):
    # Returns True once at least one subscriber is connected. Absence of a
    # subscriber is not an error -- the Nav Sim app may not be running -- so a
    # timeout just returns False and the caller still publishes.
    waited = 0.0
    while waited < timeout_sec:
      try:
        if pub.get_num_connections() > 0:
          return True
      except Exception:
        return False
      nepi_sdk.sleep(0.25,1)
      waited += 0.25
    return False

  def updateFakeGpsTargetCb(self, timer):
    # Track every Nav Sim GPS instance's status and keep the fake GPS command
    # publishers on the one bound to this vehicle's mavros node. Instances come
    # and go with the app, and an operator can rebind one in the app's RUI at
    # any time, so this runs for the life of the node.
    if nepi_sdk.is_shutdown():
      return
    try:
      status_topics = nepi_sdk.find_topics_by_msg(FAKE_GPS_STATUS_MSG)
      inst_namespaces = [t[:-len('/status')] for t in status_topics if t.endswith('/status')]
      gone_subs = []
      with self.fake_gps_lock:
        for inst_ns in list(self.fake_gps_status_subs.keys()):
          if inst_ns not in inst_namespaces:
            gone_subs.append(self.fake_gps_status_subs.pop(inst_ns))
            self.fake_gps_bound_mavros.pop(inst_ns, None)
        new_namespaces = [ns for ns in inst_namespaces if ns not in self.fake_gps_status_subs]
      for sub in gone_subs:
        if sub is not None:
          try:
            sub.unregister()
          except Exception:
            pass
      for inst_ns in new_namespaces:
        # nav_sim latches this status, so the binding arrives right away
        sub = nepi_sdk.create_subscriber(inst_ns + '/status', NepiAppGpsSimStatus,
                                         self.fakeGpsStatusCb, queue_size = 1,
                                         callback_args = (inst_ns,))
        with self.fake_gps_lock:
          self.fake_gps_status_subs[inst_ns] = sub

      own_mavros = self.mavlink_namespace.rstrip('/')
      with self.fake_gps_lock:
        bound = sorted(ns for ns, mavros_ns in self.fake_gps_bound_mavros.items()
                       if mavros_ns == own_mavros)
      target_ns = bound[0] if len(bound) > 0 else None
      if len(bound) > 1:
        # Two instances injecting into one FCU give it two GPS sources; the
        # operator has to unbind one.
        self.msg_if.pub_warn("Several Nav Sim GPS instances inject into " + own_mavros
                             + ": " + str(bound) + " -- sending fake GPS commands to "
                             + str(target_ns), throttle_s = 30)
      if target_ns != self.fake_gps_target_ns:
        self.setFakeGpsTarget(target_ns)
    except Exception as e:
      self.msg_if.pub_warn("Fake GPS target update failed: " + str(e), throttle_s = 30)
    nepi_sdk.start_timer_process(FAKE_GPS_TARGET_UPDATE_SEC, self.updateFakeGpsTargetCb, oneshot = True)

  def fakeGpsStatusCb(self, status_msg, args):
    inst_ns = args[0]
    with self.fake_gps_lock:
      # Ignore a late message from an instance whose subscriber was just dropped
      if inst_ns in self.fake_gps_status_subs:
        self.fake_gps_bound_mavros[inst_ns] = status_msg.selected_mavros_node

  def setFakeGpsTarget(self, inst_ns):
    new_pubs = dict()
    if inst_ns is not None:
      for cmd_name, (topic, msg_type) in FAKE_GPS_CMD_TOPICS.items():
        new_pubs[cmd_name] = nepi_sdk.create_publisher(inst_ns + '/' + topic, msg_type, queue_size = 1)
    old_pubs = self.fake_gps_pubs
    self.fake_gps_pubs = new_pubs
    self.fake_gps_target_ns = inst_ns
    for pub in old_pubs.values():
      if pub is not None:
        try:
          pub.unregister()
        except Exception:
          pass
    if inst_ns is None:
      self.msg_if.pub_warn("No Nav Sim GPS instance injects into " + self.mavlink_namespace.rstrip('/')
                           + " any more -- fake GPS commands are disabled")
    else:
      self.msg_if.pub_info("Sending fake GPS commands to Nav Sim GPS instance: " + inst_ns)
      self.reconcileFakeGpsApp()

  def pubFakeGps(self, cmd_name, msg):
    # No-op while no Nav Sim GPS instance is bound to this vehicle
    pub = self.fake_gps_pubs.get(cmd_name)
    if pub is not None:
      nepi_sdk.publish_pub(pub, msg)

  def pubFakeGpsLocation(self, cmd_name, geopoint_wgs84):
    # nav_sim hands its altitude to the FCU as GPS_INPUT.alt, which MAVLink
    # defines as MSL, so nav_sim's location is AMSL. This driver's geopoints
    # (navpose location, home, and the gotoLocation goal, whose navpose geoid
    # height is 0) carry WGS84 ellipsoid heights and are converted here. Sent
    # unconverted, the simulated fix would sit the geoid height (~20 m at the
    # bench) away from the FCU's own goal. A -999 hold altitude stays -999 so
    # nav_sim still holds that axis; converted, it would become a real target.
    geopoint_out = copy.deepcopy(geopoint_wgs84)
    if geopoint_wgs84.altitude != -999:
      geopoint_out = nepi_nav.convert_wgs84_to_amsl(geopoint_wgs84)
    self.pubFakeGps(cmd_name, geopoint_out)

  def reconcileFakeGpsApp(self):
    # A simulated vehicle brings its own GPS; a real airframe on this
    # deployment may not have one at all. The Nav Sim GPS instance cannot know
    # which it is looking at, so this driver tells it:
    #   SITL  -> disable the instance bound to this vehicle's mavros node (the
    #            sim has its own GPS, and a second one would keep the EKF from
    #            converging on a position)
    #   real  -> enable that instance, but only when this driver's fake_gps
    #            discovery option is on. A real airframe with its own GPS would
    #            otherwise get the same two-GPS conflict; with the option off,
    #            the instance is left untouched.
    # Runs once per binding (when an instance bound to this vehicle is first
    # found, or the binding moves), so it does not fight an operator who later
    # toggles the instance by hand in the Nav Sim RUI.
    if not self.is_sitl and not self.has_fake_gps:
      return
    enable_pub = self.fake_gps_pubs.get('enable')
    if enable_pub is None:
      return
    try:
      # A rospy publish issued before the subscriber connection completes is
      # dropped with no error, and the publisher was created just before this.
      self.waitForPubConnection(enable_pub)
      nepi_sdk.publish_pub(enable_pub, Bool(data = (not self.is_sitl)))
      if self.is_sitl:
        self.msg_if.pub_warn("Simulated vehicle (connection " + str(self.connection)
                             + ") -- disabling Nav Sim GPS instance " + str(self.fake_gps_target_ns)
                             + " so its injected GPS_INPUT cannot keep this vehicle's EKF"
                             + " from getting a position estimate")
      else:
        self.msg_if.pub_warn("Real vehicle (connection " + str(self.connection)
                             + ") with fake_gps on -- enabling Nav Sim GPS instance " + str(self.fake_gps_target_ns)
                             + ", which injects into " + str(self.mavlink_namespace))
    except Exception as e:
      # Never fatal: a convenience reconcile, not a requirement to operate
      self.msg_if.pub_warn("Could not reconcile the Nav Sim GPS instance state: " + str(e))

  def fakeGpsInputCb(self, gps_input_msg):
    # GPS_INPUT on this vehicle's mavros node comes from a Nav Sim GPS
    # instance, and PX4 drops it (see FAKE_GPS_HIL_MAX_RATE_HZ), so it is
    # resent as HilGPS in the units mavros expects (see HIL_GPS_MAX_SPEED_MPS):
    # eph/epv as the accuracies in whole metres (PX4 reads them as accuracies,
    # not as DOPs), velocities in whole m/s, course in whole degrees. Speeds
    # therefore reach PX4 rounded to 1 m/s, and a stationary fix reports course
    # 0, since mavros cannot pass HIL_GPS's "unknown". alt stays AMSL, which is
    # what PX4 reads HIL_GPS.alt as. Relayed only with the fake_gps option on:
    # a real airframe that has its own GPS and MAV_USEHILGPS left at 1 would
    # otherwise get a second, fake, GPS.
    if nepi_sdk.is_shutdown() or self.fake_gps_hil_pub is None:
      return
    if self.has_fake_gps == False:
      if self.fake_gps_relay_off_logged == False:
        self.fake_gps_relay_off_logged = True
        self.msg_if.pub_warn("A Nav Sim GPS_INPUT stream is arriving on " + self.mavlink_namespace
                             + " but the fake_gps driver option is off, so it is not relayed and"
                             + " PX4 (which ignores GPS_INPUT) gets no fix. Turn fake_gps on in"
                             + " Driver Options and restart the driver")
      return
    now = nepi_utils.get_time()
    if (now - self.fake_gps_hil_last_time) < (1.0 / FAKE_GPS_HIL_MAX_RATE_HZ):
      return
    self.fake_gps_hil_last_time = now
    flags = gps_input_msg.ignore_flags
    vn = 0.0 if (flags & GPS_INPUT_IGNORE_VEL_HORIZ) else gps_input_msg.vn
    ve = 0.0 if (flags & GPS_INPUT_IGNORE_VEL_HORIZ) else gps_input_msg.ve
    vd = 0.0 if (flags & GPS_INPUT_IGNORE_VEL_VERT) else gps_input_msg.vd
    speed_mps = math.hypot(vn, ve)
    def toAccuracyM(value):
      # At least 1 m, so a sub-metre accuracy is not reported as perfect
      return int(min(max(round(value), 1), HIL_GPS_MAX_ACCURACY_M))
    def toSpeedMps(value):
      return int(min(max(round(value), -HIL_GPS_MAX_SPEED_MPS), HIL_GPS_MAX_SPEED_MPS))
    hil_msg = HilGPS()
    hil_msg.header = gps_input_msg.header
    hil_msg.fix_type = gps_input_msg.fix_type
    hil_msg.geo.latitude = gps_input_msg.lat * 1e-7
    hil_msg.geo.longitude = gps_input_msg.lon * 1e-7
    hil_msg.geo.altitude = gps_input_msg.alt
    hil_msg.eph = (HIL_GPS_MAX_ACCURACY_M if (flags & GPS_INPUT_IGNORE_HORIZ_ACCURACY)
                   else toAccuracyM(gps_input_msg.horiz_accuracy))
    hil_msg.epv = (HIL_GPS_MAX_ACCURACY_M if (flags & GPS_INPUT_IGNORE_VERT_ACCURACY)
                   else toAccuracyM(gps_input_msg.vert_accuracy))
    hil_msg.vel = toSpeedMps(speed_mps)
    hil_msg.vn = toSpeedMps(vn)
    hil_msg.ve = toSpeedMps(ve)
    hil_msg.vd = toSpeedMps(vd)
    if speed_mps < 0.1:
      hil_msg.cog = 0
    else:
      hil_msg.cog = int(round(math.degrees(math.atan2(ve, vn)) % 360.0)) % 360
    hil_msg.satellites_visible = gps_input_msg.satellites_visible
    if nepi_sdk.publish_pub(self.fake_gps_hil_pub, hil_msg) and self.fake_gps_relay_started == False:
      self.fake_gps_relay_started = True
      self.msg_if.pub_info("Relaying Nav Sim GPS_INPUT to PX4 as HIL_GPS on "
                           + self.mavlink_namespace + "hil/gps")

  def checkHilGpsParamsCb(self, timer):
    # PX4 takes the relayed HIL_GPS only with MAV_USEHILGPS = 1 (read live, no
    # reboot) and from its own system ID (MAV_SYS_ID, fixed at PX4 boot;
    # mavros sends as its system_id param, default 1). This driver writes
    # neither: like GPS_TYPE 14 on an ArduPilot, accepting a fake GPS is the
    # airframe owner's setting, and it must go back to 0 before flying on a
    # real GPS. mavros answers ParamGet only once it has pulled the list.
    if nepi_sdk.is_shutdown():
      return
    values = dict()
    for param_id in ['MAV_USEHILGPS', 'MAV_SYS_ID']:
      response = nepi_sdk.call_service(self.param_get_client, ParamGetRequest(param_id = param_id), verbose = False)
      if response is None or response.success == False:
        break
      values[param_id] = response.value.integer
    if len(values) < 2:
      self.hil_gps_param_tries += 1
      if self.hil_gps_param_tries < self.HIL_GPS_PARAM_MAX_TRIES:
        nepi_sdk.start_timer_process(self.HIL_GPS_PARAM_RETRY_SEC, self.checkHilGpsParamsCb, oneshot = True)
      else:
        self.msg_if.pub_warn("Could not read MAV_USEHILGPS and MAV_SYS_ID from PX4 -- the fake GPS"
                             + " reaches PX4 only with MAV_USEHILGPS = 1")
      return
    mavros_sys_id = nepi_sdk.get_param(self.mavlink_namespace + 'system_id', 1)
    if values['MAV_USEHILGPS'] != 1:
      self.msg_if.pub_warn("PX4 MAV_USEHILGPS is " + str(values['MAV_USEHILGPS']) + ", so PX4 drops the"
                           + " fake GPS and gets no fix. Set MAV_USEHILGPS = 1 (no reboot needed) for"
                           + " bench tests, and back to 0 before flying on a real GPS")
    elif values['MAV_SYS_ID'] != mavros_sys_id:
      self.msg_if.pub_warn("PX4 MAV_SYS_ID is " + str(values['MAV_SYS_ID']) + " but mavros sends as system "
                           + str(mavros_sys_id) + ", and PX4 takes HIL_GPS only from its own system ID,"
                           + " so the fake GPS is dropped")
    else:
      self.msg_if.pub_info("PX4 MAV_USEHILGPS is 1 -- PX4 will use the relayed fake GPS")

  #######################
  # Mavlink Interface Methods

  ### Callback to get current state message
  def get_state_callback(self,mavlink_state_msg):
    self.mavlink_state = mavlink_state_msg
    # Update state value
    arm_val = mavlink_state_msg.armed
    if arm_val == True:
      self.state_ind=1
    else:
      self.state_ind=0
    self.state_current = self.RBX_STATES[self.state_ind]
    # Update mode value
    mode_val = mavlink_state_msg.mode
    mode_ind=-999
    for ind, mode in enumerate(self.RBX_MODES):
      if mode == mode_val:
        mode_ind=ind
    self.mode_ind=mode_ind
    if mode_ind >= 0 and mode_ind < len(self.RBX_MODES):
      self.mode_current = self.RBX_MODES[self.mode_ind]
    else:
      self.mode_current = "Undefined"
    # Arm transitions are handled here rather than in set_mavlink_arm_state,
    # so they are caught whatever caused them: this driver, an RC pilot, or
    # PX4 itself disarming after it lands.
    if self.armed_last is not None and arm_val != self.armed_last:
      if arm_val == True:
        self.captureHomeLocation()
      else:
        # A second flight must not read as airborne while still on the ground
        self.takeoff_complete = False
        self.takeoff_goal_alt = None
    self.armed_last = arm_val

  def captureHomeLocation(self):
    # Reset Home Location on Arming, from the navpose. Left unchanged when
    # there is no global position yet.
    if self.navpose_dict['has_location'] == True:
      home_loc = GeoPoint()
      home_loc.latitude = self.navpose_dict['latitude']
      home_loc.longitude = self.navpose_dict['longitude']
      home_loc.altitude = self.navpose_dict['altitude_m']
      self.home_location = home_loc
      self.msg_if.pub_info("Home location captured on arming: " + str([home_loc.latitude, home_loc.longitude, home_loc.altitude]))


  ### Function to set mavlink armed state
  def set_mavlink_arm_state(self,arm_value):
    # No client-side arm refusal: PX4 runs its own preflight checks and its
    # STATUSTEXT reason is already surfaced by get_statustext_callback.
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = arm_value
    self.msg_if.pub_info("Updating State to: " + str(arm_value))
    self.msg_if.pub_info("Waiting for armed value to set to " + str(arm_value))
    timeout_sec = self.rbx_if.status_msg.cmd_timeout
    check_interval_s = 0.25
    check_timer = 0
    # Same response-checking as set_mavlink_mode -- see its comment. mavros'
    # CommandBool replies success=False when the FCU rejects arming (a failed
    # preflight check being the usual reason).
    no_response_count = 0
    refused_count = 0
    while self.mavlink_state.armed != arm_value and check_timer < timeout_sec and not nepi_sdk.is_shutdown():
      response = nepi_sdk.call_service(self.arming_client, arm_cmd)
      if response is None:
        no_response_count += 1
        if no_response_count == 1:
          self.msg_if.pub_warn("arming service call to " + str(self.arming_client_name) +
                               " returned nothing -- the call itself is failing, not the FCU")
      elif getattr(response, 'success', True) == False:
        refused_count += 1
        if refused_count == 1:
          self.msg_if.pub_warn("FCU refused arming (success=False) -- usually a failed preflight check")
      time.sleep(check_interval_s)
      check_timer += check_interval_s
    if self.mavlink_state.armed != arm_value:
      action = "Arm" if arm_value == True else "Disarm"
      fail_msg = action + " command timed out after " + str(timeout_sec) + "s"
      if no_response_count > 0:
        fail_msg = fail_msg + "; " + str(no_response_count) + " arming calls returned nothing"
      if refused_count > 0:
        fail_msg = fail_msg + "; " + str(refused_count) + " refused by the FCU (preflight check?)"
      reason = self.get_recent_fcu_reason()
      if reason != "":
        fail_msg = fail_msg + " (FCU: " + reason + ")"
      self.msg_if.pub_warn(fail_msg)
      if self.rbx_if is not None:
        self.rbx_if.update_error_msg(fail_msg)
    self.msg_if.pub_info("Armed value set to " + str(arm_value))
    return self.mavlink_state.armed == arm_value


  ### Function to set mavlink mode
  def set_mavlink_mode(self,mode_new):
    new_mode = SetModeRequest()
    new_mode.custom_mode = mode_new
    self.msg_if.pub_info("Updating mode")
    self.msg_if.pub_info(mode_new)
    self.msg_if.pub_info("Waiting for mode to set to " + mode_new)
    timeout_sec = self.rbx_if.status_msg.cmd_timeout
    check_interval_s = 0.25
    check_timer = 0
    # The service RESPONSE is checked rather than discarded. mavros' SetMode
    # replies mode_sent=False when it refuses to forward the request at all,
    # and nepi_sdk.call_service returns None when the call itself failed.
    # Ignoring both makes a mode change that was never sent look exactly like
    # one the FCU is still working on. Logged once per distinct outcome
    # rather than every 0.25 s tick.
    no_response_count = 0
    refused_count = 0
    while self.mavlink_state.mode != mode_new and check_timer < timeout_sec and not nepi_sdk.is_shutdown():
      response = nepi_sdk.call_service(self.mode_client, new_mode)
      if response is None:
        no_response_count += 1
        if no_response_count == 1:
          self.msg_if.pub_warn("set_mode service call to " + str(self.mode_client_name) +
                               " returned nothing -- the service call itself is failing, "
                               "not the FCU (check the service exists and the request type matches)")
      elif getattr(response, 'mode_sent', True) == False:
        refused_count += 1
        if refused_count == 1:
          self.msg_if.pub_warn("mavros refused to send mode " + mode_new +
                               " (mode_sent=False) -- it is rejecting the request before the FCU sees it")
      time.sleep(check_interval_s)
      check_timer += check_interval_s
    if self.mavlink_state.mode == mode_new:
      self.msg_if.pub_info("Mode set to " + mode_new)
    else:
      # Include what the vehicle's mode ACTUALLY is, plus whether the calls
      # were even getting through.
      fail_msg = ("Setting mode to " + mode_new + " timed out after " + str(timeout_sec) +
                  "s (vehicle still reports " + str(self.mavlink_state.mode) + ")")
      if no_response_count > 0:
        fail_msg = fail_msg + "; " + str(no_response_count) + " set_mode calls returned nothing"
      if refused_count > 0:
        fail_msg = fail_msg + "; " + str(refused_count) + " refused by mavros (mode_sent=False)"
      reason = self.get_recent_fcu_reason()
      if reason != "":
        fail_msg = fail_msg + " (FCU: " + reason + ")"
      self.msg_if.pub_warn(fail_msg)
      if self.rbx_if is not None:
        self.rbx_if.update_error_msg(fail_msg)
    return self.mavlink_state.mode == mode_new



  ### Callback to get current mavlink battery message
  def get_mavlink_battery_callback(self,battery_msg):
    self.battery_percent = battery_msg.percentage


  #######################
  # Link Watchdog (SITL and SERIAL)

  def linkWatchdogCb(self, timer):
    # mavros keeps publishing State with connected False once the PX4
    # heartbeat stops. Only a continuous LINK_LOST_SEC of that counts.
    now = nepi_utils.get_time()
    connected = (self.mavlink_state is not None and self.mavlink_state.connected == True)
    if connected:
      self.link_lost_time = None
    elif self.link_lost_time is None:
      self.link_lost_time = now
    elif (now - self.link_lost_time) >= self.LINK_LOST_SEC and self.link_lost_signalled == False:
      self.link_lost_signalled = True
      self.msg_if.pub_warn("No PX4 " + str(self.connection) + " link on " + str(self.device_path) +
                           " for " + str(int(self.LINK_LOST_SEC)) +
                           " s -- shutting down so discovery can probe for it again")
      nepi_sdk.signal_shutdown(self.node_name + ": " + str(self.connection) + " link lost")


  #######################
  # Mavlink PX4 Interface Methods

  ### Function for switching to arm state
  global arm
  def arm(self):
    return self.set_mavlink_arm_state(True)

  ### Function for switching to disarm state
  global disarm
  def disarm(self):
    return self.set_mavlink_arm_state(False)

  ## Action Function for streaming, switching to OFFBOARD, arming and climbing
  global launch
  def launch(self):
    # Each stage that fails says which one it was, in the RBX status as well
    # as the log (see rbx_ardupilot_node.py's launch for why).
    self.msg_if.pub_info("Recieved Launch cmd")
    cmd_success = False

    if self.localPoseFresh() == False:
      self.reportLaunchFailure("no local position estimate")
      return False

    if self.waitForSetpointStream() == False:
      self.reportLaunchFailure("the offboard setpoint stream is not running -- not switching to OFFBOARD")
      return False

    cmd_success = self.setModeInd(self.RBX_MODE_FUNCTIONS.index("offboard"))
    if not cmd_success:
      self.reportLaunchFailure("could not switch to OFFBOARD mode -- not arming or taking off")
      return False

    cmd_success = self.setStateInd(self.RBX_STATE_FUNCTIONS.index("arm"))
    if not cmd_success:
      self.reportLaunchFailure("OFFBOARD mode was set but the vehicle would not ARM -- not taking off")
      return False

    cmd_success = self.takeoff_action()
    if not cmd_success:
      self.reportLaunchFailure("armed in OFFBOARD mode but the takeoff did not complete")
    return cmd_success

  def reportLaunchFailure(self, why):
    msg = "LAUNCH failed: " + why
    self.msg_if.pub_warn(msg)
    if self.rbx_if is not None:
      self.rbx_if.update_error_msg(msg)

  ## Function for sending takeoff command
  global takeoff
  def takeoff(self):
    if self.state_current != "ARM":
      fail_msg = "Ignoring Takeoff command as system is not Armed"
      self.msg_if.pub_warn(fail_msg)
      self.rbx_if.update_error_msg(fail_msg)
      return False
    if self.localPoseFresh() == False:
      fail_msg = "Ignoring Takeoff command: no local position estimate"
      self.msg_if.pub_warn(fail_msg)
      self.rbx_if.update_error_msg(fail_msg)
      return False
    if self.mode_current != "OFFBOARD":
      if self.setModeInd(self.RBX_MODE_FUNCTIONS.index("offboard")) == False:
        fail_msg = "Takeoff failed: could not switch to OFFBOARD mode"
        self.msg_if.pub_warn(fail_msg)
        self.rbx_if.update_error_msg(fail_msg)
        return False
    return self.takeoff_action()

  def takeoff_action(self):
    # Climbs with an offboard HOLD target straight above the current local
    # position -- PX4's own documented mavros offboard takeoff -- instead of
    # MAV_CMD_NAV_TAKEOFF, whose altitude PX4 reads as AMSL.
    self.rbx_if.update_prev_errors()
    self.rbx_if.update_current_errors( [0,0,0,0,0,0,0] )
    self.rbx_if.update_current_goals( [0,0,0,0,0,0,0] )
    cmd_success = False
    if self.state_current == "ARM":
      takeoff_height_m = float(nepi_controls.get_value(self.settings_dict,'takeoff_height_m'))
      self.rbx_if.update_current_goals( [None,None,takeoff_height_m,None,None,None,None] )
      [start_x_m, start_y_m, start_z_m, start_yaw_rad] = self.getCurrentPose()
      goal_alt = start_z_m + takeoff_height_m
      self.takeoff_goal_alt = goal_alt
      self.msg_if.pub_info("Climbing " + str(takeoff_height_m) + " meters on an offboard setpoint")
      self.setHoldTarget(start_x_m, start_y_m, goal_alt, start_yaw_rad)
      # The fake GPS climbs straight up by the same height with a RELATIVE ENU
      # move, which needs no altitude datum and no current location.
      self.pubFakeGps('goto_position', Point(x = 0.0, y = 0.0, z = takeoff_height_m))

      error_bound_m = self.rbx_if.status_msg.error_bounds.max_distance_error_m
      self.takeoff_error_bound_m = error_bound_m
      timeout_sec = self.rbx_if.status_msg.cmd_timeout
      check_interval_s = float(timeout_sec) / 100
      check_timer = 0
      left_offboard = False
      alt_error = (goal_alt - self.navpose_dict['z_m'])
      while (abs(alt_error) > error_bound_m and check_timer < timeout_sec and not nepi_sdk.is_shutdown()):
        if self.isOffboardArmed() == False:
          # An RC pilot or a failsafe took over; the climb target no longer applies
          left_offboard = True
          break
        self.rbx_if.update_current_errors( [None,None,alt_error,None,None,None,None] )
        time.sleep(check_interval_s)
        check_timer += check_interval_s
        alt_error = (goal_alt - self.navpose_dict['z_m'])
      # Success is decided by whether the vehicle actually REACHED the
      # altitude, not by whether it got there before the timeout (see
      # rbx_ardupilot_node.py's takeoff_action).
      reached_altitude = abs(alt_error) <= error_bound_m
      if reached_altitude:
        cmd_success = True
        self.takeoff_complete = True
        if check_timer >= timeout_sec:
          self.msg_if.pub_warn("Takeoff reached " + str(takeoff_height_m)
                               + " m but only as the " + str(timeout_sec)
                               + "s timeout expired (error " + str(alt_error)
                               + " m) -- treating as complete since the vehicle is"
                               + " at altitude. Raise this command's timeout if the"
                               + " simulator is running slower than realtime.")
        else:
          self.msg_if.pub_info("Takeoff action completed with error: " + str(alt_error) + " meters")
      else:
        self.takeoff_complete = False
        if left_offboard:
          fail_msg = ("Takeoff stopped with error " + str(alt_error) + " meters: the vehicle left OFFBOARD"
                      + " (now " + str(self.mode_current) + ", " + str(self.state_current) + ")")
        else:
          fail_msg = "Takeoff action timed out with error: " + str(alt_error) + " meters"
        self.msg_if.pub_warn(fail_msg)
        if self.rbx_if is not None:
          self.rbx_if.update_error_msg(fail_msg)
    else:
      fail_msg = "Ignoring Takeoff command as system is not Armed"
      self.msg_if.pub_warn(fail_msg)
      if self.rbx_if is not None:
        self.rbx_if.update_error_msg(fail_msg)
    return cmd_success

  ### Force-disarm: bypasses PX4's in-flight disarm interlock. A plain DISARM
  ### is correctly refused while airborne; this is the deliberate override,
  ### which cuts the motors wherever the vehicle is.
  global force_disarm
  def force_disarm(self):
    self.msg_if.pub_info("Received Force Disarm cmd")
    force_disarm_cmd = CommandLongRequest()
    force_disarm_cmd.broadcast = False
    force_disarm_cmd.command = self.MAV_CMD_COMPONENT_ARM_DISARM
    force_disarm_cmd.confirmation = 0
    force_disarm_cmd.param1 = 0.0                     # 0 = disarm
    force_disarm_cmd.param2 = self.MAV_CMD_FORCE_MAGIC # bypass in-flight safety interlock
    response = nepi_sdk.call_service(self.command_client, force_disarm_cmd)
    return (response is not None and response.success)

  ### Function for switching to MANUAL mode
  global manual
  def manual(self):
    """Switches PX4 to MANUAL mode; PX4 refuses it without RC."""
    return self.set_mavlink_mode('MANUAL')

  ### Function for switching to STABILIZED mode
  global stabilize
  def stabilize(self):
    cmd_success = self.set_mavlink_mode('STABILIZED')
    self.pubFakeGps('go_stop', Empty())
    return cmd_success

  ### Function for switching to ALTCTL mode
  global altctl
  def altctl(self):
    """Switches PX4 to ALTCTL (altitude control) mode; PX4 refuses it without RC."""
    return self.set_mavlink_mode('ALTCTL')

  ### Function for switching to POSCTL mode
  global posctl
  def posctl(self):
    """Switches PX4 to POSCTL (position control) mode; PX4 refuses it without RC."""
    return self.set_mavlink_mode('POSCTL')

  ### Function for switching to OFFBOARD mode
  global offboard
  def offboard(self):
    """Switches PX4 to OFFBOARD mode once the setpoint stream is running."""
    if self.waitForSetpointStream() == False:
      fail_msg = ("OFFBOARD not requested: no setpoint stream yet"
                  + " (it starts with the first local position estimate)")
      self.msg_if.pub_warn(fail_msg)
      self.rbx_if.update_error_msg(fail_msg)
      return False
    # Entering OFFBOARD holds the current position (see sendSetpointStreamCb),
    # so the fake GPS stops where it is too.
    cmd_success = self.set_mavlink_mode('OFFBOARD')
    self.pubFakeGps('go_stop', Empty())
    return cmd_success

  ### Function for switching to AUTO.TAKEOFF mode
  global auto_takeoff
  def auto_takeoff(self):
    """Switches PX4 to AUTO.TAKEOFF, which climbs to PX4's own MIS_TAKEOFF_ALT."""
    return self.set_mavlink_mode('AUTO.TAKEOFF')

  ### Function for switching to AUTO.LOITER mode
  global loiter
  def loiter(self):
    cmd_success = self.set_mavlink_mode('AUTO.LOITER')
    self.pubFakeGps('go_stop', Empty())
    return cmd_success

  ### Function for switching to AUTO.LAND mode
  global land
  def land(self):
    # PX4 disarms itself once it has landed, so disarmed is what success
    # means here.
    cmd_success = False
    if self.set_mavlink_mode('AUTO.LAND') == False:
      return False
    # Descend the fake fix at the current lat/lon to the home (takeoff)
    # altitude, i.e. the ground, rather than to 0, which is sea level. Before
    # any home exists (-999), 0.
    if self.navpose_dict['has_location'] == True:
      geo_point = GeoPoint()
      geo_point.latitude = self.navpose_dict['latitude']
      geo_point.longitude = self.navpose_dict['longitude']
      geo_point.altitude = 0.0
      if self.home_location.altitude != -999:
        geo_point.altitude = nepi_nav.convert_wgs84_to_amsl(self.home_location).altitude
      # Already AMSL, so sent without pubFakeGpsLocation's conversion
      self.pubFakeGps('goto_location', geo_point)
    self.msg_if.pub_info("Waiting for land process to complete and disarm")
    timeout_sec = self.rbx_if.status_msg.cmd_timeout
    check_interval_s = float(timeout_sec) / 100
    check_timer = 0
    while (self.state_current == "ARM" and check_timer < timeout_sec and not nepi_sdk.is_shutdown()):
      time.sleep(check_interval_s)
      check_timer += check_interval_s
    if self.state_current == "DISARM":
      self.msg_if.pub_info("Land process complete")
      cmd_success = True
    else:
      self.msg_if.pub_info("Land process timed-out")
    return cmd_success

  ### Function for sending go home command
  global rtl
  def rtl(self):
    # Done when PX4 disarms after landing at home, or when the vehicle has
    # stopped moving within max_distance_error_m of home (horizontally: PX4
    # may loiter above home at RTL_DESCEND_ALT). Stopped alone is not enough:
    # PX4's RTL pauses between phases, and a fake fix at about 1 m/s moves
    # less than the default 2 m bound per check. Movement is measured on the
    # local ENU position, in metres, against max_distance_error_m.
    cmd_success = False
    if self.set_mavlink_mode('AUTO.RTL') == False:
      return False
    # The fake fix goes straight to home and down to the home altitude, the
    # end state of PX4's default RTL and what rbx_ardupilot's RTL sends. PX4's
    # own climb to RTL_RETURN_ALT is not simulated, and on a bench PX4 does
    # not detect the landing, so it stays armed in AUTO.RTL.
    has_home = self.home_location.latitude != -999 and self.home_location.longitude != -999
    if has_home:
      self.pubFakeGpsLocation('goto_location', self.home_location)
    error_goal_m = self.rbx_if.status_msg.error_bounds.max_distance_error_m
    last_pos = self.getCurrentPose()[0:3]
    timeout_sec = self.rbx_if.status_msg.cmd_timeout
    check_interval_s = max(0.1, float(self.rbx_if.status_msg.error_bounds.min_stabilize_time_s))
    check_timer = 0
    stabilized_check = False
    disarmed_check = False
    while (stabilized_check is False and disarmed_check is False and check_timer < timeout_sec
           and not nepi_sdk.is_shutdown()):
      nepi_sdk.sleep(check_interval_s,100)
      check_timer += check_interval_s
      disarmed_check = (self.state_current == "DISARM")
      cur_pos = self.getCurrentPose()[0:3]
      max_distance_error_m = max([abs(cur_pos[ind] - last_pos[ind]) for ind in range(3)])
      stabilized_check = max_distance_error_m < error_goal_m
      last_pos = cur_pos
      if stabilized_check and has_home and self.navpose_dict['has_location'] == True:
        [east_m, north_m, up_m] = getGeoOffsetEnuM(self.navpose_dict['latitude'], self.navpose_dict['longitude'], 0.0,
                                                   self.home_location.latitude, self.home_location.longitude, 0.0)
        stabilized_check = math.hypot(east_m, north_m) <= error_goal_m
    if stabilized_check or disarmed_check:
      self.msg_if.pub_info("RTL process complete")
      cmd_success = True
    else:
      self.msg_if.pub_info("RTL process timed-out")
    return cmd_success

  ### Function for switching back to current mission
  global resume
  def resume(self):
    # Reset mode to last
    if self.mode_last not in self.RBX_MODES or self.mode_last == "RESUME":
      fail_msg = "Resume ignored: no earlier mode to return to"
      self.msg_if.pub_warn(fail_msg)
      self.rbx_if.update_error_msg(fail_msg)
      return False
    self.msg_if.pub_info("Switching mavlink mode from " + self.mode_current + " back to " + self.mode_last)
    return self.set_mavlink_mode(self.mode_last)


  ### Function for setting home location
  def set_home_location(self,geo_point):
    self.msg_if.pub_info("Sending mavlink set home command")
    cmd_home = CommandHomeRequest()
    cmd_home.current_gps = False
    cmd_home.latitude = geo_point.latitude
    cmd_home.longitude = geo_point.longitude
    cmd_home.altitude = geo_point.altitude
    response = nepi_sdk.call_service(self.set_home_client, cmd_home)
    if response is not None:
      self.home_location = geo_point
      if self.has_fake_gps:
        self.pubFakeGpsLocation('reset', geo_point)
      cmd_success = True
    else:
      cmd_success = False
    return cmd_success


  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    # Release any motor still under test. No mode change is commanded: what
    # PX4 does when the setpoint stream stops is its own configured offboard
    # loss behavior (COM_OF_LOSS_T, COM_OBL_RC_ACT). PX4 also ends any
    # actuator test within 3 s on its own if this call cannot get through.
    released_list = self.releaseAllMotors()
    if len(released_list) > 0:
      self.msg_if.pub_info("Released motors still under test: " + str(released_list))


#########################################
# Main
#########################################
if __name__ == '__main__':
  PX4Node()
