# nepi_drivers — Developer Reference

## Purpose

`nepi_drivers` provides hardware abstraction for all physical devices supported by NEPI. Drivers are organized by device category (camera, navigation, pan-tilt, lighting, robot) and follow a three-tier pattern: a discovery script that detects hardware and launches nodes, a node script that registers the device with its NEPI interface class and handles ROS communication, and an optional driver script that handles raw hardware I/O. The `drivers_mgr` node in `nepi_engine` calls discovery functions periodically; everything else is self-contained here.

## Architecture

```
nepi_drivers/
├── idx_drivers/        # Image/camera drivers (GenICam, V4L2, ONVIF, ZED via SDK, ZED without CUDA)
├── npx_drivers/        # Navigation/positioning drivers (NMEA, HNav, Microstrain)
├── ptx_drivers/        # Pan-tilt/actuator drivers (Sidus SS109, IQR, ONVIF PTZ)
├── lsx_drivers/        # Lighting drivers (Deepsea Sealite, Sidus SS182, AfTower)
├── rbx_drivers/        # Robot drivers (ArduPilot and PX4 autopilots via mavros, Gazebo-simulated robots)
├── scripts/
│   └── fake_gps_node.py   # Legacy per-device GPS simulator (superseded by the nepi_app_fake_gps app)
├── CMakeLists.txt
├── package.xml
└── setup.py
```

Each driver category directory holds one set of files per supported device, following this naming pattern:
- `{cat}_{device}_discovery.py` — hardware detection and node lifecycle
- `{cat}_{device}_node.py` — ROS node, registers with device interface class
- `{cat}_{device}_driver.py` — (optional) raw hardware I/O abstraction
- `{cat}_{device}_params.yaml` — driver metadata and configurable options

Where `{cat}` is the three-letter category prefix (idx, npx, ptx, lsx, rbx).

## How It Works

`drivers_mgr` (in `nepi_managers`) calls each discovery function on a 1-3 second polling interval, passing:
- `available_paths_list` — serial ports, USB paths, or network endpoints found on the system
- `active_paths_list` — paths that already have a running node
- `base_namespace` — the device's ROS namespace
- `drv_dict` — the parsed YAML from the driver's params file
- `retry_enabled` — whether to attempt relaunching failed nodes

**Discovery** probes hardware (serial handshake, TCP socket test, USB enumeration), then calls `nepi_sdk.launch_node()` to start a driver node when hardware is found. The driver path and device config are passed to the node via the ROS param server at `~drv_dict`. Discovery also monitors running nodes: if a node dies or hardware disconnects, it removes it from `active_paths_list` and allows re-discovery on the next cycle.

**Nodes** call `nepi_sdk.init_node()`, read `~drv_dict` from the param server, then instantiate the appropriate device interface class from `nepi_api`:

| Category | Interface class |
|---|---|
| IDX | `IDXDeviceIF` |
| NPX | `NPXDeviceIF` |
| PTX | `PTXActuatorIF` |
| LSX | `LSXDeviceIF` |
| RBX | `RBXRobotIF` |

The interface class handles all ROS publisher/subscriber/service creation. The node provides callbacks for hardware reads and commands. Nodes run until hardware disconnects or `drivers_mgr` terminates them.

**Retry and backoff:** Discovery tracks the last launch time per device path in `launch_time_dict`. A configurable `NODE_LOAD_TIME_SEC` backoff (typically 10 seconds) prevents rapid relaunch loops. A `dont_retry_list` permanently blacklists devices that fail in a non-recoverable way.

### Simulator support

Two driver categories include built-in simulators for development without hardware:
- **NPX NMEA UDP:** A `threading.Thread` TCP server that generates NMEA sentences (GGA, RMC, VTG, HDG) at a configurable rate. Enabled via `simulate_gps: true` in the driver options.
- **NPX HNav TCU:** A packet-level simulator that builds binary HNav wire-format packets (with CRC-16) and listens on a local TCP port. Enabled via the `simulate_*` options in the YAML.

When a simulator is running, the driver node connects to `localhost` on the configured port and behaves identically to the hardware case.

## Current Driver Inventory

**IDX — Image/Camera (7 drivers):**
- `idx_genicam` — GenICam/GigE Vision cameras via Baumer Harvester library
- `idx_v4l2` — V4L2 USB cameras (scans `/dev/video*`, excludes ZED and virtual devices)
- `idx_onvif_generic` — Generic ONVIF IP cameras
- `idx_onvif_econroutecam` — EconRoute-specific ONVIF cameras
- `idx_zed` — Stereoscopic ZED cameras via the ZED SDK (`pyzed`, needs CUDA); its discovery launches nothing when `pyzed` is not installed
- `idx_zed_hailo` — ZED, ZED Mini, ZED 2 and 2i on hosts without an NVIDIA GPU (first target: Raspberry Pi 5 with a Hailo-8). Reads the ZED as a plain UVC camera through `V4l2CamDriver` (YUYV side-by-side frames) and publishes three products: the left image (the IDX `color_image`), the right image (a second `ColorImageIF` at `<device>/idx/right/color_image`, `data_ref_description` `right_camera_lense`) and a `depth_map` in mm aligned with the rectified left image. Rectification uses the camera's own factory calibration file. Depth comes from OpenCV SGBM (`depth_backend` `cpu_sgbm`, the default) or StereoNet on the Hailo (`hailo`). Discovery claims a ZED only when `pyzed` is not installed, the mirror of `idx_zed`. Resolution and frame rate are discovery options (default VGA at 15 fps) because the calibration section and rectification maps depend on them. `idx_zed_hailo_driver.py` is ROS-free: calibration, serial lookup, rectification and both depth backends, testable offline
- `idx_rpi_cam3` — Raspberry Pi Camera Module 3 (IMX708) via MIPI CSI-2 on Raspberry Pi 5; uses libcamera/picamera2 stack

**NPX — Navigation/Positioning (3 drivers):**
- `npx_nmea_udp` — NMEA sentences over TCP/UDP (configurable host:port, built-in simulator)
- `npx_hnav_tcu` — HNav binary protocol over TCP (built-in packet simulator)
- `npx_microstrain` — Microstrain IMU/AHRS via `/dev/microstrain*` serial

**PTX — Pan-Tilt (3 drivers):**
- `ptx_sidus_ss109_serial` — Sidus SS109 serial pan-tilt, ±175° pan / ±75° tilt, modular addressing A-Z
- `ptx_iqr` — IQR pan-tilt via `/dev/iqr_pan_tilt` serial
- `ptx_onvif_generic` — ONVIF PTZ cameras

**LSX — Lighting (3 drivers):**
- `lsx_deepsea_sealite` — Deepsea Sealite LED lights, serial, address range 1–255
- `lsx_sidus_SS182` — Sidus SS182 strobe, serial
- `lsx_aftowerlight` — AfTower tower light system

**RBX — Robots (3 drivers):**
- `rbx_ardupilot` — ArduPilot autopilot systems (e.g. ArduCopter) via `mavros`. Unlike the other categories, the discovery script launches a `mavros_node` (as a `subprocess.Popen`, no `.launch` file) to carry the MAVLink link, then launches the `ArdupilotNode` RBX node, which attaches to that mavros namespace and registers with `RBXRobotIF`. The connection is selected by the `connection` discovery option: `SERIAL` (probes serial ports for a MAVLink heartbeat) is the production path; the code also implements `TCP`/`UDP` branches. See `rbx_drivers/SITL_IMPLEMENTATION_PLAN.md` (adding an ArduPilot SITL connection) and `rbx_drivers/DISCOVERY_EXPLAINED.md` (full discovery walkthrough).
- `rbx_gazebo` — Gazebo-simulated robots reached over a JSON-lines bridge; registers with `RBXRobotIF` as a `simulator` data source.
- `rbx_px4` — PX4 autopilot multicopters (Pixhawk) via `mavros`, for outer-loop control by NEPI while PX4 runs the inner loops. Same discovery shape as `rbx_ardupilot` (discovery launches `mavros_node`, then the `PX4Node` RBX node). The `connection` option is `USB` (the Pixhawk's own USB port, a ttyACM device), `SERIAL` (a Pixhawk TELEM port on a UART or USB-serial adapter) or `SITL` (PX4 SITL over UDP 14540). Every command — goto position/location/pose/velocity, teleop, stop, takeoff — goes through one always-on offboard setpoint stream on `setpoint_raw/local`. `gotoLocation` takes the ENU orientation 3.4 `RBXRobotIF` passes and uses its yaw as is. Each motor is its own `DeviceSVXIF` (`<device_name>_motor_<n>`) on the RUI Servo page, as in `rbx_ardupilot`. The motor count is read once at startup, from the factory `motor_count` (4), before `RBXRobotIF` builds the SettingsIF that restores saved settings, so a saved `motor_count` does not change the number of motor devices, even after a restart (`rbx_ardupilot` has the same ordering). Navpose is published by the driver's own NPX device (`<node>/npx/navpose`), which the node adds to its robot's navpose selector and selects as the RBX navpose source (see the 2026-11 own-navpose entry in the Decision Log). Robot commands also drive a Nav Sim fake GPS bound to this vehicle's mavros node, as in `rbx_ardupilot`; the `fake_gps` discovery option (default off) lets the driver switch that instance on and relay its GPS_INPUT to PX4 as HIL_GPS, which PX4 accepts only with `MAV_USEHILGPS` = 1. Ethernet links are not supported.

## ROS Interface

Driver nodes do not publish directly to fixed topic names. All topics are determined by the device interface class (`IDXDeviceIF`, `NPXDeviceIF`, etc.) and are rooted at the device's namespace as assigned by `drivers_mgr`. The interface class follows conventions established in `nepi_api`; refer to `nepi_api` source for the exact topic and service names published per device type.

The `drv_dict` parameter (passed at `~drv_dict`) contains:
- `DEVICE_DICT` — `device_name`, `device_path`, `serial_number`, `model`
- `DISCOVERY_DICT` — discovery options (baud rate, addresses, TCP endpoint, simulator flags)
- `SAVE_DATA` — data logging configuration

## Build and Dependencies

Built as part of the `nepi_engine_ws` catkin workspace. No standalone build.

Hardware-specific runtime dependencies:

| Category | Dependency |
|---|---|
| IDX GenICam | Baumer Harvester library (`harvesters`, `genicam`), Baumer GenTL producers (`.cti` files) |
| IDX ZED | ZED SDK |
| IDX ZED Hailo | OpenCV (`cv2` with `StereoSGBM`), numpy. Optional, for the `hailo` backend: `hailo_platform` (HailoRT 4.23 on the target) plus `stereonet.hef` from the Hailo Model Zoo v2.18.0 for Hailo-8, which the user installs in `/mnt/nepi_storage/ai_models/hailo_stereo/` (https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.18.0/hailo8/stereonet.hef). The camera's factory calibration file `SN<serial>.conf` in `/mnt/nepi_storage/user_cfg/cals/` |
| IDX ONVIF | `requests`, XML/HTTP libraries |
| NPX Microstrain | Device present at `/dev/microstrain*` |
| PTX Sidus | Serial port present, correct baud rate |
| LSX Sealite | Serial port present, device at configured address |
| All serial | `pyserial`, `nepi_serial` SDK module |
| IDX RPi cam3 | `picamera2`, libcamera stack (`sudo apt install python3-picamera2`); Raspberry Pi 5 only |
| NPX Blue Robotics Bar | `ms5837` Python module (Blue Robotics ms5837-python) plus `smbus2` or `smbus`; sensor on `/dev/i2c-<bus>` (the `i2c_bus` option), 3.3 V logic |

## Naming Conventions

Follows the NEPI convention established in `nepi_api`:
- **Public methods:** `snake_case` with docstrings
- **Private/internal methods:** `_camelCase`, no docstrings
- **`Cb` suffix:** ROS callback; rename requires auditing all external call sites

Discovery function signature is standardized:
```python
def discoveryFunction(available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled):
    ...
    return active_paths_list
```

## Driver Settings Pattern

All driver nodes expose device settings through four private methods and register
two of them with their device interface class. `idx_drivers/idx_v4l2_node.py` is
the reference implementation — copy its shape, adapted to the device's hardware.

```python
def initSettingsDict(self):        # build the settings once, at startup
def refreshSettingsDict(self):     # read live values (and bounds/options) back
def getSettingsFunction(self):     # return the current settings dict
def setSettingFunction(self, setting_name, setting_value):
```

Wire the last two into the device IF constructor:

```python
self.idx_if = IDXDeviceIF(device_info = self.device_info_dict,
                          getSettingsFunction = self.getSettingsFunction,
                          setSettingFunction = self.setSettingFunction,
                          ...)
```

All six device IF classes (`IDXDeviceIF`, `LSXDeviceIF`, `NPXDeviceIF`,
`PTXActuatorIF`, `RBXRobotIF`, `SVXActuatorIF`) accept exactly these two
arguments and build a `SettingsIF` only when both are non-`None`. A device with
no settings passes neither — that is the correct state, not an omission
(`npx_*`, `ptx_onvif_generic`, `lsx_aftowerlight`).

**Contract.** `getSettingsFunction()` takes no arguments and returns a
`nepi_controls` controls dict. `setSettingFunction(name, value)` returns
`[success, msg, settings_dict]` — all three, since `SettingsIF` replaces its own
dict with the third element. It is also called as a bare statement during
`SettingsIF.init()`, so it must tolerate its return being discarded.

**The settings dict is a controls dict.** Build the plain init dict, then hand it
to `nepi_controls.create_controls_dict()`. Each init entry is keyed by the
setting name and carries:

| key | applies to |
|---|---|
| `type` | required; must be in `nepi_controls.CONTROL_TYPES` |
| `default` | required; typed to match `type` |
| `bounds` | `Int`, `Float`, `FloatSlider(s)` |
| `options` | `Menu`, `Selection`, `Selections` |

Read and write values through `nepi_controls.get_value()` /
`set_control_value()` / `set_control_bounds()` / `set_control_options()` — never
by indexing the control dict directly. This applies to a node's own internal
reads too (see `rbx_ardupilot_node.py`, which reads `motor_count` and the
takeoff parameters out of its settings dict).

**Two traps when converting an old driver:**

- `'Discrete'` is *not* a `CONTROL_TYPE`. `create_controls_dict()` wraps every
  entry in a bare `except: pass`, so a `'Discrete'` setting is silently dropped
  and simply never appears in the RUI. A named option list is a `'Selection'`.
- The retired cap-settings form carried an `Int`/`Float` control's min and max in
  an `'options'` pair. Those are `'bounds'` now; `'options'` on a numeric control
  is ignored.

### Retired pattern

These no longer exist and no device IF accepts them. Any driver still using them
raises `TypeError` at construction:

```
getCapSettings()  getFactorySettings()  getSettings()  setSetting()  settingUpdateFunction()
capSettings=      factorySettings=      settingUpdateFunction=
nepi_controls.get_data_from_setting()   nepi_controls.check_valid_setting()
```

`device_if_ptx.py` still accepts a `getCapSettingsFunction` constructor argument,
but it is inert — it is never forwarded to `SettingsIF`, which has no such
parameter. Capability data (type, bounds, options) rides in the controls dict
itself. Do not pass it.

## Known Constraints and Fragile Areas

**GenICam requires Baumer libraries.** The `idx_genicam` driver loads `.cti` producer files (`libbgapi2_usb.cti`, `libbgapi2_gige.cti`). These must be present on the system at the paths expected by Harvester. Missing files cause silent failures during discovery.

**Serial device paths are not stable across reboots.** A device at `/dev/ttyUSB0` on one boot may be `/dev/ttyUSB1` on the next. Discovery handles this by re-probing all available ports, but configured addresses and baud rates must match. The `dont_retry_list` can permanently exclude a port after a bad probe sequence — check this list if a known device stops appearing.

**NMEA and HNav simulators run on localhost TCP.** If another process is already bound to the simulator port, the simulator thread will fail silently. The driver node will then fail to connect.

**RBX ArduPilot discovery launches `mavros` as a subprocess.** Unlike other categories, `rbx_ardupilot_discovery.py` does not only launch a node — it also spawns a `mavros_node` (via `subprocess.Popen` with `_fcu_url:=...`) to carry MAVLink, then launches `ArdupilotNode`. There are no `.launch` files; the `fcu_url` connection string is built in the per-type `launch*DeviceNode` helpers, and the shared `launchDeviceNode` writes a `DEVICE_DICT` param the node reads. The RBX node is connection-agnostic — it needs no change to switch transports. Only `SERIAL` is exposed in the params `connection` options today even though `TCP`/`UDP` branches exist; adding an ArduPilot SITL link is therefore mostly a new discovery branch plus a params-YAML option (see `rbx_drivers/SITL_IMPLEMENTATION_PLAN.md`). The APM configs it `rosparam load`s (`apm_pluginlists.yaml`, `apm_config.yaml`) live only at the installed path `/opt/nepi/nepi_engine/share/mavros/launch/`, not in this checkout.

**ONVIF drivers use HTTP/XML.** Network timeouts during ONVIF device probing can cause discovery to block. The `nepi_app_onvif_mgr` app (in `nepi_apps`) handles ONVIF device management at a higher level; the ONVIF drivers here are the low-level node implementations.

**RPi cam3 discovery requires libcamera-hello.** The `idx_rpi_cam3` discovery subprocess calls `libcamera-hello --list-cameras` to enumerate CSI cameras. This binary is only present on Raspberry Pi OS; the discovery node will find no cameras and remain idle on Jetson or x86 build hosts. The `device_path` for RPi cam3 is a camera index string (e.g. `'0'`), not a `/dev/` path — this is unique to this driver family.

**RBX PX4 (`rbx_px4`) — untested on hardware.** Verified only offline: the MAVLink heartbeat probe against pymavlink frames, and the node's frame, setpoint, teleop and motor-test logic against stubs. The 3.4 port was checked statically only (ast parse, constructor keywords against the workspace `nepi_api`). Not yet run under ROS, against a real Pixhawk or against PX4 SITL. Things to know before flying it:
- *USB match.* Discovery opens only ttyACM ports whose USB product string contains `PX4` and does not contain the bootloader token `BL` (e.g. `PX4 FMU v6X.x`, `PX4 KakuteH7`; never `PX4 BL FMU v5.x`), because Pololu Maestro and Teensy boards on these systems are ttyACM ports too. It accepts only a CRC-valid HEARTBEAT whose autopilot byte is `MAV_AUTOPILOT_PX4` (12). It listens 2 s first (`SYS_USB_AUTO` = 2, the current default: PX4 transmits first), then writes one GCS heartbeat and listens 3 s more (`SYS_USB_AUTO` = 1: PX4 waits for the host). A PX4 port that stays silent is logged once at info and re-probed every 15 s, not every cycle, because `drivers_mgr` runs every driver's `discoveryFunction` in turn on one thread. A PX4 on a TELEM UART is seen by `rbx_ardupilot`'s serial probe, which leaves it alone (payload byte 5 = 12) for `rbx_px4`'s `SERIAL` connection.
- *SERIAL match.* Probes every port `nepi_serial.get_serial_ports_list()` returns except ttyACM, at 921600 then 57600, listening 1.5 s at each and never writing — these ports belong to lights, pan-tilts and GPS units too. Opens with pyserial `exclusive=True` (a flock), so a port another process holds under flock is skipped rather than read from. A port with no PX4 heartbeat is logged once at info and re-probed every 30 s. PX4 side: TELEM2 is the companion port (`SER_TEL2_BAUD` 921600, `MAV_1_MODE` Onboard) but its MAVLink instance is off until `MAV_1_CONFIG` = TELEM 2; TELEM1 carries MAVLink at 57600 by default with `MAV_0_RATE` capped at 1200 B/s for a radio, so it connects but streams slowly. Wire TX, RX and GND only. Checked against PX4 v1.17.0.
- *SITL port.* SITL discovery binds UDP 14540 (PX4 SITL's offboard-API link sends from 14580 to 14540), without `SO_REUSEADDR`, for 2 s, and builds `fcu_url` from the heartbeat's sender address. If another process holds 14540 (a GCS, another mavros) it warns once and skips. A SITL on another host must be started with this device's IP as its MAVLink target. The node shuts itself down after 15 s of `State.connected == False` so discovery purges and re-probes; it does the same on a `SERIAL` link, because a UART path never disappears when the Pixhawk powers off. USB relies on the `/dev` path purge.
- *Always-on stream.* A 30 Hz `PositionTarget` stream starts with the first `local_position/pose` and never stops while the node runs. Outside OFFBOARD (and while disarmed) it holds the current position, refreshed every tick, so any switch into OFFBOARD — from the RUI, an RC pilot or LAUNCH — holds where the vehicle is. A velocity command (teleop, 0.75 s expiry; goto_velocity, its duration) turns into a hold at the position of the moment it expires. LAUNCH and TAKEOFF climb with this stream (PX4's documented mavros offboard example), not `cmd/takeoff`, whose altitude PX4 reads as AMSL. Nothing else in the driver publishes setpoints.
- *PX4 parameters the airframe owner must set.* `COM_OF_LOSS_T` and `COM_OBL_RC_ACT` decide what PX4 does when the stream stops (node exit, mavros or link loss); the node does not command a mode on shutdown. `MPC_XY_VEL_MAX`, `MPC_Z_VEL_MAX_UP`/`_DN` and related limits cap what the driver's own `max_*` settings can reach — their defaults are well below interceptor speeds. Motor tests are run per motor from the RUI Servo page (one `DeviceSVXIF` per motor) and still send `MAV_CMD_ACTUATOR_TEST` underneath, which PX4 denies while armed or with the safety switch engaged, so the safety switch must be configured (or disabled) for bench tests. A reverse command stops the motor: an actuator test cannot spin backwards.
- *Navpose needs a global fix.* The NPX device's `getNavPoseCb` returns nothing until both the local pose and a `global_position/global` fix have arrived, because `NPXDeviceIF` fixes its `has_*` capabilities from the first dict it receives. Without a GPS fix the NPX publishes no navpose, so the RBX navpose source stays silent, and the node keeps checking every 2 s until the NPX topic appears before selecting it. The driver's own setpoint logic reads its navpose dict directly, but `RBXRobotIF`'s goto and takeoff completion checks read the selected source, so with no fix those report timeouts.
- *Fake GPS.* Commands are mirrored to the Nav Sim GPS instance whose `selected_mavros_node` is this vehicle's mavros node: goto position (the offset as received) and takeoff (a relative climb) as `goto_position`; goto location, Go Home (through `gotoLocation`), RTL (straight to home and down to the home altitude, as `rbx_ardupilot` sends it) and land (to the home altitude) as `goto_location`; Stop, STABILIZED, AUTO.LOITER and OFFBOARD as `go_stop`; set home as `reset_location`. Every geopoint is converted WGS84 to AMSL first, because nav_sim's altitude is GPS_INPUT's MSL and this driver's navpose geoid height is 0. Teleop and goto_velocity have no nav_sim equivalent and do not move the fix. PX4's own RTL climb to `RTL_RETURN_ALT` is not simulated. On a bench PX4 does not detect the landing, so after AUTO.RTL or AUTO.LAND it stays armed until FORCE_DISARM. RTL completes when PX4 disarms, or when the vehicle has stopped within `max_distance_error_m` of home horizontally. Stopped alone is not enough: the fix moves at about 1 m/s, less than the default 2 m bound per check, so the old stopped-only test reported RTL complete within a second, wherever the vehicle was. In AUTO modes mavros logs "PositionTargetGlobal failed because no origin" about ten times a second. That is mavros failing to convert PX4's POSITION_TARGET_GLOBAL_INT, because it never received GPS_GLOBAL_ORIGIN. It is harmless, since nothing here uses `setpoint_raw/target_global`.
- *PX4 ignores GPS_INPUT; the fix goes in as HIL_GPS.* No PX4 release handles GPS_INPUT (checked `mavlink_receiver.cpp` in 1.14.3, 1.16.0, 1.17.0 and main). On the 2026-11-23 bench, nav_sim's GPS_INPUT reached mavros at 35 Hz while PX4 v1.16.0 reported "No satellites". PX4's only MAVLink GPS input is HIL_GPS, accepted only while `MAV_USEHILGPS` = 1 (read live, no reboot) and only from a sender whose system ID equals PX4's `MAV_SYS_ID` (fixed at PX4 boot; mavros sends as its `system_id` param, default 1). With `fake_gps` on (not on SITL), the node subscribes to its own mavros `gps_input/gps_input` and republishes each message on `hil/gps` (mavros_msgs/HilGPS), capped at 10 Hz. The conversion uses the units mavros expects. Its hil plugin (1.20.1, `hil.cpp` `gps_cb`) multiplies HilGPS `eph`, `epv`, `vel`, `vn`, `ve`, `vd` and `cog` by 100 itself, so those integer fields carry whole metres, m/s and degrees. eph/epv are the accuracies (PX4 reads them as accuracies, not DOPs), at least 1 m. Speeds reach PX4 rounded to 1 m/s. A stationary fix reports course 0, because mavros cannot pass HIL_GPS's "unknown". Altitude stays AMSL. The first bench relay sent centimetres, and PX4 saw a 100 m accuracy, flagged a GPS glitch and never fused it. At startup it reads `MAV_USEHILGPS` and `MAV_SYS_ID` through mavros `param/get` and warns if PX4 would drop the fix. The node does not write either parameter: like `GPS_TYPE` 14 on an ArduPilot, accepting a fake GPS is the airframe owner's setting, and it must go back to 0 before flying on a real GPS. With `fake_gps` off, an arriving GPS_INPUT stream is logged once and not relayed. The bench board's `EKF2_HGT_REF` is 1 (GPS), so local z should follow the injected altitude. Still unverified on hardware: that PX4 reaches a global position from the relayed fix, and how EKF2 treats a fix that moves while the IMU sits still on a bench.
- *Teleop signs.* The driver follows the `device_if_rbx.py` convention (y+ left, yaw rate+ counter-clockwise). The RUI's `TeleopKeymap.js` sends strafe-right as +y and turn-right as +yaw, so those two keys feel reversed until the keymap and the IF documentation are reconciled.

**ZED without CUDA (`idx_zed_hailo`) — untested with a ZED attached.** Verified offline only: calibration parsing against a synthetic file built from a real `SN27083277.conf`, rectification (`P2[0,3]/P2[0,0]` = −Baseline), cpu_sgbm depth on a synthetic 20 px shift (1800 mm exactly), serial lookup against a fake sysfs tree, and the Hailo pre/post-processing and fallbacks against stubs. Not yet run under ROS, and the USB mode table, the HID serial lookup, the signs of `TY`/`TZ` and the rotation vector, and both backends' depth on real imagery are unverified. Things to know before relying on it:
- *Calibration is required for depth.* Lookup order at node start: `/mnt/nepi_storage/user_cfg/cals/SN<serial>.conf` (the same folder `idx_zed` backs the SDK's files up to, so a camera that has run on a NEPI Jetson already has its file there); the only `SN*.conf` in that folder when the serial cannot be read (logged as a warning); one HTTPS GET to `https://calib.stereolabs.com/?SN=<serial>` (10 s timeout, never retried) when `download_calibration` is on, which sends the camera serial to Stereolabs and saves the file only if it parses. Otherwise the node publishes the raw left and right halves and no depth, and the depth warning names the file and folder expected. The serial comes from sysfs: a USB device with vendor id `2b03` and an all-digit serial on the same hub as the video device (the ZED's sensor HID interface).
- *The Hailo device is exclusive.* A process that opens it, such as a running NEPI Hailo detector, keeps every other process out unless HailoRT's multi-process service is in use. The `hailo` backend therefore opens the device only while it is selected and releases it when `cpu_sgbm` is selected or the node shuts down. If loading fails (no `hailo_platform`, no `.hef` in `hailo_stereo/`, device busy, HEF for another chip or a newer HailoRT, or a model that is not two inputs and one output) or inference raises, the node falls back to `cpu_sgbm` once, logs the cause, and does not retry until `hailo` is selected again or the node restarts. The `depth_backend` setting keeps showing the requested backend after a fallback. To retry from the RUI, select `cpu_sgbm` and then `hailo`.
- *The HEF must match the installed HailoRT.* The v2.18.0 `stereonet.hef` records Dataflow Compiler 3.33.1, the same as the `common_objects_yolov8m_hailo8_640.hef` NEPI ships, and loads on HailoRT 4.23. The v2.19.0 and v2.20.0 copies record 3.34.0, and a HEF built by a newer compiler than HailoRT supports fails at load, which the fallback catches.
- *Why `cpu_sgbm` is the default.* Measured on the Pi 5 in the NEPI container (cv2 4.13, 4 threads, `MODE_SGBM_3WAY`, block 5): 42 ms per VGA pair with 64 disparities, 133 ms with 128, and 240 ms at HD720 with 64. `hailortcli run` gives StereoNet 2.56 fps (about 390 ms) on the same Pi, whose Hailo PCIe link trained at Gen2 x1 although Gen3 was requested. CPU SGBM at VGA is about ten times faster, and it leaves the Hailo free for NEPI's detector. CPU depth is practical only at VGA, which is why VGA is the default resolution.
- *StereoNet is unconfirmed on ZED imagery.* It was trained on KITTI (outdoor driving, wider baseline than a ZED). The left image feeds the first input by sorted vstream name (`input_layer1`), and the output is read as disparity in model-input pixels (0 to 136) and scaled by `rectified_width / input_width`. Neither the input order nor the output scale has been confirmed. The load log names each input and which one is left, and the first inference after each load logs the output's min, median and max before scaling. Garbage depth means toggle `hailo_swap_inputs`; depth off by a constant factor means the scale reading is wrong.
- *SGBM tuning is unmeasured.* `uniquenessRatio` 10, `speckleWindowSize` 100, `speckleRange` 2 and `disp12MaxDiff` 1 are OpenCV-sample values, not tuned on ZED imagery. Both images are padded on the left by `sgbm_num_disparities` columns before matching, so the left edge of the map is kept.
- *The right image is not saved by save-data and carries no transform or navpose.* `SaveDataIF` keys files by data-product name, and the right image's product is also `color_image`, so passing the device's `SaveDataIF` would mix right frames into the left image's saves. The device transform describes the left lens.

**No hardware-in-the-loop CI.** Driver code is not covered by automated tests that require physical hardware. The built-in simulators partially address this for NPX, but IDX, PTX, and LSX drivers have no equivalent.

## Decision Log

- 2026-03 — CLAUDE.md created — Initial developer reference, Claude Code authoring pass.
- 2026-07 — Corrected stale RBX content — `rbx_drivers/` is no longer empty; documented the `rbx_ardupilot` driver (mavros subprocess launch, connection-agnostic node), fixed the RBX interface class name to `RBXRobotIF`, and noted `scripts/fake_gps_node.py` is superseded by the `nepi_app_fake_gps` app.
- 2026-10 — `rbx_ardupilot` merged back from nepi_drones; simulator machinery gated on `is_sitl` — The drones overlay copies of `rbx_ardupilot_node.py`/`_discovery.py` (motor test, SITL discovery and its `SitlMavlinkRelay`, teleop velocity via `setpoint_velocity/cmd_vel_unstamped`, camera-bridge relays, RESET_SIM/FORCE_DISARM, Fake GPS reconcile, LAUNCH failure reporting, odom/yaw frame fixes) now live here, and `rbx_ardupilot_sitl_params.yaml` adds a second driver entry pinned to `connection: SITL` so a physical drone and its SITL twin can be discovered together. Three things were changed on the way in because the driver also flies real airframes: the four VM relay listeners (0.0.0.0:9032/9034/9035/9036) and the camera bridge threads start only when `is_sitl` (device_path starts `SITL`), so a field vehicle opens no simulator ports; RESET_SIM, which force-disarms after its Gazebo reset, is dropped from the setup actions of a non-SITL instance (Robot Link maps setup actions by name, so the shorter list is safe); and the Fake GPS reconcile only enables the app for a real vehicle whose own `fake_gps` option is on. The drones' goto_position Y reflection (y+ = right for this driver only) was left out because it inverts the RBX y+ = left convention for existing callers; the teleop Y negation was kept because teleop is new. `describeArmRefusalCause` uses the new `nepi_sdk.find_publishers()` instead of `rosgraph`. Discovery LD_PRELOADs aarch64 `libgomp.so.1` (only if present) when it spawns the RBX node — that, not the node's own import order, is what avoids the static-TLS crash when `cv2`/`nepi_img` load before open3d.
- 2026-09 — Converted all drivers to the new settings functions — Every driver family (idx, lsx, ptx, rbx, svx) now uses `initSettingsDict`/`refreshSettingsDict`/`getSettingsFunction`/`setSettingFunction`, with `idx_v4l2_node.py` as the reference. The old `capSettings`/`factorySettings`/`settingUpdateFunction` arguments had already been dropped from every `device_if_*` constructor, so every unconverted driver was raising `TypeError` at startup; `ptx_iqr` and `ptx_sidus_ss109` were additionally calling settings methods that did not exist on their own classes. See the Driver Settings Pattern section above.
- 2026-10 — Added `rbx_px4` (PX4 via mavros, USB and SITL) for a partner's high-speed drone interceptor; one offboard setpoint stream carries every command — PX4 runs the inner loops and NEPI streams setpoints as the outer loop, and PX4 enters and stays in OFFBOARD only while setpoints keep arriving faster than 2 Hz, so the stream runs for the life of the node and every goto, teleop, stop and takeoff call only changes its one target (HOLD position+yaw, or VELOCITY+yaw rate with an expiry). Outside OFFBOARD the target tracks the current pose, which is what makes a switch into OFFBOARD from any source safe. Takeoff is an offboard climb, not `MAV_CMD_NAV_TAKEOFF` (PX4 reads its altitude as AMSL; the ArduPilot driver's relative height would mean metres above sea level). `gotoLocation` is converted to a local ENU offset from the current lat/lon/alt instead of using `setpoint_position/global`, so there is still one setpoint channel and the ellipsoid/AMSL offset cancels; `gotoPose` applies yaw only (a station-keeping multicopter cannot hold commanded roll/pitch). Motor tests use `MAV_CMD_ACTUATOR_TEST` (310) — PX4's commander does not handle `MAV_CMD_DO_MOTOR_TEST` — resent at 1 Hz because PX4 caps a test at 3 s, and allowed only while disarmed. Teleop and goto_velocity follow ROS REP-103 (y+ left, yaw rate+ CCW) with vector-limited horizontal speed caps that are Settings, since an interceptor outer loop needs to raise them. Discovery claims a ttyACM port only on a `PX4` USB product string plus a CRC-valid PX4 heartbeat, because Maestro and Teensy boards are ttyACM too; `rbx_ardupilot`'s serial probe now reads the heartbeat payload and leaves autopilot 12 alone. Fixed relative to the reference on the way: `land()` succeeds on disarm (the reference's check was inverted), `takeoff_complete` is reset on every disarm (a second flight read as airborne on the ground), home is captured on any disarm-to-arm transition (including an RC arm), readiness checks use the mode name so an unlisted PX4 mode cannot raise IndexError, and goHome returns at the current altitude rather than flying to the ground-level home altitude. Untested on hardware and under ROS — see Known Constraints.
- 2026-10 — `rbx_px4` gained a `SERIAL` connection (Pixhawk TELEM port) — The interceptor team chose PX4 on a Holybro Pixhawk 6C Mini on the flying airframe, and PX4 documents flying with USB connected as fine for R&D but not for production vehicles (`CBRK_USB_CHK` defaults to bypassing its USB arming check), so the companion link needed a TELEM option. USB stays the default for bench and SIH work. The serial probe is listen-only and flock-exclusive because, unlike the USB probe's `PX4` product-string filter, nothing identifies a Pixhawk on a plain UART before its heartbeat, so every non-ttyACM port is a candidate. The SITL link watchdog now covers `SERIAL` too (renamed `linkWatchdogCb`/`LINK_LOST_SEC`, internal to `rbx_px4_node.py`). Offline-tested only (stubbed pyserial with pymavlink frames).
- 2026-11 — `rbx_px4` ported to the 3.4 API, following `rbx_ardupilot_node.py` — `RBXRobotIF` no longer takes `axisControls`, the two motor-ratio callbacks, `getNavPoseCb` or `navpose_update_rate`, so the node raised `TypeError` at construction. Axes are now a `caps_dict` passed as `axisCapabilities`. Each motor is a `DeviceSVXIF` on the Servo page whose helpers call the unchanged `setMotorControlRatio`, so the actuator test, its 1 Hz refresh, `motor_test_timeout_s` release and armed refusal are as before. Navpose comes from the node's own `NPXDeviceIF`, selected as the RBX navpose source, with `getNavPoseCb` returning `None` until both a local pose and a global fix exist. `gotoLocation` no longer converts its yaw from NED, because 3.4 passes ENU and converting an already-ENU yaw again commanded the wrong heading. Static checks only; never run under ROS.
- 2026-11 — Autopilot robots select their own NPX navpose, bypassing the NavPose Manager default — `RBXRobotIF` takes the `current_*` state that its goto and takeoff completion checks compare against from the robot's selected navpose source (`device_if_rbx.py` `_updateCurrentNavpose`). nepi_engine `2be5ec8` (2026-11-11) made every device's `ConnectNavPoseIF` offer only NavPose Manager frames (filter `['navposes']`) and auto-select the first, `base_frame`. That left `rbx_ardupilot`'s and `rbx_px4`'s own-navpose selection unable to work three ways: their NPX topic was filtered out, the auto-selected frame was read as an operator's pick, and they read the topic from `npx_if.navpose_if`, which is now the node-level `<node>/navpose` the robot republishes and excludes. The 2026-11-10 ArduPilot fake-GPS bench test passed only with the robot reading its own NPX. Routing the autopilot through a NavPose Manager frame was rejected because goto and takeoff completion would then depend on NavPose page configuration and on the frame carrying x/y/z position. Both drivers now take the topic from `npx_if.navpose_pub_if` once the NPX creates it, add it to their own selector's filter list (replaced by a copy, because the default list is one module-level object shared by every `ConnectNavPoseIF` in the process; an empty list is left alone), and replace the selection unless an operator's choice is SAVED. Auto-select never persists and `set_selected_topic` always does, so the saved `selected_topic` param tells the two apart; a saved topic inside the node's own namespace counts as a stale name for the NPX. The 60-try limit was dropped because the NPX appears only after a GPS fix. Offline-tested against stubs only. The engine's `NPXDeviceIF` status still reports `navpose_topic` as the node-level topic, not `<node>/npx/navpose`.
- 2026-11 — `rbx_px4` gained the Nav Sim fake GPS path from `rbx_ardupilot` — so commanded bench tests move the FCU's position on a PX4 too. The binding, enable/disable reconcile and AMSL conversion are `rbx_ardupilot`'s. Two differences: the `fake_gps` option defaults off, because nav_sim auto-binds its GPS instance to the first mavros node it finds, and a default-on option would enable a second GPS on a real PX4 airframe that has one; and Go Home needs no separate command, because RBXRobotIF resolves it through `gotoLocation`, which already sends `goto_location`. Offline-tested against stubs only; never run against nav_sim.
- 2026-11 — `rbx_px4` relays Nav Sim's GPS_INPUT to PX4 as HIL_GPS — The first bench run against nav_sim showed PX4 v1.16.0 discarding GPS_INPUT outright. PX4 has no handler for it in any release, so the earlier port could never have given a PX4 a fix. The relay lives in the driver rather than in nav_sim, because the driver is what knows its FCU is a PX4. nav_sim stays autopilot-agnostic, and ArduPilot keeps receiving GPS_INPUT, which its `GPS_TYPE` 14 backend reads. nav_sim's GPS_INPUT still reaches PX4 through mavros alongside the relay, and PX4 drops it, which does no harm. The relay is gated on `fake_gps`, and acceptance is gated on `MAV_USEHILGPS`, so a real airframe gets a second, fake GPS only if both NEPI's option and the FCU parameter were set deliberately. The driver checks the parameter but never writes it. In the same pass, `sendSetpointStreamCb` now returns once shutdown starts and publishes through `nepi_sdk.publish_pub`, because rospy closes the publisher before the 30 Hz timer stops and each driver disable logged a `publish() to a closed topic` traceback. Offline-tested against stubs (`fakeGpsInputCb` unit conversion, rate cap, gates and ignore flags; `checkHilGpsParamsCb` retry and warnings), then on the bench: PX4 fused the relayed fix, and the robot selected its own NPX navpose at the Nav Sim location. The first bench arm, mode and LAUNCH commands then raised `AttributeError: 'RBXRobotIF' object has no attribute 'rbx_info'`. The 3.4 port had missed eight reads of the removed `rbx_info` in the arm, mode, takeoff and goto waits, because its static checks covered only constructor keywords. They now read `rbx_if.status_msg`, which `node_if.py` rebuilds every status cycle with the `cmd_timeout` and error-bound params, as `rbx_ardupilot` already does.
- 2026-11 — Added `idx_zed_hailo`, a ZED driver for hosts without CUDA — The ZED SDK (`pyzed`) needs CUDA, so `idx_zed` cannot run on a Raspberry Pi 5, and its discovery relaunched a node that died at `import pyzed.sl` every cycle. Over USB the ZED is a plain UVC camera that sends one side-by-side YUYV frame, so the new driver reuses `V4l2CamDriver` (loaded with a second `importDriverClass` call and passed into `ZedHailoStereoDriver`) instead of copying it. The two discoveries split ZED cameras on one test, made once at start: `pyzed` findable (`importlib.util.find_spec`, or a `pyzed` folder in `/home/nepi/.local/lib/python3.8/site-packages`, where `idx_zed_node.py` adds it to `sys.path`) means `idx_zed` claims them; otherwise `idx_zed_hailo` does. A bare `find_spec` was not enough, because a Jetson that installed pyzed there for the node would have had its ZED dropped by `idx_zed`. One node is one IDX device, and `SUPPORTED_DATA_PRODUCTS` has no right-image product, so the right eye is a second `ColorImageIF` with its own node_if (see the top-level 2026-07 registry-key entry), driven by a loop modeled on `IDXDeviceIF`'s color loop that pulls only while `needs_data_check()` is True. Each frame is split and rectified once and cached by timestamp, so the left image, right image and depth of one frame share one pair, and depth pixel (u,v) is left pixel (u,v). Calibration uses the ZED factory file and never a fabricated one, because a wrong rectification produces plausible-looking wrong depth. `cpu_sgbm` is the default depth backend, not `hailo`, because it measured about ten times faster at VGA on the target and keeps the Hailo free for detection. All backend failures fall back to it once instead of retrying per frame. `idx_zed_hailo_driver.py` is ROS-free and reports log lines through a queue (`get_messages`) the node drains, so its first-inference and fallback messages can be tested offline. Offline-tested only; see Known Constraints.
