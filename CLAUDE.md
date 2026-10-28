# nepi_drivers — Developer Reference

## Purpose

`nepi_drivers` provides hardware abstraction for all physical devices supported by NEPI. Drivers are organized by device category (camera, navigation, pan-tilt, lighting, robot) and follow a three-tier pattern: a discovery script that detects hardware and launches nodes, a node script that registers the device with its NEPI interface class and handles ROS communication, and an optional driver script that handles raw hardware I/O. The `drivers_mgr` node in `nepi_engine` calls discovery functions periodically; everything else is self-contained here.

## Architecture

```
nepi_drivers/
├── idx_drivers/        # Image/camera drivers (GenICam, V4L2, ONVIF, ZED)
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

**IDX — Image/Camera (6 drivers):**
- `idx_genicam` — GenICam/GigE Vision cameras via Baumer Harvester library
- `idx_v4l2` — V4L2 USB cameras (scans `/dev/video*`, excludes ZED and virtual devices)
- `idx_onvif_generic` — Generic ONVIF IP cameras
- `idx_onvif_econroutecam` — EconRoute-specific ONVIF cameras
- `idx_zed` — Stereoscopic ZED cameras
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
- `rbx_px4` — PX4 autopilot multicopters (Pixhawk) via `mavros`, for outer-loop control by NEPI while PX4 runs the inner loops. Same discovery shape as `rbx_ardupilot` (discovery launches `mavros_node`, then the `PX4Node` RBX node). The `connection` option is `USB` (the Pixhawk's own USB port, a ttyACM device) or `SITL` (PX4 SITL over UDP 14540). Every command — goto position/location/pose/velocity, teleop, stop, takeoff — goes through one always-on offboard setpoint stream on `setpoint_raw/local`. TELEM UART and Ethernet links are not supported.

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
| IDX ONVIF | `requests`, XML/HTTP libraries |
| NPX Microstrain | Device present at `/dev/microstrain*` |
| PTX Sidus | Serial port present, correct baud rate |
| LSX Sealite | Serial port present, device at configured address |
| All serial | `pyserial`, `nepi_serial` SDK module |
| IDX RPi cam3 | `picamera2`, libcamera stack (`sudo apt install python3-picamera2`); Raspberry Pi 5 only |

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

**RBX PX4 (`rbx_px4`) — untested on hardware.** Verified only offline: the MAVLink heartbeat probe against pymavlink frames, and the node's frame, setpoint, teleop and motor-test logic against stubs. Not yet run under ROS, against a real Pixhawk or against PX4 SITL. Things to know before flying it:
- *USB match.* Discovery opens only ttyACM ports whose USB product string contains `PX4` and does not contain the bootloader token `BL` (e.g. `PX4 FMU v6X.x`, `PX4 KakuteH7`; never `PX4 BL FMU v5.x`), because Pololu Maestro and Teensy boards on these systems are ttyACM ports too. It accepts only a CRC-valid HEARTBEAT whose autopilot byte is `MAV_AUTOPILOT_PX4` (12). It listens 2 s first (`SYS_USB_AUTO` = 2, the current default: PX4 transmits first), then writes one GCS heartbeat and listens 3 s more (`SYS_USB_AUTO` = 1: PX4 waits for the host). A PX4 port that stays silent is logged once at info and re-probed every 15 s, not every cycle, because `drivers_mgr` runs every driver's `discoveryFunction` in turn on one thread. A PX4 on a TELEM UART is seen by `rbx_ardupilot`'s serial probe, which now leaves it alone (payload byte 5 = 12) — and `rbx_px4` does not pick it up either.
- *SITL port.* SITL discovery binds UDP 14540 (PX4 SITL's offboard-API link sends from 14580 to 14540), without `SO_REUSEADDR`, for 2 s, and builds `fcu_url` from the heartbeat's sender address. If another process holds 14540 (a GCS, another mavros) it warns once and skips. A SITL on another host must be started with this device's IP as its MAVLink target. The node shuts itself down after 15 s of `State.connected == False` so discovery purges and re-probes; USB relies on the `/dev` path purge.
- *Always-on stream.* A 30 Hz `PositionTarget` stream starts with the first `local_position/pose` and never stops while the node runs. Outside OFFBOARD (and while disarmed) it holds the current position, refreshed every tick, so any switch into OFFBOARD — from the RUI, an RC pilot or LAUNCH — holds where the vehicle is. A velocity command (teleop, 0.75 s expiry; goto_velocity, its duration) turns into a hold at the position of the moment it expires. LAUNCH and TAKEOFF climb with this stream (PX4's documented mavros offboard example), not `cmd/takeoff`, whose altitude PX4 reads as AMSL. Nothing else in the driver publishes setpoints.
- *PX4 parameters the airframe owner must set.* `COM_OF_LOSS_T` and `COM_OBL_RC_ACT` decide what PX4 does when the stream stops (node exit, mavros or link loss); the node does not command a mode on shutdown. `MPC_XY_VEL_MAX`, `MPC_Z_VEL_MAX_UP`/`_DN` and related limits cap what the driver's own `max_*` settings can reach — their defaults are well below interceptor speeds. Motor tests use `MAV_CMD_ACTUATOR_TEST`, which PX4 denies while armed or with the safety switch engaged, so the safety switch must be configured (or disabled) for bench tests.
- *Teleop signs.* The driver follows the `device_if_rbx.py` convention (y+ left, yaw rate+ counter-clockwise). The RUI's `TeleopKeymap.js` sends strafe-right as +y and turn-right as +yaw, so those two keys feel reversed until the keymap and the IF documentation are reconciled.

**No hardware-in-the-loop CI.** Driver code is not covered by automated tests that require physical hardware. The built-in simulators partially address this for NPX, but IDX, PTX, and LSX drivers have no equivalent.

## Decision Log

- 2026-03 — CLAUDE.md created — Initial developer reference, Claude Code authoring pass.
- 2026-07 — Corrected stale RBX content — `rbx_drivers/` is no longer empty; documented the `rbx_ardupilot` driver (mavros subprocess launch, connection-agnostic node), fixed the RBX interface class name to `RBXRobotIF`, and noted `scripts/fake_gps_node.py` is superseded by the `nepi_app_fake_gps` app.
- 2026-10 — `rbx_ardupilot` merged back from nepi_drones; simulator machinery gated on `is_sitl` — The drones overlay copies of `rbx_ardupilot_node.py`/`_discovery.py` (motor test, SITL discovery and its `SitlMavlinkRelay`, teleop velocity via `setpoint_velocity/cmd_vel_unstamped`, camera-bridge relays, RESET_SIM/FORCE_DISARM, Fake GPS reconcile, LAUNCH failure reporting, odom/yaw frame fixes) now live here, and `rbx_ardupilot_sitl_params.yaml` adds a second driver entry pinned to `connection: SITL` so a physical drone and its SITL twin can be discovered together. Three things were changed on the way in because the driver also flies real airframes: the four VM relay listeners (0.0.0.0:9032/9034/9035/9036) and the camera bridge threads start only when `is_sitl` (device_path starts `SITL`), so a field vehicle opens no simulator ports; RESET_SIM, which force-disarms after its Gazebo reset, is dropped from the setup actions of a non-SITL instance (Robot Link maps setup actions by name, so the shorter list is safe); and the Fake GPS reconcile only enables the app for a real vehicle whose own `fake_gps` option is on. The drones' goto_position Y reflection (y+ = right for this driver only) was left out because it inverts the RBX y+ = left convention for existing callers; the teleop Y negation was kept because teleop is new. `describeArmRefusalCause` uses the new `nepi_sdk.find_publishers()` instead of `rosgraph`. Discovery LD_PRELOADs aarch64 `libgomp.so.1` (only if present) when it spawns the RBX node — that, not the node's own import order, is what avoids the static-TLS crash when `cv2`/`nepi_img` load before open3d.
- 2026-09 — Converted all drivers to the new settings functions — Every driver family (idx, lsx, ptx, rbx, svx) now uses `initSettingsDict`/`refreshSettingsDict`/`getSettingsFunction`/`setSettingFunction`, with `idx_v4l2_node.py` as the reference. The old `capSettings`/`factorySettings`/`settingUpdateFunction` arguments had already been dropped from every `device_if_*` constructor, so every unconverted driver was raising `TypeError` at startup; `ptx_iqr` and `ptx_sidus_ss109` were additionally calling settings methods that did not exist on their own classes. See the Driver Settings Pattern section above.
- 2026-10 — Added `rbx_px4` (PX4 via mavros, USB and SITL) for a partner's high-speed drone interceptor; one offboard setpoint stream carries every command — PX4 runs the inner loops and NEPI streams setpoints as the outer loop, and PX4 enters and stays in OFFBOARD only while setpoints keep arriving faster than 2 Hz, so the stream runs for the life of the node and every goto, teleop, stop and takeoff call only changes its one target (HOLD position+yaw, or VELOCITY+yaw rate with an expiry). Outside OFFBOARD the target tracks the current pose, which is what makes a switch into OFFBOARD from any source safe. Takeoff is an offboard climb, not `MAV_CMD_NAV_TAKEOFF` (PX4 reads its altitude as AMSL; the ArduPilot driver's relative height would mean metres above sea level). `gotoLocation` is converted to a local ENU offset from the current lat/lon/alt instead of using `setpoint_position/global`, so there is still one setpoint channel and the ellipsoid/AMSL offset cancels; `gotoPose` applies yaw only (a station-keeping multicopter cannot hold commanded roll/pitch). Motor tests use `MAV_CMD_ACTUATOR_TEST` (310) — PX4's commander does not handle `MAV_CMD_DO_MOTOR_TEST` — resent at 1 Hz because PX4 caps a test at 3 s, and allowed only while disarmed. Teleop and goto_velocity follow ROS REP-103 (y+ left, yaw rate+ CCW) with vector-limited horizontal speed caps that are Settings, since an interceptor outer loop needs to raise them. Discovery claims a ttyACM port only on a `PX4` USB product string plus a CRC-valid PX4 heartbeat, because Maestro and Teensy boards are ttyACM too; `rbx_ardupilot`'s serial probe now reads the heartbeat payload and leaves autopilot 12 alone. Fixed relative to the reference on the way: `land()` succeeds on disarm (the reference's check was inverted), `takeoff_complete` is reset on every disarm (a second flight read as airborne on the ground), home is captured on any disarm-to-arm transition (including an RC arm), readiness checks use the mode name so an unlisted PX4 mode cannot raise IndexError, and goHome returns at the current altitude rather than flying to the ground-level home altitude. Untested on hardware and under ROS — see Known Constraints.
