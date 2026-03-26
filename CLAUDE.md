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
├── rbx_drivers/        # Robot drivers (placeholder — no drivers implemented)
├── scripts/
│   └── fake_gps_node.py   # Test/development GPS simulator node
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
| RBX | `RBXDeviceIF` |

The interface class handles all ROS publisher/subscriber/service creation. The node provides callbacks for hardware reads and commands. Nodes run until hardware disconnects or `drivers_mgr` terminates them.

**Retry and backoff:** Discovery tracks the last launch time per device path in `launch_time_dict`. A configurable `NODE_LOAD_TIME_SEC` backoff (typically 10 seconds) prevents rapid relaunch loops. A `dont_retry_list` permanently blacklists devices that fail in a non-recoverable way.

### Simulator support

Two driver categories include built-in simulators for development without hardware:
- **NPX NMEA UDP:** A `threading.Thread` TCP server that generates NMEA sentences (GGA, RMC, VTG, HDG) at a configurable rate. Enabled via `simulate_gps: true` in the driver options.
- **NPX HNav TCU:** A packet-level simulator that builds binary HNav wire-format packets (with CRC-16) and listens on a local TCP port. Enabled via the `simulate_*` options in the YAML.

When a simulator is running, the driver node connects to `localhost` on the configured port and behaves identically to the hardware case.

## Current Driver Inventory

**IDX — Image/Camera (5 drivers):**
- `idx_genicam` — GenICam/GigE Vision cameras via Baumer Harvester library
- `idx_v4l2` — V4L2 USB cameras (scans `/dev/video*`, excludes ZED and virtual devices)
- `idx_onvif_generic` — Generic ONVIF IP cameras
- `idx_onvif_econroutecam` — EconRoute-specific ONVIF cameras
- `idx_zed` — Stereoscopic ZED cameras

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

**RBX — Robots (0 drivers):** Directory exists as a placeholder. No implementations present.

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

## Known Constraints and Fragile Areas

**GenICam requires Baumer libraries.** The `idx_genicam` driver loads `.cti` producer files (`libbgapi2_usb.cti`, `libbgapi2_gige.cti`). These must be present on the system at the paths expected by Harvester. Missing files cause silent failures during discovery.

**Serial device paths are not stable across reboots.** A device at `/dev/ttyUSB0` on one boot may be `/dev/ttyUSB1` on the next. Discovery handles this by re-probing all available ports, but configured addresses and baud rates must match. The `dont_retry_list` can permanently exclude a port after a bad probe sequence — check this list if a known device stops appearing.

**NMEA and HNav simulators run on localhost TCP.** If another process is already bound to the simulator port, the simulator thread will fail silently. The driver node will then fail to connect.

**`rbx_drivers/` is empty.** The directory exists and `drivers_mgr` is aware of the RBX category, but no robot drivers are implemented. Adding an RBX driver requires creating all three files (discovery, node, params) and registering with `RBXDeviceIF`.

**ONVIF drivers use HTTP/XML.** Network timeouts during ONVIF device probing can cause discovery to block. The `nepi_app_onvif_mgr` app (in `nepi_apps`) handles ONVIF device management at a higher level; the ONVIF drivers here are the low-level node implementations.

**No hardware-in-the-loop CI.** Driver code is not covered by automated tests that require physical hardware. The built-in simulators partially address this for NPX, but IDX, PTX, and LSX drivers have no equivalent.

## Decision Log

- 2026-03 — CLAUDE.md created — Initial developer reference, Claude Code authoring pass.
