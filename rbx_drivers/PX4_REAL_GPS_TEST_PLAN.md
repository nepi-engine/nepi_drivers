# rbx_px4: real GPS bench test plan

Written 2026-11-23, after the fake-GPS bench run passed. In that run, LAUNCH, Goto Position, Stop, Go Home, AUTO.RTL and AUTO.LAND all worked against Nav Sim's fake fix. This plan checks the same driver with a real GPS plugged into the Pixhawk. Takeoff and gotos are not part of it.

**Hardware:** Holybro Pixhawk 6C Mini running PX4 v1.16.0, connected over USB to the NEPI Jetson (device1, 192.168.179.103). The GPS module plugs into the Pixhawk's GPS1 port.

## Board state when this was written

Read through mavros `param/get` on 2026-11-23:

| Parameter | Value | Meaning |
|---|---|---|
| `GPS_1_CONFIG` | 201 | GPS 1 port |
| `GPS_1_PROTOCOL` | 1 | u-blox |
| `SER_GPS1_BAUD` | 0 | auto |
| `SENS_GPS_MASK` | 7 | blending on |
| `EKF2_GPS_CTRL` | 7 | GPS fusion on |
| `EKF2_HGT_REF` | 1 | GPS height |

A standard u-blox GPS on GPS1 therefore needs no PX4 GPS setup. Two settings do need attention:

- **`MAV_USEHILGPS` = 1.** It was set for the fake-GPS test, and must be 0 for a real GPS.
- **Only the internal compass is calibrated.** `CAL_MAG0_ID` = 396321 is the board's IST8310 at I2C 0x0C on bus 4, and `CAL_MAG1_ID` = 0. The GPS module's compass will appear as a new, uncalibrated sensor, and PX4 will refuse to arm until it is calibrated.

Other settings, for reference: `COM_ARM_WO_GPS` = 1, `COM_RC_IN_MODE` = 3 (arming without RC worked), and `SYS_AUTOSTART` = 4001 (generic quadcopter).

## Phase 1: Turn the fake GPS off (RUI)

1. Drivers page → PX4 Autopilot → Driver Options → `fake_gps` off → Save. Then turn the driver off and on: the node reads the option only at launch.
2. Apps → NavPose Sim → disable it.

Otherwise PX4 blends the fake fix with the real one (`SENS_GPS_MASK` 7).

## Phase 2: Connect and calibrate (laptop + QGroundControl)

QGroundControl cannot share the Pixhawk's USB port with the Jetson's mavros, so do this phase from a laptop.

1. Unplug the Pixhawk from the Jetson and plug the GPS into **GPS1**.
2. Connect the Pixhawk's USB to the laptop and open QGroundControl. The GPS LED should light. If it does not, USB may not supply enough power; use the power module or a battery.
3. Parameters → `MAV_USEHILGPS` = 0.
4. Sensors → **Compass**. Two compasses should be listed. Set the external one's orientation to match how the GPS is mounted, then calibrate.
5. Sensors → **Level Horizon**.
6. Reboot the Pixhawk. This also clears PX4's EKF origin, which is otherwise anchored at the fake-GPS location for the rest of the boot.

## Phase 3: Get a fix (needs open sky)

1. Put the Pixhawk and GPS where the antenna can see open sky. Indoors, or beside a window, it will not reach the accuracy PX4 requires (`EKF2_REQ_EPH` 3 m). Use a USB extension from the Jetson, or take the setup outside.
2. Plug the Pixhawk back into the Jetson. Discovery finds it again by itself; the port may change between `ttyACM0` and `ttyACM1`.
3. Allow 1 to 5 minutes for a cold first fix.

What to confirm (see the device-side checks below):

- a PX4 3D fix with 10 or more satellites and horizontal accuracy under 3 m;
- PX4 reporting a valid absolute horizontal position (`pos_horiz_abs_status_flag`), with `gps_glitch_status_flag` False;
- "Selected own navpose source: /nepi/device1/px4_ttyacmN/npx/navpose" in the driver log, about 10 to 20 s after PX4 has a position;
- no "Relaying Nav Sim GPS_INPUT" and no "MAV_USEHILGPS" lines in the driver log, which confirms the fake GPS is off.

## Phase 4: Check the readings

| Check | How | Pass |
|---|---|---|
| Position | Compare the Robots page NavPose latitude and longitude with a phone map pin. | Within about 5 m. |
| Altitude | Compare the NavPose altitude with a phone. NEPI shows height above the WGS84 ellipsoid, so expect sea-level height plus the geoid height, about −20 m here. The bench run showed 5 m above sea level as −14.9 m. | Within about 10 m. |
| Heading | Point the Pixhawk's arrow north using a phone compass, then turn it 90° to the right. | Heading reads about 0°, then about 90°, each within about 10°. |
| Stability | Leave it for 5 minutes. | The fix and satellite count hold, and the position wanders no more than a few metres. |
| Home capture | With props off: Set Mode → OFFBOARD, then Set State → ARM. Look for "Home location captured on arming: [lat, lon, alt]" in the log, then disarm. | Home matches the Phase 4 position. |

## Phase 5: GPS loss and recovery

Unplug the GPS cable for 30 s, plug it back in, and keep the driver log.

- PX4 should lose its fix and stop reporting a global position within seconds.
- Watch what NEPI's navpose does. It will probably keep publishing the last position with fresh timestamps, because 3.4's combined `NavPose.timestamp` is the publish time. That is a known engine issue (workspace `CLAUDE.md`, 2026-11 NavPose timing entry), and nothing downstream can tell the data has frozen. Record exactly what happens.
- After reconnecting, confirm the position returns without restarting the driver.

## Optional Phase 6: Walk test

If the Jetson can come along, or the USB cable is long enough, carry the setup 20 m due east. X should rise by about 20 and Y should stay near 0. NavPose X and Y are world East and North.

## Not covered by this plan

Takeoff and gotos: with the board sitting still, a real GPS never shows it arriving, so those commands would just time out. They need a flight test, with props, an RC safety pilot, and PX4's offboard-loss failsafe parameters (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`) set first. That needs its own plan.

## Going back to fake-GPS bench testing

Unplug the real GPS, set `MAV_USEHILGPS` = 1, enable the NavPose Sim app, and turn `fake_gps` back on. Then restart the driver. With both GPS sources connected, PX4 blends them.

## Device-side checks

Run these inside the container (`ssh -p 2222 nepi@192.168.179.103`) after sourcing `/opt/ros/noetic/setup.bash` and `/opt/nepi/nepi_engine/setup.bash`. Set `M` to the live mavros namespace (`mavlink_px4_ttyACM0` or `mavlink_px4_ttyACM1`) and `N` to the matching lowercase node namespace (`px4_ttyacm0` or `px4_ttyacm1`).

```bash
M=/nepi/device1/mavlink_px4_ttyACM0; N=/nepi/device1/px4_ttyacm0
rostopic echo -n1 $M/gpsstatus/gps1/raw | grep -E "fix_type|satellites_visible|h_acc|v_acc"
rostopic echo -n1 $M/global_position/global | grep -E "latitude|longitude|altitude"
rostopic echo -n1 $M/estimator_status | grep -E "pos_horiz_abs|gps_glitch"
rostopic echo -n1 $M/global_position/compass_hdg
rostopic echo -n1 $N/connect_navpose/status | grep -E "^selected_topic|^connected_topic"
rosservice call $M/param/get "param_id: 'MAV_USEHILGPS'"
grep -h -E "Selected own navpose|Relaying Nav Sim|MAV_USEHILGPS|Home location" \
  /mnt/nepi_storage/logs/ros_log/nepi_device1_px4_ttyACM*.log | tail
```

`h_acc` and `v_acc` are in millimetres. During the fake-GPS run, mavros logged "PositionTargetGlobal failed because no origin" about ten times a second in AUTO modes. It is harmless.
