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

# NPX (navpose) node for a Blue Robotics Bar02 or Bar30 depth sensor (TE
# MS5837) wired directly to a host I2C bus, read with Blue Robotics'
# ms5837-python library (Python module ms5837). Launched by
# npx_bluerobotics_bar_discovery.py once a probe of the configured bus passes.
#
# The library does the I2C transfers and the conversion math. This node takes
# depth from the library's pressure and the configured surface pressure and
# fluid density, never from the library's depth(), which assumes a fixed
# 101300 Pa surface.
#
# WHAT IT PUBLISHES: depth through the NPXDeviceIF (has_depth only), and the
# compensated pressure and temperature as sensor_msgs/FluidPressure (pascals)
# and sensor_msgs/Temperature (deg C) on <node>/pressure and
# <node>/temperature, because the NavPose dict has no field for either.
#
# ONE THREAD ON THE BUS: the library has no locking, and each read() blocks
# while the sensor converts. The sensor is opened and checked in the
# constructor, before the poll timer starts; after that only the poll timer
# reads it, and i2c_lock keeps cleanup from closing the bus mid-read.
#
# STALE DATA: until the first good reading the NavPose callback returns None,
# so NPXDeviceIF waits to create its NavPose publishers. After that a lost
# sensor keeps its last reading and timestamp, with has_depth False. Returning
# None there would make NPXDeviceIF publish a blank NavPose, which reads as
# depth 0 -- the surface -- the worst wrong value a depth reading can take.
# After LINK_LOST_SEC with no good reading the node exits, and discovery
# probes the bus again and relaunches it.

import copy
import math
import threading

from sensor_msgs.msg import FluidPressure, Temperature

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from nepi_api.device_if_npx import NPXDeviceIF
from nepi_api.messages_if import MsgIF

try:
    import ms5837
    MS5837_IMPORT_ERROR = None
except Exception as e:
    ms5837 = None
    MS5837_IMPORT_ERROR = str(e)

PKG_NAME = 'NPX_BLUEROBOTICS_BAR'
FILE_TYPE = 'NODE'
DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

# The sensor's fixed I2C address. ms5837-python hard-codes it; kept here for
# messages only.
MS5837_I2C_ADDRESS = 0x76

# Standard gravity, as Blue Robotics' libraries use (Bar doc, Depth equation)
GRAVITY_M_S2 = 9.80665

# Bar model -> its MS5837 variant's conversion constants (Bar doc, First-order
# calculation). The shifts are the powers of two in the datasheet formulas:
#   OFF  = C2 * 2^off_c2  + (C4 * dT) / 2^off_dt
#   SENS = C1 * 2^sens_c1 + (C3 * dT) / 2^sens_dt
#   P    = (D1 * SENS / 2^21 - OFF) / 2^p_shift, in 1/p_per_mbar mbar
# max_mbar is the top of the model's range (Bar doc, At a Glance). chip names
# the library constant (ms5837.MODEL_02BA, ms5837.MODEL_30BA). The keys are
# the depth_model option values; the discovery holds the same list.
BAR_MODELS = {
    'BlueRobotics Bar02 (MS5837-02BA)': dict(chip = '02BA', off_c2 = 17, off_dt = 6, sens_c1 = 16, sens_dt = 7,
                                             p_shift = 15, p_per_mbar = 100, max_mbar = 2000.0),
    'BlueRobotics Bar30 (MS5837-30BA)': dict(chip = '30BA', off_c2 = 16, off_dt = 7, sens_c1 = 15, sens_dt = 8,
                                             p_shift = 13, p_per_mbar = 10, max_mbar = 30000.0),
}
# The library's OSR_256 to OSR_8192
DEPTH_OSR_VALUES = [256, 512, 1024, 2048, 4096, 8192]
I2C_BUS_RANGE = (0, 15)
FLUID_DENSITY_KG_M3_RANGE = (950.0, 1100.0)
SURFACE_PRESSURE_MBAR_RANGE = (700.0, 1100.0)
# The startup model check accepts a first reading this close to
# surface_pressure_mbar, about 3 m of water. The wrong model reads about 20
# times too high or too low, far outside it.
MODEL_CHECK_WINDOW_MBAR = 300.0

# ms5837-python's read() sleeps 2.5 us per unit of OSR for each of its two
# conversions: about 5 ms at OSR 1024 and 41 ms at OSR 8192.
LIBRARY_CONVERSION_SEC_PER_OSR = 2.5e-6
# The poll period is at least this (20 Hz) and at least POLL_READ_MARGIN times
# the blocking read time, leaving room for the I2C transfers and the rest of
# the callback: 20 Hz up to OSR 4096, about 10 Hz at 8192.
MIN_POLL_INTERVAL_SEC = 0.05
POLL_READ_MARGIN = 2.5


def readSec(osr):
    # The time one library read() blocks in its two conversion waits
    return 2.0 * LIBRARY_CONVERSION_SEC_PER_OSR * osr


def pollIntervalSec(osr):
    return max(MIN_POLL_INTERVAL_SEC, POLL_READ_MARGIN * readSec(osr))


def barFirstOrder(model, prom, d1, d2):
    # The datasheet's first-order values in its integer units. // floors, as
    # the datasheet's worked examples do; C's truncating / misses them by one.
    c = BAR_MODELS[model]
    dT = d2 - prom[5] * 2**8
    TEMP = 2000 + (dT * prom[6]) // 2**23
    OFF = prom[2] * 2**c['off_c2'] + (prom[4] * dT) // 2**c['off_dt']
    SENS = prom[1] * 2**c['sens_c1'] + (prom[3] * dT) // 2**c['sens_dt']
    P = (d1 * SENS // 2**21 - OFF) // 2**c['p_shift']
    return dict(dT = dT, TEMP = TEMP, OFF = OFF, SENS = SENS, P = P)


def barCompensate(model, prom, d1, d2):
    # [pressure_mbar, temperature_c] after the second-order compensation
    # (Bar doc, Second-order compensation). Used only by the startup model
    # check; published values come from the library.
    c = BAR_MODELS[model]
    first = barFirstOrder(model, prom, d1, d2)
    dT = first['dT']
    TEMP = first['TEMP']
    if c['chip'] == '30BA':
        if TEMP < 2000:
            Ti = (3 * dT * dT) // 2**33
            OFFi = (3 * (TEMP - 2000)**2) // 2**1
            SENSi = (5 * (TEMP - 2000)**2) // 2**3
            if TEMP < -1500:
                OFFi = OFFi + 7 * (TEMP + 1500)**2
                SENSi = SENSi + 4 * (TEMP + 1500)**2
        else:
            Ti = (2 * dT * dT) // 2**37
            OFFi = ((TEMP - 2000)**2) // 2**4
            SENSi = 0
    else:
        if TEMP < 2000:
            Ti = (11 * dT * dT) // 2**35
            OFFi = (31 * (TEMP - 2000)**2) // 2**3
            SENSi = (63 * (TEMP - 2000)**2) // 2**5
        else:
            Ti = 0
            OFFi = 0
            SENSi = 0
    OFF2 = first['OFF'] - OFFi
    SENS2 = first['SENS'] - SENSi
    pressure_mbar = ((d1 * SENS2 // 2**21 - OFF2) // 2**c['p_shift']) / float(c['p_per_mbar'])
    temperature_c = (TEMP - Ti) / 100.0
    return [pressure_mbar, temperature_c]


def depthFromPressure(pressure_mbar, surface_pressure_mbar, fluid_density_kg_m3):
    # Blue Robotics' depth equation, with the configured surface pressure in
    # place of the libraries' fixed 1013 mbar (Bar doc, Depth equation).
    return (pressure_mbar - surface_pressure_mbar) * 100.0 / (fluid_density_kg_m3 * GRAVITY_M_S2)


def modelCheckText(model, prom, d1, d2, surface_pressure_mbar, fluid_density_kg_m3):
    # A warning when a first reading suggests depth_model is wrong, else None.
    # In air or at the surface only the right model's math reads near
    # surface_pressure_mbar (Bar doc, Plausibility check). A sensor that
    # starts underwater gets a warning too, so the text says what it saw and
    # nothing is refused.
    pressure = barCompensate(model, prom, d1, d2)[0]
    if abs(pressure - surface_pressure_mbar) <= MODEL_CHECK_WINDOW_MBAR:
        return None
    for other in sorted(BAR_MODELS.keys()):
        if other == model:
            continue
        other_pressure = barCompensate(other, prom, d1, d2)[0]
        if abs(other_pressure - surface_pressure_mbar) <= MODEL_CHECK_WINDOW_MBAR:
            return ("First depth sensor reading is %.1f mbar as a %s, but %.1f mbar as a %s, near "
                    "surface_pressure_mbar %.1f. If the sensor is in air or at the surface, set depth_model "
                    "to %s" % (pressure, model, other_pressure, other, surface_pressure_mbar, other))
    depth = depthFromPressure(pressure, surface_pressure_mbar, fluid_density_kg_m3)
    return ("First depth sensor reading is %.1f mbar, %.2f m deep. If the sensor is in air or at the "
            "surface, check depth_model and surface_pressure_mbar" % (pressure, depth))


def rawReading(sensor):
    # [prom words 0-6, d1, d2] from the last library read, or None if this
    # ms5837-python version does not keep them as _C, _D1 and _D2. init()'s
    # CRC check masks word 0 in place; only words 1-6 are used here.
    try:
        prom = [int(w) for w in sensor._C[:7]]
        d1 = int(sensor._D1)
        d2 = int(sensor._D2)
    except Exception:
        return None
    if len(prom) != 7:
        return None
    return [prom, d1, d2]


class BlueRoboticsBarNode:

    # A reading older than this is stale: has_depth goes False.
    STALE_SEC = 1.0
    # No good reading for this long is a lost sensor: the node exits so
    # discovery can probe the bus again and relaunch it.
    LINK_LOST_SEC = 5.0
    # depth_m_per_sec is the depth change over this much of the latest
    # samples; two neighboring samples differ mostly by sensor noise.
    RATE_WINDOW_SEC = 0.5
    # Standing problems (failed reads, over-range) are repeated this rarely.
    WARN_THROTTLE_SEC = 10.0

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")

    serial_num = "Unknown"
    hw_version = "Unknown"
    sw_version = "Unknown"
    npx_if = None

    # Sensor link
    sensor = None
    connected = False

    drv_dict = dict()


    ################################################
    def __init__(self):
        ####  NODE Initialization ####
        nepi_sdk.init_node(name= DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF()
        self.msg_if.pub_info("Starting Node Initialization Processes")

        # i2c_lock: the sensor object, used by the poll timer and closed by
        # cleanup. navpose_lock: the reading, written by the poll timer and
        # read by NPXDeviceIF's timer.
        self.i2c_lock = threading.Lock()
        self.navpose_lock = threading.Lock()

        ##############################
        # Get required drv driver dict info
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
        try:
            device_dict = self.drv_dict['DEVICE_DICT']
            self.device_name = device_dict['device_name']
            self.device_path = device_dict['device_path']
            self.i2c_bus = device_dict['i2c_bus']
            self.config = device_dict['depth_config']
            self.resolveConfig()
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        self.initReadingState()

        ################################################
        self.msg_if.pub_info("Connecting to Bar depth sensor at I2C address 0x%02X on %s" %
                             (MS5837_I2C_ADDRESS, self.device_path))
        self.connected = self.connect()
        if self.connected == False:
            self.msg_if.pub_info("Shutting down node, unable to start the Bar depth sensor")
            nepi_sdk.signal_shutdown("Unable to connect to depth sensor")
            return
        self.msg_if.pub_info("... Connected!")

        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version

        self.pressure_pub = nepi_sdk.create_publisher(nepi_sdk.create_namespace(self.node_namespace, 'pressure'),
                                                      FluidPressure, queue_size = 1)
        self.temperature_pub = nepi_sdk.create_publisher(nepi_sdk.create_namespace(self.node_namespace, 'temperature'),
                                                         Temperature, queue_size = 1)

        self.msg_if.pub_info("Launching NEPI NPX interface...")
        self.npx_if = NPXDeviceIF(device_info = self.device_info_dict,
                                  data_source_description = 'depth_sensor',
                                  data_ref_description = 'sensor_center',
                                  frame_depth = 'DEPTH',
                                  getNavPoseCb = self.getNavPoseCb,
                                  max_navpose_update_rate = max(1, int(round(1.0 / self.poll_interval_sec))),
                                  msg_if = self.msg_if
                                  )
        self.msg_if.pub_info(" ... NPX interface running")

        self.last_good_time = nepi_utils.get_time()
        nepi_sdk.start_timer_process(self.poll_interval_sec, self.pollCb)

        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever
        nepi_sdk.spin()


    def resolveConfig(self):
        # The depth_config and bus the discovery built, checked again: the
        # discovery and this node install as separate files. Raises ValueError.
        self.i2c_bus = int(self.i2c_bus)
        if not (I2C_BUS_RANGE[0] <= self.i2c_bus <= I2C_BUS_RANGE[1]):
            raise ValueError("i2c_bus " + str(self.i2c_bus) + " is outside " + str(I2C_BUS_RANGE))
        if self.device_path != '/dev/i2c-' + str(self.i2c_bus):
            raise ValueError("device_path " + str(self.device_path) + " does not match i2c_bus " + str(self.i2c_bus))
        config = self.config
        self.depth_model = str(config['depth_model'])
        if self.depth_model not in BAR_MODELS:
            raise ValueError("depth_model '" + self.depth_model + "' must be one of " + str(sorted(BAR_MODELS.keys())))
        self.model = BAR_MODELS[self.depth_model]
        self.depth_osr = int(config['depth_osr'])
        if self.depth_osr not in DEPTH_OSR_VALUES:
            raise ValueError("depth_osr " + str(self.depth_osr) + " must be one of " + str(DEPTH_OSR_VALUES))
        self.fluid_density_kg_m3 = float(config['fluid_density_kg_m3'])
        if not (FLUID_DENSITY_KG_M3_RANGE[0] <= self.fluid_density_kg_m3 <= FLUID_DENSITY_KG_M3_RANGE[1]):
            raise ValueError("fluid_density_kg_m3 " + str(self.fluid_density_kg_m3) + " is outside " +
                             str(FLUID_DENSITY_KG_M3_RANGE))
        self.surface_pressure_mbar = float(config['surface_pressure_mbar'])
        if not (SURFACE_PRESSURE_MBAR_RANGE[0] <= self.surface_pressure_mbar <= SURFACE_PRESSURE_MBAR_RANGE[1]):
            raise ValueError("surface_pressure_mbar " + str(self.surface_pressure_mbar) + " is outside " +
                             str(SURFACE_PRESSURE_MBAR_RANGE))
        self.poll_interval_sec = pollIntervalSec(self.depth_osr)


    def initReadingState(self):
        # navpose_dict stays None until the first good reading (see STALE DATA
        # at the top of this file).
        self.navpose_dict = None
        self.history = []
        self.model_checked = False
        self.last_good_time = 0.0


    #######################
    ### NPX IF Callback

    def getNavPoseCb(self):
        with self.navpose_lock:
            if self.navpose_dict is None:
                return None
            return copy.deepcopy(self.navpose_dict)


    #######################
    ### Reading

    def pollCb(self, timer):
        if self.connected == False or nepi_sdk.is_shutdown():
            return
        try:
            [reading, reason] = self.readSensor()
            if reading is None:
                self.readFailed(reason)
                return
            self.updateReading(reading)
        except Exception as e:
            # Nothing may escape a timer callback; a failure here is retried
            # on the next poll.
            self.msg_if.pub_warn("Depth sensor poll failed: " + str(e), throttle_s = self.WARN_THROTTLE_SEC)


    def readSensor(self):
        # One library read on the poll timer thread: [reading, None] or
        # [None, reason].
        with self.i2c_lock:
            sensor = self.sensor
            if sensor is None:
                return [None, "the I2C bus is closed"]
            try:
                if sensor.read(self.library_osr) != True:
                    return [None, "the library's read() returned False"]
                sample_time = nepi_utils.get_time()
                pressure_mbar = float(sensor.pressure(self.library_units_mbar))
                temperature_c = float(sensor.temperature(self.library_units_c))
                raw = rawReading(sensor)
            except Exception as e:
                return [None, str(e)]
        if raw is not None and (raw[1] == 0 or raw[2] == 0):
            # The ADC returns 0 for a read that came early or overlapped a
            # conversion (Bar doc, Failure Modes): discard it.
            return [None, "the sensor's ADC returned 0"]
        if math.isfinite(pressure_mbar) == False or math.isfinite(temperature_c) == False:
            return [None, "the library returned a non-finite value"]
        return [dict(sample_time = sample_time, pressure_mbar = pressure_mbar,
                     temperature_c = temperature_c, raw = raw), None]


    def readFailed(self, reason):
        # Keep retrying; mark the depth stale once readings stop arriving, and
        # exit after LINK_LOST_SEC so discovery can probe and relaunch.
        self.msg_if.pub_warn("Depth sensor read failed on " + str(self.device_path) + ": " + str(reason) +
                             "; retrying", throttle_s = self.WARN_THROTTLE_SEC)
        no_reading_sec = nepi_utils.get_time() - self.last_good_time
        if no_reading_sec > self.STALE_SEC:
            self.markStale()
        if no_reading_sec > self.LINK_LOST_SEC:
            self.linkLost("no good reading for %.0f s (last error: %s)" % (no_reading_sec, str(reason)))


    def updateReading(self, reading):
        sample_time = reading['sample_time']
        pressure_mbar = reading['pressure_mbar']
        temperature_c = reading['temperature_c']
        self.last_good_time = sample_time
        depth_m = depthFromPressure(pressure_mbar, self.surface_pressure_mbar, self.fluid_density_kg_m3)
        if self.model_checked == False:
            # Once per node start, not per read: a reading taken underwater
            # later would warn for no reason.
            self.model_checked = True
            text = self.modelCheck(pressure_mbar, depth_m, reading['raw'])
            if text is not None:
                self.msg_if.pub_warn(text)
            else:
                self.msg_if.pub_info("First depth sensor reading: %.1f mbar, %.2f C, depth %.3f m" %
                                     (pressure_mbar, temperature_c, depth_m))
        if pressure_mbar > self.model['max_mbar']:
            self.msg_if.pub_warn("Depth sensor reads %.0f mbar, beyond the %.0f mbar top of a %s's range" %
                                 (pressure_mbar, self.model['max_mbar'], self.depth_model),
                                 throttle_s = self.WARN_THROTTLE_SEC)
        rate = self.depthRate(sample_time, depth_m)
        with self.navpose_lock:
            if self.navpose_dict is None:
                self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                self.navpose_dict['frame_depth'] = 'DEPTH'
            self.navpose_dict['has_depth'] = True
            self.navpose_dict['time_depth'] = sample_time
            self.navpose_dict['depth_m'] = depth_m
            self.navpose_dict['depth_m_per_sec'] = rate
        self.publishPressureTemperature(sample_time, pressure_mbar, temperature_c)


    def modelCheck(self, pressure_mbar, depth_m, raw):
        # modelCheckText needs the raw words; a library version that does not
        # keep them gets the window check on its own pressure only.
        if raw is not None:
            [prom, d1, d2] = raw
            return modelCheckText(self.depth_model, prom, d1, d2,
                                  self.surface_pressure_mbar, self.fluid_density_kg_m3)
        if abs(pressure_mbar - self.surface_pressure_mbar) <= MODEL_CHECK_WINDOW_MBAR:
            return None
        return ("First depth sensor reading is %.1f mbar, %.2f m deep. If the sensor is in air or at the "
                "surface, check depth_model and surface_pressure_mbar (this ms5837 version does not expose "
                "its raw values, so the other model's math was not tried)" % (pressure_mbar, depth_m))


    def depthRate(self, sample_time, depth_m):
        # Positive while getting deeper.
        self.history.append((sample_time, depth_m))
        while len(self.history) > 1 and (sample_time - self.history[0][0]) > self.RATE_WINDOW_SEC:
            self.history.pop(0)
        (first_time, first_depth) = self.history[0]
        if sample_time - first_time <= 0.0:
            return 0.0
        return (depth_m - first_depth) / (sample_time - first_time)


    def markStale(self):
        with self.navpose_lock:
            if self.navpose_dict is not None:
                self.navpose_dict['has_depth'] = False
                self.navpose_dict['depth_m_per_sec'] = 0.0
        self.history = []


    def publishPressureTemperature(self, sample_time, pressure_mbar, temperature_c):
        stamp = nepi_sdk.msg_stamp_from_sec(sample_time)
        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = stamp
        pressure_msg.header.frame_id = self.node_name
        pressure_msg.fluid_pressure = pressure_mbar * 100.0
        pressure_msg.variance = 0.0   # unknown
        nepi_sdk.publish_pub(self.pressure_pub, pressure_msg)
        temperature_msg = Temperature()
        temperature_msg.header.stamp = stamp
        temperature_msg.header.frame_id = self.node_name
        temperature_msg.temperature = temperature_c
        temperature_msg.variance = 0.0   # unknown
        nepi_sdk.publish_pub(self.temperature_pub, temperature_msg)


    #######################
    ### Connection

    def connect(self):
        # Construct the library object for the configured model on the
        # configured bus and run init() (reset, PROM read, CRC-4). Runs once,
        # before the poll timer exists.
        if ms5837 is None:
            self.msg_if.pub_warn("Python module ms5837 (Blue Robotics ms5837-python) is not installed in the NEPI " +
                                 "container (" + str(MS5837_IMPORT_ERROR) + ")")
            return False
        try:
            self.library_osr = getattr(ms5837, 'OSR_' + str(self.depth_osr))
            self.library_units_mbar = ms5837.UNITS_mbar
            self.library_units_c = ms5837.UNITS_Centigrade
            library_model = getattr(ms5837, 'MODEL_' + self.model['chip'])
        except Exception as e:
            self.msg_if.pub_warn("This ms5837 version lacks a constant the driver needs: " + str(e))
            return False
        sensor = None
        reason = None
        try:
            sensor = ms5837.MS5837(library_model, self.i2c_bus)
            if getattr(sensor, '_bus', 'unknown') is None:
                reason = ("ms5837 could not open " + self.device_path + " (no permission, or smbus2/smbus missing " +
                          "in the NEPI container)")
            elif sensor.init() != True:
                reason = ("the device at I2C address 0x%02X on %s fails the calibration word CRC check" %
                          (MS5837_I2C_ADDRESS, self.device_path))
            elif promCoefficientsValid(getattr(sensor, '_C', None)) == False:
                reason = ("the device at I2C address 0x%02X on %s returned blank calibration words" %
                          (MS5837_I2C_ADDRESS, self.device_path))
        except Exception as e:
            reason = ("no Bar sensor answers at I2C address 0x%02X on %s (%s)" %
                      (MS5837_I2C_ADDRESS, self.device_path, str(e)))
        if reason is not None:
            self.msg_if.pub_warn("Failed to start the depth sensor: " + reason)
            closeSensorBus(sensor)
            return False

        with self.i2c_lock:
            self.sensor = sensor
        self.hw_version = self.depth_model + " on I2C bus " + str(self.i2c_bus) + " (address 0x%02X)" % MS5837_I2C_ADDRESS
        self.sw_version = "ms5837-python"
        self.msg_if.pub_info(self.hw_version + "; OSR " + str(self.depth_osr) + ", polled every %.3f s" %
                             self.poll_interval_sec + "; fluid density " + str(self.fluid_density_kg_m3) +
                             " kg/m3; surface pressure " + str(self.surface_pressure_mbar) + " mbar")
        return True

    def linkLost(self, reason):
        # Exit rather than retry in place: discovery probes the bus again and
        # relaunches this node once the sensor answers.
        if self.connected == False:
            return
        self.connected = False
        self.markStale()
        self.msg_if.pub_warn("Lost depth sensor on " + str(self.device_path) + " (" + str(reason) +
                             "); shutting down so discovery can relaunch it")
        self.closeBus()
        nepi_sdk.signal_shutdown("Lost depth sensor")

    def closeBus(self):
        with self.i2c_lock:
            closeSensorBus(self.sensor)
            self.sensor = None


    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        """Stop polling the depth sensor and release the I2C bus.

        Waits for a read in progress to finish, so the bus handle is never
        closed under the library mid-transfer.
        """
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        self.connected = False
        self.closeBus()


##########################
# Helpers shared with npx_bluerobotics_bar_discovery.py. Identical copies:
# drivers install flat into one folder and the discovery runs inside
# drivers_mgr, so the files share no module. Keep the copies in step.

def closeSensorBus(sensor):
    # ms5837-python has no close(); its SMBus handle is the private _bus.
    # Setting it to None afterwards makes the library's read() return False
    # rather than raise on a closed handle.
    if sensor is None:
        return
    bus = getattr(sensor, '_bus', None)
    if bus is not None:
        try:
            bus.close()
        except Exception:
            pass
    try:
        sensor._bus = None
    except Exception:
        pass


def promCoefficientsValid(words):
    # True unless the calibration coefficients C1-C6 are blank. An all-zero
    # PROM passes the CRC-4, and 0 or 0xFFFF coefficients mean the bus read
    # nothing real. None (a library without _C) cannot be checked and passes.
    if words is None:
        return True
    if len(words) < 7:
        return False
    for w in words[1:7]:
        if w <= 0 or w >= 0xFFFF:
            return False
    return True


if __name__ == '__main__':
    node = BlueRoboticsBarNode()
