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
"""Offline test of the HNav node decoder: NEPI SDK/API modules are stubbed, no ROS or device needed."""

import importlib.util
import os
import struct
import sys
import threading
import types

import pytest


def _convert_yaw_ned2enu(yaw_ned_deg):
    # Same as nepi_sdk.nepi_nav.convert_yaw_ned2enu
    yaw_enu_deg = 90 - yaw_ned_deg
    if yaw_enu_deg < -180:
        yaw_enu_deg = 360 + yaw_enu_deg
    elif yaw_enu_deg > 180:
        yaw_enu_deg = yaw_enu_deg - 360
    return yaw_enu_deg


def _load_hnav_module():
    nepi_sdk_pkg = types.ModuleType("nepi_sdk")
    nepi_nav = types.ModuleType("nepi_sdk.nepi_nav")
    nepi_nav.BLANK_NAVPOSE_DICT = {}
    nepi_nav.convert_yaw_ned2enu = _convert_yaw_ned2enu
    nepi_utils = types.ModuleType("nepi_sdk.nepi_utils")
    nepi_utils.get_time = lambda: 0.0
    nepi_sdk_pkg.nepi_sdk = types.ModuleType("nepi_sdk.nepi_sdk")
    nepi_sdk_pkg.nepi_utils = nepi_utils
    nepi_sdk_pkg.nepi_nav = nepi_nav
    messages_if = types.ModuleType("nepi_api.messages_if")
    messages_if.MsgIF = object
    device_if_npx = types.ModuleType("nepi_api.device_if_npx")
    device_if_npx.NPXDeviceIF = object
    stubs = {
        "nepi_sdk": nepi_sdk_pkg,
        "nepi_sdk.nepi_sdk": nepi_sdk_pkg.nepi_sdk,
        "nepi_sdk.nepi_utils": nepi_utils,
        "nepi_sdk.nepi_nav": nepi_nav,
        "nepi_api": types.ModuleType("nepi_api"),
        "nepi_api.messages_if": messages_if,
        "nepi_api.device_if_npx": device_if_npx,
    }
    saved = {name: sys.modules.get(name) for name in stubs}
    sys.modules.update(stubs)
    try:
        path = os.path.join(os.path.dirname(__file__), "..", "npx_drivers", "npx_hnav_tcu_node.py")
        spec = importlib.util.spec_from_file_location("npx_hnav_tcu_node", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        for name, mod in saved.items():
            if mod is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = mod
    return module


def _decode_heading(hnav, heading_deg):
    node = hnav.HNavNode.__new__(hnav.HNavNode)
    node.debug = False
    node._lock = threading.Lock()
    node._navpose = {}
    data = struct.pack(hnav.DATA_STRUCT_FMT,
                       0, 0, 0, 0, 0, 0,                 # version, utc, lat, lon, depth, altitude
                       0, 0, int(round(heading_deg / hnav.HNAV_ORIENT_SCALE)),  # roll, pitch, heading
                       0, 0, 0, 0, 0, 0,                 # velocities, angular rates
                       0, 0, 0.0, 0, 0,                  # sound velocity, temp, quality fields
                       0)                                # status: everything valid
    packet = bytes(hnav.DATA_OFFSET) + data
    packet += bytes(hnav.PACKET_SIZE - len(packet))
    node._decode_packet(packet)
    return node._navpose


@pytest.mark.parametrize("heading_deg, enu_yaw_deg", [(0.0, 90.0), (90.0, 0.0), (180.0, -90.0), (270.0, -180.0)])
def test_heading_is_converted_to_enu_yaw(heading_deg, enu_yaw_deg):
    navpose = _decode_heading(_load_hnav_module(), heading_deg)
    assert navpose['heading_deg'] == pytest.approx(heading_deg)
    assert navpose['yaw_deg'] == pytest.approx(enu_yaw_deg)
