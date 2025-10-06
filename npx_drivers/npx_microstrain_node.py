#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
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
import math
import os
import subprocess

from nepi_sdk import nepi_sdk, nepi_utils
from nepi_api.messages_if import MsgIF
from nepi_api.device_if_npx import NPXDeviceIF
from nepi_interfaces.msg import DeviceNPXStatus

PKG_NAME = 'NPX_MICROSTRAIN_AHAR'
DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
ros_node_name = "npx_microstrain_node"
        
def launch_ms_node(pkg_name, ros_node_name, param_file_path):
    """
    Launch the ROS1 MicroStrain driver via roslaunch and return (success, msg, subprocess_handle).
    """
    device_node_launch_cmd = [
        'roslaunch', 'microstrain_inertial_driver', 'microstrain.launch',
        f'node_name:={ros_node_name}',
        f'namespace:=/nepi/device1/{ros_node_name}',
        f'params_file:={param_file_path}',
    ]

    try:
        sub_process = subprocess.Popen(device_node_launch_cmd)
    except Exception as e:
        return (False, f"Failed to launch {ros_node_name}: {e}", None)

    # If the process died immediately, treat as failure
    rc = sub_process.poll()
    if rc is not None and rc != 0:
        return (False,
                f"Failed to start {ros_node_name} via: {' '.join(device_node_launch_cmd)} (rc={rc})",
                None)

    return (True, "Success", sub_process)


class MicrostrainNode(object):
    def __init__(self):
        # --- ROS node ---
        nepi_sdk.init_node(name=DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.node_name = nepi_sdk.get_node_name()
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting MicroStrain node init")

        # Launch underlying MicroStrain ROS driver
        self.source_node_name = self.node_name + "_source"
        params_file_path = '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_microstrain_params.yaml'
        ros_node_name = 'npx_microstrain_node'

        if not os.path.exists(params_file_path):
            self.msg_if.pub_warn("Could not find param file at: " + params_file_path)
        else:
            success, msg, pub_process = launch_ms_node(self.source_node_name, ros_node_name, params_file_path)
            self.msg_if.pub_warn("Source Node launch return msg: " + msg)



if __name__ == '__main__':
    MicrostrainNode()
