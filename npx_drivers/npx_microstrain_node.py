#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus.
# License: Numurus Software License (see project).
#
import math
import os
import subprocess


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

import numpy as np
import tf

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, Quaternion
from nav_msgs.msg import Odometry


from nepi_sdk import nepi_sdk, nepi_utils
from nepi_api.messages_if import MsgIF
from nepi_api.device_if_npx import NPXDeviceIF
from nepi_interfaces.msg import DeviceNPXStatus

PKG_NAME = 'NPX_MICROSTRAIN_AHAR'
DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
ros_node_name = "npx_microstrain_node"  # driver name + base namespace

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
    navpose_update_rate = 20
    navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT


    def __init__(self):
        ####  NODE Initialization ####
        nepi_sdk.init_node(name= DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")
        self.msg_if.pub_info("Starting MicroStrain node init")

        self.device_info_dict = dict(node_name = self.node_name,
                            device_name = "3DM-GV7-AHRS",
                            identifier = "ttyTHS0",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")
        

        sub_namespace = nepi_sdk.create_namespace(self.base_namespace, "npx_microstrain_node/imu/data")

        nepi_sdk.create_subscriber(sub_namespace, Imu, self.odom_topic_callback, queue_size = 10)

        # Launch underlying MicroStrain ROS driver
        self.source_node_name = self.node_name + "_source"
        params_file_path = '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_microstrain_params.yaml'
        driver_ros_node_name = ros_node_name

        if not os.path.exists(params_file_path):
            self.msg_if.pub_warn("Could not find param file at: " + params_file_path)
        else:
            success, msg, pub_process = launch_ms_node(self.source_node_name, driver_ros_node_name, params_file_path)
            self.msg_if.pub_info("Source Node launch return msg: " + msg)

      
        ###############################
        # Create a NPX Device IF
        if self.getNavPoseCb is not None:
            self.msg_if.pub_warn("Starting NPX Device IF Initialization")
            self.npx_if = NPXDeviceIF(device_info = self.device_info_dict, 
                data_source_description = "IMU", #self.data_source_description
                data_ref_description = "sensor_center",
                getNavPoseCb = self.getNavPoseCb,
                get3DTransformCb = None,
                max_navpose_update_rate = self.navpose_update_rate,
                msg_if = self.msg_if
                )




        ##########################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Now start zed node check process
        # self.attempts = 0
        # nepi_sdk.start_timer_process((1), self.checkZedNodeCb)
        # rospy.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()



    ### Callback to publish RBX odom topic
    def odom_topic_callback(self,imu_msg):
        # Convert quaternion to roll,pitch,yaw
        pose = imu_msg.orientation
        xyzw = list([pose.x,pose.y,pose.z,pose.w])
        rpy = nepi_nav.convert_quat2rpy(xyzw)

        timestamp = nepi_sdk.sec_from_msg_stamp(imu_msg.header.stamp)

        self.navpose_dict['has_orientation']
        self.navpose_dict['time_oreantation'] = timestamp
        # Orientation Degrees in selected 3d frame (roll,pitch,yaw)
        self.navpose_dict['roll_deg'] = rpy[0]
        self.navpose_dict['pitch_deg'] = rpy[1]
        self.navpose_dict['yaw_deg'] = rpy[2]


    def getNavPoseCb(self):
        return self.navpose_dict

if __name__ == '__main__':
    MicrostrainNode()
