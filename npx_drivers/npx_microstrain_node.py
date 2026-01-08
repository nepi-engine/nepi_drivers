#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus.
# License: Numurus Software License (see project).
#
import math
import os

########################
# # Uncomment for test mode
# ros_namespace = '/nepi/device1'
# if 'ROS_NAMESPACE' not in os.environ:
#     os.environ['ROS_NAMESPACE'] = ros_namespace
#     print("Base namespace set to: " + ros_namespace)
# else:
#     # If it's already set and you want to ensure your desired namespace is used,
#     # you can overwrite it or add a check to see if it matches your desired value.
#     if os.environ['ROS_NAMESPACE'] != ros_namespace:
#         print(f"Warning: ROS_NAMESPACE is already set to '{os.environ['ROS_NAMESPACE']}'. Overwriting with '{ros_namespace}'.", file=sys.stderr)
#         os.environ['ROS_NAMESPACE'] = ros_namespace
########################

import subprocess

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header

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


DRV_TEST_DICT = {
    'port': '/dev/ttyUSB9',
    'baudrate': 115200,
    'aux_port': 'None',
    'aux_baudrate': 115200,
    'debug': True,
    'param_file': '/opt/nepi/nepi_engine/lib/nepi_drivers/npx_microstrain_params.yaml'
}



def launch_driver_node(base_namespace, node_name, param_file_path):
    """
    Launch the ROS1 MicroStrain driver via roslaunch and return (success, msg, subprocess_handle).
    """
    nepi_namespace = nepi_sdk.create_namespace(base_namespace,node_name)
    device_node_launch_cmd = [
        'roslaunch', 'microstrain_inertial_driver', 'microstrain.launch',
        f'node_name:={node_name}',
        f'namespace:={base_namespace}',
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

def check_driver_node(sub_process):
    """
    Check on the ROS1 MicroStrain driver.
    """
    rc = sub_process.poll()
    if rc is not None and rc != 0:
        return False
    return True


def kill_driver_node(node_name):
    """
    Kill the ROS1 MicroStrain driver.
    """

    success = nepi_sdk.kill_node(node_name)

    return success




class MicrostrainNode(object):
    navpose_update_rate = 20
    navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    navpose_dict['has_orientation'] = True

    device_dict = None

    driver_node_name = DEFAULT_NODE_NAME
    driver_node_process = None
    imu_sub = None

    def __init__(self, device_dict = None):

        ####  NODE Initialization ####
        nepi_sdk.init_node(name= self.driver_node_name)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")
        

        ##############################
        # Gather Driver Settings
        self.msg_if.pub_info("Gathering driver settings")
        try:
            self.device_dict = nepi_sdk.get_param('~drv_dict',dict())['DEVICE_DICT'] 
        except:
            self.msg_if.pub_warn("Failed to load driver settings from params")
            self.device_dict = device_dict
        
        self.msg_if.pub_info("Got drv dict: " + str(self.device_dict))
        if self.device_dict is None:
            self.msg_if.pub_info("Driver Dict not provided")
            nepi_sdk.signal_shutdown(self.node_name + ": Driver Dict not provided")
            return

        try:
            port = self.device_dict['port']
            self.msg_if.pub_info("Using port: " + str(port))

            baudrate = self.device_dict['baudrate']
            self.msg_if.pub_info("Using baudrate: " + str(baudrate))

            aux_port = self.device_dict['aux_port']
            self.msg_if.pub_info("Using aux_port: " + str(aux_port))

            aux_baudrate = self.device_dict['aux_baudrate']
            self.msg_if.pub_info("Using aux_baudrate: " + str(aux_baudrate))

            debug = False
            if 'debug' in self.device_dict.keys():
                debug = self.device_dict['debug']
            self.msg_if.pub_info("Debug set to: " + str(debug))

            param_file = self.device_dict['param_file']
            self.msg_if.pub_info("Using param_file: " + str(param_file))
        except Exception as e:
            self.msg_if.pub_info("Driver Dict missing entries: " + str(e) )
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting Down - Driver Dict missing entries")
            return

        

        

        ############################
        # Update Driver Node Param File Values
        if not os.path.exists(param_file):
            self.msg_if.pub_warn("Could not find param file at: " + param_file)
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting Down - Could not find param file")

            return

        self.msg_if.pub_info("Updating params in: " + param_file)
        # Read Param File Settings
        node_params_dict = nepi_utils.read_dict_from_file(param_file)
        if node_params_dict is None:
            self.msg_if.pub_warn("Could not read params from file at: " + param_file)
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting Down - Could not read params from file")
            return
        
        # Update Param File Values
        node_params_dict['port'] = port
        node_params_dict['baudrate'] = baudrate
        node_params_dict['aux_port'] = aux_port
        node_params_dict['aux_baudrate'] = aux_baudrate
        node_params_dict['debug'] = debug
        success = nepi_utils.write_dict_to_file(node_params_dict,param_file)
        
        if success == True:
            self.msg_if.pub_warn("Updated param file at: " + param_file + " with: " + str(node_params_dict))
        else:
            self.msg_if.pub_warn("Failed to update param file at: " + param_file)
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting Down - Failed to update param file")
            return

        #############################
        # Launch Driver Node
        self.driver_node_name = self.node_name + "_driver"
        success, msg, self.driver_node_process = launch_driver_node(self.base_namespace, self.driver_node_name, param_file)
        if success == False:
            self.msg_if.pub_warn("Failed to launch driver process node: " + self.driver_node_name + " with return msg: " + str(msg))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting Down - Failed to update param file")
            return
        self.msg_if.pub_info("Source Node launch return msg: " + msg)


        #############################
        # Wait for Driver Node to Start and Get device info
        self.device_info_dict = dict(node_name = self.node_name,
                            device_name = "3DM-GV7-AHRS",
                            identifier = "ttyTHS0",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")


        ############################
        # Create Subscribers
        #sub_base_namespace = nepi_sdk.create_namespace(self.base_namespace,self.driver_node_name)
        sub_namespace = nepi_sdk.create_namespace(self.base_namespace, "imu/data")
        self.msg_if.pub_warn("Starting Imu subscriber for namespace: " + sub_namespace) 
        self.imu_sub = nepi_sdk.create_subscriber(sub_namespace, Imu, self.odomCb, queue_size = 10)

        sub_namespace = nepi_sdk.create_namespace(self.node_namespace, "shutdown")
        self.msg_if.pub_warn("Starting Shutdown subsriber for namespace: " + sub_namespace) 
        self.imu_sub = nepi_sdk.create_subscriber(sub_namespace, Empty, self.shutdownCb, queue_size = 10)
      
        ###############################
        # Create a NPX Device IF
        self.msg_if.pub_warn("Starting NPX Class Initialization")
        if self.getNavPoseCb is not None:
            self.msg_if.pub_warn("Starting NPX Device IF Initialization")
            self.npx_if = NPXDeviceIF(device_info = self.device_info_dict, 
                data_source_description = "sensor", #self.data_source_description
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
        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()



    ### Callback to publish RBX odom topic
    def odomCb(self,imu_msg):
        # Convert quaternion to roll,pitch,yaw
        pose = imu_msg.orientation
        xyzw = list([pose.x,pose.y,pose.z,pose.w])
        rpy = nepi_nav.convert_quat2rpy(xyzw)

        timestamp = nepi_sdk.sec_from_msg_stamp(imu_msg.header.stamp)

        self.navpose_dict['time_oreantation'] = timestamp
        # Orientation Degrees in selected 3d frame (roll,pitch,yaw)
        self.navpose_dict['roll_deg'] = rpy[0]
        self.navpose_dict['pitch_deg'] = rpy[1]
        self.navpose_dict['yaw_deg'] = rpy[2]


    def getNavPoseCb(self):
        return self.navpose_dict


    def shutdownCb(self,msg):
        self.msg_if.pub_warn("Recieved shutdown request for node: " + str(self.node_name))
        if self.driver_node_process is not None:
            self.msg_if.pub_warn("Killing driver node: " + str(self.driver_node_process))
            success = kill_driver_node(self.driver_node_name)
            if success == False:
                self.msg_if.pub_warn("Failed to kill driver node: " + str(self.driver_node_process))
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down on request")

    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        self.msg_if.pub_warn("Shutting down: Executing script cleanup actions")
        if self.driver_node_process is not None:
            self.msg_if.pub_warn("Killing driver node: " + str(self.driver_node_process))
            success = kill_driver_node(self.driver_node_name)
            if success == False:
                self.msg_if.pub_warn("Failed to kill driver node: " + str(self.driver_node_process))
        self.msg_if.pub_warn("Shutdown cleanup actions complete")
            

if __name__ == '__main__':
    device_dict = DRV_TEST_DICT
    MicrostrainNode(device_dict)
