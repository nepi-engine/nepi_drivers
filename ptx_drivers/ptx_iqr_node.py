#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy

from ptx_onvif_generic_node import OnvifPanTiltNode

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_nex

from nepi_drivers.ptx_device_if import ROSPTXActuatorIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

PKG_NAME = 'PTX_IQR' # Use in display menus
DESCRIPTION = 'Driver package for IQR USB pan tilt devices'
FILE_TYPE = 'NODE'
CLASS_NAME = 'None' # Discovery calls compiled iqr_ros_pan_tilt_node
GROUP ='PTX'
GROUP_ID = 'IQR' 

DRIVER_PKG_NAME = 'None' # 'Required Driver PKG Name or 'None'
DRIVER_INTERFACES = ['USB'] # 'USB','IP','SERIALUSB','SERIAL','CANBUS'
DRIVER_OPTIONS_1_NAME = 'None'
DRIVER_OPTIONS_1 = [] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_1 = 'None'
DRIVER_OPTIONS_2_NAME = 'None'
DRIVER_OPTIONS_2 = [] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_2 = 'None'

DISCOVERY_PKG_NAME = 'PTX_IQR' # 'Required Discovery PKG Name or 'None'
DISCOVERY_METHOD = 'AUTO'  # 'AUTO', 'MANUAL', or 'OTHER' if managed by seperate application
DISCOVERY_IDS = ['iqr_pan_tilt']  # List of string identifiers for discovery process
DISCOVERY_IGNORE_IDS = [] # List of string identifiers for discovery process

TEST_NEX_DICT = {
    'group': 'PTX',
    'group_id': 'IQR',
    'node_file_name': 'ptx_iqr_node.py',
    'node_file_path': '/opt/nepi/ros/lib/nep_drivers',
    'node_module_name': 'ptx_iqr_node',
    'node_class_name': 'IqrPanTiltNode',
    'driver_name': "None",
    'driver_file_name': "None" ,
    'driver_file_path':  "None",
    'driver_module_name': "None" ,
    'driver_class_name':  "None",
    'driver_interfaces': [],
    'driver_options_1': [],
    'driver_default_option_1': 'None',
    'driver_set_option_1': 'None',
    'driver_options_2': [],
    'driver_default_option_2': 'None',
    'driver_set_option_2': 'None',
    'discovery_name': 'PTX_IQR', 
    'discovery_file_name': 'ptx_iqr_discovery.py',
    'discovery_file_path': '/opt/nepi/ros/lib/nepi_drivers',
    'discovery_module_name': 'ptx_iqr_discovery',
    'discovery_class_name': 'IqrPanTiltDiscovery',
    'discovery_method': 'AUTO', 
    'discovery_ids': ['iqr_pan_tilt'],
    'discovery_ignore_ids': [],
    'device_dict': {},
    'order': 1,
    'active': True,
    'msg': ""
    }


class IqrPanTiltNode:
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  def __init__(self):
    pass

if __name__ == '__main__':
	node = IqrPanTiltNode()
