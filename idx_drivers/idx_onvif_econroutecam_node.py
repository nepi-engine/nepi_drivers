#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
import sys
import time
import math
import rospy
import threading
import cv2

from nepi_edge_sdk_base.device_if_idx import ROSIDXSensorIF
from idx_onvif_generic_node import OnvifCamNode

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img
from nepi_edge_sdk_base import nepi_nex

PKG_NAME = 'IDX_ONVIF_EconRoute' # Use in display menus
DESCRIPTION = 'Driver package for Econ Route Cam Onvif camera devices'
FILE_TYPE = 'NODE'
CLASS_NAME = 'OnvifCamEconRCNode' # Should Match Class Name
GROUP ='IDX'
GROUP_ID = 'ONVIF' 


DRIVER_PKG_NAME = 'IDX_ONVIF_EconRoute' # 'Required Driver PKG_NAME or 'None'
DRIVER_INTERFACES = ['IP'] # 'USB','IP','SERIALUSB','SERIAL','CANBUS'
DRIVER_OPTIONS_1_NAME = 'None'
DRIVER_OPTIONS_1 = [] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_1 = 'None'
DRIVER_OPTIONS_2_NAME = 'None'
DRIVER_OPTIONS_2 = [] # List of string options. Selected option passed to driver
DRIVER_DEFAULT_OPTION_2 = 'None'


DISCOVERY_PKG_NAME = 'None' # ONVIF nodes managed by onvif manager
DISCOVERY_METHOD = 'OTHER'  # 'AUTO', 'MANUAL', or 'OTHER' if managed by seperate application
DISCOVERY_IDS = []  # List of string identifiers for discovery process
DISCOVERY_IGNORE_IDS = [] # List of string identifiers for discovery process



class OnvifCamEconRCNode(OnvifCamNode):
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  pass
  
if __name__ == '__main__':
  node = OnvifCamEconRCNode()

            


        

