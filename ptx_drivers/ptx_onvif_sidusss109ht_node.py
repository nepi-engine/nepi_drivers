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

PKG_NAME = 'PTX_ONVIF_SIDUS_SS109HT' # Use in display menus
DESCRIPTION = 'Driver package for Sidus SS109 ONVIF pan tilt devices'
FILE_TYPE = 'NODE'
CLASS_NAME = 'OnvifSidusSsPanTiltNode' # Should Match Class Name
GROUP ='PTX'
GROUP_ID = 'ONVIF' 

DRIVER_PKG_NAME = 'PTX_ONVIF_SIDUS_SS109HT' # 'Required Driver PKG_NAME or 'None'
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



class OnvifSidusSsPanTiltNode(OnvifPanTiltNode):
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  def __init__(self):
    pass

if __name__ == '__main__':
	node = OnvifSidusSsPanTiltNode()
