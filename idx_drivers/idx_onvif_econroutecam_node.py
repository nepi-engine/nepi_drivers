#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

from idx_onvif_generic_node import OnvifCamNode


PKG_NAME = 'IDX_ONVIF_EconRoute' # Use in display menus
FILE_TYPE = 'NODE'
NODE_DICT = dict(
description = 'Driver package for Econ Route Cam Onvif camera devices',
class_name = 'OnvifCamEconRCNode', # Should Match Class Name,
group ='IDX',
group_id = 'ONVIF' ,
driver_pkg_name = 'IDX_ONVIF_EconRoute', # 'Required Driver PKG_NAME or 'None'
discovery_pkg_name = 'None' # 'Required Discovery PKG_NAME or 'None'
)

class OnvifCamEconRCNode(OnvifCamNode):
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  pass
  
if __name__ == '__main__':
  node = OnvifCamEconRCNode()

            


        

