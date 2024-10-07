#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


PKG_NAME = 'PTX_IQR' # Use in display menus
FILE_TYPE = 'NODE'
NODE_DICT = dict(
description = 'Driver package for IQR USB pan tilt devices',
class_name = 'IqrPanTiltNode', # Should Match Class Name,
group ='PTX',
group_id = 'IQR' ,
driver_pkg_name = 'None', # 'Required Driver PKG_NAME or 'None'
discovery_pkg_name = 'PTX_IQR' # 'Required Discovery PKG_NAME or 'None'
)

TEST_DRV_DICT = {
'group': 'PTX',
'group_id': 'IQR',
'pkg_name': 'PTX_IQR',
'NODE_DICT': {
    'file_name': 'ptx_iqr_node.py',
    'module_name': 'ptx_iqr_node',
    'class_name': 'IqrPanTiltNode',
},
'DRIVER_DICT': {
    'file_name': '' ,
    'module_name': '' ,
    'class_name':  ''
},
'DISCOVERY_DICT': {
    'file_name': 'ptx_iqr_discovery.py',
    'module_name': 'ptx_iqr_discovery',
    'class_name': 'IqrPanTiltDiscovery',
    'interfaces': ['USB'],
    'options_1_dict': {
        'default_val': 'None',
        'set_val': 'None'
    },
    'options_2_dict': {
        'default_val': 'None',
        'set_val': 'None'
    },
    'method': 'AUTO', 
    'include_ids': ['iqr_pan_tilt'],
    'exclude_ids': []
},
'DEVICE_DICT': {},
'path': '/opt/nepi/ros/lib/nepi_drivers',
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
