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

PKG_NAME = 'PTX_IQR' # Use in display menus
FILE_TYPE = 'NODE'


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
