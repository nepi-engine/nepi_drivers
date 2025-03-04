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

PKG_NAME = 'PTX_IQR' 
FILE_TYPE = 'NODE'

TEST_DRV_DICT = {
'type': 'PTX',
'group_id': 'IQR',
'NODE_DICT': {
    'file_name': 'ptx_iqr_node.py',
    'class_name': 'IqrPanTiltNode',
},
'DRIVER_DICT': {
    'file_name': 'None' ,
    'class_name':  'None'
},
'DISCOVERY_DICT': {
    'file_name': 'ptx_iqr_discovery.py',
    'class_name': 'IqrPanTiltDiscovery',
    'method': 'AUTO', 
},
'DEVICE_DICT': {},
}



class IqrPanTiltNode:
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  def __init__(self):
    pass

if __name__ == '__main__':
	node = IqrPanTiltNode()
