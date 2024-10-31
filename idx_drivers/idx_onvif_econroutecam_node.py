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


class OnvifCamEconRCNode(OnvifCamNode):
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  pass
  
if __name__ == '__main__':
  node = OnvifCamEconRCNode()

            


        

