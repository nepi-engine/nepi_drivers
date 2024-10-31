#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


from ptx_onvif_generic_node import OnvifPanTiltNode


PKG_NAME = 'PTX_ONVIF_SIDUS_SS109HT' # Use in display menus
FILE_TYPE = 'NODE'


class OnvifSidusSsPanTiltNode(OnvifPanTiltNode):
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  def __init__(self):
    pass

if __name__ == '__main__':
	node = OnvifSidusSsPanTiltNode()
