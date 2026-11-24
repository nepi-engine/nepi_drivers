#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi drivers (nepi_drivers) repo
# (see https://github.com/nepi-engine/nepi_drivers)
#
# License: NEPI Drivers repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#


import sys
import json
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_controller

from nepi_api.messages_if import MsgIF

from sensor_msgs.msg import Joy
from std_msgs.msg import String

PKG_NAME = 'JSX_XBOX_GAMEPAD' # Use in display menus
FILE_TYPE = 'NODE'


class XboxGamepadNode:

  STATUS_RATE_HZ = 1
  # After a read failure, exit so the discovery process can relaunch the node
  # if the controller is still there. An unplug removes the device path, and
  # discovery stops the node first.
  DISCONNECT_EXIT_S = 5.0

  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  drv_dict = dict()
  driver = None
  joy_pub = None
  status_pub = None
  disconnect_time = None
  last_connected = None

  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF()
    self.msg_if.pub_info("Starting Node Initialization Processes")

    self.publish_lock = threading.Lock()

    # Get required drv driver dict info
    try:
      self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
      self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
      self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
      self.driver_path = self.drv_dict['path']
      self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
      self.driver_module = self.driver_file.split('.')[0]
      self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
    except Exception as e:
      self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
      return

    # import driver class from driver module
    self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
    [success, msg, self.driver_class] = nepi_drvs.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
    if success:
      try:
        self.driver = self.driver_class(self.device_path)
      except Exception as e:
        self.msg_if.pub_warn("Failed to open controller at " + str(self.device_path) + ": " + str(e))
        sys.exit(-1)

    if self.driver is None or self.driver.is_connected() == False:
      self.msg_if.pub_warn("No controller connected at " + str(self.device_path))
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because controller not connected")
      return
    self.msg_if.pub_info("Connected to " + self.driver.get_name() + " with " + str(self.driver.get_axis_count()) +
                         " axes and " + str(self.driver.get_button_count()) + " buttons")

    ##############################
    # Publishers
    self.joy_pub = nepi_sdk.create_publisher(self.node_namespace + '/joy', Joy, queue_size = 1)
    self.status_pub = nepi_sdk.create_publisher(self.node_namespace + '/status', String, queue_size = 1, latch = True)
    self.msg_if.pub_info("Publishing sensor_msgs/Joy on " + self.node_namespace + '/joy')

    ##############################
    # Start node processes. A steady Joy rate lets consumers detect a lost
    # controller by silence.
    nepi_sdk.start_timer_process(1.0 / nepi_controller.JOY_PUBLISH_RATE_HZ, self.publishJoyCb)
    nepi_sdk.start_timer_process(1.0 / self.STATUS_RATE_HZ, self.publishStatusCb)
    nepi_sdk.on_shutdown(self.cleanup_actions)

    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    # Now start the node
    nepi_sdk.spin()


  #**********************
  # Node Functions

  def publishJoyCb(self, timer):
    # Periodic timer threads die on an uncaught exception
    try:
      if nepi_sdk.is_shutdown() or self.driver is None:
        return
      with self.publish_lock:
        self.driver.read_events()
        connected = self.driver.is_connected()
        if connected != self.last_connected:
          self.last_connected = connected
          self.publishStatus()
        if connected == False:
          self.checkDisconnected()
          return
        joy_msg = Joy()
        joy_msg.header.stamp = nepi_sdk.get_msg_stamp()
        joy_msg.axes = self.driver.get_axes()
        joy_msg.buttons = self.driver.get_buttons()
        nepi_sdk.publish_pub(self.joy_pub, joy_msg)
    except Exception as e:
      self.msg_if.pub_warn("Failed to publish controller state: " + str(e), throttle_s = 5.0)

  def checkDisconnected(self):
    cur_time = nepi_utils.get_time()
    if self.disconnect_time is None:
      self.disconnect_time = cur_time
      self.msg_if.pub_warn("Controller disconnected at " + str(self.device_path) + ". Stopped publishing Joy")
    elif (cur_time - self.disconnect_time) > self.DISCONNECT_EXIT_S:
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because controller disconnected")

  def publishStatusCb(self, timer):
    try:
      if nepi_sdk.is_shutdown() or self.driver is None:
        return
      with self.publish_lock:
        self.publishStatus()
    except Exception as e:
      self.msg_if.pub_warn("Failed to publish controller status: " + str(e), throttle_s = 5.0)

  def publishStatus(self):
    status_dict = {
      'connected': self.driver.is_connected(),
      'device_name': self.driver.get_name(),
      'device_path': self.device_path,
      'axis_count': self.driver.get_axis_count(),
      'button_count': self.driver.get_button_count()
    }
    nepi_sdk.publish_pub(self.status_pub, String(data = json.dumps(status_dict)))

  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    if self.driver is not None:
      self.driver.close()


if __name__ == '__main__':
  node = XboxGamepadNode()
