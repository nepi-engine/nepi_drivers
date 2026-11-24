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


import os
import struct
import fcntl
import select

from nepi_sdk import nepi_controller

# Reads a game controller through the Linux joystick API (/dev/input/js*),
# standard library only

PKG_NAME = 'JSX_XBOX_GAMEPAD' # Use in display menus
FILE_TYPE = 'DRIVER'

# struct js_event from linux/joystick.h: time (ms), value, type, number
JS_EVENT_FORMAT = '<IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def jsIoctlRead(nr, size):
  # _IOC(_IOC_READ, 'j', nr, size)
  return (2 << 30) | (size << 16) | (ord('j') << 8) | nr

JSIOCGAXES = jsIoctlRead(0x11, 1)
JSIOCGBUTTONS = jsIoctlRead(0x12, 1)
JS_NAME_LENGTH = 128
JSIOCGNAME = jsIoctlRead(0x13, JS_NAME_LENGTH)


class XboxGamepadReader(object):
  MAX_READS_PER_CALL = 32
  READ_SIZE = JS_EVENT_SIZE * 64

  def __init__(self, device_path):
    self.device_path = device_path
    self.fd = None
    self.connected = False
    self.remainder = b''
    self.name = 'Unknown'
    self.axis_count = nepi_controller.AXIS_COUNT
    self.button_count = nepi_controller.BUTTON_COUNT

    self.fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)
    self.connected = True
    # A pipe or other non-joystick file answers ENOTTY; keep the defaults then
    name = self.queryIoctl(JSIOCGNAME, JS_NAME_LENGTH)
    if name is not None:
      name = name.split(b'\x00')[0].decode('utf-8', 'replace').strip()
      if name != '':
        self.name = name
    axis_count = self.queryIoctl(JSIOCGAXES, 1)
    if axis_count is not None:
      self.axis_count = axis_count[0]
    button_count = self.queryIoctl(JSIOCGBUTTONS, 1)
    if button_count is not None:
      self.button_count = button_count[0]
    self.axes = [0.0] * self.axis_count
    self.buttons = [0] * self.button_count
    # The kernel sends the full current state as JS_EVENT_INIT events on open
    self.read_events()

  def is_connected(self):
    """Return False once a read has failed, as it does on an unplug."""
    return self.connected

  def get_name(self):
    """Return the controller name the kernel reports (JSIOCGNAME)."""
    return self.name

  def get_axis_count(self):
    """Return the number of axes."""
    return len(self.axes)

  def get_button_count(self):
    """Return the number of buttons."""
    return len(self.buttons)

  def get_axes(self):
    """Return a copy of the normalized axes, -1..1, stick left and up +1."""
    return list(self.axes)

  def get_buttons(self):
    """Return a copy of the buttons, 0 or 1."""
    return list(self.buttons)

  def read_events(self):
    """Read every js_event waiting on the device and apply it to the axes and buttons.

    Never blocks. A read error or end of file (an unplugged controller) marks
    the reader disconnected instead of raising.

    Returns:
        int: Number of events applied.
    """
    count = 0
    if self.connected == False or self.fd is None:
      return count
    for i in range(self.MAX_READS_PER_CALL):
      try:
        [readable, _, _] = select.select([self.fd], [], [], 0)
        if len(readable) == 0:
          break
        data = os.read(self.fd, self.READ_SIZE)
      except (BlockingIOError, InterruptedError):
        break
      except (OSError, ValueError):
        self.setDisconnected()
        break
      if len(data) == 0:
        self.setDisconnected()
        break
      count += self.parse_events(data)
    return count

  def parse_events(self, data):
    """Apply raw js_event bytes to the axes and buttons.

    A trailing partial record is held until the rest of it arrives.

    Args:
        data (bytes): Raw bytes read from the device.

    Returns:
        int: Number of events applied.
    """
    data = self.remainder + data
    whole = len(data) - (len(data) % JS_EVENT_SIZE)
    self.remainder = data[whole:]
    count = 0
    for offset in range(0, whole, JS_EVENT_SIZE):
      [time_ms, value, event_type, number] = struct.unpack_from(JS_EVENT_FORMAT, data, offset)
      event_type = event_type & ~JS_EVENT_INIT
      if event_type == JS_EVENT_AXIS:
        self.growList(self.axes, number, 0.0)
        self.axes[number] = nepi_controller.normalize_axis(value)
        count += 1
      elif event_type == JS_EVENT_BUTTON:
        self.growList(self.buttons, number, 0)
        self.buttons[number] = 1 if value != 0 else 0
        count += 1
    return count

  def close(self):
    """Close the device."""
    self.connected = False
    if self.fd is not None:
      try:
        os.close(self.fd)
      except OSError:
        pass
      self.fd = None

  def queryIoctl(self, request, size):
    try:
      return fcntl.ioctl(self.fd, request, bytes(size))
    except (OSError, IOError):
      return None

  def growList(self, values, index, fill):
    while len(values) <= index:
      values.append(fill)

  def setDisconnected(self):
    self.connected = False
    self.close()
