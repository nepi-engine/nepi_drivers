#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

# TEMPORARY TEST FIXTURE. Delete with the two sibling lsx_settings_test_* files.
#
# This driver connects to no hardware and produces no data. It declares one
# setting of every nepi_controls control type and serves its own small page with
# a live read-back box per setting so that a setting which is silently dropped
# on the way to the controls dict is visible rather than being an absence nobody
# notices.
#
# It declares device type LSX because LSX is the cheapest device interface to
# stub -- on/off and intensity, held in memory -- and because declaring an
# existing type means the stock RUI settings panel renders for it with no RUI
# change. That panel is the second view: a setting on this driver's page but
# missing from the RUI panel is a setting dropped downstream of this node.

import copy
import json
import socket
import threading

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls

from nepi_interfaces.msg import DeviceLSXStatus

from nepi_api.device_if_lsx import LSXDeviceIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'LSX_SETTINGS_TEST'
FILE_TYPE = 'NODE'

DEFAULT_PAGE_PORT = 5010
PAGE_POLL_MS = 1000

# Every option-bearing setting gets the same three options, named so they are
# unmistakable in a read-back text box.
TEST_OPTIONS = ['option_a', 'option_b', 'option_c']

# 'Selections' (the dropdown) gets a longer list on purpose. The dropdown exists
# because a row of per-option toggles stops being readable once the list runs
# long, so probing it with three options would not exercise the thing it is for.
TEST_OPTIONS_LONG = ['option_a', 'option_b', 'option_c',
                     'option_d', 'option_e', 'option_f']


#########################################
# Settings Test LSX Driver Node Class
#########################################

class SettingsTestNode(object):

  # One setting per type in nepi_controls.CONTROL_TYPES, named after its type so
  # the page reads as a checklist. Numeric min and max live in 'bounds'; an
  # 'options' pair on a numeric control is ignored by create_controls_dict().
  DECLARED_SETTINGS = dict(
    test_menu        = {'type': 'Menu',         'default': 0,                'options': TEST_OPTIONS},
    test_selection   = {'type': 'Selection',    'default': 'option_a',       'options': TEST_OPTIONS},
    # The row of per-option toggles. This is what the retired name 'Selections'
    # used to mean; the plural toggle row is 'Toggles' now, and 'Selections' was
    # reused for what 'SelectionsDD' used to be. Both still carry the identical
    # value shape (a list of chosen option strings), so a stale 'Selections'
    # does not fail -- it silently draws as a dropdown instead. The setting key
    # keeps its old name so saved configs still load.
    test_selections  = {'type': 'Toggles',      'default': ['option_a'],     'options': TEST_OPTIONS},
    # Same value shape as test_selections above (a list of chosen option
    # strings) under the dropdown widget rather than a row of toggles. Declared
    # right beside it so the two can be compared side by side in one panel.
    test_selections_dd = {'type': 'Selections', 'default': ['option_a'],     'options': TEST_OPTIONS_LONG},
    test_trigger     = {'type': 'Trigger',      'default': 0},
    test_bool        = {'type': 'Toggle',       'default': False},
    test_string      = {'type': 'String',       'default': 'option_a'},
    test_int         = {'type': 'Int',          'default': 25,               'bounds': [0, 100]},
    test_float       = {'type': 'Float',        'default': 2.5,              'bounds': [0.0, 10.0]},
    test_float_slider  = {'type': 'FloatSlider',  'default': 0.5,            'bounds': [0.0, 1.0]},
    test_float_sliders = {'type': 'RangeSlider',  'default': [0.25, 0.75],   'bounds': [0.0, 1.0]},

    # The probe this entry used to be is answered and retired. 'Discrete' was
    # the old name for a named option list and is NOT a member of
    # nepi_controls.CONTROL_TYPES; only create_controls_dict()'s legacy rewrite
    # at nepi_controls.py:110 let one survive at all. No driver in this repo
    # authors it any more, so the entry declares the current name. It is left in
    # place, deliberately identical to test_selection above, as the witness that
    # the retired name is gone -- if 'Discrete' ever reappears in this fixture's
    # dropped table, something re-introduced it.
    test_discrete    = {'type': 'Selection',    'default': 'option_a',       'options': TEST_OPTIONS},
  )

  # Factory control values for the stub light. Nothing behind them.
  FACTORY_CONTROLS = dict(
    standby_enabled = False,
    on_off_state = False,
    intensity_ratio = 0.0,
    strobe_enabled = False,
    blink_interval_sec = 0,
    blink_enabled = False
  )

  device_info_dict = dict(device_name = "",
                          path = "",
                          serial_number = "",
                          hw_version = "",
                          sw_version = "")

  init_settings_dict = dict()
  settings_dict = dict()
  dropped_settings = dict()

  # The in-memory value store. This fixture has no hardware, so this dict is
  # what a real driver's device would be.
  value_store = dict()
  settings_lock = threading.Lock()

  # Stub device state, held in memory and nothing else
  on_off_state = False
  intensity_ratio = 0.0

  lsx_if = None
  page_port = DEFAULT_PAGE_PORT
  page_server = None
  page_thread = None

  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  drv_dict = dict()
  def __init__(self):
      ####  NODE Initialization ####
      nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
      self.class_name = type(self).__name__
      self.base_namespace = nepi_sdk.get_base_namespace()
      self.node_name = nepi_sdk.get_node_name()
      self.node_namespace = nepi_sdk.get_node_namespace()

      ##############################
      # Create Msg Class
      self.msg_if = MsgIF(log_name = self.class_name)
      self.msg_if.pub_info("Starting Node Initialization Processes")
      self.msg_if.pub_warn("This is a TEMPORARY SETTINGS TEST FIXTURE. It drives no hardware.")

      ##############################
      # Initialize Class Variables
      try:
        self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
        self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
        self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
      except Exception as e:
        self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
        return

      self.page_port = self.getPagePort()

      # There is no device to connect to. Build the settings and go.
      self.settings_dict = self.initSettingsDict()
      self.settings_dict = self.refreshSettingsDict()

      self.launchDeviceIf()

      self.startPageServer()

      nepi_sdk.on_shutdown(self.cleanupActions)

      ## Initiation Complete
      self.msg_if.pub_info("Initialization Complete")
      nepi_sdk.spin()


  #**********************
  # Driver option

  def getPagePort(self):
      port = DEFAULT_PAGE_PORT
      try:
        # getDriversDict() stringifies every discovery option value, so cast it.
        port = int(self.drv_dict['DISCOVERY_DICT']['OPTIONS']['page_port']['value'])
      except Exception as e:
        self.msg_if.pub_warn("Could not read page_port option, using default " +
                             str(DEFAULT_PAGE_PORT) + " : " + str(e))
        port = DEFAULT_PAGE_PORT
      if port < 1024 or port > 65535:
        self.msg_if.pub_warn("page_port " + str(port) + " out of range, using default " +
                             str(DEFAULT_PAGE_PORT))
        port = DEFAULT_PAGE_PORT
      return port


  def buildDriverOptions(self):
      # Reports the discovery OPTIONS block exactly as this node received it.
      #
      # These are DRIVER options, not device settings -- a different surface with
      # a different owner. drivers_mgr builds their controls dict itself
      # (initDiscoverySettingsDict) and renders them in the Drivers Manager
      # panel; the node never registers them with SettingsIF. So the Drivers
      # Manager panel is where you test whether each type renders and accepts a
      # value, and this table is only the confirmation of what actually reached
      # the node.
      #
      # It is a snapshot, not a live read-back: discovery writes drv_dict to the
      # param server once, at launch (lsx_settings_test_discovery.py:130), so
      # changing an option in the RUI does NOT move these values until the
      # driver is restarted. That lag is expected -- do not read it as a
      # dropped option.
      options = []
      options_dict = dict()
      try:
        options_dict = self.drv_dict['DISCOVERY_DICT']['OPTIONS']
      except Exception as e:
        self.msg_if.pub_warn("No discovery OPTIONS in drv_dict: " + str(e))
        return options

      for option_name in options_dict.keys():
        entry = options_dict[option_name]
        declared = entry.get('type', 'None')
        # DELIBERATE BACKWARD-COMPATIBILITY PATH, display only. This driver's
        # own yaml no longer declares the retired 'Discrete', but drivers_mgr
        # still rewrites it to Selection before building the control
        # (drivers_mgr.py:885), so a yaml that does declare it renders as a type
        # it never declared. Showing both columns is how that gets caught. This
        # line formats a report; it converts, validates and applies nothing.
        rendered = 'Selection' if declared == 'Discrete' else declared
        supported = declared in nepi_controls.CONTROL_TYPES
        options.append({
          'name': option_name,
          'type': str(declared),
          'rendered_as': str(rendered),
          'range': 'options ' + str([str(item) for item in entry.get('options', [])]),
          'default': str(entry.get('default')),
          'value': str(entry.get('value', entry.get('default'))),
          'supported': 'yes' if supported else 'NO -- not in CONTROL_TYPES'
        })
      return options


  #**********************
  # Device setting functions

  def initSettingsDict(self):
      init_settings_dict = copy.deepcopy(self.DECLARED_SETTINGS)
      self.init_settings_dict = init_settings_dict

      settings_dict = dict()
      dropped_settings = dict()

      # Built one entry at a time rather than in a single create_controls_dict()
      # call, so that a control that fails to build is named instead of
      # disappearing. create_controls_dict() swallows its own exceptions, which
      # is the defect this fixture exists to make visible -- so this loop must
      # not add a second bare except on top of it.
      for setting_name in init_settings_dict.keys():
        entry = init_settings_dict[setting_name]
        setting_type = entry.get('type', 'None')
        try:
          one_dict = nepi_controls.create_controls_dict({setting_name: copy.deepcopy(entry)})
        except Exception as e:
          self.msg_if.pub_error("Setting " + setting_name + " (" + str(setting_type) +
                                ") raised building its control: " + str(e))
          dropped_settings[setting_name] = self.describeDropped(setting_name, entry,
                                                                "raised: " + str(e))
          continue

        if setting_name in one_dict.keys():
          settings_dict[setting_name] = one_dict[setting_name]
          self.value_store[setting_name] = entry.get('default')
        else:
          if setting_type not in nepi_controls.CONTROL_TYPES:
            reason = ("type '" + str(setting_type) + "' is not in nepi_controls.CONTROL_TYPES " +
                      str(nepi_controls.CONTROL_TYPES))
          else:
            reason = ("create_controls_dict() dropped it inside its own bare except, " +
                      "so no exception reached this node")
          self.msg_if.pub_error("Setting " + setting_name + " (" + str(setting_type) +
                                ") DID NOT SURVIVE create_controls_dict(): " + reason)
          dropped_settings[setting_name] = self.describeDropped(setting_name, entry, reason)

      self.dropped_settings = dropped_settings

      settings_dict_values = nepi_controls.get_controls_values_dict(settings_dict)
      self.msg_if.pub_info("Declared " + str(len(init_settings_dict)) + " settings, " +
                           str(len(settings_dict)) + " survived, " +
                           str(len(dropped_settings)) + " dropped")
      self.msg_if.pub_info("Initialized Settings: " + str(settings_dict_values))
      if len(dropped_settings) > 0:
        self.msg_if.pub_error("DROPPED SETTINGS: " + str(list(dropped_settings.keys())))
      return settings_dict


  def refreshSettingsDict(self):
      settings_dict = copy.deepcopy(self.settings_dict)
      for setting_name in settings_dict.keys():
        entry = self.init_settings_dict.get(setting_name, dict())
        setting_type = entry.get('type', 'None')

        # A Trigger carries no persistent value. set_control_value() stamps it
        # with the current time on every call, so pushing a stored value back
        # through it here would re-stamp the control on every refresh and the
        # read-back box would tick on its own. Leave the control's own stamp.
        if setting_type == 'Trigger':
          continue

        if setting_name in self.value_store.keys():
          value = self.value_store[setting_name]
          if value is not None:
            settings_dict = nepi_controls.set_control_value(settings_dict, setting_name, value)


        bounds = entry.get('bounds')
        if bounds is not None and len(bounds) == 2:
          settings_dict = nepi_controls.set_control_bounds(settings_dict, setting_name, bounds)
      return settings_dict


  def getSettingsFunction(self):
      with self.settings_lock:
        return self.settings_dict


  def setSettingFunction(self, setting_name, setting_value):
      setting_str = setting_name + ":" + str(setting_value)

      if setting_name not in self.settings_dict.keys():
        msg = (self.node_name + " Setting name " + setting_str + " is not supported")
        self.msg_if.pub_info("SET REJECTED " + setting_str + " : not a registered setting")
        return [False, msg, self.settings_dict]

      old_value = nepi_controls.get_control_value(self.settings_dict, setting_name)

      [valid, reason] = self.validateValue(setting_name, setting_value)
      if valid == False:
        msg = (self.node_name + " rejected " + setting_str + " : " + reason)
        self.msg_if.pub_info("SET REJECTED " + setting_name +
                             " old=" + str(old_value) +
                             " new=" + str(setting_value) +
                             " : " + reason)
        return [False, msg, self.settings_dict]

      with self.settings_lock:
        # A Trigger has no persistent value to store. Firing it is the effect.
        if self.init_settings_dict.get(setting_name, dict()).get('type') != 'Trigger':
          self.value_store[setting_name] = self.coerceValue(setting_name, setting_value)
        else:
          self.settings_dict = nepi_controls.set_control_value(self.settings_dict,
                                                               setting_name, setting_value)
          self.value_store[setting_name] = nepi_controls.get_control_value(self.settings_dict,
                                                                           setting_name)
        self.settings_dict = self.refreshSettingsDict()
        new_value = nepi_controls.get_control_value(self.settings_dict, setting_name)

      msg = (self.node_name + " UPDATED SETTING " + setting_str)
      self.msg_if.pub_info("SET ACCEPTED " + setting_name +
                           " old=" + str(old_value) +
                           " new=" + str(new_value))
      return [True, msg, self.settings_dict]


  def validateValue(self, setting_name, setting_value):
      entry = self.init_settings_dict.get(setting_name, dict())
      setting_type = entry.get('type', 'None')
      options = [str(item) for item in entry.get('options', [])]
      bounds = entry.get('bounds')

      if setting_type == 'Trigger':
        return [True, '']

      if setting_type == 'Menu':
        try:
          index = int(setting_value)
        except Exception as e:
          return [False, "not an integer menu index: " + str(e)]
        if index < 0 or index >= len(options):
          return [False, "menu index out of range 0.." + str(len(options) - 1)]
        return [True, '']

      if setting_type == 'Selection':
        if str(setting_value) not in options:
          return [False, "not in options " + str(options)]
        return [True, '']

      if setting_type == 'Selections' or setting_type == 'Toggles':
        values = setting_value
        if isinstance(values, (list, tuple)) == False:
          values = [values]
        for value in values:
          if str(value) not in options:
            return [False, str(value) + " not in options " + str(options)]
        return [True, '']

      if setting_type == 'Toggle' or setting_type == 'String':
        return [True, '']

      if setting_type == 'Int':
        try:
          value = int(setting_value)
        except Exception as e:
          return [False, "not an integer: " + str(e)]
        if bounds is not None and (value < bounds[0] or value > bounds[1]):
          return [False, "out of bounds " + str(bounds)]
        return [True, '']

      if setting_type == 'Float' or setting_type == 'FloatSlider':
        try:
          value = float(setting_value)
        except Exception as e:
          return [False, "not a float: " + str(e)]
        if bounds is not None and (value < bounds[0] or value > bounds[1]):
          return [False, "out of bounds " + str(bounds)]
        return [True, '']

      if setting_type == 'RangeSlider':
        try:
          low = float(setting_value[0])
          high = float(setting_value[1])
        except Exception as e:
          return [False, "not a pair of floats: " + str(e)]
        if bounds is not None and (low < bounds[0] or high > bounds[1]):
          return [False, "out of bounds " + str(bounds)]
        if low > high:
          return [False, "low bound above high bound"]
        return [True, '']

      return [True, '']


  def coerceValue(self, setting_name, setting_value):
      setting_type = self.init_settings_dict.get(setting_name, dict()).get('type', 'None')
      try:
        if setting_type == 'Menu' or setting_type == 'Int':
          return int(setting_value)
        if setting_type == 'Float' or setting_type == 'FloatSlider':
          return float(setting_value)
        if setting_type == 'RangeSlider':
          return [float(setting_value[0]), float(setting_value[1])]
        if setting_type == 'Toggle':
          return (setting_value == True or str(setting_value).lower() == 'true')
        if setting_type == 'Selections' or setting_type == 'Toggles':
          if isinstance(setting_value, (list, tuple)):
            return [str(item) for item in setting_value]
          return [str(setting_value)]
      except Exception as e:
        self.msg_if.pub_error("Failed to coerce " + setting_name + " value " +
                              str(setting_value) + " : " + str(e))
        return setting_value
      return str(setting_value)


  #**********************
  # LSX device interface

  def launchDeviceIf(self):
      self.device_info_dict["device_name"] = self.device_name
      self.device_info_dict["path"] = self.device_path
      self.device_info_dict["serial_number"] = "settings-test"
      self.device_info_dict["hw_version"] = "virtual"
      self.device_info_dict["sw_version"] = "fixture"

      self.msg_if.pub_info("Launching NEPI LSX interface...")
      self.lsx_if = LSXDeviceIF(
                  device_info = self.device_info_dict,
                  getStatusFunction = self.getStatus,
                  getSettingsFunction = self.getSettingsFunction,
                  setSettingFunction = self.setSettingFunction,
                  factoryControls = self.FACTORY_CONTROLS,
                  data_source_description = 'settings_test_fixture',
                  data_ref_description = 'none',
                  turnOnOffFunction = self.turnOnOff,
                  setIntensityRatioFunction = self.setIntensityRatio,
                  reports_temp = False,
                  reports_power = False
                  )
      self.msg_if.pub_info("... LSX interface running")


  def getStatus(self):
      status_msg = DeviceLSXStatus()
      status_msg.device_node_name = self.node_name
      status_msg.device_name = self.device_info_dict["device_name"]
      status_msg.device_path = self.device_info_dict["path"]
      status_msg.serial_num = self.device_info_dict["serial_number"]
      status_msg.hw_version = self.device_info_dict["hw_version"]
      status_msg.sw_version = self.device_info_dict["sw_version"]
      status_msg.on_off_state = self.on_off_state
      status_msg.standby_state = False
      status_msg.intensity_ratio = self.intensity_ratio
      status_msg.strobe_state = False
      status_msg.blink_state = False
      status_msg.blink_interval = 0
      status_msg.temp_c = 0
      status_msg.power_w = 0

      # publish_status() publishes the message the driver returns here and
      # discards the one the IF prepared, so the settings namespace the IF wrote
      # onto its own status message never reaches the wire unless it is copied
      # across. Without this the RUI has no settings topic to subscribe to.
      if self.lsx_if is not None:
        if_status = getattr(self.lsx_if, 'status_msg', None)
        if if_status is not None:
          status_msg.settings_topic = if_status.settings_topic
          status_msg.navpose_topic = if_status.navpose_topic
          status_msg.save_data_topic = if_status.save_data_topic
          status_msg.data_source_description = if_status.data_source_description
          status_msg.data_ref_description = if_status.data_ref_description
      return status_msg


  def turnOnOff(self, turn_on_off):
      self.on_off_state = (turn_on_off == True)
      return True


  def setIntensityRatio(self, intensity_ratio):
      try:
        value = float(intensity_ratio)
      except Exception as e:
        self.msg_if.pub_warn("Bad intensity ratio " + str(intensity_ratio) + " : " + str(e))
        return False
      if value < 0:
        value = 0.0
      if value > 1:
        value = 1.0
      self.intensity_ratio = value
      return True


  #**********************
  # Read-back page

  def buildPageDict(self):
      with self.settings_lock:
        settings_dict = copy.deepcopy(self.settings_dict)

      present = []
      for setting_name in settings_dict.keys():
        entry = self.init_settings_dict.get(setting_name, dict())
        present.append({
          'name': setting_name,
          'type': settings_dict[setting_name]['type'],
          'range': self.describeRange(entry),
          'default': str(entry.get('default')),
          'value': str(nepi_controls.get_control_value(settings_dict, setting_name))
        })

      dropped = []
      for setting_name in self.dropped_settings.keys():
        dropped.append(self.dropped_settings[setting_name])

      return {
        'node_name': self.node_name,
        'device_name': self.device_info_dict.get("device_name", ""),
        'declared_count': len(self.init_settings_dict),
        'present': present,
        'dropped': dropped,
        'options': self.buildDriverOptions(),
        'control_types': nepi_controls.CONTROL_TYPES,
        'stamp': nepi_utils.get_time()
      }


  def describeRange(self, entry):
      if 'options' in entry.keys():
        return 'options ' + str([str(item) for item in entry['options']])
      if 'bounds' in entry.keys():
        return 'bounds ' + str(entry['bounds'])
      return ''


  def describeDropped(self, setting_name, entry, reason):
      return {
        'name': setting_name,
        'type': str(entry.get('type')),
        'range': self.describeRange(entry),
        'default': str(entry.get('default')),
        'reason': reason
      }


  def renderPage(self):
      return (
        "<html><head><title>NEPI Settings Test Fixture</title></head><body>"
        "<h2>NEPI Settings Test Fixture</h2>"
        "<p>Temporary driver. Drives no hardware. Every box below is a live"
        " read-back of the node's controls dict, polled every "
        + str(PAGE_POLL_MS) + " ms. Change a setting from the RUI settings panel"
        " or from the command line and the box follows it.</p>"
        "<p id='hdr'>loading...</p>"
        "<h3>Registered settings</h3>"
        "<table border='1' cellpadding='4' id='present_tbl'>"
        "<tr><th>name</th><th>type</th><th>options or bounds</th>"
        "<th>default</th><th>current value</th></tr></table>"
        "<h3>Declared but DROPPED before reaching the controls dict</h3>"
        "<p>Anything listed here was declared by this node and did not survive"
        " create_controls_dict(). It will not appear in the RUI settings panel"
        " either. An empty table here is the pass condition.</p>"
        "<table border='1' cellpadding='4' id='dropped_tbl'>"
        "<tr><th>name</th><th>type</th><th>options or bounds</th>"
        "<th>default</th><th>why it was dropped</th></tr></table>"
        "<h3>Driver options received at launch</h3>"
        "<p>These are DISCOVERY options, not device settings. Test them from the"
        " Drivers Manager panel (SYSTEM &rarr; Drivers &rarr; this driver &rarr;"
        " Driver Options), which is where drivers_mgr renders them. This table is"
        " a snapshot of what reached the node when it launched, so it will NOT"
        " follow an option you change in the RUI until the driver is restarted."
        " 'rendered as' differs from 'type' where drivers_mgr rewrites the type"
        " before building the control.</p>"
        "<table border='1' cellpadding='4' id='options_tbl'>"
        "<tr><th>name</th><th>type</th><th>rendered as</th><th>options</th>"
        "<th>default</th><th>value at launch</th><th>type supported</th></tr></table>"
        "<p id='foot'></p>"
        "<script>"
        "function cell(t){var d=document.createElement('td');d.textContent=t;return d;}"
        "function fill(tbl,rows,keys){"
        " while(tbl.rows.length>1){tbl.deleteRow(1);}"
        " for(var i=0;i<rows.length;i++){var tr=tbl.insertRow(-1);"
        "  for(var k=0;k<keys.length;k++){"
        "   if(keys[k]=='value'){var td=document.createElement('td');"
        "    var inp=document.createElement('input');inp.type='text';inp.readOnly=true;"
        "    inp.size=32;inp.value=rows[i][keys[k]];td.appendChild(inp);tr.appendChild(td);}"
        "   else{tr.appendChild(cell(rows[i][keys[k]]));}}}}"
        "function poll(){"
        " var r=new XMLHttpRequest();"
        " r.onreadystatechange=function(){"
        "  if(r.readyState==4&&r.status==200){"
        "   var d=JSON.parse(r.responseText);"
        "   document.getElementById('hdr').textContent="
        "    'node '+d.node_name+'  device '+d.device_name+'  declared '+d.declared_count"
        "    +'  registered '+d.present.length+'  dropped '+d.dropped.length;"
        "   fill(document.getElementById('present_tbl'),d.present,"
        "        ['name','type','range','default','value']);"
        "   fill(document.getElementById('dropped_tbl'),d.dropped,"
        "        ['name','type','range','default','reason']);"
        "   fill(document.getElementById('options_tbl'),d.options,"
        "        ['name','type','rendered_as','range','default','value','supported']);"
        "   document.getElementById('foot').textContent="
        "    'CONTROL_TYPES: '+d.control_types.join(', ');"
        "  }};"
        " r.open('GET','/settings.json',true);r.send();}"
        "poll();setInterval(poll," + str(PAGE_POLL_MS) + ");"
        "</script></body></html>"
      )


  def startPageServer(self):
      node = self

      class PageHandler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.0'

        def do_GET(self):
          if self.path.startswith('/settings.json'):
            body = json.dumps(node.buildPageDict()).encode('utf-8')
            content_type = 'application/json'
          else:
            body = node.renderPage().encode('utf-8')
            content_type = 'text/html; charset=utf-8'
          self.send_response(200)
          self.send_header('Content-Type', content_type)
          self.send_header('Content-Length', str(len(body)))
          self.send_header('Cache-Control', 'no-store')
          self.end_headers()
          self.wfile.write(body)

        def log_message(self, format, *args):
          # Do not mirror every poll into the node log
          return

      class PageServer(ThreadingHTTPServer):
        allow_reuse_address = True
        daemon_threads = True

      try:
        self.page_server = PageServer(('0.0.0.0', self.page_port), PageHandler)
      except OSError as e:
        # A dead page is a nuisance, a dead driver ends the test. Keep running.
        self.msg_if.pub_warn("Settings test page NOT served: port " + str(self.page_port) +
                             " could not be bound (" + str(e) + "). Settings still work." +
                             " Change the page_port driver option or free the port," +
                             " then disable and re-enable the driver.")
        self.page_server = None
        return

      self.page_thread = threading.Thread(target = self.page_server.serve_forever)
      self.page_thread.daemon = True
      self.page_thread.start()
      self.msg_if.pub_info("Settings test page at http://" + str(self.getHostAddress()) +
                           ":" + str(self.page_port) + "/  (JSON at /settings.json)")


  def getHostAddress(self):
      try:
        return socket.gethostname()
      except Exception:
        return 'localhost'


  def stopPageServer(self):
      if self.page_server is not None:
        try:
          self.page_server.shutdown()
          self.page_server.server_close()
          self.msg_if.pub_info("Settings test page server stopped, port " +
                               str(self.page_port) + " released")
        except Exception as e:
          self.msg_if.pub_warn("Failed to stop page server cleanly: " + str(e))
        self.page_server = None


  #**********************
  # Cleanup

  def cleanupActions(self):
      self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
      self.stopPageServer()


if __name__ == '__main__':
  SettingsTestNode()
