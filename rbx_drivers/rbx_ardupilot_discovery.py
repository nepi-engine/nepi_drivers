#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
import glob
import subprocess
import time
import rospy

from nepi_edge_sdk_base import nepi_nex

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

PKG_NAME = 'RBX_ARDUPILOT' # Use in display menus
FILE_TYPE = 'DISCOVERY'
CLASS_NAME = 'ArdupilotDiscovery' # Should Match Class Name
PROCESS = 'CALL' # 'LAUNCH', 'RUN', or 'CALL'


class ArdupilotDiscovery:
  port_dict = dict()   

  ################################################          
  def __init__(self, nex_dict):
    self.log_name = PKG_NAME.lower() + "_discovery" 
    self.nex_dict = nex_dict
    self.port_id_list = nex_dict[driver_options_1]
    self.baudrate_list = nex_dict[driver_options_2]

  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace):
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    ### Purge Unresponsive Connections
    path_purge_list = []
    for path_str in self.active_devices_dict.keys():
        success = self.checkOnDevice(path_str)
        if success:
          path_purge_list.append(path_str) 
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del  self.active_devices_dict[path_str]

    ### Checking for devices on available paths
    for path_str in self.available_paths_list:
      if path_str not in self.active_paths_list:
        found = self.checkForDevice(path_str)
        if found:
          success = self.launchDeviceNode(path_str)
          if success:
            self.active_paths_list.append(path_str)
    return self.active_paths_list
  ################################################

  ################################################
  ##########  Nex Standard Functions

  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(available_paths_list, active_paths_list,base_namespace):
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    ### Purge Unresponsive Connections
    port_purge_list = []
    for port in self.port_dict.keys():
      if port.find("tty") != -1:
        port_entry = self.port_dict[port]
        active = self.checkOnDevice(port_entry)
        if active == False:
          port_purge_list.append(port) 
    # Clean up the port_dict
    for port in port_purge_list:
      del  self.port_dict[port]


    # Check available serial ports
    rospy.logdebug(self.log_name + ": " + "Looking for serial ports on device")
    port_list = []
    ports = serial.tools.list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      rospy.logdebug(self.log_name + ": " + "Found serial_port at: " + loc)
      port_list.append(loc)
    # Checking for devices on available serial ports
    for port_str in port_list:
      if port_str not in self.active_paths_list:
        [found_active_device, baudrate_str] = self.checkForDevice(port_str)
        # If this is a mavlink, load params and launch the mavros node
        if found_active_device:
          success = self.launchDeviceNode(port_str, baudrate_str)
          if success:
            self.active_paths_list.append(port_str)
    return self.active_paths_list


  ################################################
  ##########  Device specific calls

  def checkOnDevice(self,port_entry):
    active = True
    purge_node = False

    sysid = port_entry["sysid"] 
    compid = port_entry["compid"]   
    ros_node_name = port_entry["node_name"]
    mavlink_subproc = port_entry["mavlink_subproc"] 
    ardu_subproc = port_entry["ardu_subproc"] 
    fgps_subproc = port_entry["fgps_subproc"] 

    full_node_name = self.base_namespace + '/' + ros_node_name
    
    # Check that the ros_node_name process is still running
    if mavlink_subproc.poll() is not None:
      rospy.logwarn(self.log_name + ": " + "Node process for %s is no longer running... purging from managed list", ros_node_name)
    #rospy.logwarn(self.log_name + ": " + "Node process for %s is no longer running... purging from managed list", ardu_node)
      purge_node = True
    # Check that the node's port still exists
    elif port not in self.active_paths_list:
      rospy.logwarn(self.log_name + ": " + "Port %s associated with node %s no longer detected", port, ros_node_name)
      purge_node = True
    else:
      # Now check that the node is actually responsive
      # Use a service call so that we can provide are assured of synchronous response
      vehicle_info_service_name = full_node_name + '/vehicle_info_get'
      vehicle_info_query = rospy.ServiceProxy(vehicle_info_service_name, VehicleInfoGet)
      try:
        # We don't actually care about the contents of the response at this point, but we might in the future for
        # additional aliveness check logic:
        #response = capability_service()
        vehicle_info_query(sysid=sysid, compid=compid, get_all=False)

      except Exception as e: # Any exception indicates that the service call failed
        rospy.logwarn(self.log_name + ": " + " Node %s is no longer responding to vehicle info queries (%s)", ros_node_name, str(e))
        purge_node = True

    if purge_node:
      rospy.logwarn(self.log_name + ": " + "Purging node %s", ros_node_name)

      if port in self.active_paths_list:
        rospy.logwarn(self.log_name + ": " + "Removing port %s from active list as part of node purging", port)
        self.active_paths_list.remove(port)

      if mavlink_subproc.poll() is None:
        rospy.logwarn(self.log_name + ": " + "Issuing sigterm to process for %s as part of node purging", ros_node_name)
        mavlink_subproc.kill()
        # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
        # rosnode cleanup won't find the disconnected node until the process is fully terminated
        try:
          mavlink_subproc.wait(timeout=10)
        except:
          pass        
          

        if ardu_subproc.poll() is None:
          rospy.logwarn(self.log_name + ": " + "Issuing sigterm to process for %s as part of node purging", "ardupilot_node")
          ardu_subproc.kill()
          # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
          # rosnode cleanup won't find the disconnected node until the process is fully terminated
          try:
            ardu_subproc.wait(timeout=10)
          except:
            pass

        if fgps_subproc.poll() is None:
          rospy.logwarn(self.log_name + ": " + "Issuing sigterm to process for %s as part of node purging", "fgps_gps_node")
          fgps_subproc.kill()
          # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
          # rosnode cleanup won't find the disconnected node until the process is fully terminated
          try:
            fgps_subproc.wait(timeout=10)
          except:
            pass

        
        cleanup_proc = subprocess.Popen(['rosnode', 'cleanup'], stdin=subprocess.PIPE)
        try:
          cleanup_proc.communicate(input=bytes("y\r\n", 'utf-8'), timeout=10)
          cleanup_proc.wait(timeout=10) 
        except Exception as e:
          rospy.logwarn(self.log_name + ": " + "rosnode cleanup failed (%s)", str(e))
    active = purge_node == False
    return active


  def checkForDevice(self, port_str)
    found_active_device = False
    sys_id_mavlink = 0
    for baudrate_str in baudrate_list:
      baud_int = int(baudrate_str)
      rospy.logdebug(self.log_name + ": " + "Connecting to serial port " + port_str + " with baudrate: " + str(baud_int))
      try:
        # Try and open serial port
        rospy.logdebug(self.log_name + ": " + "Opening serial port " + port_str + " with baudrate: " + str(baud_int))
        serial_port = serial.Serial(port_str,baud_int,timeout = 1)
      except Exception as e:
        rospy.logwarn(self.log_name + ": " + ": Unable to open serial port " + port_str + " with baudrate: " + str(baud_int) + "(" + str(e) + ")")
        continue
      
      for i in range(0,500): # Read up to 64 packets waiting for heartbeat
        try:
          #serial_port.read_until(b'\xFE', 255) # This is the MAVLINK_1 magic number
          bytes_read = serial_port.read_until(b'\xFD', 280) # MAVLINK_2 packet start magic number, up to MAVLINK_2 max bytes in packet
          bytes_read_count = len(bytes_read)
        except Exception as e:
          #rospy.logwarn(self.log_name + ": " + "read_until() failed (%s)", str(e))
          continue

        if bytes_read_count == 0 or bytes_read_count == 255: # Timed out or read the max mavlink bytes in a packet
          rospy.logdebug('read %d bytes without finding the mavlink packet delimiter... assuming there is no mavlink on this port/baud combo', bytes_read_count)
          break

        try:
          pkt_hdr = serial_port.read(9) # MAVLINK_2 packet header length
        except Exception as e:
          print("read failed (" + str(e) + ")")
          continue

        # Initialize as a non-heartbeat packet
        pkt_len = 255
        comp_id = 255
        msg_id_l = 255

        sys_id = 0
        if len(pkt_hdr) == 9:
          #print(''.join('{:02x}'.format(x) for x in pkt_hdr))
          # This decoding assumes mavlink_2 format packet
          pkt_len = pkt_hdr[0]
          sys_id = pkt_hdr[4]
          comp_id = pkt_hdr[5]
          msg_id_l, msg_id_m, msg_id_h = pkt_hdr[6], pkt_hdr[7], pkt_hdr[8]
        
        # Identify a heartbeat packet by tell-tale signs
        if pkt_len == 9 and msg_id_l == 0x0 and msg_id_m == 0x0 and msg_id_h == 0x0: # Heartbeat message id = 0x00 00 00
          print(str(i) + ": HTBT, sys_id = " + str(sys_id) + ', comp_id = ' + str(comp_id))
          if sys_id > 0 and sys_id < 240:
            found_active_device = True
            sys_id_mavlink = sys_id
            addr_str = str(sys_id_mavlink)
            rospy.loginfo(self.log_name + ": " + "Found mavlink device at: " + port_str + "_" + addr_str)
            break
          
        
      # Clean up the serial port
      rospy.logdebug(self.log_name + ": " + "Closing serial port " + port_str)
      serial_port.close()
      nepi_ros.sleep(1,10)

    return found_active_device, baudrate_str

  def launchDeviceNode(self, port_str, baudrate_str)
    addr_str = str(sys_id_mavlink)
    port_str_short = port_str.split('/')[-1]
    ros_node_name = "mavlink_" + port_str_short
    node_name_namespace = self.base_namespace + ros_node_name
    rospy.loginfo(self.log_name + ": " + "Starting mavlink node setup: " + ros_node_name)
    # Load the proper configs for APM
    rosparam_load_cmd = ['rosparam', 'load', '/opt/nepi/ros/share/mavros/launch/apm_pluginlists.yaml', node_name_namespace]
    subprocess.run(rosparam_load_cmd)
    rosparam_load_cmd = ['rosparam', 'load', '/opt/nepi/ros/share/mavros/launch/apm_config.yaml', node_name_namespace]
    subprocess.run(rosparam_load_cmd)
    # Adjust the timesync_rate to cut down on log noise
    rospy.set_param(node_name_namespace + '/conn/timesync_rate', 1.0)
    # Allow the HIL plugin. Disabled in apm configs for some reason
    plugin_blacklist = rospy.get_param(node_name_namespace + '/plugin_blacklist')
    if 'hil' in plugin_blacklist:
      plugin_blacklist.remove('hil')
      rospy.set_param(node_name_namespace + '/plugin_blacklist', plugin_blacklist)
    
    # Launch Mavlink Node
    rospy.loginfo(self.log_name + ": " + "Launching mavlink node: " + ros_node_name)
    fcu_url = port_str + ':' + str(baud_int)
    gcs_url = ""
    node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + ros_node_name, '_fcu_url:=' + fcu_url, '_gcs_url:=' + gcs_url] 
    mav_subproc = subprocess.Popen(node_run_cmd)
  
    # Start the ardupilot RBX node for this mavlink connection
    ardu_node_name = "ardupilot_" + port_str_short
    rospy.loginfo(self.log_name + ": " + "" + "Starting ardupilot rbx node: " + ardu_node_name)
    processor_run_cmd = ["rosrun", "nepi_drivers_rbx", "ardupilot_rbx_node.py",
                          "__name:=" + ardu_node_name, f"__ns:={self.base_namespace}"]
    ardu_subproc = subprocess.Popen(processor_run_cmd)

    #Start the an RBX fake gps for this ardupilot node
    fgps_node_name = "fake_gps_" + port_str_short
    rospy.loginfo(self.log_name + ": " + "" + "Starting fake gps rbx node: " + fgps_node_name)
    processor_run_cmd = ["rosrun", "nepi_drivers_rbx", "rbx_fake_gps.py",
                          "__name:=" + fgps_node_name, f"__ns:={self.base_namespace}"]
    fgps_subproc = subprocess.Popen(processor_run_cmd)
    # And make sure it actually starts up fully by waiting for a guaranteed service
    vehicle_info_service_name = node_name_namespace + '/vehicle_info_get'
    try:
      rospy.wait_for_service(vehicle_info_service_name, timeout=10) # TODO: 10 seconds always sufficient for the driver?
      # No exception, all good
      port_entry = copy.deepcopy(mav_port_entry)
      port_entry["sysid"] = sys_id_mavlink
      port_entry["compid"] =  comp_id   
      port_entry["node_name"] =  ros_node_name
      port_entry["mavlink_subproc"] = mav_subproc
      port_entry["ardu_subproc"] = ardu_subproc
      port_entry["fgps_subproc"] = fgps_subproc
      self.port_dict[port_str] = port_entry

      return True
    except:
      rospy.logerr("%s: Failed to start %s", self.log_name, ros_node_name)
      return False



  def launchSimNode(self)
    success = False
    # Find serial ports
    rospy.logdebug(self.log_name + ": " + "Looking for ip connections for ardupilot simulators")
    # Checking for devices on available serial ports
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for ip_str in ip_port_dict.keys():
      ip_str_list = ip_str.split('.')
      ip_port= ip_port_dict[ip_str]
      ip_port_str = str(ip_port)
      ip_port_short_str = ''.join(ip_str_list)  + "_" + ip_port_str
      if ip_port_short_str not in self.port_dict.keys():
        result = sock.connect_ex((ip_str,ip_port))
        if result == 0:
          rospy.logwarn(self.log_name + ": " + "Port ip address: " + ip_str + " port: " + ip_port_str + " is open")
          sock.close()

          sys_id_mavlink = 0

          rospy.logdebug(self.log_name + ": " + "Connecting to ardupilot simulator at ip address: " + ip_str + " port: " + ip_port_str)
          addr_str = str(sys_id_mavlink)

          ros_node_name = "mavlink_SIM"  + ip_port_short_str
          node_name_namespace = self.base_namespace + ros_node_name
          rospy.loginfo(self.log_name + ": " + "Starting mavlink node setup: " + ros_node_name)
          # Load the proper configs for APM
          rosparam_load_cmd = ['rosparam', 'load', '/opt/nepi/ros/share/mavros/launch/apm_pluginlists.yaml', node_name_namespace]
          subprocess.run(rosparam_load_cmd)
          rosparam_load_cmd = ['rosparam', 'load', '/opt/nepi/ros/share/mavros/launch/apm_config.yaml', node_name_namespace]
          subprocess.run(rosparam_load_cmd)
          # Adjust the timesync_rate to cut down on log noise
          rospy.set_param(node_name_namespace + '/conn/timesync_rate', 1.0)
          # Allow the HIL plugin. Disabled in apm configs for some reason
          
          # Launch Mavlink Node
          rospy.loginfo(self.log_name + ": " + "Launching mavlink node: " + ros_node_name)
          #fcu_url = "udp://14555@014550"
          #fcu_url = "udp://14555@0.0.0.0.0:14550"
          #fcu_url = "udp://" + ip_str + ":14555@0.0.0.0.0:14550"
          #fcu_url = "tcp://" + ip_str + ":5760"
          fcu_url = "tcp://" + ip_str + ":" + ip_port_str
          gcs_url = ""
          node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + ros_node_name, '_fcu_url:=' + fcu_url ] #, '_gcs_url:=' + gcs_url] 
          mav_subproc = subprocess.Popen(node_run_cmd)

        
          # Start the ardupilot RBX node for this mavlink connection
          ardu_node_name = "ardupilot_SIM"   + ip_port_short_str 
          rospy.loginfo(self.log_name + ": " + "" + "Starting ardupilot rbx node: " + ardu_node_name)
          processor_run_cmd = ["rosrun", "nepi_drivers_rbx", "ardupilot_rbx_node.py",
                                "__name:=" + ardu_node_name, f"__ns:={self.base_namespace}"]
          ardu_subproc = subprocess.Popen(processor_run_cmd)


          #Start the an RBX fake gps for this ardupilot node
          fgps_node_name = "fake_gps_SIM"   + ip_port_short_str 
          rospy.loginfo(self.log_name + ": " + "" + "Starting fake gps rbx node: " + fgps_node_name)
          processor_run_cmd = ["rosrun", "nepi_drivers_rbx", "rbx_fake_gps.py",
                                "__name:=" + fgps_node_name, f"__ns:={self.base_namespace}"]
          fgps_subproc = subprocess.Popen(processor_run_cmd)


          # And make sure it actually starts up fully by waiting for a guaranteed service
          vehicle_info_service_name = node_name_namespace + '/vehicle_info_get'
          #try:
            #rospy.wait_for_service(vehicle_info_service_name, timeout=10) # TODO: 10 seconds always sufficient for the driver?


          port_entry = copy.deepcopy(mav_port_entry)

          port_entry["sysid"] = sys_id_mavlink
          port_entry["compid"] =  ip_port_short_str 
          port_entry["node_name"] =  ros_node_name
          port_entry["mavlink_subproc"] = mav_subproc

          port_entry["ardu_subproc"] = ardu_subproc
          port_entry["fgps_subproc"] = fgps_subproc

          self.port_dict[ip_port_short_str] = port_entry

            #break # Don't check any more baud rates since this one was already successful
          #except:
            #rospy.logerr("%s: Failed to start %s", self.log_name, ros_node_name)
          success = True
        else:
          #rospy.logwarn(self.log_name + ": " + " Ip address: " + ip_str + " port: " + ip_port_str + " is not open")
  return success

    


        
      

 
