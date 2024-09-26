#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# NEPI Auto Discovery Script for Sealite devices

import os
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy
import subprocess
import time
import serial
import serial.tools.list_ports
import socket

from nepi_ros_interfaces.srv import LSXCapabilitiesQuery
from mavros_msgs.srv import VehicleInfoGet
from mavros_msgs.msg import VehicleInfo

from nepi_edge_sdk_base import nepi_nex

PKG_NAME = 'RBX_ARDUPILOT' # Use in display menus
FILE_TYPE = 'DISCOVERY'
CLASS_NAME = 'ArdupilotDiscovery' # Should Match Class Name
PROCESS = 'CALL' # 'LAUNCH', 'RUN', or 'CALL'

TEST_NEX_DICT = {
    'group': 'RBX',
    'group_id': 'ARDU',
    'node_file_name': 'rbx_ardupilot_node.py',
    'node_file_path': '/opt/nepi/ros/lib/nep_drivers',
    'node_module_name': 'rbx_ardupilot_node',
    'node_class_name': 'ArdupilotNode',
    'driver_name': "None",
    'driver_file_name': "None" ,
    'driver_file_path':  "None",
    'driver_module_name': "None" ,
    'driver_class_name':  "None",
    'driver_interfaces': ['USBSERIAL'],
    'driver_options_1': ['Real','SITL'],
    'driver_default_option_1': 'SITL',
    'driver_set_option_1': 'SITL',
    'driver_options_2': [],
    'driver_default_option_2': 'None',
    'driver_set_option_2': 'None',
    'discovery_name': 'LSX_SEALITE', 
    'discovery_file_name': "rbx_ardupilot_discovery.py",
    'discovery_file_path': "/opt/nepi/ros/lib/nep_drivers",
    'discovery_module_name': "rbx_ardupilot_discovery",
    'discovery_class_name': "ArdupilotDiscovery",
    'discovery_method': 'AUTO', 
    'discovery_ids': [],
    'discovery_ignore_ids': ['ttyACM'],
    'device_path': '/dev/ttyUSB0',
    'order': 1,
    'active': True,
    'msg': ""
    }


#########################################
# Sealite Discover Method
#########################################

### Function to try and connect to device and also monitor and clean up previously connected devices
class ArdupilotDiscovery:
  active_devices_dict = dict()
  node_name = "ardupilot"
  baudrate_list = [57600]
  ip_addr_list = ['127.0.0.1']
  ip_port_list = ['5760']
  ################################################          
  def __init__(self):
    self.log_name = PKG_NAME.lower() + "_discovery" 

  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace,nex_dict = TEST_NEX_DICT):
    self.nex_dict = nex_dict
    #rospy.logwarn(self.log_name + ":  Discovery class instantiated with nex_dict " + str(self.nex_dict))
    self.ignore_id_list = self.nex_dict['discovery_ignore_ids']
    self.base_namespace = base_namespace
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.system_type = nex_dict['driver_set_option_1']
    #rospy.logwarn(self.log_name + ":  active path list: " + str(self.active_paths_list))


    ### Purge Unresponsive Connections
    path_purge_list = []
    for path_str in self.active_devices_dict:
        success = self.checkOnDevice(path_str)
        if success == False:
          path_purge_list.append(path_str) 
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del  self.active_devices_dict[path_str]

    ### Checking for devices on available paths
    # Check paths for device type
    #rospy.logwarn(self.log_name + ": Running discovery for type: " + self.system_type)
    if self.system_type == 'Real':
      self.path_list = []
      ports = serial.tools.list_ports.comports()
      for loc, desc, hwid in sorted(ports):
        #rospy.logdebug(self.log_name + ": Found serial_port at: " + loc)
        self.path_list.append(loc)
      for path_str in self.path_list:
        valid_path = True
        for id in self.ignore_id_list:
          if path_str.find(id) != -1 or path_str in self.active_paths_list:
            valid_path = False
        if valid_path:
          #rospy.logwarn(self.log_name + ": Looking for path: " + path_str)
          #rospy.logwarn(self.log_name + ": In path_list: " + str(self.path_list))
          found = self.checkForRealDevice(path_str)
          if found:
            success = self.launchRealDeviceNode(path_str)
            if success:
              self.active_paths_list.append(path_str)
    elif self.system_type == 'SITL':
        for ip_addr_str in self.ip_addr_list:
          for ip_port_str in self.ip_port_list:
            path_str = ip_addr_str + "_" + ip_port_str
            if path_str not in self.active_paths_list:
              #rospy.logwarn(self.log_name + ": Looking for path: " + path_str)
              #rospy.logwarn(self.log_name + ": In path_list: " + str(self.active_paths_list))
              found = self.checkForSITLDevice(path_str)
              if found:
                success = self.launchSITLDeviceNode(path_str)
                if success:
                  self.active_paths_list.append(path_str)

    else:
      return self.active_paths_list
    

    return self.active_paths_list
  ################################################

  ##########  Device specific calls


  def checkForRealDevice(self,path_str):
    found_device = False
    self.comp_id = None
    self.sys_id_mavlink = None
    self.addr_str = None
    # Find serial ports
    rospy.logdebug("Mavlink_AD: : Looking for serial ports on device")
    port_list = []
    ports = serial.tools.list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      rospy.logdebug("Mavlink_AD: : Found serial_port at: " + loc)
      port_list.append(loc)
    # Checking for devices on available serial ports
    for path_str in port_list:
      if path_str not in self.active_paths_list:
        found_device = False
        sys_id_mavlink = 0
        for baud_int in self.baudrate_list:
          rospy.logdebug("Mavlink_AD: : Connecting to serial port " + path_str + " with baudrate: " + str(baud_int))
          try:
            # Try and open serial port
            rospy.logdebug("Mavlink_AD: : Opening serial port " + path_str + " with baudrate: " + str(baud_int))
            serial_port = serial.Serial(path_str,baud_int,timeout = 1)
          except Exception as e:
            rospy.logwarn(self.log_name + ": Unable to open serial port " + path_str + " with baudrate: " + str(baud_int) + "(" + str(e) + ")")
            continue
          
          for i in range(0,500): # Read up to 64 packets waiting for heartbeat
            try:
              #serial_port.read_until(b'\xFE', 255) # This is the MAVLINK_1 magic number
              bytes_read = serial_port.read_until(b'\xFD', 280) # MAVLINK_2 packet start magic number, up to MAVLINK_2 max bytes in packet
              bytes_read_count = len(bytes_read)
            except Exception as e:
              #rospy.logwarn("Mavlink_AD: read_until() failed (%s)", str(e))
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
                found_device = True
                self.baud_int = baud_int
                self.comp_id = comp_id
                self.sys_id_mavlink = sys_id
                self.addr_str = str(sys_id_mavlink)
                rospy.loginfo("Mavlink_AD: Found mavlink autonoumou device at: " + path_str + "_" + self.addr_str)
                break
          # Clean up the serial port
          rospy.logdebug("Mavlink_AD: : Closing serial port " + path_str)
          serial_port.close()
          time.sleep(1)
    return found_device


  


  def checkOnDevice(self,path_str):
    purge_node = False
    if path_str in self.active_devices_dict.keys():
      device_entry = self.active_devices_dict[path_str]
      sysid = device_entry["sysid"] 
      compid = device_entry["compid"]   
      ros_node_name = device_entry["node_name"]
      mavlink_subproc = device_entry["mavlink_subproc"] 
      ardu_subproc = device_entry["ardu_subproc"] 
      fgps_subproc = device_entry["fgps_subproc"] 

      full_node_name = self.base_namespace + '/' + ros_node_name
      
      # Check that the ros_node_name process is still running
      if mavlink_subproc.poll() is not None:
        rospy.logwarn(self.log_name + ": " + "Node process for %s is no longer running... purging from managed list", ros_node_name)
      #rospy.logwarn(self.log_name + ": " + "Node process for %s is no longer running... purging from managed list", ardu_node)
        purge_node = True
      # Check that the node's port still exists
      elif path_str not in self.active_paths_list:
        rospy.logwarn(self.log_name + ": " + "Port %s associated with node %s no longer detected", path_str, ros_node_name)
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

        if path_str in self.active_paths_list:
          rospy.logwarn(self.log_name + ": " + "Removing port %s from active list as part of node purging", path_str)
          self.active_paths_list.remove(path_str)

        if mavlink_subproc.poll() is None:
          rospy.logwarn(self.log_name + ": " + "Issuing sigterm to process for %s as part of node purging", ros_node_name)
          mavlink_subproc.kill()
          # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
          # rosnode cleanup won't find the disconnected node until the process is fully terminated
          try:
            mavlink_subproc.wait(timeout=10)
          except:
            pass        
            
          if ardu_subproc is not None:
            if ardu_subproc.poll() is None:
              rospy.logwarn(self.log_name + ": " + "Issuing sigterm to process for %s as part of node purging", "ardupilot_node")
              ardu_subproc.kill()
              # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
              # rosnode cleanup won't find the disconnected node until the process is fully terminated
              try:
                ardu_subproc.wait(timeout=10)
              except:
                pass

          if fgps_subproc is not None:
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

  def launchRealDeviceNode(self, path_str):
    success = False
    if self.comp_id != None and self.sys_id_mavlink != None and self.addr_str != None:
          ip_str_short = path_str.split('/')[-1]
          mavlink_node_name = "mavlink_" + ip_str_short
          mavlink_node_namespace = self.base_namespace + mavlink_node_name
          rospy.loginfo("Mavlink_AD: Starting mavlink node setup: " + mavlink_node_name)
          # Load the proper configs for APM
          rosparam_load_cmd = ['rosparam', 'load', '/opt/nepi/ros/share/mavros/launch/apm_pluginlists.yaml', mavlink_node_namespace]
          subprocess.run(rosparam_load_cmd)
          rosparam_load_cmd = ['rosparam', 'load', '/opt/nepi/ros/share/mavros/launch/apm_config.yaml', mavlink_node_namespace]
          subprocess.run(rosparam_load_cmd)
          # Adjust the timesync_rate to cut down on log noise
          rospy.set_param(mavlink_node_namespace + '/conn/timesync_rate', 1.0)
          # Allow the HIL plugin. Disabled in apm configs for some reason
          plugin_blacklist = rospy.get_param(mavlink_node_namespace + '/plugin_blacklist')
          if 'hil' in plugin_blacklist:
            plugin_blacklist.remove('hil')
            rospy.set_param(mavlink_node_namespace + '/plugin_blacklist', plugin_blacklist)
          
          # Launch Mavlink Node
          rospy.loginfo("Mavlink_AD: Launching mavlink node: " + mavlink_node_name)
          fcu_url = path_str + ':' + str(self.baud_int)
          gcs_url = ""
          node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + mavlink_node_name, '_fcu_url:=' + fcu_url, '_gcs_url:=' + gcs_url] 
          mav_subproc = subprocess.Popen(node_run_cmd)
         
          # Start the ardupilot RBX node for this mavlink connection
          ardu_node_name = "ardupilot_" + ip_str_short
          rospy.loginfo("Mavlink_AD: " + "Starting ardupilot rbx node: " + ardu_node_name)
          processor_run_cmd = ["rosrun", "nepi_drivers", "rbx_ardupilot_node.py",
                                "__name:=" + ardu_node_name, f"__ns:={self.base_namespace}"]
          ardu_subproc = subprocess.Popen(processor_run_cmd)

          #Start the an RBX fake gps for this ardupilot node
          fgps_node_name = "fake_gps_" + ip_str_short
          rospy.loginfo("Mavlink_AD: " + "Starting fake gps rbx node: " + fgps_node_name)
          processor_run_cmd = ["rosrun", "nepi_edge_sdk_base", "fake_gps.py",
                                "__name:=" + fgps_node_name, f"__ns:={self.base_namespace}"]
          fgps_subproc = subprocess.Popen(processor_run_cmd)


          # And make sure it actually starts up fully by waiting for a guaranteed service
          vehicle_info_service_name = mavlink_node_namespace + '/vehicle_info_get'
          try:
            rospy.wait_for_service(vehicle_info_service_name, timeout=10) # TODO: 10 seconds always sufficient for the driver?
            # No exception, all good
            device_entry = dict()
            device_entry["sysid"] = self.sys_id_mavlink
            device_entry["compid"] =  self.comp_id   
            device_entry["node_name"] =  mavlink_node_name
            device_entry["mavlink_subproc"] = mav_subproc
            device_entry["ardu_subproc"] = ardu_subproc
            device_entry["fgps_subproc"] = fgps_subproc

            self.active_devices_dict[path_str] = device_entry
            success = True
          except Exception as e:
            rospy.logerr(self.log_name + ": Failed to start " + mavlink_node_name + " " + str(e))
    return success



  def checkForSITLDevice(self,path_str):
    found_device = False
    [ip_addr_str,ip_port_str] = path_str.split("_")
    ip_addr_str_list = ip_addr_str.split('.')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    result = sock.connect_ex((ip_addr_str,int(ip_port_str)))
    if result == 0:
        found_device = True
        rospy.logwarn("Mavlink_AD: Found device on ip address: " + ip_addr_str + " port: " + ip_port_str + " is open")
        sock.close()
    else:
      rospy.logwarn("Mavlink_AD: Did not find device on ip address: " + ip_addr_str + " port: " + ip_port_str)
    return found_device
    



  def launchSITLDeviceNode(self, path_str):
    success = False
    [ip_addr_str,ip_port_str] = path_str.split("_")
    ip_addr_str_list = ip_addr_str.split('.')
    ip_str_short = ''.join(ip_addr_str_list)


    rospy.logdebug("Mavlink_AD: : Connecting to ardupilot simulator at ip address: " + ip_addr_str + " port: " + ip_port_str)

    mavlink_node_name = "mavlink_"  + ip_str_short
    mavlink_node_namespace = self.base_namespace + mavlink_node_name
    rospy.loginfo("Mavlink_AD: Starting mavlink node setup: " + mavlink_node_name)
    # Load the proper configs for APM
    rosparam_load_cmd = ['rosparam', 'load', '/opt/nepi/ros/share/mavros/launch/apm_pluginlists.yaml', mavlink_node_namespace]
    subprocess.run(rosparam_load_cmd)
    rosparam_load_cmd = ['rosparam', 'load', '/opt/nepi/ros/share/mavros/launch/apm_config.yaml', mavlink_node_namespace]
    subprocess.run(rosparam_load_cmd)
    # Adjust the timesync_rate to cut down on log noise
    rospy.set_param(mavlink_node_namespace + '/conn/timesync_rate', 1.0)
    # Allow the HIL plugin. Disabled in apm configs for some reason
    
    # Launch Mavlink Node
    rospy.loginfo("Mavlink_AD: Launching mavlink node: " + mavlink_node_name)
    #fcu_url = "udp://14555@014550"
    #fcu_url = "udp://14555@0.0.0.0.0:14550"
    #fcu_url = "udp://" + ip_addr_str + ":14555@0.0.0.0.0:14550"
    #fcu_url = "tcp://" + ip_addr_str + ":5760"
    #fcu_url = "udp://:14540@14557"
    fcu_url = "udp://:14540@127.0.0.1:14557"
    #fcu_url = "tcp://" + ip_addr_str + ":" + ip_port_str
    #fcu_url = "udp://" + ip_addr_str + ":" + ip_port_str
    gcs_url = ""
    node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + mavlink_node_name, '_fcu_url:=' + fcu_url ] #, '_gcs_url:=' + gcs_url] 
    mav_subproc = subprocess.Popen(node_run_cmd)
    time.sleep(5)
    vehicle_info_service_name = mavlink_node_namespace + '/vehicle_info_get'
    try:
      rospy.wait_for_service(vehicle_info_service_name, timeout=10) # TODO: 10 seconds always sufficient for the driver?
    except Exception as e:
      rospy.logerr(self.log_name + ": Failed to start " + mavlink_node_name + " " + str(e))

    # Now check that the node is actually responsive
    # Use a service call so that we can provide are assured of synchronous response
    vehicle_info_service_name = full_node_name + '/vehicle_info_get'
    vehicle_info_query = rospy.ServiceProxy(vehicle_info_service_name, VehicleInfoGet)
    try:
      # We don't actually care about the contents of the response at this point, but we might in the future for
      # additional aliveness check logic:
      #response = capability_service()
      response = vehicle_info_query(sysid=sysid, compid=compid, get_all=True)
      vehicle_list = response.vehicles
      if len(vehicle_list):
        vehicle = vehicle_list[0]
        rospy.loginfo("Mavlink_AD: " + "Found robot info: " + str(vehicle))
        sysid = vehicle.sysid
        compid = vehicle.conpid
      success = True
    except Exception as e: # Any exception indicates that the service call failed
      rospy.logwarn(self.log_name + ": " + " Node %s is no longer responding to vehicle info queries (%s)", ros_node_name, str(e))
      purge_node = True  

      # No exception, all good
      device_entry = dict()
      device_entry["sysid"] = 1
      device_entry["compid"] =  1   
      device_entry["node_name"] =  mavlink_node_name
      device_entry["mavlink_subproc"] = mav_subproc
      device_entry["ardu_subproc"] = None
      device_entry["fgps_subproc"] = None


    if success:
      # Start the ardupilot RBX node for this mavlink connection
      ardu_node_name = "ardupilot_" + ip_str_short
      rospy.loginfo("Mavlink_AD: " + "Starting ardupilot rbx node: " + ardu_node_name)
      processor_run_cmd = ["rosrun", "nepi_drivers", "rbx_ardupilot_node.py",
                            "__name:=" + ardu_node_name, f"__ns:={self.base_namespace}"]
      ardu_subproc = subprocess.Popen(processor_run_cmd)
      device_entry["ardu_subproc"] = ardu_subproc
      self.active_devices_dict[path_str] = device_entry
      # And make sure it actually starts up fully by waiting for a guaranteed service
    else:
        if mavlink_subproc.poll() is None:
          rospy.logwarn(self.log_name + ": " + "Issuing sigterm to process for %s as part of node purging", ros_node_name)
          mavlink_subproc.kill()
          # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
          # rosnode cleanup won't find the disconnected node until the process is fully terminated
          try:
            mavlink_subproc.wait(timeout=10)
          except:
            pass              
    return success


  def launchSTILDeviceNode(self, path_str):
    success = False
    return success


#########################################
# Main
#########################################
if __name__ == '__main__':
  from nepi_edge_sdk_base import nepi_ros
  bn = nepi_ros.get_base_namespace()
  ardu = ArdupilotDiscovery()
  print(ardu.discoveryFunction([],[],bn))
  
