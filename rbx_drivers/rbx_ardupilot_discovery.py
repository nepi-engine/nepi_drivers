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

import os
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy
import subprocess
import time
import serial
import serial.tools.list_ports
import socket

from mavros_msgs.srv import VehicleInfoGet
from mavros_msgs.msg import VehicleInfo

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_drv

PKG_NAME = 'RBX_ARDUPILOT' # Use in display menus
FILE_TYPE = 'DISCOVERY'


#########################################
# Sealite Discover Method
#########################################

### Function to try and connect to device and also monitor and clean up previously connected devices
class ArdupilotDiscovery:
  active_devices_dict = dict()
  baudrate_list = ['57600']
  ip_addr_list = ['192.168.179.5'] #['127.0.0.1']
  ip_udp_port_list = ['14550']
  ip_tcp_port_list = ['14550']

  includeDevices = []
  excludedDevices = ['ttyACM']
  ################################################          
  def __init__(self):
    self.log_name = PKG_NAME.lower() + "_discovery" 
    nepi_msg.createMsgPublishers(self)
    time.sleep(1)
    nepi_msg.publishMsgInfo(self, ":" + self.log_name + ": Starting Initialization")
    nepi_msg.publishMsgInfo(self, ":" + self.log_name + ": Initialization Complete")


  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace, drv_dict):
    self.drv_dict = drv_dict
    #nepi_msg.printMsg(self.log_name + "Got drv_dict : " + str(self.drv_dict))
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    
    # Get required data from drv_dict

    ########################
    # Get discovery options
    try:
      #nepi_msg.publishMsgWarn(self, ": " + self.log_name + ": Starting discovery with drv_dict " + str(drv_dict))#
      connection_type = drv_dict['DISCOVERY_DICT']['OPTIONS']['connection']['value']
      self.enable_fake_gps = drv_dict['DISCOVERY_DICT']['OPTIONS']['fake_gps']['value']
    except Exception as e:
      nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Failed to load options " + str(e))#
      return None
    ########################


    ### Purge Unresponsive Connections
    path_purge_list = []
    for path_str in self.active_devices_dict:
        success = True #self.checkOnDevice(path_str)
        if success == False:
          path_purge_list.append(path_str) 
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del  self.active_devices_dict[path_str]
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)
    

    ### Checking for devices on available paths
    # Check paths for device type
    #nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Running discovery for type: " + connection_type)

    # RUN SERIAL PROCESSES
    if connection_type == 'SERIAL':
      self.path_list = []
      ports = serial.tools.list_ports.comports()
      for loc, desc, hwid in sorted(ports):
        #nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Found serial_port at: " + loc)
        self.path_list.append(loc)
      for path_str in self.path_list:
        valid_path = True
        if path_str in self.active_paths_list:
          valid_path = False
        if valid_path:
          for exlcude_device in self.excludedDevices:
            if path_str.find(exlcude_device) != -1:
              valid_path = False
          if valid_path:
            #nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Looking for path: " + path_str)
            #nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": In path_list: " + str(self.path_list))
            [found_device, path_str, comp_id, sys_id, baud_str] = self.checkForSerialDevice(path_str)
            if found_device:
              success = False
              success = self.launchSerialDeviceNode(path_str, comp_id, sys_id, baud_str)
              nepi_msg.printMsgInfo(self.log_name + ":Serial mavlink luanch process returned: " + str(success))
              if success:
                self.active_paths_list.append(path_str)
      #nepi_msg.publishMsgWarn(self, ":  " + self.log_name + ":Updated serial active path: " + str(self.active_paths_list))
    # RUN IP PROCESSES
    elif connection_type == 'TCP' or connection_type == "UDP":

      ip_addr_list = self.ip_addr_list
      '''
      # Try and get list of NEPI enabled IP Ports
      # And make sure it actually starts up fully by waiting for a guaranteed service
      nepi_ipaddr_service_name = self.base_namespace + '/ip_addr_query'
      try:
        get_ipaddr_service = rospy.ServiceProxy(nepi_ipaddr_service_name, IPAddrQuery)
        response = get_ipaddr_service(IPAddrQueryRequest())
        ip_addr_list = response.ip_addrs
        for i, addr in enumerate(ip_addr_list):
          addr = ip_addr_list[i].split("/")[0]
          addr = addr.rsplit(".",1)[0] + ".5"
          ip_addr_list[i] = addr
        nepi_msg.publishMsgWarn(self, ":  " + self.log_name + "Got IP list from service call: " + str(ip_addr_list))
      except rospy.ServiceException as e:
        nepi_msg.publishMsgWarn(self, ":  " + self.log_name + "IP Addr service call failed, falling back to backup ip list: " + str(self.ip_addr_list) + " " + str(e))
      '''
      # RUN TCP PROCESSES
      if connection_type == 'TCP':
        for ip_addr_str in ip_addr_list:
          for ip_port_str in self.ip_tcp_port_list:
            path_str = connection_type + "_" + ip_addr_str + "_" + ip_port_str
            if path_str not in self.active_paths_list:
              #nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Looking for path: " + path_str)
              #nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": In path_list: " + str(self.active_paths_list))
              [found_device, path_str] = self.checkForTcpDevice(path_str)
              if found_device:
                success = self.launchTcpDeviceNode(path_str)
                if success:
                  self.active_paths_list.append(path_str)
      # RUN UDP PROCESSES
      elif connection_type == 'UDP':
        for ip_addr_str in ip_addr_list:
          for ip_udp_port_str in self.ip_udp_port_list:
            path_str = connection_type + "_" + ip_addr_str + "_" + ip_udp_port_str
            if path_str not in self.active_paths_list:
              #nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Looking for path: " + path_str)
              #nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": In path_list: " + str(self.active_paths_list))
              [found_device, path_str] = self.checkForUdpDevice(path_str)
              if found_device:
                success = self.launchUdpDeviceNode(path_str)
                if success:
                  self.active_paths_list.append(path_str)
    # Wrap Up
    #nepi_msg.publishMsgWarn(self, ":  " + self.log_name + ":Completed discovery process with active path: " + str(self.active_paths_list))
    return self.active_paths_list
  



  ################################################

  ##########  Shared Processes 

  def checkOnDevice(self,path_str):
    purge_node = False
    if path_str in self.active_devices_dict.keys():
      device_entry = self.active_devices_dict[path_str]
      sysid = device_entry["sysid"] 
      compid = device_entry["compid"]   
      ros_node_name = device_entry["mav_node_name"]
      mavlink_subproc = device_entry["mavlink_subproc"] 
      ardu_subproc = device_entry["ardu_subproc"] 
      fgps_subproc = device_entry["fgps_subproc"] 

      full_node_name = self.base_namespace + '/' + ros_node_name
      
      # Check that the ros_node_name process is still running
      if mavlink_subproc.poll() is not None:
        nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Node process for is no longer running... purging from managed list " + ros_node_name)
        purge_node = True
      # Check that the node's port still exists
      elif path_str not in self.active_paths_list:
        nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Port  associated with node no longer detected " + ros_node_name)
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
          nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Node is no longer responding to vehicle info queries  " + ros_node_name + " "  + str(e))
          purge_node = True

      if purge_node:
        nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": " + "Purging node  " + ros_node_name)

        if path_str in self.active_paths_list:
          nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Removing port from active list as part of node purging  " + path_str)
          self.active_paths_list.remove(path_str)

        if mavlink_subproc.poll() is None:
          nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Issuing sigterm to process for as part of node purging " + ros_node_name)
          mavlink_subproc.kill()
          # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
          # rosnode cleanup won't find the disconnected node until the process is fully terminated
          try:
            mavlink_subproc.wait(timeout=10)
          except:
            pass        
            
          if ardu_subproc is not None:
            if ardu_subproc.poll() is None:
              ardu_subproc.kill()
              # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
              # rosnode cleanup won't find the disconnected node until the process is fully terminated
              try:
                ardu_subproc.wait(timeout=10)
              except:
                pass

          if fgps_subproc is not None:
            if fgps_subproc.poll() is None:
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
            nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": rosnode cleanup failed " + str(e))
      active = purge_node == False
    return active




  def launchDeviceNode(self,path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url,gcs_url):
    success = False
    mav_node_name = "mavlink_" + device_id_str
    mav_node_namespace = self.base_namespace + mav_node_name
    nepi_msg.printMsgInfo(self.log_name + ": Starting mavlink node setup: " + mav_node_name)
    # Load the proper configs for APM
    rosparam_load_cmd = ['rosparam', 'load', '/opt/nepi/ros/share/mavros/launch/apm_pluginlists.yaml', mav_node_namespace]
    subprocess.run(rosparam_load_cmd)
    rosparam_load_cmd = ['rosparam', 'load', '/opt/nepi/ros/share/mavros/launch/apm_config.yaml', mav_node_namespace]
    subprocess.run(rosparam_load_cmd)
    # Adjust the timesync_rate to cut down on log noise
    nepi_ros.set_param(self,mav_node_namespace + '/conn/timesync_rate', 1.0)
    # Allow the HIL plugin. Disabled in apm configs for some reason
    plugin_blacklist = nepi_ros.get_param(self,mav_node_namespace + '/plugin_blacklist')
    if 'hil' in plugin_blacklist:
      plugin_blacklist.remove('hil')
      nepi_ros.set_param(self,mav_node_namespace + '/plugin_blacklist', plugin_blacklist)
    
    # Launch Mavlink Node
    nepi_msg.printMsgInfo(self.log_name + ": Launching mavlink node: " + mav_node_name)

    node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + mav_node_name, '_fcu_url:=' + fcu_url, '_gcs_url:=' + gcs_url] 
    mav_subproc = subprocess.Popen(node_run_cmd)
    
    # Start the ardupilot RBX node for this mavlink connection
    ardu_node_name = None
    ardu_subproc = None

    ardu_node_name = "ardupilot_" + device_id_str
    has_fake_gps_param_namespace = self.base_namespace + ardu_node_name + "/has_fake_gps"
    nepi_ros.set_param(self,has_fake_gps_param_namespace,self.enable_fake_gps)
    nepi_msg.printMsgInfo(self.log_name + ": " + "Starting ardupilot rbx node: " + ardu_node_name)
    processor_run_cmd = ["rosrun", "nepi_drivers", "rbx_ardupilot_node.py",
                          "__name:=" + ardu_node_name, f"__ns:={self.base_namespace}"]
    ardu_subproc = subprocess.Popen(processor_run_cmd)


    fgps_node_name = None
    fgps_subproc = None

    if self.enable_fake_gps:
      #Start the an RBX fake gps for this ardupilot node
      fgps_node_name = "fake_gps_" + device_id_str
      nepi_msg.printMsgInfo(self.log_name + ": " + "Starting fake gps rbx node: " + fgps_node_name)
      processor_run_cmd = ["rosrun", "nepi_drivers", "fake_gps.py",
                            "__name:=" + fgps_node_name, f"__ns:={self.base_namespace}"]
      fgps_subproc = subprocess.Popen(processor_run_cmd)


    # And make sure it actually starts up fully by waiting for a guaranteed service
    vehicle_info_service_name = mav_node_namespace + '/vehicle_info_get'
    success = True
    '''
    try:
      rospy.wait_for_service(vehicle_info_service_name, timeout=10)
      # No exception, all good
    except Exception as e:
      success = False
      nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Failed to start " + mav_node_name + " " + str(e))
    '''
    if success == True:
      device_entry = dict()
      device_entry["sysid"] = mav_sys_id
      device_entry["compid"] =  mav_comp_id   
      device_entry["mav_node_name"] =  mav_node_name
      device_entry["mavlink_subproc"] = mav_subproc
      device_entry["ardu_subproc"] = ardu_subproc
      device_entry["fgps_subproc"] = fgps_subproc

      self.active_devices_dict[path_str] = device_entry
    return success




  ########## SERIAL PROCESSES ############

  def checkForSerialDevice(self,path_str):
    found_device = False
    mav_comp_id = None
    mav_sys_id = None
    # Find serial ports
    found_device = False
    mav_sys_id = 0
    for baud_str in self.baudrate_list:
      baud_int = int(baud_str)
      nepi_msg.printMsgDebug(self.log_name + ": Connecting to serial port " + path_str + " with baudrate: " + baud_str)
      try:
        # Try and open serial port
        nepi_msg.printMsgDebug(self.log_name + ": Opening serial port " + path_str + " with baudrate: " + baud_str)
        serial_port = serial.Serial(path_str,baud_int,timeout = 1)
      except Exception as e:
        nepi_msg.publishMsgWarn(self,  ":" + self.log_name + ": Unable to open serial port " + path_str + " with baudrate: " + baud_str + "(" + str(e) + ")")
        continue
      
      for i in range(0,500): # Read up to 64 packets waiting for heartbeat
        try:
          #serial_port.read_until(b'\xFE', 255) # This is the MAVLINK_1 magic number
          bytes_read = serial_port.read_until(b'\xFD', 280) # MAVLINK_2 packet start magic number, up to MAVLINK_2 max bytes in packet
          bytes_read_count = len(bytes_read)
        except Exception as e:
          continue

        if bytes_read_count == 0 or bytes_read_count == 255: # Timed out or read the max mavlink bytes in a packet
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
            baud_str = baud_str
            mav_comp_id = comp_id
            mav_sys_id = sys_id
            nepi_msg.printMsgInfo(self.log_name + ": Found mavlink autonoumou device at: " + path_str + " with baudrage " + baud_str + " with sys_id " + str(mav_sys_id))
            break
      # Clean up the serial port
      nepi_msg.printMsgDebug(self.log_name + ": Closing serial port " + path_str)
      serial_port.close()
      time.sleep(1)
    return found_device, path_str, mav_comp_id, mav_sys_id, baud_str 


  def launchSerialDeviceNode(self, path_str, mav_comp_id, mav_sys_id, baud_str):
    success = False
    if mav_comp_id != None and mav_sys_id != None:
          device_id_str = path_str.split('/')[-1]
          fcu_url = path_str + ':' + baud_str
          gcs_url = ""
          launch_list_str = str([path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url])
          #nepi_msg.printMsgInfo(self.log_name + ": Launching mavlink with launch list str: " + launch_list_str)
          success = self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url)
    return success

    ########## TCP PROCESSES ############

  def checkForTcpDevice(self,path_str):
    found_device = False
    [con_type,ip_addr_str,ip_port_str] = path_str.split("_")
    nepi_msg.publishMsgWarn(self, ":  " + "Mavlink_AD: Checkinbg TCP connection: " + ip_addr_str + " " + ip_port_str)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2) 
    result = sock.connect_ex((ip_addr_str,int(ip_port_str)))
    if result == 0:
        found_device = True
        nepi_msg.publishMsgWarn(self, ":  " + "Mavlink_AD: Found TCP device on ip address: " + ip_addr_str + " port: " + ip_port_str + " is open")
        sock.close()
    else:
      nepi_msg.publishMsgWarn(self, ":  " + "Mavlink_AD: Did not find TCP device on ip address: " + ip_addr_str + " port: " + ip_port_str)
    return found_device, path_str


  def launchTcpDeviceNode(self, path_str):
    success = False
    [con_type,ip_addr_str,ip_port_str] = path_str.split("_")
    ip_addr_str_list = ip_addr_str.split('.')
    ip_str_short = ''.join(ip_addr_str_list)
    device_id_str = ip_str_short + "_" + ip_port_str
    mav_comp_id = 1
    mav_sys_id = 1
    fcu_url = "tcp://" + ip_addr_str + ":" + ip_port_str
    gcs_url = ""
    success = self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url)
    return success

    ########## UDP PROCESSES ############

  def checkForUdpDevice(self,path_str):
    found_device = False
    found_device = True
    '''
    [con_type,ip_addr_str,ip_port_str] = path_str.split("_")
    result = os.system("netcat -vnzu "+ ip_addr_str +" "+ ip_addr_str)
    nepi_msg.publishMsgWarn(self, ":  " + "Mavlink_AD: netcat returned " + str(result) + " for ip address: " + ip_addr_str + " port: " + ip_port_str + " is open")
    if result == 0:
      found_device = True
      nepi_msg.publishMsgWarn(self, ":  " + "Mavlink_AD: Found UDP device on ip address: " + ip_addr_str + " port: " + ip_port_str + " is open")
    else:
      nepi_msg.publishMsgWarn(self, ":  " + "Mavlink_AD: Did not find UDP device on ip address: " + ip_addr_str + " port: " + ip_port_str)
    '''
    return found_device, path_str


  def launchUdpDeviceNode(self, path_str):
    success = False
    [con_type,ip_addr_str,ip_port_str] = path_str.split("_")
    ip_addr_str_list = ip_addr_str.split('.')
    ip_str_short = ''.join(ip_addr_str_list)
    device_id_str = ip_str_short + "_" + ip_port_str
    mav_comp_id = 1
    mav_sys_id = 1
    #fcu_url = "udp://14555@014550"
    #fcu_url = "udp://14555@0.0.0.0.0:14550"
    #fcu_url = "udp://" + ip_addr_str + ":14555@0.0.0.0.0:14550"
    #fcu_url = "udp://:14540@14557"
    #fcu_url = "udp://:14540@127.0.0.1:14557"
    #fcu_url = "udp://" + ip_addr_str + ":" + ip_port_str
    #fcu_url = "udp://" + ip_addr_str + ":" + ip_port_str + "@"
    fcu_url = "udp://192.168.179.103:14555@192.168.179.5:14550"
    gcs_url = ""
    success = self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url)
    return success




#########################################
# Main
#########################################
if __name__ == '__main__':
  from nepi_sdk import nepi_ros
  bn = nepi_ros.get_base_namespace()
  ardu = ArdupilotDiscovery()
  print(ardu.discoveryFunction([],[],bn))
