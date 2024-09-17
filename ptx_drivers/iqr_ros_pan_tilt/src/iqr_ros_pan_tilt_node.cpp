/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include "iqr_ros_pan_tilt_node.h"
#include "ptx_interface.h"
#include "iqr_ros_pan_tilt/PanTiltDriver.h"

#define NODE_NAME "iqr_pan_tilt"

#define FIXED_MODBUS_ID                 1
#define DEFAULT_DEVICE_PATH             "/dev/iqr_pan_tilt"

#define DEFAULT_FRAME_ID                "iqr_pan_tilt_frame"
#define DEFAULT_YAW_JOINT_NAME          "iqr_pan_tilt_yaw_joint"
#define DEFAULT_PITCH_JOINT_NAME        "iqr_pan_tilt_pitch_joint"
#define DEFAULT_SPEED_RATIO             0.5f

#define DEFAULT_REVERSE_YAW_CONTROL     true
#define DEFAULT_REVERSE_PITCH_CONTROL   true

// Fixed values per driver/documentation
#define MIN_YAW_HARDSTOP_DEG            -60.0f
#define MAX_YAW_HARDSTOP_DEG            60.0f
#define MIN_PITCH_HARDSTOP_DEG          -60.0f
#define MAX_PITCH_HARDSTOP_DEG          60.0f
#define MIN_SPEED_DEG_PER_S             1
#define MAX_SPEED_DEG_PER_S             30

namespace Numurus
{

IqrRosPanTiltNode::IqrRosPanTiltNode() :
    device_path{"device_path", DEFAULT_DEVICE_PATH, this}
{
    // TODO: Ideally the ptx_interface can be dynamically constructed by PTXNode constructor, but how to pass these specific defaults
    // without creating a huge cumbersome constructor for PTXNode. Not obvious how to initialize a PTXSettings object before PTXNode constructor
    // gets called
    PTXSettings settings;
    settings.frame_id = DEFAULT_FRAME_ID;
    settings.yaw_joint_name = DEFAULT_YAW_JOINT_NAME;
    settings.pitch_joint_name = DEFAULT_PITCH_JOINT_NAME;
    settings.min_yaw_hardstop_deg = MIN_YAW_HARDSTOP_DEG;
    settings.max_yaw_hardstop_deg = MAX_YAW_HARDSTOP_DEG;
    settings.min_pitch_hardstop_deg = MIN_PITCH_HARDSTOP_DEG;
    settings.max_pitch_hardstop_deg = MAX_PITCH_HARDSTOP_DEG;
    settings.min_speed_driver_units = MIN_SPEED_DEG_PER_S;
    settings.max_speed_driver_units = MAX_SPEED_DEG_PER_S;
    settings.speed_ratio = DEFAULT_SPEED_RATIO;
    settings.reverse_yaw_control = DEFAULT_REVERSE_YAW_CONTROL;
    settings.reverse_pitch_control = DEFAULT_REVERSE_PITCH_CONTROL;

    PTXCapabilities capabilities;
    capabilities.has_absolute_positioning = true;
    capabilities.has_speed_control = true;
    capabilities.has_homing = true;
    capabilities.has_waypoints = true;

    ptx_interface = new PTXInterface(this, &n, &n_priv, settings, capabilities);
}

IqrRosPanTiltNode::~IqrRosPanTiltNode()
{
  if (driver)
  {
    delete driver;
    driver = nullptr;
  }

  // ptx_interface handled by parent PTXNode destructor
}

void IqrRosPanTiltNode::retrieveParams()
{
  // Call the parent method
  PTXNode::retrieveParams();

  // Do it again here so that the warnings will be logged, but already retrieved in constructor
  device_path.retrieve();
  // Now we can launch the driver
  const std::string dev_path = device_path;
  driver = new IQR::PanTiltDriver(FIXED_MODBUS_ID, dev_path);
}

void IqrRosPanTiltNode::reportPanTiltIdentity() const
{
  IQR::PanTiltStatus pt_st;
  if (true == driver->getStatus(pt_st))
  {
    ROS_INFO("Connected to IQR ROS Pan/Tilt:"
             "\n\tModbus RTU ID: %u"
             "\n\tSerial Num: %s"
             "\n\tH/W Version: %s"
             "\n\tBoard Version: %s"
             "\n\tS/W Version %s",
             pt_st.id, pt_st.serial_num.c_str(), pt_st.hw_version.c_str(), 
             pt_st.bd_version.c_str(), pt_st.sw_version.c_str());
  }
  else
  {
    ROS_ERROR("Failed to connect to IQR Pan/Tilt unit");
  }    
}

void IqrRosPanTiltNode::moveYaw(PTX_DIRECTION direction, float speed, float time_s)
{
    const float yaw_termination_deg = (direction == PTX_DIRECTION_POSITIVE)? MAX_YAW_HARDSTOP_DEG - 1.0f : MIN_YAW_HARDSTOP_DEG + 1.0f;
    float dummy, current_pitch_deg;
    getCurrentPosition(dummy, current_pitch_deg);
    gotoPosition(yaw_termination_deg, current_pitch_deg, speed, time_s);
}

void IqrRosPanTiltNode::movePitch(PTX_DIRECTION direction, float speed, float time_s)
{
    const float pitch_termination_deg = (direction == PTX_DIRECTION_POSITIVE)? MAX_PITCH_HARDSTOP_DEG - 1.0f : MIN_PITCH_HARDSTOP_DEG + 1.0f;
    float dummy, current_yaw_deg;
    getCurrentPosition(current_yaw_deg, dummy);
    gotoPosition(current_yaw_deg, pitch_termination_deg, speed, time_s);
}

void IqrRosPanTiltNode::gotoPosition(float yaw_deg, float pitch_deg, float speed_driver_units, float move_timeout_s)
{
    driver->setPose(yaw_deg, pitch_deg, speed_driver_units);
    move_stop_timer.setPeriod(ros::Duration(move_timeout_s), true);
    move_stop_timer.start();
}

void IqrRosPanTiltNode::getCurrentPosition(float &yaw_deg_out, float &pitch_deg_out)
{
    IQR::PanTiltStatus status;
    driver->getStatus(status);

    yaw_deg_out = status.yaw_now;
    pitch_deg_out = status.pitch_now;    
}

void IqrRosPanTiltNode::getStatus(PTXStatus &status_out)
{
    IQR::PanTiltStatus pt_st;
    if (true == driver->getStatus(pt_st))
    {
        status_out.loop_count = loop_count;
        status_out.serial_num = pt_st.serial_num;
        status_out.hw_version = pt_st.hw_version;
        status_out.sw_version = pt_st.sw_version;

        status_out.yaw_goal = pt_st.yaw_goal;
        status_out.yaw_now = pt_st.yaw_now;
        
        status_out.pitch_goal = pt_st.pitch_goal;
        status_out.pitch_now = pt_st.pitch_now;
        
        status_out.speed_driver_units = pt_st.speed;

        // Error messages -- just report the numeric error codes from driver.
        // TODO: Would be nice to convert these to human-readable messages.
        if (pt_st.driver_ec != 0)
        {
            const std::string driver_err_msg = "Driver error: " + std::to_string(pt_st.driver_ec);
            status_out.driver_errors.push_back(driver_err_msg);
        }
        if (pt_st.encoder_ec != 0)
        {
            const std::string driver_err_msg = "Encoder error: " + std::to_string(pt_st.encoder_ec);
            status_out.driver_errors.push_back(driver_err_msg);            
        }
        if (pt_st.loop_ec != 0)
        {
            const std::string driver_err_msg = "Loop error: " + std::to_string(pt_st.loop_ec);
            status_out.driver_errors.push_back(driver_err_msg);            
        }
    }
    else
    {
        ROS_WARN_THROTTLE(5, "Failed to gather driver status... leaving status report unchanged");
    }
}

bool IqrRosPanTiltNode::inMotion(float &yaw_goal_out, float &pitch_goal_out)
{
    IQR::PanTiltStatus pt_st;
    if (true == driver->getStatus(pt_st))
    {
        yaw_goal_out = pt_st.yaw_goal;
        pitch_goal_out = pt_st.pitch_goal;

        return ((pt_st.yaw_goal != pt_st.yaw_now) || (pt_st.pitch_goal != pt_st.pitch_now));
    }

    ROS_WARN("Failed to gather driver status to determine if in motion... reporting false");
    yaw_goal_out = 0.0f;
    pitch_goal_out = 0.0f;
    return false;   
}

void IqrRosPanTiltNode::stopMotion()
{
    // No driver/hardware support -- Must simply update the goal to current position
    float current_yaw, current_pitch;
    const uint16_t speed = ptx_interface->currentSpeedRatioToDriverUnits();
    driver->getPose(current_yaw, current_pitch);
    driver->setPose(current_yaw, current_pitch, speed);

    // Log after the commands to make the stop as fast as possible
    ROS_INFO("Motion terminated by command");
}

} // namespace Numurus

int main(int argc, char *argv[])
{
    ros::init(argc, argv, NODE_NAME);
	ROS_INFO("Starting the %s node", NODE_NAME);
        
    Numurus::IqrRosPanTiltNode pt_node;
    pt_node.run();
}

