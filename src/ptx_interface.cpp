/*
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
*/
#include "ptx_interface.h"
#include "ptx_node.h"

#include "nepi_ros_interfaces/PTXStatus.h"

#include <tf2/LinearMath/Quaternion.h>

namespace Numurus
{

PTXInterface::PTXInterface(PTXNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh, 
                           const PTXSettings &default_settings, const PTXCapabilities &default_capabilities):
    SDKInterface{parent, parent_pub_nh, parent_priv_nh},
    frame_id{"ptx/frame_id", default_settings.frame_id, parent},
	yaw_joint_name{"ptx/yaw_joint_name", default_settings.yaw_joint_name, parent},
	pitch_joint_name{"ptx/pitch_joint_name", default_settings.pitch_joint_name, parent},
    yaw_home_pos_deg{"ptx/home_position/yaw_deg", 0.0f, parent},
    pitch_home_pos_deg{"ptx/home_position/pitch_deg", 0.0f, parent},
    min_yaw_hardstop_deg{"ptx/limits/min_yaw_hardstop_deg", default_settings.min_yaw_hardstop_deg, parent},
    max_yaw_hardstop_deg{"ptx/limits/max_yaw_hardstop_deg", default_settings.max_yaw_hardstop_deg, parent},
    min_yaw_softstop_deg{"ptx/limits/min_yaw_softstop_deg", default_settings.min_yaw_hardstop_deg, parent},
    max_yaw_softstop_deg{"ptx/limits/max_yaw_softstop_deg", default_settings.max_yaw_hardstop_deg, parent},
    min_pitch_hardstop_deg{"ptx/limits/min_pitch_hardstop_deg", default_settings.min_pitch_hardstop_deg, parent},
    max_pitch_hardstop_deg{"ptx/limits/max_pitch_hardstop_deg", default_settings.max_pitch_hardstop_deg, parent},
    min_pitch_softstop_deg{"ptx/limits/min_pitch_softstop_deg", default_settings.min_pitch_hardstop_deg, parent},
    max_pitch_softstop_deg{"ptx/limits/max_pitch_softstop_deg", default_settings.max_pitch_hardstop_deg, parent},
    speed_ratio{"ptx/speed_ratio", default_settings.speed_ratio, parent},
    reverse_yaw_control{"ptx/reverse_yaw_control", default_settings.reverse_yaw_control, parent},
    reverse_pitch_control{"ptx/reverse_pitch_control", default_settings.reverse_pitch_control, parent},
    has_absolute_positioning{"ptx/capabilities/has_absolute_positioning", default_capabilities.has_absolute_positioning, parent},
    has_speed_control{"ptx/capabilities/has_speed_control", default_capabilities.has_speed_control, parent},
    has_homing{"ptx/capabilities/has_homing", default_capabilities.has_homing, parent},
    has_waypoints{"ptx/capabilities/has_waypoints", default_capabilities.has_waypoints, parent},
    max_speed_driver_units{default_settings.max_speed_driver_units},
    min_speed_driver_units{default_settings.min_speed_driver_units}
{ 
    // Set reset values
    _min_yaw_hardstop_deg = default_settings.min_yaw_hardstop_deg;
    _max_yaw_hardstop_deg = default_settings.max_yaw_hardstop_deg;
    _min_pitch_hardstop_deg = default_settings.min_pitch_hardstop_deg;
    _max_pitch_hardstop_deg = default_settings.max_pitch_hardstop_deg;
    _speed_ratio = default_settings.speed_ratio;
    _reverse_yaw_control = default_settings.reverse_yaw_control;
    _reverse_pitch_control = default_settings.reverse_pitch_control;

    // Initialize the joint_state message
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.effort.resize(2);
    joint_state.name[0] = default_settings.yaw_joint_name;
    joint_state.name[1] = default_settings.pitch_joint_name;

    // Initialize the odometry message
    const std::string name = ros::this_node::getName();
    odometry.header.frame_id = name + "_fixed_frame";
    odometry.child_frame_id = name + "_rotating_frame";
}

PTXInterface::~PTXInterface(){}

void PTXInterface::retrieveParams()
{
    SDKInterface::retrieveParams();

    frame_id.retrieve();
	yaw_joint_name.retrieve();
	pitch_joint_name.retrieve();
    yaw_home_pos_deg.retrieve();
    pitch_home_pos_deg.retrieve();
    min_yaw_hardstop_deg.retrieve();
    max_yaw_hardstop_deg.retrieve();
    min_yaw_softstop_deg.retrieve();
    max_yaw_softstop_deg.retrieve();
    min_pitch_hardstop_deg.retrieve();
    max_pitch_hardstop_deg.retrieve();
    min_pitch_softstop_deg.retrieve();
    max_pitch_softstop_deg.retrieve();
    speed_ratio.retrieve();
    reverse_yaw_control.retrieve();
    reverse_pitch_control.retrieve();

    has_absolute_positioning.retrieve();
    has_speed_control.retrieve();
    has_homing.retrieve();
    has_waypoints.retrieve();

    // Now we can update the capabilities query response
    ptx_caps_query_response.adjustable_speed = has_speed_control;
    ptx_caps_query_response.absolute_positioning = has_absolute_positioning;
    ptx_caps_query_response.homing = has_homing;
    ptx_caps_query_response.waypoints = has_waypoints;

    // Joint state and odometry published if and only if this system has absolute positioning (for position feedback)
    if (ptx_caps_query_response.absolute_positioning == true)
    {
        jointPub = _parent_pub_nh->advertise<sensor_msgs::JointState>("/joint_states", 10);
        odomPub = _parent_priv_nh->advertise<nav_msgs::Odometry>("ptx/odometry", 10);
    }
}

void PTXInterface::initSubscribers()
{
    SDKInterface::initSubscribers();

    subscribers.push_back(_parent_priv_nh->subscribe("reset_device", 3, &PTXInterface::resetAppHandler, this));


    if (has_speed_control == true)
    {
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/set_speed_ratio", 3, &PTXInterface::setSpeedRatioHandler, this));
    }

    if (has_absolute_positioning == true)
    {
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/jog_to_position", 3, &PTXInterface::jogToPositionHandler, this));
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/jog_to_yaw_ratio", 3, &PTXInterface::jogToYawRatioHandler, this));
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/jog_to_pitch_ratio", 3, &PTXInterface::jogToPitchRatioHandler, this));
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/set_hard_limits", 3, &PTXInterface::setHardLimitsHandler, this));
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/set_soft_limits", 3, &PTXInterface::setSoftLimitsHandler, this));
    }

    if (has_homing == true)
    {
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/set_home_position", 3, &PTXInterface::setHomePositionHandler, this));
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/go_home", 3, &PTXInterface::goHomeHandler, this));
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/set_home_position_here", 3, &PTXInterface::setHomePositionHere, this));
    }

    if (has_waypoints == true)
    {
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/set_waypoint", 3, &PTXInterface::setWaypoint, this));
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/set_waypoint_here", 3, &PTXInterface::setWaypointHere, this));
        subscribers.push_back(_parent_priv_nh->subscribe("ptx/goto_waypoint", 3, &PTXInterface::gotoWaypoint, this));
    }

    // All PTX devices must support these operations
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/stop_moving", 3, &PTXInterface::stopMovingHandler, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/jog_timed_yaw", 3, &PTXInterface::jogTimedYaw, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/jog_timed_pitch", 3, &PTXInterface::jogTimedPitch, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/reverse_yaw_control", 3, &PTXInterface::reverseYawControlHandler, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/reverse_pitch_control", 3, &PTXInterface::reversePitchControlHandler, this));
}

void PTXInterface::initPublishers()
{
    SDKInterface::initPublishers();

    statusPub = _parent_priv_nh->advertise<nepi_ros_interfaces::PTXStatus>("ptx/status", 10);
}

void PTXInterface::initServices()
{
    SDKInterface::initServices();

	servicers.push_back(_parent_priv_nh->advertiseService("ptx/capabilities_query", &PTXInterface::capabilitiesQueryHandler, this));
}

bool PTXInterface::positionIsValid(float yaw_deg, float pitch_deg) const
{

    const float min_yaw = min_yaw_softstop_deg;
    const float max_yaw = max_yaw_softstop_deg;
    const float min_pitch = min_pitch_softstop_deg;
    const float max_pitch = max_pitch_softstop_deg;

    if ((yaw_deg < min_yaw) || (yaw_deg > max_yaw) || (pitch_deg < min_pitch) || (pitch_deg > max_pitch))
    {
        return false;
    }

    return true;
}


void PTXInterface::resetAppHandler(const std_msgs::Empty::ConstPtr &msg)
{
    min_yaw_hardstop_deg = _min_yaw_hardstop_deg;
    max_yaw_hardstop_deg = _max_yaw_hardstop_deg;
    min_yaw_softstop_deg = _min_yaw_hardstop_deg;
    max_yaw_softstop_deg = _max_yaw_hardstop_deg;
    min_pitch_hardstop_deg = _min_pitch_hardstop_deg;
    max_pitch_hardstop_deg = _max_pitch_hardstop_deg;
    min_pitch_softstop_deg = _min_pitch_hardstop_deg;
    max_pitch_softstop_deg = _max_pitch_hardstop_deg;
    speed_ratio = _speed_ratio;
    reverse_yaw_control = _reverse_yaw_control;
    reverse_pitch_control = _reverse_pitch_control;
}



void PTXInterface::publishJointStateAndStatus()
{
    PTXStatus status;
    static_cast<PTXNode*>(_parent_node)->getStatus(status);
  
    const float reverse_yaw_int = (reverse_yaw_control)? -1 : 1;  
    const float reverse_pitch_int = (reverse_pitch_control)? -1 : 1;

    if (ptx_caps_query_response.absolute_positioning == true)
    {
        const double yawRad = 0.01745329 * status.yaw_now;
        const double pitchRad = 0.01745329 * status.pitch_now;

        joint_state.header.stamp = ros::Time::now();
        joint_state.position[0] = yawRad * reverse_yaw_control;
        joint_state.position[1] = pitchRad * reverse_pitch_control;
        jointPub.publish(joint_state);

        odometry.header.stamp = ros::Time::now();
        odometry.header.seq = status.loop_count;
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0.0, pitchRad, yawRad);
        tf2_quat.normalize();
        odometry.pose.pose.orientation.x = tf2_quat.getX();
        odometry.pose.pose.orientation.y = tf2_quat.getY();
        odometry.pose.pose.orientation.z = tf2_quat.getZ();
        odometry.pose.pose.orientation.w = tf2_quat.getW();
        odomPub.publish(odometry);
    }

    nepi_ros_interfaces::PTXStatus status_msg;
    status_msg.header.seq = status.loop_count;
    status_msg.header.stamp = ros::Time::now();
    status_msg.header.frame_id = frame_id;
    
    status_msg.serial_num = status.serial_num;
    status_msg.hw_version = status.hw_version;
    status_msg.sw_version = status.sw_version;
    status_msg.speed_ratio = (float)(status.speed_driver_units - min_speed_driver_units) / (max_speed_driver_units - min_speed_driver_units);

    status_msg.reverse_yaw_control = reverse_yaw_control;
    status_msg.yaw_home_pos_deg = yaw_home_pos_deg * reverse_yaw_int;
    status_msg.yaw_goal_deg = status.yaw_goal * reverse_yaw_int;
    status_msg.yaw_now_deg = status.yaw_now * reverse_yaw_int;
    if (reverse_yaw_control){
        status_msg.yaw_min_hardstop_deg = -1 * max_yaw_hardstop_deg;
        status_msg.yaw_max_hardstop_deg = -1 * min_yaw_hardstop_deg;
        status_msg.yaw_min_softstop_deg = -1 * max_yaw_softstop_deg;
        status_msg.yaw_max_softstop_deg = -1 * min_yaw_softstop_deg;
    }
    else{
        status_msg.yaw_min_hardstop_deg = min_yaw_hardstop_deg;
        status_msg.yaw_max_hardstop_deg = max_yaw_hardstop_deg;
        status_msg.yaw_min_softstop_deg = min_yaw_softstop_deg;
        status_msg.yaw_max_softstop_deg = max_yaw_softstop_deg;
    }
    float yaw_now_ratio =  1 - (status_msg.yaw_now_deg - status_msg.yaw_min_softstop_deg) / (status_msg.yaw_max_softstop_deg - status_msg.yaw_min_softstop_deg) ;
    if (yaw_now_ratio < 0) {
        yaw_now_ratio = 0;
    }
    else if (yaw_now_ratio > 1){
        yaw_now_ratio = 1;
    }       
    status_msg.yaw_now_ratio = yaw_now_ratio;

    float yaw_goal_ratio =  1 - (status_msg.yaw_goal_deg - status_msg.yaw_min_softstop_deg) / (status_msg.yaw_max_softstop_deg - status_msg.yaw_min_softstop_deg) ;
    if (yaw_goal_ratio < 0) {
        yaw_goal_ratio = 0;
    }
    else if (yaw_goal_ratio > 1){
        yaw_goal_ratio = 1;
    }       
    status_msg.yaw_goal_ratio = yaw_goal_ratio;



    status_msg.reverse_pitch_control = reverse_pitch_control;
    status_msg.pitch_home_pos_deg = pitch_home_pos_deg * reverse_pitch_int;
    status_msg.pitch_goal_deg = status.pitch_goal * reverse_pitch_int;
    status_msg.pitch_now_deg = status.pitch_now * reverse_pitch_int;
    if (reverse_pitch_control){
        status_msg.pitch_min_hardstop_deg = -1 * max_pitch_hardstop_deg;
        status_msg.pitch_max_hardstop_deg = -1 * min_pitch_hardstop_deg;
        status_msg.pitch_min_softstop_deg = -1 * max_pitch_softstop_deg;
        status_msg.pitch_max_softstop_deg = -1 * min_pitch_softstop_deg;
    }
    else{
        status_msg.pitch_min_hardstop_deg = min_pitch_hardstop_deg;
        status_msg.pitch_max_hardstop_deg = max_pitch_hardstop_deg;
        status_msg.pitch_min_softstop_deg = min_pitch_softstop_deg;
        status_msg.pitch_max_softstop_deg = max_pitch_softstop_deg;
    }
    float pitch_now_ratio =  1 - (status_msg.pitch_now_deg - status_msg.pitch_min_softstop_deg) / (status_msg.pitch_max_softstop_deg - status_msg.pitch_min_softstop_deg) ;
    if (pitch_now_ratio < 0) {
        pitch_now_ratio = 0;
    }
    else if (pitch_now_ratio > 1){
        pitch_now_ratio = 1;
    }       
    status_msg.pitch_now_ratio = pitch_now_ratio; 

    float pitch_goal_ratio =  1 - (status_msg.pitch_goal_deg - status_msg.pitch_min_softstop_deg) / (status_msg.pitch_max_softstop_deg - status_msg.pitch_min_softstop_deg) ;
    if (pitch_goal_ratio < 0) {
        pitch_goal_ratio = 0;
    }
    else if (pitch_goal_ratio > 1){
        pitch_goal_ratio = 1;
    }       
    status_msg.pitch_goal_ratio = pitch_goal_ratio;

    status_msg.has_adjustable_speed = has_speed_control;
    status_msg.has_position_feedback = has_absolute_positioning;      


    status_msg.error_msgs = status.driver_errors;

    statusPub.publish(status_msg);
}

void PTXInterface::setSpeedRatioHandler(const std_msgs::Float32::ConstPtr &msg)
{
    const bool has_speed = has_speed_control;
    if (false == has_speed)
    {
        ROS_WARN("This pan/tilt unit has no speed control... ignoring");
        return;
    }

    const float requested_speed_ratio = msg->data;
    if (requested_speed_ratio < 0.0f || requested_speed_ratio > 1.0f)
    {
        ROS_WARN("Invalid speed ratio requested (%0.2f): Must be in [0.0, 1.0]", requested_speed_ratio);
        return;
    }

    ROS_DEBUG("Setting speed ratio to %0.2f", requested_speed_ratio);
    speed_ratio = requested_speed_ratio;

    // Preserve current motion goal at new speed
    float yaw_goal, pitch_goal;
    if ( true == static_cast<PTXNode*>(_parent_node)->inMotion(yaw_goal, pitch_goal))
    {
        const float new_speed = currentSpeedRatioToDriverUnits();
        static_cast<PTXNode*>(_parent_node)->gotoPosition(yaw_goal, pitch_goal, new_speed);
    }
}

void PTXInterface::setHomePositionHandler(const nepi_ros_interfaces::PanTiltPosition::ConstPtr &msg)
{

    const float reverse_yaw_int = (reverse_yaw_control)? -1 : 1;  
    const float reverse_pitch_int = (reverse_pitch_control)? -1 : 1;

    const float yaw_deg = msg->yaw_deg * reverse_yaw_int ;
    const float pitch_deg = msg->pitch_deg * reverse_pitch_int;

    if (false == positionIsValid(yaw_deg, pitch_deg))
    {
        ROS_ERROR("Invalid position requested [%0.2f, %0.2f] for new Home position... ignoring", yaw_deg, pitch_deg);
        return;
    }

    ROS_INFO("Updating home position: [%0.2f, %0.2f]", yaw_deg, pitch_deg);
    yaw_home_pos_deg = yaw_deg;
    pitch_home_pos_deg = pitch_deg;
}

void PTXInterface::setSoftLimitsHandler(const nepi_ros_interfaces::PanTiltLimits::ConstPtr &msg)
{

    const float hard_yaw_min = min_yaw_hardstop_deg;
    const float hard_yaw_max = max_yaw_hardstop_deg;
    const float hard_pitch_min = min_pitch_hardstop_deg;
    const float hard_pitch_max = max_pitch_hardstop_deg;

    
    if ((msg->min_yaw_deg < hard_yaw_min) ||
        (msg->max_yaw_deg > hard_yaw_max) ||
        (msg->min_pitch_deg < hard_pitch_min) ||
        (msg->max_pitch_deg > hard_pitch_max))
    {
        ROS_ERROR("Soft limits cannot exceed hard limits... ignoring");
        return;
    }

    ROS_INFO("Updating soft limits: Yaw = [%0.2f, %0.2f], Pitch = [%0.2f, %0.2f]", 
             msg->min_yaw_deg, msg->max_yaw_deg, msg->min_pitch_deg, msg->max_pitch_deg);
    
    min_yaw_softstop_deg = msg->min_yaw_deg;
    max_yaw_softstop_deg = msg->max_yaw_deg;
    min_pitch_softstop_deg = msg->min_pitch_deg;
    max_pitch_softstop_deg = msg->max_pitch_deg;

    // TODO: Move to soft limits if current position not within (or at least report it)?
    // At present, the next valid motion command (relative or absolute) will move system to inside soft limits
}


void PTXInterface::setHardLimitsHandler(const nepi_ros_interfaces::PanTiltLimits::ConstPtr &msg)
{

    const float hard_yaw_min = _min_yaw_hardstop_deg;
    const float hard_yaw_max = _max_yaw_hardstop_deg;
    const float hard_pitch_min = _min_pitch_hardstop_deg;
    const float hard_pitch_max = _max_pitch_hardstop_deg;

    
    if ((msg->min_yaw_deg < hard_yaw_min) ||
        (msg->max_yaw_deg > hard_yaw_max) ||
        (msg->min_pitch_deg < hard_pitch_min) ||
        (msg->max_pitch_deg > hard_pitch_max))
    {
        ROS_ERROR("Soft limits cannot exceed hard limits... ignoring");
        return;
    }

    ROS_INFO("Updating hard and soft limits: Yaw = [%0.2f, %0.2f], Pitch = [%0.2f, %0.2f]", 
             msg->min_yaw_deg, msg->max_yaw_deg, msg->min_pitch_deg, msg->max_pitch_deg);
    
    min_yaw_softstop_deg = msg->min_yaw_deg;
    max_yaw_softstop_deg = msg->max_yaw_deg;
    min_pitch_softstop_deg = msg->min_pitch_deg;
    max_pitch_softstop_deg = msg->max_pitch_deg;

    min_yaw_hardstop_deg = msg->min_yaw_deg;
    max_yaw_hardstop_deg = msg->max_yaw_deg;
    min_pitch_hardstop_deg = msg->min_pitch_deg;
    max_pitch_hardstop_deg = msg->max_pitch_deg;

    // TODO: Move to soft limits if current position not within (or at least report it)?
    // At present, the next valid motion command (relative or absolute) will move system to inside soft limits
}

void PTXInterface::goHomeHandler(const std_msgs::Empty::ConstPtr &msg)
{   
    const float reverse_yaw_int = (reverse_yaw_control)? -1 : 1;  
    const float reverse_pitch_int = (reverse_pitch_control)? -1 : 1;

    ROS_INFO("Returning home by request");
    const float yaw_deg = yaw_home_pos_deg * reverse_yaw_int;
    const float pitch_deg = pitch_home_pos_deg * reverse_pitch_int;
    const uint16_t speed = currentSpeedRatioToDriverUnits();
    
    static_cast<PTXNode*>(_parent_node)->gotoPosition(yaw_deg, pitch_deg, speed);
}

void PTXInterface::jogToPositionHandler(const nepi_ros_interfaces::PanTiltPosition::ConstPtr &msg)
{

    const bool can_position = has_absolute_positioning;
    if (false == can_position)
    {
        ROS_ERROR("This pan/tilt unit does not support absolute positioning... ignoring");
        return;
    }
    const float reverse_yaw_int = (reverse_yaw_control)? -1 : 1;  
    const float reverse_pitch_int = (reverse_pitch_control)? -1 : 1;

    const float yaw_deg = msg->yaw_deg * reverse_yaw_int;
    const float pitch_deg = msg->pitch_deg * reverse_pitch_int;
    const uint16_t speed = currentSpeedRatioToDriverUnits();

    if (false == positionIsValid(yaw_deg, pitch_deg))
    {
        ROS_ERROR("Invalid jog position requested [%0.2f, %0.2f]... ignoring", yaw_deg, pitch_deg);
        return;
    }

    static_cast<PTXNode*>(_parent_node)->gotoPosition(yaw_deg, pitch_deg, speed);    
}

void PTXInterface::jogToYawRatioHandler(const std_msgs::Float32::ConstPtr &msg)
{
    if (msg->data < 0.0f || msg->data > 1.0f)
    {
        ROS_ERROR("Invalid yaw ratio %0.2f... ignoring", msg->data);
        return;
    }
    const bool can_position = has_absolute_positioning;
    if (false == can_position)
    {
        ROS_ERROR("This pan/tilt unit does not support absolute positioning... ignoring");
        return;
    }

    const bool reverse = reverse_yaw_control;
    const float ratio = msg->data;
    const float min_yaw = min_yaw_softstop_deg;
    const float max_yaw = max_yaw_softstop_deg;
    float yaw_goal = 0;
    if (reverse_yaw_control == false){
        yaw_goal =  min_yaw + (1-ratio) * (max_yaw - min_yaw);
    }
    else {
        yaw_goal =  (max_yaw) - (1-ratio) * (max_yaw - min_yaw);
    }
    const float speed = currentSpeedRatioToDriverUnits();
    
    float dummy, pitch_goal;
    static_cast<PTXNode*>(_parent_node)->inMotion(dummy, pitch_goal); // Just to capture pitch_goal
    static_cast<PTXNode*>(_parent_node)->gotoPosition(yaw_goal, pitch_goal, speed);
}

void PTXInterface::jogToPitchRatioHandler(const std_msgs::Float32::ConstPtr &msg)
{
    if (msg->data < 0.0f || msg->data > 1.0f)
    {
        ROS_ERROR("Invalid pitch ratio %0.2f... ignoring", msg->data);
        return;
    }

    const bool can_position = has_absolute_positioning;
    if (false == can_position)
    {
        ROS_ERROR("This pan/tilt unit does not support absolute positioning... ignoring");
        return;
    }

    const bool reverse = reverse_pitch_control;
    const float ratio = msg->data;

    const float min_pitch = min_pitch_softstop_deg;
    const float max_pitch = max_pitch_softstop_deg;
    float pitch_goal = 0;
    if (reverse_pitch_control == false){
        pitch_goal =  min_pitch + (1-ratio) * (max_pitch - min_pitch);
    }
    else {
        pitch_goal =  (max_pitch) - (1-ratio) * (max_pitch - min_pitch);
    }
    const float speed = currentSpeedRatioToDriverUnits();
    
    float dummy, yaw_goal;
    static_cast<PTXNode*>(_parent_node)->inMotion(yaw_goal, dummy); // Just to capture yaw_goal
    static_cast<PTXNode*>(_parent_node)->gotoPosition(yaw_goal, pitch_goal, speed);
}

void PTXInterface::stopMovingHandler(const std_msgs::Empty::ConstPtr &msg)
{
    static_cast<PTXNode*>(_parent_node)->stopMotion();
}

void PTXInterface::jogTimedYaw(const nepi_ros_interfaces::SingleAxisTimedMove::ConstPtr &msg)
{
    if ((msg->direction != msg->DIRECTION_POSITIVE) && (msg->direction != msg->DIRECTION_NEGATIVE))
    {
        ROS_ERROR("Invalid jog direction: %d... ignoring", msg->direction);
        return;
    }

    float current_yaw_deg, current_pitch_deg;
    static_cast<PTXNode*>(_parent_node)->getCurrentPosition(current_yaw_deg, current_pitch_deg);

    const bool reverse = reverse_yaw_control;
    const PTXNode::PTX_DIRECTION direction = (reverse == true)? msg->direction : (-1 * msg->direction);

    const float speed = currentSpeedRatioToDriverUnits();
    const float duration_s = (msg->duration_s < 0.0)? 1000000.0f : msg->duration_s;
    static_cast<PTXNode*>(_parent_node)->moveYaw(direction, speed, duration_s);
    
}

void PTXInterface::jogTimedPitch(const nepi_ros_interfaces::SingleAxisTimedMove::ConstPtr &msg)
{
    if ((msg->direction != msg->DIRECTION_POSITIVE) && (msg->direction != msg->DIRECTION_NEGATIVE))
    {
        ROS_ERROR("Invalid jog direction: %d... ignoring", msg->direction);
        return;
    }

    float current_yaw_deg, current_pitch_deg;
    static_cast<PTXNode*>(_parent_node)->getCurrentPosition(current_yaw_deg, current_pitch_deg);

    const bool reverse = reverse_pitch_control;
    const PTXNode::PTX_DIRECTION direction = (reverse == true)? msg->direction : (-1 * msg->direction);

    const float speed = currentSpeedRatioToDriverUnits();
    const float duration_s = (msg->duration_s < 0.0)? 1000000.0f : msg->duration_s;

    static_cast<PTXNode*>(_parent_node)->movePitch(direction, speed, duration_s);


}

void PTXInterface::reverseYawControlHandler(const std_msgs::Bool::ConstPtr &msg)
{
    ROS_INFO("Updating reverse yaw control");
    reverse_yaw_control = msg->data;
}

void PTXInterface::reversePitchControlHandler(const std_msgs::Bool::ConstPtr &msg)
{
    ROS_INFO("Updating reverse pitch control");
    reverse_pitch_control = msg->data;
}

void PTXInterface::setHomePositionHere(const std_msgs::Empty::ConstPtr &msg)
{
    float current_yaw_deg, current_pitch_deg;
    static_cast<PTXNode*>(_parent_node)->getCurrentPosition(current_yaw_deg, current_pitch_deg);

    const float reverse_yaw_int = (reverse_yaw_control)? -1 : 1;  
    const float reverse_pitch_int = (reverse_pitch_control)? -1 : 1;

    ROS_INFO("Updating home position: [%0.2f, %0.2f]", current_yaw_deg * reverse_yaw_int, current_pitch_deg * reverse_pitch_int);
    yaw_home_pos_deg = current_yaw_deg * reverse_yaw_int;
    pitch_home_pos_deg = current_pitch_deg * reverse_pitch_int;
}

void PTXInterface::setWaypoint(const nepi_ros_interfaces::AbsolutePanTiltWaypoint::ConstPtr &msg)
{

    const float reverse_yaw_int = (reverse_yaw_control)? -1 : 1;  
    const float reverse_pitch_int = (reverse_pitch_control)? -1 : 1;

    const float yaw_deg = msg->yaw_deg * reverse_yaw_int;
    const float pitch_deg = msg->pitch_deg * reverse_pitch_int;
    
    if (false == positionIsValid(yaw_deg, pitch_deg))
    {
        ROS_ERROR("Invalid waypoint position [%0.2f, %0.2f]... ignoring", yaw_deg, pitch_deg);
        return;
    }  

    const uint8_t waypoint_index = msg->waypoint_index;
    if (waypoint_index >= WAYPOINT_COUNT)
    {
        ROS_ERROR("Invalid waypoint index (%u)... ignoring", waypoint_index);
        return;
    }

    ROS_INFO("Setting waypoint %u to [%f,%f]", waypoint_index, yaw_deg, pitch_deg);
    waypoints[waypoint_index].yaw_deg = yaw_deg;
    waypoints[waypoint_index].pitch_deg = pitch_deg;
}

void PTXInterface::setWaypointHere(const std_msgs::UInt8::ConstPtr &msg)
{
    const uint8_t waypoint_index = msg->data;
    if (waypoint_index >= WAYPOINT_COUNT)
    {
        ROS_ERROR("Invalid waypoint index (%u)... ignoring", waypoint_index);
        return;        
    }

    float current_yaw_deg, current_pitch_deg;
    static_cast<PTXNode*>(_parent_node)->getCurrentPosition(current_yaw_deg, current_pitch_deg);

    const float reverse_yaw_int = (reverse_yaw_control)? -1 : 1;  
    const float reverse_pitch_int = (reverse_pitch_control)? -1 : 1;

    ROS_INFO("Setting waypoint %u to [%f,%f]", waypoint_index, current_yaw_deg * reverse_yaw_int, current_pitch_deg * reverse_pitch_int);
    waypoints[waypoint_index].yaw_deg = current_yaw_deg * reverse_yaw_int;
    waypoints[waypoint_index].pitch_deg = current_pitch_deg * reverse_pitch_int;
}

void PTXInterface::gotoWaypoint(const std_msgs::UInt8::ConstPtr &msg)
{
    const uint8_t waypoint_index = msg->data;
    if (waypoint_index >= WAYPOINT_COUNT)
    {
        ROS_ERROR("Invalid waypoint index (%u)... ignoring", waypoint_index);
        return;        
    }

    const PTXWaypoint waypoint = waypoints[waypoint_index];
    if ((waypoint.yaw_deg == PTXWaypoint::INVALID_WAYPOINT) || (waypoint.pitch_deg == PTXWaypoint::INVALID_WAYPOINT))
    {
        ROS_ERROR("Waypoint %u is unassigned... ignoring", waypoint_index);
        return;
    }

    const float reverse_yaw_int = (reverse_yaw_control)? -1 : 1;  
    const float reverse_pitch_int = (reverse_pitch_control)? -1 : 1;

    if (false == positionIsValid(waypoint.yaw_deg * reverse_yaw_int, waypoint.pitch_deg * reverse_pitch_int))
    {
        ROS_ERROR("Invalid waypoint position [%0.2f, %0.2f]... ignoring", waypoint.yaw_deg, waypoint.pitch_deg);
        return;
    }

    const float speed = currentSpeedRatioToDriverUnits();
    static_cast<PTXNode*>(_parent_node)->gotoPosition(waypoint.yaw_deg * reverse_yaw_int, waypoint.pitch_deg * reverse_pitch_int, speed);    
}

bool PTXInterface::capabilitiesQueryHandler(nepi_ros_interfaces::PTXCapabilitiesQuery::Request &req, 
                                            nepi_ros_interfaces::PTXCapabilitiesQuery::Response &res)
{
    res = ptx_caps_query_response;
    return true;
}

}
