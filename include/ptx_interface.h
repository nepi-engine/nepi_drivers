/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef _PTX_INTERFACE_H
#define _PTX_INTERFACE_H

#include "sdk_interface.h"
#include "sdk_node.h" // NodeParam

#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "nepi_ros_interfaces/PanTiltPosition.h"
#include "nepi_ros_interfaces/PanTiltLimits.h"
#include "nepi_ros_interfaces/SingleAxisTimedMove.h"
#include "nepi_ros_interfaces/AbsolutePanTiltWaypoint.h"
#include "nepi_ros_interfaces/PTXCapabilitiesQuery.h"

namespace Numurus
{

struct PTXSettings
{
    std::string frame_id;
    std::string yaw_joint_name;
    std::string pitch_joint_name;

    float min_yaw_hardstop_deg;
    float max_yaw_hardstop_deg;
    float min_pitch_hardstop_deg;
    float max_pitch_hardstop_deg;
    float min_speed_driver_units;
    float max_speed_driver_units;
    float speed_ratio;
    bool reverse_yaw_control;
    bool reverse_pitch_control;
};

struct PTXCapabilities
{
    bool has_absolute_positioning;
    bool has_speed_control;
    bool has_homing;
    bool has_waypoints;
};

struct PTXStatus
{
    uint32_t loop_count;

    std::string serial_num;
    std::string hw_version;
    std::string sw_version;

    float yaw_goal;
    float yaw_now;
    
    float pitch_goal;
    float pitch_now;
    
    uint32_t speed_driver_units;

    std::vector<std::string> driver_errors;
};

struct PTXWaypoint
{
    static constexpr float INVALID_WAYPOINT = -1000.0f;
    float yaw_deg = INVALID_WAYPOINT; // Initialized invalid
    float pitch_deg = INVALID_WAYPOINT; // Initialized invalid
};

class PTXNode; // forward declaration

class PTXInterface : public SDKInterface
{
public:
    PTXInterface(PTXNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh, 
                 const PTXSettings &default_settings, const PTXCapabilities &default_capabilities);
    PTXInterface() = delete; // No default constructor available

    virtual ~PTXInterface();

    // SDKInterface overrides
	void retrieveParams() override;
	void initSubscribers() override;
	void initPublishers() override;
    void initServices() override;

    bool positionIsValid(float yaw_deg, float pitch_deg) const;
    inline uint16_t currentSpeedRatioToDriverUnits() const 
    { 
        const float speed = speed_ratio; 
        return (uint16_t)(speed * max_speed_driver_units + ((1.0 - speed) * min_speed_driver_units));
    }
    void publishJointStateAndStatus();
    void setSpeedRatioHandler(const std_msgs::Float32::ConstPtr &msg);
    void setHomePositionHandler(const nepi_ros_interfaces::PanTiltPosition::ConstPtr &msg);
    void setSoftLimitsHandler(const nepi_ros_interfaces::PanTiltLimits::ConstPtr &msg);
    void goHomeHandler(const std_msgs::Empty::ConstPtr &msg);
    void jogToPositionHandler(const nepi_ros_interfaces::PanTiltPosition::ConstPtr &msg);
    void jogToYawRatioHandler(const std_msgs::Float32::ConstPtr &msg);
    void jogToPitchRatioHandler(const std_msgs::Float32::ConstPtr &msg);  
    void stopMovingHandler(const std_msgs::Empty::ConstPtr &msg);

    bool capabilitiesQueryHandler(nepi_ros_interfaces::PTXCapabilitiesQuery::Request &req, 
                                  nepi_ros_interfaces::PTXCapabilitiesQuery::Response &res);

    void jogTimedYaw(const nepi_ros_interfaces::SingleAxisTimedMove::ConstPtr &msg);
    void jogTimedPitch(const nepi_ros_interfaces::SingleAxisTimedMove::ConstPtr &msg);
    void reverseYawControlHandler(const std_msgs::Bool::ConstPtr &msg);
    void reversePitchControlHandler(const std_msgs::Bool::ConstPtr &msg);
    void setHomePositionHere(const std_msgs::Empty::ConstPtr &msg);
    void setWaypoint(const nepi_ros_interfaces::AbsolutePanTiltWaypoint::ConstPtr &msg);
    void setWaypointHere(const std_msgs::UInt8::ConstPtr &msg);
    void gotoWaypoint(const std_msgs::UInt8::ConstPtr &msg);

private:
    SDKNode::NodeParam<std::string> frame_id;
	SDKNode::NodeParam<std::string> yaw_joint_name;
	SDKNode::NodeParam<std::string> pitch_joint_name;
    SDKNode::NodeParam<float> yaw_home_pos_deg;
    SDKNode::NodeParam<float> pitch_home_pos_deg;
    SDKNode::NodeParam<float> min_yaw_hardstop_deg;
    SDKNode::NodeParam<float> max_yaw_hardstop_deg;
    SDKNode::NodeParam<float> min_yaw_softstop_deg;
    SDKNode::NodeParam<float> max_yaw_softstop_deg;
    SDKNode::NodeParam<float> min_pitch_hardstop_deg;
    SDKNode::NodeParam<float> max_pitch_hardstop_deg;
    SDKNode::NodeParam<float> min_pitch_softstop_deg;
    SDKNode::NodeParam<float> max_pitch_softstop_deg;
    SDKNode::NodeParam<float> speed_ratio;
    SDKNode::NodeParam<bool> reverse_yaw_control;
    SDKNode::NodeParam<bool> reverse_pitch_control;

    SDKNode::NodeParam<bool> has_absolute_positioning;
    SDKNode::NodeParam<bool> has_speed_control;
    SDKNode::NodeParam<bool> has_homing;
    SDKNode::NodeParam<bool> has_waypoints;

    float max_speed_driver_units;
    float min_speed_driver_units;

    ros::Publisher jointPub;
    ros::Publisher odomPub;
    ros::Publisher statusPub;
        
    sensor_msgs::JointState joint_state;
    nav_msgs::Odometry odometry;
    nepi_ros_interfaces::PTXCapabilitiesQuery::Response ptx_caps_query_response;

    static constexpr size_t WAYPOINT_COUNT = 256;
    PTXWaypoint waypoints[WAYPOINT_COUNT];
};

}

#endif