/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include "ptx_node.h"

#define DEFAULT_STATUS_UPDATE_RATE_HZ   50

namespace Numurus
{

PTXNode::PTXNode() :
    status_update_rate_hz{"ptx/status_update_rate_hz", DEFAULT_STATUS_UPDATE_RATE_HZ, this}
{
    // Initialize the jog stop timer
    move_stop_timer = n.createTimer(ros::Duration(1000000.0), &PTXNode::stopMovingTimerCb, this, true);
    move_stop_timer.stop();    
}

PTXNode::~PTXNode()
{
    if (ptx_interface)
    {
        delete ptx_interface;
        ptx_interface = nullptr;
    }
}

void PTXNode::retrieveParams()
{
    SDKNode::retrieveParams();

    status_update_rate_hz.retrieve();
}

void PTXNode::run()
{
    init();
    ros::Duration(0.1).sleep();

    reportPanTiltIdentity();

    ROS_INFO("Driving to configured Home position");
    const std_msgs::Empty::ConstPtr msg;
    ptx_interface->goHomeHandler(msg);

    const float rate_hz = status_update_rate_hz;
    ros::Rate r(rate_hz);
    ROS_INFO("Starting main loop (%0.2fHz)", rate_hz);
    while (ros::ok())
    {
        ros::spinOnce();
        ptx_interface->publishJointStateAndStatus();
        ++loop_count;
        r.sleep();
    }
}

}