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