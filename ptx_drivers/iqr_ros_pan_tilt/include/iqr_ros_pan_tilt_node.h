/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef _IQR_ROS_PAN_TILT_NODE_H
#define _IQR_ROS_PAN_TILT_NODE_H

#include "ptx_interface.h"
#include "ptx_node.h"

namespace IQR
{
  class PanTiltDriver; // Forward declaration
}

namespace Numurus
{

class IqrRosPanTiltNode : public PTXNode
{
public:
  IqrRosPanTiltNode();
  ~IqrRosPanTiltNode();

  // PTXNode overrides
  void retrieveParams() override;

  // PTXNode pure virtual functions must be implemented
  // PTXInterface requires these (and that they are public)
  void moveYaw(PTX_DIRECTION direction, float speed, float time_s = 1000000.0f);
  void movePitch(PTX_DIRECTION direction, float speed, float time_s = 1000000.0f);
  void gotoPosition(float yaw_deg, float pitch_deg, float speed, float move_timeout_s = 1000000.0f);
  void getCurrentPosition(float &yaw_deg_out, float &pitch_deg_out);
  void getStatus(PTXStatus &status_out);
  bool inMotion(float &yaw_goal_out, float &pitch_goal_out);
  void stopMotion();
  void reportPanTiltIdentity() const;

private:
  NodeParam<std::string> device_path; // TODO: Wrap this in an "autostartable" interface... used by generic_autolauncher
  IQR::PanTiltDriver *driver = nullptr;
};

} // namespace Numurus

#endif
