/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef _PTX_NODE_H
#define _PTX_NODE_H

#include "sdk_node.h"
#include "ptx_interface.h"

namespace Numurus
{

class PTXInterface; // Forward declaration

class PTXNode :  public SDKNode
{
public:
  PTXNode();
  virtual ~PTXNode();

  // SDKNode overrides
  void retrieveParams() override;
  void run() override;

  typedef uint8_t PTX_DIRECTION;
  static constexpr PTX_DIRECTION PTX_DIRECTION_POSITIVE = 1;
  static constexpr PTX_DIRECTION PTX_DIRECTION_NEGATIVE = -1;
  

  // Pure virtual methods must be implemented in concrete base classes (e.g., with driver support)
  // PTXInterface requires these (and that they are public)
  virtual void moveYaw(PTX_DIRECTION direction, float speed, float time_s = 1000000.0f) = 0;
  virtual void movePitch(PTX_DIRECTION direction, float speed, float time_s = 1000000.0f) = 0;
  virtual void gotoPosition(float yaw_deg, float pitch_deg, float speed, float move_timeout_s = 1000000.0f) = 0;
  virtual void getCurrentPosition(float &yaw_deg_out, float &pitch_deg_out) = 0;
  virtual void getStatus(PTXStatus &status_out) = 0;
  virtual bool inMotion(float &yaw_goal_out, float &pitch_goal_out) = 0;
  virtual void stopMotion() = 0;
  virtual void reportPanTiltIdentity() const = 0;

protected:
  NodeParam<float> status_update_rate_hz;
  uint32_t loop_count = 0;
  PTXInterface *ptx_interface = nullptr;
  ros::Timer move_stop_timer;

  void stopMovingTimerCb(const ros::TimerEvent& ev){stopMotion();}
};

}

#endif