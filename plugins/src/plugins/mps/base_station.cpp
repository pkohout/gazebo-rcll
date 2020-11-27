/***************************************************************************
 *  base_station.cpp - controls a basesation mps
 *
 *  Generated: Wed Apr 22 13:25:39 2015
 *  Copyright  2015  Randolph Maaßen
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "base_station.h"

using namespace gazebo;

BaseStation::BaseStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    : Mps(_parent, _sdf) {}

void BaseStation::opc_process_operation(ConstOpcInstructionPtr &msg) {
  printf("BASESTATION: operation: %s\n", name_.c_str());
  if (!msg->has_base_operation())
    return;

  gazsim_msgs::BaseColor base_color = msg->base_operation().color();
  gzwrap::Pose3d spawn_pose = model_->GZWRAP_WORLD_POSE();

  gazsim_msgs::Color spawn_clr;
  switch (base_color) {
  case gazsim_msgs::BASE_COLOR_BLACK:
    spawn_clr = gazsim_msgs::Color::BLACK;
    break;
  case gazsim_msgs::BASE_COLOR_SILVER:
    spawn_clr = gazsim_msgs::Color::SILVER;
    break;
  case gazsim_msgs::BASE_COLOR_RED:
  default:
    spawn_clr = gazsim_msgs::Color::RED;
    break;
  }

  puck_on_mps_ = spawn_puck(spawn_pose, spawn_clr);
}

void BaseStation::on_new_puck(ConstNewPuckPtr &msg) {
  Mps::on_new_puck(msg);

  physics::ModelPtr new_puck = world_->GZWRAP_MODEL_BY_NAME(msg->puck_name());
  if (pose_hit(new_puck->GZWRAP_WORLD_POSE(), model_->GZWRAP_WORLD_POSE(), 1)) {
    printf("BASESTATION: new puck: %s\n", msg->puck_name().c_str());
    puck_on_mps_ = new_puck->GetName();
  }
}
