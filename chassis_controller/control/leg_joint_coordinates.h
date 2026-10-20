/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <chassis_params_generated.h>

namespace modules {

// Constant offsets only: d(link angle)/d(motor angle) = 1. Consequently
// velocity, Jacobian columns and torque remain in the same motor coordinates.
inline double JointBLinkAngle(double motor_angle) {
  return motor_angle + chassis_params::kJointBZeroOffset;
}

inline double JointDLinkAngle(double motor_angle) {
  return motor_angle + chassis_params::kJointDZeroOffset;
}

inline double JointBMotorAngle(double link_angle) {
  return link_angle - chassis_params::kJointBZeroOffset;
}

inline double JointDMotorAngle(double link_angle) {
  return link_angle - chassis_params::kJointDZeroOffset;
}

} // namespace modules
