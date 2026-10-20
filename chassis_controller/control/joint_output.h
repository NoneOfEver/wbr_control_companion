/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <algorithm>
#include <cmath>

#include "gas_spring_model.h"

namespace modules {

// Called once by the actuator, after controller selection and before CAN
// packing.
inline void PrepareJointOutput(ChassisControlState state,
                               const ChassisCycleInput &input,
                               ChassisControlOutput &output) {
  using namespace chassis_config;
  bool active_state = false;
  switch (state) {
  case ChassisControlState::kBalance:
  case ChassisControlState::kRecovery:
  case ChassisControlState::kFlight:
  case ChassisControlState::kJump:
  case ChassisControlState::kClimbStairs:
    active_state = true;
    break;
  default:
    break;
  }
  if (!active_state || !output.joint_control_active || !input.feedback_valid ||
      !input.imu_fresh || !input.dm_ready || !input.arm_complete ||
      !input.requested_enable) {
    output.joint_torque = {};
    return;
  }
  const bool position_control = state == ChassisControlState::kRecovery ||
                                state == ChassisControlState::kClimbStairs;
  const double limit = position_control
                           ? std::min(kJointTorqueLimit, kJointTorqueMax)
                           : kJointTorqueLimit;
  for (Side side : {Side::kLeft, Side::kRight}) {
    const JointPair<double> compensation =
        state == ChassisControlState::kJump &&
                output.bypass_gas_spring_compensation
            ? JointPair<double>{}
            : ConfiguredGasSpringCompensation(side, input);
    JointPair<double> &command = output.joint_torque.Get(side);
    for (Joint joint : {Joint::kB, Joint::kD}) {
      double &torque = command.Get(joint);
      // Invalid controller outputs must not turn into spring-only commands.
      const double sum = torque + compensation.Get(joint);
      torque = std::isfinite(torque) && std::isfinite(sum)
                   ? std::clamp(sum, -limit, limit)
                   : 0.0;
    }
  }
}

} // namespace modules
