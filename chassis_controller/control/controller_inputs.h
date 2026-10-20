/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "chassis_types.h"
#include <array>
namespace modules {
// One coherent host snapshot, all freshness timestamps are host monotonic time.
struct ControllerInputs {
  std::array<msg::ChassisMotorFeedbackRawFrame, 6> motors{};
  msg::RemoteInputState remote{};
  ChassisImuSample imu{};
  bool remote_disable_pose = false;
};
} // namespace modules
