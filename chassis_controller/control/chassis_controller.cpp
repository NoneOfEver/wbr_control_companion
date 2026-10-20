/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_controller.h"
#include "chassis_config.h"
#include <algorithm>
#include <cmath>
namespace modules {
namespace {
enum class ChassisJoint : uint8_t { kLeftB, kLeftD, kRightB, kRightD };
template <typename Enum> constexpr size_t ToIndex(Enum value) {
  return static_cast<size_t>(value);
}
} // namespace
using namespace chassis_config;
void ChassisController::UpdateControlState(const ChassisCycleInput &input) {
  const ChassisStateTransition transition = state_machine_.Update(input);
  if (transition.enable_changed && transition.enabled) {
    // Preserve enable-edge initialization even when a simultaneous safety
    // condition sends the first enabled cycle directly to a stop state.
    balance_controller_.ResetForEnable();
  }
  if (!transition.changed) {
    return;
  }

  // State-entry actions live here; the state machine itself remains a pure
  // transition-policy owner and never touches controllers or hardware.
  switch (transition.to) {
  case ChassisControlState::kDisabled:
  case ChassisControlState::kSafetyStop:
  case ChassisControlState::kActionFault:
  case ChassisControlState::kTiltFault:
    ResetControlState();
    actuator_.ResetArming();
    recovery_controller_.Reset();
    flight_controller_.Reset();
    jump_controller_.Reset();
    climb_stairs_controller_.Reset();
    break;
  case ChassisControlState::kDmArming:
    ResetControlState();
    actuator_.ResetArming();
    recovery_controller_.Reset();
    flight_controller_.Reset();
    jump_controller_.Reset();
    climb_stairs_controller_.Reset();
    break;
  case ChassisControlState::kWaitingFeedback:
    ResetControlState();
    break;
  case ChassisControlState::kRecovery:
    ResetControlState();
    recovery_controller_.Reset();
    break;
  case ChassisControlState::kBalance:
    if (transition.from == ChassisControlState::kRecovery ||
        transition.from == ChassisControlState::kClimbStairs ||
        transition.from == ChassisControlState::kFlight ||
        transition.from == ChassisControlState::kJump) {
      balance_controller_.SynchronizeReferences(input);
      recovery_controller_.Reset();
      climb_stairs_controller_.Reset();
      flight_controller_.Reset();
      jump_controller_.Reset();
    }
    break;
  case ChassisControlState::kFlight:
    flight_controller_.Reset();
    break;
  case ChassisControlState::kJump:
    balance_controller_.SynchronizeReferences(input);
    jump_controller_.Reset();
    break;
  case ChassisControlState::kClimbStairs:
    climb_stairs_controller_.Reset();
    break;
  }
}

void ChassisController::ComputeControlOutput(
    const ChassisCycleInput &input,
    const ChassisSupportForceEstimate &support_force,
    ChassisControlOutput &output) {
  (void)support_force;
  output = {};
  if (state_machine_.state() == ChassisControlState::kRecovery) {
    RecoveryControllerInput recovery_input = {};
    recovery_input.dt = input.dt;
    recovery_input.pitch = input.pitch;
    recovery_input.roll = input.roll;
    recovery_input.vertical_accel_g = input.imu.vertical_accel_g;
    recovery_input.leg = {input.leg.left, input.leg.right};
    recovery_input.joint_position = {
        input.joint_position.left.b,
        input.joint_position.left.d,
        input.joint_position.right.b,
        input.joint_position.right.d,
    };
    recovery_input.joint_velocity = {
        input.joint_velocity.left.b,
        input.joint_velocity.left.d,
        input.joint_velocity.right.b,
        input.joint_velocity.right.d,
    };
    // 当前工程没有云台；这些默认值构成后续接入云台反馈的接口边界。
    recovery_input.gimbal.available = false;
    recovery_input.gimbal.safe_for_leg_motion = true;
    recovery_input.gimbal.aligned = true;
    const RecoveryControllerOutput recovery =
        recovery_controller_.Update(recovery_input);
    output.joint_control_active = recovery.joint_control_active;
    output.joint_torque.left.b =
        recovery.joint_torque[ToIndex(ChassisJoint::kLeftB)];
    output.joint_torque.left.d =
        recovery.joint_torque[ToIndex(ChassisJoint::kLeftD)];
    output.joint_torque.right.b =
        recovery.joint_torque[ToIndex(ChassisJoint::kRightB)];
    output.joint_torque.right.d =
        recovery.joint_torque[ToIndex(ChassisJoint::kRightD)];
    if (recovery.failed) {
      state_machine_.MarkRecoveryFailed();
    } else if (recovery.finished) {
      state_machine_.MarkRecoveryCompleted();
    }
    return;
  }

  if (state_machine_.state() != ChassisControlState::kBalance) {
    if (state_machine_.state() == ChassisControlState::kJump) {
      const JumpControllerOutput jump =
          jump_controller_.Update(input, balance_controller_);
      output = jump.control;
      output.joint_control_active = !jump.failed && !jump.finished;
      if (jump.failed) {
        state_machine_.MarkJumpFailed();
      } else if (jump.finished) {
        state_machine_.MarkJumpCompleted();
      }
      return;
    }
    if (state_machine_.state() == ChassisControlState::kFlight) {
      const FlightControllerOutput flight = flight_controller_.Update(input);
      output = flight.control;
      output.joint_control_active = !flight.failed && !flight.finished;
      if (flight.failed) {
        state_machine_.MarkFlightFailed();
      } else if (flight.finished) {
        state_machine_.MarkFlightCompleted();
      }
      return;
    }
    if (state_machine_.state() != ChassisControlState::kClimbStairs) {
      return;
    }
    ClimbStairsControllerInput climb_input = {};
    climb_input.dt = input.dt;
    climb_input.pitch = input.pitch;
    climb_input.leg = {input.leg.left, input.leg.right};
    climb_input.joint_position = {
        input.joint_position.left.b,
        input.joint_position.left.d,
        input.joint_position.right.b,
        input.joint_position.right.d,
    };
    climb_input.joint_velocity = {
        input.joint_velocity.left.b,
        input.joint_velocity.left.d,
        input.joint_velocity.right.b,
        input.joint_velocity.right.d,
    };
    const ClimbStairsControllerOutput climb =
        climb_stairs_controller_.Update(climb_input);
    output.joint_control_active = climb.joint_control_active;
    output.joint_torque.left.b =
        climb.joint_torque[ToIndex(ChassisJoint::kLeftB)];
    output.joint_torque.left.d =
        climb.joint_torque[ToIndex(ChassisJoint::kLeftD)];
    output.joint_torque.right.b =
        climb.joint_torque[ToIndex(ChassisJoint::kRightB)];
    output.joint_torque.right.d =
        climb.joint_torque[ToIndex(ChassisJoint::kRightD)];
    if (climb.failed) {
      state_machine_.MarkClimbFailed();
    } else if (climb.finished) {
      state_machine_.MarkClimbCompleted();
    }
    return;
  }

  output.joint_control_active = true;
  balance_controller_.Update(input, output);
}

void ChassisController::ApplyControlOutput(const ChassisCycleInput &input,
                                           ChassisControlOutput &output) {
  if (actuator_.Apply(state_machine_.state(), loop_ticks_, input, output)) {
    ResetControlState();
  }
}

void ChassisController::ResetControlState() { balance_controller_.Reset(); }

ChassisControlOutput ChassisController::Tick(const ControllerInputs &sources,
                                             uint64_t now_us,
                                             bool operator_permission) {
  ChassisCycleInput input{};
  const double dt = last_us_ && now_us > last_us_
                        ? std::clamp((now_us - last_us_) * 1e-6, 0.0005, 0.01)
                        : kDefaultDt;
  last_us_ = now_us;
  input_reader_.Read(sources, input, now_us, dt, actuator_.arm_complete());
  input.requested_enable = input.requested_enable && operator_permission;
  const auto support = support_force_estimator_.Update(input);
  state_machine_.SetFlightDetected(flight_controller_.UpdateLiftoffDetection(
      input, support, state_machine_.state() == ChassisControlState::kBalance));
  UpdateControlState(input);
  ChassisControlOutput output{};
  ComputeControlOutput(input, support, output);
  ApplyControlOutput(input, output);
  ++loop_ticks_;
  return output;
}
} // namespace modules
