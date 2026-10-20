/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "balance_controller.h"
#include "chassis_actuator.h"
#include "chassis_input_reader.h"
#include "chassis_state_machine.h"
#include "climb_stairs_controller.h"
#include "flight_controller.h"
#include "jump_controller.h"
#include "recovery_controller.h"
#include "support_force_estimator.h"
namespace modules {
// Native cycle owner: no threads, USB, RTOS, global channels or telemetry.
class ChassisController {
public:
  explicit ChassisController(FrameSink &sink) : actuator_(sink) {}
  ChassisControlOutput Tick(const ControllerInputs &, uint64_t now_us,
                            bool operator_permission = false);
  ChassisControlState state() const { return state_machine_.state(); }

private:
  void UpdateControlState(const ChassisCycleInput &);
  void ComputeControlOutput(const ChassisCycleInput &,
                            const ChassisSupportForceEstimate &,
                            ChassisControlOutput &);
  void ApplyControlOutput(const ChassisCycleInput &, ChassisControlOutput &);
  void ResetControlState();
  uint64_t last_us_ = 0;
  uint32_t loop_ticks_ = 0;
  RecoveryController recovery_controller_;
  ClimbStairsController climb_stairs_controller_;
  BalanceController balance_controller_;
  ChassisInputReader input_reader_;
  SupportForceEstimator support_force_estimator_;
  FlightController flight_controller_;
  JumpController jump_controller_;
  ChassisStateMachine state_machine_;
  ChassisActuator actuator_;
};
} // namespace modules
