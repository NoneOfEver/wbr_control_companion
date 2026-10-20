/* SPDX-License-Identifier: Apache-2.0 */

#include "support_force_estimator.h"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <cmath>

#include "chassis_config.h"
#include "gas_spring_model.h"

namespace modules {
namespace {
using namespace chassis_config;

double LowPassAlpha(double cutoff_hz, double dt) {
  constexpr double kPi = 3.14159265358979323846;
  return 1.0 - std::exp(-2.0 * kPi * cutoff_hz * dt);
}
} // namespace

void SupportForceEstimator::Reset() {
  side_state_ = {};
  initialized_ = false;
}

ChassisSupportForceEstimate
SupportForceEstimator::Update(const ChassisCycleInput &input) {
  using namespace chassis_config;
  ChassisSupportForceEstimate output = {};
  if (!input.feedback_valid || !input.imu_fresh) {
    Reset();
    return output;
  }

  const double dt = std::clamp(input.dt, 0.0005, 0.01);
  const double cr = std::cos(input.roll);
  const double sr = std::sin(input.roll);
  const double cp = std::cos(input.pitch);
  const double sp = std::sin(input.pitch);
  const double world_specific_force_g =
      kSupportWorldVerticalSpecificForceSign *
      (-sp * static_cast<double>(input.imu.accel_g[0]) +
       sr * cp * static_cast<double>(input.imu.accel_g[1]) +
       cr * cp * static_cast<double>(input.imu.accel_g[2]));
  output.world_vertical_specific_force_g = world_specific_force_g;
  output.body_vertical_accel_mps2 = (world_specific_force_g - 1.0) * kGravity;
  if (!std::isfinite(output.body_vertical_accel_mps2)) {
    Reset();
    return output;
  }

  const double derivative_alpha =
      LowPassAlpha(kSupportDerivativeFilterCutoffHz, dt);
  const double force_alpha = LowPassAlpha(kSupportForceFilterCutoffHz, dt);
  bool valid = true;
  for (Side side : {Side::kLeft, Side::kRight}) {
    const LegKinematics &leg = input.leg.Get(side);
    if (!std::isfinite(leg.length) || leg.length <= 1.0e-6) {
      valid = false;
      continue;
    }
    SideState &state = side_state_.Get(side);
    if (!initialized_) {
      state.previous_length_rate = leg.length_rate;
      state.previous_theta_rate = input.theta_rate.Get(side);
    }
    const double raw_length_accel =
        (leg.length_rate - state.previous_length_rate) / dt;
    const double raw_theta_accel =
        (input.theta_rate.Get(side) - state.previous_theta_rate) / dt;
    state.previous_length_rate = leg.length_rate;
    state.previous_theta_rate = input.theta_rate.Get(side);
    state.length_accel +=
        derivative_alpha * (raw_length_accel - state.length_accel);
    state.theta_accel +=
        derivative_alpha * (raw_theta_accel - state.theta_accel);

    Eigen::Matrix2d jacobian;
    jacobian << leg.jacobian[0][0], leg.jacobian[0][1], leg.jacobian[1][0],
        leg.jacobian[1][1];
    const JointPair<double> compensation =
        ConfiguredGasSpringCompensation(side, input);
    const JointPair<double> &feedback = input.joint_torque_feedback.Get(side);
    const double torque_sign = side == Side::kLeft
                                   ? kSupportLeftJointTorqueSign
                                   : kSupportRightJointTorqueSign;
    // Net mechanism torque = calibrated motor feedback + passive spring torque.
    // Compensation is the negative passive torque. Columns are [D, B].
    const Eigen::Vector2d joint_torque(
        torque_sign * feedback.d - compensation.d,
        torque_sign * feedback.b - compensation.b);
    const Eigen::Matrix2d normal =
        jacobian * jacobian.transpose() +
        kSupportJacobianDamping * Eigen::Matrix2d::Identity();
    const Eigen::Vector2d endpoint_force =
        normal.ldlt().solve(jacobian * joint_torque);
    // J and the spring torque use raw motor/kinematic coordinates. First
    // resolve the endpoint force into radial and raw-angle components.
    // The calibrated body angle is alpha = sign * (raw_angle - offset),
    // so virtual work requires the same sign on the tangential force.
    const LegHardwareMap &hardware = kLegHardware.Get(side);
    const double radial_x = leg.hx / leg.length;
    const double radial_z = leg.hz / leg.length;
    const double radial_force =
        radial_x * endpoint_force.x() + radial_z * endpoint_force.y();
    const double tangential_force =
        hardware.leg_coordinate_sign *
        (-radial_z * endpoint_force.x() + radial_x * endpoint_force.y());
    const double alpha =
        hardware.leg_coordinate_sign *
        std::remainder(std::atan2(leg.hx, -leg.hz) - hardware.leg_angle_offset,
                       kTwoPi);
    const double body_force_x =
        radial_force * std::sin(alpha) + tangential_force * std::cos(alpha);
    const double body_force_z =
        -radial_force * std::cos(alpha) + tangential_force * std::sin(alpha);
    // Endpoint actuation is body-on-wheel. Its opposite supports the body.
    // Use the same body-to-world vertical row as the IMU projection above.
    const double leg_vertical_force =
        -(-sp * body_force_x + cr * cp * body_force_z);

    // Wheel inertia retains the sagittal-plane approximation:
    // z_rel=-L*cos(theta). Roll/lateral hip-offset accelerations are not
    // modelled here.
    const double theta = input.theta.Get(side);
    const double theta_rate = input.theta_rate.Get(side);
    const double wheel_vertical_accel =
        output.body_vertical_accel_mps2 - state.length_accel * std::cos(theta) +
        2.0 * leg.length_rate * theta_rate * std::sin(theta) +
        leg.length * state.theta_accel * std::sin(theta) +
        leg.length * theta_rate * theta_rate * std::cos(theta);
    const double raw_force =
        leg_vertical_force +
        kSupportWheelMass * (kGravity + wheel_vertical_accel);
    if (!endpoint_force.allFinite() || !std::isfinite(raw_force)) {
      valid = false;
      continue;
    }
    const double bounded_force = std::clamp(raw_force, -kSupportMaximumAbsForce,
                                            kSupportMaximumAbsForce);
    if (!initialized_) {
      state.filtered_force = bounded_force;
    } else {
      state.filtered_force +=
          force_alpha * (bounded_force - state.filtered_force);
    }
    output.raw_force_n.Get(side) = raw_force;
    output.filtered_force_n.Get(side) = state.filtered_force;
    output.leg_vertical_force_n.Get(side) = leg_vertical_force;
    output.wheel_vertical_accel_mps2.Get(side) = wheel_vertical_accel;
  }
  initialized_ = valid;
  output.valid = valid;
  return output;
}

} // namespace modules
