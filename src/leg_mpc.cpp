#include "wbr/leg_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigenvalues>

namespace wbr::control {
namespace {

template <typename Derived>
double InfinityNorm(const Eigen::MatrixBase<Derived>& value) {
  return value.template lpNorm<Eigen::Infinity>();
}

}  // namespace

LegMpc::LegMpc() : LegMpc(Parameters{}) {}

LegMpc::LegMpc(Parameters parameters) : parameters_(std::move(parameters)) {
  if (parameters_.mass_kg <= 0.0 || parameters_.roll_inertia_kg_m2 <= 0.0 ||
      parameters_.sample_time_s <= 0.0 || parameters_.minimum_force_n < 0.0 ||
      parameters_.maximum_force_n <= parameters_.minimum_force_n ||
      parameters_.maximum_force_step_n <= 0.0 ||
      parameters_.maximum_iterations <= 0) {
    throw std::invalid_argument("invalid leg MPC parameters");
  }
  BuildModelAndCost();
}

void LegMpc::BuildModelAndCost() {
  const double dt = parameters_.sample_time_s;
  const double roll_gain = parameters_.half_track_m / parameters_.roll_inertia_kg_m2;
  const double vertical_gain = 1.0 / parameters_.mass_kg;
  const double damping = parameters_.vertical_damping_ns_per_m / parameters_.mass_kg;

  // The first-order ZOH approximation intentionally matches the reference
  // document. State: [roll, roll_rate, height_error, height_rate].
  ad_ << 1.0, dt, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, dt,
         0.0, 0.0, 0.0, 1.0 - damping * dt;
  // Match the chassis roll convention used by the MCU controller: increasing
  // left support force produces positive roll acceleration, while increasing
  // right support force produces negative roll acceleration.
  bd_ << 0.0, 0.0,
         roll_gain * dt, -roll_gain * dt,
         0.0, 0.0,
         vertical_gain * dt, vertical_gain * dt;

  free_response_.template block<4, 4>(0, 0).setIdentity();
  Eigen::Matrix4d a_power = Eigen::Matrix4d::Identity();
  for (int stage = 1; stage <= kHorizon; ++stage) {
    a_power = a_power * ad_;
    free_response_.template block<4, 4>(stage * 4, 0) = a_power;
    for (int input_stage = 0; input_stage < stage; ++input_stage) {
      Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
      for (int power = 0; power < stage - 1 - input_stage; ++power) {
        transition *= ad_;
      }
      prediction_input_.template block<4, 2>(stage * 4, input_stage * 2) =
          transition * bd_;
    }
  }

  for (int stage = 0; stage <= kHorizon; ++stage) {
    const double scale = stage == kHorizon ? parameters_.terminal_weight_scale : 1.0;
    q_bar_.template block<4, 4>(stage * 4, stage * 4) =
        scale * parameters_.state_weights.asDiagonal();
  }
  Hessian r_bar = Hessian::Zero();
  for (int stage = 0; stage < kHorizon; ++stage) {
    r_bar.template block<2, 2>(stage * 2, stage * 2) =
        parameters_.input_weights.asDiagonal();
  }
  hessian_ = 2.0 * (prediction_input_.transpose() * q_bar_ * prediction_input_ + r_bar);
  Eigen::SelfAdjointEigenSolver<Hessian> eigen_solver(hessian_, Eigen::EigenvaluesOnly);
  if (eigen_solver.info() != Eigen::Success || eigen_solver.eigenvalues().maxCoeff() <= 0.0) {
    throw std::runtime_error("leg MPC Hessian is not positive definite");
  }
  step_size_ = 1.0 / eigen_solver.eigenvalues().maxCoeff();
}

LegMpc::Prediction LegMpc::MakeReferenceVector(
    const std::array<Reference, kHorizon + 1>& trajectory) const {
  Prediction reference;
  for (int stage = 0; stage <= kHorizon; ++stage) {
    reference.template segment<4>(stage * 4) <<
        trajectory[stage].roll_rad,
        trajectory[stage].roll_rate_rad_per_s,
        trajectory[stage].height_m - parameters_.nominal_height_m,
        trajectory[stage].height_rate_m_per_s;
  }
  return reference;
}

LegMpc::Decision LegMpc::ProjectForces(const Decision& force_deviation,
                                       const Input& previous_force_n) const {
  const double hover_force = parameters_.mass_kg * parameters_.gravity_m_per_s2 / 2.0;
  Decision projected;
  Input previous;
  for (int side = 0; side < kInputDimension; ++side) {
    const double candidate = std::isfinite(previous_force_n(side))
                                 ? previous_force_n(side)
                                 : hover_force;
    previous(side) = std::clamp(candidate, parameters_.minimum_force_n,
                                parameters_.maximum_force_n);
  }
  for (int stage = 0; stage < kHorizon; ++stage) {
    for (int side = 0; side < kInputDimension; ++side) {
      const double requested_force = force_deviation(stage * 2 + side) + hover_force;
      const double rate_lower = previous(side) - parameters_.maximum_force_step_n;
      const double rate_upper = previous(side) + parameters_.maximum_force_step_n;
      const double force = std::clamp(
          requested_force,
          std::max(parameters_.minimum_force_n, rate_lower),
          std::min(parameters_.maximum_force_n, rate_upper));
      projected(stage * 2 + side) = force - hover_force;
      previous(side) = force;
    }
  }
  return projected;
}

LegMpc::Result LegMpc::Solve(const State& measured_state, const Reference& reference,
                             const Input& previously_applied_force_n) {
  std::array<Reference, kHorizon + 1> trajectory;
  trajectory.fill(reference);
  return Solve(measured_state, trajectory, previously_applied_force_n);
}

LegMpc::Result LegMpc::Solve(
    const State& measured_state,
    const std::array<Reference, kHorizon + 1>& reference_trajectory,
    const Input& previously_applied_force_n) {
  State model_state = measured_state;
  model_state(2) -= parameters_.nominal_height_m;
  const Prediction tracking_offset =
      free_response_ * model_state - MakeReferenceVector(reference_trajectory);
  const Decision gradient_offset =
      2.0 * prediction_input_.transpose() * q_bar_ * tracking_offset;

  Decision decision = ProjectForces(warm_start_, previously_applied_force_n);
  Decision accelerated = decision;
  double momentum = 1.0;
  double gradient_norm = 0.0;
  bool converged = false;
  int iterations = 0;
  for (; iterations < parameters_.maximum_iterations; ++iterations) {
    const Decision gradient = hessian_ * accelerated + gradient_offset;
    const Decision next =
        ProjectForces(accelerated - step_size_ * gradient, previously_applied_force_n);
    gradient_norm = InfinityNorm(next - accelerated) / step_size_;
    if (InfinityNorm(next - decision) < parameters_.convergence_tolerance) {
      decision = next;
      converged = true;
      ++iterations;
      break;
    }
    const double next_momentum = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * momentum * momentum));
    accelerated = next + ((momentum - 1.0) / next_momentum) * (next - decision);
    decision = next;
    momentum = next_momentum;
  }

  for (int stage = 0; stage < kHorizon - 1; ++stage) {
    warm_start_.template segment<2>(stage * 2) = decision.template segment<2>((stage + 1) * 2);
  }
  warm_start_.template tail<2>() = decision.template tail<2>();

  const double hover_force = parameters_.mass_kg * parameters_.gravity_m_per_s2 / 2.0;
  Result result;
  result.converged = converged;
  result.iterations = iterations;
  result.left_force_n = decision(0) + hover_force;
  result.right_force_n = decision(1) + hover_force;
  result.gradient_inf_norm = gradient_norm;
  return result;
}

void LegMpc::Reset() { warm_start_.setZero(); }

}  // namespace wbr::control
