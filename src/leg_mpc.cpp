#include "wbr/leg_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>
#include <limits>
#include <osqp.h>

#include <Eigen/Eigenvalues>

namespace wbr::control {
struct LegMpc::QpWorkspace {
  static constexpr int kConstraints = 2 * kDecisionDimension;
  Eigen::Matrix<double, kConstraints, kDecisionDimension> a =
      Eigen::Matrix<double, kConstraints, kDecisionDimension>::Zero();
  std::vector<OSQPFloat> px, ax;
  std::vector<OSQPInt> pi, pp, ai, ap;
  std::array<OSQPFloat, kConstraints> lower{}, upper{};
  std::array<OSQPFloat, kDecisionDimension> gradient{};
  OSQPSolver* solver = nullptr;

  QpWorkspace(const Hessian& h, const Parameters& parameters) {
    // A = [I; D]. D contains u0 and all differences u[k]-u[k-1].
    a.template topRows<kDecisionDimension>().setIdentity();
    for (int i = 0; i < kDecisionDimension; ++i) {
      a(kDecisionDimension + i, i) = 1.0;
      if (i >= kInputDimension) a(kDecisionDimension + i, i-kInputDimension) = -1.0;
    }
    for (int col = 0; col < kDecisionDimension; ++col) {
      pp.push_back(static_cast<OSQPInt>(px.size()));
      ap.push_back(static_cast<OSQPInt>(ax.size()));
      for (int row = 0; row <= col; ++row) {
        pi.push_back(row); px.push_back(h(row,col));
      }
      for (int row = 0; row < kConstraints; ++row) {
        if (a(row,col) != 0.0) { ai.push_back(row); ax.push_back(a(row,col)); }
      }
    }
    pp.push_back(static_cast<OSQPInt>(px.size()));
    ap.push_back(static_cast<OSQPInt>(ax.size()));
    const double hover = parameters.mass_kg * parameters.gravity_m_per_s2 / 2.0;
    for (int i = 0; i < kDecisionDimension; ++i) {
      lower[i] = parameters.minimum_force_n-hover;
      upper[i] = parameters.maximum_force_n-hover;
      lower[kDecisionDimension+i] = -parameters.maximum_force_step_n;
      upper[kDecisionDimension+i] = parameters.maximum_force_step_n;
    }
    OSQPCscMatrix p{}, constraints{};
    OSQPCscMatrix_set_data(&p,kDecisionDimension,kDecisionDimension,
                         static_cast<OSQPInt>(px.size()),px.data(),pi.data(),pp.data());
    OSQPCscMatrix_set_data(&constraints,kConstraints,kDecisionDimension,
                         static_cast<OSQPInt>(ax.size()),ax.data(),ai.data(),ap.data());
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polishing = 0;
    settings.warm_starting = 1;
    settings.max_iter = parameters.maximum_iterations;
    settings.eps_abs = parameters.convergence_tolerance;
    settings.eps_rel = parameters.convergence_tolerance;
    settings.check_termination = 10;
    if (osqp_setup(&solver,&p,gradient.data(),&constraints,lower.data(),upper.data(),
                   kConstraints,kDecisionDimension,&settings) != 0) {
      if (solver) osqp_cleanup(solver);
      solver = nullptr;
      throw std::runtime_error("leg MPC OSQP setup failed");
    }
  }
  ~QpWorkspace() { if (solver) osqp_cleanup(solver); }
};

LegMpc::LegMpc() : LegMpc(Parameters{}) {}
LegMpc::~LegMpc() = default;

LegMpc::LegMpc(Parameters parameters) : parameters_(std::move(parameters)) {
  for (double value : {parameters_.mass_kg, parameters_.roll_inertia_kg_m2,
       parameters_.half_track_m, parameters_.vertical_damping_ns_per_m,
       parameters_.sample_time_s, parameters_.nominal_height_m,
       parameters_.gravity_m_per_s2, parameters_.minimum_force_n,
       parameters_.maximum_force_n, parameters_.maximum_force_step_n,
       parameters_.terminal_weight_scale}) {
    if (!std::isfinite(value)) throw std::invalid_argument("nonfinite leg MPC parameter");
  }
  if (parameters_.mass_kg <= 0.0 || parameters_.roll_inertia_kg_m2 <= 0.0 ||
      parameters_.sample_time_s <= 0.0 || parameters_.minimum_force_n < 0.0 ||
      parameters_.maximum_force_n <= parameters_.minimum_force_n ||
      parameters_.maximum_force_step_n <= 0.0 ||
      parameters_.maximum_iterations <= 0 ||
      parameters_.half_track_m <= 0.0 || parameters_.gravity_m_per_s2 <= 0.0 ||
      parameters_.vertical_damping_ns_per_m < 0.0 || parameters_.terminal_weight_scale <= 0.0 ||
      !(parameters_.convergence_tolerance > 0.0) ||
      !(parameters_.constraint_tolerance_n > 0.0) ||
      !std::isfinite(parameters_.convergence_tolerance) ||
      !std::isfinite(parameters_.constraint_tolerance_n) ||
      !parameters_.state_weights.allFinite() ||
      !parameters_.input_weights.allFinite() ||
      parameters_.state_weights.minCoeff() < 0.0 ||
      parameters_.input_weights.minCoeff() <= 0.0) {
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
  if (eigen_solver.info() != Eigen::Success || eigen_solver.eigenvalues().minCoeff() <= 0.0) {
    throw std::runtime_error("leg MPC Hessian is not positive definite");
  }
  qp_ = std::make_unique<QpWorkspace>(hessian_, parameters_);
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
  const double hover = parameters_.mass_kg * parameters_.gravity_m_per_s2 / 2.0;
  Input previous;
  for (int side = 0; side < kInputDimension; ++side) {
    previous(side) = std::clamp(std::isfinite(previously_applied_force_n(side))
        ? previously_applied_force_n(side) : hover,
        parameters_.minimum_force_n, parameters_.maximum_force_n);
  }
  Result result;
  result.used_fallback = true;
  result.left_force_n = previous(0);
  result.right_force_n = previous(1);
  for (auto& force : result.predicted_force_n) force = previous;
  if (!measured_state.allFinite() || !previously_applied_force_n.allFinite() ||
      previously_applied_force_n.minCoeff() < parameters_.minimum_force_n-parameters_.constraint_tolerance_n ||
      previously_applied_force_n.maxCoeff() > parameters_.maximum_force_n+parameters_.constraint_tolerance_n) return result;
  const Prediction reference = MakeReferenceVector(reference_trajectory);
  if (!reference.allFinite()) return result;
  State model_state = measured_state;
  model_state(2) -= parameters_.nominal_height_m;
  const Decision gradient = 2.0 * prediction_input_.transpose() * q_bar_ *
      (free_response_ * model_state - reference);
  if (!gradient.allFinite()) return result;
  for (int i = 0; i < kDecisionDimension; ++i) qp_->gradient[i] = gradient(i);
  for (int side = 0; side < kInputDimension; ++side) {
    qp_->lower[kDecisionDimension+side] = previous(side)-hover-parameters_.maximum_force_step_n;
    qp_->upper[kDecisionDimension+side] = previous(side)-hover+parameters_.maximum_force_step_n;
  }
  if (osqp_update_data_vec(qp_->solver,qp_->gradient.data(),qp_->lower.data(),qp_->upper.data()) != 0)
    return result;
  if (osqp_solve(qp_->solver) != 0) return result;
  const auto& info = *qp_->solver->info;
  result.solver_status = info.status_val;
  result.iterations = info.iter;
  result.primal_residual = info.prim_res;
  result.dual_residual = info.dual_res;
  // Legacy diagnostic field now reports the QP dual residual, not iterate motion.
  result.gradient_inf_norm = info.dual_res;
  if (info.status_val != OSQP_SOLVED || !std::isfinite(info.prim_res) ||
      !std::isfinite(info.dual_res)) return result;
  Decision decision;
  for (int i = 0; i < kDecisionDimension; ++i) decision(i) = qp_->solver->solution->x[i];
  if (!decision.allFinite()) return result;
  const auto constrained = (qp_->a * decision).eval();
  for (int i = 0; i < QpWorkspace::kConstraints; ++i) {
    result.constraint_violation_n = std::max(result.constraint_violation_n,
        std::max(qp_->lower[i]-constrained(i), constrained(i)-qp_->upper[i]));
  }
  if (result.constraint_violation_n > parameters_.constraint_tolerance_n) return result;
  for (int stage = 0; stage < kHorizon; ++stage)
    result.predicted_force_n[stage] = decision.template segment<2>(stage*2).array()+hover;
  result.left_force_n = result.predicted_force_n[0](0);
  result.right_force_n = result.predicted_force_n[0](1);
  result.converged = true;
  result.used_fallback = false;
  for (int stage = 0; stage < kHorizon-1; ++stage)
    warm_start_.template segment<2>(stage*2) = decision.template segment<2>((stage+1)*2);
  warm_start_.template tail<2>() = decision.template tail<2>();
  // Shift the primal trajectory, keep OSQP's dual warm start.
  osqp_warm_start(qp_->solver,warm_start_.data(),nullptr);
  return result;
}

void LegMpc::Reset() {
  warm_start_.setZero();
  osqp_cold_start(qp_->solver);
}

}  // namespace wbr::control
