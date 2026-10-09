#pragma once

#include <array>
#include <cstddef>
#include <memory>

#include <Eigen/Core>

namespace wbr::control {

class LegMpc {
 public:
  static constexpr int kStateDimension = 4;
  static constexpr int kInputDimension = 2;
  static constexpr int kHorizon = 15;

  using State = Eigen::Matrix<double, kStateDimension, 1>;
  using Input = Eigen::Matrix<double, kInputDimension, 1>;

  struct Parameters {
    // Keep these chassis constants synchronized with
    // wbr_control/src/chassis_controller/modules/chassis/chassis_params.yaml.
    double mass_kg = 12.054;
    double roll_inertia_kg_m2 = 0.113303628;
    double half_track_m = 0.215;
    double vertical_damping_ns_per_m = 8.0;
    double sample_time_s = 0.01;
    double nominal_height_m = 0.10;
    double gravity_m_per_s2 = 9.80665;
    double minimum_force_n = 20.0;
    double maximum_force_n = 200.0;
    double maximum_force_step_n = 30.0;
    // Q 顺序：[roll角度误差(rad), roll角速度误差(rad/s),
    //          高度误差(m), 高度速度误差(m/s)]。
    // 每项代价 = 权重 * 对应误差的平方；增大权重会更重视该项跟踪。
    // 速度目标为0时，提高速度权重通常更抑制运动；单位不同不能直接比大小。
    State state_weights = (State() << 1200.0, 30.0, 12000.0, 80.0).finished();
    // R 顺序：[左腿支撑力偏差(N), 右腿支撑力偏差(N)]。
    // 力偏差相对重力前馈 M*g/2，非相邻周期的力变化。
    // 增大R更抑制用力，减小R通常使控制更积极；仍受力和变化率约束。
    Input input_weights = (Input() << 0.025, 0.025).finished();
    double terminal_weight_scale = 5.0;
    int maximum_iterations = 2000;
    // QP primal/dual termination tolerance; no inter-iteration force criterion.
    double convergence_tolerance = 1.0e-6;
    double constraint_tolerance_n = 1.0e-3;
  };

  struct Reference {
    double roll_rad = 0.0;
    double roll_rate_rad_per_s = 0.0;
    double height_m = 0.10;
    double height_rate_m_per_s = 0.0;
  };

  struct Result {
    bool converged = false;
    int iterations = 0;
    double left_force_n = 0.0;
    double right_force_n = 0.0;
    double gradient_inf_norm = 0.0; // Legacy field: now the OSQP dual residual.
    bool used_fallback = false;
    int solver_status = 0;
    double primal_residual = 0.0;
    double dual_residual = 0.0;
    double constraint_violation_n = 0.0;
    std::array<Input, kHorizon> predicted_force_n{};
  };

  LegMpc();
  explicit LegMpc(Parameters parameters);
  ~LegMpc();
  LegMpc(const LegMpc&) = delete;
  LegMpc& operator=(const LegMpc&) = delete;

  Result Solve(const State& measured_state, const Reference& reference,
               const Input& previously_applied_force_n);
  Result Solve(const State& measured_state,
               const std::array<Reference, kHorizon + 1>& reference_trajectory,
               const Input& previously_applied_force_n);
  void Reset();

  [[nodiscard]] const Parameters& parameters() const { return parameters_; }
  [[nodiscard]] const Eigen::Matrix4d& discrete_a() const { return ad_; }
  [[nodiscard]] const Eigen::Matrix<double, 4, 2>& discrete_b() const { return bd_; }

 private:
  static constexpr int kDecisionDimension = kInputDimension * kHorizon;
  static constexpr int kPredictionDimension = kStateDimension * (kHorizon + 1);
  using Decision = Eigen::Matrix<double, kDecisionDimension, 1>;
  using Prediction = Eigen::Matrix<double, kPredictionDimension, 1>;
  using Hessian = Eigen::Matrix<double, kDecisionDimension, kDecisionDimension>;
  using PredictionMatrix =
      Eigen::Matrix<double, kPredictionDimension, kDecisionDimension>;
  using FreeResponseMatrix =
      Eigen::Matrix<double, kPredictionDimension, kStateDimension>;

  void BuildModelAndCost();
  struct QpWorkspace;
  std::unique_ptr<QpWorkspace> qp_;
  Prediction MakeReferenceVector(
      const std::array<Reference, kHorizon + 1>& trajectory) const;

  Parameters parameters_;
  Eigen::Matrix4d ad_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 2> bd_ = Eigen::Matrix<double, 4, 2>::Zero();
  PredictionMatrix prediction_input_ = PredictionMatrix::Zero();
  FreeResponseMatrix free_response_ = FreeResponseMatrix::Zero();
  Eigen::Matrix<double, kPredictionDimension, kPredictionDimension> q_bar_ =
      Eigen::Matrix<double, kPredictionDimension, kPredictionDimension>::Zero();
  Hessian hessian_ = Hessian::Zero();
  Decision warm_start_ = Decision::Zero();
};

}  // namespace wbr::control
