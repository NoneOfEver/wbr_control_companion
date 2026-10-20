/**
 * @file src/chassis_controller/modules/chassis/leg_kinematics.cc
 * @ingroup wbr_modules
 * @brief 实现五连杆腿部的正逆运动学与雅可比计算。
 * @details 由主机控制线程按冻结输入调用；硬件采集和执行器输出通过显式接口提供。
 */

#include "leg_kinematics.h"
#include "leg_joint_coordinates.h"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <cmath>

#include "chassis_config.h"

namespace {

using namespace modules::chassis_config;
constexpr double kJacobianStep = 1.0e-6;
constexpr double kMinLegLength = 1.0e-4;

} // namespace

namespace modules {

bool ForwardKinematics(double phi1, double phi2, int branch, double &hx,
                       double &hz) {
  // Public FK/IK coordinates stay motor angles. Apply installation offsets
  // exactly once here so numerical Jacobians and IK iterations share the map.
  const double link_d = JointDLinkAngle(phi1);
  const double link_b = JointBLinkAngle(phi2);
  const double bx = kLengthAB * std::cos(link_b);
  const double bz = -kLengthAB * std::sin(link_b);
  const double dx = kLengthAD * std::cos(link_d);
  const double dz = -kLengthAD * std::sin(link_d);
  const double gx = kLengthAG * std::cos(link_d);
  const double gz = -kLengthAG * std::sin(link_d);

  const double dbx = dx - bx;
  const double dbz = dz - bz;
  const double distance = std::hypot(dbx, dbz);
  if (distance > kLengthBC + kLengthCD - 1e-9 || distance < 1e-9) {
    return false;
  }

  const double a =
      (kLengthBC * kLengthBC - kLengthCD * kLengthCD + distance * distance) /
      (2.0 * distance);
  const double h = std::sqrt(std::fmax(0.0, kLengthBC * kLengthBC - a * a));
  const double px = bx + a * dbx / distance;
  const double pz = bz + a * dbz / distance;
  const double cx =
      branch == 1 ? px - h * dbz / distance : px + h * dbz / distance;
  const double cz =
      branch == 1 ? pz + h * dbx / distance : pz - h * dbx / distance;

  const double dcx = cx - dx;
  const double dcz = cz - dz;
  const double dc_length = std::hypot(dcx, dcz);
  if (dc_length < 1e-12) {
    return false;
  }
  hx = gx + kLengthGH * dcx / dc_length;
  hz = gz + kLengthGH * dcz / dc_length;
  return true;
}

bool InverseKinematics(double target_hx, double target_hz, int branch,
                       double seed_phi1, double seed_phi2, double &phi1,
                       double &phi2) {
  if (!std::isfinite(target_hx) || !std::isfinite(target_hz) ||
      !std::isfinite(seed_phi1) || !std::isfinite(seed_phi2)) {
    return false;
  }

  phi1 = seed_phi1;
  phi2 = seed_phi2;
  constexpr int kMaxIterations = 24;
  constexpr double kDamping = 1e-5;
  constexpr double kPositionTolerance = 2e-5;
  constexpr double kMaxJointStep = 0.12;

  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    double hx = 0.0;
    double hz = 0.0;
    double jacobian[2][2] = {};
    if (!ForwardKinematics(phi1, phi2, branch, hx, hz) ||
        !NumericalJacobian(phi1, phi2, branch, jacobian)) {
      return false;
    }

    const double error_x = target_hx - hx;
    const double error_z = target_hz - hz;
    if (std::hypot(error_x, error_z) <= kPositionTolerance) {
      return true;
    }

    // 阻尼最小二乘：dq = J^T (J J^T + lambda I)^-1 e。
    // J J^T + lambda I（lambda > 0）恒为对称正定，LLT 分解始终成立。
    Eigen::Matrix2d J;
    J << jacobian[0][0], jacobian[0][1], jacobian[1][0], jacobian[1][1];
    const Eigen::Vector2d e(error_x, error_z);
    const Eigen::Matrix2d damped =
        J * J.transpose() + kDamping * Eigen::Matrix2d::Identity();
    // 求解 damped * y = e；damped 对称正定，用 LLT 分解原地求解。
    Eigen::Vector2d y = e;
    damped.llt().solveInPlace(y);
    const Eigen::Vector2d dq = J.transpose() * y;
    const double step_phi1 = std::clamp(dq[0], -kMaxJointStep, kMaxJointStep);
    const double step_phi2 = std::clamp(dq[1], -kMaxJointStep, kMaxJointStep);
    phi1 += step_phi1;
    phi2 += step_phi2;
  }

  double hx = 0.0;
  double hz = 0.0;
  return ForwardKinematics(phi1, phi2, branch, hx, hz) &&
         std::hypot(target_hx - hx, target_hz - hz) <= 5e-4;
}

bool NumericalJacobian(double phi1, double phi2, int branch,
                       double jacobian[2][2]) {
  double hx_plus;
  double hz_plus;
  double hx_minus;
  double hz_minus;
  if (!ForwardKinematics(phi1 + kJacobianStep, phi2, branch, hx_plus,
                         hz_plus) ||
      !ForwardKinematics(phi1 - kJacobianStep, phi2, branch, hx_minus,
                         hz_minus)) {
    return false;
  }
  jacobian[0][0] = (hx_plus - hx_minus) / (2.0 * kJacobianStep);
  jacobian[1][0] = (hz_plus - hz_minus) / (2.0 * kJacobianStep);
  if (!ForwardKinematics(phi1, phi2 + kJacobianStep, branch, hx_plus,
                         hz_plus) ||
      !ForwardKinematics(phi1, phi2 - kJacobianStep, branch, hx_minus,
                         hz_minus)) {
    return false;
  }
  jacobian[0][1] = (hx_plus - hx_minus) / (2.0 * kJacobianStep);
  jacobian[1][1] = (hz_plus - hz_minus) / (2.0 * kJacobianStep);
  return true;
}

bool ComputeLegKinematics(double phi1, double phi2, double dphi1, double dphi2,
                          int branch, LegKinematics &leg) {
  if (!ForwardKinematics(phi1, phi2, branch, leg.hx, leg.hz) ||
      !NumericalJacobian(phi1, phi2, branch, leg.jacobian)) {
    return false;
  }
  leg.length = std::hypot(leg.hx, leg.hz);
  if (leg.length < kMinLegLength) {
    return false;
  }
  // 雅可比与关节速度的乘积：v = J * dq。
  Eigen::Matrix2d J;
  J << leg.jacobian[0][0], leg.jacobian[0][1], leg.jacobian[1][0],
      leg.jacobian[1][1];
  const Eigen::Vector2d dq(dphi1, dphi2);
  const Eigen::Vector2d velocity = J * dq;
  leg.length_rate = (leg.hx * velocity[0] + leg.hz * velocity[1]) / leg.length;
  leg.angle = std::atan2(leg.hx, -leg.hz);
  leg.angle_rate = (-leg.hz * velocity[0] + leg.hx * velocity[1]) /
                   (leg.length * leg.length);
  return true;
}

} // namespace modules
