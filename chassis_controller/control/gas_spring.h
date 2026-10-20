/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <array>
#include <cmath>

#include "leg_kinematics.h"

namespace modules {

// Upper mount P = (offset/AG) * (-Gz, Gx), rigidly attached to AG.
// Signed offset chooses the installed side of AG. Q = G + (b/GH)(H-G).
// The extending spring applies generalized torque +F * d|Q-P|/dq.
// Return the opposing motor torque in raw kinematic column order [D, B].
inline std::array<double, 2>
ComputeGasSpringCompensation(const LegKinematics &leg, double phi_d, double ag,
                             double gh, double offset, double b, double force) {
  if (!std::isfinite(phi_d) || !std::isfinite(ag) || !std::isfinite(gh) ||
      !std::isfinite(offset) || !std::isfinite(b) || !std::isfinite(force) ||
      ag <= 0.0 || gh <= 0.0 || b < 0.0 || b > gh || force <= 0.0)
    return {};
  const double ratio = b / gh;
  const double coefficient = 1.0 - ratio;
  const double normal_ratio = offset / ag;
  const double gx = ag * std::cos(phi_d);
  const double gz = -ag * std::sin(phi_d);
  const double sx = coefficient * gx + ratio * leg.hx + normal_ratio * gz;
  const double sz = coefficient * gz + ratio * leg.hz - normal_ratio * gx;
  const double length = std::hypot(sx, sz);
  if (!std::isfinite(length) || length < 1.0e-6)
    return {};
  std::array<double, 2> torque{};
  for (size_t column = 0U; column < 2U; ++column) {
    const double dx =
        ratio * leg.jacobian[0][column] +
        (column == 0U ? coefficient * gz - normal_ratio * gx : 0.0);
    const double dz =
        ratio * leg.jacobian[1][column] -
        (column == 0U ? coefficient * gx + normal_ratio * gz : 0.0);
    torque[column] = -force * (sx * dx + sz * dz) / length;
    if (!std::isfinite(torque[column]))
      return {};
  }
  return torque;
}

} // namespace modules
