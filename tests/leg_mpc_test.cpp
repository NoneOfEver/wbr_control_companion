#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "wbr/leg_mpc.hpp"

int main() {
  wbr::control::LegMpc controller;
  const auto& p = controller.parameters();
  const double hover = p.mass_kg * p.gravity_m_per_s2 / 2.0;
  wbr::control::LegMpc::State state;
  state << 0.10, 0.0, 0.08, 0.0;
  wbr::control::LegMpc::Reference reference;
  reference.height_m = 0.10;
  wbr::control::LegMpc::Input previous =
      wbr::control::LegMpc::Input::Constant(hover);

  // With the MCU's roll convention, a positive roll error must be corrected
  // by reducing left support and increasing right support.
  {
    wbr::control::LegMpc direction_controller;
    wbr::control::LegMpc::State direction_state;
    direction_state << 0.10, 0.0, p.nominal_height_m, 0.0;
    const auto direction_result =
        direction_controller.Solve(direction_state, reference, previous);
    assert(direction_result.left_force_n < direction_result.right_force_n);
  }

  const double initial_roll = std::abs(state(0));
  for (int step = 0; step < 300; ++step) {
    const auto result = controller.Solve(state, reference, previous);
    assert(std::isfinite(result.left_force_n));
    assert(std::isfinite(result.right_force_n));
    assert(result.left_force_n >= p.minimum_force_n - 1e-9);
    assert(result.right_force_n >= p.minimum_force_n - 1e-9);
    assert(result.left_force_n <= p.maximum_force_n + 1e-9);
    assert(result.right_force_n <= p.maximum_force_n + 1e-9);
    assert(std::abs(result.left_force_n - previous(0)) <= p.maximum_force_step_n + 1e-9);
    assert(std::abs(result.right_force_n - previous(1)) <= p.maximum_force_step_n + 1e-9);
    previous << result.left_force_n, result.right_force_n;
    wbr::control::LegMpc::State deviation = state;
    deviation(2) -= p.nominal_height_m;
    deviation = controller.discrete_a() * deviation +
                controller.discrete_b() *
                    (previous.array() - hover).matrix();
    state = deviation;
    state(2) += p.nominal_height_m;
  }

  assert(std::abs(state(0)) < 0.2 * initial_roll);
  assert(std::abs(state(2) - reference.height_m) < 0.005);
  std::cout << "final roll=" << state(0) << " height=" << state(2) << '\n';
  return 0;
}
