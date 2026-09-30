#include <iomanip>
#include <iostream>

#include "wbr/leg_mpc.hpp"

int main() {
  wbr::control::LegMpc controller;
  wbr::control::LegMpc::State state;
  state << 0.08, 0.0, 0.085, 0.0;
  wbr::control::LegMpc::Reference reference;
  reference.height_m = 0.10;
  wbr::control::LegMpc::Input previous_force;
  previous_force.setConstant(controller.parameters().mass_kg *
                             controller.parameters().gravity_m_per_s2 / 2.0);

  std::cout << "step,roll_rad,height_m,left_force_n,right_force_n\n";
  for (int step = 0; step < 200; ++step) {
    const auto result = controller.Solve(state, reference, previous_force);
    previous_force << result.left_force_n, result.right_force_n;
    const auto force_deviation = previous_force.array() -
        controller.parameters().mass_kg * controller.parameters().gravity_m_per_s2 / 2.0;
    state = controller.discrete_a() *
                (wbr::control::LegMpc::State() << state(0), state(1),
                 state(2) - controller.parameters().nominal_height_m, state(3)).finished() +
            controller.discrete_b() * force_deviation.matrix();
    state(2) += controller.parameters().nominal_height_m;
    std::cout << step << ',' << std::fixed << std::setprecision(6) << state(0) << ','
              << state(2) << ',' << result.left_force_n << ',' << result.right_force_n
              << '\n';
  }
}
