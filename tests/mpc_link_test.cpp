#include <cassert>
#include <cstdint>
#include <iostream>

#include "wbr/mpc_link.hpp"

int main() {
  wbr::protocol::StateFrame state;
  state.sequence = 0x12345678U;
  state.sender_uptime_ms = 9876U;
  state.roll_rad = -0.125F;
  state.roll_rate_rad_s = 0.75F;
  state.body_height_m = 0.24F;
  state.body_height_rate_m_s = -0.31F;
  state.left_leg_length_m = 0.23F;
  state.left_leg_rate_m_s = 0.12F;
  state.right_leg_length_m = 0.25F;
  state.right_leg_rate_m_s = -0.14F;
  state.left_support_force_n = 122.5F;
  state.right_support_force_n = 131.25F;
  state.control_state = 3U;
  state.valid_flags = wbr::protocol::kStateImuFresh |
                      wbr::protocol::kStateSupportForceValid;
  state.contact_flags = wbr::protocol::kContactLeft;

  auto state_bytes = wbr::protocol::EncodeState(state);
  wbr::protocol::StateFrame decoded_state;
  assert(wbr::protocol::DecodeState(state_bytes.data(), state_bytes.size(), decoded_state) ==
         wbr::protocol::DecodeError::kNone);
  assert(decoded_state.sequence == state.sequence);
  assert(decoded_state.right_support_force_n == state.right_support_force_n);
  state_bytes[30] ^= 0x80U;
  assert(wbr::protocol::DecodeState(state_bytes.data(), state_bytes.size(), decoded_state) ==
         wbr::protocol::DecodeError::kWrongCrc);

  wbr::protocol::CommandFrame command;
  command.sequence = 42U;
  command.sender_uptime_ms = 1100U;
  command.source_state_sequence = 41U;
  command.left_support_force_n = 120.0F;
  command.right_support_force_n = 135.0F;
  command.mode = wbr::protocol::CommandMode::kShadow;
  command.contact_mode = wbr::protocol::ContactMode::kBoth;
  command.solver_flags = wbr::protocol::kSolverConverged |
                         wbr::protocol::kSolverInputFinite;
  command.solver_iterations = 12U;
  command.valid_for_ms = 30U;
  command.solve_time_us = 420U;

  const auto command_bytes = wbr::protocol::EncodeCommand(command);
  wbr::protocol::CommandFrame decoded_command;
  assert(wbr::protocol::DecodeCommand(command_bytes.data(), command_bytes.size(),
                                      decoded_command) ==
         wbr::protocol::DecodeError::kNone);
  assert(decoded_command.source_state_sequence == command.source_state_sequence);
  assert(decoded_command.left_support_force_n == command.left_support_force_n);
  assert(decoded_command.mode == command.mode);
  std::cout << "MPC Link v1 codec passed\n";
}
