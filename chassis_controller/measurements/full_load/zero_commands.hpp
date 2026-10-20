/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "motor_commands.hpp"
#include <protocols/motors/dm_motor_protocol.h>
#include <stdexcept>
namespace wbr::load_test {
// A full six-frame load while all joints remain OUTSIDE closed-loop control.
// No MIT setpoints, Enter, ClearError or SaveZero are available in this tool.
inline bridge::Commands ZeroOutputCommands() {
  host::MotorCommands commands;
  commands.Begin();
  uint8_t exit[8]{};
  if (protocols::GetDmControlCommandFrame(protocols::DmControlCommand::kExit,
                                          exit))
    throw std::runtime_error("Cannot encode DM exit");
  const uint8_t zero[8]{};
  for (unsigned slot = 0; slot < 6; ++slot) {
    const auto &motor = platform::motor_map::kMotors[slot];
    if (commands.Submit(motor.tx_slot, motor.bus, motor.command_id,
                        slot < 2 ? zero : exit, 8))
      throw std::runtime_error("Invalid production motor mapping");
  }
  return commands.packet();
}
} // namespace wbr::load_test
