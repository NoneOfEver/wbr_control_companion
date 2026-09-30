#include "wbr/ethercat_mpc_safety.hpp"

namespace wbr::ethercat {

bool DisableMpcCommand(MpcCommandPdo& command,
                       std::uint32_t& command_sequence,
                       std::uint32_t source_state_sequence,
                       std::uint64_t send_time_ns) {
  const bool changed = (command.mode_and_flags & 0xffU) != kMpcDisabled;
  if (changed) {
    command.sequence = ++command_sequence;
    command.source_state_sequence = source_state_sequence;
    command.host_send_time_ns = send_time_ns;
  }
  command.mode_and_flags = kMpcDisabled;
  command.valid_for_us = 0U;
  return changed;
}

}  // namespace wbr::ethercat
