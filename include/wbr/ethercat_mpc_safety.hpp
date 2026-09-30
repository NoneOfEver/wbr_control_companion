#pragma once

#include <cstdint>

#include "wbr/ethercat_protocol.hpp"

namespace wbr::ethercat {

// Convert the process image to a fail-safe command. If the previous image was
// not already Disabled, this emits a new sequence so the slave cannot reject
// the transition as a replay. Returns true when a new command was generated.
bool DisableMpcCommand(MpcCommandPdo& command,
                       std::uint32_t& command_sequence,
                       std::uint32_t source_state_sequence,
                       std::uint64_t send_time_ns);

}  // namespace wbr::ethercat
