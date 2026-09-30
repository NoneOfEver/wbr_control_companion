#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include "wbr/ethercat_mpc_safety.hpp"
#include "wbr/ethercat_protocol.hpp"
#include "wbr/realtime_loop.hpp"

int main() {
  using namespace wbr::ethercat;
  assert(sizeof(MasterProcessImage) == 72U);
  assert(sizeof(SlaveProcessImage) == 96U);
  assert(offsetof(MasterProcessImage, mpc) == 32U);
  assert(offsetof(SlaveProcessImage, state) == 32U);
  assert(PayloadCrc32(1U, 2U, 3U) == PayloadCrc32(1U, 2U, 3U));
  assert(PayloadCrc32(1U, 2U, 3U) == 0xb0e02293U);
  assert(PayloadCrc32(1U, 2U, 3U) != PayloadCrc32(1U, 2U, 4U));

  MpcCommandPdo command{};
  std::uint32_t command_sequence = 41U;
  command.sequence = command_sequence;
  command.mode_and_flags = kMpcActive | kMpcConverged | kMpcInputFinite;
  command.valid_for_us = 30000U;
  assert(DisableMpcCommand(command, command_sequence, 99U, 123456U));
  assert(command_sequence == 42U);
  assert(command.sequence == 42U);
  assert(command.source_state_sequence == 99U);
  assert(command.host_send_time_ns == 123456U);
  assert(command.mode_and_flags == kMpcDisabled);
  assert(command.valid_for_us == 0U);

  // Re-applying the safe state must not consume sequence numbers every cycle.
  assert(!DisableMpcCommand(command, command_sequence, 100U, 123999U));
  assert(command_sequence == 42U);
  assert(command.sequence == 42U);

  const auto summary = wbr::realtime::Summarize({1, 2, 3, 4, 100});
  assert(summary.count == 5U);
  assert(summary.minimum_ns == 1);
  assert(summary.maximum_ns == 100);
  assert(summary.p50_ns == 3);
  std::cout << "EtherCAT ABI and statistics passed\n";
}
