/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "frame_sink.h"
#include <cerrno>
#include <chassis_controller/motor_map.h>
#include <wbr_bridge/bridge_protocol.hpp>
namespace wbr::host {
class MotorCommands final : public modules::FrameSink {
public:
  void Begin() { packet_ = {}; }
  int Submit(platform::CanTxSlot slot, uint8_t bus, uint16_t id,
             const uint8_t *data, size_t length) override {
    const auto index = static_cast<unsigned>(slot);
    if (index >= 6 || length != 8 || !data)
      return -EINVAL;
    const auto &motor = platform::motor_map::kMotors[index];
    if (motor.bus != bus || motor.command_id != id)
      return -EINVAL;
    // A later write to the same motor within one control cycle supersedes
    // the previous one before the single merged USB command is submitted.
    unsigned position = 0;
    for (; position < packet_[10]; ++position)
      if (packet_[32 + 16 * position + 12] == index)
        break;
    if (position == packet_[10]) {
      if (position == 6)
        return -ENOSPC;
      ++packet_[10];
    }
    auto *record = packet_.data() + 32 + 16 * position;
    bridge::CanRecord(record, bus, id, data, 8);
    record[12] = index;
    return 0;
  }
  bridge::Commands packet() const { return packet_; }

private:
  bridge::Commands packet_{};
};
} // namespace wbr::host
