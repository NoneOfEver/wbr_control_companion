/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "usb_link.hpp"
#include <atomic>
#include <filesystem>
#include <ostream>
#include <vector>
namespace wbr::load_test {
struct Sample {
  uint64_t host_ns = 0;
  uint32_t observed_us = 0, submit_us = 0;
  uint32_t stream_sequence = 0, parser_sequence = 0;
  uint8_t channel = 0, bytes = 0, valid = 0;
};
struct Cycle {
  uint64_t target_ns = 0, wake_ns = 0, submitted_ns = 0;
  uint32_t skipped = 0;
  uint16_t decoder_valid_mask = 0;
};
class Capture {
public:
  Capture(host::SensorStore &sensors, double seconds);
  static void Observe(void *, const bridge::Packet &, uint64_t, uint64_t);
  static void CommandCompleted(void *, bool);
  void StopRecording() { enabled_ = false; }
  // Call only after joining the USB owner. Returns observed stream coverage.
  bool Export(const std::filesystem::path &, uint64_t start, uint64_t end,
              const std::vector<Cycle> &, std::ostream &) const;
  void PrintDiagnostics(std::ostream &) const;

private:
  void Append(Sample);
  host::SensorStore &sensors_;
  std::vector<Sample> samples_;
  size_t used_ = 0, overflow_ = 0;
  uint64_t completed_commands_ = 0, failed_commands_ = 0;
  std::atomic<bool> enabled_{true};
  std::array<bridge::Packet, 2> ready_{};
  size_t ready_count_ = 0;
};
} // namespace wbr::load_test
