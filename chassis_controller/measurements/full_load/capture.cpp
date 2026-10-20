/* SPDX-License-Identifier: Apache-2.0 */
#include "capture.hpp"
#include <algorithm>
#include <chassis_controller/motor_map.h>
#include <cmath>
#include <fstream>
#include <protocols/motors/dm_motor_protocol.h>
#include <stdexcept>
namespace wbr::load_test {
namespace {
constexpr const char *names[] = {
    "left_wheel",    "right_wheel",   "left_joint_b", "left_joint_d",
    "right_joint_b", "right_joint_d", "SBUS",         "HI91",
    "onboard_IMU",   "referee"};
// Acceptance limits for this load test, not runtime controller freshness
// policy.
constexpr uint64_t gap_limits_us[] = {20000, 20000, 20000, 20000, 20000,
                                      20000, 30000, 20000, 20000, 0};
void Distribution(std::ostream &out, const char *label,
                  std::vector<double> values) {
  out << "  " << label << " us: ";
  if (values.empty()) {
    out << "unavailable\n";
    return;
  }
  std::sort(values.begin(), values.end());
  const auto quantile = [&](double p) {
    return values[size_t(std::ceil(p * values.size())) - 1];
  };
  out << "p50/p99/max=" << quantile(.5) << '/' << quantile(.99) << '/'
      << values.back() << '\n';
}
struct Stream {
  uint64_t count = 0, bytes = 0, missing = 0, backwards = 0, invalid = 0,
           parser_updates = 0;
  uint64_t last_ns = 0, max_silence_ns = 0, last_parser_ns = 0,
           max_parser_silence_ns = 0;
  uint32_t sequence = 0, parser_sequence = 0;
  bool seen = false;
  std::vector<double> queue, gaps, parser_gaps;
};
} // namespace
Capture::Capture(host::SensorStore &sensors, double seconds)
    : sensors_(sensors), samples_(size_t(seconds * 25000) + 4096) {}
void Capture::CommandCompleted(void *context, bool success) {
  auto &self = *static_cast<Capture *>(context);
  if (success)
    ++self.completed_commands_;
  else
    ++self.failed_commands_;
}
void Capture::Append(Sample sample) {
  if (used_ == samples_.size()) {
    ++overflow_;
    return;
  }
  samples_[used_++] = sample;
}
void Capture::Observe(void *context, const bridge::Packet &packet,
                      uint64_t arrival, uint64_t) {
  auto &self = *static_cast<Capture *>(context);
  const auto kind = bridge::Kind(packet[9]);
  if (kind == bridge::Kind::Ready) {
    if (self.ready_count_ < self.ready_.size())
      self.ready_[self.ready_count_++] = packet;
    return;
  }
  if (!self.enabled_)
    return;
  const uint32_t submit = bridge::U32(packet.data() + 4);
  if (kind == bridge::Kind::Can) {
    for (unsigned record = 0; record < packet[10]; ++record) {
      const auto *frame = packet.data() + 16 + 16 * record;
      for (unsigned motor = 0; motor < 6; ++motor) {
        const auto &address = platform::motor_map::kMotors[motor];
        if (frame[0] != address.bus ||
            bridge::U16(frame + 2) != address.feedback_id)
          continue;
        const bool valid =
            frame[1] == 8 &&
            (motor < 2 || !protocols::IsDmFaultStatus(frame[4] >> 4));
        self.Append({arrival, bridge::U32(frame + 12), submit, 0, 0,
                     uint8_t(motor), frame[1], uint8_t(valid)});
      }
    }
  } else if (kind == bridge::Kind::Uart) {
    const auto snapshot = self.sensors_.Read();
    const auto port = packet[10];
    const uint32_t sequence =
        port == 1 ? snapshot.control.remote.sequence
                  : (port == 2 ? snapshot.control.imu.sequence : 0);
    self.Append({arrival, bridge::U32(packet.data() + 20), submit,
                 bridge::U32(packet.data() + 16), sequence, uint8_t(5 + port),
                 packet[11], 1});
  } else if (kind == bridge::Kind::Imu) {
    self.Append({arrival, bridge::U32(packet.data() + 16), submit, 0, 0, 8,
                 packet[10], uint8_t(packet[10] == 25)});
  }
}
void Capture::PrintDiagnostics(std::ostream &out) const {
  for (size_t i = 0; i < ready_count_; ++i) {
    const auto *data = ready_[i].data();
    out << "MCU diagnostics " << (i ? "after" : "before") << ": ";
    if (bridge::U32(data + 28) == 0x34545255) {
      const auto flags = bridge::U32(data + 32);
      out << "URT4 line_idle=" << ((flags >> 4) & 1)
          << " idle_error=" << ((flags >> 5) & 1)
          << " offset_error=" << ((flags >> 6) & 1) << '\n';
    } else
      out << "URT4 unavailable (no per-command CAN completion counters)\n";
  }
}
bool Capture::Export(const std::filesystem::path &directory, uint64_t start,
                     uint64_t end, const std::vector<Cycle> &cycles,
                     std::ostream &out) const {
  std::ofstream received(directory / "received.csv"),
      timing(directory / "cycles.csv");
  if (!received || !timing)
    throw std::runtime_error("Cannot create capture CSVs");
  received << "channel,host_arrival_ns,mcu_observed_us,mcu_submit_us,queue_us,"
              "bytes,stream_sequence,parser_sequence,valid\n";
  timing << "target_ns,wake_ns,submitted_ns,skipped_slots,decoder_valid_mask\n";
  std::array<Stream, 10> streams{};
  for (size_t i = 0; i < used_; ++i) {
    const auto &sample = samples_[i];
    if (sample.host_ns < start || sample.host_ns >= end)
      continue;
    auto &stream = streams[sample.channel];
    received << names[sample.channel] << ',' << sample.host_ns << ','
             << sample.observed_us << ',' << sample.submit_us << ','
             << uint32_t(sample.submit_us - sample.observed_us) << ','
             << unsigned(sample.bytes) << ',' << sample.stream_sequence << ','
             << sample.parser_sequence << ',' << unsigned(sample.valid) << '\n';
    ++stream.count;
    stream.bytes += sample.bytes;
    stream.invalid += !sample.valid;
    stream.queue.push_back(uint32_t(sample.submit_us - sample.observed_us));
    if (stream.seen) {
      const auto gap = sample.host_ns - stream.last_ns;
      stream.gaps.push_back(gap / 1000.0);
      stream.max_silence_ns = std::max(stream.max_silence_ns, gap);
      if (sample.channel == 6 || sample.channel == 7 || sample.channel == 9) {
        const uint32_t advance = sample.stream_sequence - stream.sequence;
        if (advance && advance < 0x80000000U)
          stream.missing += advance - 1;
        else
          ++stream.backwards;
      }
    } else
      stream.max_silence_ns = sample.host_ns - start;
    if ((sample.channel == 6 || sample.channel == 7) &&
        sample.parser_sequence != stream.parser_sequence) {
      ++stream.parser_updates;
      const auto parser_gap =
          sample.host_ns -
          (stream.last_parser_ns ? stream.last_parser_ns : start);
      stream.max_parser_silence_ns =
          std::max(stream.max_parser_silence_ns, parser_gap);
      if (stream.last_parser_ns)
        stream.parser_gaps.push_back(parser_gap / 1000.0);
      stream.last_parser_ns = sample.host_ns;
      stream.parser_sequence = sample.parser_sequence;
    }
    stream.seen = true;
    stream.last_ns = sample.host_ns;
    stream.sequence = sample.stream_sequence;
  }
  std::vector<double> wake, send_gaps, submit_cost;
  uint64_t skipped = 0, previous = 0;
  for (const auto &cycle : cycles) {
    timing << cycle.target_ns << ',' << cycle.wake_ns << ','
           << cycle.submitted_ns << ',' << cycle.skipped << ','
           << cycle.decoder_valid_mask << '\n';
    wake.push_back((cycle.wake_ns - cycle.target_ns) / 1000.0);
    submit_cost.push_back((cycle.submitted_ns - cycle.wake_ns) / 1000.0);
    if (previous)
      send_gaps.push_back((cycle.submitted_ns - previous) / 1000.0);
    previous = cycle.submitted_ns;
    skipped += cycle.skipped;
  }
  const double elapsed = (end - start) / 1e9;
  out << "elapsed=" << elapsed << " s OUT_cycles=" << cycles.size()
      << " logical_CAN_frames=" << cycles.size() * 6
      << " OUT_rate=" << cycles.size() / elapsed
      << " Hz skipped_slots=" << skipped << '\n';
  Distribution(out, "host wake lateness", wake);
  Distribution(out, "snapshot/command submission time", submit_cost);
  Distribution(out, "USB command submit interval", send_gaps);
  out << "USB128_completed=" << completed_commands_
      << " failed_or_cancelled=" << failed_commands_ << '\n';
  bool passed =
      !overflow_ && !failed_commands_ && completed_commands_ == cycles.size();
  for (unsigned channel = 0; channel < streams.size(); ++channel) {
    auto &stream = streams[channel];
    stream.max_silence_ns = std::max(
        stream.max_silence_ns, end - (stream.seen ? stream.last_ns : start));
    out << names[channel] << " received=" << stream.count
        << " rate=" << stream.count / elapsed
        << " Hz missing_chunks=" << stream.missing
        << " duplicate_or_backward=" << stream.backwards
        << " invalid_or_DM_fault=" << stream.invalid
        << " parser_update_events=" << stream.parser_updates
        << " max_observed_silence=" << stream.max_silence_ns / 1000.0
        << " us\n";
    Distribution(out, "MCU observation -> IN submit", stream.queue);
    Distribution(out, "host interarrival", stream.gaps);
    if (channel == 6 || channel == 7) {
      stream.max_parser_silence_ns = std::max(
          stream.max_parser_silence_ns,
          end - (stream.last_parser_ns ? stream.last_parser_ns : start));
      Distribution(out, "parser state update interval", stream.parser_gaps);
      out << "  max parser-state silence="
          << stream.max_parser_silence_ns / 1000.0 << " us\n";
      if (stream.max_parser_silence_ns > gap_limits_us[channel] * 1000)
        passed = false;
    }
    if (channel < 9 &&
        (!stream.seen || stream.invalid || stream.missing || stream.backwards ||
         stream.max_silence_ns > gap_limits_us[channel] * 1000 ||
         ((channel == 6 || channel == 7) && !stream.parser_updates)))
      passed = false;
  }
  for (size_t i = 0; i < ready_count_; ++i) {
    const auto *data = ready_[i].data();
    if (bridge::U32(data + 28) == 0x34545255 &&
        (bridge::U32(data + 32) & (32U | 64U)))
      passed = false;
  }
  PrintDiagnostics(out);
  out << "capture_overflow=" << overflow_
      << " stream_coverage=" << (passed ? "PASS" : "FAIL") << '\n'
      << "CAN TX completion/coalescing/errors: UNMEASURED; feedback is not "
         "command acknowledgment.\n"
      << "Not measured: full UART-wire/CAN-command-to-host RTT; no algorithm "
         "executes.\n";
  received.close();
  timing.close();
  if (!received || !timing)
    throw std::runtime_error("Capture CSV write failed");
  return passed;
}
} // namespace wbr::load_test
