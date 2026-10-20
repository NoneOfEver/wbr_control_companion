/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_link.hpp"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <wbr/realtime_loop.hpp>

namespace {
namespace wire = wbr::bridge;
namespace rt = wbr::realtime;
volatile std::sig_atomic_t stopping = 0;
void Signal(int) { stopping = 1; }
struct Sample {
  uint64_t arrival_ns = 0, decode_ns = 0;
  uint32_t submit_us = 0, observed_us = 0, sequence = 0;
  uint32_t remote_sequence = 0, imu_sequence = 0;
  uint8_t port = 0, bytes = 0;
};
struct Capture {
  wbr::host::SensorStore &sensors;
  std::vector<Sample> samples;
  size_t used = 0, overflow = 0;
  static void Observe(void *context, const wire::Packet &packet,
                      uint64_t arrival, uint64_t decoded) {
    if (packet[9] != uint8_t(wire::Kind::Uart))
      return;
    auto &self = *static_cast<Capture *>(context);
    if (self.used == self.samples.size()) {
      ++self.overflow;
      return;
    }
    const auto snapshot = self.sensors.Read();
    auto &sample = self.samples[self.used++];
    sample = {arrival,
              decoded - arrival,
              wire::U32(packet.data() + 4),
              wire::U32(packet.data() + 20),
              wire::U32(packet.data() + 16),
              snapshot.control.remote.sequence,
              snapshot.control.imu.sequence,
              packet[10],
              packet[11]};
  }
};
void Distribution(const char *name, std::vector<double> values) {
  std::cout << "  " << name << ": ";
  if (values.empty()) {
    std::cout << "unavailable\n";
    return;
  }
  std::sort(values.begin(), values.end());
  auto percentile = [&](double fraction) {
    return values[size_t(std::ceil(fraction * values.size())) - 1];
  };
  std::cout << "p50/p99/max=" << percentile(.5) << '/' << percentile(.99) << '/'
            << values.back() << " us n=" << values.size() << '\n';
}
void Summarize(const Capture &capture, unsigned port) {
  const Sample *previous = nullptr;
  uint64_t gaps = 0, backwards = 0, updates = 0, bytes = 0;
  uint32_t last_update = 0;
  uint64_t last_update_ns = 0;
  std::vector<double> queue, intervals, decode, frame_intervals;
  for (size_t i = 0; i < capture.used; ++i) {
    const auto &sample = capture.samples[i];
    if (sample.port != port)
      continue;
    bytes += sample.bytes;
    queue.push_back(uint32_t(sample.submit_us - sample.observed_us));
    decode.push_back(sample.decode_ns / 1000.0);
    if (previous) {
      const uint32_t advance = sample.sequence - previous->sequence;
      if (advance && advance < 0x80000000U)
        gaps += advance - 1;
      else
        ++backwards;
      intervals.push_back((sample.arrival_ns - previous->arrival_ns) / 1000.0);
    }
    const uint32_t update =
        port == 1 ? sample.remote_sequence : sample.imu_sequence;
    if (port <= 2 && update != last_update) {
      // Parser state updates can also include invalidation. This is
      // deliberately not labelled a count of CRC-valid sensor frames.
      ++updates;
      if (last_update_ns)
        frame_intervals.push_back((sample.arrival_ns - last_update_ns) /
                                  1000.0);
      last_update_ns = sample.arrival_ns;
      last_update = update;
    }
    previous = &sample;
  }
  std::cout << "UART port=" << port << " chunks=" << queue.size()
            << " bytes=" << bytes << " missing_chunks=" << gaps
            << " duplicate_or_backward=" << backwards
            << " parser_update_events=" << updates << '\n';
  Distribution("MCU software observation -> USB IN submit", queue);
  Distribution("NUC USB chunk interarrival", intervals);
  Distribution("NUC decode and state publication", decode);
  if (port <= 2)
    Distribution("NUC parser state update interval", frame_intervals);
}
} // namespace
int main(int argc, char **argv) {
  try {
    double seconds = 60;
    std::string csv = "uart_forwarding.csv";
    wbr::host::UsbOptions options;
    for (int i = 1; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--no-rt") {
        options.realtime = false;
        continue;
      }
      if (option == "--strict-hi91-crc") {
        options.strict_hi91_crc = true;
        continue;
      }
      if (option == "--help") {
        std::cout
            << "uart_forwarding [--seconds 60] [--csv path] [--usb-cpu 3] "
               "[--priority 80] [--no-rt] [--strict-hi91-crc]\n";
        return 0;
      }
      if (++i == argc)
        throw std::runtime_error("Missing value for " + option);
      if (option == "--seconds")
        seconds = std::stod(argv[i]);
      else if (option == "--csv")
        csv = argv[i];
      else if (option == "--usb-cpu")
        options.cpu = std::stoi(argv[i]);
      else if (option == "--priority")
        options.priority = std::stoi(argv[i]);
      else
        throw std::runtime_error("Unknown option " + option);
    }
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 120 ||
        options.cpu < 0 || options.cpu >= CPU_SETSIZE || options.priority < 1 ||
        options.priority > 99)
      throw std::runtime_error("Invalid capture options (duration <=120 s)");
    std::signal(SIGINT, Signal);
    std::signal(SIGTERM, Signal);
    wbr::host::SensorStore sensors;
    Capture capture{sensors,
                    std::vector<Sample>(size_t(seconds * 20000) + 1000)};
    options.observe = Capture::Observe;
    options.observer_context = &capture;
    wbr::host::UsbLink link(sensors, options);
    if (!link.Open())
      throw std::runtime_error(link.error());
    std::cout
        << "UART-only capture: production Bulk path, empty 1 kHz heartbeats; "
           "no control algorithm or motor-enable commands.\n"
           "Capturing in memory; CSV export follows USB shutdown.\n"
        << std::flush;
    const uint64_t start = rt::MonotonicTimeNs();
    uint64_t release = start;
    std::string error;
    while (!stopping &&
           rt::MonotonicTimeNs() - start < uint64_t(seconds * 1e9)) {
      wire::Commands heartbeat{};
      if (!link.Send(heartbeat))
        break;
      release += 1000000;
      if (!rt::SleepUntilNs(release, error))
        break;
      const auto now = rt::MonotonicTimeNs();
      if (now > release + 1000000)
        release = now;
    }
    const std::string usb_error = link.error();
    link.Stop();
    link.Close(); // joins observer owner before accessing captured memory
    std::ofstream output(csv);
    if (!output)
      throw std::runtime_error("Cannot open CSV: " + csv);
    output << "port,stream_sequence,bytes,host_arrival_ns,mcu_observed_us,"
              "mcu_in_submit_us,observation_to_submit_us,decode_ns,remote_"
              "sequence,imu_sequence\n";
    for (size_t i = 0; i < capture.used; ++i) {
      const auto &s = capture.samples[i];
      output << unsigned(s.port) << ',' << s.sequence << ','
             << unsigned(s.bytes) << ',' << s.arrival_ns << ',' << s.observed_us
             << ',' << s.submit_us << ','
             << uint32_t(s.submit_us - s.observed_us) << ',' << s.decode_ns
             << ',' << s.remote_sequence << ',' << s.imu_sequence << '\n';
    }
    output.close();
    if (!output)
      throw std::runtime_error("CSV write failed: " + csv);
    for (unsigned port = 1; port <= 3; ++port)
      Summarize(capture, port);
    std::cout
        << "capture_overflow=" << capture.overflow << "; saved=" << csv
        << "\nNot measured: UART wire arrival -> software observation, or "
           "USB IN submit -> NUC arrival (unsynchronized clocks).\n";
    if (!usb_error.empty())
      throw std::runtime_error(usb_error);
    if (!error.empty())
      throw std::runtime_error(error);
    if (!capture.used || capture.overflow)
      throw std::runtime_error(
          "Incomplete measurement: no UART data or capture overflow");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
