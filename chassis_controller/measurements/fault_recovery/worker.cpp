/* SPDX-License-Identifier: Apache-2.0 */
#include "../full_load/zero_commands.hpp"
#include "usb_link.hpp"
#include <cmath>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <wbr/realtime_loop.hpp>
#include <wbr_bridge/fault_diagnostics.hpp>
namespace {
namespace wire = wbr::bridge;
namespace rt = wbr::realtime;
volatile std::sig_atomic_t stopping = 0;
void Signal(int) { stopping = 1; }
struct Probe {
  wire::FaultSnapshot snapshot;
  bool matched = false;
  static void Observe(void *context, const wire::Packet &packet, uint64_t,
                      uint64_t) {
    if (packet[9] == uint8_t(wire::Kind::Ready)) {
      auto &self = *static_cast<Probe *>(context);
      self.matched = wire::DecodeFaultSnapshot(packet, self.snapshot);
    }
  }
  void Print(uint32_t session) const {
    std::cout << "{\"event\":\"ready\",\"session\":" << session
              << ",\"snapshot\":{"
              << "\"accepted\":" << snapshot.accepted
              << ",\"rejected\":" << snapshot.rejected
              << ",\"lease_stops\":" << snapshot.lease_stops
              << ",\"explicit_stops\":" << snapshot.explicit_stops
              << ",\"last_accepted_us\":" << snapshot.last_accepted_us
              << ",\"last_stop_submit_us\":" << snapshot.last_stop_submit_us
              << ",\"stop_flags\":" << snapshot.stop_flags
              << ",\"prior_session\":" << snapshot.prior_session << "}}"
              << std::endl;
  }
};
struct Session {
  wbr::host::UsbLink &link;
  ~Session() {
    link.Stop();
    link.Close();
  }
};
} // namespace
int main(int argc, char **argv) {
  try {
    bool query = false;
    double seconds = 5;
    wbr::host::UsbOptions options;
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--query")
        query = true;
      else if (argument == "--no-rt")
        options.realtime = false;
      else if (argument == "--seconds" && ++i < argc)
        seconds = std::stod(argv[i]);
      else
        throw std::runtime_error("Unknown/missing argument: " + argument);
    }
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 10)
      throw std::runtime_error("Worker duration must be (0,10] seconds");
    std::signal(SIGINT, Signal);
    std::signal(SIGTERM, Signal);
    wbr::host::SensorStore sensors;
    Probe probe;
    options.ready_query_page = wire::kFaultQueryPage;
    options.observe = Probe::Observe;
    options.observer_context = &probe;
    std::string error;
    if (options.realtime && !rt::ConfigureCurrentThread(3, 80, error))
      throw std::runtime_error(error);
    wbr::host::UsbLink link(sensors, options);
    if (!link.Open())
      throw std::runtime_error(link.error());
    Session session{link};
    if (!probe.matched)
      throw std::runtime_error("FLT1 diagnostics unavailable: flash fault "
                               "diagnostic image before injecting faults");
    probe.Print(link.session());
    if (query)
      return 0;
    const auto zero = wbr::load_test::ZeroOutputCommands();
    const auto start = rt::MonotonicTimeNs();
    uint64_t release = start;
    while (!stopping &&
           rt::MonotonicTimeNs() - start < uint64_t(seconds * 1e9)) {
      release += 1000000;
      if (!rt::SleepUntilNs(release, error))
        throw std::runtime_error(error);
      const auto now = rt::MonotonicTimeNs();
      // SIGCONT cannot resume stale commands or silently create a new session.
      if (now - release >= wire::kCommandTimeoutUs * 1000ULL) {
        std::cerr << "Command deadline expired; no rearm or new Hello\n";
        return 2;
      }
      if (stopping)
        break;
      if (!link.Send(zero))
        throw std::runtime_error("USB session unhealthy; no reconnect/rearm");
      if (now - release >= 1000000)
        release = now;
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
