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
  // Written by event owner, read only after Close joins it.
  uint64_t event_calls = 0, fast_run = 0, max_fast_run = 0;
  uint64_t max_fast_begin_ns = 0, max_fast_end_ns = 0, fast_begin_ns = 0;
  uint64_t first_transfer_error_ns = 0, first_event_error_ns = 0;
  int first_transfer_status = -1, first_event_result = 0;
  unsigned first_transfer_kind = 0;
  // Written by control owner; same host monotonic clock as event observations.
  uint64_t last_good_wake_ns = 0, fault_wake_ns = 0, fault_release_ns = 0;
  uint64_t unhealthy_ns = 0;
  static void Trace(void *context, unsigned kind, int result, uint64_t begin,
                    uint64_t end) {
    auto &self = *static_cast<Probe *>(context);
    if (kind != 0) {
      if (!self.first_transfer_error_ns) {
        self.first_transfer_error_ns = begin;
        self.first_transfer_status = result;
        self.first_transfer_kind = kind;
      }
      return;
    }
    ++self.event_calls;
    if (result && !self.first_event_error_ns) {
      self.first_event_error_ns = end;
      self.first_event_result = result;
    }
    // Fast calls can also be ordinary data traffic; this alone proves no spin.
    if (end - begin < 10000) {
      if (!self.fast_run)
        self.fast_begin_ns = begin;
      if (++self.fast_run > self.max_fast_run) {
        self.max_fast_run = self.fast_run;
        self.max_fast_begin_ns = self.fast_begin_ns;
        self.max_fast_end_ns = end;
      }
    } else
      self.fast_run = 0;
  }
  void PrintTrace(const std::string &error) const {
    std::cerr << "USB fault trace (host monotonic ns):"
              << " event_calls=" << event_calls
              << " max_fast_run=" << max_fast_run
              << " fast_begin=" << max_fast_begin_ns
              << " fast_end=" << max_fast_end_ns
              << " first_event_error=" << first_event_error_ns
              << " event_result=" << first_event_result
              << " first_transfer_error=" << first_transfer_error_ns
              << " transfer_kind=" << first_transfer_kind
              << " transfer_status=" << first_transfer_status
              << " last_good_wake=" << last_good_wake_ns
              << " fault_wake=" << fault_wake_ns
              << " fault_release=" << fault_release_ns
              << " unhealthy=" << unhealthy_ns << " link_error=[" << error
              << "]\n";
  }
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
  Probe &probe;
  ~Session() {
    link.Stop();
    link.Close();
    probe.PrintTrace(link.error());
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
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 60)
      throw std::runtime_error("Worker duration must be (0,60] seconds");
    std::signal(SIGINT, Signal);
    std::signal(SIGTERM, Signal);
    wbr::host::SensorStore sensors;
    Probe probe;
    options.ready_query_page = wire::kFaultQueryPage;
    options.observe = Probe::Observe;
    options.observer_context = &probe;
    options.fault_trace = Probe::Trace;
    std::string error;
    if (options.realtime && !rt::ConfigureCurrentThread(3, 80, error))
      throw std::runtime_error(error);
    wbr::host::UsbLink link(sensors, options);
    if (!link.Open())
      throw std::runtime_error(link.error());
    Session session{link, probe};
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
        probe.fault_wake_ns = now;
        probe.fault_release_ns = release;
        std::cerr << "Command deadline expired; no rearm or new Hello\n";
        return 2;
      }
      if (stopping)
        break;
      probe.last_good_wake_ns = now;
      if (!link.Send(zero)) {
        probe.unhealthy_ns = rt::MonotonicTimeNs();
        throw std::runtime_error("USB session unhealthy; no reconnect/rearm");
      }
      if (now - release >= 1000000)
        release = now;
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
