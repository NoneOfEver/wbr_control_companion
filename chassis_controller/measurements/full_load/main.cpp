/* SPDX-License-Identifier: Apache-2.0 */
#include "capture.hpp"
#include "zero_commands.hpp"
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <wbr/realtime_loop.hpp>
namespace {
namespace rt = wbr::realtime;
volatile std::sig_atomic_t stopping = 0;
void Signal(int) { stopping = 1; }
// Ensure every exception after Open still requests the existing local Stop.
struct SessionGuard {
  wbr::host::UsbLink &link;
  bool finished = false;
  void Finish() {
    if (!finished) {
      link.Stop();
      link.Close();
      finished = true;
    }
  }
  ~SessionGuard() { Finish(); }
};
} // namespace
int main(int argc, char **argv) {
  try {
    double seconds = 60;
    int control_cpu = 3;
    std::filesystem::path directory;
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
            << "full_load_zero --output NEW_DIRECTORY [--seconds 60] [--cpu 3] "
               "[--usb-cpu 3] [--priority 80] [--no-rt] [--strict-hi91-crc]\n";
        return 0;
      }
      if (++i == argc)
        throw std::runtime_error("Missing value for " + option);
      if (option == "--seconds")
        seconds = std::stod(argv[i]);
      else if (option == "--output")
        directory = argv[i];
      else if (option == "--cpu")
        control_cpu = std::stoi(argv[i]);
      else if (option == "--usb-cpu")
        options.cpu = std::stoi(argv[i]);
      else if (option == "--priority")
        options.priority = std::stoi(argv[i]);
      else
        throw std::runtime_error("Unknown option " + option);
    }
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 120 ||
        directory.empty() || control_cpu < 0 || control_cpu >= CPU_SETSIZE ||
        options.cpu < 0 || options.cpu >= CPU_SETSIZE || options.priority < 1 ||
        options.priority > 99)
      throw std::runtime_error(
          "Invalid options: duration (0,120], new output directory required");
    if (!std::filesystem::create_directories(directory))
      throw std::runtime_error(
          "Output directory already exists; refusing to overwrite");
    std::signal(SIGINT, Signal);
    std::signal(SIGTERM, Signal);
    wbr::host::SensorStore sensors;
    wbr::load_test::Capture capture(sensors, seconds);
    const auto commands = wbr::load_test::ZeroOutputCommands();
    std::vector<wbr::load_test::Cycle> cycles;
    cycles.reserve(size_t(seconds * 1000) + 2);
    options.observe = wbr::load_test::Capture::Observe;
    options.observer_context = &capture;
    options.command_completed = wbr::load_test::Capture::CommandCompleted;
    std::string error;
    if (options.realtime &&
        !rt::ConfigureCurrentThread(control_cpu, options.priority, error))
      throw std::runtime_error(error);
    wbr::host::UsbLink link(sensors, options);
    if (!link.Open())
      throw std::runtime_error(link.error());
    SessionGuard guard{link};
    if ((link.capabilities() & 15) != 15)
      throw std::runtime_error("Full-load test requires CAN, SBUS, HI91 and "
                               "onboard IMU capabilities");
    std::cout << "Full-load zero-output: one 128-byte Bulk OUT/cycle, six CAN "
                 "frames, 1000 Hz; "
                 "wheels=zero current, joints=EXIT every cycle; no "
                 "Enter/MIT/algorithm.\n"
              << "RT=" << options.realtime << " control_cpu=" << control_cpu
              << " USB_cpu=" << options.cpu
              << " FIFO_priority=" << options.priority
              << "; recording in memory.\n"
              << std::flush;
    const uint64_t start = rt::MonotonicTimeNs();
    const uint64_t finish = start + uint64_t(seconds * 1e9);
    uint64_t release = start + 1000000, skipped = 0;
    while (!stopping && release <= finish) {
      if (!rt::SleepUntilNs(release, error))
        break;
      const uint64_t wake = rt::MonotonicTimeNs();
      if (wake - release >= wbr::bridge::kCommandTimeoutUs * 1000ULL) {
        error = "Host deadline exceeded command lease";
        break;
      }
      const uint32_t missed = (wake - release) / 1000000;
      skipped += missed;
      // Exercise the production state-store reader alongside the USB writer.
      const auto snapshot = sensors.Read();
      uint16_t valid_mask = 0;
      for (unsigned motor = 0; motor < 6; ++motor)
        if (snapshot.control.motors[motor].valid)
          valid_mask |= 1U << motor;
      if (snapshot.control.remote.sequence)
        valid_mask |= 1U << 6;
      if (snapshot.control.imu.valid)
        valid_mask |= 1U << 7;
      if (snapshot.peripheral.onboard_sample_us)
        valid_mask |= 1U << 8;
      if (!link.Send(commands)) {
        error = "USB session unhealthy or OUT submission failed";
        break;
      }
      cycles.push_back(
          {release, wake, rt::MonotonicTimeNs(), missed, valid_mask});
      release += (uint64_t(missed) + 1) *
                 1000000; // Never replay missed slots in a burst.
    }
    const uint64_t end = rt::MonotonicTimeNs();
    const bool complete = !stopping && error.empty() && release > finish;
    capture.StopRecording();
    guard.Finish();
    if (error.empty())
      error = link.error();
    // One final, stopped handshake reads cumulative UART diagnostic flags only.
    wbr::host::UsbLink status(sensors, options);
    if (!status.Open()) {
      if (error.empty())
        error = "Final status: " + status.error();
    } else
      status.Stop();
    status.Close();
    std::ostringstream summary;
    summary << "termination="
            << (complete ? "duration" : (stopping ? "signal" : "error"))
            << '\n';
    const bool coverage =
        capture.Export(directory, start, end, cycles, summary);
    const bool passed = complete && error.empty() && coverage &&
                        (!options.realtime || !skipped);
    summary << "RESULT=" << (passed ? "HOST_OBSERVED_PASS" : "FAIL") << '\n';
    if (!error.empty())
      summary << "error=" << error << '\n';
    std::ofstream report(directory / "summary.txt");
    report << summary.str();
    report.close();
    if (!report)
      throw std::runtime_error("Summary write failed");
    std::cout << summary.str() << "Saved: " << directory << '\n';
    return passed ? 0 : 1;
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
