/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_controller.h"
#include "motor_commands.hpp"
#include "usb_link.hpp"
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <wbr/realtime_loop.hpp>
namespace {
volatile std::sig_atomic_t stopping = 0;
void Signal(int) { stopping = 1; }
} // namespace
int main(int argc, char **argv) {
  try {
    bool simulate = false, enable_output = false;
    double seconds = 0;
    int control_cpu = 3;
    wbr::host::UsbOptions usb;
    for (int i = 1; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--simulate") {
        simulate = true;
        continue;
      }
      if (option == "--no-rt") {
        usb.realtime = false;
        continue;
      }
      if (option == "--enable-output") {
        enable_output = true;
        continue;
      }
      if (option == "--strict-hi91-crc") {
        usb.strict_hi91_crc = true;
        continue;
      }
      if (option == "--help") {
        std::cout
            << "chassis_controller [--cpu 3] [--usb-cpu 3] [--priority 80] "
               "[--seconds N]\n"
            << "  Default: disabled actuator policy; remote disabled pose must "
               "be observed\n"
            << "  after startup before --enable-output can honor the remote "
               "enable switch.\n"
            << "  --simulate --no-rt runs the real core without opening USB.\n";
        return 0;
      }
      if (++i == argc)
        throw std::runtime_error("Missing value for " + option);
      if (option == "--cpu")
        control_cpu = std::stoi(argv[i]);
      else if (option == "--usb-cpu")
        usb.cpu = std::stoi(argv[i]);
      else if (option == "--priority")
        usb.priority = std::stoi(argv[i]);
      else if (option == "--seconds")
        seconds = std::stod(argv[i]);
      else
        throw std::runtime_error("Unknown option " + option);
    }
    if (control_cpu < 0 || control_cpu >= CPU_SETSIZE || usb.cpu < 0 ||
        usb.cpu >= CPU_SETSIZE || usb.priority < 1 || usb.priority > 99 ||
        seconds < 0 || !std::isfinite(seconds))
      throw std::runtime_error("Invalid runtime options");
    if (simulate && seconds == 0)
      seconds = 1;
    if (simulate && enable_output)
      throw std::runtime_error("Simulation cannot enable real outputs");
    std::signal(SIGINT, Signal);
    std::signal(SIGTERM, Signal);
    wbr::host::SensorStore sensors;
    wbr::host::MotorCommands commands;
    modules::ChassisController controller(commands);
    std::unique_ptr<wbr::host::UsbLink> link;
    std::string error;
    if (usb.realtime && !wbr::realtime::ConfigureCurrentThread(
                            control_cpu, usb.priority, error))
      throw std::runtime_error(error);
    if (!simulate) {
      link = std::make_unique<wbr::host::UsbLink>(sensors, usb);
      if (!link->Open())
        throw std::runtime_error(link->error());
      std::cout << "Bridge capabilities=0x" << std::hex << link->capabilities()
                << std::dec << "; control IMU=HI91" << std::endl;
    }
    std::cout << "chassis_controller: 1 kHz, "
              << (simulate ? "offline" : "USB Bulk / CAN WBD")
              << ", output=" << (enable_output ? "remote-gated" : "disabled")
              << std::endl;
    const uint64_t start = wbr::realtime::MonotonicTimeNs();
    uint64_t release = start;
    const uint64_t finish =
        seconds ? start + uint64_t(seconds * 1e9) : UINT64_MAX;
    bool disabled_observed = false;
    unsigned late_cycles = 0;
    while (!stopping && wbr::realtime::MonotonicTimeNs() < finish) {
      release += 1000000;
      if (!wbr::realtime::SleepUntilNs(release, error)) {
        if (stopping)
          break;
        if (link)
          link->Stop();
        throw std::runtime_error(error);
      }
      const uint64_t now = wbr::realtime::MonotonicTimeNs();
      // Never replay catch-up control ticks. A >=20 ms stall also exceeds
      // the MCU watchdog and requires a new session, not automatic rearm.
      if (now - release >= wbr::bridge::kCommandTimeoutUs * 1000ULL) {
        if (link)
          link->Stop();
        throw std::runtime_error("Control deadline exceeded bridge watchdog");
      }
      if (now - release >= 1000000) {
        const auto missed = (now - release) / 1000000;
        release += missed * 1000000;
        late_cycles += missed;
      }
      if (link && !link->healthy()) {
        link->Stop();
        throw std::runtime_error(link->error().empty()
                                     ? "Bridge sensor stream timeout"
                                     : link->error());
      }
      const auto snapshot = sensors.Read();
      disabled_observed |= snapshot.control.remote_disable_pose;
      commands.Begin();
      controller.Tick(snapshot.control, wbr::realtime::MonotonicTimeNs() / 1000,
                      enable_output && disabled_observed);
      if (link && !link->Send(commands.packet())) {
        link->Stop();
        throw std::runtime_error(link->error());
      }
    }
    if (link) {
      link->Stop();
      link->Close();
    }
    std::cout << "Stopped; missed control releases=" << late_cycles
              << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
