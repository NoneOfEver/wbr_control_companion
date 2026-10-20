/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_link.hpp"
#include <chrono>
#include <iostream>
#include <thread>
int main() {
  wbr::host::SensorStore sensors;
  wbr::host::UsbOptions options;
  options.realtime = false;
  for (unsigned session = 0; session < 5; ++session) {
    wbr::host::UsbLink link(sensors, options);
    if (!link.Open()) {
      std::cerr << link.error();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    // New Hello clears old state; injected old-session packets must stay
    // rejected.
    if (sensors.Read().control.motors[0].valid)
      return 2;
    for (unsigned tick = 0; tick < 3; ++tick) {
      wbr::bridge::Commands empty{};
      if (!link.Send(empty))
        return 3;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!sensors.Read().control.motors[0].valid)
      return 4;
    link.Stop();
    link.Close();
  }
  std::cout << "Five USB sessions: state reset, stale-session rejection, new "
               "sequence and drain passed\n";
}
