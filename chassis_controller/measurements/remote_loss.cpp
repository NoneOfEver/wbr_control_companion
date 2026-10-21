/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_input_reader.h"
#include "full_load/zero_commands.hpp"
#include "usb_link.hpp"
#include <csignal>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <wbr/realtime_loop.hpp>
namespace {
volatile std::sig_atomic_t stopping = 0;
void Signal(int) { stopping = 1; }
struct Row {
  uint64_t elapsed_us;
  uint32_t sequence, age_ms;
  bool failsafe, lost, fresh, requested;
};
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("Usage: remote_loss NEW.csv");
    // Refuse overwrite before opening USB. Export only after shutdown.
    if (std::ifstream(argv[1]).good())
      throw std::runtime_error("CSV already exists");
    std::vector<Row> rows;
    rows.reserve(60000);
    wbr::host::SensorStore store;
    wbr::host::UsbOptions options;
    std::string error;
    if (!wbr::realtime::ConfigureCurrentThread(3, 80, error))
      throw std::runtime_error(error);
    wbr::host::UsbLink link(store, options);
    if (!link.Open())
      throw std::runtime_error(link.error());
    std::signal(SIGINT, Signal);
    std::signal(SIGTERM, Signal);
    modules::ChassisInputReader reader;
    const auto zero = wbr::load_test::ZeroOutputCommands();
    const uint64_t start = wbr::realtime::MonotonicTimeNs();
    uint64_t release = start;
    bool transport_ok = true;
    std::cout
        << "60 s stop-only capture; no controller algorithm or Enter/MIT.\n"
           "保持遥控器开启约5秒，然后关闭约20秒，再开启；保持接收机和MCU供电。"
           "\n"
           "不要拔USB或操作电机；完成后导出CSV。"
        << std::endl;
    while (!stopping && rows.size() < 60000) {
      release += 1000000;
      if (!wbr::realtime::SleepUntilNs(release, error))
        break;
      const uint64_t now = wbr::realtime::MonotonicTimeNs();
      if (now - start >= 60000000000ULL)
        break;
      if (now - release >= 20000000 || !link.healthy()) {
        transport_ok = false;
        break;
      }
      const auto snapshot = store.Read();
      modules::ChassisCycleInput input{};
      reader.Read(snapshot.control, input, now / 1000, .001, false);
      rows.push_back({(now - start) / 1000, input.remote.sequence,
                      input.remote_age_ms, snapshot.peripheral.sbus_failsafe,
                      snapshot.peripheral.sbus_frame_lost, input.remote_fresh,
                      input.requested_enable});
      if (!link.Send(zero)) {
        transport_ok = false;
        break;
      }
      if (now - release >= 1000000)
        release = now;
    }
    link.Stop();
    link.Close();
    std::ofstream csv(argv[1]);
    if (!csv)
      throw std::runtime_error("Cannot create CSV");
    csv << "elapsed_us,sequence,age_ms,failsafe,frame_lost,fresh,requested_"
           "enable\n";
    uint64_t flagged = 0, stale = 0, violations = 0;
    for (const auto &r : rows) {
      csv << r.elapsed_us << ',' << r.sequence << ',' << r.age_ms << ','
          << r.failsafe << ',' << r.lost << ',' << r.fresh << ',' << r.requested
          << '\n';
      flagged += r.sequence && (r.failsafe || r.lost);
      stale += r.sequence && !r.fresh;
      violations += r.requested && (!r.fresh || r.failsafe || r.lost);
    }
    csv.close();
    if (!csv)
      throw std::runtime_error("CSV export failed");
    std::cout << "samples=" << rows.size() << " flagged_ticks=" << flagged
              << " stale_ticks=" << stale
              << " unsafe_enable_requests=" << violations
              << " transport_ok=" << transport_ok << " saved=" << argv[1]
              << '\n'
              << "未观察到的失效类型不算通过；不验证实际电机停机或重新使能。\n";
    return transport_ok && !violations ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
