/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_config.h"
#include "chassis_input_reader.h"
#include "full_load/zero_commands.hpp"
#include "usb_link.hpp"
#include <array>
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
  uint32_t imu_sequence;
  uint64_t imu_age_us;
  bool imu_valid, imu_fresh;
  wbr::host::Hi91Diagnostics hi91;
  std::array<uint32_t, 6> motor_age{};
  uint8_t motor_stale = 0, feedback_invalid = 0;
  bool feedback_valid = false;
};
} // namespace
int main(int argc, char **argv) {
  try {
    const bool baseline = argc == 3 && std::string(argv[1]) == "--imu-baseline";
    const bool imu_mode =
        baseline || (argc == 3 && std::string(argv[1]) == "--imu");
    const bool motor_mode = argc == 3 && std::string(argv[1]) == "--motors";
    if (argc != 2 && !imu_mode && !motor_mode)
      throw std::runtime_error("Usage: remote_loss [--imu] NEW.csv");
    const char *output = argv[(imu_mode || motor_mode) ? 2 : 1];
    // Refuse overwrite before opening USB. Export only after shutdown.
    if (std::ifstream(output).good())
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
    std::string termination = "duration";
    uint64_t exit_elapsed_us = 0, exit_lateness_us = 0;
    std::cout
        << "60 s stop-only capture; no controller algorithm or Enter/MIT.\n";
    if (motor_mode)
      std::cout
          << "电机反馈失联：正常约5秒，只断左侧或右侧一组电机电源约20秒，再恢复"
             "。\n"
             "保持MCU、USB、传感器及另一组电机供电；不要拔CAN或改变终端电阻。\n"
          << "Motor timeout_us=" << modules::chassis_config::kMotorTimeoutUs
          << '\n';
    else if (baseline)
      std::cout
          << "HI91静态对照：60秒内保持所有插头连接，不插拔、不切断电源。\n";
    else if (imu_mode)
      std::cout << "HI91测试：正常约5秒，断开HI91专用插头约20秒，再恢复。\n"
                   "保持MCU、USB及其他设备供电连接；不要断开共用电源。\n"
                << "Production IMU timeout_ms="
                << modules::chassis_config::kImuTimeoutMs << '\n';
    else
      std::cout
          << "遥控输入测试：按本轮约定关闭遥控器或断开接收机，约20秒后恢复。\n";
    std::cout << "保持MCU和USB正常；完成后导出CSV。" << std::endl;
    while (!stopping && rows.size() < 60000) {
      release += 1000000;
      if (!wbr::realtime::SleepUntilNs(release, error)) {
        termination = stopping ? "signal" : "sleep_error";
        transport_ok = stopping != 0;
        break;
      }
      const uint64_t now = wbr::realtime::MonotonicTimeNs();
      if (now - start >= 60000000000ULL)
        break;
      exit_elapsed_us = (now - start) / 1000;
      exit_lateness_us = (now - release) / 1000;
      if (now - release >= 20000000) {
        termination = "control_deadline";
        transport_ok = false;
        break;
      }
      if (!link.healthy()) {
        termination = "usb_unhealthy";
        transport_ok = false;
        break;
      }
      const auto snapshot = store.Read();
      modules::ChassisCycleInput input{};
      reader.Read(snapshot.control, input, now / 1000, .001, false);
      Row row{(now - start) / 1000,
              input.remote.sequence,
              input.remote_age_ms,
              snapshot.peripheral.sbus_failsafe,
              snapshot.peripheral.sbus_frame_lost,
              input.remote_fresh,
              input.requested_enable,
              input.imu.sequence,
              input.imu_age_us,
              input.imu.valid,
              input.imu_fresh,
              snapshot.peripheral.hi91};
      std::copy_n(input.motor_feedback_age_us, 6, row.motor_age.begin());
      row.motor_stale = input.motor_stale_mask;
      row.feedback_invalid = input.feedback_invalid_mask;
      row.feedback_valid = input.feedback_valid;
      rows.push_back(row);
      if (!link.Send(zero)) {
        termination = "usb_send_failed";
        transport_ok = false;
        break;
      }
      if (now - release >= 1000000)
        release = now;
    }
    if (stopping)
      termination = "signal";
    const auto usb_error = link.error();
    link.Stop();
    link.Close();
    std::ofstream csv(output);
    if (!csv)
      throw std::runtime_error("Cannot create CSV");
    csv << "elapsed_us,sequence,age_ms,failsafe,frame_lost,fresh,requested_"
           "enable,imu_sequence,imu_age_us,imu_valid,imu_fresh,hi91_chunks,"
           "hi91_bytes,"
           "hi91_gaps,hi91_resync_bytes,hi91_length_errors,hi91_decode_errors,"
           "hi91_attitude_errors,hi91_nonfinite_errors,hi91_equal_time,hi91_"
           "backward_time,"
           "hi91_accepted,hi91_candidate_ms,hi91_accepted_ms,hi91_rejected_ms,"
           "hi91_last_chunk_host_us,hi91_time_rebases,motor_stale_mask,"
           "feedback_invalid_mask,feedback_valid,left_wheel_age_us,right_wheel_"
           "age_us,left_joint_b_age_us,left_joint_d_age_us,right_joint_b_age_"
           "us,right_joint_d_age_us\n";
    uint64_t flagged = 0, stale = 0, violations = 0, imu_stale = 0;
    std::array<uint64_t, 6> motor_stale_ticks{};
    for (const auto &r : rows) {
      csv << r.elapsed_us << ',' << r.sequence << ',' << r.age_ms << ','
          << r.failsafe << ',' << r.lost << ',' << r.fresh << ',' << r.requested
          << ',' << r.imu_sequence << ',' << r.imu_age_us << ',' << r.imu_valid
          << ',' << r.imu_fresh << ',' << r.hi91.chunks << ',' << r.hi91.bytes
          << ',' << r.hi91.gaps << ',' << r.hi91.resync_bytes << ','
          << r.hi91.length_errors << ',' << r.hi91.decode_errors << ','
          << r.hi91.attitude_errors << ',' << r.hi91.nonfinite_errors << ','
          << r.hi91.equal_time << ',' << r.hi91.backward_time << ','
          << r.hi91.accepted << ',' << r.hi91.last_candidate_ms << ','
          << r.hi91.last_accepted_ms << ',' << r.hi91.last_rejected_ms << ','
          << r.hi91.last_chunk_host_us << ',' << r.hi91.time_rebases << ','
          << unsigned(r.motor_stale) << ',' << unsigned(r.feedback_invalid)
          << ',' << r.feedback_valid;
      for (unsigned i = 0; i < 6; ++i) {
        csv << ',' << r.motor_age[i];
        motor_stale_ticks[i] += bool(r.motor_stale & (1U << i));
      }
      csv << '\n';
      imu_stale += r.imu_sequence && !r.imu_fresh;
      flagged += r.sequence && (r.failsafe || r.lost);
      stale += r.sequence && !r.fresh;
      // MCU retains prior state for isolated frame_lost; it takes priority
      // over failsafe when both bits occur in the same discarded frame.
      violations += r.requested && (!r.fresh || (r.failsafe && !r.lost));
    }
    csv.close();
    if (!csv)
      throw std::runtime_error("CSV export failed");
    std::ofstream metadata(std::string(output) + ".status.txt");
    metadata << "termination=" << termination
             << " elapsed_us=" << exit_elapsed_us
             << " lateness_us=" << exit_lateness_us << " usb_error=["
             << usb_error << "] sleep_error=[" << error << "]\n";
    metadata.close();
    if (!metadata)
      throw std::runtime_error("Status export failed");
    std::cout << "termination=" << termination
              << " elapsed_us=" << exit_elapsed_us
              << " lateness_us=" << exit_lateness_us << " usb_error=["
              << usb_error << "] sleep_error=[" << error << "]\n";
    std::cout << "samples=" << rows.size() << " flagged_ticks=" << flagged
              << " stale_ticks=" << stale
              << " unsafe_enable_requests=" << violations
              << " imu_stale_ticks=" << imu_stale
              << " transport_ok=" << transport_ok << " saved=" << output << '\n'
              << "未观察到的失效类型不算通过；不验证实际电机停机或重新使能。\n";
    if (!rows.empty() && imu_mode) {
      const auto &d = rows.back().hi91;
      std::cout << "HI91 chunks=" << d.chunks << " bytes=" << d.bytes
                << " gaps=" << d.gaps << " accepted=" << d.accepted
                << " decode_errors=" << d.decode_errors
                << " attitude_errors=" << d.attitude_errors
                << " nonfinite_errors=" << d.nonfinite_errors
                << " equal_time=" << d.equal_time
                << " backward_time=" << d.backward_time
                << " time_rebases=" << d.time_rebases
                << " candidate_ms=" << d.last_candidate_ms
                << " accepted_ms=" << d.last_accepted_ms
                << " rejected_ms=" << d.last_rejected_ms << '\n';
    }
    if (motor_mode) {
      constexpr const char *names[] = {"left_wheel",    "right_wheel",
                                       "left_joint_b",  "left_joint_d",
                                       "right_joint_b", "right_joint_d"};
      for (unsigned i = 0; i < 6; ++i)
        std::cout << names[i] << " stale_ticks=" << motor_stale_ticks[i]
                  << " final_age_us="
                  << (rows.empty() ? UINT32_MAX : rows.back().motor_age[i])
                  << '\n';
      std::cout << "startup included; left mask=13 right mask=50; inspect CSV "
                   "transitions, not only totals.\n";
    }
    return transport_ok && !violations ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
