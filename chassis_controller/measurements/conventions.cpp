/* SPDX-License-Identifier: Apache-2.0 */
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
  uint8_t stale_mask;
  bool imu_fresh, feedback_valid;
  std::array<double, 24> values;
};
struct StopOnExit {
  wbr::host::UsbLink &link;
  ~StopOnExit() {
    link.Stop();
    link.Close();
  }
};
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("Usage: conventions NEW.csv");
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
    StopOnExit cleanup{link};
    std::signal(SIGINT, Signal);
    std::signal(SIGTERM, Signal);
    modules::ChassisInputReader reader;
    const auto zero = wbr::load_test::ZeroOutputCommands();
    const uint64_t start = wbr::realtime::MonotonicTimeNs();
    uint64_t release = start;
    std::string termination = "duration";
    std::cout
        << "60 s convention capture: zero wheel current / joint Exit only.\n"
           "按本轮约定仅做一个手动动作；不运行算法、不使能电机、不设置零点。\n"
           "保持所有插头连接，结束后导出CSV。"
        << std::endl;
    while (!stopping && rows.size() < 60000) {
      release += 1000000;
      if (!wbr::realtime::SleepUntilNs(release, error)) {
        termination = "sleep";
        break;
      }
      const auto now = wbr::realtime::MonotonicTimeNs();
      if (now - start >= 60000000000ULL)
        break;
      if (now - release >= 20000000) {
        termination = "deadline";
        break;
      }
      if (!link.healthy()) {
        termination = "usb_unhealthy";
        break;
      }
      const auto snapshot = store.Read();
      modules::ChassisCycleInput input{};
      reader.Read(snapshot.control, input, now / 1000, .001, false);
      const auto &imu = input.imu;
      const auto &q = input.joint_position;
      const auto &v = input.joint_velocity;
      const auto &wheel = reader.wheel_feedback();
      rows.push_back({(now - start) / 1000,
                      input.motor_stale_mask,
                      input.imu_fresh,
                      input.feedback_valid,
                      {imu.pitch_deg,
                       imu.roll_deg,
                       imu.yaw_deg,
                       imu.pitch_rate_rad_s,
                       imu.roll_rate_rad_s,
                       imu.yaw_rate_rad_s,
                       imu.accel_g[0],
                       imu.accel_g[1],
                       imu.accel_g[2],
                       double(wheel.left.encoder),
                       double(wheel.right.encoder),
                       input.wheel_motor_speed_rpm.left,
                       input.wheel_motor_speed_rpm.right,
                       q.left.b,
                       q.left.d,
                       q.right.b,
                       q.right.d,
                       v.left.b,
                       v.left.d,
                       v.right.b,
                       v.right.d,
                       input.leg.left.length,
                       input.leg.right.length,
                       input.pitch}});
      if (!link.Send(zero)) {
        termination = "usb_send";
        break;
      }
      if (now - release >= 1000000)
        release = now;
    }
    if (stopping)
      termination = "signal";
    const auto link_error = link.error();
    link.Stop();
    link.Close();
    std::ofstream csv(argv[1]);
    if (!csv)
      throw std::runtime_error("Cannot create CSV");
    csv << "elapsed_us,motor_stale_mask,imu_fresh,feedback_valid,pitch_deg,"
           "roll_deg,yaw_deg,"
           "pitch_rate_rad_s,roll_rate_rad_s,yaw_rate_rad_s,ax_g,ay_g,az_g,"
           "left_encoder,right_encoder,left_motor_rpm,right_motor_rpm,"
           "left_b_rad,left_d_rad,right_b_rad,right_d_rad,"
           "left_b_rad_s,left_d_rad_s,right_b_rad_s,right_d_rad_s,"
           "left_leg_length_m,right_leg_length_m,control_pitch_rad\n";
    for (const auto &r : rows) {
      csv << r.elapsed_us << ',' << unsigned(r.stale_mask) << ',' << r.imu_fresh
          << ',' << r.feedback_valid;
      for (double x : r.values)
        csv << ',' << x;
      csv << '\n';
    }
    csv.close();
    if (!csv)
      throw std::runtime_error("CSV export failed");
    std::cout
        << "termination=" << termination << " samples=" << rows.size()
        << " usb_error=[" << link_error << "] saved=" << argv[1] << '\n'
        << "方向、零点、减速比需结合实际动作核对，本工具不自动判定通过。\n";
    return termination == "duration" || termination == "signal" ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
