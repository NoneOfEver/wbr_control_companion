#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "wbr/ethercat_mpc_safety.hpp"
#include "wbr/ethercat_protocol.hpp"
#include "wbr/ethercat_transport.hpp"
#include "wbr/leg_mpc.hpp"
#include "wbr/realtime_loop.hpp"

namespace {

struct Options {
  std::string backend = "igh";
  std::string interface;
  std::string csv_path = "ethercat-mpc.csv";
  std::uint32_t vendor_id = 0U;
  std::uint32_t product_code = 1U;
  std::uint32_t period_us = 1000U;
  std::uint32_t mpc_divisor = 10U;
  std::uint32_t duration_s = 60U;
  std::uint32_t mode = wbr::ethercat::kMpcShadow;
  int cpu = 2;
  int priority = 80;
  double target_roll_rad = 0.0;
  double target_height_m = 0.10;
  bool allow_active = false;
  bool distributed_clocks = false;
  bool realtime = true;
};

struct MpcRecord {
  std::uint64_t cycle;
  std::int64_t wakeup_jitter_ns;
  std::uint32_t working_counter;
  std::uint32_t state_sequence;
  std::uint32_t state_flags;
  float roll_rad;
  float roll_rate_rad_s;
  float height_m;
  float left_force_n;
  float right_force_n;
  std::uint32_t solve_time_us;
  std::uint32_t command_mode;
  std::uint32_t command_sequence;
};

bool ParseUnsigned(std::string_view text, std::uint32_t& output) {
  int base = 10;
  if (text.size() > 2U && text[0] == '0' &&
      (text[1] == 'x' || text[1] == 'X')) {
    text.remove_prefix(2U);
    base = 16;
  }
  const auto result =
      std::from_chars(text.data(), text.data() + text.size(), output, base);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

void Usage(const char* program) {
  std::cerr
      << "Usage: " << program
      << " [--backend igh|soem|mock] [--interface IFACE] --vendor ID"
         " --product ID [--mode disabled|shadow|active] [--allow-active]"
         " [--period-us N] [--mpc-divisor N] [--duration-s N]"
         " [--target-roll RAD] [--target-height M] [--cpu N]"
         " [--priority N] [--dc] [--csv PATH] [--no-rt]\n";
}

bool ParseOptions(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--allow-active") {
      options.allow_active = true;
      continue;
    }
    if (argument == "--dc") {
      options.distributed_clocks = true;
      continue;
    }
    if (argument == "--no-rt") {
      options.realtime = false;
      continue;
    }
    if (argument == "--help") {
      Usage(argv[0]);
      std::exit(0);
    }
    if (i + 1 >= argc) return false;
    const std::string value = argv[++i];
    std::uint32_t number = 0U;
    if (argument == "--backend") options.backend = value;
    else if (argument == "--interface") options.interface = value;
    else if (argument == "--csv") options.csv_path = value;
    else if (argument == "--target-roll") options.target_roll_rad = std::stod(value);
    else if (argument == "--target-height") options.target_height_m = std::stod(value);
    else if (argument == "--mode") {
      if (value == "disabled") options.mode = wbr::ethercat::kMpcDisabled;
      else if (value == "shadow") options.mode = wbr::ethercat::kMpcShadow;
      else if (value == "active") options.mode = wbr::ethercat::kMpcActive;
      else return false;
    } else if (!ParseUnsigned(value, number)) return false;
    else if (argument == "--vendor") options.vendor_id = number;
    else if (argument == "--product") options.product_code = number;
    else if (argument == "--period-us") options.period_us = number;
    else if (argument == "--mpc-divisor") options.mpc_divisor = number;
    else if (argument == "--duration-s") options.duration_s = number;
    else if (argument == "--cpu") options.cpu = static_cast<int>(number);
    else if (argument == "--priority") options.priority = static_cast<int>(number);
    else return false;
  }
  if (options.mode == wbr::ethercat::kMpcActive && !options.allow_active) {
    std::cerr << "Active mode requires --allow-active\n";
    return false;
  }
  return options.period_us > 0U && options.mpc_divisor > 0U &&
         options.duration_s > 0U;
}

std::unique_ptr<wbr::ethercat::EthercatTransport> MakeTransport(
    const std::string& backend) {
  if (backend == "mock") return wbr::ethercat::CreateMockTransport();
  if (backend == "igh") return wbr::ethercat::CreateIghTransport();
  if (backend == "soem") return wbr::ethercat::CreateSoemTransport();
  return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseOptions(argc, argv, options)) {
    Usage(argv[0]);
    return 2;
  }
  auto transport = MakeTransport(options.backend);
  if (!transport) {
    std::cerr << "Backend is unavailable: " << options.backend << "\n";
    return 2;
  }
  wbr::ethercat::TransportConfig transport_config;
  transport_config.interface = options.interface;
  transport_config.vendor_id = options.vendor_id;
  transport_config.product_code = options.product_code;
  transport_config.cycle_time_ns = options.period_us * 1000U;
  transport_config.enable_distributed_clocks = options.distributed_clocks;
  std::string error;
  if (!transport->Open(transport_config, error)) {
    std::cerr << "Failed to open EtherCAT transport: " << error << "\n";
    return 1;
  }

  wbr::control::LegMpc::Parameters mpc_parameters;
  mpc_parameters.sample_time_s =
      static_cast<double>(options.period_us * options.mpc_divisor) / 1.0e6;
  wbr::control::LegMpc mpc(mpc_parameters);
  wbr::control::LegMpc::Input previous_force;
  previous_force << mpc_parameters.mass_kg * mpc_parameters.gravity_m_per_s2 / 2.0,
      mpc_parameters.mass_kg * mpc_parameters.gravity_m_per_s2 / 2.0;
  wbr::control::LegMpc::Reference reference;
  reference.roll_rad = options.target_roll_rad;
  reference.height_m = options.target_height_m;

  const std::uint64_t total_cycles =
      static_cast<std::uint64_t>(options.duration_s) * 1'000'000U /
      options.period_us;
  std::vector<MpcRecord> records;
  records.reserve(static_cast<std::size_t>(total_cycles));
  if (options.realtime &&
      !wbr::realtime::ConfigureCurrentThread(options.cpu, options.priority,
                                             error)) {
    std::cerr << "Realtime setup failed: " << error << "\n";
    return 1;
  }
  wbr::ethercat::MasterProcessImage output{};
  wbr::ethercat::SlaveProcessImage input{};
  output.benchmark.command = wbr::ethercat::kBenchmarkEcho;
  std::uint32_t last_state_sequence = 0U;
  std::uint32_t command_sequence = 0U;
  std::uint64_t last_fresh_state_ns = 0U;
  const std::uint64_t period_ns = transport_config.cycle_time_ns;
  std::uint64_t deadline = wbr::realtime::MonotonicTimeNs() + 10U * period_ns;

  for (std::uint64_t cycle = 0U; cycle < total_cycles; ++cycle) {
    if (!wbr::realtime::SleepUntilNs(deadline, error)) {
      std::cerr << error << "\n";
      return 1;
    }
    const std::uint64_t wake_time = wbr::realtime::MonotonicTimeNs();
    output.benchmark.sequence = static_cast<std::uint32_t>(cycle + 1U);
    output.benchmark.host_send_time_ns = wake_time;
    output.benchmark.payload0 = output.benchmark.sequence ^ 0xa5a55a5aU;
    output.benchmark.payload1 = output.benchmark.sequence * 2654435761U;
    output.benchmark.payload_crc32 = wbr::ethercat::PayloadCrc32(
        output.benchmark.sequence, output.benchmark.payload0,
        output.benchmark.payload1);
    if (!transport->Exchange(output, input, deadline, error)) {
      std::cerr << "Exchange failed: " << error << "\n";
      return 1;
    }

    std::uint32_t solve_time_us = 0U;
    const bool abi_valid = input.state.abi_version == wbr::ethercat::kAbiVersion;
    const bool new_state = abi_valid && input.state.sequence != last_state_sequence;
    if (new_state) last_fresh_state_ns = wake_time;
    const bool imu_valid =
        (input.state.status_flags & wbr::ethercat::kStateImuFresh) != 0U;
    const std::uint32_t active_required = wbr::ethercat::kStateImuFresh |
                                          wbr::ethercat::kStateMotorFeedbackValid |
                                          wbr::ethercat::kStateControlEnabled |
                                          wbr::ethercat::kStateMpcOperatorEnabled;
    const bool active_safe =
        (input.state.status_flags & active_required) == active_required;
    const bool mode_safe = options.mode != wbr::ethercat::kMpcActive || active_safe;
    const bool run_mpc = new_state && imu_valid && mode_safe &&
                         (cycle % options.mpc_divisor) == 0U;
    if (run_mpc) {
      last_state_sequence = input.state.sequence;
      wbr::control::LegMpc::State measured;
      measured << input.state.roll_rad, input.state.roll_rate_rad_s,
          input.state.body_height_m, input.state.body_height_rate_m_s;
      const std::uint64_t solve_begin = wbr::realtime::MonotonicTimeNs();
      const auto result = mpc.Solve(measured, reference, previous_force);
      solve_time_us = static_cast<std::uint32_t>(
          (wbr::realtime::MonotonicTimeNs() - solve_begin) / 1000U);
      const bool finite = std::isfinite(result.left_force_n) &&
                          std::isfinite(result.right_force_n);
      output.mpc.sequence = ++command_sequence;
      output.mpc.source_state_sequence = input.state.sequence;
      output.mpc.host_send_time_ns = wake_time;
      output.mpc.left_support_force_n = static_cast<float>(result.left_force_n);
      output.mpc.right_support_force_n = static_cast<float>(result.right_force_n);
      output.mpc.valid_for_us = 30000U;
      output.mpc.solve_time_us = solve_time_us;
      output.mpc.mode_and_flags = options.mode;
      if (result.converged) output.mpc.mode_and_flags |= wbr::ethercat::kMpcConverged;
      if (result.used_fallback) output.mpc.mode_and_flags |= wbr::ethercat::kMpcUsedFallback;
      if (finite) output.mpc.mode_and_flags |= wbr::ethercat::kMpcInputFinite;
      if (!result.converged || !finite) {
        wbr::ethercat::DisableMpcCommand(
            output.mpc, command_sequence, input.state.sequence, wake_time);
      } else {
        previous_force << result.left_force_n, result.right_force_n;
      }
    }
    if (last_fresh_state_ns == 0U || wake_time - last_fresh_state_ns > 30'000'000U) {
      wbr::ethercat::DisableMpcCommand(
          output.mpc, command_sequence, input.state.sequence, wake_time);
    }
    if (options.mode == wbr::ethercat::kMpcActive && !active_safe) {
      wbr::ethercat::DisableMpcCommand(
          output.mpc, command_sequence, input.state.sequence, wake_time);
    }

    const auto transport_status = transport->status();
    const bool wkc_valid = transport_status.expected_working_counter == 0U ||
                           transport_status.working_counter ==
                               transport_status.expected_working_counter;
    if (options.mode == wbr::ethercat::kMpcActive &&
        (!wkc_valid || !transport_status.link_up ||
         !transport_status.operational)) {
      wbr::ethercat::DisableMpcCommand(
          output.mpc, command_sequence, input.state.sequence, wake_time);
    }
    records.push_back(
        {cycle,
         static_cast<std::int64_t>(wake_time - deadline),
         transport_status.working_counter,
         input.state.sequence,
         input.state.status_flags,
         input.state.roll_rad,
         input.state.roll_rate_rad_s,
         input.state.body_height_m,
         output.mpc.left_support_force_n,
         output.mpc.right_support_force_n,
         solve_time_us,
         output.mpc.mode_and_flags & 0xffU,
         output.mpc.sequence});
    deadline += period_ns;
  }
  wbr::ethercat::DisableMpcCommand(
      output.mpc, command_sequence, input.state.sequence,
      wbr::realtime::MonotonicTimeNs());
  (void)transport->Exchange(output, input, deadline, error);
  std::ofstream csv(options.csv_path);
  csv << "cycle,wakeup_jitter_ns,wkc,state_sequence,state_flags,roll_rad,"
         "roll_rate_rad_s,height_m,left_force_n,right_force_n,solve_time_us,"
         "command_mode,command_sequence\n";
  for (const auto& record : records) {
    csv << record.cycle << ',' << record.wakeup_jitter_ns << ','
        << record.working_counter << ',' << record.state_sequence << ','
        << record.state_flags << ',' << record.roll_rad << ','
        << record.roll_rate_rad_s << ',' << record.height_m << ','
        << record.left_force_n << ',' << record.right_force_n << ','
        << record.solve_time_us << ',' << record.command_mode << ','
        << record.command_sequence << '\n';
  }
  return 0;
}
