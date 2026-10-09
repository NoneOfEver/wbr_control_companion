#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "wbr/leg_mpc.hpp"
#include "wbr/mpc_link.hpp"
#include "wbr/realtime_loop.hpp"

namespace {

constexpr std::uint16_t kVid = 0x34b7;
constexpr std::uint16_t kPid = 0xffff;
constexpr unsigned char kEpOut = 0x01;
constexpr unsigned char kEpIn = 0x81;
constexpr auto kReconnectDelay = std::chrono::milliseconds(500);

struct Options {
  std::uint32_t cycles = 0;
  std::uint32_t period_us = 10000;
  std::uint32_t mpc_divisor = 1;
  int cpu = 3;
  int priority = 80;
  std::string csv;
  bool realtime = true;
  bool active = false;
};

struct Record {
  std::uint32_t cycle;
  std::uint32_t session;
  std::uint32_t state_sequence;
  std::uint32_t command_sequence;
  std::uint64_t rtt_ns;
	std::int64_t release_jitter_ns;
	std::uint64_t command_ready_ns;
	std::uint64_t cycle_execution_ns;
	std::int64_t deadline_slack_ns;
  std::uint32_t solve_us;
	std::uint32_t device_uptime_ms;
	std::uint8_t control_state;
	std::uint8_t valid_flags;
	std::uint8_t contact_flags;
	std::uint8_t motor_stale_mask;
	std::uint8_t solver_flags;
	std::uint16_t solver_iterations;
	double solver_dual_residual;
  float roll;
  float roll_rate;
  float height;
  float left_leg_length;
  float right_leg_length;
  float left_leg_rate;
  float right_leg_rate;
  float estimated_left_support_force;
  float estimated_right_support_force;
  float left_force;
  float right_force;
  float target_height;
  double reference_height;
  bool height_target_valid;
  std::uint8_t command_mode;
};

void WriteCsvHeader(std::ostream& csv) {
  csv << "cycle,session,state_sequence,command_sequence,rtt_ns,release_jitter_ns,"
         "command_ready_ns,cycle_execution_ns,deadline_slack_ns,deadline_miss,"
         "solve_us,device_uptime_ms,control_state,valid_flags,contact_flags,"
		 "motor_stale_mask,"
         "solver_flags,solver_iterations,solver_dual_residual,"
         "roll,roll_rate,height,left_leg_length,right_leg_length,left_leg_rate,"
         "right_leg_rate,"
         "estimated_left_support_force,estimated_right_support_force,"
         "commanded_left_force,commanded_right_force,target_height,reference_height,height_target_valid,command_mode\n";
}

void WriteCsvRecord(std::ostream& csv, const Record& row) {
  csv << row.cycle << ',' << row.session << ',' << row.state_sequence << ','
      << row.command_sequence
      << ',' << row.rtt_ns << ',' << row.release_jitter_ns << ','
      << row.command_ready_ns << ',' << row.cycle_execution_ns << ','
      << row.deadline_slack_ns << ',' << (row.deadline_slack_ns < 0 ? 1 : 0)
      << ',' << row.solve_us << ',' << row.device_uptime_ms << ','
      << static_cast<unsigned int>(row.control_state) << ','
      << static_cast<unsigned int>(row.valid_flags) << ','
      << static_cast<unsigned int>(row.contact_flags) << ','
      << static_cast<unsigned int>(row.motor_stale_mask) << ','
      << static_cast<unsigned int>(row.solver_flags) << ','
      << row.solver_iterations << ',' << row.solver_dual_residual << ','
      << row.roll << ',' << row.roll_rate << ',' << row.height << ','
      << row.left_leg_length << ',' << row.right_leg_length << ','
      << row.left_leg_rate << ',' << row.right_leg_rate << ','
      << row.estimated_left_support_force << ','
      << row.estimated_right_support_force << ','
      << row.left_force << ',' << row.right_force << ',' << row.target_height << ','
      << row.reference_height << ',' << row.height_target_valid << ','
      << static_cast<unsigned int>(row.command_mode) << '\n';
}

class CsvWriter {
 public:
  static constexpr std::size_t kCapacity = 4096;

  explicit CsvWriter(const std::string& path) : requested_(!path.empty()) {
    if (path.empty()) return;
    csv_.open(path);
    if (!csv_) return;
    WriteCsvHeader(csv_);
    worker_ = std::thread(&CsvWriter::Run, this);
  }

  ~CsvWriter() { Stop(); }

  bool ready() const { return !requested_ || csv_.is_open(); }

  bool Enqueue(const Record& record) {
    if (!enabled()) return true;
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1U) % kCapacity;
    if (next == tail_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    queue_[head] = record;
    head_.store(next, std::memory_order_release);
    return true;
  }

  void Stop() {
    bool expected = false;
    if (!stopping_.compare_exchange_strong(expected, true)) return;
    if (worker_.joinable()) worker_.join();
    if (csv_) csv_.flush();
  }

  std::uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

 private:
  bool enabled() const { return requested_; }

  void Run() {
    std::uint32_t rows_since_flush = 0U;
    for (;;) {
      std::size_t tail = tail_.load(std::memory_order_relaxed);
      const std::size_t head = head_.load(std::memory_order_acquire);
      if (tail == head) {
        if (stopping_.load(std::memory_order_acquire)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      WriteCsvRecord(csv_, queue_[tail]);
      tail_.store((tail + 1U) % kCapacity, std::memory_order_release);
      if (++rows_since_flush >= 400U) {
        csv_.flush();
        rows_since_flush = 0U;
      }
    }
    csv_.flush();
  }

  std::array<Record, kCapacity> queue_{};
  std::atomic<std::size_t> head_{0U};
  std::atomic<std::size_t> tail_{0U};
  std::atomic<bool> stopping_{false};
  std::atomic<std::uint64_t> dropped_{0U};
  bool requested_ = false;
  std::ofstream csv_;
  std::thread worker_;
};

volatile std::sig_atomic_t g_stop_requested = 0;

void HandleStopSignal(int) {
  g_stop_requested = 1;
}

struct Runner {
  Options options;
  libusb_context* usb = nullptr;
  libusb_device_handle* device = nullptr;
  libusb_transfer* in_transfer = nullptr;
  libusb_transfer* out_transfer = nullptr;
  bool interface_claimed = false;
  std::array<std::uint8_t, wbr::protocol::kStateFrameSize> in{};
  std::array<std::uint8_t, wbr::protocol::kCommandFrameSize> out{};
  wbr::control::LegMpc mpc;
  wbr::control::LegMpc::Input previous_force;
  wbr::control::LegMpc::Reference reference;
	bool reference_height_initialized = false;
  CsvWriter csv_writer;
  Record pending_record{};
  bool pending_record_valid = false;
  std::uint64_t cycle_start_ns = 0;
  std::uint64_t next_release_ns = 0;
	std::uint64_t in_complete_ns = 0;
	std::uint64_t out_complete_ns = 0;
	std::int64_t release_jitter_ns = 0;
  std::uint32_t command_sequence = 0;
	std::uint32_t last_state_sequence = 0;
  std::uint32_t session = 0;
  std::uint32_t reconnect_count = 0;
  std::uint32_t completed = 0;
  std::uint32_t last_solve_us = 0;
	std::uint8_t last_solver_flags = 0U;
	std::uint16_t last_solver_iterations = 0U;
	double last_solver_dual_residual = 0.0;
  bool in_complete = false;
  bool out_complete = false;
  bool in_pending = false;
  bool out_pending = false;
  bool session_stopping = false;
  bool reconnect_required = false;
  bool done = false;
  int result = 0;

  explicit Runner(const Options& input)
      : options(input),
        mpc([&] {
          wbr::control::LegMpc::Parameters p;
          p.sample_time_s = static_cast<double>(input.period_us * input.mpc_divisor) / 1e6;
          return p;
        }()),
        csv_writer(input.csv) {
    const auto& p = mpc.parameters();
    previous_force.setConstant(p.mass_kg * p.gravity_m_per_s2 / 2.0);
    reference.height_m = p.nominal_height_m;
  }
};

void Fail(Runner& runner, const std::string& message) {
  std::cerr << message << '\n';
  runner.result = 1;
  runner.done = true;
}

const char* TransferStatusName(libusb_transfer_status status) {
  switch (status) {
    case LIBUSB_TRANSFER_COMPLETED: return "COMPLETED";
    case LIBUSB_TRANSFER_ERROR: return "ERROR";
    case LIBUSB_TRANSFER_TIMED_OUT: return "TIMED_OUT";
    case LIBUSB_TRANSFER_CANCELLED: return "CANCELLED";
    case LIBUSB_TRANSFER_STALL: return "STALL";
    case LIBUSB_TRANSFER_NO_DEVICE: return "NO_DEVICE";
    case LIBUSB_TRANSFER_OVERFLOW: return "OVERFLOW";
  }
  return "UNKNOWN";
}

void ReportTransferFailure(Runner& runner, const char* endpoint,
                           const libusb_transfer* transfer,
                           std::size_t expected_length) {
  const std::uint64_t now_ns = wbr::realtime::MonotonicTimeNs();
  std::cerr << "USB " << endpoint << " failed"
            << " status=" << TransferStatusName(transfer->status)
            << " status_code=" << static_cast<int>(transfer->status)
            << " actual_length=" << transfer->actual_length
            << " expected_length=" << expected_length
            << " cycle=" << runner.completed
            << " elapsed_us=" << (now_ns - runner.cycle_start_ns) / 1000U
            << " release_lag_us="
            << (static_cast<std::int64_t>(runner.cycle_start_ns) -
                static_cast<std::int64_t>(runner.next_release_ns)) / 1000
            << '\n';
  if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE) {
    runner.reconnect_required = true;
  } else if (!runner.session_stopping) {
    runner.result = 1;
  }
  runner.done = true;
}

void SubmitCycle(Runner& runner);

void FinishCycle(Runner& runner) {
  if (!runner.in_complete || !runner.out_complete || runner.done) return;
	const std::uint64_t completion_ns =
		std::max(runner.in_complete_ns, runner.out_complete_ns);
	if (runner.pending_record_valid) {
		auto& record = runner.pending_record;
		record.cycle_execution_ns = completion_ns - runner.cycle_start_ns;
		const std::uint64_t deadline_ns = runner.next_release_ns +
			static_cast<std::uint64_t>(runner.options.period_us) * 1000U;
		record.deadline_slack_ns = static_cast<std::int64_t>(deadline_ns) -
			static_cast<std::int64_t>(completion_ns);
		(void)runner.csv_writer.Enqueue(record);
		runner.pending_record_valid = false;
	}
  runner.completed++;
  if ((runner.options.cycles != 0U &&
       runner.completed >= runner.options.cycles) ||
      g_stop_requested != 0) {
    runner.done = true;
    return;
  }
  runner.next_release_ns += static_cast<std::uint64_t>(runner.options.period_us) * 1000U;
  std::string error;
  if (!wbr::realtime::SleepUntilNs(runner.next_release_ns, error)) {
    Fail(runner, error);
    return;
  }
  SubmitCycle(runner);
}

void LIBUSB_CALL OutComplete(libusb_transfer* transfer) {
  auto& runner = *static_cast<Runner*>(transfer->user_data);
	runner.out_pending = false;
	runner.out_complete_ns = wbr::realtime::MonotonicTimeNs();
  if (runner.session_stopping &&
      transfer->status == LIBUSB_TRANSFER_CANCELLED) {
    return;
  }
  if (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
      transfer->actual_length != static_cast<int>(runner.out.size())) {
	ReportTransferFailure(runner, "command OUT", transfer, runner.out.size());
    return;
  }
  runner.out_complete = true;
  FinishCycle(runner);
}

void LIBUSB_CALL InComplete(libusb_transfer* transfer) {
  auto& runner = *static_cast<Runner*>(transfer->user_data);
  runner.in_pending = false;
  const std::uint64_t received_ns = wbr::realtime::MonotonicTimeNs();
  if (runner.session_stopping &&
      transfer->status == LIBUSB_TRANSFER_CANCELLED) {
    return;
  }
  if (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
      transfer->actual_length != static_cast<int>(runner.in.size())) {
	ReportTransferFailure(runner, "state IN", transfer, runner.in.size());
    return;
  }
  wbr::protocol::StateFrame state;
  if (wbr::protocol::DecodeState(runner.in.data(), runner.in.size(), state) !=
      wbr::protocol::DecodeError::kNone) {
    Fail(runner, "invalid MPC Link state frame");
    return;
  }

  wbr::protocol::CommandFrame command;
  command.sequence = ++runner.command_sequence;
  command.sender_uptime_ms = static_cast<std::uint32_t>(received_ns / 1000000U);
  command.source_state_sequence = state.sequence;
	runner.last_state_sequence = state.sequence;
  command.mode = runner.options.active ? wbr::protocol::CommandMode::kActive
												: wbr::protocol::CommandMode::kShadow;
  command.contact_mode = wbr::protocol::ContactMode::kBoth;
  command.valid_for_ms = 30;
  command.left_support_force_n = static_cast<float>(runner.previous_force(0));
  command.right_support_force_n = static_cast<float>(runner.previous_force(1));
	const bool motor_feedback_valid =
		(state.valid_flags & wbr::protocol::kStateMotorFeedbackValid) != 0U;
	const bool height_target_valid = motor_feedback_valid &&
		std::isfinite(state.target_body_height_m) &&
		state.target_body_height_m >= 0.15F && state.target_body_height_m <= 0.33F;
	if (height_target_valid) {
		runner.reference.height_m = state.target_body_height_m;
		runner.reference_height_initialized = true;
	} else if (!runner.reference_height_initialized && motor_feedback_valid &&
		std::isfinite(state.body_height_m) && state.body_height_m > 0.0F) {
		// Until a valid MCU target arrives, hold the startup measured height.
		runner.reference.height_m = state.body_height_m;
		runner.reference_height_initialized = true;
	}

  if ((runner.completed % runner.options.mpc_divisor) == 0U) {
    wbr::control::LegMpc::State measured;
    measured << state.roll_rad, state.roll_rate_rad_s, state.body_height_m,
        state.body_height_rate_m_s;
    const std::uint64_t solve_begin = wbr::realtime::MonotonicTimeNs();
    const auto solution = runner.mpc.Solve(measured, runner.reference,
                                           runner.previous_force);
    runner.last_solve_us = static_cast<std::uint32_t>(
        (wbr::realtime::MonotonicTimeNs() - solve_begin) / 1000U);
    const bool finite = std::isfinite(solution.left_force_n) &&
                        std::isfinite(solution.right_force_n);
    runner.last_solver_flags = 0U;
    if (solution.converged) {
		runner.last_solver_flags |= wbr::protocol::kSolverConverged;
	}
    if (solution.used_fallback) runner.last_solver_flags |= wbr::protocol::kSolverUsedFallback;
    if (finite) runner.last_solver_flags |= wbr::protocol::kSolverInputFinite;
    runner.last_solver_iterations = static_cast<std::uint16_t>(solution.iterations);
	runner.last_solver_dual_residual = solution.gradient_inf_norm;
    if (finite) {
      runner.previous_force << solution.left_force_n, solution.right_force_n;
      command.left_support_force_n = static_cast<float>(solution.left_force_n);
      command.right_support_force_n = static_cast<float>(solution.right_force_n);
    } else {
      command.mode = wbr::protocol::CommandMode::kDisabled;
    }
  }
  command.solver_flags = runner.last_solver_flags;
  command.solver_iterations = runner.last_solver_iterations;
  command.solve_time_us = runner.last_solve_us;
  runner.out = wbr::protocol::EncodeCommand(command);
	const std::uint64_t command_ready_at_ns = wbr::realtime::MonotonicTimeNs();
	runner.pending_record = {runner.completed, runner.session, state.sequence,
                            command.sequence,
                            received_ns - runner.cycle_start_ns,
							runner.release_jitter_ns,
							command_ready_at_ns - runner.cycle_start_ns,
							0U, 0,
							runner.last_solve_us, state.sender_uptime_ms,
							state.control_state, state.valid_flags, state.contact_flags,
							state.motor_stale_mask,
							command.solver_flags, command.solver_iterations,
							runner.last_solver_dual_residual,
							state.roll_rad,
							state.roll_rate_rad_s,
							state.body_height_m,
							state.left_leg_length_m, state.right_leg_length_m,
							state.left_leg_rate_m_s, state.right_leg_rate_m_s,
							state.left_support_force_n, state.right_support_force_n,
							command.left_support_force_n,
                            command.right_support_force_n, state.target_body_height_m,
                            runner.reference.height_m, height_target_valid,
                            static_cast<std::uint8_t>(command.mode)};
	runner.pending_record_valid = true;
  runner.in_complete = true;
	runner.in_complete_ns = command_ready_at_ns;
  FinishCycle(runner);
}

void SubmitCycle(Runner& runner) {
  runner.in_complete = false;
  runner.out_complete = false;
  runner.cycle_start_ns = wbr::realtime::MonotonicTimeNs();
	runner.release_jitter_ns = static_cast<std::int64_t>(runner.cycle_start_ns) -
		static_cast<std::int64_t>(runner.next_release_ns);
  libusb_fill_interrupt_transfer(runner.in_transfer, runner.device, kEpIn,
                                 runner.in.data(), runner.in.size(), InComplete,
                                 &runner, 1000);
  int rc = libusb_submit_transfer(runner.in_transfer);
  if (rc != LIBUSB_SUCCESS) {
    if (rc == LIBUSB_ERROR_NO_DEVICE) {
      std::cerr << "submit IN: LIBUSB_ERROR_NO_DEVICE; reconnecting\n";
      runner.reconnect_required = true;
      runner.done = true;
    } else {
      Fail(runner, std::string("submit IN: ") + libusb_error_name(rc));
    }
    return;
  }
  runner.in_pending = true;
  libusb_fill_interrupt_transfer(runner.out_transfer, runner.device, kEpOut,
                                 runner.out.data(), runner.out.size(), OutComplete,
                                 &runner, 1000);
  rc = libusb_submit_transfer(runner.out_transfer);
  if (rc != LIBUSB_SUCCESS) {
    libusb_cancel_transfer(runner.in_transfer);
    if (rc == LIBUSB_ERROR_NO_DEVICE) {
      std::cerr << "submit OUT: LIBUSB_ERROR_NO_DEVICE; reconnecting\n";
      runner.reconnect_required = true;
      runner.done = true;
    } else {
      Fail(runner, std::string("submit OUT: ") + libusb_error_name(rc));
    }
    return;
  }
  runner.out_pending = true;
}

bool Parse(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--no-rt") { options.realtime = false; continue; }
    if (key == "--active") { options.active = true; continue; }
    if (i + 1 >= argc) return false;
    const std::string value = argv[++i];
    if (key == "--cycles") options.cycles = std::stoul(value);
    else if (key == "--period-us") options.period_us = std::stoul(value);
    else if (key == "--mpc-divisor") options.mpc_divisor = std::stoul(value);
    else if (key == "--cpu") options.cpu = std::stoi(value);
    else if (key == "--priority") options.priority = std::stoi(value);
    else if (key == "--csv") options.csv = value;
    else return false;
  }
  return options.period_us > 0 && options.mpc_divisor > 0;
}

bool OpenUsbSession(Runner& runner) {
  bool waiting_message_printed = false;
  while (g_stop_requested == 0) {
    runner.device = libusb_open_device_with_vid_pid(runner.usb, kVid, kPid);
    if (runner.device == nullptr) {
      if (!waiting_message_printed) {
        std::cerr << "USB device unavailable; waiting for 34b7:ffff\n";
        waiting_message_printed = true;
      }
      std::this_thread::sleep_for(kReconnectDelay);
      continue;
    }

    int configuration = 0;
    int rc = libusb_get_configuration(runner.device, &configuration);
    if (rc == LIBUSB_SUCCESS && configuration != 1) {
      rc = libusb_set_configuration(runner.device, 1);
    }
    if (rc == LIBUSB_SUCCESS) {
      rc = libusb_claim_interface(runner.device, 0);
    }
    if (rc != LIBUSB_SUCCESS) {
      libusb_close(runner.device);
      runner.device = nullptr;
      if (rc == LIBUSB_ERROR_NO_DEVICE) {
        std::this_thread::sleep_for(kReconnectDelay);
        continue;
      }
      std::cerr << "USB session setup failed: " << libusb_error_name(rc) << '\n';
      runner.result = 1;
      return false;
    }
    runner.interface_claimed = true;
    runner.in_transfer = libusb_alloc_transfer(0);
    runner.out_transfer = libusb_alloc_transfer(0);
    if (runner.in_transfer == nullptr || runner.out_transfer == nullptr) {
      std::cerr << "USB transfer allocation failed\n";
      runner.result = 1;
      return false;
    }

    ++runner.session;
    runner.done = false;
    runner.reconnect_required = false;
    runner.session_stopping = false;
    runner.in_pending = false;
    runner.out_pending = false;
    runner.in_complete = false;
    runner.out_complete = false;
    runner.pending_record_valid = false;
    runner.reference_height_initialized = false;
    runner.last_state_sequence = 0U;
    runner.last_solve_us = 0U;
    runner.last_solver_flags = 0U;
    runner.last_solver_iterations = 0U;
    runner.last_solver_dual_residual = 0.0;
    const auto& parameters = runner.mpc.parameters();
    runner.previous_force.setConstant(
        parameters.mass_kg * parameters.gravity_m_per_s2 / 2.0);
    runner.reference.height_m = parameters.nominal_height_m;
    std::cout << "USB MPC session=" << runner.session << " connected\n";
    return true;
  }
  return false;
}

void ReleaseUsbSession(Runner& runner) {
  runner.session_stopping = true;
  if (runner.in_pending) {
    const int rc = libusb_cancel_transfer(runner.in_transfer);
    if (rc == LIBUSB_ERROR_NOT_FOUND) runner.in_pending = false;
  }
  if (runner.out_pending) {
    const int rc = libusb_cancel_transfer(runner.out_transfer);
    if (rc == LIBUSB_ERROR_NOT_FOUND) runner.out_pending = false;
  }
  while (runner.in_pending || runner.out_pending) {
    timeval timeout{0, 100000};
    const int rc = libusb_handle_events_timeout(runner.usb, &timeout);
    if (rc != LIBUSB_SUCCESS && rc != LIBUSB_ERROR_INTERRUPTED) {
      std::cerr << "warning: draining USB transfers failed: "
                << libusb_error_name(rc) << '\n';
      break;
    }
  }
  if (runner.in_transfer != nullptr) {
    libusb_free_transfer(runner.in_transfer);
    runner.in_transfer = nullptr;
  }
  if (runner.out_transfer != nullptr) {
    libusb_free_transfer(runner.out_transfer);
    runner.out_transfer = nullptr;
  }
  if (runner.interface_claimed && runner.device != nullptr) {
    (void)libusb_release_interface(runner.device, 0);
  }
  runner.interface_claimed = false;
  if (runner.device != nullptr) {
    libusb_close(runner.device);
    runner.device = nullptr;
  }
}

void SendDisabledBeforeExit(Runner& runner) {
  if (!runner.device) return;
  wbr::protocol::CommandFrame disabled;
  disabled.sequence = ++runner.command_sequence;
  disabled.sender_uptime_ms = static_cast<std::uint32_t>(
      wbr::realtime::MonotonicTimeNs() / 1000000U);
  disabled.source_state_sequence = runner.last_state_sequence;
  disabled.mode = wbr::protocol::CommandMode::kDisabled;
  runner.out = wbr::protocol::EncodeCommand(disabled);
  int actual_length = 0;
  const int out_rc = libusb_interrupt_transfer(
      runner.device, kEpOut, runner.out.data(), runner.out.size(),
      &actual_length, 100);
  if (out_rc != LIBUSB_SUCCESS ||
      actual_length != static_cast<int>(runner.out.size())) {
    std::cerr << "warning: final Disabled OUT failed rc="
              << libusb_error_name(out_rc) << " actual_length=" << actual_length
              << "; device watchdog remains responsible for fallback\n";
    return;
  }
  actual_length = 0;
  (void)libusb_interrupt_transfer(
      runner.device, kEpIn, runner.in.data(), runner.in.size(),
      &actual_length, 100);
  std::cout << "final Disabled command sent\n";
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!Parse(argc, argv, options)) {
    std::cerr << "Usage: usb_mpc [--cycles N] [--period-us N] "
                 "[--mpc-divisor N] [--cpu N] [--priority N] [--csv PATH] "
				 "[--no-rt] [--active]\n";
    return 2;
  }
  std::signal(SIGINT, HandleStopSignal);
  std::signal(SIGTERM, HandleStopSignal);
  Runner runner(options);
	if (!runner.csv_writer.ready()) {
		std::cerr << "Failed to open CSV output: " << options.csv << '\n';
		return 1;
	}
	std::cout << "USB MPC mode=" << (options.active ? "ACTIVE" : "SHADOW")
			  << " period_us=" << options.period_us
			  << " mpc_divisor=" << options.mpc_divisor << '\n';
	const auto& mpc_parameters = runner.mpc.parameters();
	std::cout << "MPC model mass_kg=" << mpc_parameters.mass_kg
			  << " half_track_m=" << mpc_parameters.half_track_m
			  << " roll_inertia_kg_m2=" << mpc_parameters.roll_inertia_kg_m2
			  << " hover_force_per_leg_n="
			  << 0.5 * mpc_parameters.mass_kg * mpc_parameters.gravity_m_per_s2
			  << '\n';
	std::cout << "NOTE: roll inertia uses the measured chassis value; MPC Q/R and "
			     "vertical damping still require experimental tuning.\n";
  std::string error;
  if (options.realtime &&
      !wbr::realtime::ConfigureCurrentThread(options.cpu, options.priority, error)) {
    std::cerr << "Realtime setup failed: " << error << '\n';
    return 1;
  }
  if (libusb_init(&runner.usb) != LIBUSB_SUCCESS) return 1;
  while (g_stop_requested == 0 && runner.result == 0 &&
         (runner.options.cycles == 0U ||
          runner.completed < runner.options.cycles)) {
    if (!OpenUsbSession(runner)) {
      ReleaseUsbSession(runner);
      break;
    }
    wbr::protocol::CommandFrame initial;
    initial.sequence = ++runner.command_sequence;
    initial.mode = wbr::protocol::CommandMode::kDisabled;
    runner.out = wbr::protocol::EncodeCommand(initial);
    runner.next_release_ns = wbr::realtime::MonotonicTimeNs();
    SubmitCycle(runner);
    while (!runner.done) {
      timeval timeout{0, 100000};
      const int rc = libusb_handle_events_timeout(runner.usb, &timeout);
      if (rc == LIBUSB_ERROR_NO_DEVICE) {
        runner.reconnect_required = true;
        runner.done = true;
      } else if (rc != LIBUSB_SUCCESS && rc != LIBUSB_ERROR_INTERRUPTED) {
        Fail(runner, std::string("USB event loop: ") + libusb_error_name(rc));
      }
    }
    if (!runner.reconnect_required && !runner.in_pending &&
        !runner.out_pending) {
      SendDisabledBeforeExit(runner);
    } else if (!runner.reconnect_required) {
      std::cerr << "warning: final Disabled command skipped while transfers "
                   "remain pending; device watchdog remains responsible for "
                   "fallback\n";
    }
    const bool reconnect = runner.reconnect_required &&
                           g_stop_requested == 0 && runner.result == 0;
    ReleaseUsbSession(runner);
    if (!reconnect) break;
    ++runner.reconnect_count;
    std::cerr << "USB disconnected; local watchdog fallback active; "
                 "waiting to reconnect\n";
    std::this_thread::sleep_for(kReconnectDelay);
  }
  libusb_exit(runner.usb);
  runner.csv_writer.Stop();
	std::cout << "USB MPC cycles=" << runner.completed
			<< " reconnects=" << runner.reconnect_count
			<< " csv_dropped=" << runner.csv_writer.dropped();
	if (!options.csv.empty()) std::cout << " output=" << options.csv;
	std::cout << (g_stop_requested != 0 ? " stopped_by_signal" : "") << '\n';
  return runner.result;
}
