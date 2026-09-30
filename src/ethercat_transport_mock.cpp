#include "wbr/ethercat_transport.hpp"

#include <chrono>

namespace wbr::ethercat {
namespace {

class MockTransport final : public EthercatTransport {
 public:
  bool Open(const TransportConfig&, std::string&) override {
    status_.link_up = true;
    status_.operational = true;
    status_.expected_working_counter = 3U;
    return true;
  }

  bool Exchange(const MasterProcessImage& output, SlaveProcessImage& input,
                std::uint64_t app_time_ns, std::string&) override {
    input.benchmark.abi_version = kAbiVersion;
    input.benchmark.echoed_sequence = output.benchmark.sequence;
    input.benchmark.slave_receive_ticks = app_time_ns;
    input.benchmark.slave_transmit_ticks = app_time_ns + 1'000U;
    input.benchmark.echoed_payload_crc32 = output.benchmark.payload_crc32;
    input.benchmark.status = kBenchmarkAbiValid | kBenchmarkOperational;
    if (output.benchmark.payload_crc32 ==
        PayloadCrc32(output.benchmark.sequence, output.benchmark.payload0,
                     output.benchmark.payload1)) {
      input.benchmark.status |= kBenchmarkPayloadValid;
    }
    input.state.abi_version = kAbiVersion;
    input.state.sequence = ++state_sequence_;
    input.state.sample_ticks = app_time_ns / 1000U;
    input.state.body_height_m = 0.10F;
    input.state.left_support_force_n = 127.5F;
    input.state.right_support_force_n = 127.5F;
    input.state.status_flags = kStateImuFresh | kStateSupportForceValid |
                               kStateMotorFeedbackValid | kStateControlEnabled |
                               kStateContactLeft | kStateContactRight |
                               kStateMpcOperatorEnabled;
    if ((output.mpc.mode_and_flags & 0xffU) == kMpcActive) {
      input.state.status_flags |= kStateMpcActiveApplied;
    }
    status_.working_counter = status_.expected_working_counter;
    return true;
  }

  TransportStatus status() const override { return status_; }
  const char* name() const override { return "mock"; }

 private:
  std::uint32_t state_sequence_ = 0U;
  TransportStatus status_{};
};

}  // namespace

std::unique_ptr<EthercatTransport> CreateMockTransport() {
  return std::make_unique<MockTransport>();
}

}  // namespace wbr::ethercat
