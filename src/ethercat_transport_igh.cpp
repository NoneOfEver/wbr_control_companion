#include "wbr/ethercat_transport.hpp"

#include <array>
#include <cstring>
#include <sstream>
#include <vector>

#if defined(WBR_HAVE_IGH)
#include <ecrt.h>
#endif

namespace wbr::ethercat {

#if defined(WBR_HAVE_IGH)
namespace {

struct EntryLayout {
  std::uint16_t index;
  std::uint8_t subindex;
  std::uint8_t bits;
  std::size_t image_offset;
};

constexpr std::array<EntryLayout, 16> kOutputLayout{{
    {0x7000, 1, 32, offsetof(MasterProcessImage, benchmark.abi_version)},
    {0x7000, 2, 32, offsetof(MasterProcessImage, benchmark.sequence)},
    {0x7000, 3, 64, offsetof(MasterProcessImage, benchmark.host_send_time_ns)},
    {0x7000, 4, 32, offsetof(MasterProcessImage, benchmark.command)},
    {0x7000, 5, 32, offsetof(MasterProcessImage, benchmark.payload_crc32)},
    {0x7000, 6, 32, offsetof(MasterProcessImage, benchmark.payload0)},
    {0x7000, 7, 32, offsetof(MasterProcessImage, benchmark.payload1)},
    {0x7010, 1, 32, offsetof(MasterProcessImage, mpc.abi_version)},
    {0x7010, 2, 32, offsetof(MasterProcessImage, mpc.sequence)},
    {0x7010, 3, 32, offsetof(MasterProcessImage, mpc.source_state_sequence)},
    {0x7010, 4, 32, offsetof(MasterProcessImage, mpc.mode_and_flags)},
    {0x7010, 5, 64, offsetof(MasterProcessImage, mpc.host_send_time_ns)},
    {0x7010, 6, 32, offsetof(MasterProcessImage, mpc.left_support_force_n)},
    {0x7010, 7, 32, offsetof(MasterProcessImage, mpc.right_support_force_n)},
    {0x7010, 8, 32, offsetof(MasterProcessImage, mpc.valid_for_us)},
    {0x7010, 9, 32, offsetof(MasterProcessImage, mpc.solve_time_us)},
}};

constexpr std::array<EntryLayout, 21> kInputLayout{{
    {0x6000, 1, 32, offsetof(SlaveProcessImage, benchmark.abi_version)},
    {0x6000, 2, 32, offsetof(SlaveProcessImage, benchmark.echoed_sequence)},
    {0x6000, 3, 64, offsetof(SlaveProcessImage, benchmark.slave_receive_ticks)},
    {0x6000, 4, 64, offsetof(SlaveProcessImage, benchmark.slave_transmit_ticks)},
    {0x6000, 5, 32, offsetof(SlaveProcessImage, benchmark.status)},
    {0x6000, 6, 32, offsetof(SlaveProcessImage, benchmark.echoed_payload_crc32)},
    {0x6010, 1, 32, offsetof(SlaveProcessImage, state.abi_version)},
    {0x6010, 2, 32, offsetof(SlaveProcessImage, state.sequence)},
    {0x6010, 3, 64, offsetof(SlaveProcessImage, state.sample_ticks)},
    {0x6010, 4, 32, offsetof(SlaveProcessImage, state.roll_rad)},
    {0x6010, 5, 32, offsetof(SlaveProcessImage, state.roll_rate_rad_s)},
    {0x6010, 6, 32, offsetof(SlaveProcessImage, state.body_height_m)},
    {0x6010, 7, 32, offsetof(SlaveProcessImage, state.body_height_rate_m_s)},
    {0x6010, 8, 32, offsetof(SlaveProcessImage, state.left_leg_length_m)},
    {0x6010, 9, 32, offsetof(SlaveProcessImage, state.left_leg_rate_m_s)},
    {0x6010, 10, 32, offsetof(SlaveProcessImage, state.right_leg_length_m)},
    {0x6010, 11, 32, offsetof(SlaveProcessImage, state.right_leg_rate_m_s)},
    {0x6010, 12, 32, offsetof(SlaveProcessImage, state.left_support_force_n)},
    {0x6010, 13, 32, offsetof(SlaveProcessImage, state.right_support_force_n)},
    {0x6010, 14, 32, offsetof(SlaveProcessImage, state.status_flags)},
    {0x6010, 15, 32, offsetof(SlaveProcessImage, state.reserved)},
}};

ec_pdo_entry_info_t kBenchmarkRxEntries[] = {
    {0x7000, 1, 32}, {0x7000, 2, 32}, {0x7000, 3, 64}, {0x7000, 4, 32},
    {0x7000, 5, 32}, {0x7000, 6, 32}, {0x7000, 7, 32},
};
ec_pdo_entry_info_t kMpcRxEntries[] = {
    {0x7010, 1, 32}, {0x7010, 2, 32}, {0x7010, 3, 32},
    {0x7010, 4, 32}, {0x7010, 5, 64}, {0x7010, 6, 32},
    {0x7010, 7, 32}, {0x7010, 8, 32}, {0x7010, 9, 32},
};
ec_pdo_entry_info_t kBenchmarkTxEntries[] = {
    {0x6000, 1, 32}, {0x6000, 2, 32}, {0x6000, 3, 64},
    {0x6000, 4, 64}, {0x6000, 5, 32}, {0x6000, 6, 32},
};
ec_pdo_entry_info_t kStateTxEntries[] = {
    {0x6010, 1, 32},  {0x6010, 2, 32},  {0x6010, 3, 64},
    {0x6010, 4, 32},  {0x6010, 5, 32},  {0x6010, 6, 32},
    {0x6010, 7, 32},  {0x6010, 8, 32},  {0x6010, 9, 32},
    {0x6010, 10, 32}, {0x6010, 11, 32}, {0x6010, 12, 32},
    {0x6010, 13, 32}, {0x6010, 14, 32}, {0x6010, 15, 32},
};
ec_pdo_info_t kPdos[] = {
    {0x1600, 7, kBenchmarkRxEntries}, {0x1601, 9, kMpcRxEntries},
    {0x1a00, 6, kBenchmarkTxEntries}, {0x1a01, 15, kStateTxEntries},
};
ec_sync_info_t kSyncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, &kPdos[0], EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, &kPdos[2], EC_WD_DISABLE},
    {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT},
};

class IghTransport final : public EthercatTransport {
 public:
  ~IghTransport() override {
    if (master_ != nullptr) ecrt_release_master(master_);
  }

  bool Open(const TransportConfig& config, std::string& error) override {
    config_ = config;
    master_ = ecrt_request_master(0);
    if (master_ == nullptr) return Fail("ecrt_request_master(0) failed", error);
    output_domain_ = ecrt_master_create_domain(master_);
    input_domain_ = ecrt_master_create_domain(master_);
    if (output_domain_ == nullptr || input_domain_ == nullptr)
      return Fail("ecrt_master_create_domain failed", error);
    slave_config_ = ecrt_master_slave_config(master_, config.alias, config.position,
                                             config.vendor_id,
                                             config.product_code);
    if (slave_config_ == nullptr)
      return Fail("ecrt_master_slave_config failed; verify identity", error);
    if (ecrt_slave_config_pdos(slave_config_, EC_END, kSyncs) != 0)
      return Fail("ecrt_slave_config_pdos failed", error);
    if (config.enable_distributed_clocks) {
      ecrt_slave_config_dc(slave_config_, 0x0300, config.cycle_time_ns, 0, 0, 0);
    }
    if (!RegisterDomain(output_domain_, kOutputLayout, output_offsets_, config,
                        error) ||
        !RegisterDomain(input_domain_, kInputLayout, input_offsets_, config,
                        error)) {
      return false;
    }
    if (ecrt_master_activate(master_) != 0)
      return Fail("ecrt_master_activate failed", error);
    output_data_ = ecrt_domain_data(output_domain_);
    input_data_ = ecrt_domain_data(input_domain_);
    if (output_data_ == nullptr || input_data_ == nullptr)
      return Fail("ecrt_domain_data returned null", error);
    return true;
  }

  bool Exchange(const MasterProcessImage& output, SlaveProcessImage& input,
                std::uint64_t app_time_ns, std::string& error) override {
    if (master_ == nullptr) return Fail("IgH transport is not open", error);
    ecrt_master_receive(master_);
    ecrt_domain_process(input_domain_);
    ecrt_domain_process(output_domain_);
    CopyFromDomain(input, input_data_, kInputLayout, input_offsets_);
    CopyToDomain(output_data_, output, kOutputLayout, output_offsets_);
    ecrt_domain_queue(output_domain_);
    ecrt_domain_queue(input_domain_);
    if (config_.enable_distributed_clocks) {
      ecrt_master_application_time(master_, app_time_ns);
      if ((cycle_count_ % 1000U) == 0U)
        ecrt_master_sync_reference_clock(master_);
      ecrt_master_sync_slave_clocks(master_);
    }
    ecrt_master_send(master_);
    ++cycle_count_;
    UpdateStatus();
    return true;
  }

  TransportStatus status() const override { return status_; }
  const char* name() const override { return "igh"; }

 private:
  template <std::size_t N>
  bool RegisterDomain(ec_domain_t* domain,
                      const std::array<EntryLayout, N>& layout,
                      std::array<unsigned int, N>& offsets,
                      const TransportConfig& config, std::string& error) {
    std::vector<ec_pdo_entry_reg_t> registrations(N + 1U);
    for (std::size_t i = 0U; i < N; ++i) {
      registrations[i] = {config.alias,
                          config.position,
                          config.vendor_id,
                          config.product_code,
                          layout[i].index,
                          layout[i].subindex,
                          &offsets[i],
                          nullptr};
    }
    if (ecrt_domain_reg_pdo_entry_list(domain, registrations.data()) != 0)
      return Fail("ecrt_domain_reg_pdo_entry_list failed", error);
    return true;
  }

  template <typename Image, std::size_t N>
  static void CopyToDomain(std::uint8_t* domain, const Image& image,
                           const std::array<EntryLayout, N>& layout,
                           const std::array<unsigned int, N>& offsets) {
    const auto* source = reinterpret_cast<const std::uint8_t*>(&image);
    for (std::size_t i = 0U; i < N; ++i)
      std::memcpy(domain + offsets[i], source + layout[i].image_offset,
                  layout[i].bits / 8U);
  }

  template <typename Image, std::size_t N>
  static void CopyFromDomain(Image& image, const std::uint8_t* domain,
                             const std::array<EntryLayout, N>& layout,
                             const std::array<unsigned int, N>& offsets) {
    auto* destination = reinterpret_cast<std::uint8_t*>(&image);
    for (std::size_t i = 0U; i < N; ++i)
      std::memcpy(destination + layout[i].image_offset, domain + offsets[i],
                  layout[i].bits / 8U);
  }

  bool Fail(const char* message, std::string& error) {
    error = message;
    return false;
  }

  void UpdateStatus() {
    ec_domain_state_t input_state{};
    ec_domain_state_t output_state{};
    ecrt_domain_state(input_domain_, &input_state);
    ecrt_domain_state(output_domain_, &output_state);
    status_.working_counter =
        input_state.working_counter + output_state.working_counter;
    // A logical read increments WKC once; a logical write increments it twice.
    status_.expected_working_counter = 3U;
    ec_master_state_t master_state{};
    ecrt_master_state(master_, &master_state);
    status_.link_up = master_state.link_up != 0U;
    ec_slave_config_state_t slave_state{};
    ecrt_slave_config_state(slave_config_, &slave_state);
    status_.operational = slave_state.operational != 0U;
    status_.al_status = slave_state.al_state;
  }

  TransportConfig config_{};
  ec_master_t* master_ = nullptr;
  ec_domain_t* output_domain_ = nullptr;
  ec_domain_t* input_domain_ = nullptr;
  ec_slave_config_t* slave_config_ = nullptr;
  std::uint8_t* output_data_ = nullptr;
  std::uint8_t* input_data_ = nullptr;
  std::array<unsigned int, kOutputLayout.size()> output_offsets_{};
  std::array<unsigned int, kInputLayout.size()> input_offsets_{};
  std::uint64_t cycle_count_ = 0U;
  TransportStatus status_{};
};

}  // namespace

std::unique_ptr<EthercatTransport> CreateIghTransport() {
  return std::make_unique<IghTransport>();
}

#else

std::unique_ptr<EthercatTransport> CreateIghTransport() { return nullptr; }

#endif

}  // namespace wbr::ethercat
