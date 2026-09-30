#include "wbr/ethercat_transport.hpp"

#include <array>
#include <cstring>
#include <sstream>

#if defined(WBR_HAVE_SOEM)
extern "C" {
#include <soem/ethercat.h>
}
#endif

namespace wbr::ethercat {

#if defined(WBR_HAVE_SOEM)
namespace {

class SoemTransport final : public EthercatTransport {
 public:
  ~SoemTransport() override {
    if (open_) {
      ec_slave[0].state = EC_STATE_INIT;
      ec_writestate(0);
      ec_close();
    }
  }

  bool Open(const TransportConfig& config, std::string& error) override {
    if (config.interface.empty()) {
      error = "SOEM requires --interface";
      return false;
    }
    if (config.enable_distributed_clocks) {
      error = "SOEM DC mode is not implemented by this benchmark; use FreeRun "
              "for the three-path comparison or IgH for the separate DC run";
      return false;
    }
    if (ec_init(config.interface.c_str()) <= 0) {
      error = "ec_init failed for " + config.interface;
      return false;
    }
    open_ = true;
    if (ec_config_init(FALSE) != 1) {
      error = "SOEM expected exactly one slave";
      return false;
    }
    if (config.vendor_id != 0U && ec_slave[1].eep_man != config.vendor_id) {
      std::ostringstream stream;
      stream << "vendor mismatch: expected 0x" << std::hex << config.vendor_id
             << " got 0x" << ec_slave[1].eep_man;
      error = stream.str();
      return false;
    }
    if (config.product_code != 0U && ec_slave[1].eep_id != config.product_code) {
      std::ostringstream stream;
      stream << "product mismatch: expected 0x" << std::hex
             << config.product_code << " got 0x" << ec_slave[1].eep_id;
      error = stream.str();
      return false;
    }
    ec_config_map(&io_map_);
    if (ec_slave[1].Obytes < sizeof(MasterProcessImage) ||
        ec_slave[1].Ibytes < sizeof(SlaveProcessImage)) {
      std::ostringstream stream;
      stream << "PDO image is too small: outputs=" << ec_slave[1].Obytes
             << " inputs=" << ec_slave[1].Ibytes;
      error = stream.str();
      return false;
    }
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);
    if (ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) !=
        EC_STATE_OPERATIONAL) {
      error = "slave did not reach OP";
      return false;
    }
    status_.link_up = true;
    status_.operational = true;
    status_.expected_working_counter =
        static_cast<std::uint32_t>(ec_group[0].outputsWKC * 2 +
                                   ec_group[0].inputsWKC);
    return true;
  }

  bool Exchange(const MasterProcessImage& output, SlaveProcessImage& input,
                std::uint64_t, std::string& error) override {
    if (!open_) {
      error = "SOEM transport is not open";
      return false;
    }
    std::memcpy(ec_slave[1].outputs, &output, sizeof(output));
    ec_send_processdata();
    const int wkc = ec_receive_processdata(EC_TIMEOUTRET);
    status_.working_counter = wkc > 0 ? static_cast<std::uint32_t>(wkc) : 0U;
    std::memcpy(&input, ec_slave[1].inputs, sizeof(input));
    status_.al_status = ec_slave[1].ALstatuscode;
    status_.operational = ec_slave[1].state == EC_STATE_OPERATIONAL;
    return wkc > 0;
  }

  TransportStatus status() const override { return status_; }
  const char* name() const override { return "soem"; }

 private:
  bool open_ = false;
  std::array<std::uint8_t, 4096> io_map_{};
  TransportStatus status_{};
};

}  // namespace

std::unique_ptr<EthercatTransport> CreateSoemTransport() {
  return std::make_unique<SoemTransport>();
}

#else

std::unique_ptr<EthercatTransport> CreateSoemTransport() { return nullptr; }

#endif

}  // namespace wbr::ethercat
