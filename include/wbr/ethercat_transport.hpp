#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "wbr/ethercat_protocol.hpp"

namespace wbr::ethercat {

struct TransportConfig {
  std::string interface;
  std::uint16_t alias = 0U;
  std::uint16_t position = 0U;
  std::uint32_t vendor_id = 0U;
  std::uint32_t product_code = 1U;
  std::uint32_t cycle_time_ns = 1'000'000U;
  bool enable_distributed_clocks = false;
};

struct TransportStatus {
  bool link_up = false;
  bool operational = false;
  std::uint32_t working_counter = 0U;
  std::uint32_t expected_working_counter = 0U;
  std::uint16_t al_status = 0U;
};

class EthercatTransport {
 public:
  virtual ~EthercatTransport() = default;
  virtual bool Open(const TransportConfig& config, std::string& error) = 0;
  virtual bool Exchange(const MasterProcessImage& output,
                        SlaveProcessImage& input, std::uint64_t app_time_ns,
                        std::string& error) = 0;
  virtual TransportStatus status() const = 0;
  virtual const char* name() const = 0;
};

std::unique_ptr<EthercatTransport> CreateMockTransport();
std::unique_ptr<EthercatTransport> CreateIghTransport();
std::unique_ptr<EthercatTransport> CreateSoemTransport();

}  // namespace wbr::ethercat
