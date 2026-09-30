#include "wbr/ethercat_protocol.hpp"

#include <array>

namespace wbr::ethercat {

std::uint32_t PayloadCrc32(std::uint32_t sequence, std::uint32_t payload0,
                           std::uint32_t payload1) {
  std::array<std::uint32_t, 3> words{sequence, payload0, payload1};
  std::uint32_t crc = 0xffffffffU;
  for (const std::uint32_t word : words) {
    for (unsigned int byte = 0; byte < 4U; ++byte) {
      crc ^= (word >> (byte * 8U)) & 0xffU;
      for (unsigned int bit = 0; bit < 8U; ++bit) {
        crc = (crc >> 1U) ^ (0xedb88320U & (0U - (crc & 1U)));
      }
    }
  }
  return ~crc;
}

}  // namespace wbr::ethercat
