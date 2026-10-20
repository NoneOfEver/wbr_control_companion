/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "sensor_store.hpp"
#include <array>
#include <wbr_bridge/bridge_protocol.hpp>
namespace wbr::host {
// USB event thread is the sole owner. No parsing or shared mutable channels in
// the control thread. UART parsers span chunks and reset on stream gaps.
class SensorDecoder {
public:
  explicit SensorDecoder(SensorStore &store, bool strict_hi91_crc = false)
      : store_(store), strict_crc_(strict_hi91_crc) {}
  bool Consume(const bridge::Packet &, uint64_t host_receive_us);
  void Reset();

private:
  struct Stream {
    std::array<uint8_t, 512> bytes{};
    size_t size = 0;
    uint32_t sequence = 0;
    bool has_sequence = false;
  };
  void Feed(unsigned port, const uint8_t *, size_t, uint64_t);
  void ParseRemote(Stream &, uint64_t);
  void ParseHi91(Stream &, uint64_t);
  void ConsumeBytes(Stream &, size_t);
  SensorStore &store_;
  Snapshot latest_{};
  std::array<Stream, 3> streams_{};
  uint32_t remote_sequence_ = 0, imu_sequence_ = 0, hi91_sensor_time_ = 0;
  bool has_hi91_sensor_time_ = false;
  bool strict_crc_;
};
} // namespace wbr::host
