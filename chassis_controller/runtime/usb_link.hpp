/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "sensor_decoder.hpp"
#include <array>
#include <atomic>
#include <condition_variable>
#include <libusb.h>
#include <mutex>
#include <string>
#include <thread>
namespace wbr::host {
struct UsbOptions {
  bool realtime = true;
  int cpu = 3, priority = 80;
  bool strict_hi91_crc = false;
};
class UsbLink {
public:
  UsbLink(SensorStore &, const UsbOptions &);
  ~UsbLink();
  bool Open();
  bool Send(bridge::Commands packet);
  void Stop();
  void Close();
  bool healthy() const;
  std::string error();
  uint32_t session() const { return session_; }
  uint32_t capabilities() const { return capabilities_; }

private:
  struct Input {
    UsbLink *owner = nullptr;
    libusb_transfer *transfer = nullptr;
    bridge::Packet data{};
    bool pending = false;
  };
  struct Output {
    UsbLink *owner = nullptr;
    libusb_transfer *transfer = nullptr;
    bridge::Commands data{};
    std::atomic<bool> free{true};
  };
  static void LIBUSB_CALL Receive(libusb_transfer *);
  static void LIBUSB_CALL Sent(libusb_transfer *);
  void Events();
  void Submit(Input &);
  bool Transmit(const uint8_t *, size_t);
  bool Fail(const std::string &);
  UsbOptions options_;
  SensorDecoder decoder_;
  libusb_context *context_ = nullptr;
  libusb_device_handle *device_ = nullptr;
  bool claimed_ = false;
  std::array<Input, 4> inputs_{};
  std::array<Output, 8> outputs_{};
  std::thread thread_;
  std::atomic<bool> closing_{false}, failed_{false}, ready_{false};
  std::atomic<unsigned> active_{0};
  std::atomic<uint64_t> last_receive_us_{0};
  std::atomic<bool> inputs_ready_{false};
  uint32_t session_ = 0, sequence_ = 0, capabilities_ = 0;
  std::mutex mutex_;
  std::condition_variable ready_cv_;
  std::string error_;
};
} // namespace wbr::host
