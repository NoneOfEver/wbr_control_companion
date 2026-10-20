/* SPDX-License-Identifier: Apache-2.0 */
// Test-only backend. It never opens hardware and is never linked to production.
#include <algorithm>
#include <chassis_controller/motor_map.h>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <libusb.h>
#include <mutex>
#include <protocols/imu/hi91_protocol.h>
#include <wbr_bridge/bridge_protocol.hpp>

struct libusb_context {};
struct libusb_device_handle {};
struct libusb_device {};
namespace {
namespace wire = wbr::bridge;
libusb_context context;
libusb_device_handle handle;
libusb_device device;
std::mutex mutex;
std::condition_variable changed;
std::deque<libusb_transfer *> inputs, outputs, cancelled;
std::deque<wire::Packet> replies;
uint32_t session, command_count, last_sequence;
bool ready_delivered, stop_received, disconnected;
uint32_t Now() {
  return uint32_t(std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count());
}
void FullLoadSensors(const wire::Commands &packet) {
  if (packet[10] != 6)
    std::abort();
  for (unsigned i = 0; i < 6; ++i) {
    const auto *frame = packet.data() + 32 + 16 * i;
    const unsigned slot = frame[12];
    for (unsigned byte = 0; byte < 8; ++byte) {
      const uint8_t expected = slot < 2 ? 0 : (byte == 7 ? 0xfd : 0xff);
      if (frame[4 + byte] != expected)
        std::abort();
    }
  }
  static uint32_t uart_sequence[2]{};
  const auto uart = [&](unsigned port, const uint8_t *data, size_t size) {
    while (size) {
      const auto count = std::min(size, size_t(40));
      wire::Packet packet{};
      const auto stamp = Now();
      wire::Header(packet.data(), wire::Kind::Uart, stamp, session);
      packet[10] = port;
      packet[11] = count;
      wire::Put32(packet.data() + 16, uart_sequence[port - 1]++);
      wire::Put32(packet.data() + 20, stamp - 7);
      std::copy_n(data, count, packet.data() + 24);
      replies.push_back(packet);
      data += count;
      size -= count;
    }
  };
  static uint32_t last_sbus_us = 0;
  const uint32_t now_us = Now();
  if (!last_sbus_us || uint32_t(now_us - last_sbus_us) >= 14000) {
    last_sbus_us = now_us;
    uint8_t sbus[25]{};
    sbus[0] = 0x0f;
    uart(1, sbus, sizeof(sbus));
  }
  uint8_t hi91[82]{};
  hi91[0] = 0x5a;
  hi91[1] = 0xa5;
  wire::Put16(hi91 + 2, 76);
  hi91[6] = 0x91;
  wire::Put32(hi91 + 14, command_count);
  wire::Put16(hi91 + 4, protocols::CalculateHi91FrameCrc(76, hi91 + 6));
  uart(2, hi91, sizeof(hi91));
  if (!std::getenv("WBR_MOCK_FULL_DROP_IMU")) {
    wire::Packet imu{};
    const auto stamp = Now();
    wire::Header(imu.data(), wire::Kind::Imu, stamp, session);
    imu[10] = 25;
    wire::Put32(imu.data() + 16, stamp - 7);
    replies.push_back(imu);
  }
}
void Process(libusb_transfer *transfer) {
  const auto *data = transfer->buffer;
  if (wire::HasHeader(data, transfer->length, wire::Kind::Hello)) {
    if (transfer->length != 64)
      std::abort();
    session = wire::U32(data + 12);
    ready_delivered = false;
    stop_received = false;
    last_sequence = 0;
    replies.clear();
    wire::Packet reply{};
    wire::Header(reply.data(), wire::Kind::Ready, Now(), session);
    wire::Put32(reply.data() + 16, std::getenv("WBR_MOCK_BAD_CAPS") ? 0 : 15);
    wire::Put32(reply.data() + 20, 128);
    wire::Put32(reply.data() + 24, 64);
    if (std::getenv("WBR_MOCK_UART")) {
      wire::Put32(reply.data() + 28, 0x33545255);
      wire::Put32(reply.data() + 32, 3);
      wire::Put32(reply.data() + 36, command_count);
      wire::Put32(reply.data() + 40, command_count + 1);
      wire::Put32(reply.data() + 48, command_count + 1);
      wire::Put32(reply.data() + 56, 128);
      wire::Put32(reply.data() + 60, 103 | (25U << 16));
    }
    if (std::getenv("WBR_MOCK_FULL_LOAD")) {
      wire::Put32(reply.data() + 28, 0x34545255);
      wire::Put32(reply.data() + 32, 19);
    }
    if (std::getenv("WBR_MOCK_OLD_FIRMWARE"))
      wire::Put32(reply.data(), 0x314b4341);
    replies.push_back(reply);
    if (std::getenv("WBR_MOCK_STALE_SESSION")) {
      wire::Packet stale{};
      wire::Header(stale.data(), wire::Kind::Can, Now(), session ^ 0xffffffffU);
      stale[10] = 1;
      uint8_t payload[8]{};
      wire::CanRecord(stale.data() + 16, 3, 0x201, payload, 8, Now());
      replies.push_back(stale);
    }
  } else if (wire::HasHeader(data, transfer->length, wire::Kind::Commands)) {
    if (!ready_delivered || transfer->length != 128)
      std::abort();
    wire::Commands packet{};
    std::copy_n(data, 128, packet.begin());
    if (!wire::ValidCommands(packet, session))
      std::abort();
    const auto sequence = wire::U32(data + 4);
    if (command_count && !wire::NewSequence(sequence, last_sequence))
      std::abort();
    last_sequence = sequence;
    ++command_count;
    if (std::getenv("WBR_MOCK_FULL_LOAD"))
      FullLoadSensors(packet);
    if (std::getenv("WBR_MOCK_UART")) {
      if (packet[10] != 0)
        std::abort(); // UART capture must never send motor commands.
      for (unsigned port = 1; port <= 2; ++port) {
        wire::Packet uart{};
        const uint32_t stamp = Now();
        wire::Header(uart.data(), wire::Kind::Uart, stamp, session);
        uart[10] = port;
        uart[11] = 8;
        // One deliberately missing chunk exercises stream-gap reporting.
        wire::Put32(uart.data() + 16,
                    command_count + (command_count >= 3 ? 1 : 0));
        wire::Put32(uart.data() + 20, stamp - 7);
        replies.push_back(uart);
      }
    }
    for (unsigned side = 0; side < 2; ++side) {
      wire::Packet feedback{};
      wire::Header(feedback.data(), wire::Kind::Can, Now(), session);
      feedback[10] = 3;
      const unsigned slots[2][3] = {{0, 2, 3}, {1, 4, 5}};
      for (unsigned i = 0; i < 3; ++i) {
        const auto &motor = platform::motor_map::kMotors[slots[side][i]];
        uint8_t value[8]{};
        if (slots[side][i] == 2 && std::getenv("WBR_MOCK_FULL_FAULT"))
          value[0] = 0xa0;
        wire::CanRecord(feedback.data() + 16 + 16 * i, motor.bus,
                        motor.feedback_id, value, 8,
                        wire::U32(feedback.data() + 4));
      }
      replies.push_back(feedback);
    }
  } else if (wire::HasHeader(data, transfer->length, wire::Kind::Stop)) {
    if (transfer->length != 64 || wire::U32(data + 12) != session)
      std::abort();
    stop_received = true;
  } else
    std::abort();
}
} // namespace
extern "C" {
int LIBUSB_CALL libusb_init(libusb_context **output) {
  *output = &context;
  return 0;
}
void LIBUSB_CALL libusb_exit(libusb_context *) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!inputs.empty() || !outputs.empty() || !cancelled.empty())
    std::abort();
  if (command_count && !disconnected && !stop_received)
    std::abort();
}
libusb_device_handle *LIBUSB_CALL
libusb_open_device_with_vid_pid(libusb_context *, uint16_t, uint16_t) {
  return &handle;
}
libusb_device *LIBUSB_CALL libusb_get_device(libusb_device_handle *) {
  return &device;
}
int LIBUSB_CALL libusb_get_device_speed(libusb_device *) {
  return LIBUSB_SPEED_HIGH;
}
int LIBUSB_CALL libusb_claim_interface(libusb_device_handle *, int) {
  return 0;
}
int LIBUSB_CALL libusb_release_interface(libusb_device_handle *, int) {
  return 0;
}
void LIBUSB_CALL libusb_close(libusb_device_handle *) {}
int LIBUSB_CALL libusb_get_active_config_descriptor(
    libusb_device *, libusb_config_descriptor **output) {
  static libusb_endpoint_descriptor endpoints[2]{};
  endpoints[0].bEndpointAddress = 1;
  endpoints[1].bEndpointAddress = 0x81;
  for (auto &endpoint : endpoints) {
    endpoint.bmAttributes = LIBUSB_TRANSFER_TYPE_BULK;
    endpoint.wMaxPacketSize = 512;
  }
  static libusb_interface_descriptor descriptor{};
  descriptor.bNumEndpoints = 2;
  descriptor.endpoint = endpoints;
  static libusb_interface interface{&descriptor, 1};
  auto *config = new libusb_config_descriptor{};
  config->bNumInterfaces = 1;
  config->interface = &interface;
  *output = config;
  return 0;
}
void LIBUSB_CALL
libusb_free_config_descriptor(libusb_config_descriptor *config) {
  delete config;
}
libusb_transfer *LIBUSB_CALL libusb_alloc_transfer(int) {
  return static_cast<libusb_transfer *>(
      std::calloc(1, sizeof(libusb_transfer)));
}
void LIBUSB_CALL libusb_free_transfer(libusb_transfer *transfer) {
  std::free(transfer);
}
int LIBUSB_CALL libusb_submit_transfer(libusb_transfer *transfer) {
  std::lock_guard<std::mutex> lock(mutex);
  if (transfer->endpoint == 0x81 && std::getenv("WBR_MOCK_IN_FAIL"))
    return LIBUSB_ERROR_IO;
  if (transfer->endpoint == 0x01 &&
      wire::HasHeader(transfer->buffer, transfer->length, wire::Kind::Hello) &&
      std::getenv("WBR_MOCK_REQUIRE_PREPOST") && inputs.size() != 4)
    std::abort();
  (transfer->endpoint == 0x81 ? inputs : outputs).push_back(transfer);
  changed.notify_all();
  return 0;
}
int LIBUSB_CALL libusb_cancel_transfer(libusb_transfer *transfer) {
  std::lock_guard<std::mutex> lock(mutex);
  for (auto *queue : {&inputs, &outputs}) {
    auto found = std::find(queue->begin(), queue->end(), transfer);
    if (found != queue->end()) {
      queue->erase(found);
      cancelled.push_back(transfer);
      changed.notify_all();
      return 0;
    }
  }
  return LIBUSB_ERROR_NOT_FOUND;
}
int LIBUSB_CALL libusb_handle_events_timeout_completed(libusb_context *,
                                                       timeval *, int *) {
  libusb_transfer *transfer = nullptr;
  {
    std::unique_lock<std::mutex> lock(mutex);
    changed.wait_for(lock, std::chrono::milliseconds(10), [] {
      return !cancelled.empty() || !outputs.empty() ||
             (!inputs.empty() && !replies.empty());
    });
    if (!cancelled.empty()) {
      transfer = cancelled.front();
      cancelled.pop_front();
      transfer->status = LIBUSB_TRANSFER_CANCELLED;
      transfer->actual_length = 0;
    } else if (!outputs.empty()) {
      transfer = outputs.front();
      outputs.pop_front();
      Process(transfer);
      transfer->status = LIBUSB_TRANSFER_COMPLETED;
      transfer->actual_length = transfer->length;
    } else if (!inputs.empty() && !replies.empty()) {
      transfer = inputs.front();
      inputs.pop_front();
      if (std::getenv("WBR_MOCK_DISCONNECT") && command_count >= 5 &&
          !disconnected) {
        disconnected = true;
        transfer->status = LIBUSB_TRANSFER_NO_DEVICE;
        transfer->actual_length = 0;
      } else {
        const auto packet = replies.front();
        replies.pop_front();
        std::copy(packet.begin(), packet.end(), transfer->buffer);
        if (packet[9] == uint8_t(wire::Kind::Ready))
          ready_delivered = true;
        transfer->status = LIBUSB_TRANSFER_COMPLETED;
        transfer->actual_length = 64;
      }
    }
  }
  if (transfer)
    transfer->callback(transfer);
  return 0;
}
}
