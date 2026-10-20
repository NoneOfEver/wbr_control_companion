/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_link.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <random>
#include <wbr/realtime_loop.hpp>
namespace wbr::host {
namespace rt = wbr::realtime;
UsbLink::UsbLink(SensorStore &store, const UsbOptions &options)
    : options_(options), decoder_(store, options.strict_hi91_crc) {}
UsbLink::~UsbLink() { Close(); }
bool UsbLink::Fail(const std::string &reason) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (error_.empty())
      error_ = reason;
  }
  failed_ = true;
  ready_cv_.notify_all();
  return false;
}
std::string UsbLink::error() {
  std::lock_guard<std::mutex> lock(mutex_);
  return error_;
}
bool UsbLink::healthy() const {
  if (!ready_ || failed_)
    return false;
  // Snapshot the receiver first. Sampling time before the atomic load allows
  // Receive() to publish a newer timestamp between them: unsigned age then
  // underflows and reports a healthy stream as timed out.
  const uint64_t received_us = last_receive_us_.load();
  const uint64_t now_us = rt::MonotonicTimeNs() / 1000;
  return now_us - received_us < bridge::kCommandTimeoutUs;
}

UsbStartupTiming UsbLink::startup_timing() const {
  return {event_start_ns_.load(), configured_ns_.load(),
          first_input_ns_.load(), inputs_ready_ns_.load(),
          hello_ns_.load(),       ready_ns_.load()};
}
bool UsbLink::Open() {
  if (libusb_init(&context_))
    return Fail("Cannot initialize USB");
  device_ = libusb_open_device_with_vid_pid(context_, 0x34b7, 0xffff);
  if (!device_)
    return Fail("Production USB bridge not found");
  if (libusb_get_device_speed(libusb_get_device(device_)) != LIBUSB_SPEED_HIGH)
    return Fail("USB bridge must negotiate 480 Mbps");
  libusb_config_descriptor *config = nullptr;
  if (libusb_get_active_config_descriptor(libusb_get_device(device_), &config))
    return Fail("Cannot read USB configuration");
  bool in = false, out = false;
  for (unsigned i = 0; i < config->bNumInterfaces; ++i)
    for (int alternate = 0; alternate < config->interface[i].num_altsetting;
         ++alternate) {
      const auto &interface = config->interface[i].altsetting[alternate];
      if (interface.bInterfaceNumber != 0 || interface.bAlternateSetting != 0)
        continue;
      for (unsigned endpoint = 0; endpoint < interface.bNumEndpoints;
           ++endpoint) {
        const auto &ep = interface.endpoint[endpoint];
        if ((ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) !=
                LIBUSB_TRANSFER_TYPE_BULK ||
            ep.wMaxPacketSize != 512)
          continue;
        in |= ep.bEndpointAddress == 0x81;
        out |= ep.bEndpointAddress == 0x01;
      }
    }
  libusb_free_config_descriptor(config);
  if (!in || !out)
    return Fail("USB interface 0 must provide Bulk 0x01/0x81, MPS 512");
  if (libusb_claim_interface(device_, 0))
    return Fail("Cannot claim USB bridge interface");
  claimed_ = true;
  for (auto &input : inputs_) {
    input.owner = this;
    input.transfer = libusb_alloc_transfer(0);
    if (!input.transfer)
      return Fail("USB IN allocation");
  }
  for (auto &output : outputs_) {
    output.owner = this;
    output.transfer = libusb_alloc_transfer(0);
    if (!output.transfer)
      return Fail("USB OUT allocation");
  }
  session_ = std::random_device{}();
  if (!session_)
    session_ = 1;
  thread_ = std::thread([this] { Events(); });
  // Every session starts receiving before Hello can enable MCU publication.
  // This includes RT setup/mlockall; failures must not start a session.
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!ready_cv_.wait_for(lock, std::chrono::seconds(2), [this] {
          return inputs_ready_.load() || failed_.load();
        })) {
      lock.unlock();
      return Fail("USB IN prepost timed out");
    }
    if (failed_)
      return false;
  }
  bridge::Packet hello{};
  bridge::Header(hello.data(), bridge::Kind::Hello, 0, session_);
  hello[10] = options_.ready_query_page;
  hello_ns_ = rt::MonotonicTimeNs();
  if (!Transmit(hello.data(), hello.size()))
    return false;
  std::unique_lock<std::mutex> lock(mutex_);
  if (!ready_cv_.wait_for(lock, std::chrono::seconds(2),
                          [this] { return ready_.load() || failed_.load(); })) {
    lock.unlock();
    return Fail(
        "Production handshake timed out; flash usb_can_bridge firmware");
  }
  return !failed_;
}
void UsbLink::Submit(Input &input) {
  libusb_fill_bulk_transfer(input.transfer, device_, 0x81, input.data.data(),
                            64, Receive, &input, 0);
  input.pending = true;
  ++active_;
  if (libusb_submit_transfer(input.transfer)) {
    input.pending = false;
    --active_;
    Fail("USB IN submit");
  } else if (!first_input_ns_.load()) {
    uint64_t zero = 0;
    first_input_ns_.compare_exchange_strong(zero, rt::MonotonicTimeNs());
  }
}
void LIBUSB_CALL UsbLink::Receive(libusb_transfer *transfer) {
  auto &input = *static_cast<Input *>(transfer->user_data);
  auto &self = *input.owner;
  input.pending = false;
  --self.active_;
  if (self.closing_)
    return;
  if (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
      transfer->actual_length != 64) {
    self.Fail("USB IN disconnected or invalid length");
    return;
  }
  const auto &packet = input.data;
  if (bridge::U32(packet.data()) == bridge::kMagic &&
      packet[8] == bridge::kVersion &&
      bridge::U32(packet.data() + 12) == self.session_) {
    self.last_receive_us_ = rt::MonotonicTimeNs() / 1000;
    if (packet[9] == uint8_t(bridge::Kind::Ready)) {
      if (self.ready_) {
        self.Fail("Unexpected bridge session restart");
        return;
      }
      if (bridge::U32(packet.data() + 20) != 128 ||
          bridge::U32(packet.data() + 24) != 64 ||
          (bridge::U32(packet.data() + 16) & 7) != 7) {
        self.Fail("Bridge CAN/remote/HI91 capability missing");
        return;
      }
      self.ready_ns_ = rt::MonotonicTimeNs();
      self.capabilities_ = bridge::U32(packet.data() + 16);
      self.decoder_.Reset();
      if (self.options_.observe) {
        const auto now = rt::MonotonicTimeNs();
        self.options_.observe(self.options_.observer_context, packet, now, now);
      }
      self.ready_ = true;
      self.ready_cv_.notify_all();
    } else if (self.ready_) {
      const uint64_t arrival_ns = rt::MonotonicTimeNs();
      if (!self.decoder_.Consume(packet, arrival_ns / 1000))
        self.Fail("Malformed bridge sensor packet");
      else if (self.options_.observe)
        self.options_.observe(self.options_.observer_context, packet,
                              arrival_ns, rt::MonotonicTimeNs());
    }
  }
  if (!self.closing_ && !self.failed_)
    self.Submit(input);
}
void LIBUSB_CALL UsbLink::Sent(libusb_transfer *transfer) {
  auto &output = *static_cast<Output *>(transfer->user_data);
  auto &self = *output.owner;
  --self.active_;
  if (transfer->length == bridge::kOutBytes && self.options_.command_completed)
    self.options_.command_completed(
        self.options_.observer_context,
        transfer->status == LIBUSB_TRANSFER_COMPLETED &&
            transfer->actual_length == transfer->length);
  if (!self.closing_ && (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
                         transfer->actual_length != transfer->length))
    self.Fail("USB OUT disconnected or incomplete");
  output.free.store(true, std::memory_order_release);
}
bool UsbLink::Transmit(const uint8_t *data, size_t length) {
  if (failed_ || closing_)
    return false;
  for (auto &output : outputs_) {
    bool available = true;
    if (!output.free.compare_exchange_strong(available, false))
      continue;
    std::copy_n(data, length, output.data.begin());
    libusb_fill_bulk_transfer(output.transfer, device_, 0x01,
                              output.data.data(), length, Sent, &output, 100);
    ++active_;
    if (libusb_submit_transfer(output.transfer)) {
      --active_;
      output.free = true;
      return Fail("USB OUT submit");
    }
    return true;
  }
  return Fail("USB command queue full; control is stopped instead of replaying "
              "stale commands");
}
bool UsbLink::Send(bridge::Commands packet) {
  if (!healthy())
    return false;
  bridge::Header(packet.data(), bridge::Kind::Commands, ++sequence_, session_);
  return Transmit(packet.data(), packet.size());
}
void UsbLink::Stop() {
  if (!device_ || !thread_.joinable() || closing_)
    return;
  if (!failed_) {
    bridge::Packet stop{};
    bridge::Header(stop.data(), bridge::Kind::Stop, ++sequence_, session_);
    Transmit(stop.data(), stop.size());
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(
      25)); // allow device stop, or 20 ms local watchdog
  ready_ = false;
}
void UsbLink::Events() {
  event_start_ns_ = rt::MonotonicTimeNs();
  std::string error;
  if (options_.realtime &&
      !rt::ConfigureCurrentThread(options_.cpu, options_.priority, error))
    Fail("USB realtime setup: " + error);
  configured_ns_ = rt::MonotonicTimeNs();
  if (!failed_)
    for (auto &input : inputs_)
      Submit(input);
  if (!failed_) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_ready_ns_ = rt::MonotonicTimeNs();
      inputs_ready_ = true;
    }
    ready_cv_.notify_all();
  }
  bool cancelled = false;
  for (;;) {
    if (closing_) {
      if (!cancelled) {
        for (auto &input : inputs_)
          if (input.pending)
            libusb_cancel_transfer(input.transfer);
        for (auto &output : outputs_)
          if (!output.free)
            libusb_cancel_transfer(output.transfer);
        cancelled = true;
      }
      if (!active_)
        break;
    }
    timeval timeout{0, 20000};
    const int result =
        libusb_handle_events_timeout_completed(context_, &timeout, nullptr);
    if (result && result != LIBUSB_ERROR_INTERRUPTED)
      Fail("USB event handling error");
  }
}
void UsbLink::Close() {
  if (!context_)
    return;
  closing_ = true;
  // Sole event owner cancels and drains. No lower-priority event-lock
  // acquisition.
  if (thread_.joinable())
    thread_.join();
  for (auto &input : inputs_)
    if (input.transfer) {
      libusb_free_transfer(input.transfer);
      input.transfer = nullptr;
    }
  for (auto &output : outputs_)
    if (output.transfer) {
      libusb_free_transfer(output.transfer);
      output.transfer = nullptr;
    }
  if (claimed_)
    libusb_release_interface(device_, 0);
  if (device_)
    libusb_close(device_);
  libusb_exit(context_);
  context_ = nullptr;
  device_ = nullptr;
}
} // namespace wbr::host
