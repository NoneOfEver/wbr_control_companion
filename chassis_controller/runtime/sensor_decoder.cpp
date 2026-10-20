/* SPDX-License-Identifier: Apache-2.0 */
#include "sensor_decoder.hpp"
#include <algorithm>
#include <chassis_controller/motor_map.h>
#include <cmath>
#include <cstring>
#include <hi91_imu_params_generated.h>
#include <protocols/imu/hi91_protocol.h>
#include <protocols/remote_input/wfly_sbus_protocol.h>
#include <remote_input_params_generated.h>
namespace wbr::host {
void SensorDecoder::Reset() {
  latest_ = {};
  streams_ = {};
  remote_sequence_ = imu_sequence_ = 0;
  has_hi91_sensor_time_ = false;
  store_.Publish(latest_);
}
void SensorDecoder::ConsumeBytes(Stream &stream, size_t count) {
  if (count >= stream.size) {
    stream.size = 0;
    return;
  }
  std::memmove(stream.bytes.data(), stream.bytes.data() + count,
               stream.size - count);
  stream.size -= count;
}
bool SensorDecoder::Consume(const bridge::Packet &packet, uint64_t host_us) {
  using namespace bridge;
  if (U32(packet.data()) != kMagic || packet[8] != kVersion)
    return false;
  if (packet[9] == uint8_t(Kind::Can)) {
    if (packet[10] < 1 || packet[10] > 3)
      return false;
    // Reject a malformed packet atomically before publishing partial state.
    for (unsigned i = 0; i < packet[10]; ++i)
      if (!ValidCan(packet.data() + 16 + 16 * i))
        return false;
    for (unsigned i = 0; i < packet[10]; ++i) {
      const auto *record = packet.data() + 16 + 16 * i;
      for (unsigned motor = 0; motor < 6; ++motor) {
        const auto &address = platform::motor_map::kMotors[motor];
        if (record[0] != address.bus || U16(record + 2) != address.feedback_id)
          continue;
        auto &frame = latest_.control.motors[motor];
        const uint32_t age =
            uint32_t(U32(packet.data() + 4) - U32(record + 12));
        frame.valid =
            record[1] == 8 && age <= kCommandTimeoutUs && host_us >= age;
        frame.precise_timestamp_us = frame.valid ? host_us - age : 0;
        frame.timestamp_us = frame.precise_timestamp_us;
        std::memcpy(frame.data, record + 4, 8);
      }
    }
  } else if (packet[9] == uint8_t(Kind::Uart)) {
    if (packet[10] < 1 || packet[10] > 3 || packet[11] > 40)
      return false;
    const unsigned port = packet[10] - 1;
    auto &stream = streams_[port];
    const uint32_t sequence = U32(packet.data() + 16);
    if (stream.has_sequence && sequence != stream.sequence + 1) {
      stream.size = 0;
      if (port == 0) {
        latest_.control.remote = {};
        latest_.control.remote_disable_pose = false;
        latest_.control.remote.sequence = ++remote_sequence_;
      }
      if (port == 1)
        latest_.control.imu.valid = false;
    }
    stream.sequence = sequence;
    stream.has_sequence = true;
    const uint32_t age =
        uint32_t(U32(packet.data() + 4) - U32(packet.data() + 20));
    if (age > kCommandTimeoutUs || host_us < age) {
      stream.size = 0;
      return false;
    }
    Feed(port, packet.data() + 24, packet[11], host_us - age);
  } else if (packet[9] == uint8_t(Kind::Imu)) {
    if (packet[10] != 25)
      return false;
    const uint32_t age =
        uint32_t(U32(packet.data() + 4) - U32(packet.data() + 16));
    if (age > kCommandTimeoutUs || host_us < age)
      return false;
    std::memcpy(latest_.peripheral.onboard_raw.data(), packet.data() + 20, 25);
    std::memcpy(&latest_.peripheral.onboard_temperature_c, packet.data() + 48,
                4);
    latest_.peripheral.onboard_sample_us = host_us - age;
    // Current production control source remains HI91. Raw onboard samples
    // are forwarded without silently selecting an unverified body transform.
  } else
    return false;
  store_.Publish(latest_);
  return true;
}
void SensorDecoder::Feed(unsigned port, const uint8_t *data, size_t count,
                         uint64_t timestamp) {
  auto &stream = streams_[port];
  if (port == 2) {
    std::copy_n(data, count, latest_.peripheral.referee_chunk.begin());
    latest_.peripheral.referee_bytes = count;
    latest_.peripheral.referee_sample_us = timestamp;
    return;
  }
  for (size_t i = 0; i < count; ++i) {
    if (stream.size == stream.bytes.size())
      ConsumeBytes(stream, 1);
    stream.bytes[stream.size++] = data[i];
    if (port == 0)
      ParseRemote(stream, timestamp);
    else
      ParseHi91(stream, timestamp);
  }
}
void SensorDecoder::ParseRemote(Stream &stream, uint64_t) {
  // UART10 is configured by the production board as SBUS 100k 8E2.
  while (stream.size && stream.bytes[0] != protocols::kWflySbusStartByte)
    ConsumeBytes(stream, 1);
  if (stream.size < protocols::kWflySbusFrameLength)
    return;
  protocols::WflySbusFrame decoded{};
  if (!protocols::DecodeWflySbusFrame(stream.bytes.data(), 25, &decoded)) {
    ConsumeBytes(stream, 1);
    return;
  }
  msg::RemoteInputState remote{};
  remote.source = msg::kRemoteInputWfly;
  auto normalized = [](uint16_t value) {
    return std::clamp(
        (float(value) - modules::remote_input_params::kRemoteInputWflySbusMid) /
            modules::remote_input_params::kRemoteInputWflySbusScale,
        -1.0F, 1.0F);
  };
  auto position = [](uint16_t value) {
    return value < modules::remote_input_params::
                       kRemoteInputWflySwitchLowMidThreshold
               ? 0
           : value > modules::remote_input_params::
                         kRemoteInputWflySwitchMidHighThreshold
               ? 2
               : 1;
  };
  const auto ch4 = position(decoded.msg[4]), ch5 = position(decoded.msg[5]);
  latest_.control.remote_disable_pose =
      !decoded.failsafe && !decoded.frame_lost && ch4 == 0 && ch5 == 0;
  if (!decoded.failsafe && !decoded.frame_lost) {
    remote.robot_enable = (ch4 != 0 || ch5 != 0);
    remote.run = remote.robot_enable;
    if (remote.robot_enable) {
      remote.chassis_x = normalized(decoded.msg[2]);
      remote.chassis_rotate = normalized(decoded.msg[3]);
      if (ch4 == 0) {
        remote.yaw_angle = normalized(decoded.msg[0]);
        remote.pitch_angle = normalized(decoded.msg[1]);
      }
      if (ch4 == 1) {
        remote.leg_length_delta = normalized(decoded.msg[1]);
        remote.leg_length = remote.leg_length_delta;
      }
      if (ch4 == 2) {
        remote.chassis_x = remote.chassis_rotate = 0;
        remote.climb_stairs = true;
      }
      remote.jump = ch5 == 2;
    }
  }
  remote.sequence = ++remote_sequence_;
  latest_.control.remote = remote;
  ConsumeBytes(stream, 25);
}
void SensorDecoder::ParseHi91(Stream &stream, uint64_t timestamp) {
  for (;;) {
    if (stream.size < 2)
      return;
    if (stream.bytes[0] != 0x5a || stream.bytes[1] != 0xa5) {
      ConsumeBytes(stream, 1);
      continue;
    }
    if (stream.size < 6)
      return;
    const size_t payload = bridge::U16(stream.bytes.data() + 2);
    if (payload > protocols::kHi91MaxPayloadLength ||
        payload < protocols::kHi91DataLength) {
      ConsumeBytes(stream, 1);
      continue;
    }
    const size_t size = payload + protocols::kHi91FrameHeaderSize;
    if (stream.size < size)
      return;
    protocols::Hi91Sample decoded{};
    const int result = protocols::DecodeHi91Frame(stream.bytes.data(), size,
                                                  strict_crc_, &decoded);
    if (result || !std::isfinite(decoded.pitch_deg) ||
        std::abs(decoded.pitch_deg) >
            modules::hi91_imu_params::kHi91MaximumValidPitchDeg ||
        !std::isfinite(decoded.roll_deg) || !std::isfinite(decoded.yaw_deg)) {
      ConsumeBytes(stream, 1);
      continue;
    }
    if (has_hi91_sensor_time_ &&
        !bridge::NewSequence(decoded.system_time_ms, hi91_sensor_time_)) {
      ConsumeBytes(stream, size);
      continue;
    }
    bool finite = true;
    for (unsigned i = 0; i < 3; ++i)
      finite = finite && std::isfinite(decoded.gyro_dps[i]) &&
               std::isfinite(decoded.accel_g[i]);
    if (!finite) {
      ConsumeBytes(stream, size);
      continue;
    }
    auto &imu = latest_.control.imu;
    imu = {};
    imu.sequence = ++imu_sequence_;
    imu.precise_timestamp_us = timestamp;
    imu.valid = true;
    imu.pitch_deg = decoded.pitch_deg;
    imu.roll_deg = decoded.roll_deg;
    imu.yaw_deg = decoded.yaw_deg;
    constexpr float radians = 0.01745329251994329577F;
    // Preserve existing HI91 mounting: gyro X -> pitch, Y -> roll.
    imu.pitch_rate_rad_s = decoded.gyro_dps[0] * radians;
    imu.roll_rate_rad_s = decoded.gyro_dps[1] * radians;
    imu.yaw_rate_rad_s = decoded.gyro_dps[2] * radians;
    std::copy_n(decoded.accel_g, 3, imu.accel_g);
    imu.vertical_accel_g = decoded.accel_g[2];
    hi91_sensor_time_ = decoded.system_time_ms;
    has_hi91_sensor_time_ = true;
    ConsumeBytes(stream, size);
  }
}
} // namespace wbr::host
