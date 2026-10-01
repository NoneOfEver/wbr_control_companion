#include "wbr/mpc_link.hpp"

#include <cstring>

namespace wbr::protocol {
namespace {

constexpr std::uint16_t kCrcInitial = 0xffffU;
constexpr std::uint16_t kCrcPolynomial = 0x8408U;
constexpr std::size_t kStatePayloadSize = 44U;
constexpr std::size_t kCommandPayloadSize = 24U;

void WriteLe16(std::uint8_t* out, std::uint16_t value) {
  out[0] = static_cast<std::uint8_t>(value);
  out[1] = static_cast<std::uint8_t>(value >> 8U);
}

void WriteLe32(std::uint8_t* out, std::uint32_t value) {
  out[0] = static_cast<std::uint8_t>(value);
  out[1] = static_cast<std::uint8_t>(value >> 8U);
  out[2] = static_cast<std::uint8_t>(value >> 16U);
  out[3] = static_cast<std::uint8_t>(value >> 24U);
}

std::uint16_t ReadLe16(const std::uint8_t* in) {
  return static_cast<std::uint16_t>(in[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(in[1]) << 8U);
}

std::uint32_t ReadLe32(const std::uint8_t* in) {
  return static_cast<std::uint32_t>(in[0]) |
         (static_cast<std::uint32_t>(in[1]) << 8U) |
         (static_cast<std::uint32_t>(in[2]) << 16U) |
         (static_cast<std::uint32_t>(in[3]) << 24U);
}

void WriteLeFloat(std::uint8_t* out, float value) {
  std::uint32_t raw = 0U;
  std::memcpy(&raw, &value, sizeof(raw));
  WriteLe32(out, raw);
}

float ReadLeFloat(const std::uint8_t* in) {
  const std::uint32_t raw = ReadLe32(in);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

template <std::size_t Size>
void EncodeHeader(std::array<std::uint8_t, Size>& out, std::uint8_t type,
                  std::uint8_t payload_size, std::uint32_t sequence,
                  std::uint32_t uptime_ms) {
  out[0] = kMagic0;
  out[1] = kMagic1;
  out[2] = kVersion;
  out[3] = type;
  out[4] = payload_size;
  out[5] = 0U;
  WriteLe32(&out[6], sequence);
  WriteLe32(&out[10], uptime_ms);
}

DecodeError Validate(const std::uint8_t* data, std::size_t size,
                     std::size_t expected_size, std::uint8_t type,
                     std::uint8_t payload_size) {
  if (data == nullptr || size != expected_size) return DecodeError::kWrongSize;
  if (data[0] != kMagic0 || data[1] != kMagic1 || data[2] != kVersion ||
      data[3] != type || data[4] != payload_size || data[5] != 0U) {
    return DecodeError::kWrongHeader;
  }
  return ReadLe16(&data[size - 2U]) == CalculateCrc16(data, size - 2U)
             ? DecodeError::kNone
             : DecodeError::kWrongCrc;
}

}  // namespace

std::uint16_t CalculateCrc16(const std::uint8_t* data, std::size_t size) {
  if (data == nullptr && size != 0U) return 0U;
  std::uint16_t crc = kCrcInitial;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]);
    for (std::uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc & 1U) != 0U
                ? static_cast<std::uint16_t>((crc >> 1U) ^ kCrcPolynomial)
                : static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

std::array<std::uint8_t, kStateFrameSize> EncodeState(const StateFrame& state) {
  std::array<std::uint8_t, kStateFrameSize> out{};
  EncodeHeader(out, kStateMessageType, kStatePayloadSize, state.sequence,
               state.sender_uptime_ms);
  const float values[] = {
      state.roll_rad, state.roll_rate_rad_s, state.body_height_m,
      state.body_height_rate_m_s, state.left_leg_length_m,
      state.left_leg_rate_m_s, state.right_leg_length_m,
      state.right_leg_rate_m_s, state.left_support_force_n,
      state.right_support_force_n,
  };
  for (std::size_t i = 0; i < 10U; ++i) {
    WriteLeFloat(&out[kHeaderSize + i * 4U], values[i]);
  }
  out[kHeaderSize + 40U] = state.control_state;
  out[kHeaderSize + 41U] = state.valid_flags;
  out[kHeaderSize + 42U] = state.contact_flags;
  out[kHeaderSize + 43U] = state.motor_stale_mask;
  WriteLe16(&out[kStateFrameSize - 2U], CalculateCrc16(out.data(), out.size() - 2U));
  return out;
}

std::array<std::uint8_t, kCommandFrameSize> EncodeCommand(
    const CommandFrame& command) {
  std::array<std::uint8_t, kCommandFrameSize> out{};
  EncodeHeader(out, kCommandMessageType, kCommandPayloadSize, command.sequence,
               command.sender_uptime_ms);
  WriteLe32(&out[kHeaderSize], command.source_state_sequence);
  WriteLeFloat(&out[kHeaderSize + 4U], command.left_support_force_n);
  WriteLeFloat(&out[kHeaderSize + 8U], command.right_support_force_n);
  out[kHeaderSize + 12U] = static_cast<std::uint8_t>(command.mode);
  out[kHeaderSize + 13U] = static_cast<std::uint8_t>(command.contact_mode);
  out[kHeaderSize + 14U] = command.solver_flags;
  WriteLe16(&out[kHeaderSize + 16U], command.solver_iterations);
  WriteLe16(&out[kHeaderSize + 18U], command.valid_for_ms);
  WriteLe32(&out[kHeaderSize + 20U], command.solve_time_us);
  WriteLe16(&out[kCommandFrameSize - 2U],
            CalculateCrc16(out.data(), out.size() - 2U));
  return out;
}

DecodeError DecodeState(const std::uint8_t* data, std::size_t size,
                        StateFrame& out) {
  const DecodeError error =
      Validate(data, size, kStateFrameSize, kStateMessageType, kStatePayloadSize);
  if (error != DecodeError::kNone) return error;
  out.sequence = ReadLe32(&data[6]);
  out.sender_uptime_ms = ReadLe32(&data[10]);
  float* values[] = {
      &out.roll_rad, &out.roll_rate_rad_s, &out.body_height_m,
      &out.body_height_rate_m_s, &out.left_leg_length_m,
      &out.left_leg_rate_m_s, &out.right_leg_length_m,
      &out.right_leg_rate_m_s, &out.left_support_force_n,
      &out.right_support_force_n,
  };
  for (std::size_t i = 0; i < 10U; ++i) {
    *values[i] = ReadLeFloat(&data[kHeaderSize + i * 4U]);
  }
  out.control_state = data[kHeaderSize + 40U];
  out.valid_flags = data[kHeaderSize + 41U];
  out.contact_flags = data[kHeaderSize + 42U];
  out.motor_stale_mask = data[kHeaderSize + 43U];
  return DecodeError::kNone;
}

DecodeError DecodeCommand(const std::uint8_t* data, std::size_t size,
                          CommandFrame& out) {
  const DecodeError error = Validate(data, size, kCommandFrameSize,
                                     kCommandMessageType, kCommandPayloadSize);
  if (error != DecodeError::kNone) return error;
  out.sequence = ReadLe32(&data[6]);
  out.sender_uptime_ms = ReadLe32(&data[10]);
  out.source_state_sequence = ReadLe32(&data[kHeaderSize]);
  out.left_support_force_n = ReadLeFloat(&data[kHeaderSize + 4U]);
  out.right_support_force_n = ReadLeFloat(&data[kHeaderSize + 8U]);
  out.mode = static_cast<CommandMode>(data[kHeaderSize + 12U]);
  out.contact_mode = static_cast<ContactMode>(data[kHeaderSize + 13U]);
  out.solver_flags = data[kHeaderSize + 14U];
  out.solver_iterations = ReadLe16(&data[kHeaderSize + 16U]);
  out.valid_for_ms = ReadLe16(&data[kHeaderSize + 18U]);
  out.solve_time_us = ReadLe32(&data[kHeaderSize + 20U]);
  return DecodeError::kNone;
}

}  // namespace wbr::protocol
