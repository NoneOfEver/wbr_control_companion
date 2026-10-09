#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace wbr::protocol {

constexpr std::uint8_t kMagic0 = 'W';
constexpr std::uint8_t kMagic1 = 'M';
constexpr std::uint8_t kVersion = 2U;
constexpr std::uint8_t kStateMessageType = 0x10U;
constexpr std::uint8_t kCommandMessageType = 0x20U;
constexpr std::size_t kHeaderSize = 14U;
constexpr std::size_t kStateFrameSize = 64U;
constexpr std::size_t kCommandFrameSize = 40U;

enum StateValidFlag : std::uint8_t {
  kStateImuFresh = 1U << 0,
  kStateSupportForceValid = 1U << 1,
  kStateMotorFeedbackValid = 1U << 2,
  kStateControlEnabled = 1U << 3,
  kStateOperatorEnabled = 1U << 4,
  kStateActiveApplied = 1U << 5,
};

enum ContactFlag : std::uint8_t {
  kContactLeft = 1U << 0,
  kContactRight = 1U << 1,
};

enum class CommandMode : std::uint8_t {
  kDisabled = 0U,
  kShadow = 1U,
  kActive = 2U,
};

enum class ContactMode : std::uint8_t {
  kUnknown = 0U,
  kBoth = 1U,
  kLeftOnly = 2U,
  kRightOnly = 3U,
  kAirborne = 4U,
};

enum SolverFlag : std::uint8_t {
  kSolverConverged = 1U << 0,
  kSolverInputFinite = 1U << 1,
  kSolverUsedFallback = 1U << 2,
};

struct StateFrame {
  std::uint32_t sequence = 0U;
  std::uint32_t sender_uptime_ms = 0U;
  float roll_rad = 0.0F;
  float roll_rate_rad_s = 0.0F;
  float body_height_m = 0.0F;
  float body_height_rate_m_s = 0.0F;
  float target_body_height_m = 0.0F;
  float left_leg_length_m = 0.0F;
  float left_leg_rate_m_s = 0.0F;
  float right_leg_length_m = 0.0F;
  float right_leg_rate_m_s = 0.0F;
  float left_support_force_n = 0.0F;
  float right_support_force_n = 0.0F;
  std::uint8_t control_state = 0U;
  std::uint8_t valid_flags = 0U;
  std::uint8_t contact_flags = 0U;
  std::uint8_t motor_stale_mask = 0U;
};

struct CommandFrame {
  std::uint32_t sequence = 0U;
  std::uint32_t sender_uptime_ms = 0U;
  std::uint32_t source_state_sequence = 0U;
  float left_support_force_n = 0.0F;
  float right_support_force_n = 0.0F;
  CommandMode mode = CommandMode::kDisabled;
  ContactMode contact_mode = ContactMode::kUnknown;
  std::uint8_t solver_flags = 0U;
  std::uint16_t solver_iterations = 0U;
  std::uint16_t valid_for_ms = 0U;
  std::uint32_t solve_time_us = 0U;
};

enum class DecodeError {
  kNone,
  kWrongSize,
  kWrongHeader,
  kWrongCrc,
};

std::uint16_t CalculateCrc16(const std::uint8_t* data, std::size_t size);
std::array<std::uint8_t, kStateFrameSize> EncodeState(const StateFrame& state);
std::array<std::uint8_t, kCommandFrameSize> EncodeCommand(const CommandFrame& command);
DecodeError DecodeState(const std::uint8_t* data, std::size_t size, StateFrame& out);
DecodeError DecodeCommand(const std::uint8_t* data, std::size_t size, CommandFrame& out);

}  // namespace wbr::protocol
