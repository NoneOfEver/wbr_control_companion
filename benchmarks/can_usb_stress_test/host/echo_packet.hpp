/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace stress_host
{
using Packet = std::array<unsigned char, 64>;
constexpr std::uint32_t kMagic = 0x53545253;
inline std::uint32_t ReadU32(const unsigned char *bytes)
{
	return std::uint32_t(bytes[0]) | (std::uint32_t(bytes[1]) << 8) |
	       (std::uint32_t(bytes[2]) << 16) | (std::uint32_t(bytes[3]) << 24);
}
inline void WriteU32(unsigned char *bytes, std::uint32_t value)
{
	for (unsigned index = 0; index < 4; ++index) {
		bytes[index] = static_cast<unsigned char>(value >> (index * 8));
	}
}
inline Packet EncodePacket(std::uint32_t sequence, std::uint32_t previous_rtt_us)
{
	Packet packet{};
	WriteU32(packet.data(), kMagic);
	WriteU32(packet.data() + 4, sequence);
	WriteU32(packet.data() + 8, previous_rtt_us);
	for (std::size_t index = 12; index < packet.size(); ++index) {
		packet[index] = static_cast<unsigned char>(sequence + index - 12);
	}
	return packet;
}
} // namespace stress_host
