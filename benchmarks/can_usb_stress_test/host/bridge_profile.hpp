/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "bridge_packet.hpp"

namespace bridge_wire
{
// Wire records and diagnostic motor indices stay canonical W/B/D. Only CAN
// publication order changes, so motor identity never depends on queue position.
constexpr uint32_t kProfileQuery = 0x31514650; // PFQ1
constexpr uint32_t kProfileReply = 0x31524650; // PFR1
constexpr const char *kCanOrderNames[] = {"wbd", "wdb", "bwd", "bdw", "dwb", "dbw"};
constexpr unsigned kCanOrders[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2},
				       {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
constexpr bool ValidProfile(unsigned order, unsigned fast)
{
	return order < 6 && fast <= 1;
}
inline Packet ProfileQuery(uint32_t sequence, unsigned order, bool fast)
{
	Packet packet{};
	WriteU32(packet.data(), kProfileQuery);
	WriteU32(packet.data() + 4, sequence);
	WriteU32(packet.data() + 8, order);
	WriteU32(packet.data() + 12, fast);
	return packet;
}
inline bool ValidProfileReply(const Packet &packet, uint32_t sequence, unsigned order, bool fast)
{
	return ReadU32(packet.data()) == kProfileReply && ReadU32(packet.data() + 4) == sequence &&
	       ReadU32(packet.data() + 8) == order &&
	       ReadU32(packet.data() + 12) == unsigned(fast) && ReadU32(packet.data() + 16) == 0;
}
} // namespace bridge_wire
