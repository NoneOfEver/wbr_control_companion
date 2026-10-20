/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "control_latency.hpp"
#include <algorithm>

namespace bridge_wire
{
using MergedControl = std::array<unsigned char, 128>;
constexpr uint32_t kMergedControlCapability = 0x3147524d; // MRG1
constexpr uint32_t kMergedCapabilityQuery = 0x3151524d;   // MRQ1, always 64 bytes
constexpr uint32_t kMergedCapabilityReply = 0x3150524d;   // MRP1
inline Packet MergedCapabilityQuery(uint32_t sequence)
{
	Packet p{};
	WriteU32(p.data(), kMergedCapabilityQuery);
	WriteU32(p.data() + 4, sequence);
	return p;
}
inline bool SupportsMergedControl(const Packet &p)
{
	return ReadU32(p.data()) == kMergedCapabilityReply &&
	       ReadU32(p.data() + 8) == kMergedControlCapability && ReadU32(p.data() + 12) >= 128 &&
	       ReadU32(p.data() + 16) == 64;
}
inline MergedControl MergeControl(const Packet &left, const Packet &right)
{
	MergedControl transfer{};
	std::copy(left.begin(), left.end(), transfer.begin());
	std::copy(right.begin(), right.end(), transfer.begin() + 64);
	return transfer;
}
inline bool SplitMergedControl(const MergedControl &transfer, Packet (&groups)[2])
{
	for (unsigned group = 0; group < 2; ++group) {
		std::copy_n(transfer.begin() + group * 64, 64, groups[group].begin());
		const auto &packet = groups[group];
		if (ReadU32(packet.data()) != kControlCommand ||
		    (ReadU32(packet.data() + 12) & ~7U))
			return false;
		for (unsigned item = 0; item < 3; ++item) {
			const auto *frame = packet.data() + FrameOffset(item);
			const auto &layout = kLayout[group][item];
			if (frame[0] != layout.bus || frame[1] != 8 ||
			    ReadId(frame) != layout.command_id ||
			    !std::equal(frame + 4, frame + 12, layout.zero))
				return false;
		}
	}
	const uint32_t sequence = ReadU32(groups[0].data() + 4);
	return !(sequence & 1U) && ReadU32(groups[1].data() + 4) == sequence + 1;
}
} // namespace bridge_wire
