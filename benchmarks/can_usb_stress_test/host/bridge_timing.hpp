/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "bridge_packet.hpp"
#include <limits>

namespace bridge_wire
{
constexpr uint32_t kTimingQueryMagic = 0x31514d54; // TMQ1
constexpr uint32_t kTimingReplyMagic = 0x31524d54; // TMR1
constexpr uint32_t kTimingUnavailable = 1U << 31;
constexpr uint32_t kUnsetTime = std::numeric_limits<uint32_t>::max();
constexpr unsigned kTimingCapacity = 4096;
// Sample both consecutive bus packets in each group of 32 control cycles.
inline bool SampleTiming(uint32_t sequence)
{
	return (sequence & 63U) < 2;
}
inline unsigned TimingIndex(uint32_t sequence)
{
	return (sequence / 64) * 2 + (sequence & 1);
}

enum class TimePoint : unsigned {
	kDequeued,
	kFirstSubmit,
	kLastSubmit,
	kFirstDriver,
	kLastDriver,
	kFirstTxComplete,
	kLastTxComplete,
	kFirstFeedback,
	kLastFeedback,
	kCollected,
	kQueued,
	kInSubmit,
	kCount
};
static_assert(16 + 4 * static_cast<unsigned>(TimePoint::kCount) == 64);
inline uint32_t GetTime(const Packet &packet, TimePoint point)
{
	return ReadU32(packet.data() + 16 + 4 * static_cast<unsigned>(point));
}
inline void SetTime(Packet &packet, TimePoint point, uint32_t value)
{
	WriteU32(packet.data() + 16 + 4 * static_cast<unsigned>(point), value);
}
inline Packet NewTiming(uint32_t sequence, uint32_t receive_us)
{
	Packet packet{};
	packet.fill(0xff);
	WriteU32(packet.data(), kTimingReplyMagic);
	WriteU32(packet.data() + 4, sequence);
	WriteU32(packet.data() + 8, 0);
	WriteU32(packet.data() + 12, receive_us);
	return packet;
}
inline void MarkTime(Packet &packet, TimePoint point, uint32_t now_us)
{
	SetTime(packet, point, now_us - ReadU32(packet.data() + 12));
}
inline void MarkRange(Packet &packet, TimePoint first, TimePoint last, uint32_t now_us)
{
	if (GetTime(packet, first) == kUnsetTime)
		MarkTime(packet, first, now_us);
	MarkTime(packet, last, now_us);
}
inline Packet TimingQuery(uint32_t sequence)
{
	Packet packet{};
	WriteU32(packet.data(), kTimingQueryMagic);
	WriteU32(packet.data() + 4, sequence);
	return packet;
}
inline bool ValidTiming(const Packet &packet, uint32_t sequence)
{
	if (ReadU32(packet.data()) != kTimingReplyMagic || ReadU32(packet.data() + 4) != sequence)
		return false;
	const uint32_t status = ReadU32(packet.data() + 8);
	if (status & kTimingUnavailable)
		return status == kTimingUnavailable;
	if (status & ~(kMissingMask | kBusy | kInvalid | kStopped | kCanError))
		return false;
	for (unsigned i = 0; i < static_cast<unsigned>(TimePoint::kCount); ++i) {
		const auto time = GetTime(packet, static_cast<TimePoint>(i));
		if (time != kUnsetTime && time > 1000000)
			return false;
	}
	return true;
}
} // namespace bridge_wire
