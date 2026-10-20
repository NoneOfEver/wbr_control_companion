/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "bridge_packet.hpp"

namespace bridge_wire
{
// One owner per bus. Threads protect this state against the ISR; this object
// itself has no RTOS dependency so its matching/timeout rules can be tested.
struct FeedbackObservation {
	bool active = false;
	Packet reply{};
	uint32_t receive_us = 0;
	uint8_t tx_mask = 0;
	uint8_t feedback_mask = 0;

	bool IsCurrent(uint32_t timestamp_us) const
	{
		return active && static_cast<int32_t>(timestamp_us - receive_us) >= 0;
	}
	void OnTx(unsigned item, uint32_t timestamp_us)
	{
		if (item < 3 && IsCurrent(timestamp_us))
			tx_mask |= 1U << item;
	}
	bool OnFeedback(unsigned item, uint32_t timestamp_us, uint8_t bus, uint16_t id, uint8_t dlc,
			const uint8_t *data)
	{
		if (item >= 3 || dlc != 8 || !IsCurrent(timestamp_us))
			return false;
		const uint8_t bit = 1U << item;
		if (!(tx_mask & bit) || (feedback_mask & bit))
			return false;
		auto *frame = reply.data() + FrameOffset(item);
		SetAddress(frame, bus, id, dlc);
		for (unsigned i = 0; i < 8; ++i)
			frame[4 + i] = data[i];
		WriteU32(frame + 12, timestamp_us - receive_us);
		feedback_mask |= bit;
		return true;
	}
	bool Finish(uint32_t now_us, uint32_t timeout_us, Packet &packet)
	{
		if (!active || (feedback_mask != 7 && now_us - receive_us < timeout_us))
			return false;
		packet = reply;
		WriteU32(packet.data() + 8, 7 ^ feedback_mask);
		active = false;
		return true;
	}
};
} // namespace bridge_wire
