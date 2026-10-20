/* SPDX-License-Identifier: Apache-2.0 */
#include "bridge_stream.hpp"
#include <iostream>
#define CHECK(condition)                                                                           \
	do {                                                                                       \
		if (!(condition)) {                                                                \
			std::cerr << "line " << __LINE__ << '\n';                                  \
			return 1;                                                                  \
		}                                                                                  \
	} while (false)
int main()
{
	using namespace bridge_wire;
	const auto command = StreamCommand(17, 400, 1);
	CHECK(ReadU32(command.data()) == kStreamCommand && ReadU32(command.data() + 4) == 17);
	CHECK(!ValidAck(command));
	Packet ack{};
	WriteU32(ack.data(), kStreamAck);
	CHECK(ValidAck(ack));
	WriteU32(ack.data() + 8, kBusy);
	CHECK(!ValidAck(ack)); // feedback busy has no place in this mode
	WriteU32(ack.data() + 8, kCanError);
	CHECK(ValidAck(ack));
	FeedbackMailbox mailbox{};
	FeedbackMailbox::Sample a{};
	a.bus = kLayout[0][0].bus;
	a.id = kLayout[0][0].feedback_id;
	a.received_us = 0xfffffff0U;
	a.data[0] = 1;
	mailbox.Put(0, a);
	a.data[0] = 2;
	mailbox.Put(0, a);
	auto packet = mailbox.Take(9);
	WriteU32(packet.data() + 12, 4);
	CHECK(ValidFeedback(packet) && ReadU32(packet.data() + 8) == 1 && packet[20] == 2);
	CHECK(uint32_t(ReadU32(packet.data() + 12) - ReadU32(packet.data() + 28)) == 20);
	CHECK(!mailbox.dirty);
	CHECK(ReadU32(mailbox.Take(10).data() + 8) == 0); // never waits for all three motors
	for (unsigned i = 0; i < 6; ++i) {
		const auto &motor = kLayout[i / 3][i % 3];
		a.bus = motor.bus;
		a.id = motor.feedback_id;
		a.received_us = 100;
		mailbox.Put(i, a);
	}
	auto first = mailbox.Take(11);
	auto second = mailbox.Take(12);
	WriteU32(first.data() + 12, 101);
	WriteU32(second.data() + 12, 102);
	CHECK(ValidFeedback(first) && ValidFeedback(second) && !mailbox.dirty);
	unsigned seen = 0;
	for (const auto *p : {&first, &second})
		for (unsigned i = 0; i < 3; ++i)
			seen |= 1U << FeedbackMotor(p->data() + FrameOffset(i));
	CHECK(seen == 63);
	first[17] = 7;
	CHECK(!ValidFeedback(first));
	const auto stats = StreamStatsQuery(17, 1);
	CHECK(ReadU32(stats.data()) == kStreamStatsQuery && ReadU32(stats.data() + 8) == 1);
	// Final pages 2/3 carry batch histograms, not per-motor counters.
	Packet counters{};
	WriteU32(counters.data(), kStreamStatsReply);
	WriteU32(counters.data() + 4, 19);
	WriteU32(counters.data() + 8, 2);
	WriteU32(counters.data() + 12, 60000000);
	WriteU32(counters.data() + 40, kLayout[0][0].bus);
	CHECK(ValidStreamStats(counters, 19, 2));
	CHECK(!ValidStreamStats(counters, 19, 3));
	CHECK(!ValidStreamStats(counters, 20, 2));
	CHECK(!ValidStreamStats(counters, 19, 4));
	WriteU32(counters.data() + 40, 0);
	CHECK(!ValidStreamStats(counters, 19, 2));
	WriteU32(counters.data() + 8, 0);
	CHECK(ValidStreamStats(counters, 19, 0));
	WriteU32(counters.data() + 12, 0);
	CHECK(!ValidStreamStats(counters, 19, 0));
	std::cout << "Independent ACK/feedback protocols, latest-state replacement, fair bounded "
		     "drain and timestamp wrap passed\n";
}
