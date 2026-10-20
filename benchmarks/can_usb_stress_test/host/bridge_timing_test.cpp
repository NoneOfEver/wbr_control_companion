/* SPDX-License-Identifier: Apache-2.0 */
#include "bridge_timing.hpp"
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
	CHECK(SampleTiming(0) && SampleTiming(1) && !SampleTiming(2) && !SampleTiming(63));
	CHECK(SampleTiming(64) && SampleTiming(65));
	CHECK(TimingIndex(64) == 2 && TimingIndex(65) == 3);
	CHECK(TimingIndex(131009) == kTimingCapacity - 1 && TimingIndex(131072) == kTimingCapacity);
	auto packet = NewTiming(65, 0xfffffff0U);
	CHECK(GetTime(packet, TimePoint::kInSubmit) == kUnsetTime);
	MarkTime(packet, TimePoint::kDequeued, 1);
	CHECK(GetTime(packet, TimePoint::kDequeued) == 17);
	MarkRange(packet, TimePoint::kFirstDriver, TimePoint::kLastDriver, 2);
	MarkRange(packet, TimePoint::kFirstDriver, TimePoint::kLastDriver, 8);
	CHECK(GetTime(packet, TimePoint::kFirstDriver) == 18);
	CHECK(GetTime(packet, TimePoint::kLastDriver) == 24);
	CHECK(ValidTiming(packet, 65) && !ValidTiming(packet, 64));
	WriteU32(packet.data() + 8, kTimingUnavailable);
	CHECK(ValidTiming(packet, 65));
	WriteU32(packet.data() + 8, kTimingUnavailable | kBusy);
	CHECK(!ValidTiming(packet, 65));
	WriteU32(packet.data() + 8, 0);
	SetTime(packet, TimePoint::kInSubmit, 1000001);
	CHECK(!ValidTiming(packet, 65));
	const auto query = TimingQuery(65);
	CHECK(ReadU32(query.data()) == kTimingQueryMagic && ReadU32(query.data() + 4) == 65);
	CHECK(!ValidTiming(query, 65));
	CHECK(sizeof(packet) == 64);
	std::cout << "Sample selection, trace wire contract, missing fields and timestamp wrap "
		     "passed\n";
}
