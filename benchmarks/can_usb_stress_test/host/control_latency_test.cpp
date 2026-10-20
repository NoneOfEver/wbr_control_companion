/* SPDX-License-Identifier: Apache-2.0 */
#include "control_latency.hpp"
#include <iostream>
#define CHECK(c)                                                                                   \
	do {                                                                                       \
		if (!(c)) {                                                                        \
			std::cerr << "line " << __LINE__ << '\n';                                  \
			return 1;                                                                  \
		}                                                                                  \
	} while (false)
int main()
{
	using namespace bridge_wire;
	// Static storage matches firmware: recorder is deliberately not put on a small stack.
	static ControlLatencyRecorder recorder;
	auto command = StreamCommand(0, 0, 0);
	WriteU32(command.data(), kControlCommand);
	WriteU32(command.data() + 12, 7);
	for (unsigned i = 0; i < 3; ++i)
		WriteU32(command.data() + FrameOffset(i) + 12, 0xffffffe0U);
	recorder.Stage(command, 0, 0xfffffff0U);
	for (unsigned i = 0; i < 3; ++i)
		recorder.Driver(i, 0xfffffff8U);
	// Stage a newer command while the old one is in flight. Its metadata must
	// not change the old command's completion or first subsequent feedback.
	auto newer = command;
	WriteU32(newer.data() + 4, 2); // not sampled
	recorder.Stage(newer, 0, 1);
	// Trigger belongs to the in-flight sampled group, even after newer metadata.
	for (unsigned i = 0; i < 3; ++i)
		recorder.HardwareStart(i, 0); // zero after clock wrap is a valid timestamp
	auto hardware = recorder.QueryHardware(0);
	CHECK(ValidHardwareReply(hardware, 0));
	CHECK(ReadU32(hardware.data() + 8) == 7);
	CHECK(ReadU32(hardware.data() + 16) == 0);
	CHECK(uint32_t(ReadU32(hardware.data() + 16) - 0xfffffff8U) == 8);
	CHECK(ReadU32(recorder.QueryHardware(2).data() + 8) == kControlUnavailable);
	for (unsigned i = 0; i < 3; ++i)
		recorder.BatchDone(i, 8);
	recorder.Receive(0, 7); // timestamp precedes completion: reject
	CHECK(!HasControlTime(recorder.Query(0), 0, ControlPoint::NextFeedback));
	for (unsigned i = 0; i < 3; ++i) {
		recorder.Driver(i, 10); // unsampled flight
		recorder.BatchDone(i, 20);
		recorder.Receive(i, 24);
		recorder.Receive(i, 30); // must keep the first feedback, not latest
	}
	auto reply = recorder.Query(0);
	CHECK(ValidControlReply(reply, 0));
	CHECK(!ValidControlReply(reply, 1));
	for (unsigned i = 0; i < 3; ++i) {
		CHECK(HasControlTime(reply, i, ControlPoint::Origin));
		CHECK(HasControlTime(reply, i, ControlPoint::Driver));
		CHECK(HasControlTime(reply, i, ControlPoint::BatchDone));
		CHECK(HasControlTime(reply, i, ControlPoint::NextFeedback));
		CHECK(ControlTime(reply, i, ControlPoint::BatchDone) == 8);
		CHECK(ControlTime(reply, i, ControlPoint::NextFeedback) == 24);
		CHECK(uint32_t(ControlTime(reply, i, ControlPoint::BatchDone) -
			       ControlTime(reply, i, ControlPoint::Origin)) == 40);
	}
	CHECK(ReadU32(recorder.Query(2).data() + 8) == kControlUnavailable);
	CHECK(ReadU32(recorder.Query(64 * kTimingCapacity).data() + 8) == kControlUnavailable);
	// The other bus has independent records, and timestamp zero is valid.
	WriteU32(command.data() + 4, 1);
	WriteU32(command.data() + 12, 1);
	WriteU32(command.data() + FrameOffset(0) + 12, 0);
	recorder.Stage(command, 1, 50);
	recorder.Driver(3, 60);
	recorder.HardwareStart(3, 70);
	CHECK(ReadU32(recorder.QueryHardware(1).data() + 8) == 1);
	CHECK(ReadU32(recorder.QueryHardware(1).data() + 16) == 70);
	CHECK(ReadU32(recorder.QueryHardware(0).data() + 8) == 7);
	recorder.BatchDone(3, 90);
	auto right = recorder.Query(1);
	CHECK(HasControlTime(right, 0, ControlPoint::Origin));
	CHECK(ControlTime(right, 0, ControlPoint::Origin) == 0);
	CHECK(!HasControlTime(right, 1, ControlPoint::Origin));       // warmup: no fabricated state
	CHECK(!HasControlTime(right, 0, ControlPoint::NextFeedback)); // missing remains explicit
	CHECK(ControlTime(recorder.Query(0), 0, ControlPoint::BatchDone) == 8);
	WriteU32(right.data() + 8, 1U << 7);
	CHECK(!ValidControlReply(right, 1));
	std::cout << "In-flight identity, missing feedback, independent buses and timestamp wrap "
		     "passed\n";
}
