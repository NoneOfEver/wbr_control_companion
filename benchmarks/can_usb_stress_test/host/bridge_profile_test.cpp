/* SPDX-License-Identifier: Apache-2.0 */
#include "bridge_profile.hpp"
#include "control_latency.hpp"
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
	unsigned permutations = 0;
	for (unsigned order = 0; order < 6; ++order) {
		unsigned seen = 0, identity = 0;
		for (unsigned rank = 0; rank < 3; ++rank) {
			const unsigned item = kCanOrders[order][rank];
			CHECK(item < 3 && !(seen & (1U << item)));
			seen |= 1U << item;
			identity = identity * 3 + item;
		}
		CHECK(seen == 7 && !(permutations & (1U << identity)));
		permutations |= 1U << identity;
		for (unsigned fast = 0; fast < 2; ++fast) {
			auto reply = ProfileQuery(42, order, fast);
			CHECK(!ValidProfileReply(reply, 42, order, fast));
			WriteU32(reply.data(), kProfileReply);
			CHECK(ValidProfileReply(reply, 42, order, fast));
			WriteU32(reply.data() + 16, 1);
			CHECK(!ValidProfileReply(reply, 42, order, fast));
		}
		// Changing wire send order must never relabel diagnostic motor slots.
		static ControlLatencyRecorder recorder;
		auto command = StreamCommand(0, 0, 0);
		WriteU32(command.data(), kControlCommand);
		recorder.Stage(command, 0, 10);
		for (auto item : kCanOrders[order]) {
			recorder.Driver(item, 20 + item);
			recorder.HardwareStart(item, 30 + item);
			recorder.BatchDone(item, 40 + item);
		}
		for (unsigned item = 0; item < 3; ++item) {
			CHECK(ControlTime(recorder.Query(0), item, ControlPoint::Driver) ==
			      20 + item);
			CHECK(ReadU32(recorder.QueryHardware(0).data() + 16 + 4 * item) ==
			      30 + item);
		}
	}
	CHECK(!ValidProfile(6, 0) && !ValidProfile(0, 2));
	std::cout << "Six unique CAN orders preserve motor identity; rejected profiles stay "
		     "explicit\n";
}
