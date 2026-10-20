/* SPDX-License-Identifier: Apache-2.0 */
#include "merged_control.hpp"
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
	auto capability = MergedCapabilityQuery(99);
	CHECK(!SupportsMergedControl(capability));
	WriteU32(capability.data(), kMergedCapabilityReply);
	WriteU32(capability.data() + 8, kMergedControlCapability);
	WriteU32(capability.data() + 12, 64);
	WriteU32(capability.data() + 16, 64);
	CHECK(!SupportsMergedControl(capability));
	WriteU32(capability.data() + 12, 128);
	CHECK(SupportsMergedControl(capability));
	Packet groups[2]{};
	auto left = StreamCommand(64, 0, 0), right = StreamCommand(65, 0, 1);
	for (auto *p : {&left, &right}) {
		WriteU32(p->data(), kControlCommand);
		WriteU32(p->data() + 12, 7);
		for (unsigned i = 0; i < 3; ++i)
			WriteU32(p->data() + FrameOffset(i) + 12, 0xfffffff0U + i);
	}
	const auto valid = MergeControl(left, right);
	CHECK(SplitMergedControl(valid, groups) && groups[0] == left && groups[1] == right);
	auto bad = valid;
	bad[64 + FrameOffset(2) + 11] ^= 1;      // nonzero torque on the SECOND bus
	CHECK(!SplitMergedControl(bad, groups)); // reject whole transfer before any CAN submission
	CHECK(!SplitMergedControl(MergeControl(right, left), groups));
	CHECK(!SplitMergedControl(MergeControl(left, left), groups));
	WriteU32(right.data() + 4, 67);
	CHECK(!SplitMergedControl(MergeControl(left, right), groups));
	WriteU32(right.data() + 4, 65);
	WriteU32(right.data() + 12, 8);
	CHECK(!SplitMergedControl(MergeControl(left, right), groups));
	std::cout << "Merged command layout, zero-only validation, sequence and bus order passed\n";
}
