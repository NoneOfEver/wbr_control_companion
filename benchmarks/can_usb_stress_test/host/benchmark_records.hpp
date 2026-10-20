/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "bridge_packet.hpp"
namespace stress_host
{
struct Record {
	std::uint32_t sequence;
	std::uint64_t submitted_ns;
	std::uint64_t out_complete_ns = 0;
	std::uint64_t in_complete_ns = 0;
	std::int64_t wakeup_late_ns;
	uint64_t cycle = 0;
	unsigned group = 0;
	uint32_t bridge_status = 0;
	Packet reply{};
	Packet timing{};
	bool has_timing = false;
	Packet hardware_timing{};
	bool has_hardware_timing = false;
	uint64_t feedback_wait_ns = 0;
	bool feedback_wait_expired = false;
	uint64_t state_sample_ns = 0;
	uint32_t origin_mask = 0;
	std::array<uint32_t, 3> origin_us{};
	std::array<uint64_t, 3> origin_host_ns{};
};

struct FeedbackRecord {
	uint64_t host_ns;
	uint32_t packet_sequence, motor, mcu_in_submit_us, mcu_rx_us;
	std::array<unsigned char, 8> data{};
};

} // namespace stress_host
