/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "control_latency.hpp"
#include <wbr/realtime_loop.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

namespace stress_host
{
// Runs after capture and USB shutdown. Trigger is a software timestamp at the
// TSALL MMIO request, not a per-frame wire timestamp or motor execution time.
template <class Commands>
bool WriteHardwareTimingReport(const Commands &commands, const std::string &result_path,
			       bool supported)
{
	using namespace bridge_wire;
	constexpr const char *names[] = {"left_wheel",  "left_joint_b",  "left_joint_d",
					 "right_wheel", "right_joint_b", "right_joint_d"};
	const auto directory = std::filesystem::path(result_path).parent_path();
	std::ofstream csv(directory / "can_hardware_timing.csv");
	csv << "sequence,cycle,motor,mcu_driver_us,mcu_trigger_us,mcu_batch_done_us,"
	       "driver_to_trigger_us,trigger_to_batch_done_us\n";
	std::vector<int64_t> software[6], bus[6];
	unsigned missing = 0, invalid = 0;
	for (const auto &record : commands) {
		if (!record.has_timing)
			continue;
		for (unsigned i = 0; i < 3; ++i) {
			if (!supported || !record.has_hardware_timing ||
			    !(ReadU32(record.hardware_timing.data() + 8) & (1U << i)) ||
			    ReadU32(record.hardware_timing.data() + 8) == kControlUnavailable ||
			    !HasControlTime(record.timing, i, ControlPoint::Driver) ||
			    !HasControlTime(record.timing, i, ControlPoint::BatchDone)) {
				++missing;
				continue;
			}
			const auto driver = ControlTime(record.timing, i, ControlPoint::Driver);
			const auto trigger = ReadU32(record.hardware_timing.data() + 16 + 4 * i);
			const auto done = ControlTime(record.timing, i, ControlPoint::BatchDone);
			const auto before = int32_t(trigger - driver),
				   after = int32_t(done - trigger);
			if (before < 0 || after < 0) {
				++invalid;
				continue;
			}
			const auto motor = record.group * 3 + i;
			software[motor].push_back(int64_t(before) * 1000);
			bus[motor].push_back(int64_t(after) * 1000);
			csv << record.sequence << ',' << record.cycle << ',' << names[motor] << ','
			    << driver << ',' << trigger << ',' << done << ',' << before << ','
			    << after << '\n';
		}
	}
	std::ofstream json(directory / "can_hardware_timing.json");
	json << "{\"supported\":" << (supported ? "true" : "false") << ",\"missing\":" << missing
	     << ",\"invalid\":" << invalid
	     << ",\"definition\":\"driver entry -> TSALL software trigger -> whole batch "
		"completion callback; not per-frame wire timing\",\"motors\":[";
	for (unsigned motor = 0; motor < 6; ++motor) {
		const auto before = wbr::realtime::Summarize(software[motor]);
		const auto after = wbr::realtime::Summarize(bus[motor]);
		json << (motor ? "," : "") << "{\"name\":\"" << names[motor]
		     << "\",\"n\":" << software[motor].size()
		     << ",\"driver_to_trigger_p50_us\":" << before.p50_ns / 1000.0
		     << ",\"driver_to_trigger_p99_us\":" << before.p99_ns / 1000.0
		     << ",\"trigger_to_batch_done_p50_us\":" << after.p50_ns / 1000.0
		     << ",\"trigger_to_batch_done_p99_us\":" << after.p99_ns / 1000.0 << '}';
		if (supported)
			std::cout << names[motor]
				  << " driver->trigger p50/p99=" << before.p50_ns / 1000.0 << '/'
				  << before.p99_ns / 1000.0
				  << " us; trigger->batch done=" << after.p50_ns / 1000.0 << '/'
				  << after.p99_ns / 1000.0 << " us n=" << software[motor].size()
				  << '\n';
	}
	json << "]}\n";
	std::cout << "CAN hardware timing supported=" << supported << " missing=" << missing
		  << " invalid=" << invalid << std::endl;
	return bool(csv) && bool(json) &&
	       (!supported || (!missing && !invalid && !software[0].empty()));
}
} // namespace stress_host
