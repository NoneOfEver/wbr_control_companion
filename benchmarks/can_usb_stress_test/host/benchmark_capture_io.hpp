/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "benchmark_records.hpp"
#include <filesystem>
#include <fstream>
#include <vector>
namespace stress_host
{
inline bool WriteCaptureCsv(const std::vector<Record> &records,
			    const std::vector<FeedbackRecord> &feedback_records,
			    const std::string &csv_path)
{
	if (csv_path.empty())
		return true;
	std::ofstream commands(csv_path);
	commands << "sequence,cycle,group,submit_ns,out_complete_ns,ack_ns,status,"
		    "mcu_out_us,dequeued_after_out_us,submitted_after_out_us,"
		    "feedback_wait_us,feedback_wait_expired,state_sample_ns\n";
	for (const auto &r : records)
		commands << r.sequence << ',' << r.cycle << ',' << r.group << ',' << r.submitted_ns
			 << ',' << r.out_complete_ns << ',' << r.in_complete_ns << ','
			 << r.bridge_status << ',' << ReadU32(r.reply.data() + 12) << ','
			 << ReadU32(r.reply.data() + 16) << ',' << ReadU32(r.reply.data() + 20)
			 << ',' << r.feedback_wait_ns / 1000.0 << ',' << r.feedback_wait_expired
			 << ',' << r.state_sample_ns << '\n';
	std::ofstream feedback(std::filesystem::path(csv_path).parent_path() / "feedback.csv");
	constexpr const char *names[] = {"left_wheel",  "left_joint_b",  "left_joint_d",
					 "right_wheel", "right_joint_b", "right_joint_d"};
	feedback << "packet_sequence,motor,host_receive_ns,mcu_in_submit_us,mcu_can_rx_us,"
		    "age_at_in_submit_us,can_data_hex\n";
	constexpr char hex[] = "0123456789abcdef";
	for (const auto &r : feedback_records) {
		feedback << r.packet_sequence << ',' << names[r.motor] << ',' << r.host_ns << ','
			 << r.mcu_in_submit_us << ',' << r.mcu_rx_us << ','
			 << uint32_t(r.mcu_in_submit_us - r.mcu_rx_us) << ',';
		for (auto byte : r.data)
			feedback << hex[byte >> 4] << hex[byte & 15];
		feedback << '\n';
	}
	commands.flush();
	feedback.flush();
	return bool(commands) && bool(feedback);
}
} // namespace stress_host
