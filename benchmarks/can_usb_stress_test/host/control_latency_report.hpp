/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "control_latency.hpp"
#include <wbr/realtime_loop.hpp>
#include <array>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>

namespace stress_host
{
// Reporting runs after capture, at normal priority. Match the exact CAN receive
// timestamp, never the next arbitrary USB packet. Unexported feedback is counted
// as unmatched instead of substituting a newer sample and understating latency.
template <class Commands, class Feedback>
bool WriteControlLatencyReport(const Commands &commands, const Feedback &feedback,
			       const std::string &result_path)
{
	using namespace bridge_wire;
	namespace rt = wbr::realtime;
	constexpr const char *names[] = {"left_wheel",  "left_joint_b",  "left_joint_d",
					 "right_wheel", "right_joint_b", "right_joint_d"};
	auto key = [](unsigned motor, uint32_t us) { return (uint64_t(motor) << 32) | us; };
	std::unordered_map<uint64_t, uint64_t> arrivals;
	for (const auto &r : commands) {
		if (!r.has_timing || ReadU32(r.timing.data() + 8) == kControlUnavailable)
			continue;
		for (unsigned i = 0; i < 3; ++i)
			if (HasControlTime(r.timing, i, ControlPoint::NextFeedback))
				arrivals.emplace(
					key(r.group * 3 + i,
					    ControlTime(r.timing, i, ControlPoint::NextFeedback)),
					0);
	}
	for (const auto &f : feedback) {
		auto found = arrivals.find(key(f.motor, f.mcu_rx_us));
		if (found != arrivals.end() && !found->second)
			found->second = f.host_ns;
	}
	const auto directory = std::filesystem::path(result_path).parent_path();
	std::ofstream csv(directory / "control_latency.csv");
	csv << "sequence,cycle,motor,origin_valid,mcu_origin_rx_us,mcu_out_us,mcu_driver_us,"
	       "mcu_batch_done_us,mcu_next_feedback_us,host_submit_ns,origin_host_receive_ns,"
	       "next_feedback_host_receive_ns,sensor_to_batch_done_us,host_hold_us,"
	       "command_to_observed_feedback_us\n";
	struct Metrics {
		std::vector<int64_t> loop, hold, observation, out_to_done, done_to_feedback;
		unsigned ready = 0, missing_tx = 0, missing_feedback = 0, unexported = 0;
	} metrics[6];
	struct Cycle {
		unsigned mask = 0;
		int64_t maximum_ns = 0;
		uint32_t oldest_origin_us = 0, last_done_us = 0;
	};
	std::map<uint64_t, Cycle> cycles;
	unsigned expected = 0, received = 0, unavailable = 0, invalid = 0;
	unsigned wait_expired = 0, reused_wheel_cycles = 0;
	std::vector<int64_t> gate_waits, command_intervals;
	std::array<uint32_t, 2> previous_wheels{};
	std::array<bool, 2> previous_valid{};
	uint64_t last_submit = 0;
	std::array<bool, 2> had_command{};
	for (const auto &r : commands) {
		if (r.group == 0) {
			gate_waits.push_back(r.feedback_wait_ns);
			wait_expired += r.feedback_wait_expired;
			if (last_submit)
				command_intervals.push_back(r.submitted_ns - last_submit);
			last_submit = r.submitted_ns;
		}
		if (had_command[r.group] &&
		    ((r.origin_mask & 1U) == 0 ||
		     (previous_valid[r.group] && previous_wheels[r.group] == r.origin_us[0])))
			++reused_wheel_cycles;
		previous_wheels[r.group] = r.origin_us[0];
		previous_valid[r.group] = r.origin_mask & 1U;
		had_command[r.group] = true;
	}
	for (const auto &r : commands) {
		if (!SampleTiming(r.sequence))
			continue;
		++expected;
		if (!r.has_timing) {
			++unavailable;
			continue;
		}
		++received;
		const auto &p = r.timing;
		if (ReadU32(p.data() + 8) == kControlUnavailable) {
			++unavailable;
			continue;
		}
		if (!ValidControlReply(p, r.sequence) ||
		    ReadU32(p.data() + 12) != ReadU32(r.reply.data() + 12)) {
			++invalid;
			continue;
		}
		for (unsigned i = 0; i < 3; ++i) {
			const unsigned motor = r.group * 3 + i;
			auto &m = metrics[motor];
			const bool origin = HasControlTime(p, i, ControlPoint::Origin);
			if (origin != bool(r.origin_mask & (1U << i)) ||
			    (origin && ControlTime(p, i, ControlPoint::Origin) != r.origin_us[i])) {
				++invalid;
				continue;
			}
			const uint32_t source = ControlTime(p, i, ControlPoint::Origin),
				       out = ReadU32(p.data() + 12),
				       driver = ControlTime(p, i, ControlPoint::Driver),
				       done = ControlTime(p, i, ControlPoint::BatchDone),
				       next = ControlTime(p, i, ControlPoint::NextFeedback);
			if (origin)
				++m.ready;
			if (!HasControlTime(p, i, ControlPoint::BatchDone) ||
			    !HasControlTime(p, i, ControlPoint::Driver)) {
				++m.missing_tx;
				continue;
			}
			// Reject inconsistent traces explicitly. A long valid delay remains
			// in the distribution; this is a half-range wrap/order check only.
			if (int32_t(driver - out) < 0 || int32_t(done - driver) < 0 ||
			    (origin && (int32_t(out - source) < 0 || !r.origin_host_ns[i] ||
					r.submitted_ns < r.origin_host_ns[i]))) {
				++invalid;
				continue;
			}
			const uint32_t loop_us = origin ? uint32_t(done - source) : 0;
			const uint64_t hold_ns = origin ? r.submitted_ns - r.origin_host_ns[i] : 0;
			if (origin) {
				m.loop.push_back(int64_t(loop_us) * 1000);
				m.hold.push_back(hold_ns);
				auto &c = cycles[r.cycle];
				if (!c.mask || int32_t(source - c.oldest_origin_us) < 0)
					c.oldest_origin_us = source;
				if (!c.mask || int32_t(done - c.last_done_us) > 0)
					c.last_done_us = done;
				c.mask |= 1U << motor;
				c.maximum_ns = std::max(c.maximum_ns, int64_t(loop_us) * 1000);
			}
			m.out_to_done.push_back(int64_t(uint32_t(done - out)) * 1000);
			uint64_t arrival = 0;
			if (!HasControlTime(p, i, ControlPoint::NextFeedback))
				++m.missing_feedback;
			else if (int32_t(next - done) < 0)
				++invalid;
			else {
				m.done_to_feedback.push_back(int64_t(uint32_t(next - done)) * 1000);
				arrival = arrivals.at(key(motor, next));
				if (!arrival)
					++m.unexported;
				else if (arrival < r.submitted_ns)
					++invalid;
				else
					m.observation.push_back(arrival - r.submitted_ns);
			}
			csv << r.sequence << ',' << r.cycle << ',' << names[motor] << ',' << origin
			    << ',' << source << ',' << out << ',' << driver << ',' << done << ','
			    << next << ',' << r.submitted_ns << ',' << r.origin_host_ns[i] << ','
			    << arrival << ',' << loop_us << ',' << hold_ns / 1000.0 << ',';
			if (arrival >= r.submitted_ns)
				csv << (arrival - r.submitted_ns) / 1000.0;
			csv << '\n';
		}
	}
	std::ofstream json(directory / "control_latency.json");
	json << "{\n  \"definition\": \"CAN RX -> USB IN -> next 1kHz zero-control tick -> USB OUT "
		"-> CAN batch completion; excludes motor execution and sensor sampling\",\n"
	     << "  \"command_observation\": \"host submit -> first CAN feedback timestamp after "
		"batch completion -> host receive; temporal observation, not causal execution "
		"RTT\",\n"
	     << "  \"expected_sample_packets\": " << expected
	     << ",\n  \"received_sample_packets\": " << received
	     << ",\n  \"unavailable_packets\": " << unavailable
	     << ",\n  \"invalid_traces\": " << invalid << ",\n  \"motors\": [\n";
	bool pass = expected > 0 && received == expected && !unavailable && !invalid;
	auto write_metric = [&](const char *name, const std::vector<int64_t> &values) {
		const auto s = rt::Summarize(values);
		unsigned over = 0;
		for (auto v : values)
			over += v > 1000000;
		json << '"' << name << "\": {\"n\": " << values.size()
		     << ", \"p50_us\": " << s.p50_ns / 1000.0
		     << ", \"p99_us\": " << s.p99_ns / 1000.0
		     << ", \"max_us\": " << s.maximum_ns / 1000.0 << ", \"over_1ms\": " << over
		     << '}';
		return s;
	};
	std::cout << "Control latency sample packets=" << received << '/' << expected
		  << " unavailable=" << unavailable << " invalid=" << invalid << '\n';
	for (unsigned motor = 0; motor < 6; ++motor) {
		const auto &m = metrics[motor];
		pass &= !m.loop.empty() && m.loop.size() == m.ready && !m.missing_tx;
		json << "    {\"name\": \"" << names[motor] << "\", \"origin_ready\": " << m.ready
		     << ", \"missing_tx\": " << m.missing_tx
		     << ", \"missing_post_tx_feedback\": " << m.missing_feedback
		     << ", \"unexported_post_tx_feedback\": " << m.unexported << ", ";
		auto loop = write_metric("sensor_to_batch_done", m.loop);
		json << ", ";
		write_metric("host_feedback_to_submit", m.hold);
		json << ", ";
		auto obs = write_metric("command_to_observed_feedback", m.observation);
		json << ", ";
		write_metric("mcu_out_to_batch_done", m.out_to_done);
		json << ", ";
		write_metric("batch_done_to_next_feedback", m.done_to_feedback);
		json << '}' << (motor == 5 ? "\n" : ",\n");
		std::cout << names[motor]
			  << " sensor->CAN batch done p50/p99/max=" << loop.p50_ns / 1000.0 << '/'
			  << loop.p99_ns / 1000.0 << '/' << loop.maximum_ns / 1000.0
			  << " us n=" << m.loop.size()
			  << " host submit->observed feedback=" << obs.p50_ns / 1000.0 << '/'
			  << obs.p99_ns / 1000.0 << '/' << obs.maximum_ns / 1000.0
			  << " us n=" << m.observation.size()
			  << " missing-TX/feedback/unexported=" << m.missing_tx << '/'
			  << m.missing_feedback << '/' << m.unexported << '\n';
	}
	std::vector<int64_t> six_motor, oldest_to_all;
	for (const auto &[cycle, c] : cycles) {
		(void)cycle;
		if (c.mask == 63) {
			six_motor.push_back(c.maximum_ns);
			oldest_to_all.push_back(
				int64_t(uint32_t(c.last_done_us - c.oldest_origin_us)) * 1000);
		}
	}
	json << "  ],\n  ";
	auto all = write_metric("six_motor_cycle_worst_sensor_to_batch_done", six_motor);
	json << ",\n  ";
	auto state_age = write_metric("oldest_used_feedback_to_all_six_batch_done", oldest_to_all);
	json << ",\n  ";
	auto wait_summary = write_metric("intentional_feedback_wait", gate_waits);
	json << ",\n  ";
	auto interval_summary = write_metric("actual_control_send_interval", command_intervals);
	const int64_t minimum_interval_ns =
		command_intervals.empty()
			? 0
			: *std::min_element(command_intervals.begin(), command_intervals.end());
	json << ",\n  \"actual_control_send_interval_min_us\": " << minimum_interval_ns / 1000.0;
	json << ",\n  \"feedback_wait_expired_cycles\": " << wait_expired
	     << ",\n  \"reused_or_missing_wheel_inputs\": " << reused_wheel_cycles;
	pass &= !six_motor.empty();
	json << ",\n  \"measurement_pass\": " << (pass ? "true" : "false") << "\n}\n";
	std::cout << "Six-motor cycle worst sensor->CAN batch done p50/p99/max="
		  << all.p50_ns / 1000.0 << '/' << all.p99_ns / 1000.0 << '/'
		  << all.maximum_ns / 1000.0 << " us n=" << six_motor.size()
		  << "; same MCU clock; motor execution excluded\n";
	std::cout << "Oldest used feedback->all six CAN batches done p50/p99/max="
		  << state_age.p50_ns / 1000.0 << '/' << state_age.p99_ns / 1000.0 << '/'
		  << state_age.maximum_ns / 1000.0 << " us n=" << oldest_to_all.size() << '\n';
	std::cout << "Feedback gate wait p50/p99/max=" << wait_summary.p50_ns / 1000.0 << '/'
		  << wait_summary.p99_ns / 1000.0 << '/' << wait_summary.maximum_ns / 1000.0
		  << " us timeout cycles=" << wait_expired
		  << " reused/missing wheel inputs=" << reused_wheel_cycles
		  << "; actual send interval p50/p99/max=" << interval_summary.p50_ns / 1000.0
		  << '/' << interval_summary.p99_ns / 1000.0 << '/'
		  << interval_summary.maximum_ns / 1000.0
		  << " us min=" << minimum_interval_ns / 1000.0 << " us\n";
	csv.flush();
	json.flush();
	return pass && bool(csv) && bool(json);
}
} // namespace stress_host
