/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_benchmark.hpp"
#include "benchmark_records.hpp"
#include "benchmark_capture_io.hpp"
#include "event_waiter.hpp"
#include "feedback_gate.hpp"
#include "bridge_packet.hpp"
#include "bridge_timing.hpp"
#include "bridge_stream.hpp"
#include "control_latency_report.hpp"
#include <atomic>
#include <cerrno>
#include "echo_packet.hpp"
#include <wbr/realtime_loop.hpp>
#include <libusb.h>

#include <algorithm>
#include <array>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <limits>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace stress_host
{
namespace rt = wbr::realtime;
namespace
{
constexpr unsigned kOutWindow = 8;
constexpr unsigned kInWindow = 4;
constexpr std::uint64_t kPeriodNs = 1000000;
volatile std::sig_atomic_t stop_requested;
std::string IsolatedCpus()
{
	std::ifstream stream("/sys/devices/system/cpu/isolated");
	std::string cpus;
	std::getline(stream, cpus);
	return cpus;
}

} // namespace

class UsbBenchmark::Impl
{
public:
	explicit Impl(const BenchmarkOptions &options) : options_(options)
	{
		// Reserve before mlockall/FIFO. No vector growth or disk writes in the
		// event loop. All transfers and DMA buffers are allocated before Run.
		records_.reserve(static_cast<std::size_t>(options.seconds *
							  (options.can_bridge ? 2000 : 1000)) +
				 32);
		if (options.can_stream)
			feedback_records_.reserve(
				static_cast<std::size_t>(std::min(options.seconds, 65.0) * 20000) +
				32);
	}

	bool Open()
	{
		if (libusb_init(&context_) != 0) {
			return Fail("libusb_init failed");
		}
		device_ = libusb_open_device_with_vid_pid(context_, 0x34b7, 0xffff);
		if (device_ == nullptr) {
			return Fail("USB device not found or permission denied");
		}
		speed_ = libusb_get_device_speed(libusb_get_device(device_));
		int configuration = 0;
		if (libusb_get_configuration(device_, &configuration) != 0) {
			return Fail("Cannot read USB configuration");
		}
		if (configuration != 1 && libusb_set_configuration(device_, 1) != 0) {
			return Fail("Cannot set USB configuration");
		}
		if (libusb_claim_interface(device_, 0) != 0) {
			return Fail("Cannot claim USB interface");
		}
		claimed_ = true;
		if (!ReadEndpointTransport())
			return false;
		for (auto &slot : out_) {
			slot.owner = this;
			slot.transfer = libusb_alloc_transfer(0);
			if (slot.transfer == nullptr) {
				return Fail("Cannot allocate OUT transfer");
			}
		}
		for (auto &input : in_) {
			input.owner = this;
			input.transfer = libusb_alloc_transfer(0);
			if (input.transfer == nullptr) {
				return Fail("Cannot allocate IN transfer");
			}
			FillTransfer(input.transfer, device_, 0x81, input.packet.data(),
				     input.packet.size(), OnIn, &input, 250);
		}
		if (libusb_pollfds_handle_timeouts(context_) != 1) {
			return Fail("Linux libusb timerfd support is required");
		}
		const libusb_pollfd **usb_sources = libusb_get_pollfds(context_);
		if (usb_sources == nullptr) {
			return Fail("Cannot get USB poll descriptors");
		}
		std::vector<pollfd> descriptors;
		for (std::size_t index = 0; usb_sources[index] != nullptr; ++index) {
			descriptors.push_back(
				{usb_sources[index]->fd, usb_sources[index]->events, 0});
		}
		libusb_free_pollfds(usb_sources);
		if (!event_waiter_.Prepare(std::move(descriptors))) {
			return Fail("Cannot prepare high-resolution USB event wait");
		}
		// This session never opens another device. Abort if hotplug changes
		// its descriptor set instead of polling stale descriptors silently.
		libusb_set_pollfd_notifiers(context_, OnPollfdAdded, OnPollfdRemoved, this);
		if (options_.can_bridge)
			std::cout << "CAN OUT order="
				  << (options_.right_first ? "right-left" : "left-right")
				  << std::endl;
		std::cout << "USB speed="
			  << (speed_ == LIBUSB_SPEED_HIGH ? 480
							  : (speed_ == LIBUSB_SPEED_FULL ? 12 : 0))
			  << " Mbps; collecting " << options_.seconds
			  << " s, control target=1000 Hz, OUT packets/cycle="
			  << (options_.can_bridge ? 2 : 1) << ", IN preposted=" << kInWindow
			  << "; summary follows after collection" << std::endl;
		return true;
	}

	// Resolve the actual descriptors before allocating/submitting transfers. A
	// requested A/B mode must match the flashed firmware, rather than relabel it.
	bool ReadEndpointTransport()
	{
		libusb_config_descriptor *config = nullptr;
		if (libusb_get_active_config_descriptor(libusb_get_device(device_), &config) != 0)
			return Fail("Cannot read active endpoint descriptors");
		int in_type = -1, out_type = -1;
		for (unsigned i = 0; i < config->bNumInterfaces; ++i) {
			const auto &interface = config->interface[i];
			for (int a = 0; a < interface.num_altsetting; ++a) {
				const auto &alt = interface.altsetting[a];
				if (alt.bInterfaceNumber != 0 || alt.bAlternateSetting != 0)
					continue;
				for (unsigned e = 0; e < alt.bNumEndpoints; ++e) {
					const auto &ep = alt.endpoint[e];
					const int type =
						ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;
					if (ep.bEndpointAddress == 0x81) {
						in_type = type;
						in_mps_ = ep.wMaxPacketSize;
						in_interval_ = ep.bInterval;
					}
					if (ep.bEndpointAddress == 0x01) {
						out_type = type;
						out_mps_ = ep.wMaxPacketSize;
						out_interval_ = ep.bInterval;
					}
				}
			}
		}
		libusb_free_config_descriptor(config);
		if (in_type != out_type || (in_type != LIBUSB_TRANSFER_TYPE_BULK &&
					    in_type != LIBUSB_TRANSFER_TYPE_INTERRUPT))
			return Fail("Expected matching Bulk or Interrupt IN/OUT endpoints");
		bulk_ = in_type == LIBUSB_TRANSFER_TYPE_BULK;
		if (options_.usb_transport != "auto" && options_.usb_transport != TransportName())
			return Fail("USB transport does not match requested mode; flash the "
				    "matching firmware");
		if (options_.can_stream && speed_ != LIBUSB_SPEED_HIGH)
			return Fail("Bulk/Interrupt stream comparison requires High-Speed USB");
		std::cout << "USB transport=" << TransportName()
			  << " packet bytes=64 endpoint MPS OUT/IN=" << out_mps_ << "/" << in_mps_
			  << " interval OUT/IN=" << out_interval_ << "/" << in_interval_
			  << std::endl;
		return true;
	}

	const char *TransportName() const
	{
		return bulk_ ? "bulk" : "interrupt";
	}

	void FillTransfer(libusb_transfer *transfer, libusb_device_handle *device,
			  unsigned char endpoint, unsigned char *buffer, int length,
			  libusb_transfer_cb_fn callback, void *user, unsigned timeout)
	{
		if (bulk_)
			libusb_fill_bulk_transfer(transfer, device, endpoint, buffer, length,
						  callback, user, timeout);
		else
			libusb_fill_interrupt_transfer(transfer, device, endpoint, buffer, length,
						       callback, user, timeout);
	}

	void Run()
	{
		start_ns_ = next_release_ns_ = rt::MonotonicTimeNs();
		const auto end_of_send =
			start_ns_ + static_cast<std::uint64_t>(options_.seconds * 1e9);
		if (!event_waiter_.Arm(start_ns_ + kPeriodNs, kPeriodNs)) {
			Fail("Cannot arm absolute command timer");
		}
		for (auto &input : in_) {
			SubmitIn(input);
		}
		while (!failed_ && !stop_requested) {
			if (poll_sources_changed_.load()) {
				Fail("USB poll descriptor set changed; restart the session");
				break;
			}
			const auto now = rt::MonotonicTimeNs();
			if (!sending_done_ && now >= end_of_send) {
				sending_done_ = true;
				sender_finished_ns_ = now;
			}
			if (!sending_done_ && now >= next_release_ns_) {
				const bool opening = !feedback_gate_.Active();
				if (opening) {
					const auto late_periods =
						(now - next_release_ns_) / kPeriodNs;
					skipped_slots_ += late_periods;
					next_release_ns_ += late_periods * kPeriodNs;
					feedback_gate_.Begin(next_release_ns_, now,
							     options_.feedback_grace_us);
				}
				const std::array<uint32_t, 2> wheel_rx{latest_feedback_us_[0],
								       latest_feedback_us_[3]};
				const std::array<bool, 2> wheel_valid{latest_feedback_valid_[0],
								      latest_feedback_valid_[3]};
				const std::array<uint64_t, 2> wheel_host_ns{
					latest_feedback_host_ns_[0], latest_feedback_host_ns_[3]};
				const auto decision = feedback_gate_.Evaluate(
					now, wheel_rx, wheel_valid, wheel_host_ns);
				if (decision.send) {
					const auto first_record = records_.size();
					for (unsigned packet = 0;
					     packet < (options_.can_bridge ? 2U : 1U); ++packet) {
						const unsigned group =
							options_.right_first ? 1U - packet : packet;
						SubmitOut(now, feedback_gate_.WakeupLateNs(),
							  group);
					}
					for (auto index = first_record; index < records_.size();
					     ++index) {
						records_[index].feedback_wait_ns =
							decision.waited_ns;
						records_[index].feedback_wait_expired =
							options_.feedback_grace_us &&
							decision.expired;
					}
					feedback_gate_.Commit(wheel_rx, wheel_valid);
					++cycle_sequence_;
					next_release_ns_ += kPeriodNs;
					if (options_.feedback_grace_us &&
					    !event_waiter_.Arm(next_release_ns_, kPeriodNs))
						Fail("Cannot restore absolute command timer after "
						     "feedback wait");
				} else if (opening &&
					   !event_waiter_.Arm(feedback_gate_.DeadlineNs(), 0)) {
					Fail("Cannot arm bounded feedback wait deadline");
				}
			}
			if (sending_done_ && active_slots_ == 0) {
				break;
			}
			if (sending_done_ && now - sender_finished_ns_ > 1000000000ULL) {
				Fail("Outstanding replies did not drain");
				break;
			}
			// timerfd and USB readiness share one poll. libusb only drains
			// ready events with a zero timeout; it does not pace the 1 kHz loop.
			// Hold its event lock across polling as required by libusb's API.
			libusb_lock_events(context_);
			const auto wake = event_waiter_.Wait();
			timeval nonblocking{};
			const int rc = libusb_handle_events_locked(context_, &nonblocking);
			libusb_unlock_events(context_);
			if (wake.error != 0 && wake.error != -EINTR) {
				Fail("High-resolution event wait failed");
			}
			if (rc != 0 && rc != LIBUSB_ERROR_INTERRUPTED) {
				Fail("USB event handling failed");
			}
		}
		if (sender_finished_ns_ == 0) {
			sender_finished_ns_ = rt::MonotonicTimeNs();
		}
		end_ns_ = rt::MonotonicTimeNs();
		if (stop_requested) {
			Fail("Interrupted by user");
		}
		measurement_failed_ = failed_;
		measurement_outstanding_ = active_slots_;
		if (options_.can_stream && !failed_)
			FetchStreamStats();
		if (options_.control_latency && !failed_)
			DumpTimings();
		else if (!options_.can_stream && options_.can_bridge && !failed_)
			DumpTimings();
		Close();
	}

	bool WriteResults()
	{
		if (options_.can_stream)
			return WriteStreamResults();
		std::vector<std::int64_t> rtts, wakeups, out_times, return_waits;
		for (const auto &record : records_) {
			wakeups.push_back(record.wakeup_late_ns);
			if (record.in_complete_ns != 0 && record.bridge_status == 0) {
				rtts.push_back(record.in_complete_ns - record.submitted_ns);
			}
			if (record.out_complete_ns != 0) {
				out_times.push_back(record.out_complete_ns - record.submitted_ns);
			}
			if (record.in_complete_ns >= record.out_complete_ns &&
			    record.out_complete_ns != 0) {
				return_waits.push_back(record.in_complete_ns -
						       record.out_complete_ns);
			}
		}
		const auto rtt = rt::Summarize(rtts);
		const auto wake = rt::Summarize(wakeups);
		const auto out_time = rt::Summarize(out_times);
		const auto return_wait = rt::Summarize(return_waits);
		const double elapsed = (end_ns_ - start_ns_) / 1e9;
		const double send_elapsed = (sender_finished_ns_ - start_ns_) / 1e9;
		const double rate = elapsed > 0 ? received_ / elapsed : 0;
		unsigned bridge_failures = 0, busy_replies = 0, missing_replies = 0,
			 stopped_replies = 0, invalid_replies = 0;
		std::vector<std::int64_t> cycle_rtts, device_waits;
		std::array<std::vector<std::int64_t>, 2> bus_rtts;
		uint64_t successful_cycles = 0;
		for (const auto &record : records_) {
			if (record.bridge_status != 0)
				++bridge_failures;
			busy_replies += (record.bridge_status & bridge_wire::kBusy) != 0;
			missing_replies += (record.bridge_status & bridge_wire::kMissingMask) != 0;
			stopped_replies += (record.bridge_status & bridge_wire::kStopped) != 0;
			invalid_replies += (record.bridge_status & bridge_wire::kInvalid) != 0;
			if (!record.in_complete_ns || record.bridge_status != 0)
				continue;
			bus_rtts[record.group].push_back(record.in_complete_ns -
							 record.submitted_ns);
			if (options_.can_bridge) {
				uint32_t latest = 0;
				for (unsigned i = 0; i < 3; ++i)
					latest = std::max(
						latest, ReadU32(record.reply.data() +
								bridge_wire::FrameOffset(i) + 12));
				device_waits.push_back(uint64_t(latest) * 1000);
			}
		}
		if (options_.can_bridge) {
			for (std::size_t i = 1; i < records_.size(); ++i) {
				const auto &a = records_[i - 1], &b = records_[i];
				if (a.cycle != b.cycle || a.group != 0 || b.group != 1 ||
				    a.bridge_status || b.bridge_status || !a.in_complete_ns ||
				    !b.in_complete_ns)
					continue;
				cycle_rtts.push_back(std::max(a.in_complete_ns, b.in_complete_ns) -
						     std::min(a.submitted_ns, b.submitted_ns));
				++successful_cycles;
			}
		}
		const auto cycle_rtt = rt::Summarize(cycle_rtts);
		const auto over_budget = std::count_if(cycle_rtts.begin(), cycle_rtts.end(),
						       [](auto value) { return value > 1000000; });
		const auto device_wait = rt::Summarize(device_waits);
		const double cycle_rate = elapsed > 0 ? successful_cycles / elapsed : 0;
		const bool transport_pass = !measurement_failed_ && received_ == submitted_ &&
					    out_completed_ == submitted_ && active_slots_ == 0 &&
					    rate >= (options_.can_bridge ? 1980 : 990);
		const bool pass = transport_pass && bridge_failures == 0 &&
				  (!options_.can_bridge || cycle_rate >= 990);
		using bridge_wire::TimePoint;
		struct Segment {
			const char *name;
			TimePoint start;
			TimePoint end;
			bool from_out;
		};
		const Segment segments[] = {
			{"out_to_dequeue", TimePoint::kDequeued, TimePoint::kDequeued, true},
			{"dequeue_to_first_submit", TimePoint::kDequeued, TimePoint::kFirstSubmit,
			 false},
			{"first_submit_to_first_driver", TimePoint::kFirstSubmit,
			 TimePoint::kFirstDriver, false},
			{"last_submit_to_last_driver", TimePoint::kLastSubmit,
			 TimePoint::kLastDriver, false},
			{"first_driver_to_first_tx_complete", TimePoint::kFirstDriver,
			 TimePoint::kFirstTxComplete, false},
			{"last_driver_to_last_tx_complete", TimePoint::kLastDriver,
			 TimePoint::kLastTxComplete, false},
			{"last_tx_complete_to_last_feedback", TimePoint::kLastTxComplete,
			 TimePoint::kLastFeedback, false},
			{"last_feedback_to_collected", TimePoint::kLastFeedback,
			 TimePoint::kCollected, false},
			{"collected_to_queued", TimePoint::kCollected, TimePoint::kQueued, false},
			{"queued_to_in_submit", TimePoint::kQueued, TimePoint::kInSubmit, false},
			{"out_to_in_submit", TimePoint::kInSubmit, TimePoint::kInSubmit, true}};
		std::array<std::vector<int64_t>, sizeof(segments) / sizeof(segments[0])>
			segment_values;
		unsigned expected_timings = 0, unavailable_timings = 0, valid_timings = 0,
			 incomplete_timings = 0;
		for (const auto &record : records_) {
			if (!options_.can_bridge || !bridge_wire::SampleTiming(record.sequence))
				continue;
			++expected_timings;
			if (!record.has_timing ||
			    (ReadU32(record.timing.data() + 8) & bridge_wire::kTimingUnavailable)) {
				++unavailable_timings;
				continue;
			}
			if (record.bridge_status || ReadU32(record.timing.data() + 8))
				continue;
			bool complete = true;
			for (unsigned i = 0; i < static_cast<unsigned>(TimePoint::kCount); ++i)
				complete &= bridge_wire::GetTime(record.timing,
								 static_cast<TimePoint>(i)) !=
					    bridge_wire::kUnsetTime;
			if (!complete) {
				++incomplete_timings;
				continue;
			}
			++valid_timings;
			for (unsigned i = 0; i < segment_values.size(); ++i) {
				const auto start =
					segments[i].from_out
						? 0
						: bridge_wire::GetTime(record.timing,
								       segments[i].start);
				const auto end =
					bridge_wire::GetTime(record.timing, segments[i].end);
				if (start != bridge_wire::kUnsetTime &&
				    end != bridge_wire::kUnsetTime && end >= start)
					segment_values[i].push_back(int64_t(end - start) * 1000);
			}
		}
		const bool timing_pass =
			!options_.can_bridge ||
			(expected_timings > 0 && timing_requested_ == expected_timings &&
			 timing_received_ == expected_timings && unavailable_timings == 0 &&
			 incomplete_timings == 0);

		std::ofstream json(options_.json_path);
		json << "{\n  \"mode\": \""
		     << (options_.can_bridge ? "native-can-bridge" : "native-duplex") << "\",\n"
		     << "  \"pacing\": \"timerfd-absolute\",\n"
		     << "  \"realtime\": " << (options_.realtime ? "true" : "false") << ",\n"
		     << "  \"cpu\": " << options_.cpu << ",\n"
		     << "  \"fifo_priority\": " << (options_.realtime ? options_.priority : 0)
		     << ",\n"
		     << "  \"memory_locked\": " << (options_.realtime ? "true" : "false") << ",\n"
		     << "  \"isolated_cpus\": \"" << IsolatedCpus() << "\",\n"
		     << "  \"out_order\": \""
		     << (options_.right_first ? "right-left" : "left-right") << "\",\n"
		     << "  \"feedback_grace_us\": " << options_.feedback_grace_us << ",\n"
		     << "  \"usb_transport\": \"" << TransportName() << "\",\n"
		     << "  \"endpoint_mps_out\": " << out_mps_ << ",\n"
		     << "  \"endpoint_mps_in\": " << in_mps_ << ",\n"
		     << "  \"endpoint_interval_out\": " << out_interval_ << ",\n"
		     << "  \"endpoint_interval_in\": " << in_interval_ << ",\n"
		     << "  \"usb_speed_mbps\": "
		     << (speed_ == LIBUSB_SPEED_HIGH ? 480 : (speed_ == LIBUSB_SPEED_FULL ? 12 : 0))
		     << ",\n"
		     << "  \"sent_packets\": " << submitted_ << ",\n"
		     << "  \"completed_out_packets\": " << out_completed_ << ",\n"
		     << "  \"exchanges\": " << received_ << ",\n"
		     << "  \"send_rate_hz\": " << (send_elapsed > 0 ? submitted_ / send_elapsed : 0)
		     << ",\n"
		     << "  \"rate_hz\": " << rate << ",\n"
		     << "  \"elapsed_s\": " << elapsed << ",\n"
		     << "  \"errors\": " << (failed_ ? 1 : 0) << ",\n"
		     << "  \"failure\": \"" << failure_ << "\",\n"
		     << "  \"host_skipped_slots\": " << skipped_slots_ << ",\n"
		     << "  \"peak_in_flight\": " << peak_slots_ << ",\n"
		     << "  \"outstanding_packets\": " << measurement_outstanding_ << ",\n"
		     << "  \"rtt_us_p50\": " << rtt.p50_ns / 1000.0 << ",\n"
		     << "  \"rtt_us_p99\": " << rtt.p99_ns / 1000.0 << ",\n"
		     << "  \"rtt_us_max\": " << rtt.maximum_ns / 1000.0 << ",\n"
		     << "  \"wakeup_late_us_p99\": " << wake.p99_ns / 1000.0 << ",\n"
		     << "  \"wakeup_late_us_max\": " << wake.maximum_ns / 1000.0 << ",\n"
		     << "  \"out_complete_us_p50\": " << out_time.p50_ns / 1000.0 << ",\n"
		     << "  \"out_complete_to_in_us_p50\": " << return_wait.p50_ns / 1000.0 << ",\n"
		     << "  \"can_bridge\": " << (options_.can_bridge ? "true" : "false") << ",\n"
		     << "  \"bridge_failed_replies\": " << bridge_failures << ",\n"
		     << "  \"bridge_busy_replies\": " << busy_replies << ",\n"
		     << "  \"bridge_missing_feedback_replies\": " << missing_replies << ",\n"
		     << "  \"bridge_stopped_replies\": " << stopped_replies << ",\n"
		     << "  \"bridge_invalid_replies\": " << invalid_replies << ",\n"
		     << "  \"six_motor_cycles_over_1ms\": " << over_budget << ",\n"
		     << "  \"six_motor_cycle_rate_hz\": " << cycle_rate << ",\n"
		     << "  \"six_motor_rtt_us_p50\": " << cycle_rtt.p50_ns / 1000.0 << ",\n"
		     << "  \"six_motor_rtt_us_p99\": " << cycle_rtt.p99_ns / 1000.0 << ",\n"
		     << "  \"six_motor_rtt_us_max\": " << cycle_rtt.maximum_ns / 1000.0 << ",\n"
		     << "  \"mcu_out_to_last_feedback_us_p50\": " << device_wait.p50_ns / 1000.0
		     << ",\n"
		     << "  \"mcu_out_to_last_feedback_us_p99\": " << device_wait.p99_ns / 1000.0
		     << ",\n"
		     << "  \"six_motor_cycles\": " << successful_cycles << ",\n"
		     << "  \"valid_bus_rtt_samples\": " << rtts.size() << ",\n"
		     << "  \"link_pass\": " << (pass ? "true" : "false") << ",\n"
		     << "  \"timing_export_pass\": " << (timing_pass ? "true" : "false") << ",\n"
		     << "  \"timing_expected\": " << expected_timings << ",\n"
		     << "  \"timing_received\": " << timing_received_ << ",\n"
		     << "  \"timing_unavailable\": " << unavailable_timings << ",\n"
		     << "  \"timing_incomplete_successful_samples\": " << incomplete_timings
		     << ",\n"
		     << "  \"timing_valid_successful_samples\": " << valid_timings << ",\n"
		     << "  \"usb_pass\": " << (transport_pass ? "true" : "false") << ",\n"
		     << "  \"mcu_segments_us\": {\n";
		for (unsigned i = 0; i < segment_values.size(); ++i) {
			const auto value = rt::Summarize(segment_values[i]);
			json << "    \"" << segments[i].name
			     << "\": {\"samples\": " << segment_values[i].size()
			     << ", \"p50\": " << value.p50_ns / 1000.0
			     << ", \"p99\": " << value.p99_ns / 1000.0
			     << ", \"max\": " << value.maximum_ns / 1000.0 << '}'
			     << (i + 1 == segment_values.size() ? "\n" : ",\n");
		}
		json << "  }\n}\n";
		json.close();
		if (!json) {
			std::cerr << "Cannot write JSON\n";
			return false;
		}
		if (!options_.csv_path.empty()) {
			std::ofstream csv(options_.csv_path);
			csv << "sequence,submit_ns,out_complete_ns,in_complete_ns,wakeup_late_ns,"
			       "cycle,group,status,mcu_out_us,feedback0_after_out_us,feedback1_"
			       "after_out_us,feedback2_after_out_us,feedback0_hex,feedback1_hex,"
			       "feedback2_hex,timing_present,timing_status,dequeued_after_out_us,"
			       "first_submit_after_out_us,last_submit_after_out_us,first_driver_"
			       "after_out_us,last_driver_after_out_us,first_tx_complete_after_out_"
			       "us,last_tx_complete_after_out_us,first_feedback_after_out_us,last_"
			       "feedback_after_out_us,collected_after_out_us,queued_after_out_us,"
			       "in_submit_after_out_us\n";
			for (const auto &record : records_) {
				csv << record.sequence << ',' << record.submitted_ns << ','
				    << record.out_complete_ns << ',' << record.in_complete_ns << ','
				    << record.wakeup_late_ns << ',' << record.cycle << ','
				    << record.group << ',' << record.bridge_status << ','
				    << ReadU32(record.reply.data() + 12);
				for (unsigned i = 0; i < 3; ++i)
					csv << ','
					    << ReadU32(record.reply.data() +
						       bridge_wire::FrameOffset(i) + 12);
				constexpr char hex[] = "0123456789abcdef";
				for (unsigned i = 0; i < 3; ++i) {
					csv << ',';
					for (unsigned j = 0; j < 8; ++j) {
						const auto v =
							record.reply[bridge_wire::FrameOffset(i) +
								     4 + j];
						csv << hex[v >> 4] << hex[v & 15];
					}
				}
				csv << ',' << record.has_timing << ',';
				if (record.has_timing)
					csv << ReadU32(record.timing.data() + 8);
				for (unsigned i = 0; i < static_cast<unsigned>(TimePoint::kCount);
				     ++i) {
					csv << ',';
					if (record.has_timing) {
						const auto value = bridge_wire::GetTime(
							record.timing, static_cast<TimePoint>(i));
						if (value != bridge_wire::kUnsetTime)
							csv << value;
					}
				}
				csv << '\n';
			}
			if (!csv) {
				std::cerr << "Cannot write CSV\n";
				return false;
			}
		}
		std::cout << "OUT=" << submitted_ << " IN=" << received_ << " rate=" << rate
			  << " Hz RTT p50/p99/max=" << rtt.p50_ns / 1000.0 << '/'
			  << rtt.p99_ns / 1000.0 << '/' << rtt.maximum_ns / 1000.0
			  << " us wakeup p99=" << wake.p99_ns / 1000.0 << " us\n";
		if (options_.can_bridge) {
			std::cout << "Six-motor cycles=" << successful_cycles
				  << " rate=" << cycle_rate
				  << " Hz end-to-end RTT p50/p99/max=" << cycle_rtt.p50_ns / 1000.0
				  << '/' << cycle_rtt.p99_ns / 1000.0 << '/'
				  << cycle_rtt.maximum_ns / 1000.0
				  << " us; cycles over 1 ms=" << over_budget
				  << "; rejected/missing-feedback replies=" << bridge_failures
				  << '\n';
			for (unsigned group = 0; group < 2; ++group) {
				const auto b = rt::Summarize(bus_rtts[group]);
				std::cout << (group == 0 ? "Left" : "Right")
					  << " bus RTT p50/p99/max=" << b.p50_ns / 1000.0 << '/'
					  << b.p99_ns / 1000.0 << '/' << b.maximum_ns / 1000.0
					  << " us\n";
			}
		}
		if (options_.can_bridge) {
			std::cout << "MCU timing samples=" << timing_received_ << '/'
				  << expected_timings << " valid-success=" << valid_timings
				  << " unavailable=" << unavailable_timings << '\n';
			for (unsigned i = 0; i < segment_values.size(); ++i) {
				const auto value = rt::Summarize(segment_values[i]);
				std::cout << segments[i].name
					  << " p50/p99/max=" << value.p50_ns / 1000.0 << '/'
					  << value.p99_ns / 1000.0 << '/'
					  << value.maximum_ns / 1000.0
					  << " us n=" << segment_values[i].size() << '\n';
			}
		}
		return pass && timing_pass;
	}

	~Impl()
	{
		Close();
	}

private:
	struct OutSlot {
		Impl *owner = nullptr;
		libusb_transfer *transfer = nullptr;
		stress_host::Packet packet{};
		std::size_t record_index = 0;
		unsigned query_group = 0;
		bool active = false;
		bool pending = false;
		bool out_done = false;
		bool reply_done = false;
	};
	struct InSlot {
		Impl *owner = nullptr;
		libusb_transfer *transfer = nullptr;
		stress_host::Packet packet{};
		bool pending = false;
	};
	static void LIBUSB_CALL OnPollfdAdded(int, short, void *user_data)
	{
		static_cast<Impl *>(user_data)->poll_sources_changed_.store(true);
	}
	static void LIBUSB_CALL OnPollfdRemoved(int, void *user_data)
	{
		static_cast<Impl *>(user_data)->poll_sources_changed_.store(true);
	}
	bool Fail(const char *message)
	{
		failed_ = true;
		if (failure_.empty()) {
			failure_ = message;
			std::cerr << message << std::endl;
		}
		return false;
	}
	// Record transfer status and short-transfer length before cleanup cancels peers.
	// The generic failure string previously lost the evidence needed to distinguish
	// timeout, endpoint stall, disconnection, I/O error and a short USB packet.
	bool FailTransfer(const char *direction, const libusb_transfer *transfer)
	{
		const char *status = "unknown";
		switch (transfer->status) {
		case LIBUSB_TRANSFER_COMPLETED:
			status = "completed";
			break;
		case LIBUSB_TRANSFER_ERROR:
			status = "io-error";
			break;
		case LIBUSB_TRANSFER_TIMED_OUT:
			status = "timeout";
			break;
		case LIBUSB_TRANSFER_CANCELLED:
			status = "cancelled";
			break;
		case LIBUSB_TRANSFER_STALL:
			status = "stall";
			break;
		case LIBUSB_TRANSFER_NO_DEVICE:
			status = "no-device";
			break;
		case LIBUSB_TRANSFER_OVERFLOW:
			status = "overflow";
			break;
		}
		char message[192];
		std::snprintf(message, sizeof(message),
			      "%s transfer failed: status=%s(%d) actual=%d expected=%d", direction,
			      status, static_cast<int>(transfer->status), transfer->actual_length,
			      transfer->length);
		return Fail(message);
	}

	void ReleaseSlot(OutSlot &slot)
	{
		if (slot.out_done && slot.reply_done) {
			slot.active = false;
			--active_slots_;
		}
	}
	void SubmitIn(InSlot &input)
	{
		if (libusb_submit_transfer(input.transfer) != 0) {
			Fail("Submit IN failed");
		} else {
			input.pending = true;
		}
	}
	void SubmitOut(std::uint64_t now, std::int64_t wakeup_late_ns, unsigned group)
	{
		auto available = std::find_if(out_.begin(), out_.end(),
					      [](const auto &slot) { return !slot.active; });
		if (available == out_.end()) {
			++skipped_slots_;
			return;
		}
		auto &slot = *available;
		const auto sequence = static_cast<std::uint32_t>(records_.size());
		slot.packet = options_.can_stream
				      ? bridge_wire::StreamCommand(sequence, last_rtt_us_, group)
			      : options_.can_bridge
				      ? bridge_wire::EncodeRequest(sequence, last_rtt_us_, group)
				      : stress_host::EncodePacket(sequence, last_rtt_us_);
		if (options_.control_latency) {
			bridge_wire::WriteU32(slot.packet.data(), bridge_wire::kControlCommand);
			uint32_t mask = 0;
			for (unsigned i = 0; i < 3; ++i) {
				const unsigned motor = group * 3 + i;
				if (!latest_feedback_valid_[motor])
					continue;
				mask |= 1U << i;
				bridge_wire::WriteU32(slot.packet.data() +
							      bridge_wire::FrameOffset(i) + 12,
						      latest_feedback_us_[motor]);
			}
			bridge_wire::WriteU32(slot.packet.data() + 12, mask);
		}
		slot.record_index = records_.size();
		slot.active = true;
		slot.out_done = slot.reply_done = false;
		records_.push_back(
			{sequence, now, 0, 0, wakeup_late_ns, cycle_sequence_, group, 0, {}});
		if (options_.control_latency) {
			auto &record = records_.back();
			record.origin_mask = bridge_wire::ReadU32(slot.packet.data() + 12);
			for (unsigned i = 0; i < 3; ++i) {
				record.origin_us[i] = bridge_wire::ReadU32(
					slot.packet.data() + bridge_wire::FrameOffset(i) + 12);
				record.origin_host_ns[i] = latest_feedback_host_ns_[group * 3 + i];
			}
		}
		++active_slots_;
		peak_slots_ = std::max(peak_slots_, active_slots_);
		FillTransfer(slot.transfer, device_, 0x01, slot.packet.data(), slot.packet.size(),
			     OnOut, &slot, 250);
		records_[slot.record_index].submitted_ns = rt::MonotonicTimeNs();
		if (libusb_submit_transfer(slot.transfer) != 0) {
			slot.active = false;
			--active_slots_;
			Fail("Submit OUT failed");
		} else {
			slot.pending = true;
			++submitted_;
		}
	}
	static void LIBUSB_CALL OnOut(libusb_transfer *transfer)
	{
		auto &slot = *static_cast<OutSlot *>(transfer->user_data);
		auto &self = *slot.owner;
		slot.pending = false;
		if (self.closing_) {
			return;
		}
		if (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
		    transfer->actual_length != 64) {
			self.FailTransfer("OUT", transfer);
			return;
		}
		if (!self.dumping_) {
			self.records_[slot.record_index].out_complete_ns = rt::MonotonicTimeNs();
			++self.out_completed_;
		}
		slot.out_done = true;
		self.ReleaseSlot(slot);
	}
	static void LIBUSB_CALL OnIn(libusb_transfer *transfer)
	{
		auto &input = *static_cast<InSlot *>(transfer->user_data);
		auto &self = *input.owner;
		input.pending = false;
		if (self.closing_) {
			return;
		}
		if (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
		    transfer->actual_length != 64) {
			self.FailTransfer("IN", transfer);
			return;
		}
		const auto arrival_ns = rt::MonotonicTimeNs();
		if (self.options_.can_stream &&
		    ReadU32(input.packet.data()) == bridge_wire::kStreamFeedback) {
			if (!bridge_wire::ValidFeedback(input.packet)) {
				self.Fail("Invalid streaming feedback");
				return;
			}
			if (!self.dumping_) {
				const auto packet_sequence = ReadU32(input.packet.data() + 4);
				if (!self.feedback_packets_)
					self.feedback_sequence_gaps_ += packet_sequence;
				else if (packet_sequence != self.last_feedback_packet_ + 1)
					self.feedback_sequence_gaps_ += uint32_t(
						packet_sequence - self.last_feedback_packet_ - 1);
				self.last_feedback_packet_ = packet_sequence;
				++self.feedback_packets_;
				for (unsigned i = 0; i < ReadU32(input.packet.data() + 8); ++i) {
					if (self.feedback_records_.size() ==
					    self.feedback_records_.capacity()) {
						self.Fail("Feedback recording capacity exceeded");
						return;
					}
					const auto *frame =
						input.packet.data() + bridge_wire::FrameOffset(i);
					FeedbackRecord record{
						arrival_ns,
						packet_sequence,
						static_cast<uint32_t>(
							bridge_wire::FeedbackMotor(frame)),
						ReadU32(input.packet.data() + 12),
						ReadU32(frame + 12),
						{}};
					std::copy(frame + 4, frame + 12, record.data.begin());
					self.latest_feedback_us_[record.motor] = record.mcu_rx_us;
					self.latest_feedback_host_ns_[record.motor] = arrival_ns;
					self.latest_feedback_valid_[record.motor] = true;
					self.feedback_records_.push_back(record);
				}
			}
			self.SubmitIn(input);
			return;
		}
		const auto sequence = stress_host::ReadU32(input.packet.data() + 4);
		auto found =
			std::find_if(self.out_.begin(), self.out_.end(), [&](const auto &slot) {
				return slot.active && !slot.reply_done &&
				       self.records_[slot.record_index].sequence == sequence;
			});
		if (self.dumping_ && self.options_.control_latency &&
		    ReadU32(input.packet.data()) == bridge_wire::kControlReply) {
			if (found == self.out_.end() ||
			    !bridge_wire::ValidControlReply(input.packet, sequence)) {
				self.Fail("Control timing protocol/sequence mismatch");
				return;
			}
			auto &record = self.records_[found->record_index];
			record.timing = input.packet;
			record.has_timing = true;
			++self.timing_received_;
			found->reply_done = true;
			self.ReleaseSlot(*found);
			self.SubmitIn(input);
			return;
		}
		if (self.dumping_ && self.options_.can_stream) {
			if (found == self.out_.end() ||
			    !bridge_wire::ValidStreamStats(input.packet, sequence,
							   found->query_group)) {
				self.Fail("Stream final statistics protocol/sequence mismatch");
				return;
			}
			const unsigned group = ReadU32(input.packet.data() + 8);
			self.stream_stats_[group] = input.packet;
			self.stream_stats_present_[group] = true;
			found->reply_done = true;
			self.ReleaseSlot(*found);
			self.SubmitIn(input);
			return;
		}
		if (self.dumping_) {
			if (found == self.out_.end() ||
			    !bridge_wire::ValidTiming(input.packet, sequence)) {
				self.Fail("Timing reply protocol/sequence mismatch");
				return;
			}
			auto &record = self.records_[found->record_index];
			if (!(ReadU32(input.packet.data() + 8) & bridge_wire::kTimingUnavailable) &&
			    (ReadU32(input.packet.data() + 8) != record.bridge_status ||
			     ReadU32(input.packet.data() + 12) !=
				     ReadU32(record.reply.data() + 12))) {
				self.Fail("Timing trace does not match the measured reply");
				return;
			}
			record.timing = input.packet;
			record.has_timing = true;
			++self.timing_received_;
			found->reply_done = true;
			self.ReleaseSlot(*found);
			self.SubmitIn(input);
			return;
		}
		if (found == self.out_.end() ||
		    (self.options_.can_bridge
			     ? !(self.options_.can_stream
					 ? bridge_wire::ValidAck(input.packet)
					 : bridge_wire::ValidateReply(
						   input.packet,
						   self.records_[found->record_index].group))
			     : found->packet != input.packet)) {
			self.Fail("Reply protocol, CAN records, echo bytes or sequence mismatch");
			return;
		}
		if (self.options_.control_latency &&
		    ReadU32(input.packet.data() + 24) != bridge_wire::kControlReply) {
			self.Fail("Firmware lacks control latency support; flash updated Bulk "
				  "firmware");
			return;
		}
		auto &record = self.records_[found->record_index];
		record.in_complete_ns = arrival_ns;
		if (self.options_.can_bridge) {
			record.reply = input.packet;
			record.bridge_status = ReadU32(input.packet.data() + 8);
		}
		if (record.bridge_status == 0)
			self.last_rtt_us_ = static_cast<std::uint32_t>(
				(record.in_complete_ns - record.submitted_ns) / 1000);
		++self.received_;
		found->reply_done = true;
		self.ReleaseSlot(*found);
		self.SubmitIn(input);
	}
	bool WriteStreamResults()
	{
		const double elapsed = (end_ns_ - start_ns_) / 1e9;
		std::vector<int64_t> acks, wakeups;
		unsigned rejected = 0;
		for (const auto &record : records_) {
			wakeups.push_back(record.wakeup_late_ns);
			if (record.in_complete_ns && !record.bridge_status)
				acks.push_back(record.in_complete_ns - record.submitted_ns);
			if (record.bridge_status)
				++rejected;
		}
		const auto ack = rt::Summarize(acks), wake = rt::Summarize(wakeups);
		std::array<std::vector<int64_t>, 6> ages, gaps;
		std::array<uint64_t, 6> previous{}, exported{};
		for (const auto &feedback : feedback_records_) {
			++exported[feedback.motor];
			ages[feedback.motor].push_back(
				uint32_t(feedback.mcu_in_submit_us - feedback.mcu_rx_us) * 1000ULL);
			if (previous[feedback.motor])
				gaps[feedback.motor].push_back(feedback.host_ns -
							       previous[feedback.motor]);
			previous[feedback.motor] = feedback.host_ns;
		}
		const bool transport_pass = !measurement_failed_ && !measurement_outstanding_ &&
					    submitted_ == out_completed_ &&
					    submitted_ == received_ && received_ / elapsed >= 1980;
		bool can_pass = stream_stats_present_[0] && stream_stats_present_[1];
		const char *names[] = {"left_wheel",  "left_joint_b",  "left_joint_d",
				       "right_wheel", "right_joint_b", "right_joint_d"};
		std::ofstream json(options_.json_path);
		json << "{\n  \"mode\": \"native-can-stream\",\n"
		     << "  \"pacing\": \"timerfd-absolute\",\n"
		     << "  \"realtime\": " << (options_.realtime ? "true" : "false") << ",\n"
		     << "  \"cpu\": " << options_.cpu << ",\n"
		     << "  \"fifo_priority\": " << (options_.realtime ? options_.priority : 0)
		     << ",\n"
		     << "  \"memory_locked\": " << (options_.realtime ? "true" : "false") << ",\n"
		     << "  \"isolated_cpus\": \"" << IsolatedCpus() << "\",\n"
		     << "  \"out_order\": \""
		     << (options_.right_first ? "right-left" : "left-right") << "\",\n"
		     << "  \"feedback_grace_us\": " << options_.feedback_grace_us << ",\n"
		     << "  \"usb_transport\": \"" << TransportName() << "\",\n"
		     << "  \"endpoint_mps_out\": " << out_mps_ << ",\n"
		     << "  \"endpoint_mps_in\": " << in_mps_ << ",\n"
		     << "  \"endpoint_interval_out\": " << out_interval_ << ",\n"
		     << "  \"endpoint_interval_in\": " << in_interval_ << ",\n"
		     << "  \"usb_speed_mbps\": " << (speed_ == LIBUSB_SPEED_HIGH ? 480 : 12)
		     << ",\n"
		     << "  \"elapsed_s\": " << elapsed << ",\n"
		     << "  \"sent_commands\": " << submitted_ << ",\n"
		     << "  \"received_acks\": " << received_ << ",\n"
		     << "  \"rejected_commands\": " << rejected << ",\n"
		     << "  \"host_skipped_slots\": " << skipped_slots_ << ",\n"
		     << "  \"feedback_packets\": " << feedback_packets_ << ",\n"
		     << "  \"feedback_sequence_gaps\": " << feedback_sequence_gaps_ << ",\n"
		     << "  \"ack_rtt_us_p50\": " << ack.p50_ns / 1000.0 << ",\n"
		     << "  \"ack_rtt_us_p99\": " << ack.p99_ns / 1000.0 << ",\n"
		     << "  \"ack_rtt_us_max\": " << ack.maximum_ns / 1000.0 << ",\n"
		     << "  \"errors\": " << (failed_ ? 1 : 0) << ",\n"
		     << "  \"failure\": \"" << failure_ << "\",\n  \"motors\": [\n";
		std::cout << "Commands OUT=" << submitted_ << " ACK=" << received_
			  << " rejected=" << rejected
			  << " ACK RTT p50/p99/max=" << ack.p50_ns / 1000.0 << '/'
			  << ack.p99_ns / 1000.0 << '/' << ack.maximum_ns / 1000.0
			  << " us; ACK is submission only, not motor RTT; wakeup p99="
			  << wake.p99_ns / 1000.0 << " us\n";
		for (unsigned motor = 0; motor < 6; ++motor) {
			const unsigned group = motor / 3, item = motor % 3;
			const auto *data =
				stream_stats_[group].data() + bridge_wire::FrameOffset(item);
			const uint32_t complete = ReadU32(data), rx = ReadU32(data + 4),
				       coalesced = ReadU32(data + 8), errors = ReadU32(data + 12);
			const double seconds = ReadU32(stream_stats_[group].data() + 12) / 1e6;
			const double hz = seconds > 0 ? complete / seconds : 0;
			const auto age = rt::Summarize(ages[motor]),
				   gap = rt::Summarize(gaps[motor]);
			can_pass &= hz >= 990 && coalesced == 0 && errors == 0 && rx > 0 &&
				    exported[motor] > 0;
			json << "    {\"name\": \"" << names[motor]
			     << "\", \"completed\": " << complete << ", \"can_hz\": " << hz
			     << ", \"can_rx\": " << rx << ", \"coalesced\": " << coalesced
			     << ", \"tx_errors\": " << errors
			     << ", \"feedback_updates\": " << exported[motor]
			     << ", \"feedback_export_hz\": " << exported[motor] / elapsed
			     << ", \"age_at_mcu_in_submit_us_p50\": " << age.p50_ns / 1000.0
			     << ", \"age_at_mcu_in_submit_us_p99\": " << age.p99_ns / 1000.0
			     << ", \"age_at_mcu_in_submit_us_max\": " << age.maximum_ns / 1000.0
			     << ", \"host_feedback_gap_us_p99\": " << gap.p99_ns / 1000.0 << '}'
			     << (motor == 5 ? "\n" : ",\n");
			std::cout << names[motor] << " CAN=" << hz << " Hz done=" << complete
				  << " coalesced=" << coalesced << " errors=" << errors
				  << " CAN_RX=" << rx << " exported=" << exported[motor]
				  << " feedback age at MCU IN p50/p99=" << age.p50_ns / 1000.0
				  << '/' << age.p99_ns / 1000.0 << " us\n";
		}
		bool batching_pass = stream_stats_present_[2] && stream_stats_present_[3];
		json << "  ],\n  \"bus_batches\": [\n";
		for (unsigned group = 0; group < 2; ++group) {
			const auto &packet = stream_stats_[group + 2];
			uint64_t submitted_frames = 0, completed_frames = 0, motor_frames = 0;
			uint32_t submitted[3]{}, completed[3]{};
			for (unsigned i = 0; i < 3; ++i) {
				submitted[i] = ReadU32(packet.data() + 16 + i * 4);
				completed[i] = ReadU32(packet.data() + 28 + i * 4);
				submitted_frames += uint64_t(i + 1) * submitted[i];
				completed_frames += uint64_t(i + 1) * completed[i];
				motor_frames += ReadU32(stream_stats_[group].data() +
							bridge_wire::FrameOffset(i));
			}
			batching_pass &= submitted[0] == 0 && submitted[1] == 0 &&
					 submitted[2] > 0 && submitted[2] == completed[2] &&
					 submitted_frames == completed_frames &&
					 completed_frames == motor_frames;
			const auto bus = bridge_wire::kLayout[group][0].bus;
			std::cout << "bus=" << unsigned(bus)
				  << " driver batches(1/2/3)=" << submitted[0] << '/'
				  << submitted[1] << '/' << submitted[2]
				  << " completed=" << completed[0] << '/' << completed[1] << '/'
				  << completed[2] << '\n';
			json << "    {\"bus\": " << unsigned(bus) << ", \"submitted_by_size\": ["
			     << submitted[0] << ',' << submitted[1] << ',' << submitted[2]
			     << "], \"completed_by_size\": [" << completed[0] << ',' << completed[1]
			     << ',' << completed[2] << "]}" << (group == 1 ? "\n" : ",\n");
		}
		const bool control_pass =
			!options_.control_latency ||
			WriteControlLatencyReport(records_, feedback_records_, options_.json_path);
		const bool pass = control_pass && transport_pass && can_pass && rejected == 0 &&
				  !failed_ && feedback_sequence_gaps_ == 0 && batching_pass;
		json << "  ],\n  \"usb_pass\": " << (transport_pass ? "true" : "false")
		     << ",\n  \"can_pass\": " << (can_pass ? "true" : "false")
		     << ",\n  \"group_batching_pass\": " << (batching_pass ? "true" : "false")
		     << ",\n  \"control_latency_enabled\": "
		     << (options_.control_latency ? "true" : "false")
		     << ",\n  \"control_measurement_pass\": " << (control_pass ? "true" : "false")
		     << ",\n  \"link_pass\": " << (pass ? "true" : "false") << "\n}\n";
		json.close();
		if (!json)
			return false;
		if (!WriteCaptureCsv(records_, feedback_records_, options_.csv_path))
			return false;
		return pass;
	}

	void FetchStreamStats()
	{
		std::cout << "Measurement complete; stopping motors and reading final CAN counters"
			  << std::endl;
		dumping_ = true;
		event_waiter_.Stop();
		if (records_.size() < bridge_wire::kStreamStatsPages) {
			Fail("No complete control cycle to query counters");
			return;
		}
		for (unsigned group = 0; group < bridge_wire::kStreamStatsPages; ++group) {
			auto &slot = out_[group];
			slot.record_index = group;
			slot.query_group = group;
			slot.packet = bridge_wire::StreamStatsQuery(
				records_[slot.record_index].sequence, group);
			slot.active = true;
			slot.out_done = slot.reply_done = false;
			++active_slots_;
			FillTransfer(slot.transfer, device_, 0x01, slot.packet.data(), 64, OnOut,
				     &slot, 2000);
			if (libusb_submit_transfer(slot.transfer) != 0) {
				slot.active = false;
				--active_slots_;
				Fail("Stream stats OUT submit failed");
				return;
			}
			slot.pending = true;
		}
		const auto deadline = rt::MonotonicTimeNs() + 5000000000ULL;
		while (!failed_ && !stop_requested && active_slots_) {
			if (rt::MonotonicTimeNs() > deadline) {
				Fail("Stream final statistics timed out");
				break;
			}
			timeval timeout{0, 10000};
			const int rc =
				libusb_handle_events_timeout_completed(context_, &timeout, nullptr);
			if (rc != 0 && rc != LIBUSB_ERROR_INTERRUPTED) {
				Fail("Stream stats USB event failure");
				break;
			}
		}
	}

	void DumpTimings()
	{
		std::cout << "Measurement complete; exporting sampled MCU timestamps" << std::endl;
		dumping_ = true;
		event_waiter_.Stop();
		std::size_t next = 0;
		const uint64_t deadline = rt::MonotonicTimeNs() + 10000000000ULL;
		while (!failed_ && !stop_requested) {
			for (auto &slot : out_) {
				if (slot.active)
					continue;
				while (next < records_.size() &&
				       !bridge_wire::SampleTiming(records_[next].sequence))
					++next;
				if (next == records_.size())
					break;
				slot.record_index = next++;
				slot.packet =
					options_.control_latency
						? bridge_wire::ControlQuery(
							  records_[slot.record_index].sequence)
						: bridge_wire::TimingQuery(
							  records_[slot.record_index].sequence);
				slot.active = true;
				slot.out_done = slot.reply_done = false;
				++active_slots_;
				FillTransfer(slot.transfer, device_, 0x01, slot.packet.data(),
					     slot.packet.size(), OnOut, &slot, 1000);
				if (libusb_submit_transfer(slot.transfer) != 0) {
					slot.active = false;
					--active_slots_;
					Fail("Timing query OUT submission failed");
					break;
				}
				slot.pending = true;
				++timing_requested_;
			}
			if (next == records_.size() && active_slots_ == 0)
				break;
			if (rt::MonotonicTimeNs() > deadline) {
				Fail("Timing export deadline exceeded");
				break;
			}
			timeval timeout{0, 10000};
			const int rc =
				libusb_handle_events_timeout_completed(context_, &timeout, nullptr);
			if (rc != 0 && rc != LIBUSB_ERROR_INTERRUPTED) {
				Fail("Timing export USB event failure");
				break;
			}
		}
		if (stop_requested)
			Fail("Timing export interrupted");
	}

	void Close()
	{
		if (context_ == nullptr) {
			return;
		}
		closing_ = true;
		event_waiter_.Stop();
		libusb_set_pollfd_notifiers(context_, nullptr, nullptr, nullptr);
		for (auto &slot : out_) {
			if (slot.pending) {
				libusb_cancel_transfer(slot.transfer);
			}
		}
		for (auto &slot : in_) {
			if (slot.pending) {
				libusb_cancel_transfer(slot.transfer);
			}
		}
		// libusb requires cancellation callbacks before transfers/buffers are freed.
		for (;;) {
			bool pending = false;
			for (const auto &slot : out_) {
				pending = pending || slot.pending;
			}
			for (const auto &slot : in_) {
				pending = pending || slot.pending;
			}
			if (!pending) {
				break;
			}
			timeval timeout{0, 10000};
			libusb_handle_events_timeout_completed(context_, &timeout, nullptr);
		}
		for (auto &slot : out_) {
			if (slot.transfer) {
				libusb_free_transfer(slot.transfer);
				slot.transfer = nullptr;
			}
		}
		for (auto &slot : in_) {
			if (slot.transfer) {
				libusb_free_transfer(slot.transfer);
				slot.transfer = nullptr;
			}
		}
		if (claimed_) {
			libusb_release_interface(device_, 0);
		}
		if (device_) {
			libusb_close(device_);
			device_ = nullptr;
		}
		libusb_exit(context_);
		context_ = nullptr;
	}

	BenchmarkOptions options_;
	EventWaiter event_waiter_;
	FeedbackGate feedback_gate_;
	std::atomic<bool> poll_sources_changed_{false};
	libusb_context *context_ = nullptr;
	libusb_device_handle *device_ = nullptr;
	bool claimed_ = false, closing_ = false, failed_ = false, sending_done_ = false;
	std::string failure_;
	std::array<OutSlot, kOutWindow> out_{};
	std::array<InSlot, kInWindow> in_{};
	std::vector<Record> records_;
	std::uint64_t start_ns_ = 0, next_release_ns_ = 0, sender_finished_ns_ = 0, end_ns_ = 0;
	std::uint64_t skipped_slots_ = 0;
	unsigned active_slots_ = 0, peak_slots_ = 0, received_ = 0, out_completed_ = 0,
		 submitted_ = 0;
	std::uint32_t last_rtt_us_ = 0;
	uint64_t cycle_sequence_ = 0;
	bool dumping_ = false, measurement_failed_ = false;
	unsigned timing_requested_ = 0, timing_received_ = 0, measurement_outstanding_ = 0;
	std::array<uint32_t, 6> latest_feedback_us_{};
	std::array<uint64_t, 6> latest_feedback_host_ns_{};
	std::array<bool, 6> latest_feedback_valid_{};
	std::vector<FeedbackRecord> feedback_records_;
	uint64_t feedback_packets_ = 0, feedback_sequence_gaps_ = 0;
	uint32_t last_feedback_packet_ = 0;
	std::array<Packet, bridge_wire::kStreamStatsPages> stream_stats_{};
	std::array<bool, bridge_wire::kStreamStatsPages> stream_stats_present_{};
	bool bulk_ = false;
	unsigned in_mps_ = 0, out_mps_ = 0, in_interval_ = 0, out_interval_ = 0;
	int speed_ = 0;
};

UsbBenchmark::UsbBenchmark(const BenchmarkOptions &options) : impl_(new Impl(options))
{
}
UsbBenchmark::~UsbBenchmark() = default;
bool UsbBenchmark::Open()
{
	return impl_->Open();
}
void UsbBenchmark::Run()
{
	impl_->Run();
}
bool UsbBenchmark::WriteResults()
{
	return impl_->WriteResults();
}
void UsbBenchmark::RequestStop(int)
{
	stop_requested = 1;
}
} // namespace stress_host
