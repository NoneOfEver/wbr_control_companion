/* SPDX-License-Identifier: Apache-2.0 */
#include "rmcs_style_benchmark.hpp"
#include "benchmark_capture_io.hpp"
#include "bridge_profile.hpp"
#include "hardware_timing_report.hpp"
#include "control_latency_report.hpp"
#include "event_waiter.hpp"
#include "merged_control.hpp"
#include "rmcs_style_state.hpp"
#include <wbr/realtime_loop.hpp>
#include <libusb.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sched.h>
#include <thread>

namespace stress_host
{
namespace
{
namespace rt = wbr::realtime;
using bridge_wire::MergedControl;
constexpr uint64_t kPeriodNs = 1000000;
volatile std::sig_atomic_t stop_requested = 0;

// One USB event thread owns receive parsing/recording. The control thread owns
// command creation and submission. Records are pre-sized, published with release
// ordering before submit, never moved during capture, and read after USB joins.
class RmcsStyleBenchmark
{
public:
	explicit RmcsStyleBenchmark(const RmcsStyleOptions &options) : options_(options)
	{
		records_.resize(size_t(options.seconds * 2000) + 64);
		feedback_.reserve(size_t(options.seconds * 20000) + 64);
	}
	~RmcsStyleBenchmark()
	{
		CloseUsb();
	}

	int Execute()
	{
		std::string error;
		if (options_.realtime &&
		    !rt::ConfigureCurrentThread(options_.cpu, options_.priority, error))
			throw std::runtime_error("Control realtime setup failed: " + error);
		if (!OpenUsb()) {
			CloseUsb();
			std::cerr << failure_ << '\n';
			return 1;
		}
		{
			Packet capability{};
			if (!Query(bridge_wire::MergedCapabilityQuery(0xffffffe0U),
				   bridge_wire::kMergedCapabilityReply, capability) ||
			    (!options_.split_out &&
			     !bridge_wire::SupportsMergedControl(capability))) {
				Fail("Merged OUT unsupported: flash the merged-capable test "
				     "firmware");
				CloseUsb();
				std::cerr << failure_ << '\n';
				return 1;
			}
			const bool profile_supported =
				ReadU32(capability.data() + 24) == bridge_wire::kProfileReply;
			if (profile_supported) {
				Packet reply{};
				constexpr uint32_t sequence = 0xffffffd0U;
				if (!Query(bridge_wire::ProfileQuery(sequence, options_.can_order,
								     options_.fast_ingress),
					   bridge_wire::kProfileReply, reply) ||
				    !bridge_wire::ValidProfileReply(reply, sequence,
								    options_.can_order,
								    options_.fast_ingress)) {
					Fail("MCU rejected CAN/ingress profile");
					CloseUsb();
					std::cerr << failure_ << '\n';
					return 1;
				}
			} else if (options_.can_order || options_.fast_ingress) {
				Fail("Flash profile-capable firmware before selecting CAN order or "
				     "fast ingress");
				CloseUsb();
				std::cerr << failure_ << '\n';
				return 1;
			}
			hardware_timing_supported_ =
				ReadU32(capability.data() + 20) == bridge_wire::kHardwareReply;
		}
		std::cout << "RMCS-style: control cpu=" << sched_getcpu()
			  << " priority=" << (options_.realtime ? options_.priority : 0)
			  << "; USB event cpu=" << actual_usb_cpu_
			  << " priority=" << (options_.realtime ? options_.usb_priority : 0)
			  << "; Bulk 480 Mbps; OUT bytes=" << (options_.split_out ? 64 : 128)
			  << ", transfers/cycle=" << (options_.split_out ? 2 : 1)
			  << "; four IN preposted; no disk writes during capture\n"
			  << "Two submission ACKs/cycle retained for measurement; OUT buffers "
			     "recycle "
			     "on USB completion, without waiting for ACK or feedback"
			  << std::endl;
		std::cout << "CAN batch order=" << bridge_wire::kCanOrderNames[options_.can_order]
			  << " ingress=" << (options_.fast_ingress ? "fast" : "baseline")
			  << std::endl;
		Capture();
		sched_param normal{};
		sched_setscheduler(0, SCHED_OTHER, &normal);
		// Keep receiving late replies before stopping motors/exporting records.
		const auto deadline = rt::MonotonicTimeNs() + 1000000000ULL;
		while (!failed_ && (acks_.load() != published_.load() || pending_out_.load())) {
			if (rt::MonotonicTimeNs() >= deadline) {
				Fail("Command replies did not drain");
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		capturing_.store(false);
		if (!failed_)
			ExportDiagnostics();
		std::cout << "Closing USB event thread" << std::endl;
		CloseUsb();
		std::cout << "USB closed; writing results" << std::endl;
		records_.resize(published_.load());
		return WriteResults() ? 0 : 1;
	}

private:
	struct Out {
		RmcsStyleBenchmark *owner = nullptr;
		libusb_transfer *transfer = nullptr;
		MergedControl data{};
		std::atomic<bool> free{true};
		size_t first = 0;
		unsigned count = 0;
	};
	struct In {
		RmcsStyleBenchmark *owner = nullptr;
		libusb_transfer *transfer = nullptr;
		Packet data{};
		std::atomic<bool> pending{false};
	};
	bool Fail(const std::string &message)
	{
		{
			std::lock_guard<std::mutex> lock(failure_mutex_);
			if (failure_.empty())
				failure_ = message;
			failed_.store(true);
		}
		query_cv_.notify_all();
		return false;
	}
	bool CheckDescriptors()
	{
		if (libusb_get_device_speed(libusb_get_device(device_)) != LIBUSB_SPEED_HIGH)
			return Fail("RMCS-style test requires High-Speed USB");
		libusb_config_descriptor *config = nullptr;
		if (libusb_get_active_config_descriptor(libusb_get_device(device_), &config))
			return Fail("Cannot read USB descriptors");
		unsigned found = 0;
		for (unsigned i = 0; i < config->bNumInterfaces; ++i)
			for (int j = 0; j < config->interface[i].num_altsetting; ++j) {
				const auto &alt = config->interface[i].altsetting[j];
				if (alt.bInterfaceNumber != 0 || alt.bAlternateSetting != 0)
					continue;
				for (unsigned e = 0; e < alt.bNumEndpoints; ++e) {
					const auto &ep = alt.endpoint[e];
					if ((ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) !=
						    LIBUSB_TRANSFER_TYPE_BULK ||
					    ep.wMaxPacketSize != 512)
						continue;
					if (ep.bEndpointAddress == 0x81)
						found |= 1;
					if (ep.bEndpointAddress == 0x01)
						found |= 2;
				}
			}
		libusb_free_config_descriptor(config);
		return found == 3 || Fail("Expected Bulk IN/OUT endpoints with HS MPS=512");
	}
	bool OpenUsb()
	{
		if (libusb_init(&context_))
			return Fail("libusb_init failed");
		device_ = libusb_open_device_with_vid_pid(context_, 0x34b7, 0xffff);
		if (!device_)
			return Fail("USB device not found or permission denied");
		int configuration = 0;
		if (libusb_get_configuration(device_, &configuration) ||
		    (configuration != 1 && libusb_set_configuration(device_, 1)))
			return Fail("Cannot configure USB device");
		if (libusb_claim_interface(device_, 0))
			return Fail("Cannot claim USB interface");
		claimed_ = true;
		if (!CheckDescriptors())
			return false;
		for (auto &out : out_) {
			out.owner = this;
			out.transfer = libusb_alloc_transfer(0);
			if (!out.transfer)
				return Fail("Cannot allocate OUT transfer");
		}
		for (auto &input : in_) {
			input.owner = this;
			input.transfer = libusb_alloc_transfer(0);
			if (!input.transfer)
				return Fail("Cannot allocate IN transfer");
			libusb_fill_bulk_transfer(input.transfer, device_, 0x81, input.data.data(),
						  64, OnIn, &input, 0);
		}
		query_transfer_ = libusb_alloc_transfer(0);
		if (!query_transfer_)
			return Fail("Cannot allocate diagnostic transfer");
		io_thread_ = std::thread([this] { UsbEvents(); });
		std::unique_lock<std::mutex> lock(start_mutex_);
		start_cv_.wait(lock, [this] { return io_ready_; });
		return !failed_;
	}
	void UsbEvents()
	{
		std::string error;
		if (options_.realtime &&
		    !rt::ConfigureCurrentThread(options_.usb_cpu, options_.usb_priority, error))
			Fail("USB realtime setup failed: " + error);
		actual_usb_cpu_ = sched_getcpu();
		if (!failed_)
			for (auto &input : in_)
				SubmitIn(input);
		{
			std::lock_guard<std::mutex> lock(start_mutex_);
			io_ready_ = true;
		}
		start_cv_.notify_one();
		bool cancelled = false;
		for (;;) {
			if (closing_.load()) {
				// Only this thread runs callbacks. Cancel between event-handler
				// calls so no callback can rearm a transfer after this scan.
				if (!cancelled) {
					CancelPendingTransfers();
					cancelled = true;
				}
				if (!active_transfers_.load())
					break;
			}
			timeval timeout{0, 20000};
			const int rc =
				libusb_handle_events_timeout_completed(context_, &timeout, nullptr);
			if (rc && rc != LIBUSB_ERROR_INTERRUPTED)
				Fail("USB event handling failed: " + std::to_string(rc));
		}
	}
	void SubmitIn(In &input)
	{
		input.pending.store(true);
		active_transfers_.fetch_add(1);
		if (libusb_submit_transfer(input.transfer)) {
			active_transfers_.fetch_sub(1);
			input.pending.store(false);
			Fail("USB IN submission failed");
		}
	}
	static void LIBUSB_CALL OnIn(libusb_transfer *transfer)
	{
		auto &input = *static_cast<In *>(transfer->user_data);
		auto &self = *input.owner;
		input.pending.store(false);
		self.active_transfers_.fetch_sub(1);
		if (self.closing_)
			return;
		if (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
		    transfer->actual_length != 64) {
			self.Fail("USB IN failed, status=" + std::to_string(transfer->status));
			return;
		}
		const auto arrival = rt::MonotonicTimeNs();
		const auto magic = ReadU32(input.data.data());
		if (magic == bridge_wire::kStreamFeedback)
			self.ReceiveFeedback(input.data, arrival);
		else if (magic == bridge_wire::kStreamAck)
			self.ReceiveAck(input.data, arrival);
		else
			self.ReceiveQuery(input.data);
		if (!self.closing_)
			self.SubmitIn(input);
	}
	void ReceiveFeedback(const Packet &packet, uint64_t arrival)
	{
		if (!bridge_wire::ValidFeedback(packet)) {
			Fail("Invalid feedback packet");
			return;
		}
		if (!capturing_.load())
			return;
		const auto seq = ReadU32(packet.data() + 4);
		feedback_gaps_ += feedback_packets_ ? uint32_t(seq - last_feedback_ - 1) : seq;
		last_feedback_ = seq;
		++feedback_packets_;
		for (unsigned i = 0; i < ReadU32(packet.data() + 8); ++i) {
			const auto *frame = packet.data() + bridge_wire::FrameOffset(i);
			const auto motor = unsigned(bridge_wire::FeedbackMotor(frame));
			if (feedback_.size() == feedback_.capacity()) {
				Fail("Feedback capacity exhausted");
				return;
			}
			RmcsLatestState::Motor sample{};
			sample.valid = true;
			sample.received_us = ReadU32(frame + 12);
			sample.host_ns = arrival;
			std::copy_n(frame + 4, 8, sample.data.begin());
			latest_.Put(motor, sample);
			feedback_.push_back({arrival, seq, motor, ReadU32(packet.data() + 12),
					     sample.received_us, sample.data});
		}
	}
	void ReceiveAck(const Packet &packet, uint64_t arrival)
	{
		const auto seq = ReadU32(packet.data() + 4);
		if (!bridge_wire::ValidAck(packet) ||
		    seq >= published_.load(std::memory_order_acquire)) {
			Fail("Invalid command ACK or sequence");
			return;
		}
		if (ReadU32(packet.data() + 24) != bridge_wire::kControlReply ||
		    (!options_.split_out &&
		     ReadU32(packet.data() + 28) != bridge_wire::kMergedControlCapability)) {
			Fail("Flash the updated merged-OUT firmware before this test");
			return;
		}
		auto &record = records_[seq];
		if (record.in_complete_ns) {
			Fail("Duplicate command ACK");
			return;
		}
		record.reply = packet;
		record.bridge_status = ReadU32(packet.data() + 8);
		record.in_complete_ns = arrival;
		last_ack_us_.store(uint32_t((arrival - record.submitted_ns) / 1000),
				   std::memory_order_relaxed);
		acks_.fetch_add(1, std::memory_order_release);
		if (record.bridge_status)
			Fail("MCU rejected a zero command");
	}
	Out *AcquireOut()
	{
		for (auto &slot : out_) {
			bool free = true;
			if (slot.free.compare_exchange_strong(free, false,
							      std::memory_order_acquire))
				return &slot;
		}
		Fail("USB OUT pool exhausted");
		return nullptr;
	}
	void SubmitCommands(const RmcsLatestState::Snapshot &state, uint64_t sample_ns,
			    uint64_t cycle, int64_t late_ns)
	{
		Packet packets[2]{};
		const size_t first = published_.load();
		if (first + 2 > records_.size()) {
			Fail("Command capacity exhausted");
			return;
		}
		for (unsigned group = 0; group < 2; ++group) {
			auto &r = records_[first + group];
			r = {};
			r.sequence = uint32_t(first + group);
			r.cycle = cycle;
			r.group = group;
			r.wakeup_late_ns = late_ns;
			r.state_sample_ns = sample_ns;
			packets[group] = bridge_wire::StreamCommand(
				r.sequence, last_ack_us_.load(std::memory_order_relaxed), group);
			WriteU32(packets[group].data(), bridge_wire::kControlCommand);
			for (unsigned i = 0; i < 3; ++i) {
				const auto &motor = state[group * 3 + i];
				if (!motor.valid)
					continue;
				r.origin_mask |= 1U << i;
				r.origin_us[i] = motor.received_us;
				r.origin_host_ns[i] = motor.host_ns;
				WriteU32(packets[group].data() + bridge_wire::FrameOffset(i) + 12,
					 motor.received_us);
			}
			WriteU32(packets[group].data() + 12, r.origin_mask);
		}
		auto *left = AcquireOut();
		if (!left)
			return;
		auto *right = options_.split_out ? AcquireOut() : nullptr;
		if (options_.split_out && !right) {
			left->free.store(true);
			return;
		}
		if (options_.split_out) {
			std::copy(packets[0].begin(), packets[0].end(), left->data.begin());
			std::copy(packets[1].begin(), packets[1].end(), right->data.begin());
			left->first = first;
			left->count = 1;
			right->first = first + 1;
			right->count = 1;
		} else {
			left->data = bridge_wire::MergeControl(packets[0], packets[1]);
			left->first = first;
			left->count = 2;
		}
		// Both logical groups are fully initialized before any callback can
		// observe them. Each transfer's submit timestamp is set before submit.
		records_[first].submitted_ns = records_[first + 1].submitted_ns =
			rt::MonotonicTimeNs();
		published_.store(first + 2, std::memory_order_release);
		SubmitOut(*left);
		if (right) {
			records_[first + 1].submitted_ns = rt::MonotonicTimeNs();
			SubmitOut(*right);
		}
	}
	void SubmitOut(Out &slot)
	{
		libusb_fill_bulk_transfer(slot.transfer, device_, 0x01, slot.data.data(),
					  slot.count == 2 ? 128 : 64, OnOut, &slot, 250);
		pending_out_.fetch_add(1);
		active_transfers_.fetch_add(1);
		if (libusb_submit_transfer(slot.transfer)) {
			active_transfers_.fetch_sub(1);
			pending_out_.fetch_sub(1);
			slot.free.store(true, std::memory_order_release);
			Fail("USB OUT submission failed");
		} else
			++sent_transfers_;
	}
	static void LIBUSB_CALL OnOut(libusb_transfer *transfer)
	{
		auto &slot = *static_cast<Out *>(transfer->user_data);
		auto &self = *slot.owner;
		const auto now = rt::MonotonicTimeNs();
		if (!self.closing_) {
			if (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
			    transfer->actual_length != transfer->length)
				self.Fail("USB OUT failed, status=" +
					  std::to_string(transfer->status));
			else {
				for (unsigned i = 0; i < slot.count; ++i)
					self.records_[slot.first + i].out_complete_ns = now;
				self.completed_out_.fetch_add(1);
			}
		}
		self.pending_out_.fetch_sub(1);
		self.active_transfers_.fetch_sub(1);
		// Last access: control may reuse this slot immediately after release.
		slot.free.store(true, std::memory_order_release);
	}
	void Capture()
	{
		EventWaiter timer;
		if (!timer.Prepare({})) {
			Fail("Cannot prepare control timerfd");
			return;
		}
		start_ns_ = rt::MonotonicTimeNs();
		uint64_t next = start_ns_, cycle = 0;
		const uint64_t end = start_ns_ + uint64_t(options_.seconds * 1e9);
		if (!timer.Arm(next + kPeriodNs, kPeriodNs)) {
			Fail("Cannot arm control timerfd");
			return;
		}
		capturing_.store(true);
		while (!failed_ && !stop_requested) {
			const auto now = rt::MonotonicTimeNs();
			if (now >= end)
				break;
			if (now >= next) {
				const auto missed = (now - next) / kPeriodNs;
				skipped_ += missed;
				cycle += missed;
				next += missed * kPeriodNs;
				const auto state = latest_.Read();
				const auto sample_ns = rt::MonotonicTimeNs();
				SubmitCommands(state, sample_ns, cycle++, int64_t(now - next));
				next += kPeriodNs;
			}
			if (failed_)
				break;
			const auto wake = timer.Wait();
			if (wake.error && wake.error != -EINTR)
				Fail("Control timerfd wait failed");
		}
		end_ns_ = rt::MonotonicTimeNs();
		if (stop_requested)
			Fail("Interrupted by user");
	}
	static void LIBUSB_CALL OnQueryOut(libusb_transfer *transfer)
	{
		auto &self = *static_cast<RmcsStyleBenchmark *>(transfer->user_data);
		self.active_transfers_.fetch_sub(1);
		self.query_pending_.store(false);
		if (!self.closing_ && (transfer->status != LIBUSB_TRANSFER_COMPLETED ||
				       transfer->actual_length != 64))
			self.Fail("Diagnostic USB OUT failed");
		{
			std::lock_guard<std::mutex> lock(self.query_mutex_);
			self.query_out_done_ = true;
		}
		self.query_cv_.notify_all();
	}
	void ReceiveQuery(const Packet &packet)
	{
		std::lock_guard<std::mutex> lock(query_mutex_);
		if (!query_waiting_ || ReadU32(packet.data()) != expected_magic_ ||
		    ReadU32(packet.data() + 4) != ReadU32(query_request_.data() + 4)) {
			Fail("Unexpected diagnostic reply");
			return;
		}
		query_reply_ = packet;
		query_reply_done_ = true;
		query_cv_.notify_all();
	}
	bool Query(const Packet &request, uint32_t reply_magic, Packet &reply)
	{
		{
			std::lock_guard<std::mutex> lock(query_mutex_);
			query_request_ = request;
			expected_magic_ = reply_magic;
			query_waiting_ = true;
			query_out_done_ = query_reply_done_ = false;
		}
		libusb_fill_bulk_transfer(query_transfer_, device_, 0x01, query_request_.data(), 64,
					  OnQueryOut, this, 2000);
		query_pending_.store(true);
		active_transfers_.fetch_add(1);
		if (libusb_submit_transfer(query_transfer_)) {
			active_transfers_.fetch_sub(1);
			query_pending_.store(false);
			return Fail("Diagnostic submit failed");
		}
		std::unique_lock<std::mutex> lock(query_mutex_);
		if (!query_cv_.wait_for(lock, std::chrono::seconds(2), [this] {
			    return failed_ || (query_out_done_ && query_reply_done_);
		    }))
			return Fail("Diagnostic reply timeout");
		if (failed_)
			return false;
		reply = query_reply_;
		query_waiting_ = false;
		return true;
	}
	void ExportDiagnostics()
	{
		std::cout
			<< "Measurement complete; stopping motors and exporting counters/timestamps"
			<< std::endl;
		for (unsigned page = 0; page < bridge_wire::kStreamStatsPages; ++page) {
			const uint32_t seq = 0xfffffff0U + page;
			if (!Query(bridge_wire::StreamStatsQuery(seq, page),
				   bridge_wire::kStreamStatsReply, stats_[page]))
				return;
			if (!bridge_wire::ValidStreamStats(stats_[page], seq, page)) {
				Fail("Invalid final CAN stats");
				return;
			}
		}
		const auto count = published_.load();
		const auto total = std::count_if(
			records_.begin(), records_.begin() + count, [](const Record &record) {
				return bridge_wire::SampleTiming(record.sequence);
			});
		std::cout << "CAN counters exported; MCU timestamp packets: 0/" << total
			  << std::endl;
		size_t exported = 0;
		for (size_t i = 0; i < count; ++i) {
			auto &record = records_[i];
			if (!bridge_wire::SampleTiming(record.sequence))
				continue;
			if (!Query(bridge_wire::ControlQuery(record.sequence),
				   bridge_wire::kControlReply, record.timing))
				return;
			record.has_timing =
				bridge_wire::ValidControlReply(record.timing, record.sequence);
			if (!record.has_timing) {
				Fail("Invalid MCU control timestamp record");
				return;
			}
			if (hardware_timing_supported_) {
				if (!Query(bridge_wire::HardwareQuery(record.sequence),
					   bridge_wire::kHardwareReply, record.hardware_timing))
					return;
				record.has_hardware_timing = bridge_wire::ValidHardwareReply(
					record.hardware_timing, record.sequence);
				if (!record.has_hardware_timing) {
					Fail("Invalid hardware timing reply");
					return;
				}
			}
			++exported;
			if (exported % 250 == 0 || exported == size_t(total))
				std::cout << "MCU timestamp packets: " << exported << '/' << total
					  << std::endl;
		}
	}
	void CancelPendingTransfers()
	{
		for (auto &in : in_)
			if (in.pending)
				libusb_cancel_transfer(in.transfer);
		for (auto &out : out_)
			if (!out.free && out.transfer)
				libusb_cancel_transfer(out.transfer);
		if (query_pending_ && query_transfer_)
			libusb_cancel_transfer(query_transfer_);
	}
	void CloseUsb()
	{
		if (!context_)
			return;
		// Do not compete for libusb's event lock from a lower-priority thread.
		// The event thread cancels and drains its own transfers before exiting.
		closing_.store(true);
		if (io_thread_.joinable())
			io_thread_.join();
		for (auto &in : in_)
			if (in.transfer) {
				libusb_free_transfer(in.transfer);
				in.transfer = nullptr;
			}
		for (auto &out : out_)
			if (out.transfer) {
				libusb_free_transfer(out.transfer);
				out.transfer = nullptr;
			}
		if (query_transfer_) {
			libusb_free_transfer(query_transfer_);
			query_transfer_ = nullptr;
		}
		if (claimed_)
			libusb_release_interface(device_, 0);
		if (device_)
			libusb_close(device_);
		libusb_exit(context_);
		context_ = nullptr;
	}
	bool WriteResults();

	RmcsStyleOptions options_;
	libusb_context *context_ = nullptr;
	libusb_device_handle *device_ = nullptr;
	bool claimed_ = false;
	bool hardware_timing_supported_ = false;
	std::atomic<bool> closing_{false}, failed_{false}, capturing_{false};
	std::atomic<unsigned> active_transfers_{0}, pending_out_{0};
	std::atomic<size_t> published_{0}, acks_{0};
	std::atomic<uint32_t> last_ack_us_{0};
	std::atomic<uint64_t> completed_out_{0};
	uint64_t sent_transfers_ = 0, skipped_ = 0, start_ns_ = 0, end_ns_ = 0;
	std::string failure_;
	std::mutex failure_mutex_, start_mutex_, query_mutex_;
	std::condition_variable start_cv_, query_cv_;
	bool io_ready_ = false;
	int actual_usb_cpu_ = -1;
	std::thread io_thread_;
	RmcsLatestState latest_;
	std::array<Out, 8> out_{};
	std::array<In, 4> in_{};
	std::vector<Record> records_;
	std::vector<FeedbackRecord> feedback_;
	uint64_t feedback_packets_ = 0, feedback_gaps_ = 0;
	uint32_t last_feedback_ = 0;
	std::array<Packet, bridge_wire::kStreamStatsPages> stats_{};
	libusb_transfer *query_transfer_ = nullptr;
	Packet query_request_{}, query_reply_{};
	std::atomic<bool> query_pending_{false};
	bool query_waiting_ = false, query_out_done_ = false, query_reply_done_ = false;
	uint32_t expected_magic_ = 0;
};

bool RmcsStyleBenchmark::WriteResults()
{
	const double elapsed = end_ns_ > start_ns_ ? (end_ns_ - start_ns_) / 1e9 : 0;
	std::vector<int64_t> acks, wakes;
	unsigned rejected = 0;
	for (const auto &r : records_) {
		if (r.in_complete_ns && !r.bridge_status)
			acks.push_back(r.in_complete_ns - r.submitted_ns);
		wakes.push_back(r.wakeup_late_ns);
		rejected += r.bridge_status != 0;
	}
	const auto ack = rt::Summarize(acks), wake = rt::Summarize(wakes);
	const bool usb_pass = !failed_ && records_.size() > 0 && acks_.load() == records_.size() &&
			      completed_out_.load() == sent_transfers_ && !skipped_;
	bool can_pass = true;
	std::vector<int64_t> ages[6];
	uint32_t exported[6]{};
	for (const auto &r : feedback_) {
		++exported[r.motor];
		ages[r.motor].push_back(int64_t(uint32_t(r.mcu_in_submit_us - r.mcu_rx_us)) * 1000);
	}
	std::ifstream isolated_file("/sys/devices/system/cpu/isolated");
	std::string isolated;
	std::getline(isolated_file, isolated);
	std::ofstream json(options_.json_path);
	json << "{\n  \"can_batch_order\": \"" << bridge_wire::kCanOrderNames[options_.can_order]
	     << "\",\n  \"fast_ingress\": " << (options_.fast_ingress ? "true" : "false") << ",\n";
	json << "  \"mode\": \"rmcs-style-zero-control\",\n"
	     << "  \"usb_transport\": \"bulk\",\n  \"usb_speed_mbps\": 480,\n"
	     << "  \"merged_out\": " << (options_.split_out ? "false" : "true")
	     << ",\n  \"out_order\": \"left-right\",\n  \"feedback_grace_us\": 0,\n"
	     << "  \"dedicated_usb_thread\": true,\n  \"cpu\": " << options_.cpu
	     << ",\n  \"usb_cpu\": " << actual_usb_cpu_
	     << ",\n  \"fifo_priority\": " << options_.priority
	     << ",\n  \"usb_fifo_priority\": " << options_.usb_priority
	     << ",\n  \"realtime\": " << (options_.realtime ? "true" : "false")
	     << ",\n  \"memory_locked\": " << (options_.realtime ? "true" : "false")
	     << ",\n  \"isolated_cpus\": \"" << isolated << "\",\n"
	     << "  \"elapsed_s\": " << elapsed << ",\n  \"sent_commands\": " << records_.size()
	     << ",\n  \"usb_out_transfers\": " << sent_transfers_
	     << ",\n  \"received_acks\": " << acks_.load()
	     << ",\n  \"rejected_commands\": " << rejected
	     << ",\n  \"host_skipped_slots\": " << skipped_
	     << ",\n  \"feedback_packets\": " << feedback_packets_
	     << ",\n  \"feedback_sequence_gaps\": " << feedback_gaps_
	     << ",\n  \"ack_rtt_us_p50\": " << ack.p50_ns / 1000.0
	     << ",\n  \"ack_rtt_us_p99\": " << ack.p99_ns / 1000.0
	     << ",\n  \"ack_rtt_us_max\": " << ack.maximum_ns / 1000.0
	     << ",\n  \"errors\": " << (failed_ ? 1 : 0) << ",\n  \"failure\": \"" << failure_
	     << "\",\n  \"motors\": [\n";
	constexpr const char *names[] = {"left_wheel",  "left_joint_b",  "left_joint_d",
					 "right_wheel", "right_joint_b", "right_joint_d"};
	std::cout << "USB OUT transfers=" << sent_transfers_
		  << " logical commands=" << records_.size() << " ACK=" << acks_
		  << " rejected=" << rejected << " skipped=" << skipped_
		  << " ACK RTT p50/p99/max=" << ack.p50_ns / 1000.0 << '/' << ack.p99_ns / 1000.0
		  << '/' << ack.maximum_ns / 1000.0 << " us wakeup p99=" << wake.p99_ns / 1000.0
		  << " us\n";
	for (unsigned motor = 0; motor < 6; ++motor) {
		const unsigned group = motor / 3, item = motor % 3;
		const auto *r = stats_[group].data() + bridge_wire::FrameOffset(item);
		const auto done = ReadU32(r), rx = ReadU32(r + 4), coalesced = ReadU32(r + 8),
			   errors = ReadU32(r + 12);
		const double seconds = ReadU32(stats_[group].data() + 12) / 1e6;
		const double hz = seconds ? done / seconds : 0;
		const auto age = rt::Summarize(ages[motor]);
		can_pass &= hz >= 990 && !coalesced && !errors && rx && exported[motor];
		std::cout << names[motor] << " CAN=" << hz << " Hz done=" << done
			  << " coalesced=" << coalesced << " errors=" << errors << " CAN_RX=" << rx
			  << " exported=" << exported[motor]
			  << " feedback age at MCU IN p50/p99=" << age.p50_ns / 1000.0 << '/'
			  << age.p99_ns / 1000.0 << " us\n";
		json << "    {\"name\": \"" << names[motor] << "\", \"completed\": " << done
		     << ", \"can_hz\": " << hz << ", \"can_rx\": " << rx
		     << ", \"coalesced\": " << coalesced << ", \"tx_errors\": " << errors
		     << ", \"feedback_updates\": " << exported[motor]
		     << ", \"age_at_mcu_in_submit_us_p50\": " << age.p50_ns / 1000.0
		     << ", \"age_at_mcu_in_submit_us_p99\": " << age.p99_ns / 1000.0 << '}'
		     << (motor == 5 ? "\n" : ",\n");
	}
	json << "  ],\n  \"bus_batches\": [\n";
	bool batch_pass = true;
	for (unsigned group = 0; group < 2; ++group) {
		uint32_t submitted[3]{}, completed[3]{};
		uint64_t completed_frames = 0, motor_frames = 0;
		for (unsigned i = 0; i < 3; ++i) {
			submitted[i] = ReadU32(stats_[group + 2].data() + 16 + i * 4);
			completed[i] = ReadU32(stats_[group + 2].data() + 28 + i * 4);
			completed_frames += (i + 1) * uint64_t(completed[i]);
			motor_frames += ReadU32(stats_[group].data() + bridge_wire::FrameOffset(i));
		}
		batch_pass &= !submitted[0] && !submitted[1] && !completed[0] && !completed[1] &&
			      submitted[2] > 0 && submitted[2] == completed[2] &&
			      completed_frames == motor_frames;
		const unsigned bus = bridge_wire::kLayout[group][0].bus;
		std::cout << "bus=" << bus << " driver batches(1/2/3)=" << submitted[0] << '/'
			  << submitted[1] << '/' << submitted[2] << " completed=" << completed[0]
			  << '/' << completed[1] << '/' << completed[2] << '\n';
		json << "    {\"bus\": " << bus << ", \"submitted_by_size\": [" << submitted[0]
		     << ',' << submitted[1] << ',' << submitted[2] << "], \"completed_by_size\": ["
		     << completed[0] << ',' << completed[1] << ',' << completed[2] << "]}"
		     << (group ? "\n" : ",\n");
	}
	const bool control_pass =
		WriteControlLatencyReport(records_, feedback_, options_.json_path);
	const bool hardware_pass =
		WriteHardwareTimingReport(records_, options_.json_path, hardware_timing_supported_);
	const bool files_pass = WriteCaptureCsv(records_, feedback_, options_.csv_path);
	const bool pass = usb_pass && can_pass && batch_pass && !feedback_gaps_ && control_pass &&
			  hardware_pass && files_pass;
	json << "  ],\n  \"usb_pass\": " << (usb_pass ? "true" : "false")
	     << ",\n  \"can_pass\": " << (can_pass ? "true" : "false")
	     << ",\n  \"group_batching_pass\": " << (batch_pass ? "true" : "false")
	     << ",\n  \"control_latency_enabled\": true,\n  \"control_measurement_pass\": "
	     << (control_pass ? "true" : "false")
	     << ",\n  \"link_pass\": " << (pass ? "true" : "false") << "\n}\n";
	json.flush();
	if (!failure_.empty())
		std::cerr << failure_ << '\n';
	return pass && bool(json);
}
} // namespace

int RunRmcsStyle(const RmcsStyleOptions &options)
{
	RmcsStyleBenchmark benchmark(options);
	return benchmark.Execute();
}
void RequestRmcsStyleStop(int)
{
	stop_requested = 1;
}
} // namespace stress_host
