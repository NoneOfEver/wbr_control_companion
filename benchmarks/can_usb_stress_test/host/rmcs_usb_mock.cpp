/* SPDX-License-Identifier: Apache-2.0 */
// Test-only libusb replacement: no device can be opened through this backend.
#include "merged_control.hpp"
#include "bridge_profile.hpp"
#include <libusb.h>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>
struct libusb_context {
};
struct libusb_device {
};
struct libusb_device_handle {
};
namespace
{
using bridge_wire::Packet;
struct Mock {
	std::mutex queue_mutex, event_mutex;
	std::condition_variable ready;
	std::deque<libusb_transfer *> inputs, outputs, cancelled;
	std::deque<Packet> replies;
	std::unique_ptr<bridge_wire::ControlLatencyRecorder> recorder;
	uint32_t commands[2]{}, feedback_sequence = 0;
	static uint32_t Now()
	{
		return uint32_t(std::chrono::duration_cast<std::chrono::microseconds>(
					std::chrono::steady_clock::now().time_since_epoch())
					.count());
	}
	void Command(const Packet &request, unsigned group, bool merged, uint32_t out_us)
	{
		recorder->Stage(request, group, out_us);
		Packet ack{};
		bridge_wire::WriteU32(ack.data(), bridge_wire::kStreamAck);
		bridge_wire::WriteU32(ack.data() + 4, bridge_wire::ReadU32(request.data() + 4));
		bridge_wire::WriteU32(ack.data() + 12, out_us);
		bridge_wire::WriteU32(ack.data() + 20, 2);
		bridge_wire::WriteU32(ack.data() + 24, bridge_wire::kControlReply);
		if (merged)
			bridge_wire::WriteU32(ack.data() + 28,
					      bridge_wire::kMergedControlCapability);
		replies.push_back(ack);
		Packet feedback{};
		bridge_wire::WriteU32(feedback.data(), bridge_wire::kStreamFeedback);
		bridge_wire::WriteU32(feedback.data() + 4, feedback_sequence++);
		bridge_wire::WriteU32(feedback.data() + 8, 3);
		bridge_wire::WriteU32(feedback.data() + 12, out_us + 4);
		for (unsigned i = 0; i < 3; ++i) {
			recorder->Driver(group * 3 + i, out_us + 1);
			recorder->HardwareStart(group * 3 + i, out_us + 1);
			recorder->BatchDone(group * 3 + i, out_us + 2);
			recorder->Receive(group * 3 + i, out_us + 3);
			const auto &layout = bridge_wire::kLayout[group][i];
			auto *frame = feedback.data() + bridge_wire::FrameOffset(i);
			bridge_wire::SetAddress(frame, layout.bus, layout.feedback_id, 8);
			bridge_wire::WriteU32(frame + 12, out_us + 3);
		}
		replies.push_back(feedback);
		++commands[group];
	}
	void Process(libusb_transfer *transfer)
	{
		const uint32_t receive_us = Now();
		const int length = transfer->length;
		Packet request{};
		std::copy_n(transfer->buffer, 64, request.begin());
		const auto magic = bridge_wire::ReadU32(request.data());
		if (magic == bridge_wire::kMergedCapabilityQuery) {
			Packet response{};
			bridge_wire::WriteU32(response.data(), bridge_wire::kMergedCapabilityReply);
			bridge_wire::WriteU32(response.data() + 4,
					      bridge_wire::ReadU32(request.data() + 4));
			bridge_wire::WriteU32(response.data() + 8,
					      bridge_wire::kMergedControlCapability);
			bridge_wire::WriteU32(response.data() + 12,
					      std::getenv("RMCS_MOCK_CAPACITY_64") ? 64 : 128);
			bridge_wire::WriteU32(response.data() + 16, 64);
			bridge_wire::WriteU32(response.data() + 24, bridge_wire::kProfileReply);
			if (!std::getenv("RMCS_MOCK_NO_HARDWARE"))
				bridge_wire::WriteU32(response.data() + 20,
						      bridge_wire::kHardwareReply);
			replies.push_back(response);
		} else if (magic == bridge_wire::kProfileQuery) {
			Packet response = request;
			bridge_wire::WriteU32(response.data(), bridge_wire::kProfileReply);
			const auto order = bridge_wire::ReadU32(request.data() + 8);
			const auto fast = bridge_wire::ReadU32(request.data() + 12);
			bridge_wire::WriteU32(response.data() + 16,
					      (!bridge_wire::ValidProfile(order, fast) ||
					       commands[0] || commands[1])
						      ? 1
						      : 0);
			replies.push_back(response);
		} else if (magic == bridge_wire::kControlCommand) {
			if (length == 128) {
				if (std::getenv("RMCS_MOCK_CAPACITY_64"))
					std::abort();
				bridge_wire::MergedControl raw{};
				std::copy_n(transfer->buffer, 128, raw.begin());
				Packet groups[2]{};
				if (!bridge_wire::SplitMergedControl(raw, groups))
					std::abort();
				Command(groups[0], 0, true, receive_us);
				Command(groups[1], 1, true, receive_us);
			} else {
				const auto bus = request[16];
				Command(request, bus == bridge_wire::kLayout[0][0].bus ? 0 : 1,
					false, receive_us);
			}
		} else if (magic == bridge_wire::kHardwareQuery) {
			if (std::getenv("RMCS_MOCK_NO_HARDWARE"))
				std::abort();
			replies.push_back(
				recorder->QueryHardware(bridge_wire::ReadU32(request.data() + 4)));
		} else if (magic == bridge_wire::kControlQuery) {
			replies.push_back(
				recorder->Query(bridge_wire::ReadU32(request.data() + 4)));
		} else if (magic == bridge_wire::kStreamStatsQuery) {
			const auto page = bridge_wire::ReadU32(request.data() + 8);
			Packet response{};
			bridge_wire::WriteU32(response.data(), bridge_wire::kStreamStatsReply);
			bridge_wire::WriteU32(response.data() + 4,
					      bridge_wire::ReadU32(request.data() + 4));
			bridge_wire::WriteU32(response.data() + 8, page);
			bridge_wire::WriteU32(response.data() + 12, commands[0] * 1000);
			if (page < 2) {
				for (unsigned i = 0; i < 3; ++i) {
					bridge_wire::WriteU32(response.data() + 16 + i * 16,
							      commands[page]);
					bridge_wire::WriteU32(response.data() + 20 + i * 16,
							      commands[page]);
				}
			} else {
				bridge_wire::WriteU32(response.data() + 24, commands[page - 2]);
				bridge_wire::WriteU32(response.data() + 36, commands[page - 2]);
				bridge_wire::WriteU32(response.data() + 40,
						      bridge_wire::kLayout[page - 2][0].bus);
			}
			replies.push_back(response);
		} else
			std::abort();
	}
} mock;
libusb_context context;
libusb_device_handle handle;
libusb_device device;
} // namespace
extern "C" {
int LIBUSB_CALL libusb_init(libusb_context **out)
{
	mock.recorder = std::make_unique<bridge_wire::ControlLatencyRecorder>();
	mock.commands[0] = mock.commands[1] = mock.feedback_sequence = 0;
	*out = &context;
	return 0;
}
void LIBUSB_CALL libusb_exit(libusb_context *)
{
	// Every submitted transfer must finish its callback before context teardown.
	std::lock_guard<std::mutex> lock(mock.queue_mutex);
	if (!mock.inputs.empty() || !mock.outputs.empty() || !mock.cancelled.empty())
		std::abort();
}
libusb_device_handle *LIBUSB_CALL libusb_open_device_with_vid_pid(libusb_context *, uint16_t,
								  uint16_t)
{
	return &handle;
}
libusb_device *LIBUSB_CALL libusb_get_device(libusb_device_handle *)
{
	return &device;
}
int LIBUSB_CALL libusb_get_device_speed(libusb_device *)
{
	return LIBUSB_SPEED_HIGH;
}
int LIBUSB_CALL libusb_get_configuration(libusb_device_handle *, int *value)
{
	*value = 1;
	return 0;
}
int LIBUSB_CALL libusb_set_configuration(libusb_device_handle *, int)
{
	return 0;
}
int LIBUSB_CALL libusb_claim_interface(libusb_device_handle *, int)
{
	return 0;
}
int LIBUSB_CALL libusb_release_interface(libusb_device_handle *, int)
{
	return 0;
}
void LIBUSB_CALL libusb_close(libusb_device_handle *)
{
}
int LIBUSB_CALL libusb_get_active_config_descriptor(libusb_device *, libusb_config_descriptor **out)
{
	static libusb_endpoint_descriptor endpoints[2]{};
	endpoints[0].bEndpointAddress = 0x81;
	endpoints[1].bEndpointAddress = 0x01;
	for (auto &ep : endpoints) {
		ep.bmAttributes = LIBUSB_TRANSFER_TYPE_BULK;
		ep.wMaxPacketSize = 512;
	}
	static libusb_interface_descriptor alt{};
	alt.bNumEndpoints = 2;
	alt.endpoint = endpoints;
	static libusb_interface interface{};
	interface.altsetting = &alt;
	interface.num_altsetting = 1;
	auto *config = new libusb_config_descriptor{};
	config->bNumInterfaces = 1;
	config->interface = &interface;
	*out = config;
	return 0;
}
void LIBUSB_CALL libusb_free_config_descriptor(libusb_config_descriptor *config)
{
	delete config;
}
libusb_transfer *LIBUSB_CALL libusb_alloc_transfer(int)
{
	return static_cast<libusb_transfer *>(std::calloc(1, sizeof(libusb_transfer)));
}
void LIBUSB_CALL libusb_free_transfer(libusb_transfer *transfer)
{
	std::free(transfer);
}
void LIBUSB_CALL libusb_lock_events(libusb_context *)
{
	// The shutdown regression must not depend on acquiring the event thread's
	// lock from the lower-priority control thread.
	if (std::getenv("RMCS_MOCK_FORBID_EVENT_LOCK"))
		std::abort();
	mock.event_mutex.lock();
}
void LIBUSB_CALL libusb_unlock_events(libusb_context *)
{
	mock.event_mutex.unlock();
}
int LIBUSB_CALL libusb_submit_transfer(libusb_transfer *transfer)
{
	std::lock_guard<std::mutex> lock(mock.queue_mutex);
	if (transfer->endpoint == 0x81)
		mock.inputs.push_back(transfer);
	else
		mock.outputs.push_back(transfer);
	mock.ready.notify_all();
	return 0;
}
int LIBUSB_CALL libusb_cancel_transfer(libusb_transfer *transfer)
{
	std::lock_guard<std::mutex> lock(mock.queue_mutex);
	for (auto *queue : {&mock.inputs, &mock.outputs}) {
		const auto found = std::find(queue->begin(), queue->end(), transfer);
		if (found != queue->end()) {
			queue->erase(found);
			mock.cancelled.push_back(transfer);
			mock.ready.notify_all();
			return 0;
		}
	}
	return LIBUSB_ERROR_NOT_FOUND;
}
int LIBUSB_CALL libusb_handle_events_timeout_completed(libusb_context *, timeval *timeout, int *)
{
	{
		std::unique_lock<std::mutex> lock(mock.queue_mutex);
		mock.ready.wait_for(lock,
				    std::chrono::seconds(timeout->tv_sec) +
					    std::chrono::microseconds(timeout->tv_usec),
				    [] {
					    return !mock.cancelled.empty() ||
						   !mock.outputs.empty() ||
						   (!mock.inputs.empty() && !mock.replies.empty());
				    });
	}
	std::lock_guard<std::mutex> event_lock(mock.event_mutex);
	libusb_transfer *transfer = nullptr;
	{
		std::lock_guard<std::mutex> lock(mock.queue_mutex);
		if (!mock.cancelled.empty()) {
			transfer = mock.cancelled.front();
			mock.cancelled.pop_front();
			transfer->status = LIBUSB_TRANSFER_CANCELLED;
			transfer->actual_length = 0;
		} else if (!mock.outputs.empty()) {
			transfer = mock.outputs.front();
			mock.outputs.pop_front();
			mock.Process(transfer);
			transfer->status = LIBUSB_TRANSFER_COMPLETED;
			transfer->actual_length = transfer->length;
		} else if (!mock.inputs.empty() && !mock.replies.empty()) {
			transfer = mock.inputs.front();
			mock.inputs.pop_front();
			std::copy(mock.replies.front().begin(), mock.replies.front().end(),
				  transfer->buffer);
			mock.replies.pop_front();
			transfer->status = LIBUSB_TRANSFER_COMPLETED;
			transfer->actual_length = 64;
		}
	}
	if (transfer)
		transfer->callback(transfer);
	return 0;
}
} // extern C
