/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>
#include <mutex>
#include <pthread.h>
#include <stdexcept>
namespace stress_host
{
// Short, priority-inheritance-protected cache. USB owns updates, the control
// thread takes one six-motor snapshot, then releases the lock BEFORE computing
// or submitting USB. Raw state and both timestamps are copied together.
class RmcsLatestState
{
public:
	struct Motor {
		bool valid = false;
		uint32_t received_us = 0;
		uint64_t host_ns = 0;
		std::array<unsigned char, 8> data{};
	};
	using Snapshot = std::array<Motor, 6>;
	RmcsLatestState()
	{
		pthread_mutexattr_t attributes;
		if (pthread_mutexattr_init(&attributes))
			throw std::runtime_error("mutexattr init failed");
		int error = pthread_mutexattr_setprotocol(&attributes, PTHREAD_PRIO_INHERIT);
		if (!error)
			error = pthread_mutex_init(&mutex_, &attributes);
		pthread_mutexattr_destroy(&attributes);
		if (error)
			throw std::runtime_error("Priority-inheritance cache mutex unavailable");
	}
	~RmcsLatestState()
	{
		pthread_mutex_destroy(&mutex_);
	}
	RmcsLatestState(const RmcsLatestState &) = delete;
	RmcsLatestState &operator=(const RmcsLatestState &) = delete;
	void Put(unsigned motor, const Motor &sample)
	{
		pthread_mutex_lock(&mutex_);
		motors_[motor] = sample;
		pthread_mutex_unlock(&mutex_);
	}
	Snapshot Read()
	{
		pthread_mutex_lock(&mutex_);
		const auto snapshot = motors_;
		pthread_mutex_unlock(&mutex_);
		return snapshot;
	}

private:
	pthread_mutex_t mutex_{};
	Snapshot motors_{};
};
} // namespace stress_host
