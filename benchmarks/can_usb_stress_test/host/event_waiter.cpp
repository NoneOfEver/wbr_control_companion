/* SPDX-License-Identifier: Apache-2.0 */
#include "event_waiter.hpp"

#include <cerrno>
#include <sys/timerfd.h>
#include <unistd.h>
#include <utility>

namespace stress_host
{

EventWaiter::~EventWaiter()
{
	if (timer_fd_ >= 0) {
		close(timer_fd_);
	}
}

bool EventWaiter::Prepare(std::vector<pollfd> sources)
{
	timer_fd_ = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
	if (timer_fd_ < 0) {
		return false;
	}
	sources_ = std::move(sources);
	sources_.push_back({timer_fd_, POLLIN, 0});
	return true;
}

bool EventWaiter::Arm(std::uint64_t first_deadline_ns, std::uint64_t period_ns)
{
	itimerspec timer{};
	timer.it_value.tv_sec = first_deadline_ns / 1000000000ULL;
	timer.it_value.tv_nsec = first_deadline_ns % 1000000000ULL;
	timer.it_interval.tv_sec = period_ns / 1000000000ULL;
	timer.it_interval.tv_nsec = period_ns % 1000000000ULL;
	return timerfd_settime(timer_fd_, TFD_TIMER_ABSTIME, &timer, nullptr) == 0;
}

WaitResult EventWaiter::Wait()
{
	if (poll(sources_.data(), sources_.size(), -1) < 0) {
		return {-errno, 0};
	}
	WaitResult result;
	if ((sources_.back().revents & POLLIN) != 0) {
		const auto bytes = read(timer_fd_, &result.timer_expirations,
					sizeof(result.timer_expirations));
		if (bytes != static_cast<ssize_t>(sizeof(result.timer_expirations))) {
			result.error = bytes < 0 ? -errno : -EIO;
		}
	}
	return result;
}

void EventWaiter::Stop()
{
	if (timer_fd_ >= 0) {
		const itimerspec timer{};
		(void)timerfd_settime(timer_fd_, 0, &timer, nullptr);
	}
}

} // namespace stress_host
