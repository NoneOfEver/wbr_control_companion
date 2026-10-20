/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstdint>
#include <poll.h>
#include <vector>

namespace stress_host
{

struct WaitResult {
	int error = 0; // Negative errno; -EINTR lets the caller handle stop signals.
	std::uint64_t timer_expirations = 0;
};

// A kernel timerfd wakes poll at the absolute command deadline. poll's timeout
// stays infinite, so libusb's millisecond timeout rounding cannot pace sends.
// USB descriptors are borrowed; the timer descriptor is owned by this class.
class EventWaiter
{
public:
	EventWaiter() = default;
	~EventWaiter();
	EventWaiter(const EventWaiter &) = delete;
	EventWaiter &operator=(const EventWaiter &) = delete;
	bool Prepare(std::vector<pollfd> sources);
	bool Arm(std::uint64_t first_deadline_ns, std::uint64_t period_ns);
	WaitResult Wait();
	void Stop();

private:
	int timer_fd_ = -1;
	std::vector<pollfd> sources_;
};

} // namespace stress_host
