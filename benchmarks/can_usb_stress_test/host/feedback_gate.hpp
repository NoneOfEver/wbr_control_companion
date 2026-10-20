/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <cstdint>

namespace stress_host
{
// Optional bounded sampling window within each nominal 1 ms control cycle.
// While waiting, the owner keeps servicing USB. Missing feedback never blocks
// beyond the absolute deadline; no extra command cycle is inserted.
class FeedbackGate
{
public:
	static constexpr uint64_t kMaximumFeedbackAgeNs = 500000;
	struct Decision {
		bool send = false, expired = false;
		uint64_t waited_ns = 0;
	};
	void Begin(uint64_t nominal_ns, uint64_t observed_ns, unsigned grace_us)
	{
		active_ = true;
		observed_ns_ = observed_ns;
		deadline_ns_ = nominal_ns + uint64_t(grace_us) * 1000;
		wakeup_late_ns_ = observed_ns - nominal_ns;
	}
	Decision Evaluate(uint64_t now_ns, const std::array<uint32_t, 2> &rx_us,
			  const std::array<bool, 2> &valid,
			  const std::array<uint64_t, 2> &host_receive_ns) const
	{
		// An unused sample may still be almost one whole cycle old. Requiring
		// both novelty and age prevents that case from bypassing the window.
		const bool recent = now_ns >= host_receive_ns[0] && now_ns >= host_receive_ns[1] &&
				    now_ns - host_receive_ns[0] <= kMaximumFeedbackAgeNs &&
				    now_ns - host_receive_ns[1] <= kMaximumFeedbackAgeNs;
		const bool fresh = recent && valid[0] && valid[1] &&
				   (!previous_valid_[0] || rx_us[0] != previous_us_[0]) &&
				   (!previous_valid_[1] || rx_us[1] != previous_us_[1]);
		// Prime without waiting: commands may be needed to produce feedback.
		const bool expired = now_ns >= deadline_ns_;
		return {!primed_ || fresh || expired, primed_ && !fresh && expired,
			now_ns - observed_ns_};
	}
	void Commit(const std::array<uint32_t, 2> &rx_us,
		    const std::array<bool, 2> &valid = {true, true})
	{
		previous_us_ = rx_us;
		previous_valid_ = valid;
		primed_ = true;
		active_ = false;
	}
	bool Active() const
	{
		return active_;
	}
	uint64_t DeadlineNs() const
	{
		return deadline_ns_;
	}
	uint64_t WakeupLateNs() const
	{
		return wakeup_late_ns_;
	}

private:
	bool active_ = false, primed_ = false;
	std::array<uint32_t, 2> previous_us_{};
	std::array<bool, 2> previous_valid_{};
	uint64_t observed_ns_ = 0, deadline_ns_ = 0, wakeup_late_ns_ = 0;
};
} // namespace stress_host
