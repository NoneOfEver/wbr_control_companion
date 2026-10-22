/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>
#include <wbr_bridge/bridge_protocol.hpp>
namespace wbr::host {
// Qualify a new sensor epoch without treating one reordered frame as a reboot.
class Hi91TimeGuard {
public:
  void Reset() { *this = {}; }
  void Reject() { candidates_ = 0; }
  bool Accept(uint32_t device_ms, uint64_t sample_us, uint64_t timeout_us,
              bool crc_valid) {
    if (!has_time_ || bridge::NewSequence(device_ms, accepted_ms_)) {
      Commit(device_ms, sample_us);
      return true;
    }
    if (!crc_valid || device_ms == accepted_ms_ || sample_us <= accepted_us_ ||
        sample_us - accepted_us_ <= timeout_us) {
      Reject();
      return false;
    }
    const bool consecutive = candidates_ && sample_us >= candidate_us_ &&
                             sample_us - candidate_us_ <= 20000 &&
                             bridge::NewSequence(device_ms, candidate_ms_) &&
                             uint32_t(device_ms - candidate_ms_) <= 20;
    if (!consecutive) {
      candidates_ = 1;
      first_candidate_us_ = sample_us;
    } else if (candidates_ < 3) {
      ++candidates_;
    }
    candidate_ms_ = device_ms;
    candidate_us_ = sample_us;
    // Require a real observation interval, not three buffered frames at once.
    if (candidates_ < 3 || sample_us - first_candidate_us_ < 2000)
      return false;
    Commit(device_ms, sample_us);
    return true;
  }

private:
  void Commit(uint32_t device_ms, uint64_t sample_us) {
    has_time_ = true;
    accepted_ms_ = device_ms;
    accepted_us_ = sample_us;
    Reject();
  }
  bool has_time_ = false;
  uint32_t accepted_ms_ = 0, candidate_ms_ = 0;
  unsigned candidates_ = 0;
  uint64_t accepted_us_ = 0, candidate_us_ = 0, first_candidate_us_ = 0;
};
} // namespace wbr::host
