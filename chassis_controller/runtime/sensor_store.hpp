/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "controller_inputs.h"
#include <array>
#include <pthread.h>
#include <stdexcept>
namespace wbr::host {
struct PeripheralState {
  std::array<uint8_t, 25> onboard_raw{};
  float onboard_temperature_c = 0;
  uint64_t onboard_sample_us = 0;
  std::array<uint8_t, 40> referee_chunk{};
  uint8_t referee_bytes = 0;
  uint64_t referee_sample_us = 0;
};
struct Snapshot {
  modules::ControllerInputs control{};
  PeripheralState peripheral{};
};
class SensorStore {
public:
  SensorStore() {
    pthread_mutexattr_t attributes;
    if (pthread_mutexattr_init(&attributes))
      throw std::runtime_error("mutex attributes");
    const int protocol =
        pthread_mutexattr_setprotocol(&attributes, PTHREAD_PRIO_INHERIT);
    const int init =
        protocol ? protocol : pthread_mutex_init(&mutex_, &attributes);
    pthread_mutexattr_destroy(&attributes);
    if (init)
      throw std::runtime_error("priority inheritance mutex");
  }
  ~SensorStore() { pthread_mutex_destroy(&mutex_); }
  SensorStore(const SensorStore &) = delete;
  SensorStore &operator=(const SensorStore &) = delete;
  void Publish(const Snapshot &value) {
    pthread_mutex_lock(&mutex_);
    value_ = value;
    pthread_mutex_unlock(&mutex_);
  }
  Snapshot Read() {
    pthread_mutex_lock(&mutex_);
    auto copy = value_;
    pthread_mutex_unlock(&mutex_);
    return copy;
  }

private:
  pthread_mutex_t mutex_{};
  Snapshot value_{};
};
} // namespace wbr::host
