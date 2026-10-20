/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "motor_address.h"
#include <cstddef>
namespace modules {
class FrameSink {
public:
  virtual ~FrameSink() = default;
  virtual int Submit(platform::CanTxSlot slot, uint8_t bus, uint16_t id,
                     const uint8_t *data, size_t length) = 0;
};
} // namespace modules
