/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>
namespace platform {
enum class CanTxSlot : uint8_t { kLeftWheel, kRightWheel, kLeftJointB, kLeftJointD, kRightJointB, kRightJointD, kCount };
}
