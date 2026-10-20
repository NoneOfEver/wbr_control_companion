/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <cstdint>
namespace protocols { inline uint16_t ReadBigEndian16(const uint8_t *p) { return (uint16_t(p[0]) << 8) | p[1]; } }
