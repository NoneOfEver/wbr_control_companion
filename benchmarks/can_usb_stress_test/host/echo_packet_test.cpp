/* SPDX-License-Identifier: Apache-2.0 */
#include "echo_packet.hpp"
#include <algorithm>
#include <iostream>

int main()
{
	// Python/firmware fixture: magic, sequence=17, previous RTT=2048 (LE),
	// followed by exactly 52 payload bytes 17..68.
	const unsigned char header[] = {0x53, 0x52, 0x54, 0x53, 17, 0, 0, 0, 0, 8, 0, 0};
	const auto packet = stress_host::EncodePacket(17, 2048);
	if (!std::equal(std::begin(header), std::end(header), packet.begin())) {
		return 1;
	}
	for (unsigned index = 12; index < 64; ++index) {
		if (packet[index] != index + 5) {
			return 2;
		}
	}
	const auto wrapped = stress_host::EncodePacket(0xffffffffU, 0x12345678U);
	if (stress_host::ReadU32(wrapped.data() + 4) != 0xffffffffU ||
	    stress_host::ReadU32(wrapped.data() + 8) != 0x12345678U || wrapped[12] != 255) {
		return 3;
	}
	std::cout << "Packet matches Python/firmware wire contract\n";
}
