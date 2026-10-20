/* SPDX-License-Identifier: Apache-2.0 */
#include "rmcs_style_state.hpp"
#include <atomic>
#include <iostream>
#include <thread>
int main()
{
	stress_host::RmcsLatestState cache;
	std::atomic<bool> done{false};
	std::thread usb([&] {
		for (uint32_t i = 1; i < 50000; ++i) {
			stress_host::RmcsLatestState::Motor sample{};
			sample.valid = true;
			sample.received_us = i;
			sample.host_ns = uint64_t(i) * 1000 + 7;
			sample.data.fill(static_cast<unsigned char>(i));
			cache.Put(0, sample);
		}
		done.store(true);
	});
	bool valid = true;
	unsigned reads = 0;
	do {
		const auto state = cache.Read();
		++reads;
		const auto &m = state[0];
		if (m.valid) {
			valid &= m.host_ns == uint64_t(m.received_us) * 1000 + 7;
			for (auto value : m.data)
				valid &= value == static_cast<unsigned char>(m.received_us);
		}
	} while (!done.load());
	usb.join();
	if (!valid || !reads || cache.Read()[0].received_us != 49999)
		return 1;
	std::cout << "Concurrent USB updates preserve coherent raw state and timestamps\n";
}
