/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <string>
namespace stress_host
{
struct RmcsStyleOptions {
	double seconds = 60;
	int cpu = 3, priority = 80;
	int usb_cpu = 3, usb_priority = 80;
	bool realtime = true, split_out = false;
	unsigned can_order = 0;
	bool fast_ingress = false;
	std::string json_path = "stream.json", csv_path = "commands.csv";
};
int RunRmcsStyle(const RmcsStyleOptions &options);
void RequestRmcsStyleStop(int);
} // namespace stress_host
