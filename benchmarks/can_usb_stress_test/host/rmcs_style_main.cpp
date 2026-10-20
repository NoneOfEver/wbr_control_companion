/* SPDX-License-Identifier: Apache-2.0 */
#include "rmcs_style_benchmark.hpp"
#include "bridge_profile.hpp"
#include <csignal>
#include <iostream>
#include <sched.h>
#include <stdexcept>
int main(int argc, char **argv)
{
	try {
		stress_host::RmcsStyleOptions options;
		for (int i = 1; i < argc; ++i) {
			const std::string key = argv[i];
			if (key == "--fast-ingress") {
				options.fast_ingress = true;
				continue;
			}
			if (key == "--split-out") {
				options.split_out = true;
				continue;
			}
			if (key == "--no-rt") {
				options.realtime = false;
				continue;
			}
			if (key == "--help") {
				std::cout << "can_usb_rmcs_host_rt [--seconds N] [--cpu N] "
					     "[--priority N] "
					     "[--usb-cpu N] [--usb-priority N] [--split-out] "
					     "[--no-rt] [--can-order wbd|wdb|bwd|bdw|dwb|dbw] "
					     "[--fast-ingress] "
					     "[--json PATH] [--csv PATH]\n";
				return 0;
			}
			if (++i == argc)
				throw std::runtime_error("Missing value for " + key);
			const std::string value = argv[i];
			if (key == "--can-order") {
				bool found = false;
				for (unsigned order = 0; order < 6; ++order)
					if (value == bridge_wire::kCanOrderNames[order]) {
						options.can_order = order;
						found = true;
						break;
					}
				if (!found)
					throw std::runtime_error("Invalid CAN order: " + value);
			} else if (key == "--seconds")
				options.seconds = std::stod(value);
			else if (key == "--cpu")
				options.cpu = std::stoi(value);
			else if (key == "--priority")
				options.priority = std::stoi(value);
			else if (key == "--usb-cpu")
				options.usb_cpu = std::stoi(value);
			else if (key == "--usb-priority")
				options.usb_priority = std::stoi(value);
			else if (key == "--json")
				options.json_path = value;
			else if (key == "--csv")
				options.csv_path = value;
			else
				throw std::runtime_error("Unknown option " + key);
		}
		if (!(options.seconds > 0 && options.seconds <= 65) || options.cpu < 0 ||
		    options.cpu >= CPU_SETSIZE || options.usb_cpu < 0 ||
		    options.usb_cpu >= CPU_SETSIZE || options.priority < 1 ||
		    options.priority > 99 || options.usb_priority < 1 || options.usb_priority > 99)
			throw std::runtime_error("Invalid duration, CPU or priority");
		std::signal(SIGINT, stress_host::RequestRmcsStyleStop);
		std::signal(SIGTERM, stress_host::RequestRmcsStyleStop);
		return stress_host::RunRmcsStyle(options);
	} catch (const std::exception &e) {
		std::cerr << e.what() << '\n';
		return 1;
	}
}
