/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_benchmark.hpp"
#include <wbr/realtime_loop.hpp>

#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sched.h>
#include <stdexcept>
#include <string>

namespace rt = wbr::realtime;
namespace
{
stress_host::BenchmarkOptions ParseOptions(int argc, char **argv)
{
	stress_host::BenchmarkOptions options;
	for (int index = 1; index < argc; ++index) {
		const std::string key = argv[index];
		if (key == "--right-first") {
			options.right_first = true;
			continue;
		}
		if (key == "--control-latency") {
			options.can_bridge = options.can_stream = options.control_latency = true;
			options.usb_transport = "bulk";
			continue;
		}
		if (key == "--can-stream") {
			options.can_bridge = true;
			options.can_stream = true;
			continue;
		}
		if (key == "--can-bridge") {
			options.can_bridge = true;
			continue;
		}
		if (key == "--no-rt") {
			options.realtime = false;
			continue;
		}
		if (key == "--self-check") {
			options.self_check = true;
			continue;
		}
		if (key == "--help") {
			std::cout << "can_usb_host_rt [--seconds N] [--cpu N] [--priority N] "
				     "[--control-latency] [--can-stream] [--can-bridge] [--no-rt] "
				     "[--self-check] [--right-first] "
				     "[--feedback-grace-us 0..300] [--usb-transport "
				     "auto|interrupt|bulk] [--json PATH] [--csv "
				     "PATH]\n";
			std::exit(0);
		}
		if (++index >= argc) {
			throw std::runtime_error("Missing value for " + key);
		}
		const std::string value = argv[index];
		if (key == "--feedback-grace-us") {
			const auto grace = std::stoul(value);
			if (grace > 300)
				throw std::runtime_error("Feedback grace must be 0..300 us");
			options.feedback_grace_us = static_cast<unsigned>(grace);
		} else if (key == "--usb-transport") {
			options.usb_transport = value;
		} else if (key == "--seconds") {
			options.seconds = std::stod(value);
		} else if (key == "--cpu") {
			options.cpu = std::stoi(value);
		} else if (key == "--priority") {
			options.priority = std::stoi(value);
		} else if (key == "--json") {
			options.json_path = value;
		} else if (key == "--csv") {
			options.csv_path = value;
		} else {
			throw std::runtime_error("Unknown option " + key);
		}
	}
	if (!(options.seconds > 0 && options.seconds <= 3600) || options.cpu < 0 ||
	    options.cpu >= CPU_SETSIZE || options.priority < 1 || options.priority > 99) {
		throw std::runtime_error("Invalid duration, CPU, or FIFO priority");
	}
	if (options.usb_transport != "auto" && options.usb_transport != "interrupt" &&
	    options.usb_transport != "bulk")
		throw std::runtime_error("Invalid USB transport");
	if (options.control_latency && (options.seconds > 65 || options.usb_transport != "bulk"))
		throw std::runtime_error("Control latency requires Bulk and duration <= 65 s");
	if (options.feedback_grace_us > 300 ||
	    (options.feedback_grace_us && !options.control_latency))
		throw std::runtime_error(
			"Feedback grace requires --control-latency and range 0..300 us");
	if (options.right_first && !options.can_bridge)
		throw std::runtime_error("--right-first requires a two-bus CAN bridge mode");
	return options;
}

std::string IsolatedCpus()
{
	std::ifstream stream("/sys/devices/system/cpu/isolated");
	std::string cpus;
	std::getline(stream, cpus);
	return cpus;
}

} // namespace

int main(int argc, char **argv)
{
	try {
		const auto options = ParseOptions(argc, argv);
		stress_host::UsbBenchmark benchmark(options);
		std::string error;
		if (options.realtime &&
		    !rt::ConfigureCurrentThread(options.cpu, options.priority, error)) {
			std::cerr << "Realtime setup failed: " << error
				  << "; run with realtime privileges, or use --no-rt for a "
				     "labelled baseline\n";
			return 1;
		}
		std::cout << "Native async USB: scheduler=" << sched_getscheduler(0)
			  << " cpu=" << sched_getcpu()
			  << " FIFO priority=" << (options.realtime ? options.priority : 0)
			  << " memory_locked=" << options.realtime << " isolated_cpus=\""
			  << IsolatedCpus() << "\"; no disk writes during measurement" << std::endl;
		if (options.self_check) {
			return 0;
		}
		if (!benchmark.Open()) {
			std::cerr << "Cannot open USB session\n";
			return 1;
		}
		std::signal(SIGINT, stress_host::UsbBenchmark::RequestStop);
		std::signal(SIGTERM, stress_host::UsbBenchmark::RequestStop);
		benchmark.Run();
		// Reporting and file I/O must not run at FIFO priority.
		sched_param normal{};
		sched_setscheduler(0, SCHED_OTHER, &normal);
		return benchmark.WriteResults() ? 0 : 1;
	} catch (const std::exception &error) {
		std::cerr << error.what() << '\n';
		return 1;
	}
}
