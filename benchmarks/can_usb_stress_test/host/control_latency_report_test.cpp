/* SPDX-License-Identifier: Apache-2.0 */
#include "control_latency_report.hpp"
#include <unistd.h>
#include <sstream>
struct Command {
	uint32_t sequence = 0;
	uint64_t cycle = 0, submitted_ns = 1000000;
	unsigned group = 0;
	uint64_t feedback_wait_ns = 0;
	bool feedback_wait_expired = false;
	bool has_timing = true;
	bridge_wire::Packet timing{}, reply{};
	uint32_t origin_mask = 7;
	std::array<uint32_t, 3> origin_us{100, 100, 100};
	std::array<uint64_t, 3> origin_host_ns{800000, 800000, 800000};
};
struct Feedback {
	unsigned motor;
	uint32_t mcu_rx_us;
	uint64_t host_ns;
};
int main()
{
	using namespace bridge_wire;
	static ControlLatencyRecorder recorder;
	std::vector<Command> commands(2);
	std::vector<Feedback> feedback;
	for (unsigned group = 0; group < 2; ++group) {
		auto &r = commands[group];
		r.sequence = r.group = group;
		auto p = StreamCommand(group, 0, group);
		WriteU32(p.data(), kControlCommand);
		WriteU32(p.data() + 12, 7);
		WriteU32(r.reply.data() + 12, 600);
		for (unsigned i = 0; i < 3; ++i)
			WriteU32(p.data() + FrameOffset(i) + 12, 100);
		recorder.Stage(p, group, 600);
		for (unsigned i = 0; i < 3; ++i) {
			const unsigned motor = group * 3 + i;
			recorder.Driver(motor, 620);
			recorder.BatchDone(motor, 1000 + group * 50);
			recorder.Receive(motor, 1100);
			// An earlier unrelated host feedback must not be used for RTT.
			feedback.push_back({motor, 1090, 1200000});
			feedback.push_back({motor, 1100, 1600000});
		}
		r.timing = recorder.Query(group);
	}
	const auto dir = std::filesystem::temp_directory_path() /
			 ("control-latency-report-" + std::to_string(getpid()));
	std::filesystem::create_directory(dir);
	auto run = [&] {
		return stress_host::WriteControlLatencyReport(commands, feedback,
							      (dir / "stream.json").string());
	};
	bool ok = run();
	std::ifstream file(dir / "control_latency.json");
	std::stringstream text;
	text << file.rdbuf();
	// 900us MCU loop, 200us host hold, 600us host observation; all six sampled.
	ok &= text.str().find("\"p50_us\": 900") != std::string::npos;
	ok &= text.str().find("\"p50_us\": 200") != std::string::npos;
	ok &= text.str().find("\"p50_us\": 600") != std::string::npos;
	ok &= text.str().find("\"measurement_pass\": true") != std::string::npos;
	ok &= text.str().find("\"p50_us\": 950") != std::string::npos;
	// Control-cycle joining and warmup accounting must not depend on bus order.
	std::reverse(commands.begin(), commands.end());
	ok &= run();
	std::ifstream reversed(dir / "control_latency.json");
	std::stringstream reversed_text;
	reversed_text << reversed.rdbuf();
	ok &= reversed_text.str().find("\"reused_or_missing_wheel_inputs\": 0") !=
	      std::string::npos;
	std::reverse(commands.begin(), commands.end());
	// An unexported CAN feedback is explicit; it cannot corrupt primary timing.
	feedback.pop_back();
	ok &= run();
	std::ifstream missing(dir / "control_latency.json");
	std::stringstream missing_text;
	missing_text << missing.rdbuf();
	ok &= missing_text.str().find("\"unexported_post_tx_feedback\": 1") != std::string::npos;
	// A wrong origin must invalidate the measurement instead of producing a
	// deceptively small result, and missing trace exports must fail as well.
	commands[0].origin_us[0] = 101;
	ok &= !run();
	commands[0].origin_us[0] = 100;
	commands[1].has_timing = false;
	ok &= !run();
	std::filesystem::remove_all(dir);
	if (!ok) {
		std::cerr << "Control latency reporting contract failed\n";
		return 1;
	}
	std::cout << "Same-clock loop, exact feedback join and invalid/missing trace reporting "
		     "passed\n";
}
