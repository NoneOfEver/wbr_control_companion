/* SPDX-License-Identifier: Apache-2.0 */
#include "feedback_gate.hpp"
#include <iostream>
#define CHECK(c)                                                                                   \
	do {                                                                                       \
		if (!(c)) {                                                                        \
			std::cerr << "line " << __LINE__ << '\n';                                  \
			return 1;                                                                  \
		}                                                                                  \
	} while (false)
int main()
{
	stress_host::FeedbackGate gate;
	gate.Begin(1000000, 1010000, 200);
	CHECK(gate.Evaluate(1010000, {0, 0}, {false, false}, {1010000, 1010000})
		      .send); // startup never waits
	gate.Commit({0xffffffffU, 50});
	gate.Begin(2000000, 2020000, 200);
	CHECK(gate.WakeupLateNs() == 20000 && gate.DeadlineNs() == 2200000);
	// New but 900us-old feedback must wait, otherwise the original almost
	// one-cycle delay would pass a novelty-only check unchanged.
	CHECK(!gate.Evaluate(2020000, {1, 51}, {true, true}, {1120000, 1120000}).send);
	CHECK(!gate.Evaluate(2050000, {0xffffffffU, 50}, {true, true}, {2050000, 2050000}).send);
	CHECK(!gate.Evaluate(2070000, {0, 50}, {true, true}, {2070000, 2070000})
		       .send); // wrap is a new RX
	CHECK(!gate.Evaluate(2100000, {0, 51}, {true, false}, {2100000, 2100000})
		       .send); // both must be valid
	const auto fresh = gate.Evaluate(2100000, {0, 51}, {true, true}, {2100000, 2100000});
	CHECK(fresh.send && !fresh.expired && fresh.waited_ns == 80000);
	gate.Commit({0, 51});
	CHECK(!gate.Active());
	gate.Begin(3000000, 3010000, 200);
	const auto timeout = gate.Evaluate(3200000, {0, 51}, {true, true}, {3200000, 3200000});
	CHECK(timeout.send && timeout.expired && timeout.waited_ns == 190000);
	gate.Commit({0, 51});
	gate.Begin(4000000, 4300000, 200); // a late wake never adds another 200us
	CHECK(gate.Evaluate(4300000, {0, 51}, {true, true}, {4300000, 4300000}).send);
	gate.Commit({0, 51});
	gate.Begin(5000000, 5000000, 0); // strict baseline has no wait
	CHECK(gate.Evaluate(5000000, {0, 51}, {true, true}, {5000000, 5000000}).send);
	stress_host::FeedbackGate zero;
	zero.Begin(1, 1, 200);
	zero.Commit({0, 0}, {false, false});
	zero.Begin(1000000, 1000000, 200);
	CHECK(zero.Evaluate(1000000, {0, 0}, {true, true}, {1000000, 1000000}).send);
	std::cout << "Fresh feedback, bounded absolute wait, startup, wrap and baseline passed\n";
}
