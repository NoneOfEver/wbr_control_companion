# wbr_control_companion

Linux-side control processes for the wheel-legged robot.

## Leg height / roll MPC

`wbr_leg_mpc` implements the document's lightweight linear MPC:

- state: `[roll, roll_rate, body_height, body_height_rate]`;
- output: absolute left/right normal support force in newtons;
- 100 Hz sample rate and 15-step (0.15 s) prediction horizon;
- 20-150 N force bounds and 30 N/sample slew bounds;
- gravity feed-forward (`M*g/2` per leg) outside the optimized force deviation;
- constant or time-varying height/roll reference trajectories.

The Q/R values in the document are not published. The defaults in
`LegMpc::Parameters` are safe simulation starting values, not validated robot
gains. Robot mass, inertia, geometry, constraints and weights must be checked
against the real chassis before enabling motor output.

Build and test inside the OrbStack Ubuntu machine:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/leg_mpc_demo
```

The next integration layer should provide timestamped chassis state samples
from the lower controller and send the two resulting force targets back to its
VMC. It should include stale-data rejection, sequence numbers, force-command
timeout and a gravity-feed-forward fallback before real actuators are enabled.

## MPC Link protocol

`wbr_mpc_link` implements the host-side codec for MPC Link v1. Each USB
interrupt transfer contains one complete frame: a 60-byte chassis state frame
or a 40-byte force command frame. The command echoes the source state sequence
and supports disabled, shadow and active modes. Transport and actuator enable
logic are intentionally separate from the codec.

## EtherCAT benchmark and native MPC

`ethercat_bench` provides one measurement path for SOEM, IgH generic and IgH
native. IgH generic/native intentionally use the same `ecrt` backend; the
loaded kernel driver selects the path. Results include per-cycle scheduling
jitter, exchange cost, echoed-sequence RTT, WKC and slave status in CSV/JSON.

`ethercat_mpc` runs the 1 kHz EtherCAT exchange loop and evaluates the roll /
height MPC every tenth cycle by default. It starts in Shadow mode. Active mode
requires both `--mode active` and `--allow-active`; the HPM application must
independently allow Active before applying forces.

The process-data ABI and SSC integration contract are documented in
`wbr_control/docs/ETHERCAT_CONTROL_PROTOCOL_V1.md`. The first-board procedure,
acceptance gates and exact comparison commands are in
`wbr_control/docs/ETHERCAT_BRINGUP_CHECKLIST.md`; the licensed SSC application
hook template is under `wbr_control/test/ethercat_slave_test/`.

Every host-side safety transition creates a new Disabled command sequence.
This is intentional: the slave rejects duplicate sequences so replayed Active
PDOs cannot refresh its watchdog, and merely changing the mode bits in an old
sequence would otherwise delay fallback until timeout.

Build output enables IgH and SOEM automatically when their headers and
libraries are found.
