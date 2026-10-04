# wbr_control_companion

Linux-side control processes for the wheel-legged robot.

## Leg height / roll MPC

`wbr_leg_mpc` implements the document's lightweight linear MPC:

- state: `[roll, roll_rate, body_height, body_height_rate]`;
- output: absolute left/right normal support force in newtons;
- 100 Hz sample rate and 15-step (0.15 s) prediction horizon;
- 20-150 N force bounds and 30 N/sample slew bounds;
- gravity feed-forward (`M*g/2` per leg) outside the optimized force deviation;
- 0.01 N maximum inter-iteration force change as the convergence criterion;
- constant or time-varying height/roll reference trajectories.

The default mass (12.054 kg) and half track (0.215 m) mirror the current
`wbr_control` chassis parameters. The roll inertia is the measured
0.113303628 kg m^2 value. The Q/R values and vertical damping are still
unverified tuning values. Keep the two repositories synchronized when chassis
parameters change.

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

`wbr_mpc_link` implements the host-side codec for MPC Link v2. Each USB
interrupt transfer contains one complete frame: a 64-byte chassis state frame
or a 40-byte force command frame. The state includes the MCU's live target body
height so MPC can track operator height changes while active. The command echoes
the source state sequence and supports disabled, shadow and active modes.
Transport and actuator enable logic are intentionally separate from the codec.

## USB MPC runtime

`usb_mpc` is the production USB control process. Its default schedule matches
the validated configuration: a 100 Hz USB exchange and one MPC solve per
exchange. It runs continuously until SIGINT or SIGTERM, pins the realtime loop
to CPU 3 at FIFO priority 80, and starts in Shadow mode. Applying forces always
requires the explicit `--active` option; the lower controller independently
enforces its local operator-enable gate, sequence checks and watchdog.

```sh
sudo ./build/usb_mpc --active
```

CSV collection is disabled by default so normal operation does not generate
continuous disk traffic. Enable the retained production diagnostics only when
needed:

```sh
sudo ./build/usb_mpc --active --csv /path/to/usb-mpc.csv
```

`--cycles N`, `--period-us N`, `--mpc-divisor N`, `--cpu N` and
`--priority N` remain available for controlled commissioning runs.

If the USB device disconnects, the runtime immediately abandons the stale
libusb session and relies on the lower-controller watchdog for local fallback.
It then waits for the same VID/PID to reappear, opens a fresh session, sends a
Disabled command first, and resumes Shadow or explicitly requested Active mode
only after receiving a valid state frame. SIGINT and SIGTERM still terminate
the process instead of reconnecting.

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
