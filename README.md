# wbr_control_companion

Linux-side control processes for the wheel-legged robot.

## Leg height / roll MPC

`wbr_leg_mpc` implements the document's lightweight linear MPC:

- state: `[roll, roll_rate, body_height, body_height_rate]`;
- output: absolute left/right normal support force in newtons;
- 100 Hz sample rate and 15-step (0.15 s) prediction horizon;
- 20-200 N force bounds and 30 N/sample slew bounds;
- gravity feed-forward (`M*g/2` per leg) outside the optimized force deviation;
- native OSQP v1.0.0 solves the complete 30-variable QP with all force and slew constraints;
- QP primal/dual termination tolerance 1e-6 and independent full-horizon feasibility tolerance 1e-3 N;
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

## Complete constrained leg QP (2026-10-08)

The decision vector contains force deviations from M*g/2 for both legs over
15 stages. The constraint matrix is [I; D], where D contains the first force
and every adjacent-stage difference. Absolute bounds are shifted by M*g/2;
the first two D bounds are updated relative to the previous output command.
All 60 constraint rows are optimized simultaneously. There is no sequential
force clipping in the optimizer.

Each LegMpc owns a separate OSQP workspace, initialized once. H and A remain
constant for the controller's parameters; online calls update q/l/u and warm
start the shifted primal trajectory. The maximum iteration count is 2000.
Only OSQP_SOLVED with finite output and a checked feasible full trajectory is
marked converged. Invalid input, iteration exhaustion or solver errors return
bounded previous commands as a diagnostic fallback, with converged=false and
used_fallback=true. USB/EtherCAT propagate the fallback flag; the MCU's normal
convergence gate remains responsible for rejecting an unqualified Active command.
The legacy gradient_inf_norm result now holds the dual residual, and the USB
CSV column is named solver_dual_residual.

CMake builds the native static OSQP dependency from upstream v1.0.0 (Apache-2.0).
The first configure needs network access; later builds use the cached source.
For an existing source checkout, pass -DFETCHCONTENT_SOURCE_DIR_OSQP=/absolute/path.
Test assertions remain enabled even in Release builds.

Validation on wbr-nuc: all four tests passed, including the previously failing
combined roll/height recovery and the MCU-rate 0.20 -> 0.30 -> 0.18 m target ramp.
Independent SciPy SLSQP comparison on a slew-constrained case differed by
0.000113 N at most across the complete trajectory. On CPU 2 in ordinary
scheduling, 5000 representative QPs measured mean 54.2 us, P99 61.1 us, maximum
97.1 us, no failures and no 10 ms overruns. This is a CPU-only, unpaced timing
measurement, excluding communication, not a hardware control validation or WCET.

Optional verification/benchmark commands:

```sh
ctest --test-dir build --output-on-failure
python3 tests/verify_leg_qp.py ./build/leg_mpc_qp_test # needs numpy and scipy
./build/leg_mpc_benchmark
```

## USB chassis controller migration

Development continues in Orb Ubuntu on `feature/usb-can-host-controller`.
See [migration and GitHub synchronization notes](docs/usb_can_host_migration.md).
The latest NUC communication benchmark sources are preserved under
`benchmarks/can_usb_stress_test`; they are verification references, separate from
the production controller being migrated.

### Native production chassis target

The actual migrated controller is now under [`chassis_controller/`](chassis_controller/README.md).
Build with `./scripts/build_chassis_controller.sh` in Orb Ubuntu or on NUC.
It uses a separate versioned USB gateway protocol and does not link stress tests,
VOFA or the legacy MPC offload interface. Hardware verification is pending;
actuator output is disabled by default.
