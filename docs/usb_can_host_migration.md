# USB chassis controller migration

## Working location and synchronization

Continue development in Orb Ubuntu at `~/wbr_control_companion`, branch
`feature/usb-can-host-controller`. GitHub origin is
`https://github.com/NoneOfEver/wbr_control_companion.git`.

The NUC and Orb companion checkouts had identical clean HEAD
`e9f8775862b7ac13d0684bb768b034b225b59167` before this branch was created.
`benchmarks/can_usb_stress_test` preserves the latest NUC host/tools source
captured on 2026-10-20. Build products, results, passwords and Python caches are
excluded. Its CMake and build script now locate the companion within this repo.
The benchmark remains a reference/verification target, not the production controller.

Build and run mock tests inside Orb (no physical USB or motors required):

```bash
cd ~/wbr_control_companion
bash benchmarks/can_usb_stress_test/tools/build_native.sh
```

The original `wbr-nuc` Git remote remains unchanged. After SSH is available again,
add a separate GitHub remote in the NUC checkout if it is absent:

```bash
cd ~/wbr_realtime_lab/src/wbr_control_companion
git remote add github https://github.com/NoneOfEver/wbr_control_companion.git
git fetch github
git switch --track github/feature/usb-can-host-controller
```

Do not overwrite a dirty NUC checkout. If the remote or local branch already
exists, reuse it and update by fast-forward. Credentials must be configured on
that machine; never copy authentication tokens into this repository. Subsequent
updates are `git pull --ff-only github feature/usb-can-host-controller` on NUC,
while Orb commits and pushes to origin. No SSH access to NUC is needed during
local development.

## Agreed production boundary

- Move the existing MCU chassis_controller control law/state machines/estimators
  to the NUC; a zero-command-only demo is not a completed migration.
- MCU handles hardware acquisition and USB forwarding for CAN, UART and IMU.
- USB Bulk; six motor commands in one OUT; MCU CAN batch order W/B/D on each bus.
- Preserve independent host USB event and 1 kHz control threads.
- Use the original MCU ingress scheduling. The experimental fast-ingress path
  did not demonstrate a performance benefit and is not selected.
- Production targets must not depend on benchmark/test headers. Remove VOFA,
  sampled timing exports, stress protocols and per-cycle diagnostic ACKs from
  the production link. Preserve protocol versioning, validation, freshness and
  host-loss safe motor shutdown.
- Keep historical tests available for reproducing measurements; do not erase
  unrelated work or make the production target link them.
- The robot has been on the floor throughout zero-torque tests. Building a real
  controller does not authorize launching nonzero-torque control on hardware.

## Source mapping and pending work

MCU source root on Mac:
`/Users/panpoming/Documents/zephyr_projects/wbr_control_ws/wbr_control`.

- Existing control: `src/chassis_controller/modules/chassis` (balance, recovery,
  flight, jump, climb, support-force estimation, kinematics/VMC and state machine).
- Existing adapters: ChassisInputReader/ChassisActuator, motor codecs/map and
  generated YAML parameters. Preserve the current configurations and signs.
- Acquisition to inventory: remote UART, HI91 UART IMU, onboard IMU/AHRS,
  referee UART and other board-configured inputs actually consumed by control.
- Build a portable controller boundary rather than emulating the Zephyr kernel
  in the host application. Keep shared math/control logic reusable.
- Create an independent MCU forwarding application and a production NUC
  chassis_controller executable; compile both and test actual shared control
  logic, protocol framing, freshness, loss-of-host shutdown and native USB lifecycle.
- Hardware validation is pending and requires the NUC/board to be available.

WBD vs WDB did not demonstrate a six-motor latency improvement. BWD did not
improve the overall path. BDW increased typical batch time from about 380 to
622 us. Current benchmark response observations do not identify causal motor
execution RTT; feedback carries no command sequence. Latest-value feedback does
not accumulate a FIFO backlog, but stale data can exceed one period on failures.
