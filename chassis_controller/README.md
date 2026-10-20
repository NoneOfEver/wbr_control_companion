# Native chassis controller

This is the production-target migration of the MCU chassis control code, not
an echo benchmark or a zero-torque replacement algorithm. It builds separately
from the older MPC/EtherCAT applications and needs no OSQP download.

## Build on Orb Ubuntu or NUC

```bash
cd ~/wbr_control_companion
./scripts/build_chassis_controller.sh
# Offline smoke run: no USB device or motors are opened.
./scripts/run_chassis_controller.sh --simulate --no-rt --seconds 1
```

Dependencies: Linux, C++20, Eigen3, pthreads, libusb-1.0 development headers,
CMake, Python3 and PyYAML for the existing parameter generator.

After the new MCU `usb_can_bridge` image is flashed and connected:

```bash
sudo ./scripts/run_chassis_controller.sh
```

The default actuator policy is disabled: the actual migrated controller runs,
but it cannot enter motor arming or apply nonzero torque. It emits the original
state machine's zero-wheel/DM-exit actions, not a hardcoded 1 kHz motor stress
sequence. `--seconds N` optionally bounds runtime. Ctrl-C sends Stop; USB/host
failure is also covered by a 20 ms MCU command lease.

Real control output additionally requires `--enable-output`, a valid disabled
remote switch pose observed after process startup, then the normal remote
activation, fresh HI91 data, valid motor feedback and the existing state-machine
arming/recovery conditions. Hardware control is not launched by build or tests.
This software has not yet been validated on the physical robot after migration.

## Source and behavior

- `control/`: migrated balance/LQR, state machine, recovery, flight, jump, climb,
  force estimation, gas spring compensation, VMC and leg kinematics. Original
  inputs, signs, torque limits, arming timing and parameter values are retained.
- `include/protocols/`: original DJI/DM, HI91 and UART protocol codecs with
  unnecessary Zephyr includes replaced by explicit portable byte operations.
- `config/`: exact YAML/schema inputs; generated headers are build products.
- `runtime/`: bounded latest-state store, UART parsers, native USB event owner,
  motor frame collector and absolute-period control entry point.
- `tests/`: native-only contract tests and a USB mock injected solely by CTest.
  Neither these nor `benchmarks/` are linked into the production executable.
- `source_manifest.json`: SHA-256 of imported original MCU control files, before
  host adaptation, to identify the source version used for migration.

The control thread freezes one snapshot, runs the entire original control
pipeline and emits one 128-byte Bulk OUT per tick. The MCU reorders each bus
as W/B/D. A packet can carry fewer than six changed commands, including an empty
heartbeat, as required by disabled/arming modes in the original actuator policy.
The USB event thread independently receives four preposted IN transfers, updates
state and recycles OUT buffers on USB completion without diagnostic ACKs.
Both threads default to CPU 3 and FIFO priority 80; `--cpu`, `--usb-cpu`,
`--priority` and `--no-rt` select the Linux execution configuration.

The old MCU external MPC offload port, EtherCAT publisher, VOFA, profiling
counters/exports and stress-protocol handlers are not in this runtime. The
current original local control algorithms execute on NUC instead. The separate
existing companion MPC applications remain available but are not implicitly
substituted for or connected to this controller.

## Acquisition and freshness

The active control IMU source is HI91, matching the original resolved firmware
configuration. UART10 is the existing SBUS 100000 8E2 remote input; UART2 is HI91
921600 8N1. SBUS channel mapping and HI91 axis mapping are preserved. A broken
UART stream resets partial-frame assembly; SBUS failsafe/frame-lost disables
requests immediately (stricter than the old frame-lost hold policy).

HI91 strict CRC defaults off, matching the original firmware configuration;
`--strict-hi91-crc` enables rejection after its device CRC variant is verified.
Repeated device system timestamps do not refresh the IMU state. Raw onboard SPI
bursts and temperature are received separately; they do not silently replace
HI91 with an unverified onboard body transform. Optional referee UART chunks
are retained if the MCU board actually declares that UART.

CAN and UART acquisition timestamps are carried only to enforce freshness.
MCU acquisition-to-submit age is subtracted from host arrival time using MCU
local intervals; MCU and host absolute epochs are never subtracted. This still
excludes unknown USB IN transit/callback delay from the sensor age estimate,
so it is not an exact synchronized sensor age or an execution RTT. A missing
sensor stream, old motor data, old IMU data or lost USB session fails closed.
A paused host process cannot rely on new data sitting in a FIFO: the firmware
requires a new handshake after its command lease expires.

## Wire contract

`include/wbr_bridge/bridge_protocol.hpp` is identical to the MCU header
`wbr_control/src/protocols/usb_bridge/bridge_protocol.hpp`. It is a new versioned
WRB1 protocol; the old stress-test firmware is rejected before any 128-byte OUT.

- 64-byte Hello/Ready negotiate session and 128/64-byte capacities.
- One 128-byte command includes up to six standard 8-byte CAN frames, a session
  and increasing sequence. All addresses are validated before either bus is
  published. Duplicate slots, nonzero reserved fields and old sequences fail.
- 64-byte IN packets carry up to three CAN records, a UART stream chunk, or an
  onboard raw IMU burst. Latest motor state does not create a history backlog;
  UART chunks have per-port sequences so a dropped chunk resets the parser.
- 64-byte Stop ends the session. There are no per-cycle ACKs, latency exports,
  CSV result writers, VOFA channels or benchmark modes in the formal link.

## Migration reference check

For an additional offline numerical comparison, configure with
`-DWBR_REFERENCE_CHASSIS_DIR=/path/to/original/modules/chassis`. CTest then builds
the original MCU balance/math sources as a separate reference executable and
compares 1200 actuator values over 2000 frozen-input ticks against the migrated
implementation. This optional source path is not a runtime dependency and is
not required on NUC. The current local comparison passed with maximum difference
zero. Recovery/flight/jump/climb implementations were imported from the same
manifested source version; the numerical comparison specifically covers balance.

UART forwarding baseline measurement is available as an independent tool; see
[measurement instructions and timing limits](measurements/README.md).

Every USB session completes event-thread realtime setup and preposts all four IN
transfers before sending Hello. Setup or IN-submit failure prevents session start;
this is mandatory in both the controller and capture tool. The startup backlog
A/B is recorded in `measurements/HI91_STARTUP_INVESTIGATION.md`.
Before real actuation, follow [the remaining acceptance work](PREALGORITHM_ACCEPTANCE.md).
