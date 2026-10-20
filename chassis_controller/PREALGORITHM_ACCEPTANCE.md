# Acceptance before running the migrated controller on the robot

The host USB startup order is fixed. UART10 line-idle remains an opt-in MCU
candidate pending hardware lifecycle acceptance. Native tests and isolated
communication runs do not validate the complete control loop.

## First: complete-load zero-output run

The UART capture sends empty command heartbeats. It does not reproduce six-motor
1-kHz command traffic or full controller compute. Use one matched production
host/MCU configuration to drive all six zero-torque CAN commands while acquiring
all six motor feedback streams, SBUS, HI91 and onboard raw IMU over Bulk USB.
Record per-stream gaps/freshness and error counts, command coalescing, queue
saturation, actual 1-kHz releases and their worst intervals. Include multiple
cold starts and session reestablishments. Preserve the full maxima separately
from explicitly labelled steady-state summaries; do not hide startup samples.
The independent `full_load_zero` tool now implements this baseline using wheel
zero current and joint Exit frames. See `measurements/full_load/README.md` for
metrics and limits, including unavailable MCU CAN TX completion/coalescing data.
Its mock test passes; the combined hardware run remains pending.

## Hardware lifecycle and fault behavior

Verify genuine UART transmitter silence after a known partial DMA payload. Turning
off the handset did not stop receiver UART output in the observed run, and RX
disable has its own flush, so neither substitutes for this test. Verify actual
UART close/re-enable separately, including detector rearm, offsets and parser
resynchronization. The native reentrant-callback tests cover only software state.

Check physical USB detach/reconnect, host process termination/freeze, command
lease expiry, MCU reset and sensor/motor-feedback loss. Record that the MCU
submits wheel-zero/DM-exit after command loss and that the host refuses stale
inputs. The 20-ms command lease is not an end-to-end motor stop guarantee: CAN
queue/serialization and motor response add time. Reconnection must require the
normal explicit rearm conditions, not automatically resume previous motion.
The mock USB tests are not substitutes for physical lifecycle tests.

Verify SBUS frame-lost/failsafe, invalid stream and true input timeout cause the
intended disabled state. Test HI91 stale/invalid input and motor-feedback loss.
Do not equate forwarding chunk-sequence continuity with valid sensor frames.

## Sensor and actuator conventions

Before allowing output, verify six motor IDs and left/right mapping, encoder
zero/sign, joint units and gearing, wheel direction, HI91 axes/gyro signs/units,
body frame and finite values. Check sensors are fresh during motion by hand with
actuation disabled, and that parser rejection does not refresh freshness.
Check HI91 CRC policy explicitly; the migrated legacy default is permissive.
MCU and host epochs are independent; existing UART timing does not establish
complete source-to-algorithm age.

## Actual algorithm as a shadow computation

Run the full controller and state machine with real sensor inputs and a test-only
output sink, so computed motor frames are observable but cannot reach real
actuators. Default disabled operation alone does not exercise all enabled-state
math/arming paths. Check torque clamps, finite outputs, state transitions,
initialization and failure recovery. Under complete sensor traffic and realistic
host load, measure snapshot-to-compute-to-submit time and missed 1-kHz deadlines
with production affinity/FIFO/mlock configuration. Native numerical equivalence
is useful but does not validate migrated hardware signs or closed-loop stability.

Only after those pass, validate the matched build with diagnostics disabled and
perform a controlled, limited-output motor test. Chassis control performance and
stability need their own validation; UART update cadence and communication RTT
alone cannot establish them. Record exact build commits/configs/parameters.
