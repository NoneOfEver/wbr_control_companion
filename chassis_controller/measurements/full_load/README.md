# Complete concurrent traffic, zero-output baseline

This separate test uses the production UsbLink/decoder, fixed preposted-IN
startup, 128-byte merged Bulk OUT and the gateway's W/B/D CAN group submission.
No controller algorithm runs. No firmware change is required from the currently
installed UART10 line-idle candidate.

Each 1-kHz command has exactly six unique production motor records:

- both wheel 0x200 payloads are all zero current;
- all four DM payloads are Exit (`FF FF FF FF FF FF FF FD`).

The gateway Hello also performs its normal local stop. This test never sends
Enter, MIT setpoints, error-clear or zero-save commands. Exit maintains disabled
joints. These stop-frame payloads and response behavior differ from enabled MIT
zero-torque operation, so this is not an identical on-wire replay of an active
controller. If a disabled motor does not return feedback, the run fails its
coverage check; do not enable motors automatically to make the test pass.

## Run on NUC

Do not run another USB owner concurrently. Keep the robot stopped, with the
remote, HI91 and onboard IMU connected, and use the current line-idle firmware.

```bash
cd ~/wbr_realtime_lab/src/wbr_control_companion
sudo ./scripts/run_full_load_zero.sh --seconds 60 \
  --output ~/wbr_realtime_lab/results/full_load_zero_$(date +%Y%m%d-%H%M%S)
```

The output directory must not already exist. Both the command loop and USB event
owner default to CPU 3/FIFO 80 with mlockall; options select CPU/priority or
`--no-rt` for software-only tests. Memory and output directory are prepared before
measurement. No periodic logging or disk writes occur during capture. Reports
and CSVs are written only after Stop and USB drain. A final stopped handshake
reads cumulative UART diagnostic flags. Ctrl-C stops and exports a partial run,
which is not labelled a complete duration pass.

## Outputs and interpretation

- `summary.txt`: requested loop rate versus actual submissions, skipped release
  slots, wakeup/submit intervals, successful 128-byte USB completions and failures,
  per-motor CAN feedback, UART chunk/parse updates and onboard raw IMU reception.
- `received.csv`: every observed known-motor CAN record/UART chunk/onboard IMU
  message in the measurement window, host arrival and existing MCU timestamps.
- `cycles.csv`: target release, actual wake, USB submission completion-to-caller,
  skipped slots and decoder-valid snapshot mask (bits 0..5 motors, 6 remote state
  present, 7 HI91 valid, 8 onboard state present). This mask is not a freshness
  guarantee; the stream interval checks are reported separately.

Required capability bits are CAN, remote UART, HI91 UART and onboard IMU (15).
Referee UART is optional. Coverage requires every mandatory stream, no missing
UART sequence/duplicate/overflow, no observed DM fault/malformed motor record,
and no reported URT4 idle/offset error when that diagnostic is available.
Observation-silence bounds, including beginning/end of the window, are 20 ms for
motor feedback/HI91/raw IMU and 30 ms for SBUS. HI91/SBUS parser state updates
must also meet their corresponding bounds. These are test acceptance limits,
not a replacement for the controller's own input freshness/failsafe policy.

`HOST_OBSERVED_PASS` additionally requires full duration, matching successful
USB OUT completions, no transport error, and (in RT mode) zero skipped slots.
`--no-rt` reports skipped slots but is not a scheduling acceptance run. Parser
update events can include invalidation and coalescing of multiple frames; they
are not a count of independently verified valid wire frames. CAN has no per-frame
sequence in this protocol, so zero `missing_chunks` on CAN is not proof of no CAN
loss. Per-stream raw rates are observed values; some motor streams can exceed
1 kHz, as in previous tests.

**MCU CAN TX completion, queue coalescing and driver errors are not exported by
the current formal protocol and remain UNMEASURED.** USB completion and motor
feedback do not prove every requested CAN command completed. This run is a
combined load/host-observed coverage baseline, not a complete MCU TX integrity
or end-to-end control latency certification. Full sensor-wire latency, enabled
control behavior, true UART silence and hardware restart remain separate tests.

## Software verification

The mock backend asserts six command records and exact zero/Exit payloads on
every cycle, plus all four IN transfers before Hello. It generates six motor
feedback streams, valid fragmented HI91, periodic SBUS and raw IMU. The test
checks export and successful USB completion accounting, then verifies absent
onboard IMU or a DM fault cannot produce a passing result. It opens no hardware.
