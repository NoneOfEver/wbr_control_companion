# Stop-only host fault/recovery test

The parent launches its own zero-output worker and signals only that PID. The
worker uses the production USB startup barrier, six stop-only CAN frames at 1 kHz
and independent USB event owner. Wheels are zero current, joints Exit. There is
no controller algorithm, Enter/MIT setpoint or automatic reconnect/rearm.
The worker also exits after five seconds if its parent stops supervising.

## Required MCU diagnostic image

Build the existing line-idle candidate with
`CONFIG_WBR_GATEWAY_FAULT_DIAGNOSTICS=y`. Production defaults leave this off.
No shared HPM driver change is needed for these added counters.

From the Mac Zephyr workspace:

```bash
CCACHE_DISABLE=1 /Users/panpoming/Documents/zephyr_projects/.venv/bin/west \
  build -d /tmp/wbr-uart10-plic-probe -- -DCONFIG_WBR_GATEWAY_FAULT_DIAGNOSTICS=y
/Users/panpoming/Documents/zephyr_projects/.venv/bin/west \
  flash -d /tmp/wbr-uart10-plic-probe
```

Normal Hello still returns the existing UART diagnostic page. Only the fault
worker requests Hello page 0xF1 and requires FLT1. If the image is absent, the
worker fails before reporting ready; the parent injects no signals.

## Run on NUC

Stop other USB owners first. Keep the robot and all actuators disabled, as in
the completed full-load zero-output capture. Do not physically unplug USB yet.

```bash
cd ~/wbr_realtime_lab/src/wbr_control_companion
sudo ./scripts/run_fault_recovery.sh \
  --output ~/wbr_realtime_lab/results/fault_recovery_$(date +%Y%m%d-%H%M%S)
```

Cases, in order:

1. SIGTERM: worker requests normal Stop and drains USB. MCU explicit-stop counter
   must increase once, watchdog-stop count must not change, session must be zero.
2. SIGKILL: no destructor/Stop. After 120 ms the query must observe one automatic
   stop, no valid explicit Stop and a cleared old session.
3. SIGSTOP for 120 ms, then SIGCONT: the MCU stops independently of the frozen
   host. On resume the worker must fail on stale deadline/session and exit without
   new Hello/rearm. Counters must show one automatic stop and no valid explicit
   Stop from the expired old session. No explicit stale-frame replay is injected.

Each case first establishes a live lease with 0.5 seconds of stop-only commands.
The independent post-case query happens after child exit and a further 120 ms.
An automatic stop due to lease expiry must have a last-command-to-stop-submission
interval of 20..30 ms using the same MCU clock. The upper limit is this test's
acceptance tolerance, not a general motor stop guarantee. USB deconfiguration
may trigger an earlier automatic stop and is reported with a distinct reason.
The first failing case stops the suite. Cleanup resumes/terminates/kills only a
still-running child owned by this parent. `report.json` preserves successful and
failed checks; transport/preflight exceptions are recorded in `error.txt`.

## What the MCU snapshot actually proves

FLT1 is captured BEFORE the querying Hello performs its own SafeStop/reset, so
the query cannot hide an uncleared old session or overwrite the stop event being
measured. The query itself later sends Stop; the next worker's before snapshot
includes that counter, and comparisons are per case.

Fields are cumulative accepted/rejected commands, automatic and explicit stop
counts, last accepted MCU timestamp, last policy-stop submission return timestamp,
reason/submission-error flags and prior session ID. Reasons: 1 explicit Stop,
2 lease expiry, 3 USB deconfigured, 4 failed ordinary CAN submission. Hello/boot
stops are deliberately excluded from stop-event counters. All counter/timestamp
differences use uint32 modulo arithmetic. Rejected commands never renew the lease.

A no-error event means both stop CAN groups returned successful queue submission.
**CAN hardware completion, actual bus delivery and physical motor stop remain
UNMEASURED.** This test observes software policy/session behavior under zero-output
faults. It does not prove an enabled moving robot stops, nor test physical USB
unplug, true UART silence, MCU resets or controller rearming under real inputs.
Those are separate acceptance steps.

Host mock tests verify diagnostic encoding, old-image refusal, real OS signal
handling of the worker, strict zero/Exit command bytes, cleanup and result checker
boundaries (including MCU clock wrap). They do not emulate the full MCU watchdog
across processes and are not a substitute for this hardware suite.

## First hardware result (non-RT host)

The diagnostic image was flashed and FLT1 verified. The suite ran on wbr-nuc
with `--no-rt`, no controller/Enter/MIT frames. All three cases passed; each
worker had 500 newly accepted command packets before the fault:

| Case | Last accepted MCU command -> stop group submission | Reason | Prior session |
| --- | ---: | --- | ---: |
| SIGTERM | 848 us | explicit Stop | 0 |
| SIGKILL | 20913 us | lease expired | 0 |
| SIGSTOP/SIGCONT | 20995 us | lease expired | 0 |

Both automatic cases incremented the lease-stop counter once and had no valid
explicit Stop in the expired session. Freeze resumed to worker exit code 2,
without reconnect/rearm. Stop group submissions returned no errors. These
intervals are MCU-local, **not time from the OS signal or physical motor stop**.
Raw evidence is `first_no_rt_report.json`; the NUC report is
`~/wbr_realtime_lab/results/fault_recovery_first_no_rt/report.json`.

The operator subsequently ran the production realtime suite and reported PASS
for all three cases, including the 20..30 ms lease-stop submission checks.
Hardware motor stop timing remains unmeasured.


## Physical USB unplug/replug (operator assisted)

Use the same currently flashed FLT1 diagnostic image; no new flash is needed.
The MCU must remain powered independently throughout. Stop other USB owners.
The worker sends only wheel zero-current and joint Exit frames, for at most
60 seconds. This does not execute or rearm the controller algorithm.

```bash
cd ~/wbr_realtime_lab/src/wbr_control_companion
sudo ./scripts/run_usb_unplug.sh \
  --output ~/wbr_realtime_lab/results/usb_unplug_$(date +%Y%m%d-%H%M%S)
```

1. Wait for stage 1, then unplug the USB cable while keeping MCU power on.
   Leave it unplugged until stage 2. The tool checks device removal and that the
   old worker reports USB failure rather than silently reconnecting.
2. At stage 2, reconnect to the same USB port. The tool waits for re-enumeration.
3. At stage 3, press Enter to explicitly open a diagnostic session. Its snapshot
   must preserve the earlier automatic stop, clear the old session and report no
   stop-submission error. Lease expiry requires 20..30 ms from last accepted MCU
   command; USB deconfiguration may stop earlier (0..30 ms).
4. Only after those checks pass, a fresh 0.5-second stop-only session is opened
   and must exit normally with explicit Stop. Results are saved to report.json.

A changed USB device number and retained counters help reject an invalid test;
a reset MCU cannot provide the intended retained stop evidence. Keep the supply
on rather than relying solely on these checks to establish power continuity.
The test measures MCU software stop submission and host/session recovery, not
CAN hardware completion, physical motor stop or enabled-algorithm rearming.


### Diagnosing a deadline exit during detach

The fault worker enables an optional bounded USB event observer; production
callers leave it null. No trace formatting, allocation or disk I/O occurs in
callbacks. After Stop/Close joins the event owner, stderr prints a summary,
also preserved in report.json child_stderr. CPU/FIFO settings and pass criteria
are unchanged.

All trace times are NUC CLOCK_MONOTONIC nanoseconds. first_transfer_error is
the first non-completed/invalid-length callback before shutdown (kind 1 IN,
2 OUT; status is libusb_transfer_status, NO_DEVICE=5). first_event_error and
event_result describe the first nonzero event-handler return. max_fast_run
counts consecutive event calls shorter than 10 us and records their time range;
normal traffic can also cause fast returns, so this alone does not prove spin.
Compare that range and first error with last_good_wake, fault_release and
fault_wake (deadline exit), or unhealthy (Send failure). link_error preserves
the first recorded USB link error. A zero timestamp means not observed.
The trace is diagnostic instrumentation and can slightly perturb timing.


Hardware detach trace (before fix): first IN NO_DEVICE was followed by
2,334,326 consecutive event calls shorter than 10 us over 1.591 s. The control
owner next woke 1.650 s after its scheduled release and exited on deadline.
The retained MCU record showed USB-deconfiguration stop submission 1125 us
after its last accepted command, with old session cleared and no submission
error. This confirms host event-loop spin; it does not measure motor stop.

On link failure the event owner now waits on a condition variable. Close sets
the closing predicate under the same mutex and wakes it to cancel/drain pending
transfers. This prevents failed-context polling from starving the same-CPU FIFO
control owner. No auto-reconnect, affinity change or relaxed pass criterion is
introduced. Mock detach/close regression passes; physical retest is pending.
