# Stop-only remote input loss capture

Run `sudo ./scripts/run_remote_loss.sh NEW.csv` on NUC. No firmware update.
Keep receiver/MCU power and USB connected. Start with transmitter on for about
5 seconds, turn it off for about 20 seconds, then on again for the remainder
of the 60-second capture. No algorithm runs: only wheel-zero and joint Exit
frames are sent at 1 kHz, regardless of remote switch position. No Enter/MIT.

The production SensorDecoder exposes the last parsed SBUS flags as observation
metadata; ChassisInputReader supplies actual production freshness/request policy.
Only bounded memory capture runs during measurement; CSV is written after USB
Stop/Close. SIGINT exports a partial capture. A zero exit code means transport
and observed safety invariants held, not that all fault cases were exercised.
No-data startup is excluded from stale_ticks. Flags are meaningful only with
their associated sequence and freshness. Invalid frames do not renew sequence.

Current configured remote timeout is 1000 ms; this tool does not change it.
Closing the transmitter may produce failsafe frames or silence, depending on
receiver configuration. A scenario not observed remains untested. frame_lost
may require synthetic contract tests rather than physical RF loss. Review CSV
sequence, flags, age, fresh and requested_enable transitions; transport continuity
alone cannot establish valid radio behavior. The script does not test the full
controller state machine, motor stop timing or automatic rearm behavior.

Real receiver UART silence (if not produced by transmitter off) requires a
separate operator-assisted source-disconnect test; do not unplug arbitrary wires
while running this capture. Offline contracts check failsafe/frame-lost decoding
and no-new-sequence timeout using the same production decoder/input reader.


## HI91 source loss

`sudo ./scripts/run_hi91_loss.sh NEW.csv` reuses this stop-only capture and adds
imu_sequence, imu_age_us, imu_valid and imu_fresh columns. It uses the production
ChassisInputReader (3 ms configured timeout), not an independent timeout policy.
Cached imu_valid need not clear on silence; imu_fresh must clear after timeout.
Normal operation for about 5 seconds, unplug the HI91-only connector for about
20 seconds, reconnect and wait until capture ends. Keep MCU, USB, receiver and
other sensors connected/powered. No full control algorithm runs. This verifies
input classification only, not enabled-state stopping or rearming.

Capture startup/steady state/dropout/recovery separately. The decoder qualifies a replacement time epoch after stale input; recovery
must be confirmed in the CSV, not inferred solely from USB health.
Malformed/CRC fault injection is not performed by this hardware test. Production
CRC policy remains unchanged (permissive). No enabled actuation is allowed.


HI91 diagnosis columns are cumulative within the current USB session: forwarded
UART2 chunks/bytes/gaps, discarded resynchronization bytes, invalid length,
DecodeHi91Frame error, attitude validity error, nonfinite gyro/acceleration,
equal/backward device time, and accepted frames. Candidate/accepted/rejected
system_time_ms and latest chunk host timestamp allow the dropout and recovery
intervals to be compared. Counters classify the first rejection branch only;
CRC is included in decode errors only when the existing strict policy is enabled.
No UART electrical arrival is observed; increasing byte counts prove USB chunks
arrived, not that the source bytes are valid frames. Time recovery now requires the old accepted sample to exceed the configured
3 ms timeout, then at least three CRC-correct, otherwise valid backwards-time
frames with increasing device times, at most 20 ms apart in both sample and
device clocks, spanning at least 2 ms in host sample time. Repeated timestamps,
invalid frames, stream gaps and resynchronization reset qualification. The third
qualifying frame can publish; earlier candidates never refresh freshness.
Ordinary uint32 device wrap remains forward progress. time_rebases records
qualified changes; it establishes a plausible new epoch, not hardware proof of
a reboot. Normal forward-frame CRC policy remains unchanged. This is input
recovery only; it does not implement controller rearm or motor enable.

For a connected control run, use `scripts/run_hi91_baseline.sh NEW.csv`.
This prints only keep-connected instructions; do not unplug any connectors or
interrupt sensor power during its 60-second capture. Both IN and OUT failures
now report libusb completion status and actual/expected transfer lengths.

## Motor feedback loss, one side at a time

Use `sudo ./scripts/run_motor_feedback_loss.sh NEW.csv`. Keep MCU, USB, all
sensors and the other motor group powered; interrupt only the selected side's
motor supply (normal about 5 s, off about 20 s, restore until capture ends).
Do not unplug CAN or change bus termination. User's wiring permits side-group
power switching, not individual motors. Start with left side; use a separate
run/file for right side. The capture always submits only zero-current/Exit.
No motor-enable or control algorithm is run even after power recovery.

CSV records production ChassisInputReader motor_stale_mask, six independent ages,
feedback_invalid_mask and feedback_valid. Threshold is 3000 us, fresh at equality.
Bit order: left wheel, right wheel, left B, left D, right B, right D. Expected
side masks are left=13 (0x0d), right=50 (0x32), after initial startup settles.
Cold startup samples are included in stale totals, so verify transitions and
unaffected-side continuity rather than accepting nonzero totals alone. Invalid
or undecodable feedback age is UINT32_MAX. Aggregate feedback validity also
includes IMU/leg kinematics and need not become true merely because all motors
are fresh. Hardware receipt, motor errors and physical stop are not independently
proved by this capture. This is side-level feedback-loss/recovery coverage;
individual motor electrical isolation and enabled-controller safety remain untested.
