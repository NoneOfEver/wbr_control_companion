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

Capture startup/steady state/dropout/recovery separately. The current decoder
rejects backwards HI91 device timestamps; power cycling may therefore expose
failure to recover, which must not be declared PASS solely because USB is healthy.
Malformed/CRC fault injection is not performed by this hardware test. Production
CRC policy remains unchanged (permissive). No enabled actuation is allowed.
