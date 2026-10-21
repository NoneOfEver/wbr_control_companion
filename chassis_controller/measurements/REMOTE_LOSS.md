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
