# HI91 startup queue investigation — 2026-10-21

Source: NUC `results/uart_line_idle_rt_acceptance.csv`, a complete 60-second
capture with the USB event owner's real-time configuration enabled.
The unchanged MCU line-idle candidate is installed; this investigation makes
no change to HI91 acquisition, UART10, MCU scheduling or USB packet format.

## Observed, not inferred

HI91 has 164039 chunks. MCU software-observation -> USB-submit queue timing is
p50/p99/max 57/170/7205 us. Exactly 21 chunks exceed 1000 us; the last is at
1.092680 ms after the first HI91 host observation. After the first 10 ms,
163991 remaining chunks have p50/p99/max 57/170/309 us. The full maximum remains
7205 us; separating startup is not permission to discard it from safety analysis.

The first publication groups show old observations draining rapidly:

| Host arrival since first HI91 chunk | MCU queue time | Chunk bytes |
| ---: | ---: | ---: |
| 0 us | 7122 us | 40 |
| 94.5 us | 7181 us | 40 |
| 103.1 us | 7205 us | 2 |
| 237.3 us | 6232 us | 40 |
| 748.6 us | 3508 us | 40 |
| 1043.7 us | 1611 us | 40 |
| 1350.9 us | 640 us | 40 |

These timestamps establish startup queue accumulation and drainage. They do
not locate the initial blocker or measure UART-wire -> host total latency.

## Source-level hypothesis and controlled experiment

UsbLink::Open starts the event thread and immediately submits Hello. The event
thread configures affinity/FIFO/mlockall and then submits the four IN transfers.
Those tasks are not ordered with Hello. MCU starts its session and queues sensor
data after Hello, potentially before the host has posted its receive transfers.
This ordering is a hypothesis for the accumulation, not a proven measured cause.

The capture now prints host-local startup milestones:

- `event_rt_setup_us`: event-thread start to setup completion (including preemption).
- `first_in_minus_hello_us`, `all_in_minus_hello_us`: successful libusb IN
  submission return times relative to Hello submission start; positive means later.
  They are software submission milestones, not on-wire packet timestamps.
- `hello_to_ready_us`: Hello submission start to validated Ready observation.

The new `--wait-in-ready` option waits until setup and all four successful IN
submissions finish before sending Hello. Default remains the original ordering.
It includes a bounded wait and fails on setup/IN-submit errors. It does not start
control, change motor outputs, insert warm-up exclusions or discard startup data.
Only startup milestones are stored; no periodic diagnostic packets are added.

On the same firmware, compare fresh-process starts (sudo required on NUC):

```bash
sudo ./scripts/run_uart_forwarding.sh --probe-status --seconds 10 \
  --csv ~/wbr_realtime_lab/results/hi91_startup_baseline.csv
sudo ./scripts/run_uart_forwarding.sh --probe-status --wait-in-ready --seconds 10 \
  --csv ~/wbr_realtime_lab/results/hi91_startup_preposted.csv
```

Repeat starts if needed: startup thread ordering can vary. A positive baseline
IN-minus-Hello interval matching accumulation, followed by a negative interval
and loss of startup backlog with the barrier, supports this hypothesis. If IN
was already submitted first or the backlog persists, inspect MCU Ready/first-IN
completion and uplink dequeue timing next. A single improvement does not by
itself prove real-time configuration cost is the cause.

The USB mock asserts all four IN requests precede Hello with the option enabled,
and verifies an IN-submit failure exits instead of sending Hello. All regular
production defaults remain unchanged pending hardware comparison.
