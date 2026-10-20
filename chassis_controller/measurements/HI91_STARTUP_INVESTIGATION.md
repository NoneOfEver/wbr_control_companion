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

The experiment used `--wait-in-ready` to wait for setup and all four IN
submissions before Hello. Following the measured A/B below, this order is now
mandatory in UsbLink::Open for every session. The old flag is accepted only for
command compatibility; there is no runtime opt-out.
It includes a bounded wait and fails on setup/IN-submit errors. It does not start
control, change motor outputs, insert warm-up exclusions or discard startup data.
Only startup milestones are stored; no periodic diagnostic packets are added.

## Measured A/B and production decision

The user ran two fresh-process real-time captures for 10 seconds on the same
MCU candidate, with no forwarding sequence gaps, offset errors or overflow:

| Host startup milestone / metric | Original order | IN ready before Hello |
| --- | ---: | ---: |
| Event-thread setup | 4519.81 us | 4564.73 us |
| All IN minus Hello | +4943.67 us | -98.052 us |
| Hello to validated Ready | 4988.75 us | 356.886 us |
| HI91 queue p50/p99/max | 53/169/4699 us | 55/175/313 us |

This supports the initial-IN delay as the cause of the measured startup backlog.
Event-thread setup still takes about 4.5 ms, but now happens before MCU session
publication begins. It does not prove every portion of setup is mlockall, or
measure UART-wire-to-host latency. No startup samples were discarded.

Production order is now: initialize USB -> configure event thread -> successfully
submit four IN transfers -> Hello -> validated Ready -> control operation.
Setup/IN errors prevent Hello. Every newly constructed USB session uses this
order. For a further default-mode regression, no special flag is needed:

```bash
sudo ./scripts/run_uart_forwarding.sh --probe-status --seconds 60 \
  --csv ~/wbr_realtime_lab/results/hi91_startup_production.csv
```

The USB mock asserts all four IN requests precede Hello with the option enabled,
and verifies an IN-submit failure exits instead of sending Hello. Default production and UART capture paths both use the mandatory barrier.
