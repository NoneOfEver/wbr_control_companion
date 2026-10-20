# UART forwarding capture

This independent tool uses the production UsbLink, four preposted Bulk IN
transfers and production UART decoder. The chassis controller executable has no
measurement observer installed. MCU firmware and the wire format are unchanged.

On Orb Ubuntu or NUC, with the production `usb_can_bridge` firmware installed:

```bash
cd ~/wbr_control_companion
./scripts/build_chassis_controller.sh
sudo ./scripts/run_uart_forwarding.sh --seconds 60 --csv /tmp/uart_forwarding.csv
```

Do not run another USB owner at the same time. The Hello handshake performs the
firmware's normal motor stop. This capture sends only empty 1 kHz command
heartbeats, never enable/current/torque frames. The firmware keeps its existing
stop/watchdog behavior. Exit sends Stop and drains the USB owner before export.
No sensor settings or MCU scheduling are changed for this baseline measurement.

Port 1 is UART10/SBUS (100000 8E2); port 2 is UART2/HI91 (921600 8N1).
Port 3 is optional referee UART and can legitimately have zero samples. The host
still requires the same production CAN/remote/HI91 capability bits. Data can be
captured without all motors replying. The test fails if there are no UART chunks,
USB errors or capture-memory overflow. A silent individual expected port must
also be investigated even if another port supplied data.

## What the numbers mean

- `MCU software observation -> USB IN submit`: MCU-local uint32 timestamp
  difference, wrap-safe. Starts when UartPort::Publish reads/copies the bytes;
  ends immediately before UsbSession::Send. It measures software queueing, not
  UART wire reception or completion of the USB transfer.
- `NUC USB chunk interarrival`: intervals between host event-owner observations
  of chunks from the same UART. Callback batching can produce very short gaps.
- `NUC decode and state publication`: host-local decode/Publish duration; does
  not include the test observer or the subsequent snapshot read.
- `NUC parser state update interval`: how often decoded state changes are
  observed, not source-to-host latency. Gap-induced remote invalidation also
  advances state sequence. Multiple frames in one chunk can produce one event.
- Stream gaps count missing chunk sequence numbers, not missing sensor frames;
  duplicate/backward sequence events are counted separately.

Capture timestamps use host CLOCK_MONOTONIC through the existing realtime API.
MCU absolute timestamps are never subtracted from host absolute timestamps.
CSV and percentiles are produced only after measurement, from fixed preallocated
memory. The optional observer adds a snapshot copy and bounded row write per
UART chunk; this is measurement overhead, not part of normal production runtime.
Other CAN/raw-IMU traffic continues over the same USB connection.

## Important measurement limits

**This is not yet a measurement of total UART-last-byte -> NUC latency.**
Current MCU UART timestamps are software observation timestamps. HI91 polls its
DMA ring every 1 ms and retains a 16-byte outstanding-write guard. Those bytes
may wait for the following sensor transmission; their delay is absent from the
reported MCU queue time. UART10 uses asynchronous RX-ready callbacks, and its
configured 1000-us timeout is not itself a measurement of callback latency.

USB IN submit -> NUC callback is also absent because host and MCU clocks are not
synchronized. Consequently a small queue percentile cannot establish a small
total forwarding delay. To close these two gaps, use a known UART stimulus plus
shared hardware timing (for example a logic analyzer and MCU trace GPIO), or add
a separately validated clock-offset/delay-bound protocol. Do not infer it from
sensor timestamps without verifying their units, epoch and acquisition meaning.

The native mock test checks timestamp statistics, deliberately missing chunks,
CSV export, empty-stream failure, and that heartbeats contain no motor commands.
It is not a physical UART or USB latency result.

## Offline batching investigation

Analyze the saved CSV without opening USB:

```bash
python3 chassis_controller/tools/analyze_uart_capture.py /tmp/uart_forwarding.csv
```

This groups closely spaced MCU observation timestamps (default 200 us), reports
chunk-size patterns and host data span, and locates the largest queue samples.
It separately reports queue timing after the first second while retaining the
full-capture outliers. Groups are software observation bursts, not proven UART
frame boundaries. Threshold changes can split/merge groups.

The 2026-10-21 NUC capture has exactly 365 SBUS bursts of 40/40/40/8 bytes;
its approximately 84-ms gaps are already present at MCU observation. Source
inspection found the HPM async UART init masks its PLIC source while RX enable
only enables the UART peripheral interrupt. A default-off MCU probe unmasks
only UART10's PLIC source for an A/B comparison; it does not shrink DMA buffers.
See `src/usb_can_bridge/UART10_INVESTIGATION.md` in the matching MCU repository.
This is not yet hardware-confirmed and does not change the production default.

The initial PLIC A/B run did **not** remove SBUS batching. The next opt-in MCU
image is identified by `URT2` in Ready and reports interrupt/partial/full RX
counters. Run `run_uart_forwarding.sh --probe-status --seconds 60 --csv PATH`
to read them before and after capture. The final read performs a second
stop/Hello handshake after measurement; no periodic diagnostic USB traffic is
added. Marker absence explicitly reports that the instrumented image is not
verified. Counts are cumulative since boot and include intervals outside capture.

URT3 adds IRQ-source classification and timeout-only DMA flush diagnostics.

`irq_rx_available`, `irq_timeout`, `irq_other` classify the driver's existing
IIR observation; `timeout_flush`, `zero_flush`, `dma_status_errors` describe
partial publication. `last_total/offset/pending/added` are the last timeout
snapshot, not counts or maxima. All-zero lengths with zero flush count mean
unavailable. The matched MCU probe requires the packaged HPM UART driver patch;
see its investigation notes. The host remains compatible with URT2.

URT4 reports seven disjoint cumulative IIR classes in Ready words 36..60:
no-pending (1), RX-available (4), timeout (12), TX-empty (2), line-status (6),
modem (0), and all unexpected IDs. Flags bit 3 identifies the MCU's opt-in
UART10 RFIFOT=0 hardware-erratum comparison. It does not report FCR readback.
URT4 replaces the URT3 flush/position fields and is decoded separately.
Both modes observe the driver's existing IIR read without a second read.
URT4 also displays flags bit 4 (`line_idle`), bit 5 (`line_idle_error`) and
bit 6 (`rx_offset_error`). The dust V3 line-idle candidate uses GPTMR events,
so its disabled UART FIFO IRQ counters remain zero. `fifo_erratum_probe`
identifies the older FIFO-only experiment; line-idle mode independently sets
RFIFOT=0 even with that experiment flag off. RX offset checks cover contiguous
bounded publications and buffer release, not UART wire losslessness.

Acceptance follow-up adds a five-session USB mock reconnect test and an SBUS
partial-stream reset contract. They verify host behavior, not physical detach or
hardware UART restart. Native driver contracts are in the MCU repository at
`test/native/uart_line_idle`. Actual UART silence and realtime USB capture remain
pending; the attempted automated RT run could not start because NUC sudo needs
a password. Run the existing tool without `--no-rt` for that capture. Do not infer
full chassis scheduling validation from the UART-only tool.

HI91's 7.205-ms full-capture maximum is now being investigated separately from
SBUS. See [startup evidence and optional IN-before-Hello A/B](HI91_STARTUP_INVESTIGATION.md).
The `--wait-in-ready` experiment changes host startup ordering only and needs no
MCU reflash. The original ordering is still the default until comparison.
