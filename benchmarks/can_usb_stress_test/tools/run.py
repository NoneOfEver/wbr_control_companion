#!/usr/bin/env python3
"""Measure USB duplex throughput separately from per-packet round-trip latency."""
import argparse
from dataclasses import dataclass, field
import errno
import json
from pathlib import Path
import struct
import threading
import time

PACKET_BYTES = 64
FRAME_MAGIC = 0x53545253
OUT_ENDPOINT = 0x01
IN_ENDPOINT = 0x81


def encode_packet(sequence, previous_rtt_us):
    """Wire format is unchanged; the firmware reports previous RTT to VOFA."""
    header = struct.pack('<III', FRAME_MAGIC, sequence, previous_rtt_us)
    payload = bytes((sequence + offset) & 255 for offset in range(52))
    return header + payload


@dataclass
class PendingPacket:
    data: bytes
    submitted_ns: int


@dataclass
class Measurements:
    condition: threading.Condition = field(default_factory=threading.Condition)
    pending: dict = field(default_factory=dict)
    rtts_us: list = field(default_factory=list)
    failures: list = field(default_factory=list)
    sent: int = 0
    skipped_slots: int = 0
    peak_in_flight: int = 0
    last_received_ns: int = 0
    aborted: bool = False

    def fail(self, message):
        with self.condition:
            self.failures.append(message)
            self.aborted = True
            self.condition.notify_all()

    def accept_reply(self, reply):
        received_ns = time.perf_counter_ns()
        with self.condition:
            if len(reply) != PACKET_BYTES:
                self.fail(f'Unexpected IN length: {len(reply)}')
                return
            magic, sequence = struct.unpack_from('<II', reply)
            packet = self.pending.pop(sequence, None)
            if magic != FRAME_MAGIC or packet is None:
                self.fail(f'Unexpected or duplicate reply: sequence={sequence}')
                return
            if reply != packet.data:
                self.fail(f'Echo mismatch: sequence={sequence}')
                return
            self.rtts_us.append((received_ns - packet.submitted_ns) / 1000)
            self.last_received_ns = received_ns
            self.condition.notify_all()


def receive_replies(device, measurements, sender_done, reader_ready):
    """Keep IN reads outstanding independently of the 1 kHz OUT schedule."""
    reader_ready.set()
    while True:
        with measurements.condition:
            if measurements.aborted or (sender_done.is_set() and not measurements.pending):
                return
        try:
            reply = bytes(device.read(IN_ENDPOINT, PACKET_BYTES, timeout=100))
        except OSError as exc:
            # A read timeout is normal before the first OUT or during shutdown.
            if isinstance(exc, TimeoutError) or exc.errno == errno.ETIMEDOUT:
                continue
            measurements.fail(f'USB IN failed: {exc}')
            return
        measurements.accept_reply(reply)


def build_result(measurements, start_ns, sender_finished_ns, end_ns, mode, target_hz):
    rtts = sorted(measurements.rtts_us)
    send_elapsed = (sender_finished_ns - start_ns) / 1e9
    last_exchange = max(sender_finished_ns, measurements.last_received_ns)
    exchange_elapsed = (last_exchange - start_ns) / 1e9

    def percentile(fraction):
        if not rtts:
            return None
        return rtts[min(len(rtts) - 1, int(len(rtts) * fraction))]

    rate = len(rtts) / exchange_elapsed
    return dict(
        mode=mode,
        sent_packets=measurements.sent,
        exchanges=len(rtts),
        send_elapsed_s=send_elapsed,
        elapsed_s=exchange_elapsed,
        total_elapsed_s=(end_ns - start_ns) / 1e9,
        send_rate_hz=measurements.sent / send_elapsed,
        rate_hz=rate,
        errors=len(measurements.failures),
        failure_details=measurements.failures,
        outstanding_packets=len(measurements.pending),
        host_skipped_slots=measurements.skipped_slots,
        peak_in_flight=measurements.peak_in_flight,
        rtt_us_p50=percentile(.5),
        rtt_us_p99=percentile(.99),
        rtt_us_max=max(rtts, default=None),
        usb_pass=(not measurements.failures and not measurements.pending
                  and rate >= target_hz * .99),
    )


def run_benchmark(device, seconds, mode='duplex', max_in_flight=8, target_hz=1000,
                  drain_timeout_s=2, progress=True):
    """Duplex permits bounded overlap; roundtrip retains the old serial test.

    In both modes, each reply is matched against its exact packet and measured
    from the same host clock. Pipelining improves throughput without relabeling
    a 2 ms RTT as a 1 ms latency.
    """
    if seconds <= 0 or target_hz <= 0 or max_in_flight < 1:
        raise ValueError('Duration, target rate and in-flight limit must be positive')
    measurements = Measurements()
    sender_done = threading.Event()
    reader_ready = threading.Event()
    reader = None
    if mode == 'duplex':
        reader = threading.Thread(
            target=receive_replies,
            args=(device, measurements, sender_done, reader_ready),
            name='usb-in-reader',
        )
        reader.start()
        reader_ready.wait()
        # Yield once so the reader can submit its first libusb IN request.
        time.sleep(.001)

    period_ns = int(1e9 / target_hz)
    start_ns = deadline_ns = time.perf_counter_ns()
    end_of_send_ns = start_ns + int(seconds * 1e9)
    sequence = 0
    try:
        while time.perf_counter_ns() < end_of_send_ns:
            delay_ns = deadline_ns - time.perf_counter_ns()
            if delay_ns > 0:
                time.sleep(delay_ns / 1e9)
            if time.perf_counter_ns() >= end_of_send_ns:
                break

            with measurements.condition:
                # Bound queueing delay and memory if the receiver cannot keep up.
                while len(measurements.pending) >= max_in_flight and not measurements.aborted:
                    remaining = (end_of_send_ns - time.perf_counter_ns()) / 1e9
                    if remaining <= 0:
                        break
                    measurements.condition.wait(timeout=min(.05, remaining))
                if measurements.aborted or time.perf_counter_ns() >= end_of_send_ns:
                    break
                previous_rtt = int(measurements.rtts_us[-1]) if measurements.rtts_us else 0
                packet = encode_packet(sequence, previous_rtt)
                measurements.pending[sequence] = PendingPacket(packet, time.perf_counter_ns())
                measurements.peak_in_flight = max(
                    measurements.peak_in_flight, len(measurements.pending))
            try:
                written = device.write(OUT_ENDPOINT, packet, timeout=200)
                if written != PACKET_BYTES:
                    raise OSError(f'Short OUT transfer: {written}')
                measurements.sent += 1
                if mode == 'roundtrip':
                    reply = bytes(device.read(IN_ENDPOINT, PACKET_BYTES, timeout=200))
                    measurements.accept_reply(reply)
            except OSError as exc:
                measurements.fail(f'USB OUT/roundtrip failed: {exc}')
                break

            sequence += 1
            if progress and sequence % 1000 == 0:
                elapsed = (time.perf_counter_ns() - start_ns) / 1e9
                with measurements.condition:
                    latest_rtt = measurements.rtts_us[-1] if measurements.rtts_us else 0
                    print(f'USB OUT={measurements.sent / elapsed:.1f} Hz, '
                          f'IN={len(measurements.rtts_us) / elapsed:.1f} Hz, '
                          f'RTT={latest_rtt:.0f} us, '
                          f'in_flight={len(measurements.pending)}', flush=True)
            deadline_ns += period_ns
            late_ns = time.perf_counter_ns() - deadline_ns
            if late_ns > period_ns:
                skipped = late_ns // period_ns
                measurements.skipped_slots += skipped
                deadline_ns += skipped * period_ns
    except KeyboardInterrupt:
        measurements.fail('Interrupted by user')
    finally:
        sender_finished_ns = time.perf_counter_ns()
        sender_done.set()
        if reader is not None:
            drain_deadline = time.perf_counter() + drain_timeout_s
            with measurements.condition:
                while measurements.pending and not measurements.aborted:
                    remaining = drain_deadline - time.perf_counter()
                    if remaining <= 0:
                        measurements.fail('Timed out draining outstanding replies')
                        break
                    measurements.condition.wait(timeout=remaining)
            # Blocking IN calls have a 100 ms timeout, so the reader exits promptly.
            reader.join()
    return build_result(measurements, start_ns, sender_finished_ns,
                        time.perf_counter_ns(), mode, target_hz)


def linux_usb_speed_mbps(device):
    """Read the negotiated speed without resetting/reconfiguring the device."""
    root = Path('/sys/bus/usb/devices')
    if not root.exists():
        return None
    for entry in root.iterdir():
        try:
            if (int((entry / 'busnum').read_text()) == device.bus
                    and int((entry / 'devnum').read_text()) == device.address):
                return float((entry / 'speed').read_text())
        except (OSError, ValueError):
            continue
    return None


def main():
    # Delay PyUSB import so the benchmark/codec tests run without USB hardware.
    import usb.core
    import usb.util

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=65)
    parser.add_argument('--vid', type=lambda value: int(value, 0), default=0x34b7)
    parser.add_argument('--pid', type=lambda value: int(value, 0), default=0xffff)
    parser.add_argument('--json', default='usb_stress_result.json')
    parser.add_argument('--mode', choices=['duplex', 'roundtrip'], default='duplex')
    parser.add_argument('--max-in-flight', type=int, default=8)
    args = parser.parse_args()
    if args.seconds <= 0 or args.max_in_flight < 1:
        parser.error('seconds and max-in-flight must be positive')
    device = usb.core.find(idVendor=args.vid, idProduct=args.pid)
    if device is None:
        raise SystemExit('USB device not found')
    speed = linux_usb_speed_mbps(device)
    print(f'USB negotiated speed: {speed if speed is not None else "unknown"} Mbps; '
          f'mode={args.mode}', flush=True)
    device.set_configuration()
    usb.util.claim_interface(device, 0)
    try:
        result = run_benchmark(device, args.seconds, args.mode, args.max_in_flight)
    finally:
        usb.util.release_interface(device, 0)
        usb.util.dispose_resources(device)
    result['usb_speed_mbps'] = speed
    Path(args.json).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    print('USB result only: inspect CAN statistics separately. RTT remains a separate metric.')
    return 0 if result['usb_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
