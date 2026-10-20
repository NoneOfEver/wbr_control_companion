#!/usr/bin/env python3
"""Host protocol/duplex tests; no PyUSB import or motor hardware is needed."""
import errno
import queue
import struct
import time
import unittest

from run import FRAME_MAGIC, Measurements, PendingPacket, encode_packet, run_benchmark


class DelayedEchoDevice:
    """Each reply has 4 ms latency but packets can overlap in the transport."""
    def __init__(self, corrupt=False, drop=False):
        self.replies = queue.Queue()
        self.corrupt = corrupt
        self.drop = drop

    def write(self, endpoint, packet, timeout):
        self.replies.put((time.perf_counter() + .004, bytes(packet)))
        return len(packet)

    def read(self, endpoint, length, timeout):
        try:
            ready_at, packet = self.replies.get(timeout=timeout / 1000)
        except queue.Empty as exc:
            raise TimeoutError(errno.ETIMEDOUT, 'simulated idle IN') from exc
        if self.drop:
            raise TimeoutError(errno.ETIMEDOUT, 'simulated lost reply')
        remaining = ready_at - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        if self.corrupt:
            packet = packet[:-1] + bytes([packet[-1] ^ 1])
        return packet


class UsbBenchmarkTests(unittest.TestCase):
    def test_wire_format_stays_compatible_with_firmware(self):
        packet = encode_packet(17, 2048)
        self.assertEqual(len(packet), 64)
        self.assertEqual(struct.unpack_from('<III', packet), (FRAME_MAGIC, 17, 2048))
        self.assertEqual(packet[12:], bytes(range(17, 69)))

    def test_duplex_overlaps_packets_without_hiding_roundtrip_latency(self):
        result = run_benchmark(DelayedEchoDevice(), .05, max_in_flight=8, progress=False)
        self.assertEqual(result['errors'], 0)
        self.assertEqual(result['exchanges'], result['sent_packets'])
        self.assertEqual(result['outstanding_packets'], 0)
        self.assertGreater(result['peak_in_flight'], 1)
        self.assertLessEqual(result['peak_in_flight'], 8)
        self.assertGreaterEqual(result['rtt_us_p50'], 3000)

    def test_roundtrip_keeps_only_one_packet_outstanding(self):
        result = run_benchmark(DelayedEchoDevice(), .025, mode='roundtrip', progress=False)
        self.assertEqual(result['errors'], 0)
        self.assertEqual(result['peak_in_flight'], 1)
        self.assertEqual(result['exchanges'], result['sent_packets'])

    def test_corruption_is_a_failure(self):
        result = run_benchmark(DelayedEchoDevice(corrupt=True), .025, progress=False)
        self.assertFalse(result['usb_pass'])
        self.assertTrue(any('Echo mismatch' in failure for failure in result['failure_details']))

    def test_missing_replies_stop_after_bounded_drain(self):
        result = run_benchmark(DelayedEchoDevice(drop=True), .01, max_in_flight=2,
                               drain_timeout_s=.02, progress=False)
        self.assertFalse(result['usb_pass'])
        self.assertEqual(result['exchanges'], 0)
        self.assertEqual(result['outstanding_packets'], 2)
        self.assertTrue(any('draining' in failure for failure in result['failure_details']))

    def test_sequence_matching_accepts_reordered_replies_but_rejects_duplicates(self):
        measurements = Measurements()
        for sequence in [10, 11]:
            packet = encode_packet(sequence, 0)
            measurements.pending[sequence] = PendingPacket(packet, time.perf_counter_ns())
        measurements.accept_reply(encode_packet(11, 0))
        measurements.accept_reply(encode_packet(10, 0))
        self.assertEqual(len(measurements.rtts_us), 2)
        self.assertFalse(measurements.pending)
        measurements.accept_reply(encode_packet(10, 0))
        self.assertTrue(measurements.aborted)
        self.assertIn('duplicate', measurements.failures[-1])


if __name__ == '__main__':
    unittest.main()
