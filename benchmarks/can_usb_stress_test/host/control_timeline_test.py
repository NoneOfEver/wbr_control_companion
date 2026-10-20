"""Check reconstruction across MCU wrap, reversed OUT order and pre-batch RX."""
import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

script = Path(__file__).resolve().parent.parent / "tools/analyze_control_timeline.py"
spec = importlib.util.spec_from_file_location("timeline", script)
timeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timeline)


def write_rows(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


class TimelineTest(unittest.TestCase):
    def fixture(self, directory, right_first):
        base = 0xFFFFFFF0
        stamp = lambda delta: (base + delta) & 0xFFFFFFFF
        offsets = [121, 0] if right_first else [0, 121]
        order = [1, 0] if right_first else [0, 1]
        (directory / "stream.json").write_text(json.dumps({
            "feedback_grace_us": 0,
            "out_order": "right-left" if right_first else "left-right",
        }))
        commands, feedback, sampled = [], [], []
        for group in order:
            offset = offsets[group]
            host = 1000000 + (6000 if offset else 0)
            motor = "right_wheel" if group else "left_wheel"
            commands.append(dict(cycle=0, group=group, status=0,
                                 submit_ns=host, mcu_out_us=stamp(offset)))
            first = 600 if group else 100
            next_rx = first if group else 1100
            for rx in [first, 1100] if group == 0 else [first]:
                feedback.append(dict(motor=motor, mcu_can_rx_us=stamp(offset + rx),
                                     mcu_in_submit_us=stamp(offset + rx + 10),
                                     host_receive_ns=host + (rx + 100) * 1000))
            sampled.append(dict(sequence=group, motor=motor, mcu_out_us=stamp(offset),
                                mcu_driver_us=stamp(offset + 20),
                                mcu_batch_done_us=stamp(offset + 400),
                                mcu_next_feedback_us=stamp(offset + next_rx),
                                host_submit_ns=host,
                                next_feedback_host_receive_ns=host + (next_rx + 100) * 1000))
        write_rows(directory / "commands.csv", commands)
        write_rows(directory / "feedback.csv", feedback)
        write_rows(directory / "control_latency.csv", sampled)

    def test_wrap_and_prebatch_feedback(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.fixture(directory, False)
            result = timeline.analyze(directory)
        paired = result["paired_out"]
        self.assertEqual(paired["right_minus_left_out_path"]["p50_us"], 115)
        self.assertEqual(paired["second_minus_first_out_path"]["p50_us"], 115)
        left = result["motors"]["left_wheel"]
        self.assertEqual(left["rx_before_batch_done"], 1)
        self.assertEqual(left["driver_to_batch_done"]["p50_us"], 380)
        self.assertEqual(left["first_rx_to_usb_in_submit"]["p50_us"], 10)
        self.assertEqual(left["prebatch_rx_to_recorded_next_rx"]["p50_us"], 1000)
        self.assertEqual(left["outside_mcu_out_to_recorded_rx"]["p50_us"], 100)
        self.assertEqual(result["motors"]["right_wheel"]["rx_before_batch_done"], 0)

    def test_reversed_order_keeps_signed_direction(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.fixture(directory, True)
            result = timeline.analyze(directory)
        paired = result["paired_out"]
        self.assertEqual(paired["right_minus_left_out_path"]["p50_us"], -115)
        self.assertEqual(paired["second_minus_first_out_path"]["p50_us"], 115)


if __name__ == "__main__":
    unittest.main()
