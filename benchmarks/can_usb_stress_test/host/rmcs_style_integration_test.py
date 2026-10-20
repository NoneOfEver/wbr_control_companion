"""Test-only USB simulation: exercise real host threads without physical USB."""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

binary, mock = sys.argv[1:]
cases = [(False, True, "wbd", False), (True, True, "wbd", False), (False, False, "wbd", False)]
cases += [(False, True, order, True) for order in ("wbd", "wdb", "bwd", "bdw", "dwb", "dbw")]
for split, hardware_supported, order, fast in cases:
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        # A VM may pause this ordinary-priority process for tens of ms.
        # Collect enough control ticks to verify sampled diagnostics as well.
        command = [binary, "--no-rt", "--seconds", "0.5", "--json", str(directory / "stream.json"),
                   "--csv", str(directory / "commands.csv")]
        command += ["--can-order", order]
        if fast:
            command.append("--fast-ingress")
        if split:
            command.append("--split-out")
        environment = dict(os.environ, LD_PRELOAD=mock, RMCS_MOCK_FORBID_EVENT_LOCK="1")
        if not hardware_supported:
            environment["RMCS_MOCK_NO_HARDWARE"] = "1"
        result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=10)
        # Scheduling misses are allowed in a normal-priority CI simulation. The
        # transport, timestamps, exports and clean shutdown must still be valid.
        if result.returncode not in (0, 1):
            raise AssertionError(result.stdout + result.stderr)
        stream = json.loads((directory / "stream.json").read_text())
        timing = json.loads((directory / "control_latency.json").read_text())
        assert not stream["failure"], result.stdout + result.stderr
        assert stream["can_batch_order"] == order
        assert stream["fast_ingress"] == fast
        assert "CAN counters exported; MCU timestamp packets:" in result.stdout
        assert "USB closed; writing results" in result.stdout
        hardware = json.loads((directory / "can_hardware_timing.json").read_text())
        assert hardware["supported"] == hardware_supported
        assert hardware["invalid"] == 0
        if hardware_supported:
            assert hardware["missing"] == 0
            assert all(motor["n"] > 0 for motor in hardware["motors"])
            assert all(motor["driver_to_trigger_p50_us"] == 0 for motor in hardware["motors"])
            assert all(motor["trigger_to_batch_done_p50_us"] == 1 for motor in hardware["motors"])
        else:
            assert all(motor["n"] == 0 for motor in hardware["motors"])
        assert stream["received_acks"] == stream["sent_commands"] > 64, result.stdout + result.stderr
        assert stream["usb_out_transfers"] * (1 if split else 2) == stream["sent_commands"]
        assert stream["group_batching_pass"]
        assert timing["measurement_pass"], timing
        assert timing["invalid_traces"] == timing["unavailable_packets"] == 0
        assert stream["merged_out"] == (not split)
        analyzer = Path(__file__).resolve().parent.parent / "tools/analyze_control_timeline.py"
        subprocess.run([sys.executable, str(analyzer), str(directory)], check=True,
                       capture_output=True, text=True, timeout=5)
        timeline = json.loads((directory / "can_timeline_analysis.json").read_text())
        assert timeline["merged_out"] == (not split)
        if not split:
            assert timeline["paired_out"]["right_minus_left_mcu_receive"]["max_us"] == 0
print("Merged and split OUT: callbacks, command identity, diagnostics and cancellation passed")

# A 64-byte-only firmware must be rejected before a single 128-byte command.
# The mock aborts if that invariant is broken.
with tempfile.TemporaryDirectory() as temporary:
    directory = Path(temporary)
    environment = dict(os.environ, LD_PRELOAD=mock, RMCS_MOCK_CAPACITY_64="1",
                       RMCS_MOCK_FORBID_EVENT_LOCK="1")
    result = subprocess.run([binary, "--no-rt", "--seconds", "0.08", "--json",
                             str(directory / "stream.json")], env=environment,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 1, result.stdout + result.stderr
    assert "Merged OUT unsupported" in result.stderr
    assert not (directory / "stream.json").exists()
print("Old firmware rejected before oversized USB commands")
