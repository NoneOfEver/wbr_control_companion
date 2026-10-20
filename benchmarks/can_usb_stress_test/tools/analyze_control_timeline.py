#!/usr/bin/env python3
"""Reconstruct the recorded CAN/USB timeline without synchronizing host/MCU clocks.

TX timestamps are driver submission and WHOLE-BATCH completion, not individual
CAN wire completions. RX is an observed feedback; it carries no command sequence.
Only exported feedback can be reconstructed. Run after capture, never in the RT loop.
"""
import argparse
import bisect
import csv
import json
from collections import defaultdict
from pathlib import Path


def relative_us(timestamp, reference):
    """Signed local timeline across 32-bit MCU microsecond wrap (< 2**31 us)."""
    return ((int(timestamp) - reference + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)


def distribution(values):
    if not values:
        return {"n": 0}
    ordered = sorted(values)
    return {
        "n": len(values),
        "min_us": round(ordered[0], 3),
        "p50_us": round(ordered[int((len(ordered) - 1) * 0.50)], 3),
        "p99_us": round(ordered[int((len(ordered) - 1) * 0.99)], 3),
        "max_us": round(ordered[-1], 3),
    }


def rows(path):
    with path.open(newline="") as stream:
        yield from csv.DictReader(stream)


def analyze(directory):
    stream = json.loads((directory / "stream.json").read_text())
    commands = defaultdict(dict)
    reference = None
    for row in rows(directory / "commands.csv"):
        if int(row["status"]):
            continue
        if reference is None:
            reference = int(row["mcu_out_us"])
        commands[int(row["cycle"])][int(row["group"])] = (
            int(row["submit_ns"]), relative_us(row["mcu_out_us"], reference)
        )
    if reference is None:
        raise ValueError("No successful command timestamps")

    feedback = defaultdict(list)
    for row in rows(directory / "feedback.csv"):
        feedback[row["motor"]].append((
            relative_us(row["mcu_can_rx_us"], reference),
            int(row["host_receive_ns"]),
            relative_us(row["mcu_in_submit_us"], reference),
        ))
    for samples in feedback.values():
        samples.sort()
    rx_times = {name: [sample[0] for sample in samples]
                for name, samples in feedback.items()}

    paired = defaultdict(list)
    for pair in commands.values():
        if 0 not in pair or 1 not in pair:
            continue
        left, right = pair[0], pair[1]
        host_delta = (right[0] - left[0]) / 1000
        mcu_delta = right[1] - left[1]
        paired["right_minus_left_host_submit"].append(host_delta)
        paired["right_minus_left_mcu_receive"].append(mcu_delta)
        paired["right_minus_left_out_path"].append(mcu_delta - host_delta)
        paired["second_minus_first_out_path"].append(
            (mcu_delta - host_delta) * (1 if host_delta >= 0 else -1))

    metrics = defaultdict(lambda: defaultdict(list))
    counts = defaultdict(lambda: {"sampled_batches": 0, "rx_before_batch_done": 0,
                                  "no_exported_rx_after_driver": 0})
    examples = {}
    for row in rows(directory / "control_latency.csv"):
        name = row["motor"]
        out = relative_us(row["mcu_out_us"], reference)
        driver = relative_us(row["mcu_driver_us"], reference)
        done = relative_us(row["mcu_batch_done_us"], reference)
        observed = relative_us(row["mcu_next_feedback_us"], reference)
        if not out <= driver <= done:
            raise ValueError(f"Invalid TX timeline: sequence={row['sequence']} {name}")
        counts[name]["sampled_batches"] += 1
        values = metrics[name]
        values["mcu_out_to_driver"].append(driver - out)
        values["driver_to_batch_done"].append(done - driver)
        values["batch_done_to_recorded_next_rx"].append(observed - done)
        host_end = int(row["next_feedback_host_receive_ns"])
        if host_end:
            # Clock OFFSETS cancel. This duration residual still assumes both
            # clocks' time units are accurate over the short exchange. It is
            # NOT an independently measured USB OUT or USB IN one-way delay.
            residual = ((host_end - int(row["host_submit_ns"])) / 1000
                        - (observed - out))
            values["outside_mcu_out_to_recorded_rx"].append(residual)
        times = rx_times.get(name, [])
        index = bisect.bisect_left(times, driver)
        if index == len(times):
            counts[name]["no_exported_rx_after_driver"] += 1
            continue
        first = times[index]
        counts[name]["rx_before_batch_done"] += first < done
        values["driver_to_first_exported_rx"].append(first - driver)
        values["first_rx_to_usb_in_submit"].append(feedback[name][index][2] - first)
        values["host_submit_to_first_postdriver_rx_observation"].append(
            (feedback[name][index][1] - int(row["host_submit_ns"])) / 1000)
        if first < done:
            values["prebatch_rx_to_recorded_next_rx"].append(observed - first)
        if name not in examples and first < done:
            lo = bisect.bisect_left(times, driver - 300)
            hi = bisect.bisect_right(times, done + 1500)
            examples[name] = {
                "sequence": int(row["sequence"]), "mcu_out_is_zero_us": out,
                "driver_after_out_us": driver - out,
                "batch_done_after_out_us": done - out,
                "recorded_next_rx_after_out_us": observed - out,
                "exported_rx_after_out_us": [value - out for value in times[lo:hi]],
            }

    return {
        "source": str(directory.resolve()),
        "merged_out": stream.get("merged_out", False),
        "out_order": stream.get("out_order", "left-right (historical default)"),
        "feedback_grace_us": stream.get("feedback_grace_us"),
        "limits": [
            "CAN TX endpoints are driver submission and whole-batch completion; per-frame wire TX is unavailable",
            "Feedback has no command sequence: before/after TX is temporal observation, never execution proof",
            "Only exported CAN feedback is available; mailbox coalescing may hide received frames",
            "Outside-MCU residual combines USB OUT/IN and host queue/callback time; it is not either one-way delay",
        ],
        "paired_out": {name: distribution(values) for name, values in paired.items()},
        "motors": {name: {**counts[name], **{
            field: distribution(values) for field, values in segments.items()
        }} for name, segments in metrics.items()},
        "prebatch_examples": examples,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="Completed control-latency result directory")
    parser.add_argument("--output", type=Path, help="Alternative writable JSON path")
    args = parser.parse_args()
    result = analyze(args.directory)
    output = args.output or args.directory / "can_timeline_analysis.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(f"Saved: {output}")
    print("OUT order:", result["out_order"])
    if result["merged_out"]:
        print("Both logical CAN groups share one 128-byte OUT; paired zero skew is not a USB one-way delay measurement")
    for field, values in result["paired_out"].items():
        print(field, "p50/p99 us:", values.get("p50_us"), values.get("p99_us"))
    for name, values in result["motors"].items():
        before = values["rx_before_batch_done"]
        total = values["sampled_batches"]
        print(f"{name}: RX between driver submit and batch done {before}/{total}; "
              f"RX->IN submit p50={values.get('first_rx_to_usb_in_submit', {}).get('p50_us')} us")


if __name__ == "__main__":
    main()
