#!/usr/bin/env python3
"""Create a compact Markdown comparison from ethercat_bench JSON files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def microseconds(value: int | float) -> str:
    return f"{float(value) / 1000.0:.3f}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rows: list[dict] = []
    for path in sorted(args.result_dir.glob("*.json")):
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream)
        if not {"backend", "period_us", "wakeup_jitter", "round_trip"} <= data.keys():
            continue
        data["file"] = path.name
        rows.append(data)
    if not rows:
        raise SystemExit(f"no JSON results in {args.result_dir}")

    lines = [
        "# EtherCAT benchmark comparison",
        "",
        "| Path | Load | DC | Period us | Samples | Wake p99.9 us | Wake p99.99 us | Wake max us | Exchange p99.9 us | RTT p99.9 us | RTT p99.99 us | RTT max us | WKC fail | Exchange fail | Bad echo | Bad CRC | Verdict |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in rows:
        wake = row["wakeup_jitter"]
        exchange = row["exchange_time"]
        rtt = row["round_trip"]
        failures = (
            row.get("wkc_failures", 0)
            + row.get("exchange_failures", 0)
            + row.get("bad_echoes", 0)
            + row.get("bad_payloads", 0)
        )
        lines.append(
            f"| {row.get('path', row['backend'])} | {row.get('load', 'unknown')} | "
            f"{'on' if row.get('distributed_clocks', False) else 'off'} | "
            f"{row['period_us']} | {wake['count']} | "
            f"{microseconds(wake['p999_ns'])} | "
            f"{microseconds(wake['p9999_ns'])} | "
            f"{microseconds(wake['max_ns'])} | "
            f"{microseconds(exchange['p999_ns'])} | "
            f"{microseconds(rtt['p999_ns'])} | "
            f"{microseconds(rtt['p9999_ns'])} | "
            f"{microseconds(rtt['max_ns'])} | {row.get('wkc_failures', 0)} | "
            f"{row.get('exchange_failures', 0)} | {row.get('bad_echoes', 0)} | "
            f"{row.get('bad_payloads', 0)} | {'PASS' if failures == 0 else 'FAIL'} |"
        )
    text = "\n".join(lines) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
