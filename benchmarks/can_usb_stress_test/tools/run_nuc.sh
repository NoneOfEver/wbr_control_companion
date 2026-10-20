#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
lab_dir="${WBR_REALTIME_LAB:-$HOME/wbr_realtime_lab}"
result_dir="$lab_dir/results/can_usb_stress_test/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$result_dir"
printf 'Results: %s\n' "$result_dir"
python3 -u "$script_dir/run.py" --seconds 65 --json "$result_dir/usb.json" "$@" 2>&1 | tee "$result_dir/host.log"
