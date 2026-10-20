#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
lab_dir="${WBR_REALTIME_LAB:-$(cd -- "$script_dir/../../.." && pwd)}"
binary="$lab_dir/build/can_usb_host_rt/can_usb_host_rt"
if [[ ! -x "$binary" ]]; then
  echo "Build first: $script_dir/build_native.sh" >&2
  exit 1
fi
if [[ "$EUID" -eq 0 ]]; then ulimit -l unlimited; fi
result_dir="$lab_dir/results/can_usb_control_latency/$(date +%Y%m%d-%H%M%S)-native-bulk"
mkdir -p "$result_dir"
printf 'Results: %s\n' "$result_dir"
"$binary" --control-latency --feedback-grace-us 0 --seconds 60 --cpu 3 --priority 80 \
  --json "$result_dir/stream.json" --csv "$result_dir/commands.csv" "$@" \
  2>&1 | tee "$result_dir/host.log"
# Post-process only after the RT measurement and timestamp export have ended.
python3 "$script_dir/analyze_control_timeline.py" "$result_dir" \
  2>&1 | tee -a "$result_dir/host.log"
