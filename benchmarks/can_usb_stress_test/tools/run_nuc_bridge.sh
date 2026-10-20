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
transport="bulk"
if [[ "${1:-}" == "bulk" || "${1:-}" == "interrupt" ]]; then
  transport="$1"
  shift
fi
result_dir="$lab_dir/results/can_usb_stream_test/$(date +%Y%m%d-%H%M%S)-native-$transport"
mkdir -p "$result_dir"
printf 'Results: %s\n' "$result_dir"
"$binary" --usb-transport "$transport" --can-stream --seconds 60 --cpu 3 --priority 80 \
  --json "$result_dir/stream.json" --csv "$result_dir/commands.csv" "$@" \
  2>&1 | tee "$result_dir/host.log"
