#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
lab_dir="${WBR_REALTIME_LAB:-$(cd -- "$script_dir/../../.." && pwd)}"
companion_dir="${WBR_COMPANION_DIR:-$lab_dir}"
build_dir="$lab_dir/build/can_usb_host_rt"
cmake -S "$script_dir/../host" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DWBR_COMPANION_DIR="$companion_dir"
cmake --build "$build_dir" -j 2
ctest --test-dir "$build_dir" --output-on-failure
