#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$EUID" -eq 0 ]]; then ulimit -l unlimited; fi
exec python3 "$root/chassis_controller/measurements/fault_recovery/usb_unplug.py" \
  --worker "$root/build/chassis_controller/fault_zero_worker" "$@"
