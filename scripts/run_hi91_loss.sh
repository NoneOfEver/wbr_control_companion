#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$EUID" -eq 0 ]]; then ulimit -l unlimited; fi
exec "$root/build/chassis_controller/remote_loss" --imu "$@"
