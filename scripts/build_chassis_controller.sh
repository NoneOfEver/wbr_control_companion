#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$root/chassis_controller" -B "$root/build/chassis_controller" -DCMAKE_BUILD_TYPE=Release
cmake --build "$root/build/chassis_controller" -j "${WBR_BUILD_JOBS:-2}"
ctest --test-dir "$root/build/chassis_controller" --output-on-failure
