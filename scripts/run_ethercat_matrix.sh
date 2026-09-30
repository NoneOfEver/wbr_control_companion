#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Run as root so SCHED_FIFO, mlockall and SOEM raw sockets are available." >&2
  exit 2
fi
if [[ $# -lt 3 ]]; then
  echo "Usage: sudo $0 VENDOR_ID PRODUCT_CODE RESULT_DIR [interface]" >&2
  exit 2
fi

vendor_id=$1
product_code=$2
result_dir=$3
interface=${4:-enp88s0}
duration_s=${DURATION_S:-600}
periods_us=${PERIODS_US:-"1000 500 250"}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_dir=$(cd -- "$script_dir/.." && pwd)
bench=${BENCHMARK_BIN:-$project_dir/build/ethercat_bench}
lab_build=$(cd -- "$project_dir/../.." && pwd)/build/wbr_control_companion/ethercat_bench
if [[ ! -x $bench && -x $lab_build ]]; then
  bench=$lab_build
fi
if [[ ! -x $bench ]]; then
  echo "ethercat_bench not found; set BENCHMARK_BIN or build the project" >&2
  exit 2
fi
mkdir -p "$result_dir"

stress_pid=
restoring_native=0
cleanup() {
  if [[ -n ${stress_pid:-} ]]; then
    kill "$stress_pid" 2>/dev/null || true
    wait "$stress_pid" 2>/dev/null || true
  fi
}
restore_native() {
  local exit_code=$?
  trap - EXIT INT TERM
  cleanup
  if [[ $restoring_native -eq 0 ]]; then
    restoring_native=1
    "$script_dir/select_ethercat_path.sh" native "$interface" || true
  fi
  exit "$exit_code"
}
trap restore_native EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

run_case() {
  local path=$1
  local load=$2
  local period=$3
  local backend=igh
  local extra=()
  if [[ $path == soem ]]; then
    backend=soem
    extra=(--interface "$interface")
  fi
  local stem="$result_dir/${path}-${load}-${period}us"
  "$bench" --backend "$backend" "${extra[@]}" \
    --vendor "$vendor_id" --product "$product_code" \
    --period-us "$period" --duration-s "$duration_s" --warmup 1000 \
    --cpu 2 --priority 80 --path-label "$path" --load-label "$load" \
    --csv "$stem.csv" --json "$stem.json"
}

for path in soem generic native; do
  "$script_dir/select_ethercat_path.sh" "$path" "$interface"
  for period in $periods_us; do
    run_case "$path" idle "$period"
    stress-ng --cpu 8 --cpu-method matrixprod --vm 2 --vm-bytes 2G \
      --iomix 2 --iomix-bytes 1G --timeout "$((duration_s + 20))s" \
      >"$result_dir/${path}-stress-${period}us.log" 2>&1 &
    stress_pid=$!
    run_case "$path" stress "$period"
    cleanup
    stress_pid=
  done
done

"$script_dir/select_ethercat_path.sh" native "$interface"
restoring_native=1
echo "Matrix complete; native path restored. Results: $result_dir"
