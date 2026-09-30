#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Run as root: sudo $0 native|generic|soem [interface]" >&2
  exit 2
fi

path=${1:-}
interface=${2:-enp88s0}
config=/etc/ethercat.conf

if [[ ! $path =~ ^(native|generic|soem)$ ]]; then
  echo "Usage: sudo $0 native|generic|soem [interface]" >&2
  exit 2
fi

systemctl stop ethercat.service 2>/dev/null || true

case "$path" in
  native)
    sed -i \
      -e 's/^DEVICE_MODULES=.*/DEVICE_MODULES="igc"/' \
      -e 's/^UPDOWN_INTERFACES=.*/UPDOWN_INTERFACES=""/' \
      "$config"
    systemctl start ethercat.service
    ;;
  generic)
    sed -i \
      -e 's/^DEVICE_MODULES=.*/DEVICE_MODULES="generic"/' \
      -e "s/^UPDOWN_INTERFACES=.*/UPDOWN_INTERFACES=\"$interface\"/" \
      "$config"
    modprobe igc
    ip link set dev "$interface" up
    systemctl start ethercat.service
    ;;
  soem)
    modprobe igc
    ip link set dev "$interface" up
    ;;
esac

echo "Selected EtherCAT path: $path"
if [[ $path == soem ]]; then
  ethtool -i "$interface"
else
  grep -E '^(MASTER0_DEVICE|DEVICE_MODULES|UPDOWN_INTERFACES)=' "$config"
  ethercat master
fi
