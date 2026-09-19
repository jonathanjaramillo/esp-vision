#!/usr/bin/env bash
# Build/flash wrapper for the visual-odometry firmware variants.
#
#   ./run.sh exp1          build the ORB-features firmware
#   ./run.sh exp1 upload   flash it
#   ./run.sh exp2 monitor  flash-free: open the serial monitor
#   ./run.sh ip            print this laptop's current en0 IP
#
# STREAM_HOST_IP (this laptop's IP on the K10's WiFi) is REQUIRED at build
# time: the firmware unicast-streams to it, and subnet broadcast measured DOA
# on this network (see firmware/k10-fast-corners/PLAN.md). It's auto-detected
# from en0 here so a DHCP change doesn't silently produce a firmware that
# streams into the void -- set STREAM_HOST_IP=... to override.
set -euo pipefail
cd "$(dirname "$0")"

usage() { grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 1; }

if [[ "${1:-}" == "ip" ]]; then
  ipconfig getifaddr en0 || { echo "en0 has no IPv4 address"; exit 1; }
  exit 0
fi

[[ "${1:-}" == exp1 || "${1:-}" == exp2 ]] || usage
case "$1" in
  exp1) proj=../../firmware/k10-fast-corners ;;
  exp2) proj=../../firmware/k10-lk-track ;;
esac
task="${2:-run}"

export PATH="/opt/homebrew/bin:$PATH"
: "${WIFI_SSID:?}"
: "${WIFI_PASSWORD:?}"
if [[ -z "${STREAM_HOST_IP:-}" ]]; then
  STREAM_HOST_IP="$(ipconfig getifaddr en0)" || {
    echo "error: en0 has no IPv4 -- connect the laptop to the K10's WiFi" >&2
    echo "(or set STREAM_HOST_IP= manually)" >&2; exit 1; }
fi
export STREAM_HOST_IP
echo "==> $1: pio $task  (SSID=$WIFI_SSID host=$STREAM_HOST_IP)"
cd "$proj"
case "$task" in
  run)    exec pio run ;;
  upload) exec pio run -t upload ;;
  monitor) exec pio device monitor ;;
  *) exec pio "$@" ;;
esac
