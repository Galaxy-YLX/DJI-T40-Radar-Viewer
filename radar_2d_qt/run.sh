#!/usr/bin/env bash
# One-click launcher for the DJI T40 2D radar viewer (C++/Qt6).
#
# Usage:
#   ./run.sh demo             synthetic points, no radar hardware
#   ./run.sh live             full pipeline: FC client + streamer + viewer
#   ./run.sh dump             deploy streamer, decode one frame, exit
#
# Environment overrides:
#   RADAR_IP (192.168.8.2)   CAN_IFACE (can0)
#   RANGE (25)  POINT_SIZE (2)  AZIMUTH_OFFSET (180)  MIN_SNR (0)  MIN_RANGE (0)
#
# Prerequisites for live/dump: the host owns 192.168.8.1 and can0 is up
# (see setup/start_ap.sh + setup/start_can.sh), radar powered on WiFi.
set -euo pipefail
cd "$(dirname "$0")"

BIN=build/radar2d
RADAR_IP="${RADAR_IP:-192.168.8.2}"
CAN_IFACE="${CAN_IFACE:-can0}"
RANGE="${RANGE:-25}"
POINT_SIZE="${POINT_SIZE:-2}"
AZIMUTH_OFFSET="${AZIMUTH_OFFSET:-180}"
MIN_SNR="${MIN_SNR:-0}"
MIN_RANGE="${MIN_RANGE:-0}"

VIEWER_ARGS=(--host "$RADAR_IP" --range "$RANGE" --point-size "$POINT_SIZE"
             --azimuth-offset "$AZIMUTH_OFFSET" --min-snr "$MIN_SNR"
             --min-range "$MIN_RANGE")

# Build once if the binary is missing.
if [ ! -x "$BIN" ]; then
    echo "[run] building radar2d ..."
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j
fi

MODE="${1:-live}"
case "$MODE" in
  demo)
    exec "$BIN" --demo --range "$RANGE" --point-size "$POINT_SIZE"
    ;;
  dump)
    python3 "$(dirname "$0")/../radar_client/deploy_side_streamer.py" \
        --radar-ip "$RADAR_IP"
    exec "$BIN" --host "$RADAR_IP" --dump
    ;;
  live)
    echo "[run] starting FC client (can-bridge $CAN_IFACE) -> /tmp/radar2d_fc.log"
    python3 "$(dirname "$0")/../radar_client/radar_min_client.py" \
        --can-bridge "$CAN_IFACE" --radar-ip "$RADAR_IP" \
        >/tmp/radar2d_fc.log 2>&1 &
    FC_PID=$!
    trap 'kill "$FC_PID" 2>/dev/null || true' EXIT

    echo "[run] deploying SIDE streamer ..."
    python3 "$(dirname "$0")/../radar_client/deploy_side_streamer.py" \
        --radar-ip "$RADAR_IP"

    echo "[run] launching 2D viewer ..."
    "$BIN" "${VIEWER_ARGS[@]}"
    ;;
  *)
    echo "usage: $0 [demo|live|dump]" >&2
    exit 1
    ;;
esac
