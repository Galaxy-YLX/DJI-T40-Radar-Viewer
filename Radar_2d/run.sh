#!/usr/bin/env bash
# One-click launcher for the DJI T40 2D radar viewer (C++/Qt6).
#
# Usage:
#   ./run.sh                  live mode (default — no argument)
#   ./run.sh live             2D PPI via the DJI 1E61 v2 stream (UDP 40204)
#   ./run.sh demo             synthetic points, no radar hardware
#   ./run.sh dump             env setup + FC client, decode one 1E61 frame, exit
#
# The live/dump modes automatically check and (re)configure the environment:
#   - host owns 192.168.8.1  (AR1021x AP)  -> setup/start_ap.sh
#   - can0 is up             (PCAN-USB)     -> setup/start_can.sh
# Set AUTO_SETUP=0 to only check (and fail with a hint) instead of running sudo.
#
# Environment overrides:
#   RADAR_IP (192.168.8.2)   CAN_IFACE (can0)
#   RANGE (25)  POINT_SIZE (0.5)  AZIMUTH_OFFSET (0)  MIN_SNR (0)  MIN_RANGE (0)
#   FLIP (1)  — 1 mirrors the point cloud left-right, 0 disables the mirror
set -euo pipefail
cd "$(dirname "$0")"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

BIN=build/radar2d
RADAR_IP="${RADAR_IP:-192.168.8.2}"
CAN_IFACE="${CAN_IFACE:-can0}"
RANGE="${RANGE:-25}"
POINT_SIZE="${POINT_SIZE:-0.5}"
AZIMUTH_OFFSET="${AZIMUTH_OFFSET:-0}"
MIN_SNR="${MIN_SNR:-0}"
MIN_RANGE="${MIN_RANGE:-0}"
FLIP="${FLIP:-1}"

# Optional --no-flip only when FLIP=0 (the viewer flips by default).
FLIP_ARGS=()
if [ "$FLIP" = "0" ]; then
    FLIP_ARGS=(--no-flip)
fi

# ── Environment auto-config ─────────────────────────────────────────────
ensure_env() {
    local ip_ok=0 can_ok=0

    if ip -4 addr show 2>/dev/null | grep -q '192\.168\.8\.1'; then
        ip_ok=1
    fi
    if [ -e "/sys/class/net/$CAN_IFACE" ] \
       && [ "$(cat "/sys/class/net/$CAN_IFACE/operstate" 2>/dev/null)" = "up" ]; then
        can_ok=1
    fi

    if [ "$ip_ok" = 1 ] && [ "$can_ok" = 1 ]; then
        echo "[env] link OK (192.168.8.1 owned, $CAN_IFACE up)"
        return 0
    fi

    if [ "${AUTO_SETUP:-1}" != "1" ]; then
        echo "[env] not ready (ip_ok=$ip_ok can_ok=$can_ok). Run:" >&2
        echo "      sudo $ROOT/setup/start_ap.sh && sudo $ROOT/setup/start_can.sh" >&2
        exit 1
    fi

    if [ "$ip_ok" != 1 ]; then
        echo "[env] 192.168.8.1 missing -> running setup/start_ap.sh"
        sudo "$ROOT/setup/start_ap.sh"
    fi
    if [ "$can_ok" != 1 ]; then
        echo "[env] $CAN_IFACE not up -> running setup/start_can.sh"
        sudo "$ROOT/setup/start_can.sh"
    fi

    # Re-verify.
    if ! ip -4 addr show 2>/dev/null | grep -q '192\.168\.8\.1'; then
        echo "[env] error: still no 192.168.8.1 (AR1021x adapter plugged in?)" >&2
        exit 1
    fi
    if ! { [ -e "/sys/class/net/$CAN_IFACE" ] \
           && [ "$(cat "/sys/class/net/$CAN_IFACE/operstate" 2>/dev/null)" = "up" ]; }; then
        echo "[env] error: $CAN_IFACE still not up (PCAN-USB adapter?)" >&2
        exit 1
    fi
    echo "[env] environment configured"
}

# Build once if the binary is missing.
if [ ! -x "$BIN" ]; then
    echo "[run] building radar2d ..."
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j
fi

MODE="${1:-live}"
case "$MODE" in
  demo)
    exec "$BIN" --demo --range "$RANGE" --point-size 0.5 "${FLIP_ARGS[@]}"
    ;;
  dump)
    ensure_env
    echo "[run] starting FC client (can-bridge $CAN_IFACE, no v2 listen) ..."
    python3 "$ROOT/radar_client/radar_min_client.py" \
        --can-bridge "$CAN_IFACE" --radar-ip "$RADAR_IP" --no-v2-listen \
        >/tmp/radar2d_fc.log 2>&1 &
    FC_PID=$!
    trap 'kill "$FC_PID" 2>/dev/null || true' EXIT

    exec "$BIN" --host "$RADAR_IP" --dump
    ;;
  live)
    ensure_env
    echo "[run] starting FC client (can-bridge $CAN_IFACE, no v2 listen) ..."
    python3 "$ROOT/radar_client/radar_min_client.py" \
        --can-bridge "$CAN_IFACE" --radar-ip "$RADAR_IP" --no-v2-listen \
        >/tmp/radar2d_fc.log 2>&1 &
    FC_PID=$!
    trap 'kill "$FC_PID" 2>/dev/null || true' EXIT

    echo "[run] launching 2D viewer (DJI 1E61 v2 stream, UDP 40204) ..."
    "$BIN" --host "$RADAR_IP" --range "$RANGE" --point-size "$POINT_SIZE" \
        --azimuth-offset "$AZIMUTH_OFFSET" --min-snr "$MIN_SNR" \
        --min-range "$MIN_RANGE" "${FLIP_ARGS[@]}"
    ;;
  *)
    echo "usage: $0 [demo|live|dump]" >&2
    exit 1
    ;;
esac
