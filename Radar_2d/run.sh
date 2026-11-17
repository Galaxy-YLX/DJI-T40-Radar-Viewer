#!/usr/bin/env bash
# One-click launcher for the DJI T40 2D radar viewer (C++/Qt6).
#
# Usage:
#   ./run.sh                udp mode (default): attach directly to the radar's
#                           perception stream on UDP 40204
#   ./run.sh udp            same as default
#   ./run.sh demo           synthetic points, no radar hardware
#                           (extra args are forwarded, e.g. --record x.jsonl)
#   ./run.sh dump           decode one UDP frame as text and exit
#   ./run.sh selftest       decoder + frame-pipeline / replay regression
#   ./run.sh record x.jsonl live UDP + JSONL frame recording
#   ./run.sh replay x.jsonl [--speed N] [--loop]   JSONL replay
#
# udp / dump automatically check and (re)configure the environment:
#   - can0 is up             (PCAN-USB)     -> setup/start_can.sh
#   - host owns 192.168.8.1  (AR1021x AP)  -> setup/start_ap.sh
# Set AUTO_SETUP=0 to only check (and fail with a hint) instead of running sudo.
#
# Environment overrides:
#   RADAR_IP (192.168.8.2)  CAN_IFACE (can0)  MIN_AMP (0)
#   RANGE (25)  POINT_SIZE (0.5)  AZIMUTH_OFFSET (0)  MIN_RANGE (0)
#   MOTOR_RPM (900)  MOTOR_CONTROL_PORT (50008)  — runtime ESC setpoint
#   FLIP (1)  — 1 mirrors the point cloud left-right, 0 disables the mirror
#   STATUS_SHOW (1)  startup status panel; STATUS_SHOW=0 disables it
#   STATUS_LISTEN (1.0)  seconds of passive CAN traffic listening
#   CAN_PROBE (0)  set 1 to send a 0x1E/F9 request and wait for the ESC echo
#   RADAR_SSH_PORT (107)  radar port used for reachability check
#   FM_RAMP_SEC (0.5)  flight_mode=1 hold before the 1->2 pm edge (0=disable)
set -euo pipefail
cd "$(dirname "$0")"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

BIN=build/radar2d
RADAR_IP="${RADAR_IP:-192.168.8.2}"
CAN_IFACE="${CAN_IFACE:-can0}"
MIN_AMP="${MIN_AMP:-0}"
RANGE="${RANGE:-25}"
POINT_SIZE="${POINT_SIZE:-0.5}"
AZIMUTH_OFFSET="${AZIMUTH_OFFSET:-0}"
MIN_RANGE="${MIN_RANGE:-0}"
MOTOR_RPM="${MOTOR_RPM:-900}"
MOTOR_CONTROL_PORT="${MOTOR_CONTROL_PORT:-50008}"
FLIP="${FLIP:-1}"
STATUS_SHOW="${STATUS_SHOW:-1}"
STATUS_LISTEN="${STATUS_LISTEN:-1.0}"
CAN_PROBE="${CAN_PROBE:-0}"
RADAR_SSH_PORT="${RADAR_SSH_PORT:-107}"
FM_RAMP_SEC="${FM_RAMP_SEC:-0.5}"
MODE="${1:-udp}"

# ── Progress logging ────────────────────────────────────────────────────
STEP=0
step() {
    STEP=$((STEP + 1))
    echo "[run] [$STEP] $*"
}

echo "=================================================="
echo "[run] radar2d launcher  mode=$MODE  $(date '+%F %T')"
echo "[run] radar=$RADAR_IP can=$CAN_IFACE range=${RANGE}m min_amp=$MIN_AMP motor=${MOTOR_RPM}rpm flip=$FLIP"
echo "=================================================="

# Optional --no-flip only when FLIP=0 (the viewer flips by default).
FLIP_ARGS=()
if [ "$FLIP" = "0" ]; then
    FLIP_ARGS=(--no-flip)
fi

# ── Environment auto-config ───────────────────────────────────────────────
ensure_env() {
    local can_ok=0 ip_ok=0

    step "check CAN interface $CAN_IFACE"
    if [ -e "/sys/class/net/$CAN_IFACE" ] \
       && [ "$(cat "/sys/class/net/$CAN_IFACE/operstate" 2>/dev/null)" = "up" ]; then
        can_ok=1
    fi

    if [ "$can_ok" != 1 ] && [ "${AUTO_SETUP:-1}" = "1" ]; then
        step "start CAN: sudo setup/start_can.sh"
        echo "[env] $CAN_IFACE not up -> running setup/start_can.sh"
        sudo "$ROOT/setup/start_can.sh" || true
        if [ -e "/sys/class/net/$CAN_IFACE" ] \
           && [ "$(cat "/sys/class/net/$CAN_IFACE/operstate" 2>/dev/null)" = "up" ]; then
            can_ok=1
        fi
    fi

    # Always report CAN before touching AP: a failing AP must not hide the
    # CAN result.
    show_status can
    if [ "$can_ok" != 1 ]; then
        echo "[env] error: $CAN_IFACE not up (PCAN-USB adapter?)" >&2
        if [ "${AUTO_SETUP:-1}" != "1" ]; then
            echo "      run: sudo $ROOT/setup/start_can.sh" >&2
        fi
        exit 1
    fi

    step "check AP address 192.168.8.1"
    if ip -4 addr show 2>/dev/null | grep -q '192\.168\.8\.1'; then
        ip_ok=1
    fi

    if [ "$ip_ok" = 1 ]; then
        echo "[env] CAN/AP OK ($CAN_IFACE up, 192.168.8.1 owned)"
        show_status net
        return 0
    fi

    if [ "${AUTO_SETUP:-1}" != "1" ]; then
        show_status net
        echo "[env] not ready (can_ok=$can_ok ip_ok=$ip_ok). Run:" >&2
        echo "      sudo $ROOT/setup/start_ap.sh" >&2
        exit 1
    fi

    step "start AP: sudo setup/start_ap.sh"
    echo "[env] 192.168.8.1 missing -> running setup/start_ap.sh"
    if ! sudo "$ROOT/setup/start_ap.sh"; then
        show_status net
        echo "[env] error: start_ap.sh failed (AR1021x adapter / driver?)" >&2
        exit 1
    fi

    step "re-verify AP"
    if ! ip -4 addr show 2>/dev/null | grep -q '192\.168\.8\.1'; then
        show_status net
        echo "[env] error: still no 192.168.8.1 (AR1021x adapter plugged in?)" >&2
        exit 1
    fi

    show_status net
    echo "[env] environment configured"
}

# ── Startup status panel ────────────────────────────────────────────────
# show_status [can|net|all] — CAN is shown before AP setup so a failing AP
# can never hide the CAN result.
show_status() {
    [ "$STATUS_SHOW" = "1" ] || return 0
    local script="$ROOT/radar_client/link_status.py"
    [ -f "$script" ] || return 0

    local part="${1:-all}"
    local part_arg=() label="CAN + AP + radar"
    if [ "$part" = "can" ]; then
        part_arg=(--can-only)
        label="CAN only"
    elif [ "$part" = "net" ]; then
        part_arg=(--net-only)
        label="AP + radar"
    fi

    step "collect startup status ($label)"
    local probe_arg=()
    if [ "$CAN_PROBE" = "1" ]; then
        probe_arg=(--probe)
    fi
    python3 "$script" --iface "$CAN_IFACE" --host-ip 192.168.8.1 \
        --radar-ip "$RADAR_IP" --ssh-port "$RADAR_SSH_PORT" \
        --listen "$STATUS_LISTEN" "${part_arg[@]}" "${probe_arg[@]}" || true
}

# Build once if the binary is missing.
step "check binary $BIN"
if [ ! -x "$BIN" ]; then
    echo "[run] building radar2d ..."
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j
fi

COMMON_ARGS=(--range "$RANGE" --point-size "$POINT_SIZE"
             --azimuth-offset "$AZIMUTH_OFFSET"
             --min-range "$MIN_RANGE" --min-amp "$MIN_AMP"
             "${FLIP_ARGS[@]}")
UDP_ARGS=(--host "$RADAR_IP"
          --can-iface "$CAN_IFACE"
          --fc-client "$ROOT/radar_client/radar_min_client.py"
          --fm-ramp-sec "$FM_RAMP_SEC"
          --motor-rpm "$MOTOR_RPM"
          --motor-control-port "$MOTOR_CONTROL_PORT")

case "$MODE" in
  demo)
    shift 1
    step "launch synthetic-point demo"
    exec "$BIN" --demo --range "$RANGE" --point-size 0.5 "${FLIP_ARGS[@]}" "$@"
    ;;
  selftest)
    step "run decoder / frame-pipeline selftest"
    export QT_QPA_PLATFORM=offscreen
    V="$ROOT/t40-radar-link/android/app/src/test/resources"
    "$BIN" --selftest "$V/frames.bin:$V/expected.txt"
    "$BIN" --selftest "pipe:$V/frames.bin:$V/expected.txt"
    ;;
  replay)
    [ "$#" -ge 2 ] || { echo "usage: $0 replay <file.jsonl> [--speed N] [--loop]" >&2; exit 1; }
    FILE="$2"
    shift 2
    step "replay $FILE"
    exec "$BIN" --replay "$FILE" "${COMMON_ARGS[@]}" "$@"
    ;;
  record)
    [ "$#" -ge 2 ] || { echo "usage: $0 record <file.jsonl> [args...]" >&2; exit 1; }
    ensure_env
    FILE="$2"
    shift 2
    step "launch live UDP recorder -> $FILE"
    echo "[run] live UDP + recording to $FILE ..."
    exec "$BIN" --record "$FILE" "${UDP_ARGS[@]}" "${COMMON_ARGS[@]}" "$@"
    ;;
  dump)
    ensure_env
    step "start FC client (can-bridge $CAN_IFACE)"
    echo "[run] starting FC client (can-bridge $CAN_IFACE, no v2 listen) ..."
    python3 "$ROOT/radar_client/radar_min_client.py" \
        --can-bridge "$CAN_IFACE" --radar-ip "$RADAR_IP" \
        --fm-ramp-sec "$FM_RAMP_SEC" --no-v2-listen \
        >/tmp/radar2d_fc.log 2>&1 &
    FC_PID=$!
    trap 'kill "$FC_PID" 2>/dev/null || true' EXIT
    step "launch radar2d --dump"
    exec "$BIN" --host "$RADAR_IP" --dump "${COMMON_ARGS[@]}"
    ;;
  udp)
    ensure_env
    step "launch 2D viewer from radar UDP"
    # The viewer's 开机 / 关机 buttons own the FC client; starting it here
    # as well would fight over UDP 50007.
    echo "[run] launching 2D viewer from radar UDP (press 开机 to start the radar) ..."
    "$BIN" "${UDP_ARGS[@]}" "${COMMON_ARGS[@]}"
    ;;
  *)
    echo "usage: $0 [udp|demo|dump|selftest|record <file>|replay <file> [--speed N]]" >&2
    exit 1
    ;;
esac
