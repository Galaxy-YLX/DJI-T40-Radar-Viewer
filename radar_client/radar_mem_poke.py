#!/usr/bin/env python3
"""
radar_mem_poke.py — Deploy and invoke the on-target ptrace writer
(radar_mem_poke ELF, built from radar_mem_poke.c) against app.elf on
the DJI T40 radar. Reuses SSH helpers from radar_viewer.

Common tasks:

  # Activate side antenna (g_flight_mode_from_fc = 2)
  python3 radar_mem_poke.py activate

  # Revert (= 0)
  python3 radar_mem_poke.py deactivate

  # Force power_mode override: byte_17A3F8 = 0 (exits 0xFCFCFCFC channel mask)
  python3 radar_mem_poke.py pm0
  python3 radar_mem_poke.py pm3   # real-flight mode

  # Arbitrary poke
  python3 radar_mem_poke.py poke 0x283295 2
  python3 radar_mem_poke.py poke 0x17A3F8 0 1 2 3   # write 4 bytes starting @17A3F8

  # Read a status snapshot (uses dd on /proc/PID/mem, no ptrace)
  python3 radar_mem_poke.py status

Options:
  --ssh USER@HOST    SSH target  (default: root@192.168.8.2)
  --ssh-port N       SSH port    (default: 107)
  --password PW      SSH pw      (default: root)
  --no-redeploy      skip SFTP upload if remote binary already present
"""

from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

from radar_viewer import (
    DEFAULT_SSH_TARGET, DEFAULT_SSH_PORT, DEFAULT_SSH_PASSWORD,
    ssh_run_cmd, scp_upload,
)

POKE_REMOTE_PATH = "/tmp/radar_mem_poke"
LOCAL_BIN_NAME   = "radar_mem_poke"

# Known addresses (from prior analysis)
ADDR_FLIGHT_MODE  = 0x283295    # g_flight_mode_from_fc
ADDR_POWER_SRC    = 0x17A3F8    # byte_17A3F8 (power_mode source)
ADDR_DFA_STATE    = 0xC21DF31
ADDR_CFAR_ERROR   = 0xC2D5C64
ADDR_ROTATION     = 0xC2CF824   # rot_angle + sectors_decoded
ADDR_CFAR_SECTOR  = 0xC21DF50
ADDR_OMNI_SNAP    = 0xC140DD8
ADDR_POINT_COUNT  = 0xC442F90
ADDR_DFA_COND     = 0xC2CF810
ADDR_POWER_MODE   = 0xC2CF7F0   # live power_mode (copy of byte_17A3F8)
ADDR_MOTOR_CMD    = 0x282DEC    # unk_282DEC — motor FSM gate, 2 = MOTOR_RUNING
ADDR_MOTOR_FIX    = 0x282DA0    # unk_282DA0 — FIX_MODE branch flag
ADDR_MOTOR_RPM    = 0x282D74    # unk_282D74 — target rpm setpoint (u16)
# Areacode cache (get_drone_areacode_ack_handler populates these).
# 156 = China → pm2 (800M low PGA), 408±1 → pm1, anything else (0) → pm3.
ADDR_AREACODE_FLAG  = 0xCD72040   # unk_CD72040 — "areacode ACK received" flag
ADDR_AREACODE_VALUE = 0xCD72048   # unk_CD72048 — uint64 LE, the ACK value
ADDR_AREACODE_CACHE = 0xCD72038   # qword_CD72038 — flash-cached uint64

# ── fixed_height pipeline tunables (0x177578 control block) ─────────────
# Controls the 32×64 raw-magnitude heatmap peak extraction used by the
# heatmap viewer. Lowering thresholds makes small/weak reflections show up.
ADDR_FH_RANGE_START = 0x177578  # u16, default 1  — peak detect range bin start
ADDR_FH_LAT_START   = 0x17757A  # u16, default 12 — peak detect lateral bin start
ADDR_FH_RANGE_END   = 0x17757C  # u16, default 31 — peak detect range bin end
ADDR_FH_LAT_END     = 0x17757E  # u16, default 52 — peak detect lateral bin end
ADDR_FH_NB_SIZE     = 0x177580  # u16, default 64 — CFAR neighborhood size
ADDR_FH_DOA_WIN     = 0x177582  # u16, default 64 — DOA window
ADDR_FH_GLOBAL_MUL  = 0x177584  # f32, default 7.0  — global avg × N threshold
ADDR_FH_RANGE_SCALE = 0x177588  # f32, default 0.73   — m/bin (DO NOT write)
ADDR_FH_LAT_SCALE   = 0x17758C  # f32, default 0.2296 — m/bin (DO NOT write)
ADDR_FH_LOCAL_MUL   = 0x177590  # f32, default 3.0  — local mean × N threshold
ADDR_FH_OS_FRAC     = 0x177594  # f32, default 0.1667 — order-statistic frac

FH_DEFAULTS = {
    "sens_default":  {"global_mul": 7.0, "local_mul": 3.0},
    "sens_medium":   {"global_mul": 3.0, "local_mul": 1.8},
    "sens_high":     {"global_mul": 1.5, "local_mul": 1.2},
    "sens_max":      {"global_mul": 1.0, "local_mul": 1.0},
}

# Physical constants from IDA (must match fixed_height pipeline)
FH_RANGE_M_PER_BIN = 0.73
FH_LAT_M_PER_BIN   = 0.2296
FH_LAT_CENTER_BIN  = 32
FH_MAX_RANGE_BIN   = 31
FH_MAX_LAT_BIN     = 63


# ── Deploy helpers ──────────────────────────────────────────────────────
def deploy(target: str, port: int, password: str,
           redeploy: bool = True) -> None:
    script_dir = Path(__file__).resolve().parent
    local_bin = script_dir / LOCAL_BIN_NAME
    if not local_bin.exists():
        raise RuntimeError(f"Local binary missing: {local_bin}. "
                           f"Compile with arm toolchain first.")

    if not redeploy:
        rc, out, _ = ssh_run_cmd(target,
            f"test -x {POKE_REMOTE_PATH} && echo OK",
            port=port, password=password, timeout=5)
        if b"OK" in out:
            return

    scp_upload(target, str(local_bin), POKE_REMOTE_PATH,
               port=port, password=password)


def get_pid(target: str, port: int, password: str) -> int:
    rc, out, err = ssh_run_cmd(target, "pidof app.elf",
                                port=port, password=password, timeout=5)
    if not out.strip():
        raise RuntimeError("app.elf not running on target")
    return int(out.decode().strip().split()[0])


def poke(target: str, port: int, password: str,
         pid: int, addr: int, values: list[int]) -> str:
    hex_vals = " ".join(f"0x{v & 0xff:02x}" for v in values)
    cmd = f"{POKE_REMOTE_PATH} {pid} 0x{addr:x} {hex_vals}"
    rc, out, err = ssh_run_cmd(target, cmd,
                                port=port, password=password, timeout=10)
    out_s = out.decode(errors='ignore').strip()
    err_s = err.decode(errors='ignore').strip()
    if rc != 0:
        raise RuntimeError(f"poke failed (rc={rc}): out='{out_s}' err='{err_s}'")
    return out_s or err_s


# ── Typed poke helpers ──────────────────────────────────────────────────
def poke_u16(target, port, password, pid, addr, value):
    lo = value & 0xFF
    hi = (value >> 8) & 0xFF
    return poke(target, port, password, pid, addr, [lo, hi])


def poke_float(target, port, password, pid, addr, value):
    b = struct.pack("<f", float(value))
    return poke(target, port, password, pid, addr, list(b))


# ── Status (read-only, via dd on /proc/PID/mem) ─────────────────────────
def _dd_read(target, port, password, pid, addr, size):
    cmd = f"dd if=/proc/{pid}/mem bs=1 skip={addr} count={size} 2>/dev/null"
    rc, out, _ = ssh_run_cmd(target, cmd, port=port, password=password, timeout=10)
    return out


def _read_u16(target, port, password, pid, addr):
    d = _dd_read(target, port, password, pid, addr, 2)
    if d and len(d) >= 2:
        return struct.unpack_from('<H', d, 0)[0]
    return None


def _read_float(target, port, password, pid, addr):
    d = _dd_read(target, port, password, pid, addr, 4)
    if d and len(d) >= 4:
        return struct.unpack_from('<f', d, 0)[0]
    return None


# ── Sensitivity / range tuning commands ─────────────────────────────────
def show_sens(target, port, password, pid):
    """Print current fixed_height peak-detection params."""
    rs = _read_u16(target, port, password, pid, ADDR_FH_RANGE_START)
    re = _read_u16(target, port, password, pid, ADDR_FH_RANGE_END)
    ls = _read_u16(target, port, password, pid, ADDR_FH_LAT_START)
    le = _read_u16(target, port, password, pid, ADDR_FH_LAT_END)
    gm = _read_float(target, port, password, pid, ADDR_FH_GLOBAL_MUL)
    lm = _read_float(target, port, password, pid, ADDR_FH_LOCAL_MUL)
    nb = _read_u16(target, port, password, pid, ADDR_FH_NB_SIZE)
    print("=== fixed_height heatmap peak-detection params ===")
    if rs is not None and re is not None:
        print(f"  range bins  : [{rs}..{re}]  "
              f"({rs*FH_RANGE_M_PER_BIN:.2f} m .. {re*FH_RANGE_M_PER_BIN:.2f} m)")
    if ls is not None and le is not None:
        l_min = (ls - FH_LAT_CENTER_BIN) * FH_LAT_M_PER_BIN
        l_max = (le - FH_LAT_CENTER_BIN) * FH_LAT_M_PER_BIN
        print(f"  lateral bins: [{ls}..{le}]  ({l_min:+.2f} m .. {l_max:+.2f} m)")
    if nb is not None:
        print(f"  CFAR neighborhood: {nb}")
    if gm is not None:
        print(f"  global_mul  : {gm:.3f}  (peak > avg × this)")
    if lm is not None:
        print(f"  local_mul   : {lm:.3f}  (peak > local mean × this)")


def set_sens(target, port, password, pid, preset):
    """Apply a threshold preset: sens_default / sens_medium / sens_high / sens_max."""
    params = FH_DEFAULTS.get(preset)
    if params is None:
        raise ValueError(f"unknown preset: {preset}; choices: {list(FH_DEFAULTS)}")
    print(f"[sens] applying '{preset}': {params}")
    poke_float(target, port, password, pid, ADDR_FH_GLOBAL_MUL, params["global_mul"])
    poke_float(target, port, password, pid, ADDR_FH_LOCAL_MUL,  params["local_mul"])


def set_range(target, port, password, pid, r_min_m, r_max_m):
    """Set peak detection range in meters (clamped to 0..23m heatmap FOV)."""
    rs = max(0, min(FH_MAX_RANGE_BIN, round(r_min_m / FH_RANGE_M_PER_BIN)))
    re_ = max(0, min(FH_MAX_RANGE_BIN, round(r_max_m / FH_RANGE_M_PER_BIN)))
    if rs >= re_:
        raise ValueError(f"range_min ({r_min_m}) must be < range_max ({r_max_m})")
    print(f"[range] setting peak detect range bins [{rs}..{re_}] "
          f"({rs*FH_RANGE_M_PER_BIN:.2f}m .. {re_*FH_RANGE_M_PER_BIN:.2f}m)")
    poke_u16(target, port, password, pid, ADDR_FH_RANGE_START, rs)
    poke_u16(target, port, password, pid, ADDR_FH_RANGE_END,   re_)


def set_lateral(target, port, password, pid, l_min_m, l_max_m):
    """Set peak detection lateral width in meters (±7.35m heatmap FOV)."""
    ls = max(0, min(FH_MAX_LAT_BIN,
                    round(FH_LAT_CENTER_BIN + l_min_m / FH_LAT_M_PER_BIN)))
    le_ = max(0, min(FH_MAX_LAT_BIN,
                     round(FH_LAT_CENTER_BIN + l_max_m / FH_LAT_M_PER_BIN)))
    if ls >= le_:
        raise ValueError(f"lat_min ({l_min_m}) must be < lat_max ({l_max_m})")
    l_min_actual = (ls - FH_LAT_CENTER_BIN) * FH_LAT_M_PER_BIN
    l_max_actual = (le_ - FH_LAT_CENTER_BIN) * FH_LAT_M_PER_BIN
    print(f"[lateral] setting peak detect lateral bins [{ls}..{le_}] "
          f"({l_min_actual:+.2f}m .. {l_max_actual:+.2f}m)")
    poke_u16(target, port, password, pid, ADDR_FH_LAT_START, ls)
    poke_u16(target, port, password, pid, ADDR_FH_LAT_END,   le_)


# ── FPGA CFAR threshold control (affects 1E61 ground/wire/pcla) ─────────
# These thresholds live in the FPGA ctrl block @ 0x43C30070..0x7C. We
# access them via devmem because they are MMIO, not process memory.
FPGA_CFAR_THRESHOLDS = {
    "cfar_default": [0x999,  0x3000, 0xA000, 0x21BD],  # T40 default (case 2)
    "cfar_low":     [0x100,  0x500,  0x1000, 0x400],   # ~6x more sensitive
    "cfar_max":     [1,      1,      1,      1],       # accept everything
}


def _devmem_write(target, port, password, addr, value):
    cmd = f"devmem 0x{addr:08X} 32 0x{value:X}"
    rc, out, err = ssh_run_cmd(target, cmd, port=port, password=password, timeout=5)
    if rc != 0:
        raise RuntimeError(f"devmem failed (rc={rc}): "
                           f"{err.decode(errors='ignore').strip()}")


def _devmem_read(target, port, password, addr):
    cmd = f"devmem 0x{addr:08X} 32"
    rc, out, _ = ssh_run_cmd(target, cmd, port=port, password=password, timeout=5)
    if rc != 0:
        return None
    try:
        return int(out.decode().strip(), 16)
    except ValueError:
        return None


def set_fpga_cfar(target, port, password, preset):
    """Apply FPGA CFAR threshold preset via devmem."""
    vals = FPGA_CFAR_THRESHOLDS.get(preset)
    if vals is None:
        raise ValueError(f"unknown preset: {preset}; choices: {list(FPGA_CFAR_THRESHOLDS)}")
    print(f"[cfar] applying FPGA CFAR preset '{preset}': {vals}")
    for i, v in enumerate(vals):
        _devmem_write(target, port, password, 0x43C30070 + i * 4, v)
        print(f"  0x43C300{0x70+i*4:02X} = 0x{v:X}")


def show_fpga_cfar(target, port, password):
    """Read current FPGA CFAR thresholds."""
    print("=== FPGA CFAR thresholds (0x43C30070..0x7C) ===")
    for i in range(4):
        a = 0x43C30070 + i * 4
        v = _devmem_read(target, port, password, a)
        tag = "DEFAULT" if v == [0x999, 0x3000, 0xA000, 0x21BD][i] else ""
        print(f"  [{i}] 0x{a:08X} = 0x{v:X}  ({v})  {tag}")


def status(target: str, port: int, password: str, pid: int):
    fm  = _dd_read(target, port, password, pid, ADDR_FLIGHT_MODE, 1)
    dfa = _dd_read(target, port, password, pid, ADDR_DFA_STATE, 1)
    pm  = _dd_read(target, port, password, pid, ADDR_POWER_MODE, 1)
    src = _dd_read(target, port, password, pid, ADDR_POWER_SRC, 1)
    err = _dd_read(target, port, password, pid, ADDR_CFAR_ERROR, 1)
    cond = _dd_read(target, port, password, pid, ADDR_DFA_COND, 32)
    rot  = _dd_read(target, port, password, pid, ADDR_ROTATION, 8)
    cfar = _dd_read(target, port, password, pid, ADDR_CFAR_SECTOR, 8)
    omni = _dd_read(target, port, password, pid, ADDR_OMNI_SNAP, 4)
    ptc  = _dd_read(target, port, password, pid, ADDR_POINT_COUNT, 4)
    mcmd = _dd_read(target, port, password, pid, ADDR_MOTOR_CMD, 1)
    mfix = _dd_read(target, port, password, pid, ADDR_MOTOR_FIX, 1)
    mrpm = _dd_read(target, port, password, pid, ADDR_MOTOR_RPM, 2)

    def b(x, i=0, default='?'):
        return x[i] if x and len(x) > i else default

    print(f"  flight_mode={b(fm)} dfa_state={b(dfa)} "
          f"power_mode={b(pm)} byte_17A3F8={b(src)} cfar_error={b(err)}")
    # unk_282DEC: 0=uninit, 1=stop, 2=run-gate-open (written by hms_switch ACK handler)
    mrpm_val = struct.unpack_from('<H', mrpm, 0)[0] if mrpm and len(mrpm) >= 2 else '?'
    print(f"  motor_cmd(282DEC)={b(mcmd)} [0=uninit 1=stop 2=RUN-GATE-OPEN]  "
          f"fix_flag(282DA0)={b(mfix)}  rpm_set(282D74)={mrpm_val}")
    if cond and len(cond) >= 16:
        print(f"  cond: 810={cond[0]} 811={cond[1]} 818={cond[8]} "
              f"81c={cond[12]} 81d={cond[13]}")
    if rot and len(rot) >= 8:
        # Per CLAUDE.md: rot_angle occupies bytes 0-1, 2-3, 6-7 (three
        # sub-fields all in sync); sectors_decoded is at bytes 4-5.
        ra1 = struct.unpack_from('<H', rot, 0)[0]
        ra2 = struct.unpack_from('<H', rot, 2)[0]
        sd  = struct.unpack_from('<H', rot, 4)[0]
        ra3 = struct.unpack_from('<H', rot, 6)[0]
        print(f"  rot_angle=[{ra1},{ra2},{ra3}] sectors_decoded={sd}", end='')
    if cfar and len(cfar) >= 7:
        print(f"  cfar_valid={cfar[0]} cfar_n_det={cfar[6]} "
              f"cfar_angle={struct.unpack_from('<H', cfar, 3)[0]}")
    if omni and len(omni) >= 2:
        print(f"  omni_active={omni[0]} omni_n_pts={omni[1]}", end='')
    if ptc and len(ptc) >= 2:
        print(f"  point_count={struct.unpack_from('<H', ptc, 0)[0]}")


# ── CLI ─────────────────────────────────────────────────────────────────
def parse_int(s: str) -> int:
    return int(s, 0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('action',
        help=('activate|deactivate|pm0|pm2|pm3|area_zero|poke|status|monitor|'
              'sens_default|sens_medium|sens_high|sens_max|show_sens|'
              'set_range|set_lateral|'
              'cfar_default|cfar_low|cfar_max|show_cfar'))
    p.add_argument('args', nargs='*',
                   help=('poke: ADDR BYTE [BYTE...];  '
                         'set_range: MIN_M MAX_M;  '
                         'set_lateral: MIN_M MAX_M'))
    p.add_argument('--ssh',       default=DEFAULT_SSH_TARGET)
    p.add_argument('--ssh-port',  type=int, default=DEFAULT_SSH_PORT)
    p.add_argument('--password',  default=DEFAULT_SSH_PASSWORD)
    p.add_argument('--no-redeploy', action='store_true')
    p.add_argument('--pid', type=int, default=None)
    p.add_argument('--watch', type=float, default=0.0,
                   help='after action, poll status every N seconds (0 = once)')
    args = p.parse_args()

    target = args.ssh
    port = args.ssh_port
    pw = args.password

    pid = args.pid or get_pid(target, port, pw)
    print(f"app.elf PID: {pid}")

    action = args.action.lower()
    sens_actions = ('sens_default', 'sens_medium', 'sens_high', 'sens_max')
    range_actions = ('set_range', 'set_lateral')
    cfar_actions = ('cfar_default', 'cfar_low', 'cfar_max')
    ro_actions = ('show_sens', 'show_cfar', 'status', 'monitor')
    needs_deploy = (
        action in ('activate', 'deactivate', 'pm0', 'pm2', 'pm3', 'area_zero', 'poke')
        or action in sens_actions or action in range_actions
    )

    if needs_deploy:
        deploy(target, port, pw, redeploy=not args.no_redeploy)

    if action == 'activate':
        print(poke(target, port, pw, pid, ADDR_FLIGHT_MODE, [2]))
    elif action == 'deactivate':
        print(poke(target, port, pw, pid, ADDR_FLIGHT_MODE, [0]))
    elif action == 'pm0':
        print(poke(target, port, pw, pid, ADDR_POWER_SRC, [0]))
    elif action == 'pm2':
        print(poke(target, port, pw, pid, ADDR_POWER_SRC, [2]))
    elif action == 'pm3':
        print(poke(target, port, pw, pid, ADDR_POWER_SRC, [3]))
    elif action == 'area_zero':
        # Override the cached drone areacode to 0. This pushes sub_65F10
        # down the `unk_CD80290 == 0` branch which can reach pm=3 — the
        # areacode=156 (China) branch caps the radar at pm=2 (800M low PGA)
        # which in turn keeps the FFT/CFAR chain inactive on bench.
        zeros = [0] * 8
        # Set flag = 1 (ACK received) and both value + cache to 0
        print(poke(target, port, pw, pid, ADDR_AREACODE_FLAG, [1]))
        print(poke(target, port, pw, pid, ADDR_AREACODE_VALUE, zeros))
        print(poke(target, port, pw, pid, ADDR_AREACODE_CACHE, zeros))
    elif action == 'poke':
        if len(args.args) < 2:
            p.error("poke needs ADDR BYTE [BYTE...]")
        addr = parse_int(args.args[0])
        vals = [parse_int(x) for x in args.args[1:]]
        print(poke(target, port, pw, pid, addr, vals))
    elif action in sens_actions:
        # Fixed_height heatmap peak-threshold preset
        set_sens(target, port, pw, pid, action)
        show_sens(target, port, pw, pid)
    elif action == 'show_sens':
        show_sens(target, port, pw, pid)
    elif action == 'set_range':
        if len(args.args) != 2:
            p.error("set_range needs MIN_METERS MAX_METERS")
        r_min = float(args.args[0])
        r_max = float(args.args[1])
        set_range(target, port, pw, pid, r_min, r_max)
        show_sens(target, port, pw, pid)
    elif action == 'set_lateral':
        if len(args.args) != 2:
            p.error("set_lateral needs MIN_METERS MAX_METERS (e.g. -4 4)")
        l_min = float(args.args[0])
        l_max = float(args.args[1])
        set_lateral(target, port, pw, pid, l_min, l_max)
        show_sens(target, port, pw, pid)
    elif action in cfar_actions:
        set_fpga_cfar(target, port, pw, action)
        show_fpga_cfar(target, port, pw)
    elif action == 'show_cfar':
        show_fpga_cfar(target, port, pw)
    elif action in ('status', 'monitor'):
        pass
    else:
        p.error(f"unknown action: {action}")

    # Post-action status (skip for read-only sens/cfar queries — those
    # already printed their own detailed output).
    skip_status = action in ('show_sens', 'show_cfar')
    if args.watch > 0:
        try:
            while True:
                print(f"\n--- {time.strftime('%H:%M:%S')} ---")
                status(target, port, pw, pid)
                time.sleep(args.watch)
        except KeyboardInterrupt:
            print()
    elif not skip_status:
        print("\n--- status ---")
        status(target, port, pw, pid)


if __name__ == '__main__':
    main()
