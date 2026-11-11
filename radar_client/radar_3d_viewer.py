#!/usr/bin/env python3
"""3D raw point-cloud viewer for the DJI T40 RD2484R radar (SIDE array).

With no arguments this is a one-shot launcher: it checks the host link,
starts the FC stand-in client (UDP <-> CAN bridge so the radar spins up),
deploys the on-radar streamer, then shows and emits the RAW point cloud.

Raw point cloud = every CFAR detection from the 360 sectors, projected 1:1
into body coordinates by SideFrame.compute_body_xyz() (no persistence
filter, no SNR/range filtering unless explicitly requested):

    x = forward (m), y = right (m), z = up (m)

Default look: bright green points on a black canvas. Use --color-by
z / snr / sector to switch to a colour scale instead.

Examples:
    python3 radar_3d_viewer.py
    python3 radar_3d_viewer.py --export /tmp/raw_points.csv --print-points 5
    python3 radar_3d_viewer.py --dump
    python3 radar_3d_viewer.py --udp 192.168.8.2       # attach only
"""
from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
from types import SimpleNamespace

import radar_viewer as rv
from radar_min_client import parse_v2_frame, decode_perception_1e61_entries

HERE = Path(__file__).resolve().parent
SETUP = HERE.parent / 'setup'
DEFAULT_RADAR_IP = '192.168.8.2'
DEFAULT_HOST_IP = '192.168.8.1'
GREEN = '#00ff66'


def build_parser():
    p = argparse.ArgumentParser(
        description='DJI T40 Radar — one-shot 3D RAW point cloud',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  radar_3d_viewer.py                              # full auto start\n"
            "  radar_3d_viewer.py --export /tmp/raw.csv        # also save points\n"
            "  radar_3d_viewer.py --dump                       # one frame, text\n"
            "  radar_3d_viewer.py --demo --point-size 4         # smaller dots\n"
            "  radar_3d_viewer.py --udp 192.168.8.2            # attach only\n"
        ))
    src = p.add_mutually_exclusive_group()
    src.add_argument('--udp', metavar='HOST',
                     help='Attach to a running SIDE streamer on HOST:9877 '
                          '(no auto start)')
    src.add_argument('--deploy', action='store_true',
                     help='Deploy the streamer but do not start the FC client')

    p.add_argument('--radar-ip', default=DEFAULT_RADAR_IP)
    p.add_argument('--ssh', metavar='USER@HOST', default=None)
    p.add_argument('--ssh-port', type=int, default=rv.DEFAULT_SSH_PORT)
    p.add_argument('--password', default=rv.DEFAULT_SSH_PASSWORD)
    p.add_argument('--pid', type=int, default=None,
                   help='PID of app.elf (auto-detected if omitted)')
    p.add_argument('--stream-interval', type=int, default=20000,
                   help='Remote streamer sleep in us (default 20000 = 50Hz)')
    p.add_argument('--can-iface', default='can0')
    p.add_argument('--auto-setup', action='store_true',
                   help='Run setup/start_ap.sh + start_can.sh via sudo if the '
                        'host link / CAN is not ready')
    p.add_argument('--wait-sec', type=float, default=60.0,
                   help='How long to wait for radar SSH after starting FC')
    p.add_argument('--start-fc', action='store_true',
                   help='Also start the FC client in --udp/--deploy mode')
    p.add_argument('--no-fc', action='store_true',
                   help='Do not start the FC client in auto mode')

    p.add_argument('--range', dest='range_m', type=float, default=25.0,
                   help='Horizontal half-extent of the view in m (default 25)')
    p.add_argument('--z-range', type=float, default=5.0,
                   help='Vertical half-extent of the view in m (default 5)')
    p.add_argument('--azimuth-offset', type=float,
                   default=rv.SIDE_DEFAULT_OFFSET)
    p.add_argument('--min-snr', type=int, default=0,
                   help='Drop detections with SNR below this (0 = raw)')
    p.add_argument('--min-range', type=float, default=0.0,
                   help='Drop detections closer than this in m (0 = raw)')

    p.add_argument('--persist-window', type=int, default=16)
    p.add_argument('--persist-min-hits', type=int, default=0,
                   help='CFAR persistence threshold; 0 = RAW points (default)')
    p.add_argument('--persist-tol-az', type=float, default=10.0)
    p.add_argument('--persist-tol-r', type=float, default=1.0)

    p.add_argument('--color-by', choices=('none', 'z', 'snr', 'sector'),
                   default='none',
                   help='Point colour: none = green points on black (default), '
                        'or map z / snr / sector with a colour scale')
    p.add_argument('--interval', type=int, default=80,
                   help='GUI refresh interval in ms (default 80)')
    p.add_argument('--point-size', type=float, default=4.0,
                   help='Marker area of the largest point (default 4). '
                        'Signal strength still modulates '
                        'size within 0.4x..1x of this value. In the window '
                        'use ] to enlarge and [ to shrink (also +/-)')
    p.add_argument('--export', metavar='PATH',
                   help='Emit RAW points to .csv / .jsonl / .ply '
                        '(PLY writes one file per frame)')
    p.add_argument('--export-every', type=int, default=1,
                   help='Write only every Nth frame (default 1)')
    p.add_argument('--print-points', type=int, default=0, metavar='N',
                   help='Print the first N raw points of every frame')
    p.add_argument('--quiet', action='store_true',
                   help='Suppress the per-frame status line')
    p.add_argument('--dump', action='store_true',
                   help='Print one frame of raw point-cloud stats and exit')
    p.add_argument('--demo', action='store_true',
                   help='Render a synthetic green point cloud without any '
                        'radar hardware (verifies the display path)')
    # ─── Point source ────────────────────────────────────────────────────
    # 'side' (default) reads the on-radar omni memory snapshot via the
    # streamer (needs the encoder->FPGA wire to be live). 'v2' instead
    # consumes the DJI 1E61 perception stream (ground/wire/pcla) which the
    # radar emits over UDP 40204 — this is the radar's real point cloud and
    # works even when the omni snapshot is empty (e.g. bench units whose
    # encoder isn't driven into the FPGA).
    src = p.add_argument_group('point source')
    src.add_argument('--source', choices=('side', 'v2'), default='v2',
                     help="Point source: 'v2' = DJI 1E61 UDP perception "
                          "stream (default — works even when the omni memory "
                          "snapshot is empty on bench units), 'side' = omni "
                          "memory snapshot")
    src.add_argument('--local-ip', default=DEFAULT_HOST_IP,
                     help='Local host IP to bind for the V2 listener '
                          '(default 192.168.8.1)')
    src.add_argument('--v2-local-port', type=int, default=40204,
                     metavar='PORT',
                     help='Local UDP port for the 1E61 stream (default 40204)')
    src.add_argument('--v2-ground-window', type=float, default=2.0,
                     metavar='SEC',
                     help='Ground/wire/pcla accumulation window for the V2 '
                          'source (default 2.0 s)')
    return p


# ─── Host link preflight ────────────────────────────────────────────────
def host_has_ip(ip: str) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind((ip, 0))
        return True
    except OSError:
        return False
    finally:
        s.close()


def iface_is_up(iface: str) -> bool:
    try:
        flags = int(Path(f'/sys/class/net/{iface}/flags').read_text().strip(), 16)
        return bool(flags & 0x1)
    except (OSError, ValueError):
        return False


def ensure_can_queue(iface: str, want: int = 1000):
    """Raise the SocketCAN tx queue when possible (default qlen is 10)."""
    try:
        cur = int(Path(f'/sys/class/net/{iface}/tx_queue_len').read_text().strip())
    except (OSError, ValueError):
        return
    if cur >= want:
        return
    res = subprocess.run(['sudo', '-n', 'ip', 'link', 'set', iface,
                          'txqueuelen', str(want)],
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    if res.returncode == 0:
        print(f"[boot] {iface} txqueuelen {cur} -> {want}")
    else:
        print(f"[boot] {iface} txqueuelen={cur} (optional: "
              f"sudo ip link set {iface} txqueuelen {want})")


def preflight(args):
    ok_ip = host_has_ip(DEFAULT_HOST_IP)
    ok_can = iface_is_up(args.can_iface)
    if ok_ip and ok_can:
        print(f"[boot] link OK ({DEFAULT_HOST_IP} owned, {args.can_iface} up)")
        ensure_can_queue(args.can_iface)
        return
    print(f"[boot] link check: host IP={ok_ip}, {args.can_iface} up={ok_can}")
    if not args.auto_setup:
        print("[boot] fix with:  sudo ./setup/start_ap.sh && sudo ./setup/start_can.sh")
        print("[boot] or rerun with --auto-setup")
        raise SystemExit(2)
    if not ok_ip:
        print("[boot] running setup/start_ap.sh ...")
        res = subprocess.run(['sudo', str(SETUP / 'start_ap.sh')])
        if res.returncode != 0:
            raise RuntimeError(
                "start_ap.sh failed — check that the AR1021x USB device is "
                "enumerated (lsusb | grep 0cf3) and the radar has 15V power")
    if not ok_can:
        print("[boot] running setup/start_can.sh ...")
        res = subprocess.run(['sudo', str(SETUP / 'start_can.sh')])
        if res.returncode != 0:
            raise RuntimeError("start_can.sh failed — check the PCAN-USB "
                               "adapter with 'ip link show can0'")
    if not host_has_ip(DEFAULT_HOST_IP):
        raise RuntimeError(f"host still does not own {DEFAULT_HOST_IP}")
    print("[boot] link ready")
    ensure_can_queue(args.can_iface)


# ─── FC stand-in client ─────────────────────────────────────────────────
def start_fc_client(args):
    cmd = [sys.executable, str(HERE / 'radar_min_client.py'),
           '--can-bridge', args.can_iface,
           '--radar-ip', args.radar_ip]
    # When the 3D viewer owns the 1E61 port (source='v2'), keep the FC
    # client from also binding 40204 so the viewer can listen exclusively.
    if getattr(args, 'source', 'side') == 'v2':
        cmd.append('--no-v2-listen')
    print(f"[boot] starting FC client: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, cwd=str(HERE), stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, bufsize=1)

    def pump(pipe):
        last_key = None
        suppressed = 0
        for line in iter(pipe.readline, ''):
            text = line.rstrip()
            low = text.lower()
            if any(k in low for k in ('[err]', 'traceback', 'failed')):
                key = low[:80]
                if key == last_key:
                    suppressed += 1
                    continue
                if suppressed:
                    print(f"[fc] ... ({suppressed} repeated lines suppressed)",
                          flush=True)
                    suppressed = 0
                print(f"[fc] {text}", flush=True)
                last_key = key
        if suppressed:
            print(f"[fc] ... ({suppressed} repeated lines suppressed)",
                  flush=True)

    threading.Thread(target=pump, args=(proc.stdout,), daemon=True).start()
    time.sleep(1.0)
    if proc.poll() is not None:
        raise RuntimeError(f"FC client exited immediately (code {proc.returncode})")
    return proc


def stop_fc_client(proc):
    if proc is None or proc.poll() is not None:
        return
    print("[stop] stopping FC client (sends motor_speed=0) ...", flush=True)
    proc.terminate()
    try:
        proc.wait(timeout=6.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2.0)


def wait_for_radar(args):
    """Wait until dropbear answers and app.elf is running."""
    deadline = time.monotonic() + args.wait_sec
    last_report = 0.0
    while time.monotonic() < deadline:
        try:
            s = socket.create_connection((args.radar_ip, args.ssh_port),
                                         timeout=1.0)
            s.close()
            rc, out, _ = rv.ssh_run_cmd(args.ssh, 'pidof app.elf',
                                        port=args.ssh_port,
                                        password=args.password, timeout=3.0)
            pid_text = out.decode(errors='replace').strip()
            if rc == 0 and pid_text:
                pid = int(pid_text.split()[0])
                print(f"[boot] radar ready, app.elf PID={pid}")
                return pid
        except (OSError, ValueError, TimeoutError):
            pass
        now = time.monotonic()
        if now - last_report >= 5.0:
            print("[boot] waiting for radar SSH / app.elf ...", flush=True)
            last_report = now
        time.sleep(1.0)
    raise RuntimeError(f"radar not ready within {args.wait_sec:.0f}s "
                       f"(check 15V power and the WiFi association)")


# ─── Streamer / UDP ─────────────────────────────────────────────────────
def make_side_reader(args):
    if args.deploy:
        host = args.ssh.split('@')[-1] if '@' in args.ssh else args.ssh
        pid = args.pid
        if pid is None:
            pid = rv.find_remote_pid(args.ssh, "app.elf",
                                     port=args.ssh_port,
                                     password=args.password)
            print(f"[boot] found app.elf PID: {pid}")
        if not rv.deploy_streamer(args.ssh, port=args.ssh_port,
                                  password=args.password):
            raise RuntimeError("streamer deploy failed")
        rv.launch_streamer_pair(args.ssh, pid, port=args.ssh_port,
                                password=args.password,
                                interval_us=args.stream_interval)
    else:
        host = args.udp.split(':')[0] if ':' in args.udp else args.udp
    return rv.UDPStreamReader(host, rv.SIDE_PORT, rv.SIDE_FRAME_SIZE)


class V2PointSource:
    """Listen on the DJI V2 perception UDP port (default 40204) and aggregate
    the 1E61 ground/wire/pcla point clouds into the same
    ``{x, y, z, r, sector, snr}`` dicts the SIDE path produces.

    The 1E61 payload already carries body-frame coordinates (x forward,
    y right, z up, metres) so no azimuth projection is needed. This is the
    radar's real perception output and is available even when the on-radar
    omni memory snapshot (the 'side' source) is empty — e.g. bench units
    where the encoder->FPGA wire isn't driven.
    """

    def __init__(self, local_ip=DEFAULT_HOST_IP, v2_port=40204,
                 ground_window=2.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
        self.sock.bind((local_ip, v2_port))
        self.sock.setblocking(False)
        self.ground_window = max(0.1, float(ground_window))
        # wire/pcla are sparse (subtype 1 ~0.17 Hz, subtype 2 ~1.1 Hz); keep
        # the latest frame for a longer window so they stay visible.
        self.slow_window = 10.0
        self.ground_frames = []     # list[(ts, [pts])]
        self.wire_latest = None     # (ts, [pts]) | None
        self.pcla_latest = None     # (ts, [pts]) | None
        self.total = 0
        self.lock = threading.Lock()
        print(f"[v2] listening for 1E61 perception on {local_ip}:{v2_port}")

    def _ingest(self, payload):
        try:
            subtype, entries = decode_perception_1e61_entries(payload)
        except Exception:
            return
        pts = []
        for e in entries:
            x, y, z = e['x_m'], e['y_m'], e['z_m']
            if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                continue
            r = float(np.sqrt(x * x + y * y + z * z))
            pts.append({'x': float(x), 'y': float(y), 'z': float(z),
                        'r': r, 'sector': int(e['id']),
                        'snr': float(e['metric'])})
        if not pts:
            return
        ts = time.monotonic()
        with self.lock:
            if subtype == 0:
                self.ground_frames.append((ts, pts))
            elif subtype == 1:
                self.wire_latest = (ts, pts)
            elif subtype == 2:
                self.pcla_latest = (ts, pts)
            else:
                return
            self.total += len(pts)

    def _pump(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                break
            f = parse_v2_frame(data)
            if f is None or f.msg_id != 0x1E61:
                continue
            self._ingest(f.payload)

    def get_points(self, args, persistence=None):
        self._pump()
        now = time.monotonic()
        with self.lock:
            self.ground_frames = [(t, p) for (t, p) in self.ground_frames
                                  if now - t <= self.ground_window]
            pts = []
            for _, p in self.ground_frames:
                pts.extend(p)
            if self.wire_latest and now - self.wire_latest[0] <= self.slow_window:
                pts.extend(self.wire_latest[1])
            if self.pcla_latest and now - self.pcla_latest[0] <= self.slow_window:
                pts.extend(self.pcla_latest[1])
            total = self.total
            n_ground = sum(len(p) for _, p in self.ground_frames)
        if args.min_range:
            pts = [p for p in pts if p['r'] >= args.min_range]
        if args.min_snr:
            pts = [p for p in pts if p['snr'] >= args.min_snr]
        fr = SimpleNamespace(
            frame_token=total, sectors_decoded=len(pts),
            rot_angle_a=0, rot_angle_b=0, rot_angle_c=0,
            cfar_error=0, point_count_total=len(pts))
        diag = {'sector_pts': len(pts), 'fallback': False, 'active': n_ground}
        return fr, pts, [], diag

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def make_v2_source(args):
    return V2PointSource(local_ip=args.local_ip, v2_port=args.v2_local_port,
                         ground_window=args.v2_ground_window)


def make_source(args):
    if args.source == 'v2':
        return make_v2_source(args)
    return make_side_reader(args)


def pull_frame(source, args, persistence):
    """Return (fr, pts, hits, diag) from either source, or None if the side
    reader has no frame yet."""
    if isinstance(source, rv.UDPStreamReader):
        raw = source.try_read_frame()
        if raw is None:
            return None
        return decode_frame(raw, args, persistence)
    return source.get_points(args, persistence)


def project_ignoring_active(fr, args):
    """Fallback projection when sectors carry points but their 'active'
    byte is 0. Some firmware builds leave that flag unset even though
    count/points are valid; compute_body_xyz() would silently skip all
    of them and the point cloud stays empty. Same math, no flag check.
    """
    offset = np.radians(args.azimuth_offset)
    out = []
    for sec in fr.sectors:
        if sec['count'] == 0:
            continue
        az = -np.deg2rad(sec['idx'] * rv.SIDE_DEG_PER_SECTOR) + offset
        cos_az, sin_az = np.cos(az), np.sin(az)
        for p in sec['points']:
            r = p['range_m']
            if not np.isfinite(r) or r < 0.1 or r > 120.0 or r < args.min_range:
                continue
            if p['snr'] < args.min_snr:
                continue
            el = p['el_rad']
            if not np.isfinite(el) or abs(el) > np.pi:
                el = 0.0
            cos_el, sin_el = np.cos(el), np.sin(el)
            out.append({'x': cos_el * r * cos_az,
                        'y': cos_el * r * sin_az,
                        'z': sin_el * r,
                        'r': r, 'sector': sec['idx'], 'snr': p['snr']})
    return out


def demo_frame(n):
    from types import SimpleNamespace
    return SimpleNamespace(frame_token=n, sectors_decoded=rv.SIDE_N_SECTORS,
                           rot_angle_a=n % rv.SIDE_N_SECTORS,
                           rot_angle_b=0, rot_angle_c=0,
                           cfar_error=0, point_count_total=0)


def demo_points(n, count=320):
    phase = n * 0.02
    pts = []
    for i in range(count):
        az = 2 * np.pi * i / count + phase
        r = 9.0 + 2.5 * np.sin(3 * az + phase)
        x, y = r * np.cos(az), r * np.sin(az)
        z = 1.2 * np.sin(5 * az + phase) + 0.15 * np.sin(i * 1.7)
        pts.append({'x': x, 'y': y, 'z': z, 'r': float(np.hypot(x, y)),
                    'sector': i % rv.SIDE_N_SECTORS,
                    'snr': 60.0 + 90.0 * abs(np.sin(2 * az))})
    return pts


def decode_frame(raw, args, persistence):
    fr = rv.SideFrame(raw)
    pts = fr.compute_body_xyz(azimuth_offset_deg=args.azimuth_offset,
                              min_snr=args.min_snr,
                              min_range_m=args.min_range)
    sector_pts = fr.all_valid_sector_points()
    fallback = False
    if not pts and sector_pts:
        pts = project_ignoring_active(fr, args)
        fallback = bool(pts)
    if args.persist_min_hits > 0:
        pts, hits = persistence.update_and_filter(pts)
    else:
        hits = []
    diag = {'sector_pts': len(sector_pts), 'fallback': fallback,
            'active': sum(1 for s in fr.sectors
                          if s['active'] and s['count'])}
    return fr, pts, hits, diag


# ─── Raw point output ───────────────────────────────────────────────────
class PointExporter:
    """Stream raw points to .csv / .jsonl, or one ASCII .ply per frame."""

    def __init__(self, path_str: str):
        self.path = Path(path_str)
        suffix = self.path.suffix.lower()
        if suffix not in ('.csv', '.jsonl', '.ply'):
            raise ValueError('--export must end in .csv, .jsonl or .ply')
        self.fmt = suffix[1:]
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.frame_no = 0
        self.fp = None
        if self.fmt in ('csv', 'jsonl'):
            self.fp = self.path.open('w', encoding='utf-8')
            if self.fmt == 'csv':
                self.fp.write('frame,token,sector,x,y,z,r,snr\n')
        print(f"[out] exporting raw points -> {self.path} ({self.fmt})")

    def write(self, fr, pts):
        self.frame_no += 1
        n = self.frame_no
        if self.fmt == 'csv':
            for p in pts:
                self.fp.write(
                    f"{n},{fr.frame_token},{p['sector']},"
                    f"{p['x']:.3f},{p['y']:.3f},{p['z']:.3f},"
                    f"{p['r']:.3f},{p['snr']:.1f}\n")
            self.fp.flush()
        elif self.fmt == 'jsonl':
            rec = {'frame': n, 'token': fr.frame_token,
                   'sectors_decoded': fr.sectors_decoded,
                   'rot_angle': [fr.rot_angle_a, fr.rot_angle_b,
                                 fr.rot_angle_c],
                   'cfar_error': fr.cfar_error,
                   'points': [{'x': round(p['x'], 3), 'y': round(p['y'], 3),
                               'z': round(p['z'], 3), 'r': round(p['r'], 3),
                               'sector': p['sector'], 'snr': p['snr']}
                              for p in pts]}
            self.fp.write(json.dumps(rec) + '\n')
            self.fp.flush()
        else:
            path = self.path.with_name(f"{self.path.stem}_{n:06d}.ply")
            with path.open('w', encoding='utf-8') as f:
                f.write('ply\nformat ascii 1.0\n'
                        f'element vertex {len(pts)}\n'
                        'property float x\nproperty float y\n'
                        'property float z\nproperty float range\n'
                        'property float snr\nproperty int sector\n'
                        'end_header\n')
                for p in pts:
                    f.write(f"{p['x']:.3f} {p['y']:.3f} {p['z']:.3f} "
                            f"{p['r']:.3f} {p['snr']:.1f} {p['sector']}\n")

    def close(self):
        if self.fp is not None:
            self.fp.close()


def emit_frame(args, exporter, fr, pts, frame_no, fps, diag=None):
    if not args.quiet:
        kind = 'raw' if args.persist_min_hits == 0 else 'filtered'
        extra = ''
        if diag:
            extra = (f" active_sec={diag['active']} "
                     f"sector_pts={diag['sector_pts']}")
            if diag['fallback']:
                extra += ' [fallback]'
        if pts:
            xs = np.array([p['x'] for p in pts])
            ys = np.array([p['y'] for p in pts])
            zs = np.array([p['z'] for p in pts])
            extra += (f" bbox=x[{xs.min():.0f},{xs.max():.0f}] "
                      f"y[{ys.min():.0f},{ys.max():.0f}] "
                      f"z[{zs.min():.0f},{zs.max():.0f}]")
        print(f"[f{frame_no:05d}] token={fr.frame_token} "
              f"sectors={fr.sectors_decoded} "
              f"rot={fr.rot_angle_a}/{fr.rot_angle_b}/{fr.rot_angle_c} "
              f"{kind}_pts={len(pts)} cfar_error={fr.cfar_error} "
              f"fps={fps:.1f}{extra}", flush=True)
    if args.print_points:
        for p in pts[:args.print_points]:
            print(f"    sector={p['sector']:3d} x={p['x']:7.2f} "
                  f"y={p['y']:7.2f} z={p['z']:7.2f} "
                  f"r={p['r']:6.2f} snr={p['snr']:.1f}", flush=True)
    if exporter and frame_no % max(1, args.export_every) == 0:
        exporter.write(fr, pts)


def dump_once(source, args, persistence, exporter):
    deadline = time.monotonic() + 8.0
    result = None
    while time.monotonic() < deadline:
        result = pull_frame(source, args, persistence)
        if result is None:
            time.sleep(0.1)
            continue
        fr, pts, _, diag = result
        # For the V2 source a result is always returned, but it may be empty
        # until the first 1E61 frame arrives — keep waiting for points.
        if isinstance(source, rv.UDPStreamReader) or pts:
            break
        time.sleep(0.1)
    if result is None:
        print("no frame within 8s — is the source producing data?")
        return
    fr, pts, _, diag = result
    print("=== RAW 3D point cloud (one frame) ===")
    emit_frame(args, exporter, fr, pts, 1, 0.0, diag=diag)
    if not pts:
        print(f"no points — active_sec={diag['active']} "
              f"sector_pts={diag['sector_pts']}")
        return
    for axis in ('x', 'y', 'z', 'r'):
        vals = np.array([p[axis] for p in pts])
        print(f"  {axis}: {vals.min():7.2f} .. {vals.max():7.2f} m")


# ─── 3D scene ───────────────────────────────────────────────────────────
class PointCloud3D:
    """Matplotlib 3D scatter, kept alive across animation frames."""

    def __init__(self, args, backend=None):
        import matplotlib
        matplotlib.use(backend or rv._select_gui_backend())
        import matplotlib.pyplot as plt
        from matplotlib.colors import Normalize
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        self.plt = plt
        self.args = args
        self.use_cmap = args.color_by != 'none'
        if self.use_cmap:
            self.cmap, vmin, vmax = {
                'z': ('coolwarm', -args.z_range, args.z_range),
                'snr': ('viridis', 0.0, 255.0),
                'sector': ('hsv', 0.0, 360.0),
            }[args.color_by]
            self.norm = Normalize(vmin=vmin, vmax=vmax)
            colour_kwargs = dict(c=[], cmap=self.cmap, norm=self.norm)
        else:
            self.cmap = self.norm = None
            colour_kwargs = dict(c=GREEN)

        self.fig = plt.figure(figsize=(10, 8))
        self.fig.patch.set_facecolor('black')
        self.ax = self.fig.add_subplot(111, projection='3d')
        ax = self.ax
        ax.set_facecolor('black')
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            try:
                axis.set_pane_color((0.0, 0.0, 0.0, 1.0))
            except Exception:
                pass
            try:
                axis.pane.set_facecolor((0.0, 0.0, 0.0, 1.0))
                axis.pane.set_edgecolor('#24442f')
            except Exception:
                pass
        try:
            ax.grid(color='#1d3a27', linewidth=0.4)
        except Exception:
            pass
        ax.tick_params(colors='#9fb8a6', labelsize=8)

        R, Z = args.range_m, args.z_range
        ax.set_xlabel('x forward (m)', color='#c8d8ce')
        ax.set_ylabel('y right (m)', color='#c8d8ce')
        ax.set_zlabel('z up (m)', color='#c8d8ce')
        ax.set_xlim(-R, R)
        ax.set_ylim(-R, R)
        ax.set_zlim(-Z, Z)
        ax.view_init(elev=28, azim=-60)
        raw_tag = 'RAW' if args.persist_min_hits == 0 else 'filtered'
        ax.set_title(f'DJI T40 — {raw_tag} 3D point cloud',
                     color=GREEN if not self.use_cmap else '#e0e0e0')
        try:
            ax.set_box_aspect((2 * R, 2 * R, 2 * Z))
        except AttributeError:
            pass

        t = np.linspace(0, 2 * np.pi, 120)
        for radius in (5, 10, 15, 20, 25, 30):
            if radius <= R:
                ax.plot(radius * np.cos(t), radius * np.sin(t),
                        np.zeros_like(t), color='#1f4d2e', lw=0.6, alpha=0.8)
        ax.plot([-R, R], [0, 0], [0, 0], color='#2d6b40', lw=0.6, alpha=0.8)
        ax.plot([0, 0], [-R, R], [0, 0], color='#2d6b40', lw=0.6, alpha=0.8)
        ax.scatter([0], [0], [0], marker='^', c='white', edgecolors=GREEN,
                   s=90, depthshade=False, zorder=10)

        self.sc = ax.scatter([], [], [], s=[], depthshade=False, alpha=0.9,
                             edgecolors='none', **colour_kwargs)
        if self.use_cmap:
            cb = self.fig.colorbar(self.sc, ax=ax, shrink=0.55, pad=0.1,
                                   label={'z': 'height z (m)', 'snr': 'SNR',
                                          'sector': 'sector id'}[args.color_by])
            cb.ax.yaxis.label.set_color('#c8d8ce')
            cb.ax.tick_params(colors='#c8d8ce')
            try:
                cb.outline.set_edgecolor('#3a3a3a')
            except Exception:
                pass
        self.stats = self.fig.text(0.02, 0.02, '', fontsize=9,
                                   family='monospace', color='#8effb0')
        self.warning = self.ax.text2D(0.5, 0.5, '', transform=self.ax.transAxes,
                                      ha='center', va='center', fontsize=14,
                                      color=GREEN, alpha=0.9)
        self.point_size = max(0.5, float(args.point_size))
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

    def _on_key(self, event):
        key = event.key or ''
        if key in (']', '+', '='):
            self.point_size *= 1.25
        elif key in ('[', '-'):
            self.point_size /= 1.25
        else:
            return
        self.point_size = float(np.clip(self.point_size, 0.5, 200.0))
        print(f"[view] point size = {self.point_size:.1f} "
              f"(] larger, [ smaller)", flush=True)

    def update(self, fr, pts, hits, frames, fps, error=None, diag=None):
        args = self.args
        if pts:
            xs = np.array([p['x'] for p in pts], dtype=float)
            ys = np.array([p['y'] for p in pts], dtype=float)
            zs = np.array([p['z'] for p in pts], dtype=float)
            if self.use_cmap:
                vals = np.array([p[args.color_by] for p in pts], dtype=float)
            else:
                vals = None
            snr = np.array([p['snr'] for p in pts], dtype=float)
            sizes = self.point_size * (0.4 + 0.6 * np.clip(snr / 255.0,
                                                           0.0, 1.0))
        else:
            xs = ys = zs = sizes = np.array([])
            vals = np.array([]) if self.use_cmap else None

        if not self.use_cmap:
            # Rebuild the collection every frame. Updating the private
            # _offsets3d attribute does not reliably refresh the 3D
            # projection in a live QtAgg animation.
            try:
                self.sc.remove()
            except Exception:
                pass
            self.sc = self.ax.scatter(xs, ys, zs, c=GREEN, s=sizes,
                                      depthshade=False, alpha=0.9,
                                      edgecolors='none')
        else:
            try:
                self.sc._offsets3d = (xs, ys, zs)
                self.sc.set_sizes(sizes)
                self.sc.set_array(vals)
            except Exception:
                self.sc.remove()
                self.sc = self.ax.scatter(xs, ys, zs, c=vals, s=sizes,
                                          cmap=self.cmap, norm=self.norm,
                                          depthshade=False, alpha=0.9,
                                          edgecolors='none')

        if fr is None:
            head = "waiting for SIDE frames (UDP 9877) ..."
            body = ""
            self.warning.set_text('WAITING FOR SIDE FRAMES (UDP 9877)')
        else:
            head = (f"token={fr.frame_token}  sectors={fr.sectors_decoded}  "
                    f"rot_angle={fr.rot_angle_a}/{fr.rot_angle_b}/"
                    f"{fr.rot_angle_c}  cfar_error={fr.cfar_error}")
            colour_tag = 'GREEN' if not self.use_cmap else args.color_by
            body = (f"raw_pts={len(pts):4d}  frames={frames}  fps={fps:4.1f}  "
                    f"filter={'off (RAW)' if args.persist_min_hits == 0 else 'on'}  "
                    f"color={colour_tag}  size={self.point_size:.1f} ([/])")
            if diag:
                body += (f"  active_sec={diag['active']} "
                         f"sector_pts={diag['sector_pts']}"
                         + ('  [fallback]' if diag['fallback'] else ''))
            if not pts:
                if diag and diag['sector_pts'] > 0:
                    hint = 'sector points present, projection produced none'
                else:
                    hint = ('no sector data: check FC client / CAN bridge / '
                            '900 rpm / streamer')
                self.warning.set_text('NO POINTS\n' + hint)
            else:
                R, Z = args.range_m, args.z_range
                inside = int(((np.abs(xs) <= R) & (np.abs(ys) <= R) &
                              (np.abs(zs) <= Z)).sum())
                if inside == 0:
                    self.warning.set_text(
                        f'ALL {len(pts)} POINTS OUTSIDE VIEW\n'
                        f'raise --range (now {R}m) / --z-range (now {Z}m)')
                else:
                    self.warning.set_text('')
        if error:
            body += f"\n[err] {error}"
            self.warning.set_text('ERROR\n' + str(error))
        self.stats.set_text(head + "\n" + body)
        return (self.sc, self.stats, self.warning)


# ─── Main ───────────────────────────────────────────────────────────────
def run_demo(args):
    """Synthetic point cloud: verifies the whole display/output path."""
    print("[demo] synthetic green point cloud — no radar hardware needed")
    exporter = PointExporter(args.export) if args.export else None
    try:
        if args.dump:
            fr, pts = demo_frame(1), demo_points(1)
            emit_frame(args, exporter, fr, pts, 1, 0.0)
            print(f"demo points: {len(pts)}")
            return
        scene = PointCloud3D(args)
        t0 = time.monotonic()
        counter = [0]

        def tick(_):
            counter[0] += 1
            n = counter[0]
            fr, pts = demo_frame(n), demo_points(n)
            fps = n / max(time.monotonic() - t0, 1e-6)
            emit_frame(args, exporter, fr, pts, n, fps)
            return scene.update(fr, pts, [], n, fps)

        from matplotlib.animation import FuncAnimation
        scene.ani = FuncAnimation(scene.fig, tick, interval=args.interval,
                                  blit=False, cache_frame_data=False)
        scene.plt.show()
    finally:
        if exporter:
            exporter.close()


def run(args):
    if args.demo:
        return run_demo(args)
    auto = args.udp is None and not args.deploy
    args.ssh = args.ssh or f"root@{args.radar_ip}"
    # The V2 source needs the radar driven into its active state, so it also
    # requires the FC client (unless the user supplies one via --no-fc).
    need_fc = (args.start_fc or auto or args.source == 'v2') and not args.no_fc

    if auto:
        print("[boot] auto mode: preflight -> FC client -> streamer -> 3D view")
        preflight(args)

    fc_proc = start_fc_client(args) if need_fc else None
    reader = None
    exporter = PointExporter(args.export) if args.export else None
    try:
        if auto:
            args.pid = wait_for_radar(args)
            args.deploy = True
        persistence = rv.PersistenceFilter(
            window=args.persist_window,
            min_hits=args.persist_min_hits or 1,
            tol_az_deg=args.persist_tol_az,
            tol_r_m=args.persist_tol_r,
        )
        source = make_source(args)
        reader = source  # kept for cleanup in finally
        mode = 'v2 (1E61)' if args.source == 'v2' else (
            'raw' if args.persist_min_hits == 0 else 'filtered')
        print(f"[boot] {mode} point cloud starting (source={args.source})",
              flush=True)

        if args.dump:
            dump_once(source, args, persistence, exporter)
            return

        scene = PointCloud3D(args)
        state = {'fr': None, 'pts': [], 'hits': [], 'diag': None,
                 'error': None, 'frames': 0, 'fps': 0.0,
                 't0': time.monotonic()}

        def tick(_):
            result = pull_frame(source, args, persistence)
            if result is not None:
                try:
                    fr, pts, hits, diag = result
                    state.update(fr=fr, pts=pts, hits=hits, diag=diag,
                                 error=None)
                    state['frames'] += 1
                    elapsed = time.monotonic() - state['t0']
                    state['fps'] = state['frames'] / elapsed if elapsed > 0 else 0.0
                    emit_frame(args, exporter, fr, pts,
                               state['frames'], state['fps'], diag=diag)
                except Exception as exc:
                    state['error'] = str(exc)
                    print(f"[err] frame decode failed: {exc}", flush=True)
            return scene.update(state['fr'], state['pts'], state['hits'],
                                state['frames'], state['fps'],
                                error=state['error'], diag=state['diag'])

        from matplotlib.animation import FuncAnimation
        # Keep a reference: FuncAnimation is otherwise garbage-collected
        # before the first draw and the window closes immediately.
        scene.ani = FuncAnimation(scene.fig, tick, interval=args.interval,
                                  blit=False, cache_frame_data=False)
        scene.plt.show()
    finally:
        if exporter:
            exporter.close()
        if reader:
            reader.close()
        stop_fc_client(fc_proc)


def main():
    run(build_parser().parse_args())


if __name__ == '__main__':
    main()
