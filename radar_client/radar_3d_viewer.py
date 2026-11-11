#!/usr/bin/env python3
"""3D raw point-cloud viewer for the DJI T40 RD2484R radar (SIDE array).

With no arguments this is a one-shot launcher: it checks the host link,
starts the FC stand-in client (UDP <-> CAN bridge so the radar spins up),
deploys the on-radar streamer, then shows and emits the RAW point cloud.

Raw point cloud = every CFAR detection from the 360 sectors, projected 1:1
into body coordinates by SideFrame.compute_body_xyz() (no persistence
filter, no SNR/range filtering unless explicitly requested):

    x = forward (m), y = right (m), z = up (m)

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

import radar_viewer as rv

HERE = Path(__file__).resolve().parent
SETUP = HERE.parent / 'setup'
DEFAULT_RADAR_IP = '192.168.8.2'
DEFAULT_HOST_IP = '192.168.8.1'


def build_parser():
    p = argparse.ArgumentParser(
        description='DJI T40 Radar — one-shot 3D RAW point cloud',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  radar_3d_viewer.py                              # full auto start\n"
            "  radar_3d_viewer.py --export /tmp/raw.csv        # also save points\n"
            "  radar_3d_viewer.py --dump                       # one frame, text\n"
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

    p.add_argument('--color-by', choices=('z', 'snr', 'sector'), default='z',
                   help='Point colour source (default z = height)')
    p.add_argument('--interval', type=int, default=80,
                   help='GUI refresh interval in ms (default 80)')
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


def preflight(args):
    ok_ip = host_has_ip(DEFAULT_HOST_IP)
    ok_can = iface_is_up(args.can_iface)
    if ok_ip and ok_can:
        print(f"[boot] link OK ({DEFAULT_HOST_IP} owned, {args.can_iface} up)")
        return
    print(f"[boot] link check: host IP={ok_ip}, {args.can_iface} up={ok_can}")
    if not args.auto_setup:
        print("[boot] fix with:  sudo ./setup/start_ap.sh && sudo ./setup/start_can.sh")
        print("[boot] or rerun with --auto-setup")
        raise SystemExit(2)
    if not ok_ip:
        print("[boot] running setup/start_ap.sh ...")
        subprocess.run(['sudo', str(SETUP / 'start_ap.sh')], check=True)
    if not ok_can:
        print("[boot] running setup/start_can.sh ...")
        subprocess.run(['sudo', str(SETUP / 'start_can.sh')], check=True)
    if not host_has_ip(DEFAULT_HOST_IP):
        raise RuntimeError(f"host still does not own {DEFAULT_HOST_IP}")
    print("[boot] link ready")


# ─── FC stand-in client ─────────────────────────────────────────────────
def start_fc_client(args):
    cmd = [sys.executable, str(HERE / 'radar_min_client.py'),
           '--can-bridge', args.can_iface,
           '--radar-ip', args.radar_ip]
    print(f"[boot] starting FC client: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, cwd=str(HERE), stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, bufsize=1)

    def pump(pipe):
        for line in iter(pipe.readline, b''):
            text = line.decode('utf-8', errors='replace').rstrip()
            if any(k in text.lower() for k in ('[err]', 'traceback', 'failed')):
                print(f"[fc] {text}", flush=True)

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


def decode_frame(raw, args, persistence):
    fr = rv.SideFrame(raw)
    pts = fr.compute_body_xyz(azimuth_offset_deg=args.azimuth_offset,
                              min_snr=args.min_snr,
                              min_range_m=args.min_range)
    if args.persist_min_hits > 0:
        pts, hits = persistence.update_and_filter(pts)
    else:
        hits = []
    return fr, pts, hits


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


def emit_frame(args, exporter, fr, pts, frame_no, fps):
    if not args.quiet:
        kind = 'raw' if args.persist_min_hits == 0 else 'filtered'
        print(f"[f{frame_no:05d}] token={fr.frame_token} "
              f"sectors={fr.sectors_decoded} "
              f"rot={fr.rot_angle_a}/{fr.rot_angle_b}/{fr.rot_angle_c} "
              f"{kind}_pts={len(pts)} cfar_error={fr.cfar_error} "
              f"fps={fps:.1f}", flush=True)
    if args.print_points:
        for p in pts[:args.print_points]:
            print(f"    sector={p['sector']:3d} x={p['x']:7.2f} "
                  f"y={p['y']:7.2f} z={p['z']:7.2f} "
                  f"r={p['r']:6.2f} snr={p['snr']:.1f}", flush=True)
    if exporter and frame_no % max(1, args.export_every) == 0:
        exporter.write(fr, pts)


def dump_once(reader, args, persistence, exporter):
    deadline = time.monotonic() + 8.0
    raw = None
    while time.monotonic() < deadline:
        raw = reader.try_read_frame()
        if raw is not None:
            break
        time.sleep(0.1)
    if raw is None:
        print("no SIDE frame within 8s — is the streamer running?")
        return
    fr, pts, _ = decode_frame(raw, args, persistence)
    print("=== RAW 3D point cloud (one SIDE frame) ===")
    emit_frame(args, exporter, fr, pts, 1, 0.0)
    if not pts:
        print("no points — radar may not be in working state yet "
              "(FC client + CAN bridge are required)")
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
        self.cmap, vmin, vmax = {
            'z': ('coolwarm', -args.z_range, args.z_range),
            'snr': ('viridis', 0.0, 255.0),
            'sector': ('hsv', 0.0, 360.0),
        }[args.color_by]
        self.norm = Normalize(vmin=vmin, vmax=vmax)

        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        ax = self.ax
        R, Z = args.range_m, args.z_range
        ax.set_xlabel('x forward (m)')
        ax.set_ylabel('y right (m)')
        ax.set_zlabel('z up (m)')
        ax.set_xlim(-R, R)
        ax.set_ylim(-R, R)
        ax.set_zlim(-Z, Z)
        ax.view_init(elev=28, azim=-60)
        raw_tag = 'RAW' if args.persist_min_hits == 0 else 'filtered'
        ax.set_title(f'DJI T40 — {raw_tag} 3D point cloud')
        try:
            ax.set_box_aspect((2 * R, 2 * R, 2 * Z))
        except AttributeError:
            pass

        t = np.linspace(0, 2 * np.pi, 120)
        for radius in (5, 10, 15, 20, 25, 30):
            if radius <= R:
                ax.plot(radius * np.cos(t), radius * np.sin(t),
                        np.zeros_like(t), color='#999999', lw=0.5, alpha=0.5)
        ax.plot([-R, R], [0, 0], [0, 0], color='#777777', lw=0.5, alpha=0.6)
        ax.plot([0, 0], [-R, R], [0, 0], color='#777777', lw=0.5, alpha=0.6)
        ax.scatter([0], [0], [0], marker='^', c='white', edgecolors='black',
                   s=90, depthshade=False, zorder=10)

        self.sc = ax.scatter([], [], [], s=[], c=[], cmap=self.cmap,
                             norm=self.norm, depthshade=False, alpha=0.85,
                             edgecolors='none')
        self.fig.colorbar(self.sc, ax=ax, shrink=0.55, pad=0.1,
                          label={'z': 'height z (m)', 'snr': 'SNR',
                                 'sector': 'sector id'}[args.color_by])
        self.stats = self.fig.text(0.02, 0.02, '', fontsize=9,
                                   family='monospace')

    def update(self, fr, pts, hits, frames, fps, error=None):
        args = self.args
        if pts:
            xs = np.array([p['x'] for p in pts], dtype=float)
            ys = np.array([p['y'] for p in pts], dtype=float)
            zs = np.array([p['z'] for p in pts], dtype=float)
            key = {'z': 'z', 'snr': 'snr', 'sector': 'sector'}[args.color_by]
            vals = np.array([p[key] for p in pts], dtype=float)
            snr = np.array([p['snr'] for p in pts], dtype=float)
            sizes = np.clip(snr / 5.0, 8, 60)
        else:
            xs = ys = zs = vals = sizes = np.array([])

        try:
            self.sc._offsets3d = (xs, ys, zs)
            self.sc.set_array(vals)
            self.sc.set_sizes(sizes)
        except Exception:
            self.sc.remove()
            self.sc = self.ax.scatter(xs, ys, zs, c=vals, s=sizes,
                                      cmap=self.cmap, norm=self.norm,
                                      depthshade=False, alpha=0.85,
                                      edgecolors='none')

        if fr is None:
            head = "waiting for SIDE frames (UDP 9877) ..."
            body = ""
        else:
            head = (f"token={fr.frame_token}  sectors={fr.sectors_decoded}  "
                    f"rot_angle={fr.rot_angle_a}/{fr.rot_angle_b}/"
                    f"{fr.rot_angle_c}  cfar_error={fr.cfar_error}")
            body = (f"raw_pts={len(pts):4d}  frames={frames}  fps={fps:4.1f}  "
                    f"filter={'off (RAW)' if args.persist_min_hits == 0 else 'on'}  "
                    f"color={args.color_by}")
            if fr.sectors_decoded == 0 and not pts:
                body += "\n(no points: radar not in working state yet)"
        if error:
            body += f"\n[err] {error}"
        self.stats.set_text(head + "\n" + body)
        return (self.sc, self.stats)


# ─── Main ───────────────────────────────────────────────────────────────
def run(args):
    auto = args.udp is None and not args.deploy
    args.ssh = args.ssh or f"root@{args.radar_ip}"
    need_fc = (args.start_fc or auto) and not args.no_fc

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
        reader = make_side_reader(args)
        print(f"[boot] {'raw' if args.persist_min_hits == 0 else 'filtered'} "
              f"point cloud starting", flush=True)

        if args.dump:
            dump_once(reader, args, persistence, exporter)
            return

        scene = PointCloud3D(args)
        state = {'fr': None, 'pts': [], 'hits': [], 'error': None,
                 'frames': 0, 'fps': 0.0, 't0': time.monotonic()}

        def tick(_):
            raw = reader.try_read_frame()
            if raw is not None:
                try:
                    fr, pts, hits = decode_frame(raw, args, persistence)
                    state.update(fr=fr, pts=pts, hits=hits, error=None)
                    state['frames'] += 1
                    elapsed = time.monotonic() - state['t0']
                    state['fps'] = state['frames'] / elapsed if elapsed > 0 else 0.0
                    emit_frame(args, exporter, fr, pts,
                               state['frames'], state['fps'])
                except Exception as exc:
                    state['error'] = str(exc)
                    print(f"[err] frame decode failed: {exc}", flush=True)
            return scene.update(state['fr'], state['pts'], state['hits'],
                                state['frames'], state['fps'],
                                error=state['error'])

        from matplotlib.animation import FuncAnimation
        FuncAnimation(scene.fig, tick, interval=args.interval,
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
