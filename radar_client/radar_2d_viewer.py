#!/usr/bin/env python3
"""2D radar (PPI) view for the DJI T40 side rotating array.

Top-down plan-position-indicator: azimuth runs around the circle, range
along the radius. Plots every raw CFAR sector point from the 360-slot
omni snapshot (the classic rotating-radar scope), coloured by SNR.

    azimuth = slot_idx x 1deg + offset   (default offset 180deg -> forward
              at the top of the scope)
    radius  = range in metres

With no arguments this is a one-shot launcher: preflight -> FC stand-in
client (drives the motor) -> deploy the side streamer -> 2D PPI view.

Controls (matplotlib window):
    mouse wheel ......... zoom in / out (range limit)
    i / o ............... zoom in / out (keyboard fallback)
    ] / + ............... larger blips
    [ / - ............... smaller blips

Examples:
    python3 radar_2d_viewer.py --demo
    python3 radar_2d_viewer.py --udp 192.168.8.2    # attach only
    python3 radar_2d_viewer.py --dump               # one frame, text
    python3 radar_2d_viewer.py                      # full auto start
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np

import radar_viewer as rv
from radar_3d_viewer import (preflight, start_fc_client, stop_fc_client,
                             wait_for_radar)

DEFAULT_RADAR_IP = '192.168.8.2'
GREEN = '#00ff66'


def build_parser():
    p = argparse.ArgumentParser(
        description='DJI T40 Radar — 2D PPI (azimuth x range) side view',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  radar_2d_viewer.py --demo                        # synthetic\n"
            "  radar_2d_viewer.py --udp 192.168.8.2             # attach only\n"
            "  radar_2d_viewer.py --dump                        # one frame\n"
            "  radar_2d_viewer.py                               # full auto\n"
        ))
    src = p.add_mutually_exclusive_group()
    src.add_argument('--udp', metavar='HOST',
                     help='Attach to a running SIDE streamer on HOST:9877 '
                          '(no auto start)')
    src.add_argument('--deploy', action='store_true',
                     help='Deploy the side streamer but do not start the FC '
                          'client')

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
                   help='Range limit of the scope in m (default 25)')
    p.add_argument('--azimuth-offset', type=float,
                   default=rv.SIDE_DEFAULT_OFFSET)
    p.add_argument('--min-snr', type=int, default=0,
                   help='Drop detections with SNR below this (0 = raw)')
    p.add_argument('--min-range', type=float, default=0.0,
                   help='Drop detections closer than this in m (0 = raw)')
    p.add_argument('--interval', type=int, default=80,
                   help='GUI refresh interval in ms (default 80)')
    p.add_argument('--point-size', type=float, default=10.0,
                   help='Blip marker area (default 10). In the window use ] '
                        'to enlarge and [ to shrink (also +/-)')
    p.add_argument('--print-points', type=int, default=0, metavar='N',
                   help='Print the first N raw points of every frame')
    p.add_argument('--quiet', action='store_true',
                   help='Suppress the per-frame status line')
    p.add_argument('--dump', action='store_true',
                   help='Print one frame of raw sector stats and exit')
    p.add_argument('--demo', action='store_true',
                   help='Render a synthetic PPI without any radar hardware')
    return p


# ─── Synthetic demo data ────────────────────────────────────────────────
def demo_frame(n):
    return SimpleNamespace(frame_token=n, sectors_decoded=rv.SIDE_N_SECTORS,
                           rot_angle_a=n % rv.SIDE_N_SECTORS,
                           rot_angle_b=0, rot_angle_c=0,
                           cfar_error=0, point_count_total=0)


def demo_points(n, count=320):
    """Synthetic raw sector points (polar: slot idx -> azimuth)."""
    phase = n * 0.02
    pts = []
    for i in range(count):
        az_deg = i * 360.0 / count
        r = 9.0 + 2.5 * np.sin(3 * np.deg2rad(az_deg) + phase)
        snr = 60.0 + 90.0 * abs(np.sin(2 * np.deg2rad(az_deg)))
        pts.append({'sector': int(az_deg), 'range_m': float(r),
                    'snr': float(snr), 'el_rad': 0.0, 'vel_idx': 0})
    return pts


def emit_frame(args, fr, pts, frame_no, fps):
    if args.quiet:
        return
    print(f"[f{frame_no:05d}] token={fr.frame_token} "
          f"sectors={fr.sectors_decoded} "
          f"rot={fr.rot_angle_a}/{fr.rot_angle_b}/{fr.rot_angle_c} "
          f"pts={len(pts)} cfar_error={fr.cfar_error} "
          f"fps={fps:.1f}", flush=True)
    if args.print_points:
        for p in pts[:args.print_points]:
            print(f"    sector={p['sector']:3d} r={p['range_m']:6.2f} "
                  f"snr={p['snr']:.1f}", flush=True)


# ─── 2D PPI scene ───────────────────────────────────────────────────────
class RadarPPI:
    """Matplotlib polar scatter (azimuth x range), kept alive across frames."""

    def __init__(self, args, backend=None):
        import matplotlib
        matplotlib.use(backend or rv._select_gui_backend())
        import matplotlib.pyplot as plt
        from matplotlib.colors import Normalize

        self.plt = plt
        self.args = args
        self.fig = plt.figure(figsize=(9, 8))
        self.fig.patch.set_facecolor('black')
        self.ax = self.fig.add_subplot(111, projection='polar')
        ax = self.ax
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)          # clockwise sweep, forward at top
        ax.set_rlim(0, args.range_m * 1.05)
        ax.set_rlabel_position(135)
        ax.set_facecolor('#0b0e12')
        for sp in ax.spines.values():
            sp.set_color('#2a2f3a')
        ax.grid(color='#3a3a3a', alpha=0.4, linestyle=':')
        ax.tick_params(colors='#b8c0cc', labelsize=8)

        # Range rings: keep a readable set inside the current limit.
        self._set_rticks(args.range_m)
        # Forward marker + label.
        ax.plot([0], [0], marker='^', color='white', markersize=9,
                markeredgecolor='black', zorder=7)
        ax.text(0, args.range_m * 0.92, 'fwd', color='#ffcc66', fontsize=8,
                ha='center', va='bottom')

        ax.set_title('DJI T40 — 2D radar (azimuth × range)',
                     color=GREEN, pad=16)
        self.snr_norm = Normalize(vmin=0.0, vmax=255.0)
        self.sc = ax.scatter([], [], c=[], cmap='viridis',
                             norm=self.snr_norm, s=[], alpha=0.6,
                             edgecolors='none', zorder=4)
        cb = self.fig.colorbar(self.sc, ax=ax, shrink=0.7, pad=0.08,
                               label='SNR')
        cb.ax.yaxis.label.set_color('#c8d8ce')
        cb.ax.tick_params(colors='#c8d8ce')
        self.stats = self.fig.text(0.02, 0.02, '', fontsize=9,
                                   family='monospace', color='#8effb0')
        self.warning = ax.text(0, args.range_m * 0.5, '',
                               ha='center', va='center', fontsize=13,
                               color=GREEN, alpha=0.9)
        self.zoom = 1.0
        self.point_size = max(1.0, float(args.point_size))
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self.fig.canvas.mpl_connect('scroll_event', self._on_scroll)

    def _set_rticks(self, r):
        ticks = [t for t in (2, 5, 10, 15, 20, 25, 30, 40, 50) if t <= r]
        self.ax.set_rticks(ticks)

    def _apply_zoom(self):
        r = self.args.range_m / self.zoom
        self.ax.set_rlim(0, r * 1.05)
        self._set_rticks(r)
        try:
            self.fig.canvas.draw_idle()
        except Exception:
            pass

    def _zoom(self, factor):
        self.zoom = float(np.clip(self.zoom * factor, 0.2, 4.0))
        self._apply_zoom()

    def _on_scroll(self, event):
        if getattr(event, 'inaxes', None) is not self.ax:
            return
        btn = event.button
        zoom_in = btn == 'up' if isinstance(btn, str) else (btn is not None
                                                            and btn > 0)
        self._zoom(1.1 if zoom_in else 1.0 / 1.1)

    def _on_key(self, event):
        key = event.key or ''
        if key in (']', '+', '='):
            self.point_size *= 1.25
        elif key in ('[', '-'):
            self.point_size /= 1.25
        elif key in ('i', 'I'):
            self._zoom(1.1)
        elif key in ('o', 'O'):
            self._zoom(1.0 / 1.1)
        else:
            return
        if key in (']', '+', '=', '[', '-'):
            self.point_size = float(np.clip(self.point_size, 1.0, 400.0))
            print(f"[view] point size = {self.point_size:.1f} "
                  f"(] larger, [ smaller)", flush=True)
        else:
            print(f"[view] range = {self.args.range_m / self.zoom:.1f}m "
                  f"(i in, o out, wheel to zoom)", flush=True)

    def update(self, fr, pts, frames, fps, error=None):
        args = self.args
        if pts:
            az_deg = np.array([(p['sector'] * rv.SIDE_DEG_PER_SECTOR
                                + args.azimuth_offset) % 360.0
                               for p in pts], dtype=float)
            r = np.array([p['range_m'] for p in pts], dtype=float)
            snr = np.array([p['snr'] for p in pts], dtype=float)
            sizes = np.full_like(snr, self.point_size)
        else:
            az_deg = r = snr = sizes = np.array([])

        try:
            self.sc.set_offsets(np.column_stack([np.deg2rad(az_deg), r]))
            self.sc.set_array(snr)
            self.sc.set_sizes(sizes)
        except Exception:
            self.sc.remove()
            self.sc = self.ax.scatter(np.deg2rad(az_deg), r, c=snr,
                                      cmap='viridis', norm=self.snr_norm,
                                      s=sizes, alpha=0.6, edgecolors='none',
                                      zorder=4)

        if fr is None:
            head = "waiting for SIDE frames (UDP 9877) ..."
            body = ""
            self.warning.set_text('WAITING FOR SIDE FRAMES (UDP 9877)')
        else:
            head = (f"token={fr.frame_token}  sectors={fr.sectors_decoded}  "
                    f"rot_angle={fr.rot_angle_a}/{fr.rot_angle_b}/"
                    f"{fr.rot_angle_c}  cfar_error={fr.cfar_error}")
            body = (f"blips={len(pts):4d}  frames={frames:<6d}  "
                    f"fps={fps:5.1f}Hz  range={args.range_m / self.zoom:.1f}m  "
                    f"size={self.point_size:.1f} ([/])")
            if not pts:
                self.warning.set_text(
                    'NO POINTS\ncheck FC client / CAN bridge / 900 rpm / '
                    'streamer')
            else:
                self.warning.set_text('')
        if error:
            body += f"\n[err] {error}"
            self.warning.set_text('ERROR\n' + str(error))
        self.stats.set_text(head + "\n" + body)
        return (self.sc, self.stats, self.warning)


# ─── Launch helpers ─────────────────────────────────────────────────────
def make_side_reader(args):
    """Deploy + launch only the side streamer, or attach to an existing one."""
    if args.deploy:
        host = args.ssh.split('@')[-1] if '@' in args.ssh else args.ssh
        pid = args.pid
        if pid is None:
            pid = rv.find_remote_pid(args.ssh, "app.elf", port=args.ssh_port,
                                     password=args.password)
            print(f"[boot] found app.elf PID: {pid}")
        if not rv.deploy_streamer(args.ssh, port=args.ssh_port,
                                  password=args.password):
            raise RuntimeError("streamer deploy failed")
        rv.launch_streamer(args.ssh, 'side', pid, port=args.ssh_port,
                           password=args.password,
                           interval_us=args.stream_interval, kill_first=True)
    else:
        host = args.udp.split(':')[0] if ':' in args.udp else args.udp
    return rv.UDPStreamReader(host, rv.SIDE_PORT, rv.SIDE_FRAME_SIZE)


def _kill_matching(pattern, exclude_pids=()):
    try:
        out = subprocess.run(['pgrep', '-f', pattern],
                             capture_output=True, text=True).stdout.split()
    except (OSError, subprocess.SubprocessError):
        return []
    killed = []
    for tok in out:
        try:
            pid = int(tok)
        except ValueError:
            continue
        if pid == os.getpid() or pid in exclude_pids:
            continue
        try:
            os.kill(pid, signal.SIGTERM)
            killed.append(pid)
        except (ProcessLookupError, PermissionError):
            pass
    return killed


def kill_stale_instances(args):
    """Clear previous 2D/3D viewers (both hold UDP 9877) and, when we manage
    the FC client, any stray radar_min_client."""
    my_pid = os.getpid()
    killed = _kill_matching('radar_2d_viewer.py', exclude_pids=(my_pid,))
    killed += _kill_matching('radar_3d_viewer.py', exclude_pids=(my_pid,))
    killed_fcs = []
    if not getattr(args, 'no_fc', False):
        killed_fcs = _kill_matching('radar_min_client.py')
    if killed or killed_fcs:
        print(f"[boot] cleared previous runs: viewers={killed} "
              f"fc_clients={killed_fcs}", flush=True)
        time.sleep(1.0)


def decode_sector_points(fr, args):
    """Raw sector points, filtered for range and SNR."""
    pts = fr.all_valid_sector_points()
    return [p for p in pts
            if p['snr'] >= args.min_snr and p['range_m'] >= args.min_range]


# ─── Main ───────────────────────────────────────────────────────────────
def run_demo(args):
    print("[demo] synthetic 2D PPI — no radar hardware needed")
    scene = RadarPPI(args)
    meter = rv.FrameRateMeter()
    counter = [0]

    def tick(_):
        counter[0] += 1
        n = counter[0]
        fr, pts = demo_frame(n), demo_points(n)
        fps = meter.mark()
        emit_frame(args, fr, pts, n, fps)
        return scene.update(fr, pts, n, fps)

    from matplotlib.animation import FuncAnimation
    scene.ani = FuncAnimation(scene.fig, tick, interval=args.interval,
                              blit=False, cache_frame_data=False)
    scene.plt.show()


def dump_once(reader, args):
    deadline = time.monotonic() + 8.0
    raw = None
    while time.monotonic() < deadline:
        raw = reader.try_read_frame()
        if raw is not None:
            break
        time.sleep(0.1)
    if raw is None:
        print("no side frame within 8s — is the streamer producing data?")
        return
    fr = rv.SideFrame(raw)
    pts = decode_sector_points(fr, args)
    print("=== SIDE 2D radar (one frame) ===")
    emit_frame(args, fr, pts, 1, 0.0)
    if not pts:
        print(f"no points — sectors={fr.sectors_decoded}")
        return
    rs = np.array([p['range_m'] for p in pts])
    snrs = np.array([p['snr'] for p in pts])
    print(f"  range: {rs.min():7.2f} .. {rs.max():7.2f} m   "
          f"snr: {snrs.min():.0f} .. {snrs.max():.0f}")


def run(args):
    if args.demo:
        return run_demo(args)
    kill_stale_instances(args)
    auto = args.udp is None and not args.deploy
    args.ssh = args.ssh or f"root@{args.radar_ip}"
    need_fc = (args.start_fc or auto) and not args.no_fc

    if auto:
        print("[boot] auto mode: preflight -> FC client -> streamer -> PPI")
        preflight(args)

    fc_proc = start_fc_client(args) if need_fc else None
    reader = None
    try:
        if auto:
            args.pid = wait_for_radar(args)
            args.deploy = True
        reader = make_side_reader(args)
        print("[boot] 2D PPI starting (source=side omni sectors)", flush=True)

        if args.dump:
            dump_once(reader, args)
            return

        scene = RadarPPI(args)
        state = {'fr': None, 'pts': [], 'error': None, 'frames': 0,
                 'meter': rv.FrameRateMeter()}

        def tick(_):
            raw = reader.try_read_frame()
            if raw is not None:
                try:
                    fr = rv.SideFrame(raw)
                    pts = decode_sector_points(fr, args)
                    state.update(fr=fr, pts=pts, error=None)
                    state['frames'] += 1
                    fps = state['meter'].mark()
                    emit_frame(args, fr, pts, state['frames'], fps)
                except Exception as exc:
                    state['error'] = str(exc)
                    print(f"[err] frame decode failed: {exc}", flush=True)
            # Recomputed every tick so a stalled stream decays to 0 Hz.
            fps = state['meter'].rate()
            return scene.update(state['fr'], state['pts'], state['frames'],
                                fps, error=state['error'])

        from matplotlib.animation import FuncAnimation
        scene.ani = FuncAnimation(scene.fig, tick, interval=args.interval,
                                  blit=False, cache_frame_data=False)
        scene.plt.show()
    finally:
        if reader:
            reader.close()
        stop_fc_client(fc_proc)


def main():
    run(build_parser().parse_args())


if __name__ == '__main__':
    main()
