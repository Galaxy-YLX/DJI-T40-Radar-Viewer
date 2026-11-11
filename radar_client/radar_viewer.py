#!/usr/bin/env python3
"""
DJI T40 radar viewer — top 4x4 + side 12x6 in one window.

Feeds from two `radar_streamer` processes on the radar (UDP ports
9876 and 9877). Use --deploy to auto-upload/launch them, or --udp to
attach to an already-running pair.

Output panels (single window):
  - Side 12x6 rotating array 360° PPI (polar, main)
      body_xyz is computed client-side from raw sectors (covers full
      360°; firmware body_xyz array at 0xC3DEE00 is capped at 250 and
      biased to first ~42 slots so we ignore it entirely). Points
      colored by z (height), sized by SNR. Persistence filter on by
      default (filters CFAR flicker).
  - Top 4x4 fixed_height peak list (aux) + 32×64 magnitude map
      HARDWARE NOTE: top antenna is 1T1R monostatic. No physical
      aperture for angle-of-arrival, so the map's "lateral" column
      axis is NOT spatial lateral position — it is Doppler-derived
      "y" assuming fixed height + forward flight. Only meaningful
      while airborne at typical cruise.

Key flags:
  --deploy / --udp HOST      connection mode
  --dump                     one-frame text dump, no GUI
  --hide-filter              hide rear ±60° filter-zone shading
  --show-sectors             overlay raw per-sector CFAR dots
  --azimuth-offset N         sector→compass offset (default 180°)
  --persist-min-hits N       0 disables persistence filter (default 6)
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import paramiko


def _enable_legacy_ssh_rsa():
    """Accept the radar's dropbear 2014.63 host key (ssh-rsa / SHA-1).

    paramiko >= 4 removed ssh-rsa from the preferred host-key algorithms,
    the key-type lookup table and the RSA signature hash table. Restore
    all three; modern algorithms remain preferred for other hosts.
    """
    from cryptography.hazmat.primitives import hashes
    from paramiko import RSAKey, Transport

    if "ssh-rsa" not in Transport._preferred_keys:
        Transport._preferred_keys += ("ssh-rsa",)
    Transport._key_info = dict(Transport._key_info)
    Transport._key_info.setdefault("ssh-rsa", RSAKey)
    RSAKey.HASHES = dict(RSAKey.HASHES)
    RSAKey.HASHES.setdefault("ssh-rsa", hashes.SHA1)


def _select_gui_backend():
    """Pick the first interactive backend that can actually be imported.

    TkAgg needs python3-tkinter, which is not always installed (Fedora),
    so prefer Qt/GTK when available.
    """
    import importlib

    for backend, module in (
        ("QtAgg", "matplotlib.backends.backend_qtagg"),
        ("Gtk4Agg", "matplotlib.backends.backend_gtk4agg"),
        ("TkAgg", "matplotlib.backends.backend_tkagg"),
    ):
        try:
            importlib.import_module(module)
            return backend
        except Exception:
            continue
    raise RuntimeError(
        "no interactive matplotlib backend available; install PyQt6 or python3-tkinter"
    )


# ─── Constants ──────────────────────────────────────────────────────────
DEFAULT_SSH_TARGET   = "root@192.168.8.2"
DEFAULT_SSH_PORT     = 107
DEFAULT_SSH_PASSWORD = "root"

STREAMER_REMOTE_PATH = "/tmp/radar_streamer"
STREAMER_BIN_NAME    = "radar_streamer"

# TOP mode (heatmap + snapshot + peaks)
TOP_PORT          = 9876
TOP_HEATMAP_ROWS  = 32
TOP_HEATMAP_COLS  = 64
TOP_HEATMAP_SIZE  = TOP_HEATMAP_ROWS * TOP_HEATMAP_COLS * 4   # 8192
TOP_SNAPSHOT_SIZE = 0x224                                     # 548
TOP_PEAKS_SIZE    = 0xA8                                      # 168
TOP_FRAME_SIZE    = TOP_HEATMAP_SIZE + TOP_SNAPSHOT_SIZE + TOP_PEAKS_SIZE  # 8908

TOP_RANGE_SCALE_M    = 0.73    # m per range bin
TOP_LATERAL_SCALE_M  = 0.2296  # m per lateral bin
TOP_LATERAL_CENTER   = 32
TOP_RANGE_BIN_START  = 1
TOP_RANGE_BIN_END    = 31
TOP_LATERAL_BIN_START = 12
TOP_LATERAL_BIN_END   = 52

# SIDE mode (omni 360 + body_xyz)
SIDE_PORT             = 9877
SIDE_HEADER_SIZE      = 32
SIDE_N_SECTORS        = 360
SIDE_SECTOR_DUMP      = 160
SIDE_OMNI_BODY_SIZE   = SIDE_N_SECTORS * SIDE_SECTOR_DUMP    # 57600
SIDE_BODY_XYZ_MAX     = 250
SIDE_BODY_XYZ_STRIDE  = 12                                    # 3 floats
SIDE_BODY_XYZ_SIZE    = SIDE_BODY_XYZ_MAX * SIDE_BODY_XYZ_STRIDE  # 3000
SIDE_FRAME_SIZE       = SIDE_HEADER_SIZE + SIDE_OMNI_BODY_SIZE + SIDE_BODY_XYZ_SIZE  # 60632
SIDE_MAX_PTS_PER_SEC  = 7   # (160 - 15) // 20

SIDE_DEG_PER_SECTOR   = 360.0 / SIDE_N_SECTORS   # 1.0 deg
SIDE_RANGE_MIN_M      = 1.5     # hardware spec
SIDE_RANGE_MAX_M      = 50.0
SIDE_ALGO_AZ_HALF     = 120.0   # algorithm filter: keeps +-120 from forward
SIDE_DEFAULT_OFFSET   = 180.0   # azimuth offset applied to BOTH layers (deg).
                                # raw sectors: az_compass = slot_idx*1° + offset
                                # body_xyz:    theta = atan2(y, x) + offset
                                # The 180° default puts the antenna's
                                # physical "forward" at the top of the polar
                                # plot. The encoder zero (slot 0 / field 0)
                                # is physically pointing rear; without the
                                # offset, "fwd" lands in the filter zone
                                # where no data exists — visually wrong.
                                # body_xyz needs the SAME offset to align
                                # with raw sectors (without it, body_xyz
                                # renders 180° opposite to raw sectors).


# ─── SSH / deploy helpers ────────────────────────────────────────────────
def _connect_ssh(host=None, port=None, username=None, password=None,
                 target=None) -> paramiko.SSHClient:
    if target:
        if '@' in target:
            username, host = target.split('@', 1)
        else:
            host = target
            username = username or 'root'
    host = host or DEFAULT_SSH_TARGET.split('@')[-1]
    username = username or DEFAULT_SSH_TARGET.split('@')[0]
    port = port or DEFAULT_SSH_PORT
    password = password or DEFAULT_SSH_PASSWORD

    _enable_legacy_ssh_rsa()
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client._host_keys.clear()
    client._system_host_keys.clear()
    client.connect(host, port=port, username=username, password=password,
                   look_for_keys=False, allow_agent=False, timeout=10)
    return client


def ssh_run_cmd(target, cmd, port=None, password=None, timeout=10.0):
    client = _connect_ssh(target=target, port=port, password=password)
    try:
        _, stdout, stderr = client.exec_command(cmd, timeout=timeout)
        out = stdout.read()
        err = stderr.read()
        rc = stdout.channel.recv_exit_status()
        return rc, out, err
    finally:
        client.close()


def scp_upload(target, local_path, remote_path, port=None, password=None):
    client = _connect_ssh(target=target, port=port, password=password)
    try:
        sftp = client.open_sftp()
        sftp.put(local_path, remote_path)
        sftp.chmod(remote_path, 0o755)
        sftp.close()
    finally:
        client.close()


def find_remote_pid(target, name="app.elf", port=DEFAULT_SSH_PORT,
                    password=None):
    rc, out, err = ssh_run_cmd(target, f"pidof {name}", port=port,
                               password=password)
    if rc != 0 or not out.strip():
        err_str = err.decode(errors='ignore').strip()
        raise RuntimeError(
            f"Cannot find remote process '{name}'"
            + (f" (ssh: {err_str})" if err_str else ""))
    return int(out.decode().strip().split()[0])


def deploy_streamer(target, port=DEFAULT_SSH_PORT, password=None):
    script_dir = Path(__file__).resolve().parent
    local_path = script_dir / STREAMER_BIN_NAME
    if not local_path.exists():
        print(f"[deploy] Binary not found: {local_path}")
        return False

    print(f"[deploy] Killing old {STREAMER_BIN_NAME} if any...")
    try:
        ssh_run_cmd(target, f"killall {STREAMER_BIN_NAME} 2>/dev/null; sleep 0.3",
                    port=port, password=password, timeout=5)
    except Exception:
        pass

    print(f"[deploy] Uploading {local_path} -> {target}:{STREAMER_REMOTE_PATH}")
    try:
        scp_upload(target, str(local_path), STREAMER_REMOTE_PATH,
                   port=port, password=password)
    except Exception as e:
        print(f"[deploy] Upload failed: {e}")
        return False
    print("[deploy] Upload OK")
    return True


def launch_streamer(target, mode, pid, port=DEFAULT_SSH_PORT, password=None,
                    udp_port=None, interval_us=20000, kill_first=True):
    """Launch one radar_streamer instance on the target.

    Set kill_first=False when launching the second of a pair so the first
    streamer isn't killed.
    """
    if udp_port is None:
        udp_port = TOP_PORT if mode == 'top' else SIDE_PORT
    if kill_first:
        ssh_run_cmd(target, f"killall {STREAMER_BIN_NAME} 2>/dev/null; sleep 0.3",
                    port=port, password=password, timeout=5)
    cmd = (f"nohup {STREAMER_REMOTE_PATH} {mode} {pid} {udp_port} {interval_us} "
           f">/dev/null 2>>/tmp/{STREAMER_BIN_NAME}.log &")
    print(f"[launch] {STREAMER_BIN_NAME} {mode} {pid} {udp_port} {interval_us}")
    ssh_run_cmd(target, cmd, port=port, password=password, timeout=5)

    time.sleep(0.8)
    rc, out, _ = ssh_run_cmd(target, f"pidof {STREAMER_BIN_NAME}",
                             port=port, password=password, timeout=5)
    if rc != 0 or not out.strip():
        _, log, _ = ssh_run_cmd(target,
                                f"cat /tmp/{STREAMER_BIN_NAME}.log 2>/dev/null",
                                port=port, password=password, timeout=5)
        raise RuntimeError(f"Streamer failed: {log.decode(errors='ignore').strip()}")
    print(f"[launch] {STREAMER_BIN_NAME} running (PIDs {out.decode().strip()})")


def launch_streamer_pair(target, pid, port=DEFAULT_SSH_PORT, password=None,
                         interval_us=20000):
    """Launch BOTH top and side streamers on the target."""
    launch_streamer(target, 'top',  pid, port=port, password=password,
                    interval_us=interval_us, kill_first=True)
    launch_streamer(target, 'side', pid, port=port, password=password,
                    interval_us=interval_us, kill_first=False)
    rc, out, _ = ssh_run_cmd(target, f"pidof {STREAMER_BIN_NAME}",
                             port=port, password=password, timeout=5)
    pids = out.decode().strip().split()
    if len(pids) < 2:
        raise RuntimeError(
            f"expected 2 streamers running, got {len(pids)}: {pids}")
    print(f"[launch] both streamers running ({pids})")


# ─── UDP transport ──────────────────────────────────────────────────────
class UDPStreamReader:
    def __init__(self, host: str, port: int, frame_size: int):
        import socket
        self._frame_size = frame_size
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(5.0)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF,
                              frame_size * 3)
        self._remote = (host, port)
        self._sock.sendto(b'\x00', self._remote)
        print(f"[udp] Registered with {host}:{port} (frame {frame_size} B)")

    def read_frame(self) -> bytes:
        latest = self.try_read_frame()
        if latest is None:
            self._sock.settimeout(5.0)
            self._sock.setblocking(True)
            data, _ = self._sock.recvfrom(self._frame_size + 128)
            if len(data) != self._frame_size:
                raise RuntimeError(f"Bad frame size: {len(data)}")
            latest = data
        return latest

    def try_read_frame(self):
        """Non-blocking: drain socket buffer, return latest frame or None."""
        latest = None
        self._sock.setblocking(False)
        try:
            while True:
                data, _ = self._sock.recvfrom(self._frame_size + 128)
                if len(data) == self._frame_size:
                    latest = data
        except BlockingIOError:
            pass
        return latest

    def close(self):
        self._sock.close()


# ═══ TOP MODE ════════════════════════════════════════════════════════════

@dataclass
class TopSnapshot:
    distance: float
    valid: int
    present: int
    lateral_a: float
    lateral_b: float


@dataclass
class TopPeaks:
    valid: int
    count: int
    peaks: list
    avg_power: float


def parse_top_snapshot(data: bytes) -> TopSnapshot:
    if len(data) < TOP_SNAPSHOT_SIZE:
        return TopSnapshot(0.0, 0, 0, 0.0, 0.0)
    return TopSnapshot(
        distance=struct.unpack_from('<f', data, 0)[0],
        valid=data[544],
        present=data[545],
        lateral_a=struct.unpack_from('<f', data, 256)[0],
        lateral_b=struct.unpack_from('<f', data, 384)[0],
    )


def parse_top_peaks(data: bytes) -> TopPeaks:
    if len(data) < TOP_PEAKS_SIZE:
        return TopPeaks(0, 0, [], 0.0)
    valid = data[160]
    count = data[161]
    avg_power = struct.unpack_from('<f', data, 164)[0]
    peaks = []
    for i in range(min(count, 8)):
        off = i * 20
        peaks.append({
            'range_bin':  struct.unpack_from('<H', data, off)[0],
            'angle_bin':  struct.unpack_from('<H', data, off + 2)[0],
            'strength':   struct.unpack_from('<f', data, off + 4)[0],
            'threshold':  struct.unpack_from('<f', data, off + 8)[0],
            'range_m':    struct.unpack_from('<f', data, off + 12)[0],
            'lateral_m':  struct.unpack_from('<f', data, off + 16)[0],
        })
    return TopPeaks(valid, count, peaks, avg_power)


def parse_top_frame(raw: bytes):
    hm_raw = raw[:TOP_HEATMAP_SIZE]
    sn_raw = raw[TOP_HEATMAP_SIZE:TOP_HEATMAP_SIZE + TOP_SNAPSHOT_SIZE]
    pk_raw = raw[TOP_HEATMAP_SIZE + TOP_SNAPSHOT_SIZE:]
    heatmap = np.frombuffer(hm_raw, dtype=np.float32).reshape(
        TOP_HEATMAP_ROWS, TOP_HEATMAP_COLS).copy()
    return heatmap, parse_top_snapshot(sn_raw), parse_top_peaks(pk_raw)



def dump_top_once(reader: UDPStreamReader):
    heatmap, snapshot, peaks = parse_top_frame(reader.read_frame())
    print(f"Heatmap: shape={heatmap.shape}  max={heatmap.max():.2f}  "
          f"min={heatmap.min():.2f}  nz={np.count_nonzero(heatmap)}/{heatmap.size}")
    print(f"AGL snapshot: dist={snapshot.distance:.3f}m valid={snapshot.valid} "
          f"present={snapshot.present} latA={snapshot.lateral_a:.3f} "
          f"latB={snapshot.lateral_b:.3f}")
    print(f"Peaks: valid={peaks.valid} count={peaks.count} "
          f"avg_power={peaks.avg_power:.4f}")
    for i, p in enumerate(peaks.peaks):
        print(f"  [{i}] r={p['range_m']:.2f}m lat={p['lateral_m']:+.2f}m "
              f"str={p['strength']:.0f} thr={p['threshold']:.0f} "
              f"rb/ab={p['range_bin']}/{p['angle_bin']}")


# ═══ SIDE MODE ═══════════════════════════════════════════════════════════

class SideFrame:
    """Decoded side-antenna frame (360 sector omni + header meta).

    We parse header + per-sector raw CFAR detections only. The
    firmware's body_xyz tail (at streamer frame offset 57632+) is
    IGNORED — it's capped at 250 entries and processed slot-sequentially
    so it only covers the first ~42 active slots (~60-90° band), making
    it visually useless. Instead, we compute body_xyz CLIENT-SIDE from
    raw sectors (see compute_body_xyz) which covers the full ~360°.
    """
    __slots__ = ('magic', 'version', 'export_busy',
                 'rotation_bytes', 'rot_angle_a', 'rot_angle_b', 'rot_angle_c',
                 'sectors_decoded', 'point_count_total', 'frame_token',
                 'cfar_error', 'sectors')

    def __init__(self, raw: bytes):
        if len(raw) != SIDE_FRAME_SIZE:
            raise ValueError(f"Bad side frame size: {len(raw)} (want {SIDE_FRAME_SIZE})")
        self.magic = bytes(raw[0:4])
        self.version = raw[4]
        self.export_busy = raw[6]
        self.rotation_bytes = bytes(raw[8:24])
        self.rot_angle_a     = struct.unpack_from('<H', raw, 8)[0]
        self.rot_angle_b     = struct.unpack_from('<H', raw, 10)[0]
        self.sectors_decoded = struct.unpack_from('<H', raw, 12)[0]
        self.rot_angle_c     = struct.unpack_from('<H', raw, 14)[0]
        self.point_count_total = struct.unpack_from('<H', raw, 24)[0]
        self.frame_token       = struct.unpack_from('<H', raw, 26)[0]
        self.cfar_error = raw[28]

        # Per-sector polar
        self.sectors = []
        body = raw[SIDE_HEADER_SIZE:SIDE_HEADER_SIZE + SIDE_OMNI_BODY_SIZE]
        for i in range(SIDE_N_SECTORS):
            s = body[i * SIDE_SECTOR_DUMP : (i + 1) * SIDE_SECTOR_DUMP]
            count = s[1]
            field_u16 = struct.unpack_from('<H', s, 5)[0]
            n_valid = min(count, SIDE_MAX_PTS_PER_SEC)
            pts = []
            for pi in range(n_valid):
                off = 15 + pi * 20
                r_m = struct.unpack_from('<f', s, off)[0]
                # record+4 (u32): fed to sub_422C8 as elevation angle
                # after sign-flip (per IDA). Reinterpret as float.
                tbl_raw = struct.unpack_from('<I', s, off + 4)[0]
                el_raw_bits = tbl_raw ^ 0x80000000
                el_rad = struct.unpack('<f', struct.pack('<I', el_raw_bits))[0]
                snr = struct.unpack_from('<H', s, off + 16)[0]
                vel = struct.unpack_from('<H', s, off + 18)[0]
                pts.append({
                    'sector': i,
                    'range_m': r_m,
                    'el_rad': el_rad,
                    'snr': snr,
                    'vel_idx': vel,
                })
            self.sectors.append({
                'idx': i,
                'active': s[0],
                'count': count,
                'field': field_u16,  # FPGA sector_idx, for diagnostic
                'points': pts,
            })
        # (firmware body_xyz tail ignored — see class docstring)

    def all_valid_sector_points(self):
        """Per-sector raw CFAR points, filtered for sane range."""
        out = []
        for sec in self.sectors:
            for p in sec['points']:
                r = p['range_m']
                if not np.isfinite(r) or r < 0.1 or r > 120.0:
                    continue
                out.append(p)
        return out

    def compute_body_xyz(self, azimuth_offset_deg=SIDE_DEFAULT_OFFSET,
                         min_snr=0, min_range_m=0.0):
        """Compute body_xyz from raw sector data.

        Math mirrors firmware sub_422C8 but negates slot_idx because
        the motor rotates CCW (viewed from above) — slot_idx increasing
        corresponds to CCW sweep in body frame, so to get standard
        (x=forward, y=right) body coords we need az = -slot × 1° + offset:
          el_rad = float(table_lookup XOR 0x80000000)
          x = cos(el) × range × cos(az)   (forward)
          y = cos(el) × range × sin(az)   (right)
          z = sin(el) × range              (up)

        Filters:
          min_snr       drop detections with SNR below this (0 = no filter).
                        Useful to cut low-SNR CFAR noise that the
                        persistence filter can't remove (it doesn't
                        flicker — stays in place every frame).
          min_range_m   drop detections closer than this (0 = no filter).
                        Useful to mask the radar's own near-field
                        self-reflections (housing/cables/mount) which
                        appear as persistent fixed clusters.
        """
        offset_rad = np.radians(azimuth_offset_deg)
        deg_per_slot = SIDE_DEG_PER_SECTOR
        out = []
        for sec in self.sectors:
            if not sec['active'] or sec['count'] == 0:
                continue
            az_rad = -np.deg2rad(sec['idx'] * deg_per_slot) + offset_rad
            cos_az, sin_az = np.cos(az_rad), np.sin(az_rad)
            for p in sec['points']:
                r = p['range_m']
                if not np.isfinite(r) or r < 0.1 or r > 120.0:
                    continue
                if r < min_range_m:
                    continue
                if p['snr'] < min_snr:
                    continue
                # Clamp el instead of dropping. We THINK table_lookup is
                # signed-flipped float elevation (per IDA sub_422C8 input),
                # but if that interpretation is wrong for some records
                # (NaN/Inf/huge values), dropping them would mask real
                # detections. Clamp to 0 (horizontal plane) so every raw
                # sector point produces a body_xyz entry — body_xyz is
                # now a strict 1-to-1 projection of raw sectors, just
                # with (x,y,z) coords instead of (az,r).
                el = p['el_rad']
                if not np.isfinite(el) or abs(el) > np.pi:
                    el = 0.0
                cos_el, sin_el = np.cos(el), np.sin(el)
                out.append({
                    'x': cos_el * r * cos_az,
                    'y': cos_el * r * sin_az,
                    'z': sin_el * r,
                    'r': r,
                    'sector': sec['idx'],
                    'snr': p['snr'],
                })
        return out


# ─── Temporal persistence filter ────────────────────────────────────────
class PersistenceFilter:
    """Keep only detections that persist across N of the last M frames.

    CFAR produces lots of random noise peaks, especially at far range
    where the local threshold is low. These flicker frame-to-frame.
    Real reflectors produce detections that repeat at the same
    (azimuth, range) cell across many frames. A simple cell-hit
    histogram over a sliding window filters noise cheaply.

    tol_az_deg and tol_r_m define the quantization grid:
      cell = (round(az_deg / tol_az_deg), round(range_m / tol_r_m))
    A detection is KEPT if its cell has been hit in >= min_hits of
    the last window_frames frames.
    """
    __slots__ = ('window', 'min_hits', 'tol_az_deg', 'tol_r_m',
                 'history', 'cell_counts', 'last_kept', 'last_hits',
                 'cell_ema')

    EMA_ALPHA = 0.3

    def __init__(self, window=16, min_hits=6, tol_az_deg=10.0, tol_r_m=1.0):
        from collections import deque
        self.window = int(window)
        self.min_hits = int(min_hits)
        self.tol_az_deg = float(tol_az_deg)
        self.tol_r_m = float(tol_r_m)
        self.history = deque(maxlen=self.window)
        self.cell_counts = {}
        self.last_kept = []
        self.last_hits = []
        self.cell_ema = {}

    def _cell(self, az_deg, range_m):
        return (int(round(az_deg / self.tol_az_deg)),
                int(round(range_m / self.tol_r_m)))

    def _expire_oldest(self):
        if len(self.history) == self.window:
            for c in self.history[0]:
                self.cell_counts[c] = self.cell_counts.get(c, 0) - 1
                if self.cell_counts[c] <= 0:
                    self.cell_counts.pop(c, None)
                    self.cell_ema.pop(c, None)

    def update_and_filter(self, points):
        """Add current-frame points to history and return filtered list.

        Each point must have x, y, z (body frame). az_deg = atan2(y, x).
        range_m = sqrt(x²+y²) (horizontal projection).
        Returns (kept_points, hits_per_point).

        Output is deduplicated per cell and position-smoothed via EMA
        so that noise points within the same cell don't cause visual
        flicker (raw coordinates vary frame-to-frame; EMA is stable).
        """
        if not points:
            self._expire_oldest()
            self.history.append(set())
            self.last_kept = []
            self.last_hits = []
            return [], []

        cur_cells = set()
        point_cells = []
        for p in points:
            az = np.rad2deg(np.arctan2(p['y'], p['x']))
            r_xy = float(np.hypot(p['x'], p['y']))
            cell = self._cell(az, r_xy)
            cur_cells.add(cell)
            point_cells.append(cell)

        self._expire_oldest()
        self.history.append(cur_cells)
        for c in cur_cells:
            self.cell_counts[c] = self.cell_counts.get(c, 0) + 1

        from collections import defaultdict
        cell_pts = defaultdict(list)
        for p, c in zip(points, point_cells):
            cell_pts[c].append(p)

        a = self.EMA_ALPHA
        for c, cps in cell_pts.items():
            n = len(cps)
            ax_ = sum(p['x'] for p in cps) / n
            ay_ = sum(p['y'] for p in cps) / n
            az_ = sum(p['z'] for p in cps) / n
            as_ = sum(p['snr'] for p in cps) / n
            if c in self.cell_ema:
                ox, oy, oz, os = self.cell_ema[c]
                self.cell_ema[c] = ((1-a)*ox + a*ax_, (1-a)*oy + a*ay_,
                                    (1-a)*oz + a*az_, (1-a)*os + a*as_)
            else:
                self.cell_ema[c] = (ax_, ay_, az_, as_)

        kept = []
        hits = []
        for c in cur_cells:
            h = self.cell_counts.get(c, 0)
            if h >= self.min_hits and c in self.cell_ema:
                ex, ey, ez, es = self.cell_ema[c]
                kept.append({'x': ex, 'y': ey, 'z': ez, 'snr': es,
                             'r': float(np.hypot(ex, ey)),
                             'sector': cell_pts[c][0]['sector']})
                hits.append(h)
        self.last_kept = kept
        self.last_hits = hits
        return kept, hits


def _radar_health_tag(change_times, frame_count, now_wall):
    """Common radar-health short tag for status panels."""
    if len(change_times) >= 2:
        idle_s = now_wall - change_times[-1]
        if idle_s > 2.0:
            return f"IDLE {idle_s:.1f}s"
        hz = (len(change_times) - 1) / max(
            change_times[-1] - change_times[0], 0.01)
        if hz >= 3.0:
            return f"LIVE {hz:.1f}Hz"
        return f"slow {hz:.2f}Hz"
    if frame_count >= 10:
        return f"FROZEN {frame_count}f"
    return "warming up"


def _setup_dark_textpanel(ax, title=None, title_color='#9db2d3'):
    """Dark-theme text panel used for peak lists / status boxes."""
    ax.set_facecolor('#0f1115')
    ax.set_xticks([]); ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color('#2a2f3a')
    if title:
        ax.set_title(title, fontsize=10, color=title_color,
                     loc='left', pad=6)



def dump_side_once(reader: UDPStreamReader, azimuth_offset_deg=SIDE_DEFAULT_OFFSET):
    raw = reader.read_frame()
    fr = SideFrame(raw)
    print(f"Magic: {fr.magic!r}  version: {fr.version}")
    print(f"  rot_angle A/B/C: {fr.rot_angle_a}/{fr.rot_angle_b}/{fr.rot_angle_c}")
    print(f"  sectors_decoded: {fr.sectors_decoded}")
    print(f"  frame_token: {fr.frame_token}   point_count_total: {fr.point_count_total}")
    print(f"  cfar_error: {fr.cfar_error}   export_busy: {fr.export_busy}")

    sec_pts = fr.all_valid_sector_points()
    sectors_with_pts = sum(1 for s in fr.sectors if s['points'])
    print(f"\n{sectors_with_pts}/{SIDE_N_SECTORS} sectors active  "
          f"({len(sec_pts)} raw CFAR points)")

    body = fr.compute_body_xyz(azimuth_offset_deg=azimuth_offset_deg)
    print(f"\nbody_xyz (computed client-side from raw sectors, offset {azimuth_offset_deg:.0f}°): "
          f"{len(body)} points")
    if body:
        # Azimuth histogram (30° bins)
        bins = [0] * 12
        for p in body:
            az = np.rad2deg(np.arctan2(p['y'], p['x'])) % 360.0
            bins[int(az // 30) % 12] += 1
        print(f"  az distribution (30° bins):")
        for b, cnt in enumerate(bins):
            bar = '#' * min(60, cnt)
            print(f"    {b*30:>3}-{b*30+29:>3}°  {cnt:>4}  {bar}")
        # Nearest-15 points by horizontal range
        print(f"\n  nearest-15 points (horizontal projection r = √(x²+y²)):")
        print(f"    {'#':>3}  {'az°':>7}  {'r_xy':>6}  {'z':>6}  "
              f"{'snr':>4}  {'sect':>4}")
        by_r = sorted(body, key=lambda p: np.hypot(p['x'], p['y']))
        for i, p in enumerate(by_r[:15]):
            r_xy = float(np.hypot(p['x'], p['y']))
            az = float(np.rad2deg(np.arctan2(p['y'], p['x']))) % 360.0
            print(f"    {i:>3}  {az:>+7.1f}  {r_xy:>5.2f}m {p['z']:>+5.2f}m "
                  f"  {p['snr']:>4d}  {p['sector']:>4d}")


# ═══ BOTH MODE (top + side simultaneously) ══════════════════════════════
def run_both_viewer(top_reader: UDPStreamReader,
                    side_reader: UDPStreamReader,
                    interval_ms=80,
                    show_filter=True, show_sectors=False, hide_body=False,
                    azimuth_offset_deg=SIDE_DEFAULT_OFFSET, cw=True,
                    vmin_snr=30.0, vmax_snr=255.0,
                    persist_window=16, persist_min_hits=6,
                    persist_tol_az=10.0, persist_tol_r=1.0,
                    min_snr=0, min_range_m=0.0):
    """
    Compact combined view: TOP block, SIDE block, STATS in one narrow
    strip so this window sits alongside radar_min_client on the same
    screen. TOP panels are visually grouped with a "TOP 4×4" banner,
    SIDE panels grouped with a "SIDE 12×6" banner, stats in a clearly
    separated bottom strip.

    Layout (figsize 13×7.5):
        ┌── SIDE 12×6 ────────────────┬── TOP 4×4 ────────────────┐
        │                             │ ┌── peaks (primary) ────┐ │
        │    polar PPI                │ │ # range lat str thr  │ │
        │    + body_xyz (z color)     │ └───────────────────────┘ │
        │    + sectors (dim bg)       │ ┌── 32×64 hm (aux) ─────┐ │
        │                             │ │                       │ │
        │  body_xyz nearest-N  (below)│ │                       │ │
        │                             │ └───────────────────────┘ │
        ├─────────────────────────────┴───────────────────────────┤
        │ STATS: TOP frame/Hz/health  |  SIDE frame/Hz/health/... │
        └─────────────────────────────────────────────────────────┘
    """
    import matplotlib
    matplotlib.use(_select_gui_backend())
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm, Normalize
    from matplotlib.cm import ScalarMappable
    import matplotlib.patches as mpatches

    fig = plt.figure(figsize=(13, 7.5))
    fig.patch.set_facecolor('#161a22')  # dark canvas helps visual grouping

    # Outer split: 2 rows (content / stats-strip), content split 2 cols
    outer = fig.add_gridspec(
        2, 1, height_ratios=[1.0, 0.09],
        hspace=0.06, left=0.035, right=0.985, top=0.94, bottom=0.04)

    content = outer[0, 0].subgridspec(
        1, 2, width_ratios=[1.05, 1.0], wspace=0.12)

    # SIDE block (left column): PPI big, body_xyz list small below
    side_block = content[0, 0].subgridspec(
        2, 1, height_ratios=[3.2, 1.0], hspace=0.15)
    ax_ppi        = fig.add_subplot(side_block[0, 0], projection='polar')
    ax_side_peaks = fig.add_subplot(side_block[1, 0])

    # TOP block (right column): peaks big, heatmap small below
    top_block = content[0, 1].subgridspec(
        2, 1, height_ratios=[1.3, 1.6], hspace=0.18)
    ax_top_peaks = fig.add_subplot(top_block[0, 0])
    ax_top_hm    = fig.add_subplot(top_block[1, 0])

    # Stats strip (bottom)
    ax_status = fig.add_subplot(outer[1, 0])

    # Section banners (figure-level text; tight-positioned top margin).
    # Panel sub-titles are intentionally omitted below to avoid overlap.
    fig.text(0.27, 0.985, 'SIDE 12×6 rotating',
             fontsize=10, color='#9db2d3', ha='center', va='top', weight='bold')
    fig.text(0.76, 0.985, 'TOP 4×4  (1T1R)',
             fontsize=10, color='#d3af9d', ha='center', va='top', weight='bold')

    # ─── SIDE PPI setup ───
    _ZOOM_PRESETS = [10, 20, 30, 50]
    _zoom_idx = [_ZOOM_PRESETS.index(SIDE_RANGE_MAX_M)
                 if SIDE_RANGE_MAX_M in _ZOOM_PRESETS else 2]
    _cur_range = [float(_ZOOM_PRESETS[_zoom_idx[0]])]

    ax_ppi.set_theta_zero_location('N')
    ax_ppi.set_theta_direction(-1 if cw else 1)
    ax_ppi.set_rlim(0, _cur_range[0] * 1.05)
    ax_ppi.set_rlabel_position(135)
    ax_ppi.grid(color='#444', alpha=0.4, linestyle=':')
    ax_ppi.tick_params(colors='#b8c0cc', labelsize=7)
    ax_ppi.set_facecolor('#0b0e12')
    for sp in ax_ppi.spines.values(): sp.set_color('#2a2f3a')
    thetas_full = np.linspace(0, 2 * np.pi, 360)
    ax_ppi.plot(thetas_full, np.full_like(thetas_full, SIDE_RANGE_MIN_M),
                color='#ff6666', linestyle='--', linewidth=1, alpha=0.7)
    _outer_ring, = ax_ppi.plot(thetas_full, np.full_like(thetas_full, _cur_range[0]),
                color='#ff6666', linestyle='--', linewidth=0.8, alpha=0.5)
    _filter_fill = None
    if show_filter:
        rear = np.linspace(np.deg2rad(SIDE_ALGO_AZ_HALF),
                           np.deg2rad(360 - SIDE_ALGO_AZ_HALF), 100)
        _filter_fill = ax_ppi.fill_between(rear, SIDE_RANGE_MIN_M, _cur_range[0],
                            color='gray', alpha=0.15)
    sec_scat = ax_ppi.scatter([], [], c=[], cmap='viridis',
                              norm=Normalize(vmin=vmin_snr, vmax=vmax_snr),
                              s=[], alpha=0.45, edgecolors='none', zorder=4)
    body_norm = Normalize(vmin=-3.0, vmax=3.0)
    body_cmap = plt.cm.coolwarm
    body_scat = ax_ppi.scatter([], [], c=[], cmap='coolwarm',
                               norm=body_norm, s=[], alpha=0.75,
                               edgecolors='black', linewidths=0.4, zorder=6)
    ax_ppi.plot([0], [0], marker='^', color='white', markersize=8,
                markeredgecolor='black', zorder=7)
    _fwd_label = ax_ppi.text(0, _cur_range[0] * 0.92, 'fwd',
                color='#ffcc66', fontsize=7.5, ha='center', va='bottom')

    def _apply_zoom():
        nonlocal _filter_fill
        r = _cur_range[0]
        ax_ppi.set_rlim(0, r * 1.05)
        ticks = [t for t in [2, 5, 10, 15, 20, 30, 40, 50] if t <= r]
        ax_ppi.set_rticks(ticks)
        _outer_ring.set_ydata(np.full(360, r))
        _fwd_label.set_position((0, r * 0.92))
        if _filter_fill is not None:
            _filter_fill.remove()
            _filter_fill = None
        if show_filter:
            rear = np.linspace(np.deg2rad(SIDE_ALGO_AZ_HALF),
                               np.deg2rad(360 - SIDE_ALGO_AZ_HALF), 100)
            _filter_fill = ax_ppi.fill_between(rear, SIDE_RANGE_MIN_M, r,
                                               color='gray', alpha=0.15)
        fig.canvas.draw_idle()

    def _on_zoom_key(event):
        if event.key in ('+', '='):
            _zoom_idx[0] = min(_zoom_idx[0] + 1, len(_ZOOM_PRESETS) - 1)
        elif event.key in ('-', '_'):
            _zoom_idx[0] = max(_zoom_idx[0] - 1, 0)
        else:
            return
        _cur_range[0] = float(_ZOOM_PRESETS[_zoom_idx[0]])
        _apply_zoom()

    fig.canvas.mpl_connect('key_press_event', _on_zoom_key)
    _apply_zoom()
    # Compact inline colorbars — put them INSIDE the polar plot cell as
    # tiny inset axes so they don't steal width from the right column
    cax_snr = fig.add_axes([0.48, 0.55, 0.008, 0.15])
    cax_z   = fig.add_axes([0.48, 0.35, 0.008, 0.15])
    sm_snr = ScalarMappable(norm=Normalize(vmin=vmin_snr, vmax=vmax_snr),
                            cmap='viridis'); sm_snr.set_array([])
    cb_snr = fig.colorbar(sm_snr, cax=cax_snr)
    cb_snr.set_label('SNR', fontsize=6.5, color='#b8c0cc')
    cb_snr.ax.tick_params(labelsize=5, colors='#b8c0cc')
    sm_z = ScalarMappable(norm=body_norm, cmap='coolwarm'); sm_z.set_array([])
    cb_z = fig.colorbar(sm_z, cax=cax_z)
    cb_z.set_label('z (m)', fontsize=6.5, color='#b8c0cc')
    cb_z.ax.tick_params(labelsize=5, colors='#b8c0cc')

    # ─── SIDE body_xyz nearest-N list (below PPI) ───
    ax_side_peaks.set_facecolor('#0f1115')
    ax_side_peaks.set_xticks([]); ax_side_peaks.set_yticks([])
    for sp in ax_side_peaks.spines.values(): sp.set_color('#2a2f3a')
    side_peak_text = ax_side_peaks.text(
        0.01, 0.95, '', transform=ax_side_peaks.transAxes,
        fontsize=8.5, family='monospace', va='top', ha='left', color='#e6edf3')

    # ─── TOP peaks (primary) ───
    ax_top_peaks.set_facecolor('#0f1115')
    ax_top_peaks.set_xticks([]); ax_top_peaks.set_yticks([])
    for sp in ax_top_peaks.spines.values(): sp.set_color('#2a2f3a')
    top_peak_text = ax_top_peaks.text(
        0.008, 0.95, '', transform=ax_top_peaks.transAxes,
        fontsize=10, family='monospace', va='top', ha='left', color='#e6edf3')
    top_peak_summary = ax_top_peaks.text(
        0.008, 0.05, '', transform=ax_top_peaks.transAxes,
        fontsize=8, family='monospace', va='bottom', ha='left', color='#8b949e')

    # ─── TOP heatmap ───
    range_m   = np.arange(TOP_HEATMAP_ROWS) * TOP_RANGE_SCALE_M
    lateral_m = (np.arange(TOP_HEATMAP_COLS) - TOP_LATERAL_CENTER) * TOP_LATERAL_SCALE_M
    init_top = parse_top_frame(top_reader.read_frame())
    init_hm = init_top[0]
    hm_disp_init = np.maximum(init_hm, 1e-6)
    hm_max = max(hm_disp_init.max(), 1.0)
    norm = LogNorm(vmin=max(0.01, hm_max * 1e-4), vmax=hm_max)
    im = ax_top_hm.imshow(
        hm_disp_init, aspect='auto', origin='lower',
        extent=[lateral_m[0], lateral_m[-1], range_m[0], range_m[-1]],
        cmap='viridis', norm=norm, interpolation='bilinear')
    ax_top_hm.set_xlabel('"lat" (Doppler)', fontsize=6.5)
    ax_top_hm.set_ylabel('range (m)', fontsize=6.5)
    ax_top_hm.tick_params(labelsize=5.5)
    ax_top_hm.plot([0.0], [0.0], marker='^', color='white', markersize=6,
                   markeredgecolor='black', zorder=6, clip_on=False)
    rect = mpatches.Rectangle(
        (lateral_m[TOP_LATERAL_BIN_START], range_m[TOP_RANGE_BIN_START]),
        (TOP_LATERAL_BIN_END - TOP_LATERAL_BIN_START) * TOP_LATERAL_SCALE_M,
        (TOP_RANGE_BIN_END - TOP_RANGE_BIN_START) * TOP_RANGE_SCALE_M,
        linewidth=0.7, edgecolor='white', facecolor='none',
        linestyle='--', alpha=0.5)
    ax_top_hm.add_patch(rect)
    fig.colorbar(im, ax=ax_top_hm, pad=0.02, fraction=0.04
                 ).ax.tick_params(labelsize=5)
    top_peak_scatter = ax_top_hm.scatter([], [], c='red', marker='x',
                                          s=25, linewidths=1.0, zorder=5)

    # ─── Stats strip (bottom, distinct visual box) ───
    ax_status.set_facecolor('#1a1f2a')
    ax_status.set_xticks([]); ax_status.set_yticks([])
    for sp in ax_status.spines.values():
        sp.set_color('#3a4150'); sp.set_linewidth(1.0)
    status_text_top  = ax_status.text(
        0.005, 0.5, '', transform=ax_status.transAxes,
        fontsize=8, family='monospace', va='center', ha='left', color='#d3af9d')
    status_text_side = ax_status.text(
        0.50, 0.5, '', transform=ax_status.transAxes,
        fontsize=8, family='monospace', va='center', ha='left', color='#9db2d3')

    # State
    top_count = [0]; side_count = [0]
    t_start = [time.time()]
    last_top_hash = [None];  top_change_times  = deque(maxlen=20)
    last_side_key = [None];  side_change_times = deque(maxlen=20)
    last_top_frame  = [init_top]
    last_side_frame = [None]
    persist_on = persist_min_hits > 0
    pfilter = PersistenceFilter(
        window=persist_window, min_hits=persist_min_hits,
        tol_az_deg=persist_tol_az, tol_r_m=persist_tol_r)
    last_kept = [0]; last_raw = [0]  # stored for status display
    got_new_side = [False]
    cached_body_list = [[]]  # cached body_pts_for_list for stale ticks

    def update(_=None):
        now_wall = time.time()
        got_new_side[0] = False

        # ── Poll both streams (non-blocking) ──
        top_raw = top_reader.try_read_frame()
        if top_raw is not None:
            try:
                last_top_frame[0] = parse_top_frame(top_raw)
                top_count[0] += 1
                h = hash(last_top_frame[0][0].tobytes())
                if h != last_top_hash[0]:
                    top_change_times.append(now_wall); last_top_hash[0] = h
            except Exception as e:
                print(f"[top] parse error: {e}")
        side_raw = side_reader.try_read_frame()
        if side_raw is not None:
            try:
                last_side_frame[0] = SideFrame(side_raw)
                side_count[0] += 1
                got_new_side[0] = True
                fr = last_side_frame[0]
                key = (fr.frame_token, fr.sectors_decoded, fr.point_count_total)
                if key != last_side_key[0]:
                    side_change_times.append(now_wall); last_side_key[0] = key
            except Exception as e:
                print(f"[side] parse error: {e}")

        # ═══ TOP ═══
        heatmap, snapshot, pinfo = last_top_frame[0]
        hm_disp = np.maximum(heatmap, 1e-6)
        im.set_data(hm_disp)
        cur_top_max = hm_disp.max()
        if cur_top_max > 0.01:
            im.set_norm(LogNorm(vmin=max(0.01, cur_top_max * 1e-4), vmax=cur_top_max))
        if pinfo.valid and pinfo.count > 0:
            top_peak_scatter.set_offsets(np.column_stack(
                [[p['lateral_m'] for p in pinfo.peaks],
                 [p['range_m']   for p in pinfo.peaks]]))
        else:
            top_peak_scatter.set_offsets(np.empty((0, 2)))

        header = (f"{'#':>2}  {'range':>6}  {'lat':>7}  {'str':>8}  "
                  f"{'thr':>7}  {'SNR':>5}")
        rows = [header, '─' * len(header)]
        if pinfo.valid and pinfo.count > 0:
            for i, p in enumerate(pinfo.peaks[:6]):
                thr = p['threshold'] if p['threshold'] > 1.0 else 1.0
                snr_db = 20.0 * np.log10(max(p['strength'], 1.0) / thr)
                rows.append(f"{i:>2}  {p['range_m']:>5.2f}m {p['lateral_m']:>+5.2f}m  "
                            f"{p['strength']:>8.0f}  {p['threshold']:>7.0f}  "
                            f"{snr_db:>+5.1f}")
        else:
            rows += ["", "  (no peaks this frame)"]
        top_peak_text.set_text('\n'.join(rows))
        max_str = max((p['strength'] for p in pinfo.peaks[:pinfo.count]), default=0.0)
        top_peak_summary.set_text(
            f"valid={pinfo.valid}  n={pinfo.count if pinfo.valid else 0:>2}  "
            f"max={max_str:>7.0f}  avg={pinfo.avg_power:>7.1f}"
        )

        # ═══ SIDE ═══
        if last_side_frame[0] is not None:
            fr = last_side_frame[0]
            # Optional raw sector diagnostic overlay
            if show_sectors:
                sec_pts = fr.all_valid_sector_points()
                if sec_pts:
                    az = np.array([(p['sector'] * SIDE_DEG_PER_SECTOR
                                    + azimuth_offset_deg) % 360.0
                                   for p in sec_pts])
                    sec_scat.set_offsets(np.column_stack(
                        [np.deg2rad(az),
                         np.array([p['range_m'] for p in sec_pts])]))
                    snrs = np.array([p['snr'] for p in sec_pts], dtype=float)
                    sec_scat.set_array(snrs)
                    sec_scat.set_sizes(np.full_like(snrs, 7.0))
                else:
                    sec_scat.set_offsets(np.empty((0, 2)))
                    sec_scat.set_array(np.array([]))
                    sec_scat.set_sizes(np.array([]))
            else:
                sec_scat.set_offsets(np.empty((0, 2)))

            # Client-computed body_xyz (full 360°) with persistence filter.
            # Only recompute when a genuinely new side frame arrived —
            # re-feeding stale data inflates persistence cell_counts.
            if hide_body:
                body_scat.set_offsets(np.empty((0, 2)))
                body_scat.set_array(np.array([]))
                body_scat.set_sizes(np.array([]))
                last_raw[0] = 0; last_kept[0] = 0
                cached_body_list[0] = []
            elif got_new_side[0]:
                all_pts = fr.compute_body_xyz(
                    azimuth_offset_deg=azimuth_offset_deg,
                    min_snr=min_snr, min_range_m=min_range_m)
                if persist_on:
                    pts, _hits = pfilter.update_and_filter(all_pts)
                else:
                    pts = all_pts
                last_raw[0] = len(all_pts); last_kept[0] = len(pts)
                blist = []
                if pts:
                    xs = np.array([p['x'] for p in pts])
                    ys = np.array([p['y'] for p in pts])
                    zs = np.array([p['z'] for p in pts])
                    snrs = np.array([p['snr'] for p in pts], dtype=float)
                    rs_xy = np.sqrt(xs ** 2 + ys ** 2)
                    thetas = np.arctan2(ys, xs)
                    sizes = 8 + np.clip(np.sqrt(snrs) * 0.8, 0, 12)
                    body_scat.set_offsets(np.column_stack([thetas, rs_xy]))
                    body_scat.set_array(zs)
                    body_scat.set_sizes(sizes)
                    snr_brightness = np.clip(snrs / 200.0, 0.15, 1.0)
                    edge_rgba = np.column_stack([
                        snr_brightness, snr_brightness, snr_brightness,
                        np.ones_like(snr_brightness)])
                    body_scat.set_edgecolors(edge_rgba)
                    for i in np.argsort(rs_xy)[:6]:
                        blist.append({
                            'r': float(rs_xy[i]),
                            'az': float(np.rad2deg(thetas[i])) % 360.0,
                            'z': float(zs[i]),
                            'snr': float(snrs[i]),
                        })
                else:
                    body_scat.set_offsets(np.empty((0, 2)))
                    body_scat.set_array(np.array([]))
                    body_scat.set_sizes(np.array([]))
                cached_body_list[0] = blist

            body_pts_for_list = cached_body_list[0]
            sh = f"{'#':>2}  {'az°':>6}  {'r':>6}  {'z':>6}  {'snr':>4}"
            srows = [sh, '─' * len(sh)]
            if body_pts_for_list:
                for i, p in enumerate(body_pts_for_list):
                    az_s = (p['az'] - 360) if p['az'] > 180 else p['az']
                    srows.append(f"{i:>2}  {az_s:>+6.1f}  {p['r']:>5.2f}m  "
                                 f"{p['z']:>+5.2f}  {p['snr']:>4.0f}")
            else:
                srows += ["", "  (no points this frame)"]
            side_peak_text.set_text('\n'.join(srows))

        # ═══ Stats strip ═══
        elapsed = max(now_wall - t_start[0], 0.01)
        top_fps  = top_count[0]  / elapsed
        side_fps = side_count[0] / elapsed
        th = _radar_health_tag(top_change_times,  top_count[0], now_wall)
        sh_ = _radar_health_tag(side_change_times, side_count[0], now_wall)
        status_text_top.set_text(
            f"TOP  f={top_count[0]:<4} s={top_fps:>4.1f}Hz {th:>12}   "
            f"AGL={snapshot.distance:.2f}m v/p={snapshot.valid}/{snapshot.present} "
            f"max={cur_top_max:>5.0f}"
        )
        if last_side_frame[0] is not None:
            fr = last_side_frame[0]
            sectors_with_pts = sum(1 for s in fr.sectors if s['points'])
            raw_pts_n = sum(len(s['points']) for s in fr.sectors)
            cfar_tag = "CFARerr" if fr.cfar_error else "CFARok"
            # raw_pts_n (direct count) vs last_raw (after compute filters)
            # vs last_kept (after persistence filter). body_xyz is always
            # a subset of raw: we should see last_raw <= raw_pts_n.
            if hide_body:
                main_tag = f"sec_pts={raw_pts_n}"
            elif persist_on:
                main_tag = f"sec_pts={raw_pts_n} body_raw={last_raw[0]} kept={last_kept[0]}"
            else:
                main_tag = f"sec_pts={raw_pts_n} body={last_raw[0]}"
            status_text_side.set_text(
                f"SIDE f={side_count[0]:<4} s={side_fps:>4.1f}Hz {sh_:>12}   "
                f"{main_tag} sec_w_pts={sectors_with_pts}/360 "
                f"tk={fr.frame_token} {cfar_tag} range={int(_cur_range[0])}m[+/-]"
            )
        else:
            status_text_side.set_text(
                f"SIDE f={side_count[0]:<4} s={side_fps:>4.1f}Hz {sh_:>12}   "
                f"(no frame yet)"
            )

        return [im, top_peak_text, top_peak_summary, top_peak_scatter,
                sec_scat, body_scat, side_peak_text,
                status_text_top, status_text_side]

    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update, interval=interval_ms, blit=False,
                        cache_frame_data=False)
    plt.show()


def dump_both_once(top_reader, side_reader, azimuth_offset_deg=SIDE_DEFAULT_OFFSET):
    print("=== TOP frame ===")
    dump_top_once(top_reader)
    print()
    print("=== SIDE frame ===")
    dump_side_once(side_reader, azimuth_offset_deg=azimuth_offset_deg)


# ═══ MAIN ════════════════════════════════════════════════════════════════
def make_both_readers(args):
    """Launch both streamers if needed, then connect to both UDP ports."""
    pid = args.pid
    ssh_port = args.ssh_port
    ssh_pw = args.password

    if args.deploy:
        target = args.ssh or DEFAULT_SSH_TARGET
        if pid is None:
            pid = find_remote_pid(target, "app.elf", port=ssh_port,
                                  password=ssh_pw)
            print(f"Found app.elf PID: {pid}")
        if not deploy_streamer(target, port=ssh_port, password=ssh_pw):
            raise RuntimeError("deploy failed")
        launch_streamer_pair(target, pid, port=ssh_port, password=ssh_pw,
                             interval_us=args.stream_interval)
        host = target.split('@')[-1] if '@' in target else target
    elif args.udp:
        host = args.udp.split(':')[0] if ':' in args.udp else args.udp
    else:
        raise SystemExit("Error: specify --deploy or --udp.")

    top_reader  = UDPStreamReader(host, TOP_PORT,  TOP_FRAME_SIZE)
    side_reader = UDPStreamReader(host, SIDE_PORT, SIDE_FRAME_SIZE)
    return top_reader, side_reader


def main():
    parser = argparse.ArgumentParser(
        description='DJI T40 Radar — unified viewer (top + side together)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  radar_viewer.py --deploy                     # launch + view\n"
            "  radar_viewer.py --udp 192.168.8.2            # attach to running pair\n"
            "  radar_viewer.py --deploy --dump              # one-frame text dump\n"
            "  radar_viewer.py --deploy --show-sectors      # overlay raw CFAR\n"
            "  radar_viewer.py --deploy --persist-min-hits 0  # disable filter\n"))

    # Common: connection / deploy
    parser.add_argument('--ssh', metavar='USER@HOST', nargs='?',
                        const=DEFAULT_SSH_TARGET, default=None,
                        help=f'SSH target (default: {DEFAULT_SSH_TARGET})')
    parser.add_argument('--ssh-port', type=int, default=DEFAULT_SSH_PORT)
    parser.add_argument('--password', default=DEFAULT_SSH_PASSWORD)
    parser.add_argument('--udp', metavar='HOST[:PORT]', nargs='?',
                        const=None, default=None,
                        help='UDP connect to running streamer pair '
                             '(default ports 9876/9877)')
    parser.add_argument('--deploy', action='store_true',
                        help='Upload + launch radar_streamer pair, then connect')
    parser.add_argument('--pid', type=int, default=None,
                        help='PID of app.elf (auto-detected if omitted)')
    parser.add_argument('--stream-interval', type=int, default=20000,
                        help='Remote streamer sleep in µs (default 20000 = 50Hz)')
    parser.add_argument('--dump', action='store_true',
                        help='Print one frame as text and exit (no GUI)')
    parser.add_argument('--interval', type=int, default=80,
                        help='Display update interval ms (default 80)')

    # SIDE-specific
    parser.add_argument('--hide-filter', dest='show_filter',
                        action='store_false', default=True,
                        help='Hide the rear ±60° filter-zone shading')
    parser.add_argument('--show-sectors', action='store_true', default=False,
                        help='Overlay raw per-sector CFAR dots (diagnostic). '
                             'Combine with --hide-body to see ONLY raw sectors.')
    parser.add_argument('--hide-body', action='store_true', default=False,
                        help='Hide the computed body_xyz layer. Useful with '
                             '--show-sectors to isolate layers for diagnosis.')
    parser.add_argument('--azimuth-offset', type=float,
                        default=SIDE_DEFAULT_OFFSET,
                        help=f'Sector→compass azimuth offset in deg '
                             f'(default {SIDE_DEFAULT_OFFSET:.0f}; '
                             f'empirical encoder-zero to drone-forward)')
    parser.add_argument('--ccw', action='store_true',
                        help='Plot CCW (math convention). Default = CW.')
    parser.add_argument('--vmin-snr', type=float, default=30.0)
    parser.add_argument('--vmax-snr', type=float, default=255.0)

    # Persistence filter (enabled by default — filters CFAR flicker)
    parser.add_argument('--persist-window', type=int, default=16,
                        help='Sliding window frames for persistence '
                             '(default 16 ≈ 1.3 s at 12.5 Hz refresh)')
    parser.add_argument('--persist-min-hits', type=int, default=6,
                        help='Hits required within window to keep a point '
                             '(default 6 = ~37%% of default window). '
                             'Set 0 to disable filter.')
    parser.add_argument('--persist-tol-az', type=float, default=10.0,
                        help='Azimuth tolerance ° for binning (default 10)')
    parser.add_argument('--persist-tol-r', type=float, default=1.0,
                        help='Range tolerance m for binning (default 1.0)')

    # Absolute filters (for persistent fake signals that persistence
    # filter cannot catch — typically radar self-reflections and low-SNR
    # CFAR noise that stays in the same cell every frame).
    parser.add_argument('--min-snr', type=int, default=0,
                        help='Drop detections with SNR below this (default 0 = no filter). '
                             'Useful to cut low-SNR CFAR noise.')
    parser.add_argument('--min-range', type=float, default=0.0,
                        help='Drop detections closer than N meters (default 0 = no filter). '
                             'Useful to mask radar near-field self-reflections.')

    args = parser.parse_args()

    readers_to_close = []
    try:
        top_reader, side_reader = make_both_readers(args)
        readers_to_close.extend([top_reader, side_reader])
        if args.dump:
            dump_both_once(top_reader, side_reader,
                           azimuth_offset_deg=args.azimuth_offset)
        else:
            run_both_viewer(
                top_reader, side_reader,
                interval_ms=args.interval,
                show_filter=args.show_filter,
                show_sectors=args.show_sectors,
                hide_body=args.hide_body,
                azimuth_offset_deg=args.azimuth_offset,
                cw=(not args.ccw),
                vmin_snr=args.vmin_snr, vmax_snr=args.vmax_snr,
                persist_window=args.persist_window,
                persist_min_hits=args.persist_min_hits,
                persist_tol_az=args.persist_tol_az,
                persist_tol_r=args.persist_tol_r,
                min_snr=args.min_snr,
                min_range_m=args.min_range)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        for r in readers_to_close:
            try: r.close()
            except Exception: pass


if __name__ == '__main__':
    main()
