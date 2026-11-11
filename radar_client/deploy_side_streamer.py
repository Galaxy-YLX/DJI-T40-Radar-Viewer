#!/usr/bin/env python3
"""Deploy + launch the SIDE streamer (UDP 9877) on the radar, then exit.

Used by radar_2d_qt/run.sh so the C++ viewer only has to receive UDP.
Reuses radar_viewer.py's SSH / streamer helpers.
"""
from __future__ import annotations

import argparse
import time

import radar_viewer as rv


def main():
    p = argparse.ArgumentParser(
        description='Deploy the radar_streamer "side" instance and exit')
    p.add_argument('--radar-ip', default='192.168.8.2')
    p.add_argument('--ssh', default=None, metavar='USER@HOST')
    p.add_argument('--ssh-port', type=int, default=rv.DEFAULT_SSH_PORT)
    p.add_argument('--password', default=rv.DEFAULT_SSH_PASSWORD)
    p.add_argument('--pid', type=int, default=None,
                   help='PID of app.elf (auto-detected if omitted)')
    p.add_argument('--interval', type=int, default=20000,
                   help='streamer sleep in us (default 20000 = 50Hz)')
    p.add_argument('--wait-sec', type=float, default=60.0,
                   help='how long to wait for the radar to be reachable')
    args = p.parse_args()

    target = args.ssh or f"root@{args.radar_ip}"

    pid = args.pid
    if pid is None:
        deadline = time.monotonic() + args.wait_sec
        while True:
            try:
                pid = rv.find_remote_pid(target, "app.elf",
                                         port=args.ssh_port,
                                         password=args.password)
                break
            except Exception:
                if time.monotonic() >= deadline:
                    raise SystemExit(
                        f"radar not reachable within {args.wait_sec:.0f}s "
                        f"(is it powered and on WiFi?)")
                time.sleep(1.0)
    print(f"[deploy] app.elf PID={pid}")

    if not rv.deploy_streamer(target, port=args.ssh_port,
                              password=args.password):
        raise SystemExit("streamer deploy failed")

    rv.launch_streamer(target, 'side', pid, port=args.ssh_port,
                       password=args.password, interval_us=args.interval,
                       kill_first=True)
    print("[deploy] SIDE streamer running on UDP 9877")


if __name__ == '__main__':
    main()
