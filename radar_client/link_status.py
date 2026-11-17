#!/usr/bin/env python3
"""Startup diagnostics for radar2d: CAN health and AP/radar reachability.

Passive by default: it only listens on the CAN interface.  Use --probe to
send a 0x1E/F9 (motor state) request and wait for the ESC echo.
"""
from __future__ import annotations

import argparse
import re
import select
import socket
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from duml_can_tool import (  # noqa: E402
    CAN_SFF_MASK,
    DEFAULT_CMD_TYPE,
    DEFAULT_LOCAL_ID,
    DEFAULT_LOCAL_TYPE,
    DEFAULT_REMOTE_ID,
    DEFAULT_REMOTE_TYPE,
    DumlStreamParser,
    SocketCANDevice,
    build_request_frame,
    parse_frame,
)

PREFIX = "[status]"


def say(text: str = "") -> None:
    print(f"{PREFIX} {text}" if text else PREFIX)


def read_text(path: Path) -> str | None:
    try:
        return path.read_text().strip()
    except OSError:
        return None


def ip_output(*args: str) -> str:
    try:
        return subprocess.check_output(
            ["ip", *args], stderr=subprocess.DEVNULL, text=True, timeout=2)
    except (OSError, subprocess.SubprocessError):
        return ""


def can_details(iface: str) -> dict:
    base = Path(f"/sys/class/net/{iface}")
    stats = base / "statistics"
    info = {
        "present": base.exists(),
        "state": read_text(base / "operstate"),
        "mtu": read_text(base / "mtu"),
        "txqueue": read_text(base / "tx_queue_len"),
        "driver": None,
        "bitrate": None,
        "rx_pkts": read_text(stats / "rx_packets"),
        "tx_pkts": read_text(stats / "tx_packets"),
        "rx_bytes": read_text(stats / "rx_bytes"),
        "tx_bytes": read_text(stats / "tx_bytes"),
        "rx_err": read_text(stats / "rx_errors"),
        "tx_err": read_text(stats / "tx_errors"),
        "rx_drop": read_text(stats / "rx_dropped"),
        "tx_drop": read_text(stats / "tx_dropped"),
        "bus_err": read_text(stats / "bus_error"),
    }
    try:
        info["driver"] = (base / "device" / "driver").resolve().name
    except OSError:
        pass
    match = re.search(r"bitrate\s+(\d+)", ip_output("-details", "link", "show", iface))
    if match:
        info["bitrate"] = match.group(1)
    return info


def probe_traffic(iface: str, seconds: float, do_probe: bool) -> dict:
    result = {"frames": 0, "ids": set(), "probe": "skipped", "error": None}
    try:
        dev = SocketCANDevice(iface)
    except OSError as exc:
        result["error"] = str(exc)
        return result

    seq = int(time.time() * 1000) & 0xFFFF
    parser = DumlStreamParser()
    sent_at = None
    if do_probe:
        request = build_request_frame(
            local_type=DEFAULT_LOCAL_TYPE, local_id=DEFAULT_LOCAL_ID,
            remote_type=DEFAULT_REMOTE_TYPE, remote_id=DEFAULT_REMOTE_ID,
            seq=seq, cmd_set=0x1E, cmd_id=0xF9, payload=b"",
            cmd_type=DEFAULT_CMD_TYPE, enc_type=0,
        )
        try:
            dev.send_stream(0x0A2, request)
            sent_at = time.monotonic()
            result["probe"] = "no reply (yet)"
        except OSError as exc:
            result["probe"] = f"send failed: {exc}"

    deadline = time.monotonic() + seconds
    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            readable, _, _ = select.select([dev.fileno()], [], [], remaining)
            if not readable:
                break
            try:
                can_id, _dlc, data, _ext, is_rtr, is_err = dev.recv()
            except OSError:
                break
            if is_rtr or is_err:
                continue
            result["frames"] += 1
            result["ids"].add(can_id & CAN_SFF_MASK)
            if sent_at is not None and can_id == 0x0A3:
                for duml in parser.feed(data):
                    info, errors = parse_frame(duml)
                    if errors:
                        continue
                    if (info["seq"] == seq and info["cmd_set"] == 0x1E
                            and info["cmd_id"] == 0xF9):
                        result["probe"] = f"reply in {(time.monotonic() - sent_at) * 1000:.0f} ms"
    finally:
        dev.close()
    return result


def network_info(host_ip: str, radar_ip: str, ssh_port: int) -> dict:
    info = {"host_iface": None, "ssh": False, "rtt_ms": None}
    for line in ip_output("-4", "-o", "addr", "show").splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[3].split("/")[0] == host_ip:
            info["host_iface"] = parts[1]
            break
    started = time.monotonic()
    try:
        with socket.create_connection((radar_ip, ssh_port), timeout=0.8):
            info["ssh"] = True
    except OSError:
        pass
    info["rtt_ms"] = (time.monotonic() - started) * 1000
    return info


def report(args: argparse.Namespace) -> None:
    if not args.net_only:
        report_can(args)
    if not args.can_only:
        report_net(args)


def report_can(args: argparse.Namespace) -> None:
    can = can_details(args.iface)
    say(f"--- CAN {args.iface} ---")
    if not can["present"]:
        say("  state     : interface not found (adapter plugged in?)")
    else:
        driver = f", driver {can['driver']}" if can["driver"] else ""
        say(f"  state     : {can['state']}{driver}, mtu {can['mtu']}, txqueuelen {can['txqueue']}")
        if can["bitrate"]:
            say(f"  bitrate   : {can['bitrate']} bit/s")
        say(f"  counters  : rx {can['rx_pkts']} pkt / {can['rx_bytes']} B, "
            f"tx {can['tx_pkts']} pkt / {can['tx_bytes']} B")
        bus_err = can["bus_err"] if can["bus_err"] is not None else "n/a"
        say(f"  errors    : rx {can['rx_err']}, tx {can['tx_err']}, "
            f"dropped {can['rx_drop']}/{can['tx_drop']}, bus {bus_err}")

    traffic = probe_traffic(args.iface, args.listen, args.probe)
    if traffic["error"]:
        say(f"  traffic   : unavailable ({traffic['error']})")
    else:
        ids = ", ".join(f"0x{i:03X}" for i in sorted(traffic["ids"]))
        say(f"  traffic   : {traffic['frames']} frames in {args.listen:.1f}s"
            + (f"  IDs {ids}" if ids else ""))
        say("              -> " + ("BUS ACTIVE" if traffic["frames"] else
                                 "no frames: check wiring / radar power"))
        if args.probe:
            say(f"  echo probe: {traffic['probe']}")


def report_net(args: argparse.Namespace) -> None:
    net = network_info(args.host_ip, args.radar_ip, args.ssh_port)
    say("--- AP / radar ---")
    iface = f" on {net['host_iface']}" if net["host_iface"] else ""
    say(f"  host IP   : {args.host_ip}{iface}")
    if net["ssh"]:
        say(f"  radar     : {args.radar_ip}:{args.ssh_port} reachable "
            f"({net['rtt_ms']:.0f} ms)")
    else:
        say(f"  radar     : {args.radar_ip}:{args.ssh_port} unreachable")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iface", default="can0", help="SocketCAN interface")
    parser.add_argument("--host-ip", default="192.168.8.1")
    parser.add_argument("--radar-ip", default="192.168.8.2")
    parser.add_argument("--ssh-port", type=int, default=107)
    parser.add_argument("--listen", type=float, default=1.0,
                        help="seconds to listen for CAN frames")
    parser.add_argument("--probe", action="store_true",
                        help="send a 0x1E/F9 request and wait for the ESC echo")
    parser.add_argument("--can-only", action="store_true",
                        help="only print CAN diagnostics")
    parser.add_argument("--net-only", action="store_true",
                        help="only print AP/radar diagnostics")
    report(parser.parse_args())
    return 0


if __name__ == "__main__":
    sys.exit(main())
