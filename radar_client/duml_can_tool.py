#!/usr/bin/env python3
from __future__ import annotations

import argparse
import errno
import select
import shlex
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

from duml_gen import CMDS, _crc16, _crc8, hex_str, parse_frame


CAN_FRAME_STRUCT = struct.Struct("=IB3x8s")
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x7FF
CAN_EFF_MASK = 0x1FFFFFFF

DEFAULT_LINK_ID = 0x520518A2
DEFAULT_IFACE = "can0"
DEFAULT_LOCAL_TYPE = 0
DEFAULT_LOCAL_ID = 0x0A
DEFAULT_REMOTE_TYPE = 0
DEFAULT_REMOTE_ID = 0x18
DEFAULT_CMD_TYPE = 2
DEFAULT_VERSION = 1


_CMD_BY_NAME = {name.replace("_", "-"): (name, cmd_set, cmd_id, payload, desc) for name, cmd_set, cmd_id, payload, desc in CMDS}
_CMD_BY_PAIR = {(cmd_set, cmd_id): (name, desc) for name, cmd_set, cmd_id, _payload, desc in CMDS}
_TYPE_NAMES = {0: "DEV", 1: "PC", 2: "WL2", 3: "RADAR", 7: "AGENT"}
_ID_NAMES = {
    0x00: "default",
    0x02: "app",
    0x03: "fc",
    0x05: "center",
    0x08: "1860",
    0x0A: "pc_assist",
    0x11: "autotest",
    0x13: "nego_s",
    0x14: "nego_h",
    0x18: "esc",
    0x1D: "hms",
    0x1F: "broadcast",
}
_SPECIAL_COMMANDS = [
    ("echo", 0x00, 0x00, "Echo. Default payload is ASCII 'A'. Supports --text or --payload."),
    ("activation", 0x00, 0x32, "Activation helper. Supports --subcmd/--mode/--sn or raw --payload."),
    ("motor-enable", 0x1E, 0x09, "1-byte payload. Supports --value."),
    ("motor-disable", 0x1E, 0x0A, "1-byte payload. Supports --value."),
    ("motor-stop", 0x1E, 0x82, "No payload."),
    ("motor-speed", 0x1E, 0x80, "2-byte little-endian speed setpoint. Supports --speed."),
    ("motor-position", 0x1E, 0x81, "6-byte payload <hI>. Supports --angle and --grating."),
    ("motor-pwm", 0x1E, 0x01, "8-byte payload <II>. Supports --mode and --value2."),
    ("custom", None, None, "Arbitrary DUML frame. Requires --cmd-set and --cmd-id, optional --payload."),
]


def parse_int(value: str) -> int:
    return int(value, 0)


def parse_hex_bytes(value: str) -> bytes:
    cleaned = value.replace(",", " ").replace("0x", "").replace("0X", "")
    parts = [part for part in cleaned.split() if part]
    if not parts:
        return b""
    return bytes.fromhex("".join(parts))


def endpoint_label(dev_type: int, dev_id: int) -> str:
    type_name = _TYPE_NAMES.get(dev_type, f"T{dev_type}")
    id_name = _ID_NAMES.get(dev_id, "")
    if id_name:
        return f"{type_name}:{id_name}(0x{dev_id:02X})"
    return f"{type_name}:0x{dev_id:02X}"


def decode_link_id(link_id: int) -> tuple[int, int, int]:
    bus = (link_id >> 22) & 0x3
    dev_tx_id = (link_id >> 11) & CAN_SFF_MASK
    dev_rx_id = link_id & CAN_SFF_MASK
    return bus, dev_tx_id, dev_rx_id


def build_duml_frame(
    *,
    sender_type: int,
    sender_id: int,
    receiver_type: int,
    receiver_id: int,
    seq: int,
    cmd_set: int,
    cmd_id: int,
    payload: bytes = b"",
    cmd_type: int = DEFAULT_CMD_TYPE,
    is_response: bool = False,
    enc_type: int = 0,
    version: int = DEFAULT_VERSION,
) -> bytes:
    total_len = 13 + len(payload)
    if total_len > 1023:
        raise ValueError(f"frame too large: {total_len}")
    if not 0 <= cmd_type <= 3:
        raise ValueError(f"cmd_type out of range: {cmd_type}")
    if not 0 <= enc_type <= 0x0F:
        raise ValueError(f"enc_type out of range: {enc_type}")
    header = bytes([0x55, total_len & 0xFF, ((total_len >> 8) & 0x03) | ((version & 0x3F) << 2)])
    header_crc = _crc8(header)
    flags = (enc_type & 0x0F) | ((cmd_type & 0x03) << 5) | ((1 if is_response else 0) << 7)
    frame = (
        header
        + bytes(
            [
                header_crc,
                ((sender_type & 0x07) << 5) | (sender_id & 0x1F),
                ((receiver_type & 0x07) << 5) | (receiver_id & 0x1F),
                seq & 0xFF,
                (seq >> 8) & 0xFF,
                flags,
                cmd_set & 0xFF,
                cmd_id & 0xFF,
            ]
        )
        + payload
    )
    return frame + struct.pack("<H", _crc16(frame))


def build_response_frame(info: dict, payload: bytes, enc_type: int = 0) -> bytes:
    return build_duml_frame(
        sender_type=info["receiver_type"],
        sender_id=info["receiver_id"],
        receiver_type=info["sender_type"],
        receiver_id=info["sender_id"],
        seq=info["seq"],
        cmd_set=info["cmd_set"],
        cmd_id=info["cmd_id"],
        payload=payload,
        cmd_type=(info["flags"] >> 5) & 0x03,
        is_response=True,
        enc_type=enc_type,
        version=info.get("version", 1),
    )


def build_request_frame(
    *,
    local_type: int,
    local_id: int,
    remote_type: int,
    remote_id: int,
    seq: int,
    cmd_set: int,
    cmd_id: int,
    payload: bytes,
    cmd_type: int,
    enc_type: int,
) -> bytes:
    return build_duml_frame(
        sender_type=local_type,
        sender_id=local_id,
        receiver_type=remote_type,
        receiver_id=remote_id,
        seq=seq,
        cmd_set=cmd_set,
        cmd_id=cmd_id,
        payload=payload,
        cmd_type=cmd_type,
        is_response=False,
        enc_type=enc_type,
    )


def pack_can_frame(can_id: int, data: bytes, *, extended: bool = False, rtr: bool = False) -> bytes:
    if len(data) > 8:
        raise ValueError("classic CAN payload length must be <= 8")
    wire_id = can_id & (CAN_EFF_MASK if extended else CAN_SFF_MASK)
    if extended:
        wire_id |= CAN_EFF_FLAG
    if rtr:
        wire_id |= CAN_RTR_FLAG
    return CAN_FRAME_STRUCT.pack(wire_id, len(data), data.ljust(8, b"\x00"))


def unpack_can_frame(raw: bytes) -> tuple[int, int, bytes, bool, bool, bool]:
    can_id, dlc, data = CAN_FRAME_STRUCT.unpack(raw[: CAN_FRAME_STRUCT.size])
    is_extended = bool(can_id & CAN_EFF_FLAG)
    is_rtr = bool(can_id & CAN_RTR_FLAG)
    is_err = bool(can_id & CAN_ERR_FLAG)
    frame_id = can_id & (CAN_EFF_MASK if is_extended else CAN_SFF_MASK)
    return frame_id, dlc, data[:dlc], is_extended, is_rtr, is_err


@dataclass
class ActivationCallbackConfig:
    sn: bytes = b""
    activated: bool = True
    timestamp_low: int = 0
    timestamp_high: int = 0
    reverse_status: int = 0
    verify_status: int = 0
    unsupported_status: int = 0xE0
    stage14_status: int = 0xE1

    def timestamp_payload(self) -> bytes:
        sn = self.sn[:16]
        payload = bytearray(26)
        payload[0] = 0
        payload[1] = 1 if self.activated else 0
        struct.pack_into("<I", payload, 2, self.timestamp_low & 0xFFFFFFFF)
        struct.pack_into("<H", payload, 6, self.timestamp_high & 0xFFFF)
        payload[9] = len(sn)
        payload[10 : 10 + len(sn)] = sn
        return bytes(payload)


@dataclass
class MonitorConfig:
    iface: str
    tx_id: int
    rx_id: int
    local_type: int
    local_id: int
    remote_type: int
    remote_id: int
    show_raw_can: bool
    print_hex: bool
    full_payload: bool
    auto_callbacks: set[str]
    activation: ActivationCallbackConfig
    listen_all: bool = False
    respond_broadcast: bool = True


class DumlStreamParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, chunk: bytes) -> list[bytes]:
        self.buffer.extend(chunk)
        frames: list[bytes] = []
        while True:
            if not self.buffer:
                break
            sof_pos = self.buffer.find(0x55)
            if sof_pos < 0:
                self.buffer.clear()
                break
            if sof_pos:
                del self.buffer[:sof_pos]
            if len(self.buffer) < 4:
                break
            total_len = ((self.buffer[2] & 0x03) << 8) | self.buffer[1]
            version = self.buffer[2] >> 2
            if version != 1 or total_len < 13 or total_len > 1023:
                del self.buffer[0]
                continue
            if len(self.buffer) < total_len:
                break
            candidate = bytes(self.buffer[:total_len])
            info, errors = parse_frame(candidate)
            if errors:
                del self.buffer[0]
                continue
            frames.append(candidate)
            del self.buffer[:total_len]
        if len(self.buffer) > 4096:
            del self.buffer[:-1024]
        return frames


class SocketCANDevice:
    def __init__(self, iface: str, listen_ids: Optional[Iterable[int]] = None) -> None:
        self.iface = iface
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.setblocking(False)
        ids = [can_id & CAN_SFF_MASK for can_id in (listen_ids or [])]
        if ids and hasattr(socket, "CAN_RAW_FILTER"):
            filt = bytearray()
            for can_id in ids:
                filt += struct.pack("=II", can_id, CAN_SFF_MASK)
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, bytes(filt))

    def fileno(self) -> int:
        return self.sock.fileno()

    def close(self) -> None:
        self.sock.close()

    def recv(self) -> tuple[int, int, bytes, bool, bool, bool]:
        raw = self.sock.recv(CAN_FRAME_STRUCT.size)
        return unpack_can_frame(raw)

    def send_can(self, can_id: int, data: bytes, *, extended: bool = False) -> None:
        frame = pack_can_frame(can_id, data, extended=extended)
        # SocketCAN tx queues are short (often qlen 10). A DUML frame is
        # split into many 8B chunks written back-to-back, so a non-blocking
        # socket can hit ENOBUFS/EAGAIN. Wait for writability instead of
        # failing the whole frame.
        deadline = time.monotonic() + 1.0
        while True:
            try:
                self.sock.send(frame)
                return
            except OSError as exc:
                if exc.errno not in (errno.ENOBUFS, errno.EAGAIN,
                                     errno.EWOULDBLOCK):
                    raise
                if time.monotonic() >= deadline:
                    raise
                select.select([], [self.sock.fileno()], [], 0.02)

    def send_stream(self, tx_id: int, data: bytes) -> int:
        sent = 0
        for offset in range(0, len(data), 8):
            chunk = data[offset : offset + 8]
            self.send_can(tx_id, chunk)
            sent += len(chunk)
        return sent


class MonitorApp:
    def __init__(self, cfg: MonitorConfig) -> None:
        listen_ids = None if cfg.listen_all else [cfg.rx_id]
        self.cfg = cfg
        self.can = SocketCANDevice(cfg.iface, listen_ids=listen_ids)
        self.stream = DumlStreamParser()

    def close(self) -> None:
        self.can.close()

    def log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        ms = int((time.time() % 1) * 1000)
        print(f"[{stamp}.{ms:03d}] {text}")

    def payload_limit(self, default_limit: int) -> Optional[int]:
        if self.cfg.full_payload:
            return None
        if self.cfg.print_hex:
            return max(default_limit, 128)
        return default_limit

    def send_frame(self, frame: bytes, *, note: str = "") -> None:
        self.can.send_stream(self.cfg.tx_id, frame)
        info, _errors = parse_frame(frame)
        label = describe_frame(info, payload_limit=self.payload_limit(64))
        suffix = f" {note}" if note else ""
        self.log(f"tx {label}{suffix}")

    def should_callback(self, info: dict) -> bool:
        if info.get("is_ack") or ((info.get("flags", 0) >> 7) & 1):
            return False
        if info["receiver_type"] == self.cfg.local_type and info["receiver_id"] == self.cfg.local_id:
            return True
        if self.cfg.respond_broadcast and info["receiver_id"] == 0x1F:
            return True
        return False

    def maybe_callback(self, info: dict) -> None:
        if not self.should_callback(info):
            return
        name = _CMD_BY_PAIR.get((info["cmd_set"], info["cmd_id"]), ("unknown", ""))[0].replace("_", "-")
        handler: Optional[Callable[[dict], Optional[bytes]]] = None
        if name == "echo" and "echo" in self.cfg.auto_callbacks:
            handler = lambda frame: frame["payload"]
        elif name == "activation" and "activation" in self.cfg.auto_callbacks:
            handler = self.activation_reply
        elif name == "heartbeat" and "heartbeat" in self.cfg.auto_callbacks:
            handler = self.heartbeat_reply
        if handler is None:
            return
        payload = handler(info)
        if payload is None:
            return
        self.send_frame(build_response_frame(info, payload), note="(auto-callback)")

    def activation_reply(self, info: dict) -> Optional[bytes]:
        payload = info["payload"]
        if not payload:
            return None
        subcmd = payload[0]
        cfg = self.cfg.activation
        if subcmd == 0x00:
            return struct.pack("<H", 0x0100)
        if subcmd == 0x11:
            return cfg.timestamp_payload()
        if subcmd == 0x12:
            if len(payload) > 1 and payload[1] == 1:
                return bytes([cfg.verify_status & 0xFF])
            return bytes([cfg.reverse_status & 0xFF])
        if subcmd == 0x14:
            return bytes([cfg.stage14_status & 0xFF])
        return bytes([cfg.unsupported_status & 0xFF])

    @staticmethod
    def heartbeat_reply(info: dict) -> Optional[bytes]:
        if info["flags"] & 0x0F:
            return bytes(6)
        return None

    def run(self) -> int:
        self.log(
            f"monitor iface={self.cfg.iface} stream_tx=0x{self.cfg.tx_id:03X} stream_rx=0x{self.cfg.rx_id:03X} "
            f"local={endpoint_label(self.cfg.local_type, self.cfg.local_id)}"
        )
        try:
            while True:
                readable, _, _ = select.select([self.can.fileno()], [], [], 0.5)
                if not readable:
                    continue
                frame_id, dlc, data, is_extended, is_rtr, is_err = self.can.recv()
                if self.cfg.show_raw_can:
                    tags = []
                    if is_extended:
                        tags.append("EFF")
                    if is_rtr:
                        tags.append("RTR")
                    if is_err:
                        tags.append("ERR")
                    self.log(f"can id=0x{frame_id:X} dlc={dlc} data={hex_str(data)} {' '.join(tags)}".rstrip())
                if is_err or is_rtr:
                    continue
                if not self.cfg.listen_all and frame_id != self.cfg.rx_id:
                    continue
                for duml in self.stream.feed(data):
                    info, errors = parse_frame(duml)
                    if errors:
                        self.log(f"rx invalid {'; '.join(errors)} raw={hex_str(duml)}")
                        continue
                    self.log(f"rx {describe_frame(info, payload_limit=self.payload_limit(32))}")
                    extra = decode_payload_summary(info)
                    if extra:
                        self.log(f"   {extra}")
                    self.maybe_callback(info)
        except KeyboardInterrupt:
            self.log("stopped")
            return 0


def describe_frame(info: dict, *, payload_limit: Optional[int]) -> str:
    src = endpoint_label(info["sender_type"], info["sender_id"])
    dst = endpoint_label(info["receiver_type"], info["receiver_id"])
    cmd_name = info.get("cmd_name") or _CMD_BY_PAIR.get((info["cmd_set"], info["cmd_id"]), ("unknown", ""))[0]
    payload = info["payload"]
    if payload_limit is None:
        payload_hex = hex_str(payload)
    else:
        payload_hex = hex_str(payload[:payload_limit])
        if len(payload) > payload_limit:
            payload_hex += " ..."
    flags = []
    if info["flags"] & 0x80:
        flags.append("rsp")
    else:
        flags.append("req")
    flags.append(f"ct={((info['flags'] >> 5) & 0x03)}")
    flags.append(f"enc={info['flags'] & 0x0F}")
    return (
        f"{src} -> {dst} seq={info['seq']} cmd=0x{info['cmd_set']:02X}/0x{info['cmd_id']:02X}"
        f"({cmd_name}) flg=0x{info['flags']:02X}[{' '.join(flags)}] len={len(payload)}"
        + (f" pay={payload_hex}" if payload else " pay=<empty>")
    )


def decode_payload_summary(info: dict) -> str:
    cmd_set = info["cmd_set"]
    cmd_id = info["cmd_id"]
    payload = info["payload"]
    if cmd_set == 0x00 and cmd_id == 0x32 and payload:
        subcmd = payload[0]
        if info["flags"] & 0x80:
            if subcmd in (0xE0, 0xE1):
                return f"activation status=0x{subcmd:02X}"
            if len(payload) == 2:
                return f"activation stage0 capability=0x{struct.unpack('<H', payload)[0]:04X}"
            if len(payload) == 26:
                status = payload[0]
                activated = payload[1]
                ts_low = struct.unpack_from("<I", payload, 2)[0]
                ts_high = struct.unpack_from("<H", payload, 6)[0]
                sn_len = payload[9]
                sn = payload[10 : 10 + min(sn_len, len(payload) - 10)].decode("ascii", errors="ignore")
                return f"activation ts status=0x{status:02X} activated={activated} ts=0x{ts_high:04X}{ts_low:08X} sn={sn!r}"
        else:
            detail = f"activation subcmd=0x{subcmd:02X}"
            if subcmd == 0x12 and len(payload) > 1:
                detail += f" mode=0x{payload[1]:02X}"
            if subcmd == 0x12 and len(payload) > 10:
                maybe_sn = payload[10:].split(b"\x00", 1)[0]
                if maybe_sn:
                    detail += f" sn={maybe_sn.decode('ascii', errors='ignore')!r}"
            return detail
    if cmd_set == 0x1E and cmd_id == 0x21 and len(payload) == 66:
        pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, aux_x, aux_y, aux_z = struct.unpack_from("<9h", payload, 16)
        flight_mode = payload[38]
        flight_flags = payload[39]
        lat = struct.unpack_from("<d", payload, 40)[0]
        lon = struct.unpack_from("<d", payload, 48)[0]
        gps_valid = payload[56]
        pose_tag = payload[65]
        return (
            f"flyinfo pos=({pos_x/1000:.3f},{pos_y/1000:.3f},{pos_z/1000:.3f})m "
            f"vel=({vel_x/1000:.3f},{vel_y/1000:.3f},{vel_z/1000:.3f})m/s "
            f"aux=({aux_x/100:.2f},{aux_y/100:.2f},{aux_z/100:.2f}) "
            f"mode={flight_mode} flags=0x{flight_flags:02X} gps={gps_valid} "
            f"lat={lat:.7f} lon={lon:.7f} pose_tag={pose_tag}"
        )
    if cmd_set == 0x1E and cmd_id == 0x0F and len(payload) == 44:
        wr_power_raw = struct.unpack_from("<I", payload, 21)[0]
        temperature = payload[25]
        angle_delta = struct.unpack_from("<h", payload, 26)[0]
        ctrl_mode = payload[28]
        cur_speed = struct.unpack_from("<H", payload, 29)[0]
        sample_counter = struct.unpack_from("<I", payload, 31)[0]
        status_a = struct.unpack_from("<I", payload, 35)[0]
        status_b = struct.unpack_from("<I", payload, 39)[0]
        health_flag = payload[43]
        return (
            f"rdtt push temp={temperature}C angle_delta={angle_delta} "
            f"ctrl_mode={ctrl_mode} cur_speed={cur_speed} "
            f"wr_power=0x{wr_power_raw:08X} sample={sample_counter} "
            f"status=0x{status_a:08X}/0x{status_b:08X} health={health_flag}"
        )
    if cmd_set == 0x1E and cmd_id == 0x80 and len(payload) == 2:
        speed = struct.unpack("<H", payload)[0]
        return f"motor speed_ref={speed}"
    if cmd_set == 0x1E and cmd_id == 0x81 and len(payload) == 6:
        angle_ref, grat_num = struct.unpack("<hI", payload)
        return f"motor position angle_ref={angle_ref} grat_num={grat_num}"
    if cmd_set == 0x1E and cmd_id == 0x09 and payload:
        return f"motor enable={payload[0]}"
    if cmd_set == 0x1E and cmd_id == 0x0A and payload:
        return f"motor disable={payload[0]}"
    if cmd_set == 0x00 and cmd_id == 0x01 and (info["flags"] & 0x80) and len(payload) >= 31:
        status = struct.unpack_from("<H", payload, 0)[0]
        device_name = payload[2:18].split(b"\x00", 1)[0].decode("ascii", errors="replace")
        fw_ver = struct.unpack_from("<I", payload, 18)[0]
        hw_ver = struct.unpack_from("<I", payload, 22)[0]
        dev_type = struct.unpack_from("<I", payload, 26)[0]
        health = payload[30]
        fw_str = f"{(fw_ver >> 24) & 0xFF}.{(fw_ver >> 16) & 0xFF}.{(fw_ver >> 8) & 0xFF}.{fw_ver & 0xFF}"
        hw_str = f"{(hw_ver >> 24) & 0xFF}.{(hw_ver >> 16) & 0xFF}.{(hw_ver >> 8) & 0xFF}.{hw_ver & 0xFF}"
        return (
            f"device_info status=0x{status:04X} name={device_name!r} "
            f"fw={fw_str}(0x{fw_ver:08X}) hw={hw_str}(0x{hw_ver:08X}) "
            f"dev_type={dev_type} health={health}"
        )
    if cmd_set == 0x00 and cmd_id == 0x00 and payload:
        try:
            return f"echo ascii={payload.decode('ascii')!r}"
        except UnicodeDecodeError:
            return ""
    return ""


def build_named_payload(command: str, args: argparse.Namespace) -> tuple[int, int, bytes]:
    name = command.replace("_", "-")
    if name == "custom":
        return args.cmd_set, args.cmd_id, parse_hex_bytes(args.payload or "")
    if name == "echo":
        if args.text is not None:
            return 0x00, 0x00, args.text.encode("ascii")
        if args.payload:
            return 0x00, 0x00, parse_hex_bytes(args.payload)
        return 0x00, 0x00, b"A"
    if name == "activation":
        if args.payload:
            return 0x00, 0x32, parse_hex_bytes(args.payload)
        payload = bytearray([args.subcmd & 0xFF])
        if args.mode is not None:
            payload.append(args.mode & 0xFF)
        if args.sn is not None:
            payload.extend(b"\x00" * max(0, 10 - len(payload)))
            payload.extend(args.sn.encode("ascii"))
        return 0x00, 0x32, bytes(payload)
    if name == "motor-enable":
        value = 1 if args.value is None else args.value
        return 0x1E, 0x09, bytes([value & 0xFF])
    if name == "motor-disable":
        value = 0 if args.value is None else args.value
        return 0x1E, 0x0A, bytes([value & 0xFF])
    if name == "motor-stop":
        return 0x1E, 0x82, b""
    if name == "motor-speed":
        return 0x1E, 0x80, struct.pack("<H", args.speed & 0xFFFF)
    if name == "motor-position":
        return 0x1E, 0x81, struct.pack("<hI", args.angle, args.grating & 0xFFFFFFFF)
    if name == "motor-pwm":
        mode = 0 if args.mode is None else args.mode
        return 0x1E, 0x01, struct.pack("<II", mode & 0xFFFFFFFF, args.value2 & 0xFFFFFFFF)
    cmd = _CMD_BY_NAME.get(name)
    if cmd is None:
        raise ValueError(f"unknown command {command!r}")
    _orig_name, cmd_set, cmd_id, default_payload, _desc = cmd
    return cmd_set, cmd_id, parse_hex_bytes(args.payload) if getattr(args, "payload", None) else default_payload


def cmd_list(args: argparse.Namespace) -> int:
    _bus, gd32_tx_id, gd32_rx_id = decode_link_id(DEFAULT_LINK_ID)
    print("SocketCAN DUMLv1 helper")
    print(
        f"default link_id=0x{DEFAULT_LINK_ID:08X} "
        f"gd32_tx=0x{gd32_tx_id:03X} gd32_rx=0x{gd32_rx_id:03X} "
        f"host_tx=0x{gd32_rx_id:03X} host_rx=0x{gd32_tx_id:03X}"
    )
    print(
        f"default local endpoint={endpoint_label(DEFAULT_LOCAL_TYPE, DEFAULT_LOCAL_ID)} "
        f"remote endpoint={endpoint_label(DEFAULT_REMOTE_TYPE, DEFAULT_REMOTE_ID)}"
    )
    print()
    print("Script-level commands:")
    for name, cmd_set, cmd_id, desc in _SPECIAL_COMMANDS:
        if cmd_set is None:
            print(f"  {name:<16} custom         {desc}")
        else:
            print(f"  {name:<16} 0x{cmd_set:02X}/0x{cmd_id:02X}  {desc}")
    print()
    print("Known protocol commands from duml_gen.py:")
    overridden = {name for name, _cmd_set, _cmd_id, _desc in _SPECIAL_COMMANDS if name != "custom"}
    for name, cmd_set, cmd_id, _payload, desc in CMDS:
        shown = name.replace("_", "-")
        if shown in overridden:
            continue
        print(f"  {shown:<16} 0x{cmd_set:02X}/0x{cmd_id:02X}  {desc}")
    if args.verbose:
        print()
        print("Auto-callback names:")
        print("  activation, echo, heartbeat")
        print()
        print("Examples:")
        print("  python3 duml_can_tool.py monitor --iface can0 --auto-callback activation")
        print("  python3 duml_can_tool.py send echo --iface can0 --wait 1.0")
        print("  python3 duml_can_tool.py send motor-speed --iface can0 --speed 1200")
        print("  python3 duml_can_tool.py send custom --iface can0 --cmd-set 0x1E --cmd-id 0xEB --payload \"01 02 03\"")
    return 0


def wait_for_matching_response(
    can_dev: SocketCANDevice,
    rx_id: int,
    expected_seq: int,
    expected_cmd_set: int,
    expected_cmd_id: int,
    timeout: float,
    payload_limit: Optional[int],
) -> int:
    parser = DumlStreamParser()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remain = max(0.0, deadline - time.monotonic())
        readable, _, _ = select.select([can_dev.fileno()], [], [], remain)
        if not readable:
            break
        frame_id, _dlc, data, _is_extended, is_rtr, is_err = can_dev.recv()
        if is_err or is_rtr or frame_id != rx_id:
            continue
        for duml in parser.feed(data):
            info, errors = parse_frame(duml)
            if errors:
                continue
            print(f"rx {describe_frame(info, payload_limit=payload_limit)}")
            extra = decode_payload_summary(info)
            if extra:
                print(f"   {extra}")
            if (
                info["seq"] == expected_seq
                and info["cmd_set"] == expected_cmd_set
                and info["cmd_id"] == expected_cmd_id
                and (info["flags"] & 0x80)
            ):
                return 0
    return 1


def cmd_monitor(args: argparse.Namespace) -> int:
    link_id = args.link_id if args.link_id is not None else DEFAULT_LINK_ID
    _bus, dev_tx_id, dev_rx_id = decode_link_id(link_id)
    tx_id = args.tx_id if args.tx_id is not None else dev_rx_id
    rx_id = args.rx_id if args.rx_id is not None else dev_tx_id
    callbacks = {item.strip() for item in args.auto_callback.split(",") if item.strip()}
    activation = ActivationCallbackConfig(
        sn=args.activation_sn.encode("ascii") if args.activation_sn else b"",
        activated=not args.activation_not_activated,
        timestamp_low=args.activation_ts_low,
        timestamp_high=args.activation_ts_high,
        reverse_status=args.activation_reverse_status,
        verify_status=args.activation_verify_status,
        unsupported_status=args.activation_unsupported_status,
        stage14_status=args.activation_stage14_status,
    )
    app = MonitorApp(
        MonitorConfig(
            iface=args.iface,
            tx_id=tx_id,
            rx_id=rx_id,
            local_type=args.local_type,
            local_id=args.local_id,
            remote_type=args.remote_type,
            remote_id=args.remote_id,
            show_raw_can=args.show_can,
            print_hex=args.print_hex,
            full_payload=args.full_payload,
            auto_callbacks=callbacks,
            activation=activation,
            listen_all=args.listen_all,
            respond_broadcast=not args.no_broadcast_reply,
        )
    )
    try:
        return app.run()
    finally:
        app.close()


def cmd_send(args: argparse.Namespace) -> int:
    link_id = args.link_id if args.link_id is not None else DEFAULT_LINK_ID
    _bus, dev_tx_id, dev_rx_id = decode_link_id(link_id)
    tx_id = args.tx_id if args.tx_id is not None else dev_rx_id
    rx_id = args.rx_id if args.rx_id is not None else dev_tx_id
    cmd_set, cmd_id, payload = build_named_payload(args.command, args)
    if args.seq is None:
        print("error: --seq is required for CLI send", file=sys.stderr)
        return 1
    seq = args.seq
    frame = build_request_frame(
        local_type=args.local_type,
        local_id=args.local_id,
        remote_type=args.remote_type,
        remote_id=args.remote_id,
        seq=seq,
        cmd_set=cmd_set,
        cmd_id=cmd_id,
        payload=payload,
        cmd_type=args.cmd_type,
        enc_type=args.enc_type,
    )
    can_dev = SocketCANDevice(args.iface)
    try:
        can_dev.send_stream(tx_id, frame)
        info, _errors = parse_frame(frame)
        payload_limit = None if args.full_payload else 128
        print(f"tx {describe_frame(info, payload_limit=payload_limit)}")
        if args.wait > 0:
            return wait_for_matching_response(can_dev, rx_id, seq, cmd_set, cmd_id, args.wait, payload_limit)
        return 0
    finally:
        can_dev.close()


def cmd_send_can(args: argparse.Namespace) -> int:
    can_id = parse_int(args.can_id)
    data = parse_hex_bytes(" ".join(args.data))
    can_dev = SocketCANDevice(args.iface)
    try:
        can_dev.send_can(can_id, data, extended=args.extended)
    finally:
        can_dev.close()
    print(f"tx can id=0x{can_id:X} len={len(data)} data={hex_str(data)}")
    return 0


class DumlShell:
    """Interactive REPL with three-pane TUI: passive log | command I/O | input."""

    HELP_TEXT = (
        "Commands:\n"
        "  echo [text]                  Send echo (default payload 'A')\n"
        "  get-device-info              Query device info\n"
        "  motor-speed <rpm>            Set motor speed (0-1800)\n"
        "  motor-stop                   Send motor stop\n"
        "  motor-enable [value]         Enable motor (default 1)\n"
        "  motor-disable [value]        Disable motor (default 0)\n"
        "  reboot                       Reboot ESC\n"
        "  custom <cmd_set> <cmd_id> [payload_hex]\n"
        "                               Send arbitrary DUML frame\n"
        "  raw <can_id> <hex_bytes>     Send raw CAN frame (no DUML framing)\n"
        "  seq [value]                  Show or set current seq\n"
        "  verbose [on|off]             Toggle or set verbose RX display\n"
        "  status                       Show config, seq, and RX statistics\n"
        "  clear [cmd|log|all]          Clear pane (default: cmd)\n"
        "  help                         Show this help\n"
        "  exit / quit / Ctrl-D         Exit shell\n"
        "\n"
        "RX display modes:\n"
        "  quiet (default) -- only show responses to your commands + auto-callbacks\n"
        "  verbose         -- show all incoming DUML frames in passive log\n"
    )

    SHELL_COMMANDS = [
        "echo", "get-device-info", "motor-speed", "motor-stop",
        "motor-enable", "motor-disable", "reboot", "custom", "raw",
        "seq", "verbose", "status", "clear", "help", "exit", "quit",
    ]

    _LOG_MAX_LINES = 2000

    def __init__(self, cfg: MonitorConfig) -> None:
        listen_ids = None if cfg.listen_all else [cfg.rx_id]
        self.cfg = cfg
        self.can = SocketCANDevice(cfg.iface, listen_ids=listen_ids)
        self.stream = DumlStreamParser()
        self.seq = 1
        self.verbose = False
        self._pending_seqs: set[int] = set()
        self._lock = threading.Lock()
        self._rx_total = 0
        self._rx_shown = 0
        self._rx_suppressed = 0
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._passive_area: Optional["TextArea"] = None
        self._cmdio_area: Optional["TextArea"] = None
        self._app: Optional["Application"] = None

    @staticmethod
    def _timestamp() -> str:
        stamp = time.strftime("%H:%M:%S")
        ms = int((time.time() % 1) * 1000)
        return f"{stamp}.{ms:03d}"

    @staticmethod
    def _append_area(area: "TextArea", text: str, max_lines: int = 2000) -> None:
        from prompt_toolkit.document import Document
        old = area.text
        new = (old + "\n" + text) if old else text
        lines = new.split("\n")
        if len(lines) > max_lines:
            new = "\n".join(lines[-max_lines:])
        area.buffer.set_document(Document(new, len(new)), bypass_readonly=True)

    def log_passive(self, text: str) -> None:
        line = f"[{self._timestamp()}] {text}"
        if self._passive_area is not None:
            self._append_area(self._passive_area, line, self._LOG_MAX_LINES)
        if self._app is not None:
            self._app.invalidate()

    def log_cmd(self, text: str) -> None:
        line = f"[{self._timestamp()}] {text}"
        if self._cmdio_area is not None:
            self._append_area(self._cmdio_area, line, self._LOG_MAX_LINES)
        if self._app is not None:
            self._app.invalidate()

    def _next_seq(self) -> int:
        s = self.seq & 0xFFFF
        self.seq = (self.seq + 1) & 0xFFFF
        if self.seq == 0:
            self.seq = 1
        return s

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                readable, _, _ = select.select([self.can.fileno()], [], [], 0.3)
            except (OSError, ValueError):
                break
            if not readable:
                continue
            try:
                frame_id, _dlc, data, _is_extended, is_rtr, is_err = self.can.recv()
            except (OSError, ValueError):
                break
            if is_err or is_rtr:
                continue
            if not self.cfg.listen_all and frame_id != self.cfg.rx_id:
                continue
            for duml in self.stream.feed(data):
                info, errors = parse_frame(duml)
                if errors:
                    self.log_passive(f"rx invalid {'; '.join(errors)} raw={hex_str(duml)}")
                    continue
                self._rx_total += 1
                is_response = bool(info["flags"] & 0x80)
                with self._lock:
                    is_our_response = is_response and info["seq"] in self._pending_seqs
                    if is_our_response:
                        self._pending_seqs.discard(info["seq"])
                plimit = None if self.cfg.full_payload else 64
                desc = describe_frame(info, payload_limit=plimit)
                extra = decode_payload_summary(info)
                if is_our_response:
                    self._rx_shown += 1
                    self.log_cmd(f"rx {desc}")
                    if extra:
                        self.log_cmd(f"   {extra}")
                elif self.verbose:
                    self._rx_shown += 1
                    self.log_passive(f"rx {desc}")
                    if extra:
                        self.log_passive(f"   {extra}")
                else:
                    self._rx_suppressed += 1
                self._maybe_callback(info)

    def _maybe_callback(self, info: dict) -> None:
        if info.get("is_ack") or ((info.get("flags", 0) >> 7) & 1):
            return
        is_for_us = (info["receiver_type"] == self.cfg.local_type and info["receiver_id"] == self.cfg.local_id)
        is_broadcast = self.cfg.respond_broadcast and info["receiver_id"] == 0x1F
        if not is_for_us and not is_broadcast:
            return
        name = _CMD_BY_PAIR.get((info["cmd_set"], info["cmd_id"]), ("unknown", ""))[0].replace("_", "-")
        payload: Optional[bytes] = None
        if name == "echo" and "echo" in self.cfg.auto_callbacks:
            payload = info["payload"]
        elif name == "activation" and "activation" in self.cfg.auto_callbacks:
            acfg = self.cfg.activation
            subcmd = info["payload"][0] if info["payload"] else 0xFF
            if subcmd == 0x00:
                payload = struct.pack("<H", 0x0100)
            elif subcmd == 0x11:
                payload = acfg.timestamp_payload()
            elif subcmd == 0x12:
                if len(info["payload"]) > 1 and info["payload"][1] == 1:
                    payload = bytes([acfg.verify_status & 0xFF])
                else:
                    payload = bytes([acfg.reverse_status & 0xFF])
            elif subcmd == 0x14:
                payload = bytes([acfg.stage14_status & 0xFF])
            else:
                payload = bytes([acfg.unsupported_status & 0xFF])
        elif name == "heartbeat" and "heartbeat" in self.cfg.auto_callbacks:
            if info["flags"] & 0x0F:
                payload = bytes(6)
        if payload is not None:
            resp = build_response_frame(info, payload)
            self.can.send_stream(self.cfg.tx_id, resp)
            resp_info, _ = parse_frame(resp)
            self.log_passive(f"tx {describe_frame(resp_info, payload_limit=64)} (auto-cb)")

    def _send_duml(self, cmd_set: int, cmd_id: int, payload: bytes = b"", *, seq: Optional[int] = None) -> None:
        if seq is None:
            seq = self._next_seq()
        with self._lock:
            self._pending_seqs.add(seq)
        frame = build_request_frame(
            local_type=self.cfg.local_type,
            local_id=self.cfg.local_id,
            remote_type=self.cfg.remote_type,
            remote_id=self.cfg.remote_id,
            seq=seq,
            cmd_set=cmd_set,
            cmd_id=cmd_id,
            payload=payload,
            cmd_type=DEFAULT_CMD_TYPE,
            enc_type=0,
        )
        self.can.send_stream(self.cfg.tx_id, frame)
        info, _ = parse_frame(frame)
        plimit = None if self.cfg.full_payload else 64
        self.log_cmd(f"tx {describe_frame(info, payload_limit=plimit)}")

    def _dispatch(self, line: str) -> bool:
        """Parse and execute one command line. Returns False to exit."""
        line = line.strip()
        if not line or line.startswith("#"):
            return True
        try:
            parts = shlex.split(line)
        except ValueError as e:
            self.log_cmd(f"parse error: {e}")
            return True
        cmd = parts[0].lower()
        args = parts[1:]

        try:
            if cmd in ("exit", "quit"):
                return False
            elif cmd == "help":
                self.log_cmd(self.HELP_TEXT)
            elif cmd == "echo":
                text = " ".join(args) if args else "A"
                self._send_duml(0x00, 0x00, text.encode("ascii", errors="replace"))
            elif cmd == "get-device-info":
                self._send_duml(0x00, 0x01)
            elif cmd == "motor-speed":
                if not args:
                    self.log_cmd("usage: motor-speed <rpm>")
                    return True
                speed = int(args[0], 0) & 0xFFFF
                self._send_duml(0x1E, 0x80, struct.pack("<H", speed))
            elif cmd == "motor-stop":
                self._send_duml(0x1E, 0x82)
            elif cmd == "motor-enable":
                val = int(args[0], 0) if args else 1
                self._send_duml(0x1E, 0x09, bytes([val & 0xFF]))
            elif cmd == "motor-disable":
                val = int(args[0], 0) if args else 0
                self._send_duml(0x1E, 0x0A, bytes([val & 0xFF]))
            elif cmd == "reboot":
                self._send_duml(0x00, 0x0B)
            elif cmd == "custom":
                if len(args) < 2:
                    self.log_cmd("usage: custom <cmd_set> <cmd_id> [payload_hex]")
                    return True
                cs = int(args[0], 0)
                ci = int(args[1], 0)
                pay = parse_hex_bytes(" ".join(args[2:])) if len(args) > 2 else b""
                self._send_duml(cs, ci, pay)
            elif cmd == "raw":
                if len(args) < 2:
                    self.log_cmd("usage: raw <can_id> <hex_bytes>")
                    return True
                can_id = int(args[0], 0)
                data = parse_hex_bytes(" ".join(args[1:]))
                self.can.send_can(can_id, data)
                self.log_cmd(f"tx raw can id=0x{can_id:X} len={len(data)} data={hex_str(data)}")
            elif cmd == "seq":
                if args:
                    self.seq = int(args[0], 0) & 0xFFFF
                    self.log_cmd(f"seq set to {self.seq}")
                else:
                    self.log_cmd(f"seq = {self.seq}")
            elif cmd == "verbose":
                if args:
                    self.verbose = args[0].lower() in ("on", "1", "true", "yes")
                else:
                    self.verbose = not self.verbose
                self.log_cmd(f"verbose {'on' if self.verbose else 'off'}")
            elif cmd == "clear":
                from prompt_toolkit.document import Document
                target = args[0].lower() if args else "cmd"
                if target in ("cmd", "all") and self._cmdio_area:
                    self._cmdio_area.buffer.set_document(Document("", 0), bypass_readonly=True)
                if target in ("log", "all") and self._passive_area:
                    self._passive_area.buffer.set_document(Document("", 0), bypass_readonly=True)
                if target not in ("cmd", "log", "all"):
                    self.log_cmd("usage: clear [cmd|log|all]")
            elif cmd == "status":
                self.log_cmd(
                    f"iface={self.cfg.iface} tx=0x{self.cfg.tx_id:03X} rx=0x{self.cfg.rx_id:03X} "
                    f"local={endpoint_label(self.cfg.local_type, self.cfg.local_id)} "
                    f"remote={endpoint_label(self.cfg.remote_type, self.cfg.remote_id)} "
                    f"seq={self.seq} verbose={'on' if self.verbose else 'off'} "
                    f"callbacks={','.join(sorted(self.cfg.auto_callbacks)) or 'none'} "
                    f"rx: total={self._rx_total} shown={self._rx_shown} suppressed={self._rx_suppressed}"
                )
            else:
                self.log_cmd(f"unknown command: {cmd!r} (type 'help' for list)")
        except Exception as e:
            self.log_cmd(f"error: {e}")
        return True

    def run(self) -> int:
        from prompt_toolkit import Application
        from prompt_toolkit.completion import WordCompleter
        from prompt_toolkit.history import InMemoryHistory
        from prompt_toolkit.key_binding import KeyBindings
        from prompt_toolkit.layout.containers import HSplit, Window
        from prompt_toolkit.layout.controls import FormattedTextControl
        from prompt_toolkit.layout.dimension import Dimension
        from prompt_toolkit.layout.layout import Layout
        from prompt_toolkit.styles import Style
        from prompt_toolkit.widgets import TextArea

        style = Style.from_dict({
            "separator": "bg:#333333 fg:#aaaaaa bold",
        })

        self._passive_area = TextArea(
            read_only=True,
            scrollbar=True,
            wrap_lines=False,
            focusable=True,
            style="fg:#888888",
        )
        self._cmdio_area = TextArea(
            read_only=True,
            scrollbar=True,
            wrap_lines=False,
            focusable=True,
        )
        input_area = TextArea(
            height=1,
            multiline=False,
            completer=WordCompleter(self.SHELL_COMMANDS, ignore_case=True),
            history=InMemoryHistory(),
            style="fg:#ffffff bold",
            prompt=[("fg:#e94560 bold", "duml> ")],
        )

        def on_accept(buf) -> bool:
            line = buf.text
            if not self._dispatch(line):
                if self._app is not None:
                    self._app.exit()
            return False  # do not keep text

        input_area.buffer.accept_handler = on_accept

        passive_label = Window(
            content=FormattedTextControl(
                lambda: f" Passive Log (verbose={'on' if self.verbose else 'off'}) "
                        f"| rx: total={self._rx_total} suppressed={self._rx_suppressed} "
            ),
            height=1,
            style="class:separator",
        )
        cmdio_label = Window(
            content=FormattedTextControl(
                lambda: f" Command I/O | seq={self.seq} "
                        f"| shown={self._rx_shown} "
            ),
            height=1,
            style="class:separator",
        )
        input_label = Window(
            content=FormattedTextControl(" Input (Tab=complete, Up/Down=history) "),
            height=1,
            style="class:separator",
        )

        layout = Layout(
            HSplit([
                passive_label,
                self._passive_area,
                cmdio_label,
                self._cmdio_area,
                input_label,
                input_area,
            ]),
            focused_element=input_area,
        )

        kb = KeyBindings()

        @kb.add("c-d")
        def on_ctrl_d(event):
            event.app.exit()

        @kb.add("c-c")
        def on_ctrl_c(event):
            event.app.exit()

        app: Application = Application(
            layout=layout,
            key_bindings=kb,
            style=style,
            full_screen=True,
            mouse_support=False,
        )
        self._app = app

        self.log_cmd(
            f"shell iface={self.cfg.iface} tx=0x{self.cfg.tx_id:03X} rx=0x{self.cfg.rx_id:03X} "
            f"local={endpoint_label(self.cfg.local_type, self.cfg.local_id)} "
            f"remote={endpoint_label(self.cfg.remote_type, self.cfg.remote_id)} "
            f"callbacks={','.join(sorted(self.cfg.auto_callbacks)) or 'none'}"
        )
        self.log_cmd("type 'help' for commands, Ctrl-C/Ctrl-D to exit")

        self._rx_thread.start()
        try:
            app.run()
        finally:
            self._stop.set()
            self._rx_thread.join(timeout=1.0)
            self.can.close()
        return 0


def cmd_shell(args: argparse.Namespace) -> int:
    link_id = args.link_id if args.link_id is not None else DEFAULT_LINK_ID
    _bus, dev_tx_id, dev_rx_id = decode_link_id(link_id)
    tx_id = args.tx_id if args.tx_id is not None else dev_rx_id
    rx_id = args.rx_id if args.rx_id is not None else dev_tx_id
    callbacks = {item.strip() for item in args.auto_callback.split(",") if item.strip()}
    activation = ActivationCallbackConfig(
        sn=args.activation_sn.encode("ascii") if args.activation_sn else b"",
        activated=not args.activation_not_activated,
    )
    cfg = MonitorConfig(
        iface=args.iface,
        tx_id=tx_id,
        rx_id=rx_id,
        local_type=args.local_type,
        local_id=args.local_id,
        remote_type=args.remote_type,
        remote_id=args.remote_id,
        show_raw_can=False,
        print_hex=False,
        full_payload=args.full_payload,
        auto_callbacks=callbacks,
        activation=activation,
        listen_all=args.listen_all,
        respond_broadcast=not args.no_broadcast_reply,
    )
    shell = DumlShell(cfg)
    return shell.run()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "DUMLv1 over SocketCAN for the T40 radar ESC. "
            "Reverse-engineered CAN byte-stream link: "
            "firmware side link_id 0x520518A2 => GD32 tx=0x0A3 rx=0x0A2, "
            "so host defaults to tx=0x0A2 rx=0x0A3."
        )
    )
    sub = parser.add_subparsers(dest="subcmd", required=True)

    list_cmd = sub.add_parser("list", help="List supported script commands and known DUML commands")
    list_cmd.add_argument("--verbose", action="store_true")
    list_cmd.set_defaults(func=cmd_list)

    mon = sub.add_parser("monitor", help="Monitor DUMLv1 traffic from the CAN byte stream and optionally auto-reply")
    mon.add_argument("--iface", default=DEFAULT_IFACE)
    mon.add_argument("--link-id", type=parse_int, default=DEFAULT_LINK_ID)
    mon.add_argument("--tx-id", type=parse_int, default=None)
    mon.add_argument("--rx-id", type=parse_int, default=None)
    mon.add_argument("--local-type", type=parse_int, default=DEFAULT_LOCAL_TYPE)
    mon.add_argument("--local-id", type=parse_int, default=DEFAULT_LOCAL_ID)
    mon.add_argument("--remote-type", type=parse_int, default=DEFAULT_REMOTE_TYPE)
    mon.add_argument("--remote-id", type=parse_int, default=DEFAULT_REMOTE_ID)
    mon.add_argument("--auto-callback", default="activation", help="Comma-separated: activation,echo,heartbeat")
    mon.add_argument("--activation-sn", default="")
    mon.add_argument("--activation-ts-low", type=parse_int, default=0)
    mon.add_argument("--activation-ts-high", type=parse_int, default=0)
    mon.add_argument("--activation-reverse-status", type=parse_int, default=0)
    mon.add_argument("--activation-verify-status", type=parse_int, default=0)
    mon.add_argument("--activation-unsupported-status", type=parse_int, default=0xE0)
    mon.add_argument("--activation-stage14-status", type=parse_int, default=0xE1)
    mon.add_argument("--activation-not-activated", action="store_true")
    mon.add_argument("--show-can", action="store_true", help="Also print raw CAN frames")
    mon.add_argument("--print-hex", action="store_true", help="Print longer payload hex in DUML summaries")
    mon.add_argument("--full-payload", action="store_true", help="Do not truncate DUML payload hex")
    mon.add_argument("--listen-all", action="store_true", help="Do not install a CAN id filter")
    mon.add_argument("--no-broadcast-reply", action="store_true")
    mon.set_defaults(func=cmd_monitor)

    send = sub.add_parser("send", help="Send one DUMLv1 command over the CAN byte stream")
    send.add_argument("command", help="Known command name or custom")
    send.add_argument("--iface", default=DEFAULT_IFACE)
    send.add_argument("--link-id", type=parse_int, default=DEFAULT_LINK_ID)
    send.add_argument("--tx-id", type=parse_int, default=None)
    send.add_argument("--rx-id", type=parse_int, default=None)
    send.add_argument("--local-type", type=parse_int, default=DEFAULT_LOCAL_TYPE)
    send.add_argument("--local-id", type=parse_int, default=DEFAULT_LOCAL_ID)
    send.add_argument("--remote-type", type=parse_int, default=DEFAULT_REMOTE_TYPE)
    send.add_argument("--remote-id", type=parse_int, default=DEFAULT_REMOTE_ID)
    send.add_argument("--seq", type=parse_int, default=None, help="Sequence number (required)")
    send.add_argument("--cmd-type", type=parse_int, default=DEFAULT_CMD_TYPE, help="0..3, default 2 => request with ACK")
    send.add_argument("--enc-type", type=parse_int, default=0)
    send.add_argument("--wait", type=float, default=0.0, help="Wait N seconds for the matching DUML response")
    send.add_argument("--full-payload", action="store_true", help="Do not truncate DUML payload hex when printing tx/rx")
    send.add_argument("--payload", default="", help="Hex payload for generic/custom commands")
    send.add_argument("--cmd-set", type=parse_int, default=None)
    send.add_argument("--cmd-id", type=parse_int, default=None)
    send.add_argument("--text", default=None, help="ASCII payload for echo")
    send.add_argument("--subcmd", type=parse_int, default=0, help="Activation sub-command")
    send.add_argument("--mode", type=parse_int, default=None, help="Activation mode byte for subcmd 0x12")
    send.add_argument("--sn", default=None, help="Activation SN string for subcmd 0x12")
    send.add_argument("--value", type=parse_int, default=None, help="Payload byte for enable/disable; defaults: enable=1 disable=0")
    send.add_argument("--speed", type=parse_int, default=0, help="Motor speed setpoint")
    send.add_argument("--angle", type=int, default=0, help="Motor position angle_ref")
    send.add_argument("--grating", type=parse_int, default=0, help="Motor position grat_num")
    send.add_argument("--value2", type=parse_int, default=0, help="Second u32 value for motor-pwm")
    send.set_defaults(func=cmd_send)

    send_can = sub.add_parser("send-can", help="Send one raw CAN frame")
    send_can.add_argument("can_id")
    send_can.add_argument("data", nargs="*")
    send_can.add_argument("--iface", default=DEFAULT_IFACE)
    send_can.add_argument("--extended", action="store_true")
    send_can.set_defaults(func=cmd_send_can)

    sh = sub.add_parser("shell", help="Interactive DUML shell with auto-incrementing seq")
    sh.add_argument("--iface", default=DEFAULT_IFACE)
    sh.add_argument("--link-id", type=parse_int, default=DEFAULT_LINK_ID)
    sh.add_argument("--tx-id", type=parse_int, default=None)
    sh.add_argument("--rx-id", type=parse_int, default=None)
    sh.add_argument("--local-type", type=parse_int, default=DEFAULT_LOCAL_TYPE)
    sh.add_argument("--local-id", type=parse_int, default=DEFAULT_LOCAL_ID)
    sh.add_argument("--remote-type", type=parse_int, default=DEFAULT_REMOTE_TYPE)
    sh.add_argument("--remote-id", type=parse_int, default=DEFAULT_REMOTE_ID)
    sh.add_argument("--auto-callback", default="activation", help="Comma-separated: activation,echo,heartbeat")
    sh.add_argument("--activation-sn", default="")
    sh.add_argument("--activation-not-activated", action="store_true")
    sh.add_argument("--full-payload", action="store_true")
    sh.add_argument("--listen-all", action="store_true")
    sh.add_argument("--no-broadcast-reply", action="store_true")
    sh.set_defaults(func=cmd_shell)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.subcmd == "send" and args.command == "custom":
        if args.cmd_set is None or args.cmd_id is None:
            parser.error("send custom requires --cmd-set and --cmd-id")
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
