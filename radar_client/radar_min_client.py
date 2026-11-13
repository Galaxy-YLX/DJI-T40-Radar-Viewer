#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter, deque
import json
import math
import numpy as np
import select
import socket
import struct
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional

DJI_CRC8_INIT = 0x77
DJI_CRC16_INIT = 0x3692
V1_HEADER_LEN = 11
V1_TAIL_LEN = 2
V2_HEADER_CRC_OFFSET = 23
V2_MSG_OFFSET = 24
V2_PAYLOAD_OFFSET = 27
V2_MIN_FRAME_LEN = 29
PERCEPTION_1E61_HEADER_LEN = 69
PERCEPTION_1E61_ENTRY_LEN = 16

MSG_NAMES = {
    0x03AF: "v1_03af_unknown",
    0x03D7: "v1_03d7_unknown",
    0x0E01: "v1_0e01_unknown",
    0x1E09: "motor_enable",
    0x1E0A: "motor_disable",
    0x1E0B: "radar_imu_inter_pack",
    0x1E0F: "motor_bottom_data_pack",  # ESC→app periodic feedback (44B)
    0x1E21: "flight_pos",
    0x1E22: "flight_ctrl",
    0x1E25: "flight_debug",
    0x1E26: "sample_ctrl",
    0x1E27: "time_sync",
    0x1E32: "avoidance_result_alt",
    0x1E61: "perception_v2",
    0x1E62: "avoidance_result_fc",
    0x1E80: "motor_speed",           # app→ESC speed setpoint (2B)
    0x1E81: "motor_source_pos",      # app→ESC fix-mode source/position (6B)
    0x1E82: "motor_stop",            # app→ESC stop (0B)
    0x1E83: "hms_switch",            # app→ESC start/stop (1B), ACK unlocks unk_282DEC
    0x1EF9: "motor_state_f9",        # app→ESC periodic state-2 toggle (1B)
    0x1EFC: "req_or_terminate_data",
    0x1EFD: "data_transfer_reply",
    0x1EFE: "motor_state_query",     # app↔ESC internal-state get
    0xF000: "blackbox_v2",
    0x2243: "v1_2243_unknown",
}

# Motor/ESC-oriented cmd_ids under cmd_set=0x1E that travel through the
# FC→CAN bridge path on a real aircraft. On bench the radar has no CAN, so
# we either transparently forward these to a connected ESC via SocketCAN
# (--can-bridge), replay a captured stream (--replay-esc), or simulate
# the ESC side in software (--sim-esc).
MOTOR_CMD_IDS = {
    0x09: "motor_enable",
    0x0A: "motor_disable",
    0x0F: "bottom_data_pack",
    0x80: "motor_speed",
    0x81: "motor_source_pos",
    0x82: "motor_stop",
    0x83: "hms_switch",
    0xF9: "motor_state_f9",
    0xFE: "motor_state_query",
}

PERCEPTION_SUBTYPE_NAMES = {
    0: "ground_points",
    1: "wire_candidates",
    2: "pcla_obstacle_candidates",
}

PERCEPTION_DDS_HINTS = {
    0: "/device/payload/2/data/radar_data_ground (tentative)",
    2: "/device/payload/2/data/radar_data_obstacle (tentative)",
}


def dji_crc8(data: bytes, init: int = DJI_CRC8_INIT) -> int:
    crc = init & 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ 0x8C) & 0xFF
            else:
                crc = (crc >> 1) & 0xFF
    return crc


def dji_crc16(data: bytes, init: int = DJI_CRC16_INIT) -> int:
    crc = init & 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = ((crc >> 1) ^ 0x8408) & 0xFFFF
            else:
                crc = (crc >> 1) & 0xFFFF
    return crc


@dataclass
class V1Frame:
    total_len: int
    sender: int
    receiver: int
    seq: int
    flags: int
    msg_id: int
    payload: bytes


@dataclass
class V2Frame:
    total_len: int
    seq: int
    sender: int
    receiver: int
    msg_id: int
    payload: bytes


@dataclass
class ReplayEvent:
    """One ESC→app DUML frame sourced from a captured duml.jsonl."""
    rel_ts: float          # seconds relative to the first CAN_TO_UDP frame
    frame: bytes           # full DUML bytes ready to sendto app.elf
    cmd_set: int
    cmd_id: int


def load_replay_events(jsonl_path: str, only_motor: bool = False) -> list[ReplayEvent]:
    """Read a captured can/duml.jsonl, return ESC→app (CAN_TO_UDP) events
    with timestamps normalized so the first event is at t=0. Invalid or
    unparseable records are silently skipped."""
    events: list[ReplayEvent] = []
    path = Path(jsonl_path)
    with path.open() as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            if r.get("dir") != "CAN_TO_UDP" or r.get("parse_error"):
                continue
            try:
                frame = bytes.fromhex(r["frame_hex"])
                cs = int(r.get("cmd_set", "0x00"), 0)
                ci = int(r.get("cmd_id", "0x00"), 0)
            except (KeyError, ValueError):
                continue
            if only_motor and cs != 0x1E:
                continue
            events.append(ReplayEvent(
                rel_ts=r["ts"], frame=frame, cmd_set=cs, cmd_id=ci,
            ))
    if not events:
        return events
    events.sort(key=lambda e: e.rel_ts)
    t0 = events[0].rel_ts
    for e in events:
        e.rel_ts -= t0
    return events


@dataclass
class PeriodicTask:
    name: str
    interval: float
    builder: Callable[[], bytes]
    msg_id: int
    next_at: float = 0.0
    sender: Optional[int] = None     # None → use ClientConfig.sender
    receiver: Optional[int] = None   # None → use ClientConfig.receiver
    flags: Optional[int] = None      # None → use ClientConfig.flags


@dataclass
class ClientConfig:
    local_ip: str = "192.168.8.1"
    radar_ip: str = "192.168.8.2"
    control_local_port: int = 50007
    control_remote_port: int = 50007
    v2_local_port: int = 40204
    v2_listen: bool = True        # when False, do not bind the V2 rx socket
                                  # (lets a co-located viewer own port 40204)
    sender: int = 0x03
    receiver: int = 0x00
    flags: int = 0x00
    capture_dir: Optional[str] = None
    plot: bool = False
    plot_range_m: float = 25.0
    plot_history_sec: float = 3.0
    plot_refresh_hz: float = 10.0
    plot_wire_window_sec: float = 1.0
    plot_ground_window_sec: float = 2.0
    plot_terrain: str = "generic"      # generic | ground | mountain | fruiter
    pos_hz: float = 20.0
    debug_hz: float = 2.0
    time_hz: float = 10.0
    dump_entries: int = 3
    send_motor_start: bool = False
    motor_start_subid: int = 1
    motor_run_rpm: int = 1200      # rotor speed setpoint sent over CAN while running
    motor_stop_rpm: int = 50       # rotor speed setpoint sent at shutdown (idle)
    motor_speed_hz: float = 5.0    # how often the running setpoint is re-sent
    send_sample_start: bool = False
    send_sample_mark: bool = False
    send_data_start_type: Optional[int] = None
    send_data_end_type: Optional[int] = None
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    pos_x_m: float = 0.0
    pos_y_m: float = 0.0
    pos_z_m: float = 0.0
    vel_x_mps: float = 0.0
    vel_y_mps: float = 0.0
    vel_z_mps: float = 0.0
    aux_x: float = 0.0
    aux_y: float = 0.0
    aux_z: float = 0.0
    flight_mode: int = 2
    # Seconds to send flight_mode=1 before switching to flight_mode (above).
    # Reverse-engineered requirement: pm state machine in sub_65F10 only
    # lets the radar switch into pm3 on the 1→2 transition. 0 disables the
    # ramp (for backwards compatibility).
    fm_ramp_sec: float = 3.0
    flight_flags: int = 0
    gps_valid: int = 1
    pose_tag: int = 0
    sim_axis: Optional[str] = None
    sim_amplitude_m: float = 0.0
    sim_period_sec: float = 6.0
    flight_debug_hex: Optional[str] = None
    rti_history: int = 200
    rti_mode: bool = False
    # ── FC-bridge / ESC-sim extension ───────────────────────────────────
    # app.elf emits motor DUML (cmd_set=0x1E) expecting the FC to relay to
    # ESC over CAN. On bench we either bridge to a real SocketCAN ESC
    # (--can-bridge), or simulate ESC in software (--sim-esc).
    can_bridge: Optional[str] = None
    can_tx_id: int = 0x0A2          # our tx (ESC rx) — decode_link_id(0x520518A2)
    can_rx_id: int = 0x0A3          # our rx (ESC tx)
    sim_esc: bool = False
    sim_esc_fix_ok: bool = True     # bottom_data_pack byte[28]: 1 = fix OK, 0 = fault
    # --replay-esc: replay a captured can/duml.jsonl back at app.elf.
    replay_esc: Optional[str] = None
    replay_esc_loop: bool = True
    replay_esc_speed: float = 1.0
    replay_esc_only_motor: bool = False


def build_v1_frame(msg_id: int, payload: bytes, seq: int, sender: int, receiver: int, flags: int) -> bytes:
    total_len = V1_HEADER_LEN + len(payload) + V1_TAIL_LEN
    frame = bytearray(total_len)
    frame[0] = 0x55
    frame[1] = total_len & 0xFF
    frame[2] = 0x04 | ((total_len >> 8) & 0x03)
    frame[3] = dji_crc8(frame[:3])
    frame[4] = sender & 0xFF
    frame[5] = receiver & 0xFF
    struct.pack_into("<H", frame, 6, seq & 0xFFFF)
    frame[8] = flags & 0xFF
    frame[9] = (msg_id >> 8) & 0xFF
    frame[10] = msg_id & 0xFF
    frame[11:-2] = payload
    struct.pack_into("<H", frame, total_len - 2, dji_crc16(frame[:-2]))
    return bytes(frame)


def parse_v1_frame(data: bytes) -> Optional[V1Frame]:
    if len(data) < V1_HEADER_LEN + V1_TAIL_LEN:
        return None
    if data[0] != 0x55 or (data[2] & 0xFC) != 0x04:
        return None
    total_len = data[1] | ((data[2] & 0x03) << 8)
    if total_len != len(data):
        return None
    if dji_crc8(data[:3]) != data[3]:
        return None
    recv_crc16 = struct.unpack_from("<H", data, total_len - 2)[0]
    if dji_crc16(data[:-2]) != recv_crc16:
        return None
    return V1Frame(
        total_len=total_len,
        sender=data[4],
        receiver=data[5],
        seq=struct.unpack_from("<H", data, 6)[0],
        flags=data[8],
        msg_id=(data[9] << 8) | data[10],
        payload=data[11:-2],
    )


def parse_v2_frame(data: bytes) -> Optional[V2Frame]:
    if len(data) < V2_MIN_FRAME_LEN:
        return None
    if data[0] != 0x55:
        return None
    if (data[1] & 0x3F) != 0x18:
        return None
    if (data[2] & 0xFC) != 0x08:
        return None
    total_len = struct.unpack_from("<H", data, 3)[0]
    if total_len != len(data):
        return None
    if dji_crc8(data[:V2_HEADER_CRC_OFFSET]) != data[V2_HEADER_CRC_OFFSET]:
        return None
    recv_crc16 = struct.unpack_from("<H", data, total_len - 2)[0]
    if dji_crc16(data[:-2]) != recv_crc16:
        return None
    return V2Frame(
        total_len=total_len,
        seq=struct.unpack_from("<I", data, 7)[0],
        sender=struct.unpack_from("<H", data, 12)[0],
        receiver=struct.unpack_from("<H", data, 18)[0],
        msg_id=(data[V2_MSG_OFFSET] << 8) | data[V2_MSG_OFFSET + 1],
        payload=data[V2_PAYLOAD_OFFSET:-2],
    )


def encode_scaled_i16(value: float, scale: float) -> int:
    raw = int(round(value / scale))
    return max(-32768, min(32767, raw))


def build_flight_debug_payload() -> bytes:
    payload = bytearray(38)
    # [0-27]: 7 dwords — flight debug state values (leave zero)
    # [28]: byte
    # [29]: byte
    # [30]: byte
    # [31]: bitfield byte:
    #   b0 = byte_283227/283236
    #   b1 = g_pitch_correction_enable
    #   b2 = byte_283229
    #   b3 = byte_28322A
    #   b4 = byte_28322B
    #   b5-7 = byte_28322C
    # [32]: bitfield byte:
    #   b0-2 = byte_28322D
    #   b3 = byte_28322E
    #   b4 = byte_28322F
    #   b5-6 = byte_283230
    #   b7 = partial byte_283231
    # [33]: b0 = partial byte_283231
    # [35-36]: uint16 word_283232
    # [37]: byte_283234
    # Set g_flight_debug_received=1 on radar side (just by sending this msg)
    # Enable pitch correction: set bit1 of byte[31]
    payload[31] = 0x02
    return bytes(payload)


def parse_fixed_hex_payload(spec: str, expected_len: int) -> bytes:
    text = spec.replace(" ", "").replace(":", "").replace("-", "")
    if text.startswith("0x"):
        text = text[2:]
    data = bytes.fromhex(text)
    if len(data) != expected_len:
        raise ValueError(f"expected {expected_len} bytes, got {len(data)}")
    return data


def build_time_sync_payload(counter: int) -> bytes:
    payload = bytearray(82)
    struct.pack_into("<I", payload, 0, counter & 0xFFFFFFFF)
    payload[4] = 1
    payload[5] = 0
    struct.pack_into("<H", payload, 6, 0)
    return bytes(payload)


def build_flight_ctrl_payload(sub_id: int, start_motor: bool) -> bytes:
    payload = bytearray(7)
    payload[0] = sub_id & 0xFF
    payload[3] = 1 if start_motor else 0
    return bytes(payload)


def build_sample_ctrl_payload(sample_type: int) -> bytes:
    payload = bytearray(18)
    payload[0] = sample_type & 0xFF
    return bytes(payload)


def build_req_or_terminate_data_payload(data_flag: int, data_type: int) -> bytes:
    payload = bytearray(7)
    payload[0] = data_flag & 0xFF
    payload[1] = data_type & 0xFF
    return bytes(payload)


# ── ESC simulator templates ────────────────────────────────────────────
# These byte templates are byte-for-byte what a real ESC was observed to
# emit in cap1/radar_capture_20260417_005657/can/duml.jsonl (241s run,
# CAN-attached DJI T40 ESC). Using exact wire templates instead of
# handcrafted values is what gets app.elf to accept us as a healthy ESC.

# cmd_set=0x1E cmd_id=0x0F bottom_data_pack (44B).
# Key non-zero bytes observed constant across the whole recording:
#   [21..24] = 3b 03 59 2c    (unknown state/counter word)
#   [25]     = 1f             (status flag)
#   [26..27] = 00 00          (fix-mode angle, always 0 during running mode)
#   [28]     = 01             (fix-mode OK)
#   [29..32] = 3d 00 29 02    (unknown state block)
# Every other byte was zero across all 123 captured samples.
SIM_ESC_BOTTOM_DATA_PACK_HEX = (
    "00" * 21                     # bytes 0..20 (zero)
    + "3b03592c1f0000013d002902"  # bytes 21..32 (state block)
    + "00" * 11                   # bytes 33..43 (zero)
)
assert len(bytes.fromhex(SIM_ESC_BOTTOM_DATA_PACK_HEX)) == 44

# cmd_set=0x1E cmd_id=0x0B radar_imu_inter_pack (26B).
# Template observed: byte[0]=0x01 flag, bytes[1..13]=0, bytes[14..25] = 3
# float32 values. We set them all to 0.0 (bits 00 00 00 00) — represents
# a stationary aircraft, which is physically consistent with bench conditions.
SIM_ESC_IMU_INTER_PACK_HEX = "01" + "00" * 25
assert len(bytes.fromhex(SIM_ESC_IMU_INTER_PACK_HEX)) == 26

# cmd_set=0x1E cmd_id=0xEA keepalive (2B). 0xaaaa pattern.
SIM_ESC_KEEPALIVE_EA_HEX = "aaaa"

# cmd_set=0x00 cmd_id=0x32 activation reply (18B). ASCII SN "5F2BK5E10201EZ"
# sandwiched between small preamble/postamble.
SIM_ESC_ACTIVATION_HEX = "1310354632424b35453130323031455a0000"
assert len(bytes.fromhex(SIM_ESC_ACTIVATION_HEX)) == 18

# cmd_set=0x0E cmd_id=0x01 diagnostic (1B = 0x07).
SIM_ESC_DIAG_0E01_HEX = "07"


def build_bottom_data_pack_payload(angle_i16: int, fix_mode_ok: bool = True,
                                   status_byte25: int = 0) -> bytes:
    """Build the 44-byte cmd_internal_state_set payload that ESC pushes to
    app.elf (cmd_set=0x1E, cmd_id=0x0F).

    Reverse-engineered from sub_5FBE0 (cmd_internal_state_set_handler) and
    sub_3DC5C (get_bottom_data_pack_scaled_angle):
      - Handler requires payload size == exactly 44 bytes.
      - byte[25] is stored separately in unk_CD71F98 (status flag).
      - bytes[26..27] are int16 little-endian, scaled to degrees by
        (val + 0x8000) / 0x10000 × (264 or 268) — i.e. full int16 range
        maps to ~one motor revolution.
      - byte[28] appears to be the fix-mode OK status (=2 when ESC is
        holding position).
      - Other bytes are copied into unk_CD71FC0..FEB but not read by the
        code paths we've traced — zero-fill is safe.
    """
    payload = bytearray(44)
    payload[25] = status_byte25 & 0xFF
    struct.pack_into("<h", payload, 26,
                     max(-32768, min(32767, angle_i16)))
    payload[28] = 2 if fix_mode_ok else 0
    return bytes(payload)


def build_hms_switch_ack_payload(ack_ret: int = 0, value: int = 1) -> bytes:
    """Build the 2-byte payload that hms_switch_ack_handler (sub_3EA50)
    expects: [ack_ret, value]. ack_ret=0 & value!=0 → configure_motor_runtime
    writes unk_282DEC=2 (unblocks motor FSM state 2)."""
    return bytes([ack_ret & 0xFF, value & 0xFF])


@dataclass
class DataTransferState:
    """Reassembly state for 0x1EFD data transfer frames."""
    current_frame_index: int = -1
    expected_pkg_num: int = 0
    received_pkgs: int = 0
    data_buf: bytearray = field(default_factory=bytearray)
    chirp_count: int = 0
    col_count: int = 0
    row_count: int = 0
    frame_kind: str = "unknown"

    def reset(self, frame_index: int = -1) -> None:
        self.current_frame_index = frame_index
        self.expected_pkg_num = 0
        self.received_pkgs = 0
        self.data_buf = bytearray()
        self.chirp_count = 0
        self.col_count = 0
        self.row_count = 0
        self.frame_kind = "unknown"


def parse_data_transfer_packet(payload: bytes) -> Optional[dict[str, Any]]:
    """Parse a 0x1EFD data transfer reply payload.

    Normal data packet (781 bytes payload after V1 header strip):
      [0:2]  frame_index  uint16_le
      [2:4]  pkg_index    uint16_le
      [4]    block_shift / 0xFF=final marker
      [5]    chirp_count
      [6]    col_count (for normal) / flag (for final)
      [7]    row_count (for normal) / crc32 start (for final)
      [8:10] data_valid_num uint16_le (for normal)
      [10:12] pkg_num uint16_le (for normal)
      [12:]  data (up to 768 bytes)

    Final packet (shorter, block_shift==0xFF):
      [7:11] frame_crc32 uint32_le
      [11:13] pkg_num uint16_le
    """
    if len(payload) < 13:
        return None
    frame_index = struct.unpack_from("<H", payload, 0)[0]
    pkg_index = struct.unpack_from("<H", payload, 2)[0]
    block_shift = payload[4]

    if block_shift == 0xFF:
        # Final packet
        chirp_count = payload[5]
        flag = payload[6]
        frame_crc32 = struct.unpack_from("<I", payload, 7)[0] if len(payload) >= 11 else 0
        pkg_num = struct.unpack_from("<H", payload, 11)[0] if len(payload) >= 13 else 0
        return {
            "type": "final",
            "frame_index": frame_index,
            "pkg_index": pkg_index,
            "chirp_count": chirp_count,
            "flag": flag,
            "frame_crc32": frame_crc32,
            "pkg_num": pkg_num,
        }
    else:
        # Normal data packet
        chirp_count = payload[5]
        col_count = payload[6]
        row_count = payload[7]
        data_valid_num = struct.unpack_from("<H", payload, 8)[0]
        pkg_num = struct.unpack_from("<H", payload, 10)[0]
        data = payload[12:]
        return {
            "type": "data",
            "frame_index": frame_index,
            "pkg_index": pkg_index,
            "block_shift": block_shift,
            "chirp_count": chirp_count,
            "col_count": col_count,
            "row_count": row_count,
            "data_valid_num": data_valid_num,
            "pkg_num": pkg_num,
            "data": data,
        }


def classify_data_transfer_frame(chirp_count: int, col_count: int, row_count: int, total_bytes: int) -> str:
    if chirp_count == 1 and col_count == 1 and row_count == 1 and total_bytes in (1024, 2048, 4096):
        return "profile256_i16_shifted_u32"
    if chirp_count == 0 and col_count == 1 and row_count == 4 and total_bytes >= 4096:
        return "adc_like_4x1024_u12_planes"
    return "unknown"


def summarize_data_transfer_frame(
    frame_kind: str,
    data: bytes,
    chirp_count: int,
    col_count: int,
    row_count: int,
) -> tuple[str, dict[str, Any]]:
    meta: dict[str, Any] = {
        "frame_kind": frame_kind,
        "total_bytes": len(data),
        "chirp_count": chirp_count,
        "col_count": col_count,
        "row_count": row_count,
    }
    if frame_kind == "profile256_i16_shifted_u32":
        usable = min(len(data) // 4, 256)
        values = np.frombuffer(data[: usable * 4], dtype="<u4").astype(np.int64) if usable else np.array([], dtype=np.int64)
        meta["u32_count"] = int(usable)
        if usable:
            meta["min"] = int(values.min())
            meta["max"] = int(values.max())
            meta["first16"] = values[:16].tolist()
            summary = (
                f"[1EFD] profile256 frame: {len(data)} bytes, {usable} u32, "
                f"min={meta['min']} max={meta['max']} first4={values[:4].tolist()}"
            )
        else:
            summary = f"[1EFD] profile256 frame: {len(data)} bytes, empty payload"
        return summary, meta
    if frame_kind == "adc_like_4x1024_u12_planes":
        usable = len(data) // 4
        values = np.frombuffer(data[: usable * 4], dtype="<u4").astype(np.int64) if usable else np.array([], dtype=np.int64)
        meta["u32_count"] = int(usable)
        if usable:
            meta["min"] = int(values.min())
            meta["max"] = int(values.max())
            meta["first16"] = values[:16].tolist()
            summary = (
                f"[1EFD] adc-like frame: {len(data)} bytes, {usable} u32, "
                f"min={meta['min']} max={meta['max']} first4={values[:4].tolist()}"
            )
        else:
            summary = f"[1EFD] adc-like frame: {len(data)} bytes, empty payload"
        return summary, meta
    preview = data[:32].hex(" ")
    meta["preview_hex"] = preview
    summary = (
        f"[1EFD] unknown frame: {len(data)} bytes, "
        f"shape=({chirp_count},{col_count},{row_count}) preview={preview}"
    )
    return summary, meta


def hexdump(data: bytes, width: int = 16) -> str:
    rows = []
    for offset in range(0, len(data), width):
        chunk = data[offset:offset + width]
        rows.append(f"{offset:04x}: {chunk.hex(' ')}")
    return "\n".join(rows)


def safe_label(text: str) -> str:
    return "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in text)


def perception_subtype_name(subtype: int) -> str:
    return PERCEPTION_SUBTYPE_NAMES.get(subtype, f"unknown_subtype_{subtype}")


def decode_perception_1e61_entries(payload: bytes) -> tuple[int, list[dict[str, Any]]]:
    if len(payload) < PERCEPTION_1E61_HEADER_LEN:
        raise ValueError(f"payload too short: {len(payload)}")
    body_len = len(payload) - PERCEPTION_1E61_HEADER_LEN
    if body_len % PERCEPTION_1E61_ENTRY_LEN != 0:
        raise ValueError(f"unexpected body length: {body_len}")
    subtype = payload[65]
    count = body_len // PERCEPTION_1E61_ENTRY_LEN
    body = payload[PERCEPTION_1E61_HEADER_LEN:]
    entries: list[dict[str, Any]] = []
    for index in range(count):
        entry = body[index * PERCEPTION_1E61_ENTRY_LEN:(index + 1) * PERCEPTION_1E61_ENTRY_LEN]
        if subtype == 0:
            # ground_points: same layout as wire — uint16 id, 3×float xyz, uint16 metric
            item_id, x_m, y_m, z_m, metric = struct.unpack_from("<HfffH", entry, 0)
            entries.append(
                {
                    "index": index,
                    "id": item_id,
                    "x_m": x_m,
                    "y_m": y_m,
                    "z_m": z_m,
                    "metric": metric,
                }
            )
        elif subtype == 1:
            item_id, x_m, y_m, z_m, metric = struct.unpack_from("<HfffH", entry, 0)
            entries.append(
                {
                    "index": index,
                    "id": item_id,
                    "x_m": x_m,
                    "y_m": y_m,
                    "z_m": z_m,
                    "metric": metric,
                }
            )
        elif subtype == 2:
            # pcla entries share the ground/wire float xyz layout; earlier
            # revisions mis-decoded this as int32 and produced bit-pattern
            # numbers like -1071113679 which are the IEEE-754 bits of
            # values around ±2..6 m. Confirmed by reinterpreting captured
            # cap1 entries: int32 -1071113679 → float -2.627 m (xy range).
            item_id, x_m, y_m, z_m, metric = struct.unpack_from("<HfffH", entry, 0)
            entries.append(
                {
                    "index": index,
                    "id": item_id,
                    "x_m": x_m,
                    "y_m": y_m,
                    "z_m": z_m,
                    "metric": metric,
                }
            )
        else:
            entries.append({"index": index, "raw_hex": entry.hex()})
    return subtype, entries


def build_perception_csv(subtype: int, entries: list[dict[str, Any]]) -> str:
    if subtype in (0, 1, 2):
        lines = ["index,id,x_m,y_m,z_m,metric"]
        for item in entries:
            lines.append(
                f"{item['index']},{item['id']},{item['x_m']:.6f},"
                f"{item['y_m']:.6f},{item['z_m']:.6f},{item['metric']}"
            )
        return "\n".join(lines) + "\n"
    lines = ["index,raw_hex"]
    for item in entries:
        lines.append(f"{item['index']},{item['raw_hex']}")
    return "\n".join(lines) + "\n"


def format_perception_1e61(subtype: int, entries: list[dict[str, Any]], payload_len: int, dump_entries: int) -> str:
    subtype_name = perception_subtype_name(subtype)
    lines = [f"subtype={subtype}({subtype_name}) count={len(entries)} payload_len={payload_len}"]
    dds_hint = PERCEPTION_DDS_HINTS.get(subtype)
    if dds_hint is not None:
        lines.append(f"dds_hint={dds_hint}")
    limit = min(len(entries), dump_entries)
    for index in range(limit):
        if subtype == 0:
            lines.append(
                f"  [{index}] gnd id={entries[index]['id']} x={entries[index]['x_m']:.3f} "
                f"y={entries[index]['y_m']:.3f} z={entries[index]['z_m']:.3f} metric={entries[index]['metric']}"
            )
        elif subtype == 1:
            lines.append(
                f"  [{index}] wire id={entries[index]['id']} x={entries[index]['x_m']:.3f} "
                f"y={entries[index]['y_m']:.3f} z={entries[index]['z_m']:.3f} metric={entries[index]['metric']}"
            )
        elif subtype == 2:
            lines.append(
                f"  [{index}] pcla id={entries[index]['id']} x={entries[index]['x_m']:.3f} "
                f"y={entries[index]['y_m']:.3f} z={entries[index]['z_m']:.3f} metric={entries[index]['metric']}"
            )
        else:
            lines.append(f"  [{index}] raw={entries[index]['raw_hex']}")
    return "\n".join(lines)


def parse_perception_1e61(payload: bytes, dump_entries: int) -> str:
    try:
        subtype, entries = decode_perception_1e61_entries(payload)
    except ValueError as exc:
        return str(exc)
    return format_perception_1e61(subtype, entries, len(payload), dump_entries)


def parse_blackbox_f000(payload: bytes) -> str:
    trailer = ""
    if len(payload) >= 4:
        trailer = f" last_u32_le=0x{struct.unpack_from('<I', payload, len(payload) - 4)[0]:08x}"
    return f"payload_len={len(payload)}{trailer} note=raw_blackbox_chunk"


@dataclass
class AvoidSectorEntry:
    index: int
    azimuth_cdeg: int
    elevation_cdeg: int
    distance_cm: int

    @property
    def azimuth_deg(self) -> float:
        return self.azimuth_cdeg / 100.0

    @property
    def elevation_deg(self) -> float:
        return self.elevation_cdeg / 100.0

    @property
    def distance_m(self) -> float:
        return self.distance_cm / 100.0

    @property
    def is_default(self) -> bool:
        return self.azimuth_cdeg == 9000 and self.elevation_cdeg == 0 and self.distance_cm == 10000


@dataclass
class AvoidanceResult:
    type_ver: tuple[int, int]
    status_flags: int
    btm_avoid_dis_cm: int
    up_avoid_dis_cm: int
    sectors: list[AvoidSectorEntry]
    tail: bytes
    # Decoded from tail: height fusion values (Kalman filter output)
    height_kalman_cm: int = 0       # tail[1:3] — flt_CACBDB0 * 100, checked against 1.6m gate
    height_filtered_cm: int = 0     # tail[3:5] — secondary filtered height
    height_confidence: int = 0      # tail[5:7] — validity/confidence

    @property
    def btm_avoid_m(self) -> float:
        return self.btm_avoid_dis_cm / 100.0

    @property
    def up_avoid_m(self) -> float:
        return self.up_avoid_dis_cm / 100.0

    @property
    def height_kalman_m(self) -> float:
        return self.height_kalman_cm / 100.0


def decode_avoidance_result(payload: bytes) -> AvoidanceResult:
    if len(payload) != 116:
        raise ValueError(f"unexpected avoidance payload len: {len(payload)}")
    type_byte = payload[0]
    ver_byte = payload[1]
    status_flags = struct.unpack_from("<H", payload, 2)[0]
    btm_avoid = struct.unpack_from("<H", payload, 4)[0]
    up_avoid = struct.unpack_from("<H", payload, 6)[0]
    sectors: list[AvoidSectorEntry] = []
    for i in range(16):
        base = 8 + i * 6
        entry = payload[base:base + 6]
        dist_lo = entry[5]
        dist_hi = entry[0] if i > 0 else 0
        distance_cm = (dist_hi << 8) | dist_lo
        if i == 0:
            distance_cm = struct.unpack_from("<H", entry, 4)[0]
        else:
            distance_cm = (entry[0] << 8) | entry[5]
        azimuth = struct.unpack_from("<H", entry, 1)[0]
        elevation = struct.unpack_from("<H", entry, 3)[0]
        sectors.append(AvoidSectorEntry(
            index=i,
            azimuth_cdeg=azimuth,
            elevation_cdeg=elevation,
            distance_cm=distance_cm,
        ))
    tail = payload[104:116]
    # Tail layout: [0]=counter, [1:3]=height_kalman_cm, [3:5]=height_filtered_cm, [5:7]=height_confidence
    height_kalman_cm = struct.unpack_from("<H", tail, 1)[0] if len(tail) >= 3 else 0
    height_filtered_cm = struct.unpack_from("<H", tail, 3)[0] if len(tail) >= 5 else 0
    height_confidence = struct.unpack_from("<H", tail, 5)[0] if len(tail) >= 7 else 0
    return AvoidanceResult(
        type_ver=(type_byte, ver_byte),
        status_flags=status_flags,
        btm_avoid_dis_cm=btm_avoid,
        up_avoid_dis_cm=up_avoid,
        sectors=sectors,
        tail=tail,
        height_kalman_cm=height_kalman_cm,
        height_filtered_cm=height_filtered_cm,
        height_confidence=height_confidence,
    )


def summarize_avoidance_result(msg_id: int, payload: bytes) -> str:
    try:
        ar = decode_avoidance_result(payload)
    except ValueError as exc:
        return str(exc)
    active = [s for s in ar.sectors if not s.is_default]
    lines = [
        f"type={ar.type_ver[0]} ver={ar.type_ver[1]} flags=0x{ar.status_flags:04X} "
        f"btm={ar.btm_avoid_m:.2f}m up={ar.up_avoid_m:.2f}m "
        f"height={ar.height_kalman_m:.2f}m(kalman) filt={ar.height_filtered_cm}cm conf={ar.height_confidence} "
        f"active_sectors={len(active)}/16 tail={ar.tail.hex()}"
    ]
    for s in active:
        lines.append(
            f"  sector[{s.index}] az={s.azimuth_deg:.2f}° el={s.elevation_deg:.2f}° "
            f"dist={s.distance_m:.2f}m"
        )
    return "\n".join(lines)


class LivePlotter:
    def __init__(self, config: ClientConfig,
                 on_motor_rpm: Optional[Callable[[int], None]] = None) -> None:
        try:
            import matplotlib
            import matplotlib.pyplot as plt
        except Exception as exc:  # pragma: no cover - optional runtime dependency
            raise RuntimeError(f"matplotlib unavailable: {exc}") from exc
        self.plt = plt
        self.range_m = max(config.plot_range_m, 1.0)
        self.history_sec = max(config.plot_history_sec, 0.5)
        self.refresh_interval = 1.0 / max(config.plot_refresh_hz, 1.0)
        self.last_redraw = 0.0
        self.arrival_times: deque[float] = deque(maxlen=240)
        self.total_frames: Counter[int] = Counter()
        self.latest_counts: dict[int, int] = {0: 0, 1: 0, 2: 0}
        # Cap1 analysis (subtype=0 ground, subtype=1 wire, subtype=2 pcla):
        #   ground_points — no tracking (all id=0); each frame is independent
        #     CFAR peaks. 87 stable reflector bins (≥10% hit rate) carry the
        #     real scene structure, 37% bins are 1-2 hit flicker / multipath.
        #     → long time-window density heatmap + stable-reflector overlay.
        #   wire_candidates — ids exist but are NOT per-target tracking. Same
        #     id often reappears at completely unrelated coordinates (e.g.
        #     id=7 jumps (-0.7,-3.9)→(+3.3,+1.7), 5 m away). Drawing per-id
        #     trails manufactures fake trajectories.
        #     → show last-N-frame snapshot, metric-colored, no per-id lines.
        #   pcla — id=1..22 is an AZIMUTH SECTOR INDEX (the side array cuts
        #     its rotation into 22 angular bins, higher id = more forward).
        #     Each frame lists multiple points per sector = range peaks inside
        #     that beam. → color by sector id, show latest snapshot only.
        # 1E61 data-rate reality (measured from cap1 / 241 s run):
        #   subtype=0 ground_points : ~1.6 Hz
        #   subtype=1 wire          : ~0.17 Hz (one frame every ~6 s)
        #   subtype=2 pcla          : ~1.1 Hz
        # Wire is too slow to aggregate — any time window either leaves
        # the panel empty (short window) or stacks stale data (long
        # window). Show only the LATEST wire frame, same as pcla.
        # Ground_points is fast enough that a short (1-2 s) time window
        # is useful for the density heatmap (multiple frames accumulate).
        self.ground_window_sec = max(0.1, float(config.plot_ground_window_sec))
        self.ground_frames: deque[dict[str, Any]] = deque()  # pruned by time
        self.wire_frames: deque[dict[str, Any]] = deque(maxlen=1)  # latest only
        self.pcla_frames: deque[dict[str, Any]] = deque()    # latest pcla only
        self.pcla_trail_len = 1         # pcla shows just latest frame
        # wire_window_sec retained for the status-text label only;
        # the deque is length-bounded, not time-bounded.
        self.wire_window_sec = max(0.1, float(config.plot_wire_window_sec))
        # Visualization mirroring FC avoidance model (see CLAUDE.md
        # "FC avoidance pipeline"). Two non-conflicting wedge layers are
        # always drawn on ax_top: narrow saturated wedges for 1E62 (FC
        # truth) + wide faint wedges for 8-dir bin of our 1E61 points
        # (what FC would see if it binned our raw perception directly).
        # Terrain mode only changes per-point color coding.
        self.terrain = (config.plot_terrain or "generic").lower()
        # Stable reflector extraction (recomputed on redraw from ground_frames)
        self.stable_threshold_pct = 0.10  # bin hit in ≥10% of frames = stable
        self.status_history: dict[int, deque[dict[str, float]]] = {
            0x1E62: deque(),
            0x1E32: deque(),
        }
        self.latest_status: dict[int, dict[str, float]] = {}
        self.last_seq: Optional[int] = None
        self.last_payload_len: int = 0
        self.last_update_wall: float = 0.0
        self.last_nonempty_1e61_wall: float = 0.0
        self.empty_1e61_streak: int = 0
        self.rti_mode = config.rti_mode
        self.rti_history_len = max(config.rti_history, 10)
        self.rti_buf: Optional[np.ndarray] = None  # shape (rti_history_len, 256), dB values
        self.rti_row_idx = 0
        self.rti_frame_count = 0
        self.rti_fps_times: deque[float] = deque(maxlen=120)
        # Lazy-created colorbar handle for the zenith panel; kept so that
        # _plot_zenith_panel doesn't stack a new colorbar on every redraw.
        self._zenith_cbar = None
        # Shared z-height colorbar for the side-antenna scope (wire + pcla)
        self._ax_top_cbar = None

        plt.ion()
        if self.rti_mode:
            self.fig = plt.figure(figsize=(11, 6))
            grid = self.fig.add_gridspec(2, 2, height_ratios=[3.0, 1.0],
                                         width_ratios=[3.0, 1.0],
                                         hspace=0.25, wspace=0.20,
                                         left=0.06, right=0.98,
                                         top=0.93, bottom=0.09)
            self.ax_waterfall = self.fig.add_subplot(grid[0, :])
            self.ax_spectrum = self.fig.add_subplot(grid[1, 0])
            self.ax_info = self.fig.add_subplot(grid[1, 1])
            self.ax_top = None
            self.ax_side = None
            self.ax_counts = None
            self.ax_zenith = None
            self.ax_height = None
        else:
            # Compact 2×3 layout sized to sit half-screen alongside
            # radar_viewer.py (which is itself 13×7.5). Content:
            #   [ax_top (side-ant XY)][ax_side (side-ant z-cut)][ax_zenith (top-ant)]
            #   [ax_counts (btm_avoid)][ax_height (kalman)    ][ax_info]
            # ax_zenith is the dedicated top-antenna panel: ground_points
            # (1E61 subtype=0) come from the side rotating array's
            # below-horizon beams — NOT the top 4×4 (see CLAUDE.md for
            # the full data-source mapping).
            self.fig = plt.figure(figsize=(13, 7.5))
            grid = self.fig.add_gridspec(2, 3, height_ratios=[3.0, 1.4],
                                         hspace=0.35, wspace=0.30,
                                         left=0.05, right=0.98,
                                         top=0.93, bottom=0.07)
            self.ax_top = self.fig.add_subplot(grid[0, 0])
            self.ax_side = self.fig.add_subplot(grid[0, 1])
            self.ax_zenith = self.fig.add_subplot(grid[0, 2])
            self.ax_counts = self.fig.add_subplot(grid[1, 0])
            self.ax_height = self.fig.add_subplot(grid[1, 1])
            self.ax_info = self.fig.add_subplot(grid[1, 2])
            self.ax_waterfall = None
            self.ax_spectrum = None
        manager = getattr(self.fig.canvas, "manager", None)
        backend_name = matplotlib.get_backend().lower()
        if backend_name in {
            "agg",
            "pdf",
            "ps",
            "svg",
            "template",
            "module://matplotlib_inline.backend_inline",
        } or type(self.fig.canvas).__name__ == "FigureCanvasAgg":
            plt.close(self.fig)
            raise RuntimeError(
                "matplotlib is using a non-interactive backend "
                f"({matplotlib.get_backend()!r}). This environment cannot show a live plot window. "
                "Run under a desktop/X11 session and select an interactive backend such as TkAgg/QtAgg, "
                "or keep using --capture-dir for offline analysis."
            )
        if manager is not None and hasattr(manager, "set_window_title"):
            manager.set_window_title("Radar RTI Waterfall" if self.rti_mode else "Radar Live Plot")

        _TERRAIN_CYCLE = ["generic", "ground", "mountain", "fruiter"]
        plotter_self = self
        def _on_key(event):
            if event.key != 't':
                return
            idx = _TERRAIN_CYCLE.index(plotter_self.terrain) if plotter_self.terrain in _TERRAIN_CYCLE else -1
            plotter_self.terrain = _TERRAIN_CYCLE[(idx + 1) % len(_TERRAIN_CYCLE)]
            if plotter_self.ax_top is not None:
                plotter_self.ax_top.set_title(plotter_self._ax_top_title(),
                                              fontsize=8, color="#aaa")
            plotter_self._ax_top_cbar_dirty = True
            plotter_self.fig.canvas.draw_idle()
        self.fig.canvas.mpl_connect('key_press_event', _on_key)
        self._ax_top_cbar_dirty = False

        # ---- rotation-speed dropdown ----
        # Motor RPM is commanded over CAN by the host client; this widget lets
        # the user pick a preset live and pushes it through on_motor_rpm.
        from matplotlib.widgets import Button, RadioButtons
        self.motor_rpm_presets = [600, 900, 1200, 1500, 1800]
        self.motor_rpm = int(getattr(config, "motor_run_rpm", 1200))
        self.on_motor_rpm = on_motor_rpm
        self._rpm_btn_ax = self.fig.add_axes([0.840, 0.945, 0.140, 0.035])
        self._rpm_btn = Button(self._rpm_btn_ax, f"RPM {self.motor_rpm} ▼")
        self._rpm_menu_ax = self.fig.add_axes([0.840, 0.620, 0.140, 0.320])
        self._rpm_radio = RadioButtons(
            self._rpm_menu_ax, [f"{r}" for r in self.motor_rpm_presets])
        self._rpm_menu_ax.set_visible(False)
        self._rpm_menu_open = False
        # Suppress the callback that set_active() fires during init so we
        # don't push a speed command before the bridge is ready.
        self._rpm_suppress_cb = True
        try:
            self._rpm_radio.set_active(self.motor_rpm_presets.index(self.motor_rpm))
        except ValueError:
            pass
        self._rpm_suppress_cb = False

        def _toggle_rpm_menu(_event):
            self._rpm_menu_open = not self._rpm_menu_open
            self._rpm_menu_ax.set_visible(self._rpm_menu_open)
            self.fig.canvas.draw_idle()

        def _on_rpm_select(label):
            try:
                rpm = int(str(label))
            except ValueError:
                return
            self.motor_rpm = rpm
            self._rpm_btn.label.set_text(f"RPM {rpm} ▼")
            self._rpm_menu_open = False
            self._rpm_menu_ax.set_visible(False)
            self.fig.canvas.draw_idle()
            if self._rpm_suppress_cb or self.on_motor_rpm is None:
                return
            self.on_motor_rpm(rpm)

        self._rpm_btn.on_clicked(_toggle_rpm_menu)
        self._rpm_radio.on_clicked(_on_rpm_select)

        self._redraw(force=True)

    def is_alive(self) -> bool:
        return self.plt.fignum_exists(self.fig.number)

    def poll(self) -> None:
        if not self.is_alive():
            return
        if time.monotonic() - self.last_redraw >= self.refresh_interval:
            self._redraw(force=False)
        # Avoid plt.pause(): it internally calls show(block=False) which
        # raises the window to the front on each frame (TkAgg/QtAgg),
        # stealing focus and hiding terminal/other windows. Driving
        # events via canvas.flush_events + start_event_loop keeps the
        # window at its current z-order.
        canvas = self.fig.canvas
        try:
            canvas.flush_events()
            canvas.start_event_loop(0.001)
        except Exception:
            time.sleep(0.001)

    def update_perception(self, seq: int, subtype: int, entries: list[dict[str, Any]], payload_len: int) -> None:
        now = time.monotonic()
        self.arrival_times.append(now)
        self.total_frames[subtype] += 1
        self.latest_counts[subtype] = len(entries)
        self.last_seq = seq
        self.last_payload_len = payload_len
        self.last_update_wall = time.time()
        if entries:
            self.last_nonempty_1e61_wall = self.last_update_wall
            self.empty_1e61_streak = 0
        else:
            self.empty_1e61_streak += 1
        if subtype == 0:
            # Ground snapshot — keep for the full time window (default 30 s)
            # so stable reflectors accumulate; prune older frames by time.
            self.ground_frames.append({
                "t": now, "seq": seq,
                "points": [
                    (e["x_m"], e["y_m"], e["z_m"], e.get("metric", 0))
                    for e in entries
                    if -10.0 <= e["z_m"] <= 1.0  # drop multipath outliers early
                ],
            })
            ground_cutoff = now - self.ground_window_sec
            while self.ground_frames and self.ground_frames[0]["t"] < ground_cutoff:
                self.ground_frames.popleft()
        elif subtype == 1:
            # Wire candidates — latest frame only (deque has maxlen=1).
            # 1E61 subtype=1 is emitted at ~0.17 Hz so any time window
            # either empties the panel or locks stale data in place.
            # Carry metric so the plot can size/color by signal strength.
            self.wire_frames.append({
                "t": now, "seq": seq,
                "points": [
                    (e["x_m"], e["y_m"], e["z_m"], e["metric"], e["id"])
                    for e in entries
                ],
            })
        elif subtype == 2:
            # pcla — latest snapshot only. id is azimuth sector index, not
            # class; plot will color-cycle per sector to make the scan
            # pattern visible.
            self.pcla_frames.append({
                "t": now, "seq": seq,
                "points": [
                    (e["x_m"], e["y_m"], e["z_m"], e.get("metric", 0), e.get("id", 0))
                    for e in entries
                ],
            })
            while len(self.pcla_frames) > self.pcla_trail_len:
                self.pcla_frames.popleft()
        self._redraw(force=False)

    def update_v1_status(self, msg_id: int, seq: int, payload: bytes) -> None:
        if msg_id not in self.status_history:
            return
        try:
            ar = decode_avoidance_result(payload)
        except ValueError:
            return
        now = time.monotonic()
        active_count = sum(1 for s in ar.sectors if not s.is_default)
        active_sector_items = [
            {
                "index": float(s.index),
                "azimuth_deg": s.azimuth_deg,
                "elevation_deg": s.elevation_deg,
                "distance_m": s.distance_m,
            }
            for s in ar.sectors
            if not s.is_default
        ]
        item = {
            "t": now,
            "seq": float(seq),
            "btm_m": ar.btm_avoid_m,
            "up_m": ar.up_avoid_m,
            "active_sectors": float(active_count),
            "height_m": ar.height_kalman_m,
            "height_filt_cm": ar.height_filtered_cm,
            "height_conf": ar.height_confidence,
            "sector_items": active_sector_items,
            "status_flags": float(ar.status_flags),
        }
        self.status_history[msg_id].append(item)
        self.latest_status[msg_id] = item
        self._prune_status_history(now)
        self._redraw(force=False)

    def update_rti(self, magnitudes: np.ndarray) -> None:
        """Add a new 256-bin magnitude spectrum row to the RTI waterfall."""
        if self.rti_buf is None:
            self.rti_buf = np.full((self.rti_history_len, len(magnitudes)), -60.0, dtype=np.float64)
        # Convert to dB (20*log10), clamp floor
        with np.errstate(divide="ignore", invalid="ignore"):
            db = 20.0 * np.log10(np.maximum(magnitudes, 1e-12))
        row_len = min(len(db), self.rti_buf.shape[1])
        self.rti_buf[self.rti_row_idx % self.rti_history_len, :row_len] = db[:row_len]
        self.rti_row_idx += 1
        self.rti_frame_count += 1
        self.rti_fps_times.append(time.monotonic())
        self.last_update_wall = time.time()

    @staticmethod
    def _fps_is_stale(times: deque[float]) -> bool:
        return len(times) < 2 or time.monotonic() - times[-1] > 0.5

    def _rti_fps(self) -> float:
        if self._fps_is_stale(self.rti_fps_times):
            return 0.0
        span = self.rti_fps_times[-1] - self.rti_fps_times[0]
        return (len(self.rti_fps_times) - 1) / span if span > 0 else 0.0

    def _prune_status_history(self, now: Optional[float] = None) -> None:
        cutoff = (time.monotonic() if now is None else now) - self.history_sec
        for queue in self.status_history.values():
            while queue and queue[0]["t"] < cutoff:
                queue.popleft()

    def _frame_rate(self) -> float:
        if self._fps_is_stale(self.arrival_times):
            return 0.0
        span = self.arrival_times[-1] - self.arrival_times[0]
        if span <= 0.0:
            return 0.0
        return (len(self.arrival_times) - 1) / span

    def _wire_nearest_distance(self) -> Optional[float]:
        """Shortest distance from drone to any wire candidate in the
        current display window."""
        best: Optional[float] = None
        for frame in self.wire_frames:
            for x, y, z, _metric, _id in frame["points"]:
                d = (x * x + y * y + z * z) ** 0.5
                if best is None or d < best:
                    best = d
        return best

    def _pcla_min_metric(self) -> Optional[int]:
        if not self.pcla_frames:
            return None
        latest = self.pcla_frames[-1]["points"]
        if not latest:
            return None
        return min(int(p[3]) for p in latest)

    def _pcla_sector_ids(self) -> list[int]:
        """Distinct pcla sector ids (1..22) present in the latest frame."""
        if not self.pcla_frames:
            return []
        return sorted({int(p[4]) for p in self.pcla_frames[-1]["points"]})

    def _wire_z_range_line(self) -> str:
        """z-range summary line for the info panel — gives numeric context
        for the color encoding on ax_top."""
        zs: list[float] = []
        for frame in self.wire_frames:
            for _x, _y, z, _m, _id in frame["points"]:
                zs.append(z)
        if not zs:
            return "wire z range: n/a"
        return (f"wire z range: [{min(zs):+.2f}..{max(zs):+.2f}] m "
                f"(med {float(np.median(zs)):+.2f})")

    def _pcla_z_range_line(self) -> str:
        if not self.pcla_frames or not self.pcla_frames[-1]["points"]:
            return "pcla z range: n/a"
        zs = [p[2] for p in self.pcla_frames[-1]["points"]]
        return (f"pcla z range: [{min(zs):+.2f}..{max(zs):+.2f}] m "
                f"(med {float(np.median(zs)):+.2f})")

    def _ground_point_count(self) -> int:
        if not self.ground_frames:
            return 0
        return len(self.ground_frames[-1]["points"])

    def _extract_stable_reflectors(self, bin_size: float = 0.5
                                   ) -> list[tuple[float, float, float]]:
        """Compute the cap1-style stable reflector list from the current
        ground_frames time window: bins hit in ≥stable_threshold_pct frames.
        Returns list of (x_center, y_center, hit_fraction)."""
        if not self.ground_frames:
            return []
        bin_frame_hits: dict[tuple[int, int], set[int]] = {}
        for i, frame in enumerate(self.ground_frames):
            seen_this_frame: set[tuple[int, int]] = set()
            for x, y, _z, _m in frame["points"]:
                bx, by = int(x / bin_size), int(y / bin_size)
                seen_this_frame.add((bx, by))
            for key in seen_this_frame:
                bin_frame_hits.setdefault(key, set()).add(i)
        n_frames = len(self.ground_frames)
        threshold = max(2, int(n_frames * self.stable_threshold_pct))
        stable = []
        for (bx, by), frames_hit in bin_frame_hits.items():
            if len(frames_hit) >= threshold:
                stable.append(
                    (bx * bin_size + bin_size / 2,
                     by * bin_size + bin_size / 2,
                     len(frames_hit) / n_frames)
                )
        stable.sort(key=lambda r: -r[2])
        return stable

    def _ax_top_title(self) -> str:
        """Title string for ax_top that names the active point-coloring
        mode so screenshots are self-documenting. Wedge overlays
        (1E62 narrow + 8-dir faint) are always on, no need to announce."""
        color_mode = {
            "generic":  "color=z height (coolwarm)",
            "ground":   "color=ground-terrain threat (z>-0.3m = red)",
            "mountain": "color=z height (mountain mode, FC uses slope; same cmap)",
            "fruiter":  "color=fruiter threat (canopy green, above red)",
        }.get(self.terrain, f"color=z ({self.terrain})")
        return f"Side Scope — Wire+PCLA — {color_mode}"

    def _urgency_color(self, dist_m: float) -> str:
        """Map obstacle distance → urgency-ramp color (FC mental model:
        close = must-brake-now, far = safe). Gray means "no obstacle /
        default sector"."""
        if dist_m >= self.range_m:
            return "#999999"              # default / saturated "no obstacle"
        frac = dist_m / max(self.range_m, 1e-6)
        if frac < 0.33:
            return "#e63946"              # red: close, brake
        if frac < 0.66:
            return "#f4a261"              # orange: caution
        return "#2a9d8f"                  # teal: far, safe

    def _terrain_point_color(self, z: float) -> Optional[str]:
        """Categorical color for a point's z under the active terrain mode.
        Returns None when the terrain is 'generic' (caller should use the
        coolwarm z colormap instead)."""
        t = self.terrain
        if t == "ground":
            # z≈0 is aircraft level; ground points are typically below the
            # drone by 1-2 m so z<-0.3 is "ground", everything above is a
            # potential above-ground obstacle.
            return "#9aa0a6" if z < -0.3 else "#e63946"
        if t == "fruiter":
            # T40 spraying over fruit trees: canopy at roughly aircraft
            # level, ground 1-3 m below, obstacles (wires/poles) above.
            if z < -2.0:
                return "#9aa0a6"          # ground (not a threat)
            if z < 2.0:
                return "#2a9d8f"          # canopy — spray target / work zone
            return "#e63946"              # obstacle above canopy
        # 'mountain' and 'generic' → caller uses coolwarm cmap
        return None

    def _apply_terrain_colors(self, zs: Any, base_kwargs: dict[str, Any]
                              ) -> dict[str, Any]:
        """If terrain mode uses categorical coloring, replace cmap/vmin/vmax
        kwargs with explicit per-point colors. No-op for 'generic'/'mountain'."""
        if self.terrain in ("generic", "mountain"):
            return base_kwargs
        colors = [self._terrain_point_color(float(z)) for z in zs]
        base_kwargs["c"] = colors
        for k in ("cmap", "vmin", "vmax"):
            base_kwargs.pop(k, None)
        return base_kwargs

    def _plot_sector_overlay(self, ax: Any) -> bool:
        """Two-layer avoidance-visualisation overlay on ax_top:

          * Background (zorder=0.1, faint): 8 wide wedges from our own
            1E61 points binned into LMAP_DIRECTION_* compass bins — "what
            FC would see if it consumed our raw perception directly".
          * Foreground (zorder=0.5, saturated): 16 narrow wedges from the
            latest 1E62 frame — FC's actual computed avoidance output.

        The two layers don't conflict: narrow-opaque over wide-faint makes
        divergences between FC's aggregated view and our raw points
        visually obvious (e.g. faint red behind a saturated-green sector
        = FC is filtering out something we still see).

        1E32 is dropped from overlay (same body as 1E62, just a duplicate
        destined for the altimeter — double-rendering adds no information).
        """
        try:
            from matplotlib.patches import Wedge
        except Exception:
            return False

        plotted = False

        # Background: 8-direction bin of our 1E61 points
        plotted = self._plot_8dir_wedges(ax, Wedge) or plotted

        # Foreground: 1E62 16-sector filled wedges (FC truth)
        latest = self.latest_status.get(0x1E62)
        if latest is not None:
            for sector in latest.get("sector_items", []):
                az = float(sector["azimuth_deg"])
                full = float(sector["distance_m"])
                draw_r = min(full, self.range_m)
                if draw_r <= 0.0:
                    continue
                # matplotlib Wedge: θ=0 is +screen_x, CCW. Our PPI has
                # screen_x = body y_m (right), screen_y = body x_m
                # (forward). Body az=0 → forward → screen +y → θ=90.
                theta = 90.0 - az
                halfw = 11.25  # 360° / 16 sectors / 2
                c = self._urgency_color(full)
                w = Wedge((0.0, 0.0), draw_r,
                          theta - halfw, theta + halfw,
                          facecolor=c, alpha=0.35,
                          edgecolor=c, linewidth=0.6, zorder=0.5)
                ax.add_patch(w)
                # Capped-out reading (> display range): mark rim with 'x'.
                if full > self.range_m:
                    theta_r = math.radians(az)
                    x_m = self.range_m * math.cos(theta_r)
                    y_m = self.range_m * math.sin(theta_r)
                    ax.scatter([y_m], [x_m], c="#bbbbbb",
                               marker="x", s=20, alpha=0.7, zorder=1)
                plotted = True

        return plotted

    def _collect_body_frame_points(self
                                   ) -> tuple[list[float], list[float]]:
        """Return (xs, ys) body-frame coords of every recent 1E61 point
        (wire last-N + pcla latest + ground latest). Used by the 8-dir
        binning overlay."""
        xs: list[float] = []
        ys: list[float] = []
        for frame in self.wire_frames:
            for x, y, *_ in frame["points"]:
                xs.append(float(x)); ys.append(float(y))
        if self.pcla_frames and self.pcla_frames[-1]["points"]:
            for p in self.pcla_frames[-1]["points"]:
                xs.append(float(p[0])); ys.append(float(p[1]))
        if self.ground_frames and self.ground_frames[-1]["points"]:
            for p in self.ground_frames[-1]["points"]:
                xs.append(float(p[0])); ys.append(float(p[1]))
        return xs, ys

    def _plot_8dir_wedges(self, ax: Any, Wedge) -> bool:
        """Bin our 1E61 points into 8 LMAP_DIRECTION_* compass wedges
        (front / front-right / right / back-right / back / back-left /
        left / front-left), and shade each by nearest-point distance.

        This is the "how FC would see OUR points" overlay — it doesn't
        replace the 1E62 FC-truth wedges but gives a side-by-side on the
        same axes for validation / divergence spotting.
        """
        xs, ys = self._collect_body_frame_points()
        if not xs:
            return False
        x_arr = np.array(xs, dtype=np.float64)
        y_arr = np.array(ys, dtype=np.float64)
        dist = np.sqrt(x_arr * x_arr + y_arr * y_arr)
        # Body azimuth: forward=+x, right=+y → az = atan2(y, x) in (-180, 180].
        az_deg = np.degrees(np.arctan2(y_arr, x_arr))
        # Eight compass centers matching dji_perception's LMAP_DIRECTION_*:
        # FRONT, F-RIGHT, RIGHT, B-RIGHT, BACK, B-LEFT, LEFT, F-LEFT.
        centers = (0.0, 45.0, 90.0, 135.0, 180.0, -135.0, -90.0, -45.0)
        halfw = 22.5
        drawn = False
        for center in centers:
            diff = ((az_deg - center + 180.0) % 360.0) - 180.0
            mask = np.abs(diff) < halfw
            if not np.any(mask):
                continue
            nearest = float(np.min(dist[mask]))
            theta = 90.0 - center
            c = self._urgency_color(nearest)
            w = Wedge((0.0, 0.0), min(nearest, self.range_m),
                      theta - halfw, theta + halfw,
                      facecolor=c, alpha=0.10,
                      edgecolor="none", zorder=0.1)
            ax.add_patch(w)
            drawn = True
        return drawn

    def _plot_zenith_panel(self, now: float) -> None:
        """Omni ground-return density map.

        Data source semantics (verified against cap1 recording):
          - `ground_points` (1E61 subtype=0) comes from the SIDE 12×6
            rotating array — specifically its below-horizon elevation
            beams accumulated into an omni (360°) snapshot per frame.
            NOT the top 4×4 array, which doesn't surface on the UDP
            perception channel.
          - Every point has id=0 (no per-target tracking), every z < 0
            (below aircraft), so the "trail" picture is meaningless —
            a static scene still produces a different random set of
            CFAR peak positions each frame. Drawing individual dots
            creates the "flickering confetti" effect the user saw.

        Correct rendering: time-decayed 2D density heatmap over (x, y).
        Stable reflectors accumulate; flicker outliers stay sparse.
        Z < -10 m points are dropped as CFAR multipath/image artefacts
        (real ground at 1-2 m AGL gives z near 0).
        """
        ax = self.ax_zenith
        rng = min(self.range_m, 15.0)
        ax.set_title("Omni Ground (side 12×6 rotating array, density)")
        # Forward-up PPI: screen y = x_m (forward), screen x = y_m (right)
        ax.set_xlabel("y (m, right →)")
        ax.set_ylabel("x (m, ↑ forward)")
        ax.set_xlim(-rng, rng)
        ax.set_ylim(-rng, rng)
        ax.set_aspect("equal", adjustable="box")
        # Range rings 3/5/10 m
        theta = np.linspace(0.0, 2.0 * math.pi, 120)
        for radius in (3.0, 5.0, 10.0):
            if radius > rng:
                continue
            ax.plot(
                radius * np.cos(theta), radius * np.sin(theta),
                color="white", linewidth=0.6, alpha=0.7, linestyle=":",
            )
        ax.plot([0.0], [0.0], marker="^", color="white",
                markersize=9, markeredgecolor="black", label="drone")

        # Build density grid from the full time window (default 30 s).
        # Outlier filtering already applied at update_perception.
        N_BINS = 48
        edges = np.linspace(-rng, rng, N_BINS + 1)
        grid = np.zeros((N_BINS, N_BINS), dtype=np.float64)
        n_frames = 0
        latest_seq = 0
        latest_n_pts = 0
        if self.ground_frames:
            n_frames = len(self.ground_frames)
            latest_t = self.ground_frames[-1]["t"]
            latest_seq = self.ground_frames[-1]["seq"]
            latest_n_pts = len(self.ground_frames[-1]["points"])
            for frame in self.ground_frames:
                if not frame["points"]:
                    continue
                # Age-weight: newest ~ 1.0, oldest at window edge ~ 0.3
                age = latest_t - frame["t"]
                weight = max(0.3, 1.0 - age / max(self.ground_window_sec, 0.1))
                xs = [p[0] for p in frame["points"]
                      if abs(p[0]) <= rng and abs(p[1]) <= rng]
                ys = [p[1] for p in frame["points"]
                      if abs(p[0]) <= rng and abs(p[1]) <= rng]
                if xs:
                    h, _, _ = np.histogram2d(xs, ys, bins=(edges, edges))
                    grid += h * weight
        if grid.max() > 0:
            # histogram2d(xs, ys) returns H[x_bin, y_bin].
            # imshow rows → vertical axis, cols → horizontal axis.
            # We want forward(x) vertical, right(y) horizontal, so
            # use H as-is (rows=x, cols=y) — NO transpose.
            display = np.log1p(grid)
            ax.imshow(
                display, origin="lower",
                extent=[-rng, rng, -rng, rng],
                cmap="magma", aspect="equal",
                vmin=0, vmax=max(display.max(), 0.5),
                interpolation="nearest",
            )
            ax.legend(loc="upper right", fontsize=7, facecolor="white",
                      framealpha=0.7)
            # Overlay stable reflectors (bin hit in ≥10% of window frames).
            # These are the real physical reflectors in the scene.
            stable = self._extract_stable_reflectors(bin_size=0.5)
            for cx, cy, frac in stable[:20]:
                if abs(cx) > rng or abs(cy) > rng:
                    continue
                # (cx, cy) = (x_m, y_m). Plot as (y_m, x_m) → forward-up.
                ax.plot([cy], [cx], marker="o", color="#00ffff",
                        markerfacecolor="none", markersize=11,
                        markeredgewidth=1.3, zorder=5)
            ax.text(
                0.02, 0.98,
                f"window={self.ground_window_sec:.0f}s  frames={n_frames}\n"
                f"latest seq={latest_seq} n={latest_n_pts}\n"
                f"stable reflectors={len(stable)} "
                f"(cyan ○ ≥{int(self.stable_threshold_pct*100)}% hit)",
                transform=ax.transAxes, fontsize=7.5, va="top",
                family="monospace", color="white",
                bbox=dict(boxstyle="round", facecolor="black", alpha=0.5),
            )
        else:
            ax.text(
                0.5, 0.5,
                "No 1E61 subtype=0 ground_points yet\n"
                "(side 12×6 array omni snapshot\n"
                "→ ground_plane_fit → height_fusion)",
                ha="center", va="center", transform=ax.transAxes,
                fontsize=8, color="gray",
            )

    def _plot_height_timeseries(self, now: float) -> None:
        ax = self.ax_height
        ax.set_title("Ground-Plane Height Fusion (1E62 tail)")
        ax.set_xlabel("seconds from now")
        ax.set_ylabel("height (m)")
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.5, 30.0)
        history = self.status_history.get(0x1E62) or deque()
        if history:
            xs = [item["t"] - now for item in history]
            kalman = [item["height_m"] for item in history]
            filtered = [item["height_filt_cm"] / 100.0 for item in history]
            conf = [item["height_conf"] for item in history]
            ax.plot(xs, kalman, marker="o", markersize=3, linewidth=1.2,
                    color="#2a9d8f", label="kalman")
            ax.plot(xs, filtered, marker=".", markersize=2, linewidth=0.8,
                    color="#e76f51", label="filtered", alpha=0.7)
            ax.legend(loc="upper right", fontsize=7)
            latest = history[-1]
            ax.text(
                0.02, 0.95,
                f"h_kalman={latest['height_m']:.2f}m  "
                f"h_filt={latest['height_filt_cm']/100:.2f}m  "
                f"conf={int(latest['height_conf'])}",
                transform=ax.transAxes, fontsize=8, va="top",
                family="monospace",
            )
        else:
            ax.text(
                0.5, 0.5,
                "Waiting for 0x1E62 frames\n(height_kalman lives in payload tail)",
                ha="center", va="center", transform=ax.transAxes,
                fontsize=8, color="gray",
            )

    def _plot_status_series(self, ax: Any, value_key: str, title: str, ylabel: str, now: float) -> None:
        ax.set_title(title)
        ax.set_xlabel("seconds from now")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        plotted = False
        for msg_id, color in ((0x1E62, "#d1495b"), (0x1E32, "#0077b6")):
            history = self.status_history[msg_id]
            if not history:
                continue
            xs = [item["t"] - now for item in history]
            ys = [item[value_key] for item in history]
            ax.plot(xs, ys, marker="o", markersize=3, linewidth=1.2, color=color, label=f"0x{msg_id:04X}")
            plotted = True
        if plotted:
            ax.legend(loc="best")
        else:
            ax.text(0.5, 0.5, "No recent 1E62/1E32 samples", ha="center", va="center", transform=ax.transAxes)

    def _redraw_rti(self, now: float) -> None:
        self.ax_waterfall.clear()
        self.ax_spectrum.clear()
        self.ax_info.clear()

        if self.rti_buf is not None and self.rti_row_idx > 0:
            n_rows = min(self.rti_row_idx, self.rti_history_len)
            # Build ordered waterfall: oldest at top, newest at bottom
            if self.rti_row_idx <= self.rti_history_len:
                img = self.rti_buf[:n_rows, :]
            else:
                idx = self.rti_row_idx % self.rti_history_len
                img = np.concatenate([self.rti_buf[idx:, :], self.rti_buf[:idx, :]], axis=0)

            # Range axis: 256 bins, ~0.75m/bin at 200MHz BW
            n_bins = img.shape[1]
            self.ax_waterfall.imshow(
                img, aspect="auto", origin="upper",
                extent=[0, n_bins, n_rows, 0],
                cmap="viridis", vmin=-40, vmax=60,
                interpolation="nearest",
            )
            self.ax_waterfall.set_title(f"RTI Waterfall ({n_rows} frames)")
            self.ax_waterfall.set_xlabel("Range Bin")
            self.ax_waterfall.set_ylabel("Frame (newest at bottom)")

            # Latest spectrum line plot
            latest_idx = (self.rti_row_idx - 1) % self.rti_history_len
            latest_row = self.rti_buf[latest_idx, :]
            self.ax_spectrum.plot(np.arange(n_bins), latest_row, color="#2a9d8f", linewidth=0.8)
            self.ax_spectrum.set_title("Latest 1D Range Spectrum (dB)")
            self.ax_spectrum.set_xlabel("Range Bin")
            self.ax_spectrum.set_ylabel("Magnitude (dB)")
            self.ax_spectrum.set_xlim(0, n_bins)
            self.ax_spectrum.grid(True, alpha=0.3)
        else:
            self.ax_waterfall.text(0.5, 0.5, "Waiting for 0x1EFD data transfer frames...",
                                   ha="center", va="center", transform=self.ax_waterfall.transAxes)
            self.ax_spectrum.text(0.5, 0.5, "No data yet", ha="center", va="center",
                                  transform=self.ax_spectrum.transAxes)

        # Info panel
        info_lines = [
            f"rotor speed: {self.motor_rpm} rpm  [cmd]",
            f"RTI frames: {self.rti_frame_count}",
            f"RTI fps: {self._rti_fps():5.1f} Hz",
            f"history depth: {self.rti_history_len}",
            f"bins: 256 (FFT magnitude)",
        ]
        if self.last_update_wall:
            info_lines.append(
                f"last update: {datetime.fromtimestamp(self.last_update_wall).strftime('%H:%M:%S.%f')[:-3]}"
            )
        self.ax_info.axis("off")
        self.ax_info.set_title("RTI Info")
        self.ax_info.text(0.02, 0.98, "\n".join(info_lines), va="top", ha="left", family="monospace")
        self.fig.canvas.draw_idle()

    def _redraw(self, force: bool) -> None:
        now = time.monotonic()
        if not force and now - self.last_redraw < self.refresh_interval:
            return
        self.last_redraw = now

        if self.rti_mode:
            self._redraw_rti(now)
            return

        self._prune_status_history(now)

        self.ax_top.clear()
        self.ax_side.clear()
        self.ax_counts.clear()
        self.ax_info.clear()
        self.ax_zenith.clear()
        self.ax_height.clear()

        # ── Side antenna scope: wire last-2s + pcla latest by sector ───
        # Semantics (verified against cap1):
        #   - Wire ids are NOT per-target tracking; drawing per-id trails
        #     creates fake trajectories. Show last 2 s of wire points as
        #     plain scatter, size/alpha encode metric (signal strength).
        #   - PCLA id 1..22 is an azimuth sector index. Color points by
        #     sector so the rotating scan pattern is visible.
        self.ax_top.set_title(self._ax_top_title())
        # Standard PPI convention: forward at top of plot. 1E61 data
        # has x=forward, y=right; we plot (screen_x, screen_y) = (y, x)
        # so +x goes up (forward) and +y goes right on the display.
        self.ax_top.set_xlabel("y (m, right →)")
        self.ax_top.set_ylabel("x (m, ↑ forward)")
        self.ax_top.set_xlim(-self.range_m, self.range_m)
        self.ax_top.set_ylim(-self.range_m, self.range_m)
        self.ax_top.set_aspect("equal", adjustable="box")
        self.ax_top.grid(True, alpha=0.3)
        self.ax_top.axhline(0.0, color="gray", linewidth=0.8)
        self.ax_top.axvline(0.0, color="gray", linewidth=0.8)
        self.ax_top.plot([0.0], [0.0], marker="^", color="black",
                         markersize=8, zorder=5)

        has_points = False
        # Wire: last-N-seconds, color=z, size=metric (5s window smooths
        # CFAR flicker; color tells operator the vertical position of each
        # target so (x,y,z) is readable from a single panel).
        all_wx, all_wy, all_wz, all_wm = [], [], [], []
        for frame in self.wire_frames:
            for x, y, z, m, _id in frame["points"]:
                all_wx.append(x); all_wy.append(y)
                all_wz.append(z); all_wm.append(m)
        # Common z-color range keeps wire/pcla on the same colorbar
        # (only used when terrain == generic/mountain; the categorical
        # terrain modes override c= with explicit per-point colors).
        Z_VMIN, Z_VMAX = -2.0, 3.0
        uses_cmap = self.terrain in ("generic", "mountain")
        wire_sc = None
        if all_wx:
            sizes = np.clip(np.array(all_wm) / 5.0, 12, 80)
            kwargs = dict(c=all_wz, cmap="coolwarm",
                          vmin=Z_VMIN, vmax=Z_VMAX,
                          s=sizes, alpha=0.85,
                          edgecolor="black", linewidth=0.3,
                          label=f"wire latest (n={len(all_wx)}, "
                                f"age={now - self.wire_frames[-1]['t']:.1f}s)")
            kwargs = self._apply_terrain_colors(all_wz, kwargs)
            # Note: plot (y_m, x_m) not (x_m, y_m) — forward-up PPI
            wire_sc = self.ax_top.scatter(all_wy, all_wx, **kwargs)
            has_points = True
        # PCLA: latest frame, color=z (same colorbar), marker=x so it's
        # visually distinct from wire dots even when they overlap
        pcla_sc = None
        if self.pcla_frames and self.pcla_frames[-1]["points"]:
            pcla_pts = self.pcla_frames[-1]["points"]
            px = [p[0] for p in pcla_pts]
            py = [p[1] for p in pcla_pts]
            pz = [p[2] for p in pcla_pts]
            kwargs = dict(c=pz, cmap="coolwarm", vmin=Z_VMIN, vmax=Z_VMAX,
                          marker="x", s=22, alpha=0.8, linewidths=1.2,
                          label=f"pcla n={len(pcla_pts)} "
                                f"sectors={len(self._pcla_sector_ids())}")
            kwargs = self._apply_terrain_colors(pz, kwargs)
            # Same axis swap as wire: (y_m, x_m) → (screen_x, screen_y)
            pcla_sc = self.ax_top.scatter(py, px, **kwargs)
            has_points = True
        # Single shared colorbar for z (lazy-created). Only attach in
        # cmap mode — terrain-categorical modes have no meaningful
        # continuous scale.
        if not uses_cmap and self._ax_top_cbar is not None:
            try:
                self._ax_top_cbar.ax.remove()
            except Exception:
                pass
            self._ax_top_cbar = None
        if uses_cmap and (wire_sc is not None or pcla_sc is not None):
            sc_for_bar = wire_sc if wire_sc is not None else pcla_sc
            if getattr(self, "_ax_top_cbar", None) is None:
                self._ax_top_cbar = self.fig.colorbar(
                    sc_for_bar, ax=self.ax_top,
                    fraction=0.035, pad=0.02
                )
                self._ax_top_cbar.set_label("z (m,  blue ↓  red ↑)",
                                            fontsize=8)
            else:
                self._ax_top_cbar.update_normal(sc_for_bar)
        has_points = self._plot_sector_overlay(self.ax_top) or has_points
        if has_points:
            self.ax_top.legend(loc="upper right", fontsize=7)
        else:
            self.ax_top.text(
                0.5, 0.5,
                "No wire / pcla in current window",
                ha="center", va="center", transform=self.ax_top.transAxes,
                fontsize=9, color="gray",
            )

        # ── Side view: z vs horizontal distance — last 2s of all types ─
        # Wire + pcla + ground-latest combined, so heights at different
        # distances are readable. Wire stands out in red; ground in teal;
        # pcla in gray.
        self.ax_side.set_title("Side View — Height vs Range (last 2s)")
        self.ax_side.set_xlabel("horiz distance (m)")
        self.ax_side.set_ylabel("z (m)")
        self.ax_side.set_xlim(0, self.range_m)
        self.ax_side.set_ylim(-5.0, 5.0)
        self.ax_side.grid(True, alpha=0.3)
        self.ax_side.axhline(0.0, color="gray", linewidth=0.8)
        plotted_side = False
        # Ground latest frame only (dense; stacking multiple would drown panel)
        if self.ground_frames and self.ground_frames[-1]["points"]:
            pts = self.ground_frames[-1]["points"]
            if pts:
                gh = [(p[0] ** 2 + p[1] ** 2) ** 0.5 for p in pts]
                gz = [p[2] for p in pts]
                self.ax_side.scatter(gh, gz, c="#2a9d8f", s=10, alpha=0.55,
                                     label=f"ground latest (n={len(pts)})")
                plotted_side = True
        # Wire last-N-sec
        wh_all, wz_all = [], []
        for frame in self.wire_frames:
            for x, y, z, _m, _id in frame["points"]:
                wh_all.append((x * x + y * y) ** 0.5)
                wz_all.append(z)
        if wh_all:
            self.ax_side.scatter(wh_all, wz_all, c="#e76f51", s=25, alpha=0.8,
                                 label=f"wire (n={len(wh_all)})")
            plotted_side = True
        # PCLA latest
        if self.pcla_frames and self.pcla_frames[-1]["points"]:
            pts = self.pcla_frames[-1]["points"]
            ph = [(p[0] ** 2 + p[1] ** 2) ** 0.5 for p in pts]
            pz = [p[2] for p in pts]
            self.ax_side.scatter(ph, pz, c="#777777", s=12, marker="x",
                                 alpha=0.6, label=f"pcla (n={len(pts)})")
            plotted_side = True
        if plotted_side:
            self.ax_side.legend(loc="upper right", fontsize=7)

        # ── Top-antenna zenith panel ──────────────────────────────────────
        # 1E61 subtype=0 ground_points come from the fixed-height pipeline
        # driven by the 4×4 upward array. Plot top-down with z as color so
        # terrain undulations and ground-plane tilt become readable. Origin
        # marker is the drone; range rings at 3/5/10 m give distance sense.
        self._plot_zenith_panel(now)

        # ── Height-fusion timeseries ─────────────────────────────────────
        self._plot_height_timeseries(now)

        self._plot_status_series(self.ax_counts, "btm_m", "Bottom Avoid Distance", "meters", now)

        counts = [
            self.latest_counts.get(0, 0),
            self.latest_counts.get(1, 0),
            self.latest_counts.get(2, 0),
        ]
        wire_dist = self._wire_nearest_distance()
        pcla_metric = self._pcla_min_metric()
        pcla_sectors = self._pcla_sector_ids()
        latest_1e62 = self.latest_status.get(0x1E62)
        latest_1e32 = self.latest_status.get(0x1E32)
        gnd_count = self._ground_point_count()
        wire_last_n = sum(len(f["points"]) for f in self.wire_frames)
        stable_count = len(self._extract_stable_reflectors(bin_size=0.5))
        info_lines = [
            f"rotor speed: {self.motor_rpm} rpm  [cmd]",
            f"last seq: {self.last_seq if self.last_seq is not None else 'n/a'}",
            f"1E61 fps: {self._frame_rate():5.1f} Hz",
            f"latest payload_len: {self.last_payload_len}",
            f"latest counts: gnd={counts[0]} wire={counts[1]} pcla={counts[2]}",
            f"empty 1E61 streak: {self.empty_1e61_streak}",
            f"ground pts (this frame): {gnd_count}",
            f"stable reflectors: {stable_count} (≥{int(self.stable_threshold_pct*100)}% hit)",
            f"wire latest frame: {wire_last_n} pts",
            f"wire nearest: {wire_dist:.2f} m" if wire_dist is not None else "wire nearest: n/a",
            self._wire_z_range_line(),
            f"pcla sectors present: {len(pcla_sectors)}/22 {pcla_sectors}",
            self._pcla_z_range_line(),
            f"pcla min metric: {pcla_metric}" if pcla_metric is not None else "pcla min metric: n/a",
            (
                f"1E62: btm={latest_1e62['btm_m']:.2f}m up={latest_1e62['up_m']:.2f}m "
                f"height={latest_1e62['height_m']:.2f}m active={int(latest_1e62['active_sectors'])}"
                if latest_1e62 is not None
                else "1E62 latest: n/a"
            ),
            (
                f"1E32: btm={latest_1e32['btm_m']:.2f}m up={latest_1e32['up_m']:.2f}m "
                f"height={latest_1e32['height_m']:.2f}m active={int(latest_1e32['active_sectors'])}"
                if latest_1e32 is not None
                else "1E32 latest: n/a"
            ),
            f"history window: {self.history_sec:.1f}s",
            f"terrain: {self.terrain}  [press 't' to cycle]",
            (
                f"last non-empty 1E61: {datetime.fromtimestamp(self.last_nonempty_1e61_wall).strftime('%H:%M:%S')}"
                if self.last_nonempty_1e61_wall
                else "last non-empty 1E61: never"
            ),
            "if 1E61 stays empty: check height gate / terrain_mode / point-count filters",
        ]
        if self.last_update_wall:
            info_lines.append(
                f"last update: {datetime.fromtimestamp(self.last_update_wall).strftime('%H:%M:%S')}"
            )
        self.ax_info.axis("off")
        self.ax_info.set_title("Radar Summary")
        self.ax_info.text(0.02, 0.98, "\n".join(info_lines), va="top", ha="left", family="monospace")
        self.fig.canvas.draw_idle()


class RadarClient:
    def __init__(self, config: ClientConfig) -> None:
        self.config = config
        self.capture_root = self._init_capture_root(config.capture_dir)
        self.plotter = self._init_plotter(config)
        self.start_monotonic = time.monotonic()
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock.bind((config.local_ip, config.control_local_port))
        self.control_sock.setblocking(False)
        # V2 rx socket is optional: a co-located 3D viewer may want to own
        # port 40204 itself (--no-v2-listen), so we only bind when enabled.
        self.v2_sock = None
        if config.v2_listen and config.v2_local_port > 0:
            self.v2_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.v2_sock.bind((config.local_ip, config.v2_local_port))
            self.v2_sock.setblocking(False)
        self.v1_seq = 0
        self.time_sync_counter = 0
        self.data_transfer = DataTransferState()
        # FC-bridge / ESC-sim state
        self.can_device = None
        self.can_parser = None
        self._can_raw_log_fh = None
        self._can_duml_log_fh = None
        if config.can_bridge:
            # Lazy import — only pay the dependency cost when bridging.
            from duml_can_tool import SocketCANDevice, DumlStreamParser
            self.can_device = SocketCANDevice(
                config.can_bridge, listen_ids=[config.can_rx_id]
            )
            self.can_parser = DumlStreamParser()
            print(
                f"[cfg] can_bridge=on iface={config.can_bridge} "
                f"tx=0x{config.can_tx_id:03X} rx=0x{config.can_rx_id:03X}"
            )
            # If the user gave --capture-dir, also tee every bridged CAN
            # frame (raw 8B chunks) AND every reassembled DUML frame to
            # disk. raw_can.log is candump-compatible so `canplayer` can
            # replay it verbatim; duml.jsonl is one-line-per-frame JSON
            # keyed by direction/can_id/cmd_set/cmd_id for easy replay
            # from Python when writing the pure-sim server later.
            if self.capture_root is not None:
                raw_path = self.capture_root / "can" / "raw_can.log"
                duml_path = self.capture_root / "can" / "duml.jsonl"
                self._can_raw_log_fh = raw_path.open("a", buffering=1)
                self._can_duml_log_fh = duml_path.open("a", buffering=1)
                print(
                    f"[cfg] can_log=on raw={raw_path} duml={duml_path}"
                )
        now = time.monotonic()
        self.tasks = [
            PeriodicTask("flight_pos", 1.0 / config.pos_hz, self._build_flight_pos_payload, 0x1E21, now),
            PeriodicTask("flight_debug", 1.0 / config.debug_hz, self._build_flight_debug_payload, 0x1E25, now),
            PeriodicTask("time_sync", 1.0 / config.time_hz, self._build_time_sync_payload, 0x1E27, now),
        ]
        # --sim-esc: impersonate ESC with the exact periodic stream observed
        # on a real CAN-attached T40 ESC (bottom_data_pack + imu_inter_pack +
        # keepalive + activation reply + diagnostic), plus synthesize
        # hms_switch ACK on demand so app.elf accepts the ESC as healthy.
        #
        # Packed sender/receiver pairs:
        #   0x1E/0x0F bottom_data_pack:  sender=DEV:esc(0x18) → RADAR:esc(0x78)
        #   0x1E/0x0B imu_inter_pack:    sender=DEV:esc(0x18) → RADAR:esc(0x78)
        #   0x1E/0xEA keepalive:         sender=DEV:esc(0x18) → RADAR:app(0x02)
        #   0x00/0x32 activation reply:  sender=DEV:esc(0x18) → RADAR:app(0x02)
        #                                 [+ duplicate to DEV:pc_assist(0x0A)]
        #   0x0E/0x01 diagnostic:        sender=DEV:esc(0x18) → 0xC3
        if config.sim_esc and not config.can_bridge:
            # 4.3 Hz ≈ 233 ms — the actual observed rate in cap1
            esc_fast = 1.0 / 4.3
            esc_slow = 2.0
            self.tasks.extend([
                PeriodicTask("sim_bottom_data_pack", esc_fast,
                             self._build_sim_bottom_payload, 0x1E0F, now,
                             sender=0x18, receiver=0x78, flags=0x00),
                PeriodicTask("sim_imu_inter_pack", esc_fast,
                             self._build_sim_imu_payload, 0x1E0B, now,
                             sender=0x18, receiver=0x78, flags=0x00),
                PeriodicTask("sim_keepalive_ea", esc_slow,
                             lambda: bytes.fromhex(SIM_ESC_KEEPALIVE_EA_HEX),
                             0x1EEA, now,
                             sender=0x18, receiver=0x02, flags=0x00),
                PeriodicTask("sim_activation_app", esc_slow,
                             lambda: bytes.fromhex(SIM_ESC_ACTIVATION_HEX),
                             0x0032, now,
                             sender=0x18, receiver=0x02, flags=0x20),
                PeriodicTask("sim_activation_assist", esc_slow,
                             lambda: bytes.fromhex(SIM_ESC_ACTIVATION_HEX),
                             0x0032, now + 0.001,  # tiny stagger
                             sender=0x18, receiver=0x0A, flags=0x20),
                PeriodicTask("sim_diag_0e01", esc_slow,
                             lambda: bytes.fromhex(SIM_ESC_DIAG_0E01_HEX),
                             0x0E01, now,
                             sender=0x18, receiver=0xC3, flags=0x20),
            ])

        # --replay-esc: load captured ESC→app stream and schedule its
        # replay. Same seq_bump trick as the standalone helper used to
        # have: each loop iteration shifts every seq by +1000 so the
        # app doesn't dedup us against the previous pass.
        self._replay_events: list[ReplayEvent] = []
        self._replay_idx = 0
        self._replay_origin: Optional[float] = None
        self._replay_seq_bump = 0
        self._replay_loops_completed = 0
        if config.replay_esc and not config.can_bridge:
            self._replay_events = load_replay_events(
                config.replay_esc, only_motor=config.replay_esc_only_motor,
            )
            if not self._replay_events:
                print(f"[warn] --replay-esc: no CAN_TO_UDP frames in "
                      f"{config.replay_esc}, replay disabled")
            else:
                from collections import Counter
                bd = Counter((e.cmd_set, e.cmd_id) for e in self._replay_events)
                duration = self._replay_events[-1].rel_ts
                print(f"[cfg] replay_esc={config.replay_esc} "
                      f"events={len(self._replay_events)} "
                      f"duration={duration:.1f}s "
                      f"loop={config.replay_esc_loop} speed={config.replay_esc_speed}x "
                      f"only_motor={config.replay_esc_only_motor}")
                for (cs, ci), cnt in sorted(bd.items(), key=lambda x: -x[1]):
                    name = MOTOR_CMD_IDS.get(ci, "") if cs == 0x1E else ""
                    print(f"       cs=0x{cs:02X} ci=0x{ci:02X}"
                          f"{('(' + name + ')') if name else '':<22} x{cnt}")

    def _init_plotter(self, config: ClientConfig) -> Optional[LivePlotter]:
        if not config.plot:
            return None
        try:
            return LivePlotter(config, on_motor_rpm=self._on_motor_rpm_change)
        except RuntimeError as exc:
            print(f"[warn] live plot disabled: {exc}")
            return None

    def _init_capture_root(self, capture_dir: Optional[str]) -> Optional[Path]:
        if not capture_dir:
            return None
        session_name = f"radar_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        root = Path(capture_dir) / session_name
        for dirname in ("v1_raw", "v2_raw", "perception", "blackbox", "data_transfer", "rti", "can"):
            (root / dirname).mkdir(parents=True, exist_ok=True)
        return root

    def _write_capture_bytes(self, relative_path: str, data: bytes) -> None:
        if self.capture_root is None:
            return
        path = self.capture_root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)

    def _write_capture_text(self, relative_path: str, data: str) -> None:
        if self.capture_root is None:
            return
        path = self.capture_root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(data, encoding="utf-8")

    def _write_capture_json(self, relative_path: str, payload: dict[str, Any]) -> None:
        if self.capture_root is None:
            return
        path = self.capture_root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _capture_v1_frame(self, frame: V1Frame, raw_frame: bytes) -> None:
        label = safe_label(MSG_NAMES.get(frame.msg_id, f"unknown_0x{frame.msg_id:04x}"))
        self._write_capture_bytes(
            f"v1_raw/{frame.seq:05d}_{frame.msg_id:04x}_{label}.bin",
            raw_frame,
        )

    def _capture_v2_frame_raw(self, frame: V2Frame, raw_frame: bytes) -> None:
        label = safe_label(MSG_NAMES.get(frame.msg_id, f"unknown_0x{frame.msg_id:04x}"))
        self._write_capture_bytes(
            f"v2_raw/{frame.seq:010d}_{frame.msg_id:04x}_{label}.bin",
            raw_frame,
        )

    def _capture_perception_frame(self, frame: V2Frame) -> None:
        subtype = frame.payload[65] if len(frame.payload) > 65 else -1
        subtype_name = perception_subtype_name(subtype)
        stem = f"1e61_seq{frame.seq:010d}_sub{subtype}_{safe_label(subtype_name)}"
        self._write_capture_bytes(f"perception/{stem}.bin", frame.payload)
        try:
            decoded_subtype, entries = decode_perception_1e61_entries(frame.payload)
        except ValueError as exc:
            self._write_capture_json(
                f"perception/{stem}.json",
                {
                    "seq": frame.seq,
                    "sender": f"0x{frame.sender:04X}",
                    "receiver": f"0x{frame.receiver:04X}",
                    "payload_len": len(frame.payload),
                    "error": str(exc),
                },
            )
            return
        self._write_capture_json(
            f"perception/{stem}.json",
            {
                "seq": frame.seq,
                "sender": f"0x{frame.sender:04X}",
                "receiver": f"0x{frame.receiver:04X}",
                "payload_len": len(frame.payload),
                "subtype": decoded_subtype,
                "subtype_name": perception_subtype_name(decoded_subtype),
                "dds_hint": PERCEPTION_DDS_HINTS.get(decoded_subtype),
                "entry_count": len(entries),
                "header_hex": frame.payload[:PERCEPTION_1E61_HEADER_LEN].hex(),
                "entry_preview": entries[: min(32, len(entries))],
            },
        )
        self._write_capture_text(
            f"perception/{stem}.csv",
            build_perception_csv(decoded_subtype, entries),
        )

    def _capture_blackbox_frame(self, frame: V2Frame) -> None:
        stem = f"f000_seq{frame.seq:010d}"
        self._write_capture_bytes(f"blackbox/{stem}.bin", frame.payload)
        last_u32 = None
        if len(frame.payload) >= 4:
            last_u32 = f"0x{struct.unpack_from('<I', frame.payload, len(frame.payload) - 4)[0]:08X}"
        self._write_capture_json(
            f"blackbox/{stem}.json",
            {
                "seq": frame.seq,
                "sender": f"0x{frame.sender:04X}",
                "receiver": f"0x{frame.receiver:04X}",
                "payload_len": len(frame.payload),
                "last_u32_le": last_u32,
                "note": "raw blackbox V2 chunk; reassembly format still under analysis",
            },
        )

    def _current_simulated_motion(self) -> tuple[float, float, float, float, float, float]:
        cfg = self.config
        if not cfg.sim_axis or cfg.sim_amplitude_m <= 0.0 or cfg.sim_period_sec <= 0.0:
            return (cfg.pos_x_m, cfg.pos_y_m, cfg.pos_z_m, cfg.vel_x_mps, cfg.vel_y_mps, cfg.vel_z_mps)
        phase = (time.monotonic() - self.start_monotonic) * (2.0 * math.pi / cfg.sim_period_sec)
        offset = cfg.sim_amplitude_m * math.sin(phase)
        velocity = cfg.sim_amplitude_m * (2.0 * math.pi / cfg.sim_period_sec) * math.cos(phase)
        pos_x = cfg.pos_x_m
        pos_y = cfg.pos_y_m
        pos_z = cfg.pos_z_m
        vel_x = cfg.vel_x_mps
        vel_y = cfg.vel_y_mps
        vel_z = cfg.vel_z_mps
        if cfg.sim_axis == "x":
            pos_x += offset
            vel_x += velocity
        elif cfg.sim_axis == "y":
            pos_y += offset
            vel_y += velocity
        elif cfg.sim_axis == "z":
            pos_z += offset
            vel_z += velocity
        return (pos_x, pos_y, pos_z, vel_x, vel_y, vel_z)

    def _build_flight_debug_payload(self) -> bytes:
        cfg = self.config
        if cfg.flight_debug_hex:
            return parse_fixed_hex_payload(cfg.flight_debug_hex, 38)
        return build_flight_debug_payload()

    def _current_flight_mode(self) -> int:
        """Return the flight_mode byte to put in the next flight_pos payload.

        Reverse-engineered quirk (sub_65F10 pm state machine): the radar
        only ever transitions into pm3 (200 MHz real-flight mode) when it
        sees flight_mode change from 1 → 2. Holding flight_mode == 2
        statically leaves pm stuck at 1. On real aircraft the FC naturally
        sweeps through flight_mode=1 during init; on bench we need to
        emulate that ramp. Default: spend the first fm_ramp_sec seconds
        sending flight_mode=1, then switch to cfg.flight_mode (=2).
        """
        cfg = self.config
        if cfg.fm_ramp_sec > 0.0:
            elapsed = time.monotonic() - self.start_monotonic
            if elapsed < cfg.fm_ramp_sec:
                return 1
        return cfg.flight_mode & 0xFF

    def _build_flight_pos_payload(self) -> bytes:
        payload = bytearray(66)
        cfg = self.config
        pos_x_m, pos_y_m, pos_z_m, vel_x_mps, vel_y_mps, vel_z_mps = self._current_simulated_motion()
        struct.pack_into("<4f", payload, 0, 0.0, 0.0, 0.0, 1.0)
        struct.pack_into("<h", payload, 16, encode_scaled_i16(pos_x_m, 0.001))
        struct.pack_into("<h", payload, 18, encode_scaled_i16(pos_y_m, 0.001))
        struct.pack_into("<h", payload, 20, encode_scaled_i16(pos_z_m, 0.001))
        struct.pack_into("<h", payload, 22, encode_scaled_i16(vel_x_mps, 0.001))
        struct.pack_into("<h", payload, 24, encode_scaled_i16(vel_y_mps, 0.001))
        struct.pack_into("<h", payload, 26, encode_scaled_i16(vel_z_mps, 0.001))
        struct.pack_into("<h", payload, 28, encode_scaled_i16(cfg.aux_x, 0.01))
        struct.pack_into("<h", payload, 30, encode_scaled_i16(cfg.aux_y, 0.01))
        struct.pack_into("<h", payload, 32, encode_scaled_i16(cfg.aux_z, 0.01))
        struct.pack_into("<f", payload, 34, 0.0)
        payload[38] = self._current_flight_mode()
        payload[39] = cfg.flight_flags & 0xFF
        struct.pack_into("<d", payload, 40, cfg.lat_deg)
        struct.pack_into("<d", payload, 48, cfg.lon_deg)
        payload[56] = cfg.gps_valid & 0xFF
        struct.pack_into("<d", payload, 57, time.time())
        payload[65] = cfg.pose_tag & 0xFF
        return bytes(payload)

    def _build_time_sync_payload(self) -> bytes:
        self.time_sync_counter += 1
        return build_time_sync_payload(self.time_sync_counter)

    def _next_seq(self) -> int:
        value = self.v1_seq & 0xFFFF
        self.v1_seq = (self.v1_seq + 1) & 0xFFFF
        return value

    def send_v1(self, msg_id: int, payload: bytes, label: str,
                sender: Optional[int] = None,
                receiver: Optional[int] = None,
                flags: Optional[int] = None,
                seq: Optional[int] = None) -> None:
        seq_val = seq if seq is not None else self._next_seq()
        sender_b = sender if sender is not None else self.config.sender
        receiver_b = receiver if receiver is not None else self.config.receiver
        flags_b = flags if flags is not None else self.config.flags
        frame = build_v1_frame(
            msg_id=msg_id,
            payload=payload,
            seq=seq_val,
            sender=sender_b,
            receiver=receiver_b,
            flags=flags_b,
        )
        self.control_sock.sendto(frame, (self.config.radar_ip, self.config.control_remote_port))
        print(
            f"[tx/v1] {label} msg=0x{msg_id:04X} len={len(payload)} "
            f"seq={seq_val} sender=0x{sender_b:02X} receiver=0x{receiver_b:02X} "
            f"flags=0x{flags_b:02X} "
            f"to={self.config.radar_ip}:{self.config.control_remote_port}"
        )

    def send_one_shots(self) -> None:
        if self.config.send_motor_start:
            self.send_v1(
                0x1E22,
                build_flight_ctrl_payload(self.config.motor_start_subid, True),
                f"motor_start_subid_{self.config.motor_start_subid}",
            )
        if self.config.send_sample_start:
            self.send_v1(0x1E26, build_sample_ctrl_payload(1), "sample_start")
        if self.config.send_sample_mark:
            self.send_v1(0x1E26, build_sample_ctrl_payload(10), "sample_mark")
        if self.config.send_data_start_type is not None:
            self.send_v1(
                0x1EFC,
                build_req_or_terminate_data_payload(0, self.config.send_data_start_type),
                f"data_start_type_{self.config.send_data_start_type}",
            )
        if self.config.send_data_end_type is not None:
            self.send_v1(
                0x1EFC,
                build_req_or_terminate_data_payload(1, self.config.send_data_end_type),
                f"data_end_type_{self.config.send_data_end_type}",
            )

    def run(self) -> None:
        self.send_one_shots()
        motor_speed_interval = 1.0 / max(0.1, self.config.motor_speed_hz)
        motor_speed_next_at = 0.0
        while True:
            if self.plotter is not None and not self.plotter.is_alive():
                print("[info] plot window closed, exiting")
                break
            now = time.monotonic()
            # Periodically push the running rotor-speed setpoint to the ESC
            # so it never lapses to its safety-timeout default.
            if self.can_device is not None and now >= motor_speed_next_at:
                self._send_motor_speed_on_can(self.config.motor_run_rpm)
                motor_speed_next_at = now + motor_speed_interval
            timeout = 0.25
            for task in self.tasks:
                if now >= task.next_at:
                    self.send_v1(
                        task.msg_id, task.builder(), task.name,
                        sender=task.sender,
                        receiver=task.receiver,
                        flags=task.flags,
                    )
                    task.next_at = now + task.interval
                timeout = min(timeout, max(0.0, task.next_at - now))
            # Replay ESC stream (if loaded)
            if self._replay_events:
                timeout = min(timeout, self._tick_replay(now))
            if self.plotter is not None:
                timeout = min(timeout, 0.05)
            read_socks = [self.control_sock]
            if self.v2_sock is not None:
                read_socks.append(self.v2_sock)
            if self.can_device is not None:
                read_socks.append(self.can_device.sock)
            readable, _, _ = select.select(read_socks, [], [], timeout)
            for sock_obj in readable:
                if self.can_device is not None and sock_obj is self.can_device.sock:
                    self._handle_can_readable()
                    continue
                data, peer = sock_obj.recvfrom(65535)
                if sock_obj is self.control_sock:
                    self.handle_v1_datagram(data, peer)
                else:
                    self.handle_v2_datagram(data, peer)
            if self.plotter is not None:
                self.plotter.poll()

    def handle_v1_datagram(self, data: bytes, peer: tuple[str, int]) -> None:
        frame = parse_v1_frame(data)
        if frame is None:
            print(f"[rx/v1] invalid frame from {peer[0]}:{peer[1]} len={len(data)}")
            print(hexdump(data[:64]))
            return
        self._capture_v1_frame(frame, data)
        name = MSG_NAMES.get(frame.msg_id, "unknown")
        cmd_set = (frame.msg_id >> 8) & 0xFF
        cmd_id = frame.msg_id & 0xFF
        print(
            f"[rx/v1] msg=0x{frame.msg_id:04X}({name}) len={len(frame.payload)} seq={frame.seq} "
            f"sender=0x{frame.sender:02X} receiver=0x{frame.receiver:02X} flags=0x{frame.flags:02X} "
            f"from={peer[0]}:{peer[1]}"
        )
        # Motor / ESC-oriented DUML traveling through the FC bridge path.
        # Handle BEFORE the generic print paths so we log the forward/ACK
        # action alongside the rx log.
        if cmd_set == 0x1E and cmd_id in MOTOR_CMD_IDS:
            if self._handle_motor_cmd(frame, data, cmd_id):
                return
        if frame.msg_id == 0x1EFD:
            self._handle_data_transfer(frame)
        elif frame.msg_id in (0x1E62, 0x1E32) and len(frame.payload) == 116:
            if self.plotter is not None:
                self.plotter.update_v1_status(frame.msg_id, frame.seq, frame.payload)
            print(summarize_avoidance_result(frame.msg_id, frame.payload))
        elif frame.payload:
            print(hexdump(frame.payload[: min(48, len(frame.payload))]))

    # ── CAN logging helpers ─────────────────────────────────────────────
    def _log_can_raw(self, can_id: int, data: bytes) -> None:
        """Write one candump-compatible line. Direction is encoded by
        can_id (tx uses can_tx_id, rx uses can_rx_id) — `canplayer` can
        replay the file verbatim without modification."""
        if self._can_raw_log_fh is None:
            return
        try:
            self._can_raw_log_fh.write(
                f"({time.time():.6f}) {self.config.can_bridge} "
                f"{can_id:03X}#{data.hex().upper()}\n"
            )
        except OSError:
            pass

    def _log_can_duml(self, direction: str, can_id: int, frame_bytes: bytes) -> None:
        """Write one JSON-lines record describing a reassembled DUML frame.
        direction ∈ {"UDP_TO_CAN","CAN_TO_UDP"}."""
        if self._can_duml_log_fh is None:
            return
        parsed = parse_v1_frame(frame_bytes)
        record: dict[str, Any] = {
            "ts": time.time(),
            "dir": direction,
            "can_id": f"0x{can_id:03X}",
            "frame_len": len(frame_bytes),
            "frame_hex": frame_bytes.hex(),
        }
        if parsed is not None:
            cs = (parsed.msg_id >> 8) & 0xFF
            ci = parsed.msg_id & 0xFF
            cmd_name = MOTOR_CMD_IDS.get(ci, "") if cs == 0x1E else ""
            record.update({
                "sender": f"0x{parsed.sender:02X}",
                "receiver": f"0x{parsed.receiver:02X}",
                "seq": parsed.seq,
                "flags": f"0x{parsed.flags:02X}",
                "cmd_set": f"0x{cs:02X}",
                "cmd_id": f"0x{ci:02X}",
                "cmd_name": cmd_name,
                "payload_hex": parsed.payload.hex(),
            })
        else:
            record["parse_error"] = True
        try:
            self._can_duml_log_fh.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError:
            pass

    def _send_motor_speed_on_can(self, rpm: int) -> None:
        """Command the ESC rotor speed over CAN (0x1E80 motor_speed,
        RADAR:esc → DEV:esc). ``rpm`` is a u16 LE setpoint. Sender/receiver
        match what app.elf uses for its own motor_speed frames.
        """
        if self.can_device is None:
            return
        payload = struct.pack("<H", max(0, min(65535, int(rpm))))
        frame = build_v1_frame(
            msg_id=0x1E80,          # cmd_set=0x1E, cmd_id=0x80 motor_speed
            payload=payload,
            seq=self._next_seq(),
            sender=0x78,            # RADAR:esc (packed = (3<<5)|0x18)
            receiver=0x18,          # DEV:esc
            flags=0x00,
        )
        try:
            for offset in range(0, len(frame), 8):
                chunk = frame[offset : offset + 8]
                self.can_device.send_can(self.config.can_tx_id, chunk)
                self._log_can_raw(self.config.can_tx_id, chunk)
            self._log_can_duml("UDP_TO_CAN", self.config.can_tx_id, frame)
        except OSError as exc:
            print(f"[bridge] motor-speed tx failed: {exc}")
            return
        print(f"[bridge] sent motor_speed={int(rpm)} rpm to ESC "
              f"({len(frame)}B → can_id=0x{self.config.can_tx_id:03X})")

    def _on_motor_rpm_change(self, rpm: int) -> None:
        """Live rotation-speed change from the plot UI dropdown.

        Updates the running setpoint (so the periodic re-send keeps using it)
        and pushes it to the ESC immediately.
        """
        self.config.motor_run_rpm = int(rpm)
        self._send_motor_speed_on_can(self.config.motor_run_rpm)
        print(f"[plot] rotation speed → {self.config.motor_run_rpm} rpm")

    def _send_motor_stop_on_can(self) -> None:
        """Emit a final motor_speed setpoint on CAN before releasing the
        bridge. Many ESCs hold the last commanded speed if no new setpoint
        arrives within their safety timeout — explicitly sending the idle
        setpoint (config.motor_stop_rpm, default 50) gives a deterministic
        ramp-down to idle instead of relying on that timeout.
        """
        self._send_motor_speed_on_can(self.config.motor_stop_rpm)
        # Short grace period so the kernel CAN queue is actually flushed
        # to the wire before we tear the socket down.
        time.sleep(0.05)

    def _send_motor_stop_via_udp(self) -> None:
        """Send a 0x1E22 flight_ctrl (motor_stop) to app.elf over UDP.

        This is the DUML command that a real FC would issue to bring
        app.elf's motor FSM back down — if the radar's session was
        promoted to MOTOR_RUNNING_NORMAL by us (via flight_pos
        keepalive + DFA activation), leaving it in that state at client
        exit can leave the motor FSM and `unk_282DEC` stuck, which has
        been observed to affect the next boot's initialisation.

        This is fire-and-forget. If the control socket is already
        gone (or was never opened) we silently skip.
        """
        sock = getattr(self, "control_sock", None)
        if sock is None:
            return
        try:
            payload = build_flight_ctrl_payload(
                sub_id=self.config.motor_start_subid, start_motor=False)
            frame = build_v1_frame(
                msg_id=0x1E22,
                payload=payload,
                seq=self._next_seq(),
                sender=self.config.sender,
                receiver=self.config.receiver,
                flags=self.config.flags,
            )
            sock.sendto(frame, (self.config.radar_ip,
                                 self.config.control_remote_port))
            print(f"[shutdown] sent 0x1E22 flight_ctrl motor_stop "
                  f"(sub_id={self.config.motor_start_subid}) → "
                  f"{self.config.radar_ip}:{self.config.control_remote_port}")
        except OSError as exc:
            print(f"[shutdown] motor_stop UDP tx failed: {exc}")

    def close(self) -> None:
        """Graceful shutdown. Safe to call multiple times.

        Order matters:
          (1) tell app.elf via UDP to stop the motor (DFA command),
              so the firmware's motor FSM / unk_282DEC unwinds cleanly;
          (2) give app.elf ~150 ms to process that before tearing
              down the CAN / UDP paths;
          (3) drive ESC to the idle setpoint (config.motor_stop_rpm,
              default 50 rpm) on CAN (if bridge is up);
          (4) flush disk logs;
          (5) close sockets.
        """
        # 1) UDP motor-stop (fire-and-forget)
        self._send_motor_stop_via_udp()
        # 2) Short grace period so app.elf can dispatch the flight_ctrl
        #    and start its own shutdown path. This is empirical — ~100 ms
        #    is enough for sub_3E85C + set_motor_state_from_fc(0).
        try:
            time.sleep(0.15)
        except Exception:
            pass
        # 3) If a physical CAN bridge is up, command the ESC to stop
        #    before we drop the bus.
        if getattr(self, "can_device", None) is not None:
            self._send_motor_stop_on_can()
            try:
                self.can_device.close()
            except OSError:
                pass
            self.can_device = None
        # 4) Flush and close the capture log handles.
        for attr in ("_can_raw_log_fh", "_can_duml_log_fh"):
            fh = getattr(self, attr, None)
            if fh is not None:
                try:
                    fh.flush()
                    fh.close()
                except OSError:
                    pass
                setattr(self, attr, None)
        # 5) Close the UDP sockets.
        for attr in ("control_sock", "v2_sock"):
            sock = getattr(self, attr, None)
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
                setattr(self, attr, None)

    def _handle_motor_cmd(self, frame: V1Frame, raw: bytes, cmd_id: int) -> bool:
        """Route motor/ESC DUML (cmd_set=0x1E). Returns True if we handled it
        and the caller should skip the generic hexdump path."""
        label = MOTOR_CMD_IDS.get(cmd_id, f"0x{cmd_id:02X}")
        is_response = (frame.flags & 0x80) != 0
        direction = "app→esc" if not is_response else "esc→app(ack)"
        if self.can_device is not None:
            # Transparent bridge: split the DUML bytes into 8B CAN chunks
            # ourselves so we can log each chunk individually (candump
            # parity) instead of relying on SocketCANDevice.send_stream.
            try:
                for offset in range(0, len(raw), 8):
                    chunk = raw[offset : offset + 8]
                    self.can_device.send_can(self.config.can_tx_id, chunk)
                    self._log_can_raw(self.config.can_tx_id, chunk)
            except OSError as exc:
                print(f"[bridge] CAN tx failed: {exc}")
                return False
            self._log_can_duml("UDP_TO_CAN", self.config.can_tx_id, raw)
            print(
                f"[bridge] UDP→CAN {direction} motor_{label} "
                f"cmd_id=0x{cmd_id:02X} {len(raw)}B → can_id=0x{self.config.can_tx_id:03X}"
            )
            return True
        # Software simulator path — triggers when either --sim-esc or
        # --replay-esc is active (both impersonate ESC on UDP and need the
        # same inbound-handling semantics for app→ESC cmds).
        if self.config.sim_esc or self.config.replay_esc:
            if cmd_id == 0x83 and not is_response:
                # hms_switch request — reply with ACK [ack_ret=0, value=1]
                # so hms_switch_ack_handler writes unk_282DEC = 2.
                ack_payload = build_hms_switch_ack_payload(0, 1)
                # Swap sender/receiver and set is_response bit, preserving
                # the cmd_type/enc_type nibbles.
                ack_flags = (frame.flags & 0x7F) | 0x80
                self.send_v1(
                    msg_id=frame.msg_id,
                    payload=ack_payload,
                    label=f"sim_ack_{label}",
                    sender=frame.receiver,
                    receiver=frame.sender,
                    flags=ack_flags,
                    seq=frame.seq,       # ACK MUST echo the request's seq
                )
                return True
            if cmd_id == 0xF9 and not is_response:
                # Real ESC was observed (cap1) to echo F9 back with the
                # same sender/receiver/seq/payload after ~270 ms. This is
                # NOT socketcan loopback (wrong timing) — the ESC genuinely
                # re-emits the frame as a liveness response. Without this
                # echo, app.elf may consider the ESC silent on its periodic
                # state-check channel and hold perception output.
                try:
                    self.send_v1(
                        msg_id=frame.msg_id,
                        payload=frame.payload,
                        label="sim_echo_motor_state_f9",
                        sender=frame.sender,       # same, no swap
                        receiver=frame.receiver,   # same
                        flags=frame.flags,
                        seq=frame.seq,             # echo the request's seq
                    )
                except Exception as exc:
                    print(f"[sim] F9 echo failed: {exc}")
                return True
            if cmd_id in (0x80, 0x81, 0x82, 0x09, 0x0A):
                # Fire-and-forget — app.elf does not register a callback
                # (sub_3321C / sub_30208 paths). Just log and swallow.
                print(
                    f"[sim] {direction} motor_{label} cmd_id=0x{cmd_id:02X} "
                    f"payload={frame.payload.hex()} (no ack)"
                )
                return True
        # No bridge, no sim → log the payload preview so the user can see
        # what app.elf tried to send, then defer to the generic hexdump.
        print(
            f"[motor] {direction} {label} cmd_id=0x{cmd_id:02X} "
            f"payload={frame.payload.hex()}"
        )
        return False

    def _handle_can_readable(self) -> None:
        """Pump ready CAN frames through DumlStreamParser and relay
        complete DUML frames back to app.elf over UDP."""
        while True:
            try:
                can_id, dlc, data, _, _, is_err = self.can_device.recv()
            except BlockingIOError:
                break
            except OSError as exc:
                print(f"[bridge] CAN rx error: {exc}")
                break
            if is_err or not data:
                continue
            # Log every raw CAN rx chunk — this is what a pure-sim server
            # will need to emit on its virtual CAN bus when replaying.
            chunk = data[:dlc]
            self._log_can_raw(can_id, chunk)
            for duml_frame in self.can_parser.feed(chunk):
                try:
                    self.control_sock.sendto(
                        duml_frame,
                        (self.config.radar_ip, self.config.control_remote_port),
                    )
                except OSError as exc:
                    print(f"[bridge] UDP tx failed: {exc}")
                    continue
                self._log_can_duml("CAN_TO_UDP", can_id, duml_frame)
                parsed = parse_v1_frame(duml_frame)
                if parsed is None:
                    print(f"[bridge] CAN→UDP {len(duml_frame)}B (unparseable)")
                    continue
                cs = (parsed.msg_id >> 8) & 0xFF
                ci = parsed.msg_id & 0xFF
                label = MOTOR_CMD_IDS.get(ci, f"0x{ci:02X}") if cs == 0x1E else f"0x{parsed.msg_id:04X}"
                print(
                    f"[bridge] CAN→UDP msg=0x{parsed.msg_id:04X}({label}) "
                    f"len={len(parsed.payload)} seq={parsed.seq} "
                    f"sender=0x{parsed.sender:02X} receiver=0x{parsed.receiver:02X} "
                    f"flags=0x{parsed.flags:02X}"
                )

    def _tick_replay(self, now: float) -> float:
        """Emit any replay frames whose scheduled time has arrived; return
        the time (seconds) until the next replay frame is due so the outer
        select() can sleep efficiently."""
        if self._replay_origin is None:
            self._replay_origin = now
        speed = max(self.config.replay_esc_speed, 0.01)
        while self._replay_idx < len(self._replay_events):
            ev = self._replay_events[self._replay_idx]
            due = self._replay_origin + ev.rel_ts / speed
            if due > now:
                return due - now
            self._send_replay_frame(ev)
            self._replay_idx += 1
        # Hit end-of-recording — either loop or stop.
        if self.config.replay_esc_loop:
            self._replay_loops_completed += 1
            self._replay_seq_bump = (self._replay_seq_bump + 1000) & 0xFFFF
            self._replay_origin = now
            self._replay_idx = 0
            print(f"[replay] looped iteration #{self._replay_loops_completed+1} "
                  f"(seq_bump={self._replay_seq_bump})")
            return 0.0
        # No loop: disable replay so we don't keep falling through here.
        self._replay_events = []
        print("[replay] end of recording, replay disabled (--no-replay-loop)")
        return 0.25

    def _send_replay_frame(self, ev: ReplayEvent) -> None:
        """Send one ESC→app replay frame, rewriting its seq on each loop
        so app.elf doesn't dedup us against the previous pass."""
        if self._replay_seq_bump:
            frame = bytearray(ev.frame)
            orig_seq = struct.unpack_from("<H", frame, 6)[0]
            new_seq = (orig_seq + self._replay_seq_bump) & 0xFFFF
            struct.pack_into("<H", frame, 6, new_seq)
            # CRC16 covers everything except the last 2 bytes
            struct.pack_into("<H", frame, len(frame) - 2,
                             dji_crc16(bytes(frame[:-2])))
            payload = bytes(frame)
        else:
            payload = ev.frame
        try:
            self.control_sock.sendto(
                payload,
                (self.config.radar_ip, self.config.control_remote_port),
            )
        except OSError as exc:
            print(f"[replay] sendto failed: {exc}")

    def _build_sim_bottom_payload(self) -> bytes:
        """Emit the byte-precise bottom_data_pack (44B) observed from a
        real CAN-attached ESC. byte[28] is toggled by sim_esc_fix_ok so
        we can also model the 'ESC not reached target' state for tests."""
        pkt = bytearray(bytes.fromhex(SIM_ESC_BOTTOM_DATA_PACK_HEX))
        pkt[28] = 0x01 if self.config.sim_esc_fix_ok else 0x00
        return bytes(pkt)

    def _build_sim_imu_payload(self) -> bytes:
        """radar_imu_inter_pack (26B): byte[0]=0x01, rest zero — the three
        float32 fields at [14..25] decode to 0.0 which matches a static
        bench aircraft pose."""
        return bytes.fromhex(SIM_ESC_IMU_INTER_PACK_HEX)

    def _handle_data_transfer(self, frame: V1Frame) -> None:
        """Reassemble 0x1EFD data transfer packets and summarize the recovered frame."""
        pkt = parse_data_transfer_packet(frame.payload)
        if pkt is None:
            return

        dt = self.data_transfer

        if pkt["type"] == "data":
            # New frame?
            if pkt["frame_index"] != dt.current_frame_index:
                if dt.current_frame_index >= 0 and dt.received_pkgs > 0:
                    print(f"[1EFD] frame {dt.current_frame_index} incomplete ({dt.received_pkgs} pkgs), discarded")
                dt.reset(pkt["frame_index"])
                dt.expected_pkg_num = pkt["pkg_num"]
                dt.chirp_count = pkt["chirp_count"]
                dt.col_count = pkt["col_count"]
                dt.row_count = pkt["row_count"]
                dt.frame_kind = "unknown"

            valid_len = min(pkt["data_valid_num"], len(pkt["data"]))
            dt.data_buf.extend(pkt["data"][:valid_len])
            dt.received_pkgs += 1

        elif pkt["type"] == "final":
            if pkt["frame_index"] != dt.current_frame_index:
                # Orphan final packet
                print(f"[1EFD] orphan final for frame {pkt['frame_index']}")
                dt.reset()
                return

            total_bytes = len(dt.data_buf)
            if total_bytes < 1:
                print(f"[1EFD] frame {dt.current_frame_index}: too little data ({total_bytes} bytes)")
                dt.reset()
                return

            dt.frame_kind = classify_data_transfer_frame(
                dt.chirp_count,
                dt.col_count,
                dt.row_count,
                total_bytes,
            )
            summary, meta = summarize_data_transfer_frame(
                dt.frame_kind,
                bytes(dt.data_buf),
                dt.chirp_count,
                dt.col_count,
                dt.row_count,
            )

            print(
                f"[1EFD] frame {dt.current_frame_index} complete: pkgs={dt.received_pkgs}/{dt.expected_pkg_num} "
                f"bytes={total_bytes} kind={dt.frame_kind} crc32=0x{pkt['frame_crc32']:08X}"
            )
            print(summary)

            stem = f"data_transfer/frame_{dt.current_frame_index:06d}_{safe_label(dt.frame_kind)}"
            self._write_capture_bytes(f"{stem}.bin", bytes(dt.data_buf))
            self._write_capture_json(
                f"{stem}.json",
                {
                    "frame_index": dt.current_frame_index,
                    "frame_kind": dt.frame_kind,
                    "chirp_count": dt.chirp_count,
                    "col_count": dt.col_count,
                    "row_count": dt.row_count,
                    "received_pkgs": dt.received_pkgs,
                    "expected_pkg_num": dt.expected_pkg_num,
                    "total_bytes": total_bytes,
                    "frame_crc32": f"0x{pkt['frame_crc32']:08X}",
                    "summary": meta,
                },
            )
            self._write_capture_text(f"{stem}.txt", summary + "\n")

            if dt.frame_kind == "profile256_i16_shifted_u32":
                usable = min(len(dt.data_buf) // 4, 256)
                values = np.frombuffer(bytes(dt.data_buf[: usable * 4]), dtype="<u4").astype(np.float64)
                if usable < 256:
                    values = np.pad(values, (0, 256 - usable))
                elif usable > 256:
                    values = values[:256]
                if self.plotter is not None:
                    self.plotter.update_rti(values)
                self._write_capture_bytes(
                    f"rti/frame_{dt.current_frame_index:06d}.bin",
                    bytes(dt.data_buf),
                )

            dt.reset()

    def handle_v2_datagram(self, data: bytes, peer: tuple[str, int]) -> None:
        frame = parse_v2_frame(data)
        if frame is None:
            print(f"[rx/v2] invalid frame from {peer[0]}:{peer[1]} len={len(data)}")
            print(hexdump(data[:64]))
            return
        self._capture_v2_frame_raw(frame, data)
        name = MSG_NAMES.get(frame.msg_id, "unknown")
        print(
            f"[rx/v2] msg=0x{frame.msg_id:04X}({name}) len={len(frame.payload)} seq={frame.seq} "
            f"sender=0x{frame.sender:04X} receiver=0x{frame.receiver:04X} from={peer[0]}:{peer[1]}"
        )
        if frame.msg_id == 0x1E61:
            self._capture_perception_frame(frame)
            try:
                subtype, entries = decode_perception_1e61_entries(frame.payload)
            except ValueError as exc:
                print(str(exc))
            else:
                if self.plotter is not None:
                    self.plotter.update_perception(frame.seq, subtype, entries, len(frame.payload))
                print(format_perception_1e61(subtype, entries, len(frame.payload), self.config.dump_entries))
        elif frame.msg_id == 0xF000:
            self._capture_blackbox_frame(frame)
            print(parse_blackbox_f000(frame.payload))
        elif frame.payload:
            print(hexdump(frame.payload[: min(64, len(frame.payload))]))


def parse_args() -> ClientConfig:
    parser = argparse.ArgumentParser(
        description=(
            "Minimal radar client skeleton. Sends DJI V1 control traffic to 192.168.8.2:50007 and "
            "listens for DJI V2 perception traffic on local 40204."
        )
    )
    parser.add_argument("--local-ip", default="192.168.8.1")
    parser.add_argument("--radar-ip", default="192.168.8.2")
    parser.add_argument("--control-local-port", type=int, default=50007)
    parser.add_argument("--control-remote-port", type=int, default=50007)
    parser.add_argument("--v2-local-port", type=int, default=40204)
    parser.add_argument("--no-v2-listen", action="store_true",
                        help="Do not bind the V2 perception rx socket "
                             "(40204). Use when a co-located 3D viewer owns "
                             "that port to read the 1E61 point cloud.")
    parser.add_argument("--sender", type=lambda v: int(v, 0), default=0x03)
    parser.add_argument("--receiver", type=lambda v: int(v, 0), default=0x00)
    parser.add_argument("--flags", type=lambda v: int(v, 0), default=0x00)
    parser.add_argument("--capture-dir")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--plot-range-m", type=float, default=25.0)
    parser.add_argument("--plot-history-sec", type=float, default=3.0)
    parser.add_argument("--plot-refresh-hz", type=float, default=10.0)
    parser.add_argument("--plot-wire-window-sec", type=float, default=1.0,
                        help="Wire/PCLA accumulation window (sec). Lower "
                             "= more responsive to rotation, higher = more "
                             "CFAR smoothing. Default 1.0")
    parser.add_argument("--plot-ground-window-sec", type=float, default=2.0,
                        help="Ground_points density heatmap window (sec). "
                             "Default 2.0")
    parser.add_argument("--plot-terrain",
                        choices=("generic", "ground", "mountain", "fruiter"),
                        default="generic",
                        help="Terrain mode for point colouring. "
                             "'generic' keeps the coolwarm z-height colormap. "
                             "'ground': z<-0.3m grey (ground), else red (obstacle). "
                             "'fruiter': z<-2 grey (below canopy), -2..+2 green "
                             "(canopy/work zone), >2 red (obstacle). 'mountain': "
                             "no z threshold (slope-based in FC, not mimicked here).")
    parser.add_argument("--pos-hz", type=float, default=20.0)
    parser.add_argument("--debug-hz", type=float, default=2.0)
    parser.add_argument("--time-hz", type=float, default=10.0)
    parser.add_argument("--dump-entries", type=int, default=3)
    parser.add_argument("--send-motor-start", action="store_true")
    parser.add_argument("--motor-start-subid", type=lambda v: int(v, 0), default=1)
    parser.add_argument("--motor-run-rpm", type=int, default=1200,
                        help="Rotor speed setpoint (rpm) sent over CAN while "
                             "running (default 1200).")
    parser.add_argument("--motor-stop-rpm", type=int, default=50,
                        help="Rotor speed setpoint (rpm) sent at shutdown "
                             "(idle; default 50).")
    parser.add_argument("--motor-speed-hz", type=float, default=5.0,
                        help="How often to re-send the running speed setpoint "
                             "(default 5 Hz).")
    parser.add_argument("--send-sample-start", action="store_true")
    parser.add_argument("--send-sample-mark", action="store_true")
    parser.add_argument("--send-data-start-type", type=int, choices=(0, 1))
    parser.add_argument("--send-data-end-type", type=int, choices=(0, 1))
    parser.add_argument("--lat", type=float, default=0.0)
    parser.add_argument("--lon", type=float, default=0.0)
    parser.add_argument("--pos-x-m", type=float, default=0.0)
    parser.add_argument("--pos-y-m", type=float, default=0.0)
    parser.add_argument("--pos-z-m", type=float, default=0.0,
                        help="z-position in 0x1E21; NOTE: does NOT influence radar height gate (height is self-measured from ground points)")
    parser.add_argument("--vel-x-mps", type=float, default=0.0)
    parser.add_argument("--vel-y-mps", type=float, default=0.0)
    parser.add_argument("--vel-z-mps", type=float, default=0.0)
    parser.add_argument("--aux-x", type=float, default=0.0)
    parser.add_argument("--aux-y", type=float, default=0.0)
    parser.add_argument("--aux-z", type=float, default=0.0)
    parser.add_argument("--fm-ramp-sec", type=float, default=3.0,
                        help="Seconds to send flight_mode=1 before switching to "
                             "--flight-mode (default 2). Required because sub_65F10 "
                             "only enters pm3 on the 1→2 transition; static "
                             "flight_mode=2 leaves pm stuck at 1. 0 disables.")
    parser.add_argument("--flight-mode", type=lambda v: int(v, 0), default=2,
                        help="flight mode byte in 0x1E21; must be 2 or 4 to activate avoidance chain. "
                             "Height gate (1.6m) blocks avoidance for first ~50min on bench; "
                             "counter at word_CACDEB2 auto-bypasses after 300 × 10s")
    parser.add_argument("--flight-flags", type=lambda v: int(v, 0), default=0)
    parser.add_argument("--gps-valid", type=lambda v: int(v, 0), default=1)
    parser.add_argument("--pose-tag", type=lambda v: int(v, 0), default=0)
    parser.add_argument("--sim-axis", choices=("x", "y", "z"))
    parser.add_argument("--sim-amplitude-m", type=float, default=0.0)
    parser.add_argument("--sim-period-sec", type=float, default=6.0)
    parser.add_argument("--flight-debug-hex")
    parser.add_argument("--rti-history", type=int, default=200,
                        help="RTI waterfall depth in frames (default 200)")
    parser.add_argument("--rti", action="store_true",
                        help="Enable RTI waterfall mode (auto-enabled with --plot --send-data-start-type)")
    # FC-bridge / motor-sim
    bridge = parser.add_argument_group(
        "fc-bridge",
        "Bridge app.elf's motor DUML (cmd_set=0x1E) to a real ESC over CAN "
        "OR simulate ESC feedback in software (for bench without FC/ESC).",
    )
    bridge_mode = bridge.add_mutually_exclusive_group()
    bridge_mode.add_argument(
        "--can-bridge", metavar="IFACE",
        help="SocketCAN interface to bridge UDP↔CAN (e.g. can0). Forwards "
             "motor DUML from UDP to CAN verbatim and relays reassembled "
             "ESC CAN DUML back to app.elf via UDP.",
    )
    bridge_mode.add_argument(
        "--sim-esc", action="store_true",
        help="Simulate ESC entirely in software: emit the byte-precise "
             "periodic stream a real T40 ESC was observed to push "
             "(bottom_data_pack, imu_inter_pack, EA keepalive, activation "
             "reply, 0x0E/0x01 diag), plus synthesize hms_switch ACK on "
             "demand. Replaces the older --sim-bottom which was "
             "insufficient for app.elf to fully accept the ESC as healthy.",
    )
    bridge_mode.add_argument(
        "--sim-bottom", dest="sim_esc_alias", action="store_true",
        help="DEPRECATED alias for --sim-esc (same behaviour).",
    )
    bridge_mode.add_argument(
        "--replay-esc", metavar="DUML_JSONL",
        help="Replay a captured can/duml.jsonl (from a prior --can-bridge "
             "+ --capture-dir run) back at app.elf. Highest-fidelity "
             "option since it's a byte-exact replay of real ESC output. "
             "hms_switch ACKs are synthesized on-demand since the "
             "recording typically won't cover them.",
    )
    bridge.add_argument("--can-tx-id", type=lambda v: int(v, 0), default=0x0A2,
                        help="CAN id for our tx (default 0x0A2 = ESC rx, "
                             "from decode_link_id(0x520518A2))")
    bridge.add_argument("--can-rx-id", type=lambda v: int(v, 0), default=0x0A3,
                        help="CAN id we listen on (default 0x0A3 = ESC tx)")
    bridge.add_argument("--sim-esc-fix-bad", action="store_true",
                        help="Mark bottom_data_pack byte[28]=0 instead of 1 "
                             "(simulate ESC-not-reached-target fault)")
    bridge.add_argument("--replay-esc-speed", type=float, default=1.0,
                        help="Playback speed multiplier for --replay-esc "
                             "(1.0 = original recording timing)")
    bridge.add_argument("--no-replay-loop", action="store_true",
                        help="Play the recording once and stop (default: "
                             "loop forever until Ctrl-C)")
    bridge.add_argument("--replay-only-motor", action="store_true",
                        help="Only replay cmd_set=0x1E frames (skip the "
                             "activation / 0x0E diagnostic / 0x1E/0xEA "
                             "keepalive from the recording)")
    args = parser.parse_args()
    return ClientConfig(
        local_ip=args.local_ip,
        radar_ip=args.radar_ip,
        control_local_port=args.control_local_port,
        control_remote_port=args.control_remote_port,
        v2_local_port=args.v2_local_port,
        sender=args.sender,
        receiver=args.receiver,
        flags=args.flags,
        capture_dir=args.capture_dir,
        plot=args.plot,
        plot_range_m=args.plot_range_m,
        plot_history_sec=args.plot_history_sec,
        plot_refresh_hz=args.plot_refresh_hz,
        plot_wire_window_sec=args.plot_wire_window_sec,
        plot_ground_window_sec=args.plot_ground_window_sec,
        plot_terrain=args.plot_terrain,
        pos_hz=args.pos_hz,
        debug_hz=args.debug_hz,
        time_hz=args.time_hz,
        dump_entries=args.dump_entries,
        send_motor_start=args.send_motor_start,
        motor_start_subid=args.motor_start_subid,
        motor_run_rpm=args.motor_run_rpm,
        motor_stop_rpm=args.motor_stop_rpm,
        motor_speed_hz=args.motor_speed_hz,
        send_sample_start=args.send_sample_start,
        send_sample_mark=args.send_sample_mark,
        send_data_start_type=args.send_data_start_type,
        send_data_end_type=args.send_data_end_type,
        lat_deg=args.lat,
        lon_deg=args.lon,
        pos_x_m=args.pos_x_m,
        pos_y_m=args.pos_y_m,
        pos_z_m=args.pos_z_m,
        vel_x_mps=args.vel_x_mps,
        vel_y_mps=args.vel_y_mps,
        vel_z_mps=args.vel_z_mps,
        aux_x=args.aux_x,
        aux_y=args.aux_y,
        aux_z=args.aux_z,
        flight_mode=args.flight_mode,
        fm_ramp_sec=args.fm_ramp_sec,
        flight_flags=args.flight_flags,
        gps_valid=args.gps_valid,
        pose_tag=args.pose_tag,
        sim_axis=args.sim_axis,
        sim_amplitude_m=args.sim_amplitude_m,
        sim_period_sec=args.sim_period_sec,
        flight_debug_hex=args.flight_debug_hex,
        rti_history=args.rti_history,
        rti_mode=args.rti or (args.plot and args.send_data_start_type is not None),
        can_bridge=args.can_bridge,
        can_tx_id=args.can_tx_id,
        can_rx_id=args.can_rx_id,
        # --sim-bottom is a deprecated alias for --sim-esc
        sim_esc=args.sim_esc or args.sim_esc_alias,
        sim_esc_fix_ok=not args.sim_esc_fix_bad,
        replay_esc=args.replay_esc,
        replay_esc_loop=not args.no_replay_loop,
        replay_esc_speed=args.replay_esc_speed,
        replay_esc_only_motor=args.replay_only_motor,
        v2_listen=not args.no_v2_listen,
    )


def main() -> int:
    config = parse_args()
    print("[cfg] minimal radar client skeleton")
    print(
        f"[cfg] control tx/rx bind={config.local_ip}:{config.control_local_port} -> "
        f"{config.radar_ip}:{config.control_remote_port}"
    )
    if config.v2_listen and config.v2_local_port > 0:
        print(f"[cfg] v2 rx bind={config.local_ip}:{config.v2_local_port}")
    else:
        print("[cfg] v2 rx disabled (port 40204 left free for a co-located viewer)")
    print(
        f"[cfg] sender=0x{config.sender:02X} receiver=0x{config.receiver:02X} flags=0x{config.flags:02X} "
        f"pos_hz={config.pos_hz} debug_hz={config.debug_hz} time_hz={config.time_hz} "
        f"flight_mode={config.flight_mode}"
    )
    if config.capture_dir:
        print(f"[cfg] capture_base={config.capture_dir}")
    if config.plot:
        print(
            f"[cfg] live_plot=on range_m={config.plot_range_m} history_sec={config.plot_history_sec} "
            f"refresh_hz={config.plot_refresh_hz}"
        )
    print(
        f"[cfg] lat={config.lat_deg} lon={config.lon_deg} "
        f"pos=({config.pos_x_m}, {config.pos_y_m}, {config.pos_z_m}) "
        f"vel=({config.vel_x_mps}, {config.vel_y_mps}, {config.vel_z_mps}) gps_valid={config.gps_valid}"
    )
    if config.sim_axis and config.sim_amplitude_m > 0.0:
        print(
            f"[cfg] sim_motion axis={config.sim_axis} amplitude_m={config.sim_amplitude_m} "
            f"period_sec={config.sim_period_sec}"
        )
    if config.send_data_start_type is not None or config.send_data_end_type is not None:
        print(
            f"[cfg] data_req start_type={config.send_data_start_type} end_type={config.send_data_end_type}"
        )
    if config.rti_mode:
        print(f"[cfg] rti_mode=on history={config.rti_history} frames")
    if config.can_bridge:
        print(
            f"[cfg] can_bridge=on iface={config.can_bridge} "
            f"tx=0x{config.can_tx_id:03X} rx=0x{config.can_rx_id:03X}"
        )
    if config.sim_esc:
        print(
            f"[cfg] sim_esc=on (bottom + imu @ 4.3Hz, keepalive/activation/diag @ 0.5Hz) "
            f"fix_ok={config.sim_esc_fix_ok}"
        )
    try:
        client = RadarClient(config)
    except OSError as exc:
        print(f"[err] socket setup failed: {exc}")
        print("[err] ensure the host owns the configured local IP, especially 192.168.8.1 for 40204 reception")
        return 1
    if client.capture_root is not None:
        print(f"[cfg] capture_session={client.capture_root}")
    # Route SIGTERM through the same KeyboardInterrupt path as Ctrl-C so
    # both `kill <pid>` and interactive interrupt trigger client.close()
    # (which sends the motor_stop_rpm idle setpoint on CAN, flushes logs,
    # closes sockets).
    import signal
    def _raise_kbd_interrupt(signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _raise_kbd_interrupt)
    try:
        client.run()
    except KeyboardInterrupt:
        print("\n[info] stopped, running graceful shutdown...")
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
