#!/usr/bin/env python3
"""DJI DUML V1 frame generator for T40 Radar ESC (GD32F305)"""

import struct
import sys

# CRC tables from firmware ROM
_T8 = [
    0,94,188,226,97,63,221,131,194,156,126,32,163,253,31,65,
    157,195,33,127,252,162,64,30,95,1,227,189,62,96,130,220,
    35,125,159,193,66,28,254,160,225,191,93,3,128,222,60,98,
    190,224,2,92,223,129,99,61,124,34,192,158,29,67,161,255,
    70,24,250,164,39,121,155,197,132,218,56,102,229,187,89,7,
    219,133,103,57,186,228,6,88,25,71,165,251,120,38,196,154,
    101,59,217,135,4,90,184,230,167,249,27,69,198,152,122,36,
    248,166,68,26,153,199,37,123,58,100,134,216,91,5,231,185,
    140,210,48,110,237,179,81,15,78,16,242,172,47,113,147,205,
    17,79,173,243,112,46,204,146,211,141,111,49,178,236,14,80,
    175,241,19,77,206,144,114,44,109,51,209,143,12,82,176,238,
    50,108,142,208,83,13,239,177,240,174,76,18,145,207,45,115,
    202,148,118,40,171,245,23,73,8,86,180,234,105,55,213,139,
    87,9,235,181,54,104,138,212,149,203,41,119,244,170,72,22,
    233,183,85,11,136,214,52,106,43,117,151,201,74,20,246,168,
    116,42,200,150,21,75,169,247,182,232,10,84,215,137,107,53,
]
_T16 = [
    0,4489,8978,12955,17956,22445,25910,29887,35912,40385,44890,48851,
    51820,56293,59774,63735,4225,264,13203,8730,22181,18220,30135,25662,
    40137,36160,49115,44626,56045,52068,63999,59510,8450,12427,528,5017,
    26406,30383,17460,21949,44362,48323,36440,40913,60270,64231,51324,55797,
    12675,8202,4753,792,30631,26158,21685,17724,48587,44098,40665,36688,
    64495,60006,55549,51572,16900,21389,24854,28831,1056,5545,10034,14011,
    52812,57285,60766,64727,34920,39393,43898,47859,21125,17164,29079,24606,
    5281,1320,14259,9786,57037,53060,64991,60502,39145,35168,48123,43634,
    25350,29327,16404,20893,9506,13483,1584,6073,61262,65223,52316,56789,
    43370,47331,35448,39921,29575,25102,20629,16668,13731,9258,5809,1848,
    65487,60998,56541,52564,47595,43106,39673,35696,33800,38273,42778,46739,
    49708,54181,57662,61623,2112,6601,11090,15067,20068,24557,28022,31999,
    38025,34048,47003,42514,53933,49956,61887,57398,6337,2376,15315,10842,
    24293,20332,32247,27774,42250,46211,34328,38801,58158,62119,49212,53685,
    10562,14539,2640,7129,28518,32495,19572,24061,46475,41986,38553,34576,
    62383,57894,53437,49460,14787,10314,6865,2904,32743,28270,23797,19836,
    50700,55173,58654,62615,32808,37281,41786,45747,19012,23501,26966,30943,
    3168,7657,12146,16123,54925,50948,62879,58390,37033,33056,46011,41522,
    23237,19276,31191,26718,7393,3432,16371,11898,59150,63111,50204,54677,
    41258,45219,33336,37809,27462,31439,18516,23005,11618,15595,3696,8185,
    63375,58886,54429,50452,45483,40994,37561,33584,31687,27214,22741,18780,
    15843,11370,7921,3960,
]

def _crc8(data: bytes) -> int:
    c = 0x77
    for b in data:
        c = _T8[c ^ b]
    return c

def _crc16(data: bytes) -> int:
    c = 0x3692
    for b in data:
        c = _T16[(c & 0xFF) ^ b] ^ (c >> 8)
    return c

def build_v1(src_type, src_id, dst_type, dst_id, seq, cmd_set, cmd_id,
             payload=b"", need_ack=True):
    """Build a DUML V1 request frame.

    Returns bytes of the complete frame ready to send.
    """
    total = 11 + len(payload) + 2
    if total > 1023:
        raise ValueError("frame too large (%d > 1023)" % total)

    hdr = bytes([0x55, total & 0xFF, ((total >> 8) & 0x03) | 0x04])
    crc8 = _crc8(hdr)

    flags = 0x40 if need_ack else 0x00
    body = bytes([
        crc8,
        (src_type << 5) | (src_id & 0x1F),
        (dst_type << 5) | (dst_id & 0x1F),
        seq & 0xFF, (seq >> 8) & 0xFF,
        flags,
        cmd_set & 0xFF,
        cmd_id & 0xFF,
    ]) + payload

    frame = hdr + body
    c16 = _crc16(frame)
    frame += struct.pack("<H", c16)
    return frame

def hex_str(data: bytes) -> str:
    return " ".join("%02X" % b for b in data)

# ── ESC self-id = 0x18, default sender = PC (type=1, id=6) ──

CMDS = [
    # (name, cmd_set, cmd_id, default_payload, description)
    # --- cmd_set 0x00 general ---
    ("echo",            0x00, 0x00, b"\x41",  "Echo (payload='A')"),
    ("get_device_info", 0x00, 0x01, b"",      "Get device info"),
    ("upgrade",         0x00, 0x07, b"",      "Upgrade entry"),
    ("reboot",          0x00, 0x0B, b"",      "Reboot"),
    ("heartbeat",       0x00, 0x0C, b"",      "Heartbeat"),
    ("activation",      0x00, 0x32, b"",      "Activation"),
    ("push_stub",       0x00, 0x50, b"",      "Push stub"),
    ("sn_get",          0x00, 0x51, b"",      "Get product SN"),
    ("get_cfg_info",    0x00, 0xFF, b"",      "Get config info"),
    # --- cmd_set 0x03 product ---
    ("sn_set_stub",     0x03, 0x73, b"",      "Set product SN (stub)"),
    ("sn_read",         0x03, 0x74, b"",      "Read product SN"),
    # --- cmd_set 0x10 RDTT ---
    ("rdtt_negotiate",  0x10, 0x80, b"",      "RDTT link negotiate"),
    ("rdtt_send_test",  0x10, 0x82, b"",      "RDTT send test"),
    # --- cmd_set 0x1E ESC/motor/radar ---
    ("motor_pwm",       0x1E, 0x01, b"",      "Motor PWM control"),
    ("motor_pid",       0x1E, 0x07, b"",      "Motor PID params"),
    ("motor_enable",    0x1E, 0x09, b"",      "Motor enable"),
    ("motor_disable",   0x1E, 0x0A, b"",      "Motor disable"),
    ("angle_sync",      0x1E, 0x0D, b"",      "Angle sync ack"),
    ("flyinfo",         0x1E, 0x21, b"",      "Fly info handler"),
    ("motor_speed",     0x1E, 0x80, b"",      "Motor speed set"),
    ("motor_position",  0x1E, 0x81, b"",      "Motor position"),
    ("motor_stop",      0x1E, 0x82, b"",      "Motor stop"),
    ("hms_switch",      0x1E, 0x83, b"",      "HMS switch"),
    ("esc_e5_stub",     0x1E, 0xE5, b"",      "ESC E5 stub"),
    ("ar1021x_read",    0x1E, 0xE6, b"",      "AR1021x flash read"),
    ("esc_ea_stub",     0x1E, 0xEA, b"",      "ESC EA stub"),
    ("esc_config_set",  0x1E, 0xEB, b"",      "ESC config set"),
    ("rdtt_status",     0x1E, 0xFE, b"",      "RDTT status request"),
]

_CMD_NAMES = {(cs, ci): (name, desc) for name, cs, ci, _, desc in CMDS}

_TYPE_NAMES = {0: "DEV", 1: "PC", 2: "WL2", 3: "RADAR", 7: "AGENT"}
_ID_NAMES = {
    0x00: "default", 0x02: "app", 0x03: "fc", 0x05: "center",
    0x08: "1860", 0x0A: "pc_assist", 0x11: "autotest", 0x13: "nego_s",
    0x14: "nego_h", 0x18: "esc", 0x1D: "hms", 0x1F: "broadcast",
}

def parse_frame(data: bytes):
    """Parse and validate a DUML V1 frame. Returns (info_dict, errors_list)."""
    errors = []
    info = {}

    if len(data) < 4:
        errors.append("too short for header (%d bytes)" % len(data))
        return info, errors

    if data[0] != 0x55:
        errors.append("SOF=0x%02X, expected 0x55" % data[0])

    total_len = ((data[2] & 0x03) << 8) | data[1]
    version = data[2] >> 2
    info["length"] = total_len
    info["version"] = version

    # CRC8
    crc8_ok = _crc8(data[:4]) == 0
    info["crc8_ok"] = crc8_ok
    if not crc8_ok:
        expected = _crc8(data[:3])
        errors.append("CRC8 FAIL: byte[3]=0x%02X, expected 0x%02X" % (data[3], expected))

    if version != 1:
        errors.append("version=%d, expected 1" % version)

    if total_len != len(data):
        errors.append("length field=%d, actual=%d" % (total_len, len(data)))

    if total_len < 13:
        errors.append("length %d < minimum 13" % total_len)

    if len(data) < 13:
        errors.append("frame too short to parse fields (%d bytes)" % len(data))
        return info, errors

    # CRC16
    crc16_ok = _crc16(data) == 0
    info["crc16_ok"] = crc16_ok
    if not crc16_ok:
        expected = _crc16(data[:-2])
        actual = data[-2] | (data[-1] << 8)
        errors.append("CRC16 FAIL: got 0x%04X, expected 0x%04X" % (actual, expected))

    # Fields
    info["sender_id"]     = data[4] & 0x1F
    info["sender_type"]   = data[4] >> 5
    info["receiver_id"]   = data[5] & 0x1F
    info["receiver_type"] = data[5] >> 5
    info["seq"]           = data[6] | (data[7] << 8)
    info["flags"]         = data[8]
    info["need_ack"]      = bool(data[8] & 0x40)
    info["is_ack"]        = bool(data[8] & 0x80)
    info["cmd_set"]       = data[9]
    info["cmd_id"]        = data[10]

    payload_len = total_len - 13
    info["payload"] = data[11:11 + payload_len] if payload_len > 0 else b""

    # Lookup command name
    key = (info["cmd_set"], info["cmd_id"])
    if key in _CMD_NAMES:
        info["cmd_name"], info["cmd_desc"] = _CMD_NAMES[key]
    else:
        info["cmd_name"] = "unknown"
        info["cmd_desc"] = ""

    return info, errors

def format_parsed(data: bytes) -> str:
    """Return a human-readable string of a parsed frame."""
    info, errors = parse_frame(data)
    lines = []
    lines.append("Raw: %s" % hex_str(data))
    lines.append("Len: %d bytes" % len(data))

    if "version" in info:
        lines.append("Ver: %d  Length field: %d" % (info["version"], info["length"]))

    crc8_tag = "OK" if info.get("crc8_ok") else "FAIL"
    crc16_tag = "OK" if info.get("crc16_ok") else "FAIL"
    lines.append("CRC: CRC8=%s  CRC16=%s" % (crc8_tag, crc16_tag))

    if "sender_id" in info:
        st = info["sender_type"]
        si = info["sender_id"]
        rt = info["receiver_type"]
        ri = info["receiver_id"]
        stn = _TYPE_NAMES.get(st, "?")
        sin = _ID_NAMES.get(si, "")
        rtn = _TYPE_NAMES.get(rt, "?")
        rin = _ID_NAMES.get(ri, "")
        lines.append("Src: type=%d(%s) id=%d(0x%02X) %s" % (st, stn, si, si, sin))
        lines.append("Dst: type=%d(%s) id=%d(0x%02X) %s" % (rt, rtn, ri, ri, rin))
        lines.append("Seq: %d" % info["seq"])

        flag_parts = []
        if info["need_ack"]: flag_parts.append("NEED_ACK")
        if info["is_ack"]:   flag_parts.append("IS_ACK")
        lines.append("Flg: 0x%02X %s" % (info["flags"], " ".join(flag_parts)))

        cs, ci = info["cmd_set"], info["cmd_id"]
        cname = info["cmd_name"]
        lines.append("Cmd: set=0x%02X id=0x%02X  %s" % (cs, ci, cname))

        pl = info["payload"]
        if pl:
            lines.append("Pay: (%d bytes) %s" % (len(pl), hex_str(pl)))
            # Try ASCII
            if all(0x20 <= b < 0x7F for b in pl):
                lines.append("     ASCII: %s" % pl.decode("ascii"))
        else:
            lines.append("Pay: (empty)")

    if errors:
        for e in errors:
            lines.append("ERR: %s" % e)

    return "\n".join(lines)

def parse_multi(data: bytes) -> list:
    """Scan a byte stream for DUML frames (SOF=0x55). Returns list of (offset, frame_bytes)."""
    frames = []
    i = 0
    while i < len(data):
        if data[i] != 0x55:
            i += 1
            continue
        if i + 3 > len(data):
            break
        total = ((data[i + 2] & 0x03) << 8) | data[i + 1]
        if total < 13 or i + total > len(data):
            i += 1
            continue
        frame = data[i:i + total]
        if _crc8(frame[:4]) == 0 and _crc16(frame) == 0:
            frames.append((i, frame))
            i += total
        else:
            i += 1
    return frames

def gen_all(dst_id=0x18, dst_type=0, src_type=1, src_id=6):
    """Generate frames for all known commands."""
    for i, (name, cs, ci, pl, desc) in enumerate(CMDS):
        frame = build_v1(src_type, src_id, dst_type, dst_id,
                         seq=i+1, cmd_set=cs, cmd_id=ci, payload=pl)
        print("%-18s  set=%02X id=%02X  %s" % (name, cs, ci, hex_str(frame)))

def main():
    import argparse
    p = argparse.ArgumentParser(description="DJI DUML V1 frame generator")
    sub = p.add_subparsers(dest="cmd")

    # list all
    sub.add_parser("list", help="List all known commands with hex frames")

    # gen single
    g = sub.add_parser("gen", help="Generate a single frame")
    g.add_argument("name", help="Command name (or 'custom')")
    g.add_argument("--dst-id",   type=lambda x: int(x, 0), default=0x18)
    g.add_argument("--dst-type", type=lambda x: int(x, 0), default=0)
    g.add_argument("--src-id",   type=lambda x: int(x, 0), default=6)
    g.add_argument("--src-type", type=lambda x: int(x, 0), default=1)
    g.add_argument("--seq",      type=lambda x: int(x, 0), default=1)
    g.add_argument("--payload",  type=str, default="",
                   help="Hex payload e.g. '41 42 43'")
    g.add_argument("--cmd-set",  type=lambda x: int(x, 0), default=None)
    g.add_argument("--cmd-id",   type=lambda x: int(x, 0), default=None)
    g.add_argument("--no-ack",   action="store_true")

    # parse
    pa = sub.add_parser("parse", help="Parse and validate DUML frame(s)")
    pa.add_argument("hex_data", nargs="?", default=None,
                    help="Hex string e.g. '55 0E 04 66 ...' (or read from --file)")
    pa.add_argument("--file", "-f", type=str, default=None,
                    help="Read raw binary from file and scan for all frames")

    # batch: output binary file
    b = sub.add_parser("batch", help="Output all frames as raw binary (one per line to stdout)")

    args = p.parse_args()

    if args.cmd == "list" or args.cmd is None:
        gen_all()
        return

    if args.cmd == "parse":
        if args.file:
            with open(args.file, "rb") as fh:
                raw = fh.read()
            frames = parse_multi(raw)
            if not frames:
                print("No valid DUML frames found in file.")
            for idx, (offset, frame) in enumerate(frames):
                print("=== Frame %d @ offset %d (0x%X) ===" % (idx, offset, offset))
                print(format_parsed(frame))
                print()
        elif args.hex_data:
            raw = bytes.fromhex(args.hex_data.replace(" ", "").replace(",", ""))
            # Could be one frame or concatenated frames
            frames = parse_multi(raw)
            if len(frames) <= 1:
                # Parse as single frame (show errors even if CRC fails)
                print(format_parsed(raw))
            else:
                for idx, (offset, frame) in enumerate(frames):
                    print("=== Frame %d @ offset %d ===" % (idx, offset))
                    print(format_parsed(frame))
                    print()
        else:
            # Read from stdin
            raw = sys.stdin.buffer.read()
            frames = parse_multi(raw)
            if not frames:
                print("No valid DUML frames found.")
            for idx, (offset, frame) in enumerate(frames):
                print("=== Frame %d @ offset %d ===" % (idx, offset))
                print(format_parsed(frame))
                print()
        return

    if args.cmd == "batch":
        for i, (name, cs, ci, pl, _) in enumerate(CMDS):
            frame = build_v1(1, 6, 0, 0x18, seq=i+1, cmd_set=cs, cmd_id=ci, payload=pl)
            sys.stdout.buffer.write(frame)
        return

    if args.cmd == "gen":
        payload = bytes.fromhex(args.payload.replace(" ", "")) if args.payload else b""

        if args.name == "custom":
            if args.cmd_set is None or args.cmd_id is None:
                p.error("custom requires --cmd-set and --cmd-id")
            cs, ci = args.cmd_set, args.cmd_id
        else:
            match = [c for c in CMDS if c[0] == args.name]
            if not match:
                print("Unknown command '%s'. Available:" % args.name)
                for c in CMDS:
                    print("  %-18s  set=0x%02X id=0x%02X  %s" % (c[0], c[1], c[2], c[4]))
                sys.exit(1)
            _, cs, ci, default_pl, desc = match[0]
            if not payload:
                payload = default_pl

        frame = build_v1(args.src_type, args.src_id, args.dst_type, args.dst_id,
                         seq=args.seq, cmd_set=cs, cmd_id=ci,
                         payload=payload, need_ack=not args.no_ack)
        print(hex_str(frame))

if __name__ == "__main__":
    main()
