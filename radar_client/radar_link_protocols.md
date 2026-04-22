# Radar Link Protocol Notes

Scope: UDP protocols between the radar's app.elf (Zynq AP) and the flight
controller (FC). Source: static IDA reversal of app.elf + cross-checked
against FC-side `dji_perception`. Sibling doc `DUML_V1_Protocol.md` covers
the DUML wire format used on the ESC↔app.elf CAN path.

## 1. Transport

`linux_net_init` registers two UDP endpoints on the radar:

| endpoint | local              | remote             | device id  | role                  |
| -------- | ------------------ | ------------------ | ---------- | --------------------- |
| #0       | 192.168.8.2:50007  | learned at runtime | 0xC00000   | V1 control plane      |
| #1       | 192.168.8.2:40204  | 192.168.8.1:40204  | 0xC00001   | V2 uplink (perception)|

- **50007** carries flight-control ingress (`0x1E21`, `0x1E22`, `0x1E25`…)
  and periodic outbound fusion results (`0x1E62`, `0x1E32`).
- **40204** carries two uplink streams: `perception_over_net_v2`
  (msg `0x1E61`) and `blackbox_over_net_v2` (msg `0x00F0`).

---

## 2. 40204 — V2 link

### 2.1 Static routes

`link_drv_init` registers these routes against device `0xC00001`:

| route key    | mask         | name                      | purpose                       |
| ------------ | ------------ | ------------------------- | ----------------------------- |
| `0x06120400` | `0xFFFFFF00` | `perception_over_net_v2`  | perception results            |
| `0x061D0000` | `0xFFFFFF00` | `blackbox_over_net_v2`    | blackbox / chunked dumps      |

Key = `(protocol_type << 24) | (internal[4] << 16) | (internal[5] << 8)`.

### 2.2 Wire format

Parsers: `dji_v2_check_preamble_and_hdr_crc`, `dji_v2_unpack_frame`,
`dji_v2_pack_frame`, `dji_v2_check_crc16`.

| offset     | contents                                  |
| ---------- | ----------------------------------------- |
| +0         | `0x55` SOF                                |
| +1         | low 6 bits = `0x18`                       |
| +2         | `(byte2 & 0xFC) == 0x08`                  |
| +3..+4     | uint16 total_len                          |
| +7..+10    | uint32 seq                                |
| +12..+13   | endpoint field 1                          |
| +18..+19   | endpoint field 2                          |
| +23        | header CRC8 (over +0..+22)                |
| +24        | msg_hi                                    |
| +25        | msg_lo                                    |
| +27..      | payload                                   |
| tail-2..-1 | CRC16                                     |

Internal message object after unpack:
- `internal[8] = wire[24]`, `internal[9] = wire[25]`
- `internal[20] = 6`
- `internal[28..31] = seq`
- `internal[32..35] = payload_ptr`
- `internal[12..13] = total_len - 29`

### 2.3 `perception_over_net_v2` — msg `0x1E61`

Sender `perception_v2_send_msg_1E61`. Payload = 69-byte header +
`N × 16`-byte records.

Header (69 B):
- +0..+2: sequence counters
- +3: record count `N`
- +5: packed flags
- +7..+66: state snapshot (session id, timestamps, sensor mode, calibration
  flags, drone-state bits) — copied verbatim from caller context
- +65: **subtype (u16)** — 0 = ground, 1 = wire, 2 = pcla
- +67..+68: reserved

16-byte record (all three subtypes use the same layout `<H f f f H>`):
- +0  u16 id
- +2  f32 x (body forward, m)
- +6  f32 y (body right, m)
- +10 f32 z (body up, m)
- +14 u16 metric

Per-subtype semantics (source = side rotating array for all three):

| subtype | builder                                                  | `id`                       | `metric`                         |
| ------- | -------------------------------------------------------- | -------------------------- | -------------------------------- |
| 0       | `ground_points_build_and_send_perception_msg_1E61`       | 0 (never set)              | 0 (never set)                    |
| 1       | `wire_candidates_build_and_send_perception_msg_1E61`     | tracker slot id            | scaled reflection strength       |
| 2       | `pcla_build_and_send_perception_msg_1E61`                | **azimuth sector index**   | scaled per-point strength        |

Notes:
- "ground_points" is a firmware naming artifact — subtype 0 is the
  side-rotating array's below-horizon beam output, not the top antenna.
  Top 4×4 does **not** appear on the perception UDP channel; its only
  network-visible output is the height tail in 1E62/1E32.
- Wire `id` is **not per-target tracking** — the tracker reuses slot
  ids, so the same id can jump to unrelated coordinates across frames.
- PCLA `id` ∈ 1..22 is an azimuth bin, not a class label; multiple
  records in a frame share the same id.

### 2.4 `blackbox_over_net_v2` — msg `0x00F0`

Sender `blackbox_v2_send_msg_F000`. `task_pcl_blackbox_publish` packs
point-cloud / target data into `byte_C442FB8` and sends it as
fragments; the last fragment appends `frame_crc32`. Each fragment
carries up to ~5000 points × 12 bytes (xyz in mm + angles).

---

## 3. 50007 — V1 control plane

### 3.1 Static routes

`0xC00000` routes (subset):

| key          | mask         | name                           | purpose                          |
| ------------ | ------------ | ------------------------------ | -------------------------------- |
| `0x00030000` | `0x00FF0000` | `fc_over_bottom`               | flight controller                |
| `0x00020000` | `0x00FF0000` | `app_over_bottom`              | app                              |
| `0x00110000` | `0x00FF0000` | `mvo_over_net`                 | visual odometry                  |
| `0x00080000` | `0x00FF0000` | `1860_over_net`                | 1860 OFDM module                 |
| `0x001D0000` | `0x00FFFF00` | `blackbox_over_net`            | legacy blackbox                  |
| `0x001D0700` | `0x00FFFF00` | `dji_hms_over_net`             | HMS                              |
| `0x00180000` | `0x00FFFF00` | `esc_over_bottom`              | ESC                              |
| `0x00180400` | `0x00FFFF00` | `rear_radar_over_self`         | rear radar                       |
| `0x00180500` | `0x00FFFF00` | `btm_fixed_radar_over_self`    | bottom fixed radar               |
| `0x000A0000` | `0x00FFFF00` | `pc_over_net`                  | PC debug                         |

50007 has no static peer — remote address is learned at runtime.

### 3.2 Wire format

Parsers: `dji_v1_check_preamble_and_hdr_crc`, `dji_v1_unpack_frame`,
`dji_v1_pack_frame`, `dji_v1_check_crc16`.

| offset     | contents                                  |
| ---------- | ----------------------------------------- |
| +0         | `0x55` SOF                                |
| +1..+2     | len10                                     |
| +3         | header CRC8                               |
| +4         | packed endpoint 1 (sender: `(type<<5)\|id`)|
| +5         | packed endpoint 2 (receiver)              |
| +6..+7     | uint16 seq                                |
| +8         | flags / ack byte                          |
| +9         | msg_hi                                    |
| +10        | msg_lo                                    |
| +11..      | payload                                   |
| tail-2..-1 | CRC16                                     |

Internal message object:
- `internal[0] = wire[4] & 0x1F` (sender id)
- `internal[1] = wire[4] >> 5`   (sender type)
- `internal[4] = wire[5] & 0x1F` (receiver id)
- `internal[5] = wire[5] >> 5`   (receiver type)
- `internal[8] = wire[9]`, `internal[9] = wire[10]`
- `internal[10] = wire[8] & 0xF`, `internal[11] = (wire[8] >> 5) & 3`
- `internal[18] = wire[8] >> 7` (ack/request bit)
- `internal[20] = 1`
- `internal[28..31] = seq`
- `internal[12..13] = total_len - 13`

Reply helper `sub_137D4` swaps sender/receiver and keeps the msg id.
`link_host_device_rx_callback` dispatches ACK packets separately from
regular handlers.

Handler table key = `(protocol_type << 24) | (msg_hi << 8) | msg_lo`.
All V1 keys are `0x0100xxxx`.

### 3.3 Inbound handlers (FC → radar)

| key          | msg id | handler                              | payload | purpose                         |
| ------------ | ------ | ------------------------------------ | ------- | ------------------------------- |
| `0x01001E21` | 0x1E21 | `cmd_flight_pos_handler`             | 66 B    | flight pose / kinematics         |
| `0x01001E22` | 0x1E22 | `cmd_flight_ctrl_handler`            | 7 B     | flight control (motor gates)    |
| `0x01001E25` | 0x1E25 | `cmd_flight_debug_handler`           | 38 B    | flight debug state              |
| `0x01001E26` | 0x1E26 | `cmd_sample_ctrl_handler`            | 18 B    | sample start/stop/mark          |
| `0x01001E27` | 0x1E27 | `cmd_radar_time_sync_handler`        | 82 B    | time sync                       |
| `0x01001E0B` | 0x1E0B | `cmd_radar_imu_inter_pack_handler`   | 26 B    | radar IMU interpack             |
| `0x01001EFC` | 0x1EFC | `cmd_handler_req_or_terminate_data`  | dynamic | start/end data transfer         |
| `0x01001E52` | 0x1E52 | `cmd_get_rear_radar_result`          | 62 B    | rear radar result (fixed_board) |
| `0x01001E51` | 0x1E51 | `cmd_get_bottom_radar_result`        | 88 B    | bottom fixed-radar result       |

### 3.4 Handler details

#### `0x1E21` flight_pos

- Length check = 66, NaN check on several floats/doubles
- `payload[0..15]`: 4× f32 quaternion
- `payload[16..27]`: 6× int16 position + velocity (×0.001 m/mps)
- `payload[28..33]`: 3× int16 aux (×0.01)
- `payload[38]`: flight mode → `g_flight_mode_from_fc` (byte_283295)
- `payload[39]`: bitfield — bit0 `unk_283294`, bit3 `unk_283297`
- `payload[40..55]`: double latitude / longitude
- `payload[56]`: GPS valid byte
- `payload[57..64]`: 8-byte time
- `payload[65]`: pose tag

Side effects: sets `unk_28453C = 1` (FC keepalive flag, watched by
`monitor_fc_uart_keepalive_and_disconnect`), updates height-fusion
state, writes pose snapshot used by downstream transforms.

#### `0x1E22` flight_ctrl

Length = 7. `payload[0]` is sub_id; sub_id 1 or 3 calls
`set_motor_state_from_fc(payload[3])` which in turn emits an outbound
DUML `cmd_set=0x1E cmd_id=0x83` (hms_switch) with a 500 ms ACK
timeout. The ACK handler writes `unk_282DEC = 2` (motor run-gate
open) when payload[1] != 0.

#### `0x1E25` flight_debug

Length = 38. The handler splits the 38-byte payload into dwords and
bitfields written to `dword_283208..dword_283220`,
`byte_283224..byte_283234`. The key consumer is the `pitch_correction`
flag at `payload[31] bit1`. Bytes `payload[28..33]` also select the
DFA profile read by `sub_4B10C` (values land in `C2CF810..C2CF81D`).
After a valid 0x1E25 is received, the radar sets
`g_flight_debug_received=1`; subsequent 0x1E21s apply a −3° pitch
correction when `g_pitch_correction_enable` is set.

#### `0x1E26` sample_ctrl

`type` ∈ {1 = start, 2 = stop, 10 = mark}.

#### `0x1E27` time_sync

Length = 82. Full payload copied to a global sync state area; used
for tick-base compensation based on local board tick count.

#### `0x1EFC` data_transfer_req

`payload[0]` = flag (0 = START, 1 = END),
`payload[1]` = data_type (0 = side-antenna horizontal chirps,
1 = top-antenna ground chirps). Drives `data_transfer_task` which
sends results as V1 `0x1EFD` frames.

---

## 4. 1E62 / 1E32 — fusion avoidance result

Built by `fusion_avoid_periodic_build_and_send_1E62_1E32` (0x3FBAC).
Triggered every 6th `fusion_periodic_callback` (10 s). `0x1E62` is
always sent to (type=6, id=3) = FC; `0x1E32` is sent every other tick
to (type=0, id=2) = alt receiver.

Payload (116 B, layout at global `0x284548`):

| offset    | field                                              |
| --------- | -------------------------------------------------- |
| +0        | marker byte (fixed `6`)                            |
| +1        | reserved                                           |
| +2..+3    | u16 status flags                                   |
| +4..+5    | u16 `btm_avoid_dis_cm` (default 10000)             |
| +6..+7    | u16 `up_avoid_dis_cm` (default 10000)              |
| +8..+103  | 16 × 6 B sector entries                            |
| +104      | rolling counter                                    |
| +105..+106| u16 `height_kalman_cm` (Kalman-fused AGL height)   |
| +107..+108| u16 `height_filtered_cm`                           |
| +109..+110| u16 `height_confidence`                            |
| +111..+115| reserved                                           |

6-byte sector entry (first byte is a header, subsequent entries' +0
is the previous entry's distance high byte — the distance u16 at +5
straddles into the next entry's header slot):

| byte  | field                         |
| ----- | ----------------------------- |
| +0    | header (0x10 first entry)     |
| +1..+2| u16 azimuth centideg (×100°)  |
| +3..+4| u16 elevation centideg        |
| +5    | distance low byte             |

Default "no obstacle" entry: azimuth=9000 (90°), elevation=0,
distance=10000 cm. Source: `avoid_entries_build_16_sectors` (0x50A54),
which reads a 0x224-byte snapshot via `copy_avoidance_state_from_global`
(0x5BEB4) from `unk_CD44330`.

---

## 5. 1E61 gating and filters

Two gates must both be satisfied before 1E61 carries real data:

1. `is_radar_init_complete() == 1` — driven by
   `radar_motor_init_state_machine` (0x5DC7C) which progresses
   0→1→2→3→4→6 and sets `g_radar_init_complete = 1` at state 4.
2. `g_height_fusion_mode_state & 0xFF` ∈ {2, 4} — written from
   `0x1E21 payload[38]` via `update_height_fusion_from_flight_pos`
   (0x5837C). Gate = `((mode - 2) & 0xFD) == 0`.

Additional filters inside `pcl_avoidance_main_process` (terrain_mode
= 2):
- 3D range > 2.0 m (`sqrt(x²+y²+z²) > 2.0`)
- elevation `z / sqrt(x²+y²)` in `[-1.1918, +0.8391]` (≈ −50°..+40°)
- after filter need > 119 points to enter gridding
- after gridding need > 99 cells to enter height estimation

Grid: azimuth ±120° at 1.5°, range 0..20 m (or 30 m when
`byte_CC82448==1`) at 0.5 m.

`pcla_build_and_send_perception_msg_1E61` adds: distance < 2.0 m
drops, angle/SNR/speed/ground-plane checks.

### 1.6 m height gate

`sub_5494C` (0x54C74), invoked from `fusion_periodic_callback` every
10 s. When `flight_mode == 2` and the fused height `flt_CACBDB0 < 1.6`,
the avoidance + sector state snapshots are zeroed for the frame. A
counter at `word_CACDEB2` increments each invocation and bypasses the
gate once it exceeds 300 (≈ 50 min); it is reset when flight_mode
leaves 2/4.

Height source chain:
`flt_CACBDB0` ← Kalman in `ground_points_height_fusion_periodic_publish`
 ← `sub_583D8` ← ground-plane fit in `sub_565E4` ← filtered ground
points from FPGA.

The height value is driven entirely by radar ground-plane fit and is
independent of `0x1E21 pos_z_m`.

---

## 6. `0x1EFD` data transfer packets

Produced by `data_transfer_task` state machine triggered by `0x1EFC`.
Single-antenna single-beam 1D range magnitude spectrum (256 bins).

State machine:
```
0  wait for request (byte_1779F4 != 0xFF)
1  wait for FPGA sample-ready
2  copy samples + (type 0) windowing / transpose
3  FFT (sub_61BC0)
4  paginate + send as V1 0x1EFD (sub_62818 → sub_62750)
5  done
```

FFT pipeline (`sub_61BC0`):
1. u32 ADC → f32 (NEON vcvtq_f32_u32)
2. DC removal (subtract mean of last 200 samples)
3. windowing (256-pt window `unk_C2500`)
4. 256-pt FFT (`sub_5C600`)
5. magnitude `sqrt(re² + im²)`

Normal packet (781 B = 0x30D):

| offset   | field                                           |
| -------- | ----------------------------------------------- |
| +0..+1   | u16 frame_index                                 |
| +2..+3   | u16 pkg_index (0-based)                         |
| +4       | block_shift = `(768 * pkg_index) >> 11`         |
| +5       | chirp_count                                     |
| +6       | col_count                                       |
| +7       | row_count                                       |
| +8..+9   | u16 data_valid_num (≤ 768)                      |
| +10..+11 | u16 pkg_num                                     |
| +12..+780| payload (≤ 768 B)                               |

Tail packet (21 B = 0x15):

| offset  | field                          |
| ------- | ------------------------------ |
| +0..+1  | u16 frame_index                |
| +2..+3  | u16 pkg_index (last)           |
| +4      | 0xFF marker                    |
| +5      | chirp_count                    |
| +6      | flag byte                      |
| +7..+10 | u32 frame_crc32                |
| +11..+12| u16 pkg_num                    |

Total bytes per frame = `(col_count × row_count + 1) × 2048`.

Data is a 1D range magnitude spectrum per chirp direction. Angle
and elevation are resolved inside the FPGA beamformer before reaching
the MCU; 0x1EFD does not include them.

---

## 7. FC-side consumption

`dji_perception` runs on the **flight controller board**, not the
radar AP. The radar↔FC link is UDP only (no shared memory between
the two). Inside `dji_perception`, `PayloadEntityAgricultureRadar_ctor`
registers these topics:

| topic / class                              | kind              | contents                              |
| ------------------------------------------ | ----------------- | ------------------------------------- |
| `PayloadDataRadarHeightSlopeData`          | publisher         | structured height/slope result        |
| `PayloadDataRadarOmdAvoidData`             | publisher         | omni-direction avoidance result       |
| `PayloadDataRadarUpAvoidData`              | publisher         | up-direction avoidance result         |
| `PayloadDataRadarDataDump`                 | publisher         | raw/blackbox dump                     |
| `PayloadDataRadarDataGround`               | SHM publisher     | ground points (bulk)                  |
| `PayloadDataRadarDataObstacle`             | SHM publisher     | obstacle candidates (bulk)            |
| `PayloadDataRadarDataLabel`                | SHM publisher     | labelled/classified points (bulk)     |
| `PayloadDataRadarDataRaw`                  | SHM publisher     | raw points                            |
| `PayloadActionRadarFlightCtrlPushDroneState`| action           | drone state → radar                   |
| `PayloadActionRadarFlightCtrlPushFlyInfo`  | action            | fly info → radar                      |

The `radar_data_*` topics are an **intra-process bus inside
dji_perception** — not a cross-machine SHM. Inside `dji_perception`:

- `lmap_grid_task_entry` subscribes to the obstacle / ground / label
  topics and drives `update_pointcloud_map` (local grid map).
- The LMAP pipeline produces per-direction nearest-obstacle results
  which are packaged into `radar_omd_avoid_data` (cmd_id `0x1E28`)
  and `radar_up_avoid_data` (cmd_id `0x1E50`) and forwarded to the
  flight-control MCU over DUML.
- Terrain modes: `GROUND / MOUNTAIN / FRUITER` (fruit-tree). Fusion
  rules and obstacle classification vary by terrain mode.
- Status enum: NORMAL / SLIGHT_ABNORMAL / MIDDLE_ABNORMAL /
  SERIOUS_ABNORMAL — feeds fusion weight.

Inferred cross-machine mapping (radar UDP → FC perception topic):

| radar output                      | FC topic (guess)           | confidence |
| --------------------------------- | -------------------------- | ---------- |
| `0x1E61 subtype 0`                | `radar_data_ground`        | high       |
| `0x1E61 subtype 1`                | `radar_data_label`         | medium     |
| `0x1E61 subtype 2`                | `radar_data_obstacle`      | high       |
| `0x00F0`                          | `radar_data_raw` / dump    | medium     |
| `0x1E62 / 0x1E32`                 | consumed directly (not via topic) | high |

FC Linux binaries for reference:
- `dji_perception` — the big one, runs LMAP, publishes avoid results
- `dji_flight`, `dji_navigation`, `dji_autoflight` — thin CLIs
- `radar_agent` — power / HMS only, NOT a UDP↔SHM bridge
- The real flight-control loop is on a separate MCU, not Linux.

---

## 8. Confidence summary

High:
- V1/V2 wire formats, endpoint layout
- All inbound V1 handler lengths and handler-side field offsets
- 1E62/1E32 payload layout
- 1E61 record layout (all three subtypes)
- 1E61 gating conditions
- 1EFD fragment format

Medium:
- Exact semantics of V1 `wire[8]` ack/flag bit fields
- Exact radar-UDP ↔ FC-topic routing inside `dji_perception`
- `0x00F0` fragment header and reassembly protocol

Open:
- Full 69 B 1E61 header field-by-field semantics
- Runtime peer-learning on 50007
