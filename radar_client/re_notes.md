# Reverse Engineering Notes — DJI T40 Radar `app.elf`

Static reversal of `app.elf` on the radar Zynq AP. No-PIE, base `0x8000`.
Cross-checked against captures (`cap1/` recordings) where noted.

Sibling docs:
- `CLAUDE.md` — operational notes (scripts, workflow, plot panels)
- `radar_link_protocols.md` — UDP V1/V2 protocol, per-handler payload layout
- `DUML_V1_Protocol.md` — DUML wire format on ESC↔app CAN

Symbols whose name starts with `sub_` are IDA-default (not renamed);
`unk_` / `dword_` / `byte_` likewise. Named symbols are either IDA
imports or were renamed during analysis.

---

## 1. Binary basics

- ELF class: ARM, little-endian, Zynq AP (user-space, under Linux).
- Base: `0x8000` (No-PIE). Code, data and BSS are offset from that.
- Data addresses quoted in this doc are absolute (already include the
  `0x8000` base). When poking via `/proc/PID/mem`, use these as-is.
- Hex-Rays decompilation is used throughout. Many `sub_XXXX` names
  have been re-identified by behavior — see §4–§9.

---

## 2. FPGA control block @ `0x43C30000`

4 KB MMIO page. Mapped into app.elf under three pointer aliases of
the same physical block:

| alias              | global pointer            | init function |
| ------------------ | ------------------------- | ------------- |
| `2dcfar_map`       | `unk_282CD4`              | `sub_3C53C`   |
| `grat_ctrl_cfg`    | `off_12048C = base+0x15C`, `off_120490 = base+0x1D4` | `sub_32454` |
| `par_ctrl_cfg`     | `dword_1204AC = base+0x1C0` | `sub_32660` |

Register layout (verified by reading back on a live bench):

| Offset | Meaning | Evidence |
|--------|---------|----------|
| +0x00  | version/ID | observed `0x20220325` |
| +0x04  | (read-only) | `sub_3BECC` reads |
| +0x08  | enable bits — bit0=2dcfar enable (init `\|=1`), bit1 pulsed by motor FSM via `sub_3BF04`/`sub_3BEE0` edge | observed `0x3` steady |
| +0x10  | result lane mask | init writes `0xF`, runtime `0xC` (lanes 2,3) |
| +0x14  | (unused — `sub_3C034` pulses bit0 but no code xref) | dead |
| +0x18  | status/ctrl word — `sub_3BFE0` reads bit1, `sub_3C01C` reads bit4, `sub_3BFF8` sets bit1 (UNUSED) | observed `0xFCFCFCFC` in some states |
| +0x1C  | (freestyle R/W, `sub_3BFCC` UNUSED, no xref) | observed `0x14` |
| +0x20  | packed status — `sub_3C06C` reads low16, `sub_3C088` reads bits 16..19, `sub_3C0AC` reads byte at bit 20 | |
| +0x24  | FPGA readback (not SW-written). Likely per-channel RF status (4 bytes) | pm0=0, pm3=`0x8C80808C` |
| +0x28  | FPGA readback (per-channel) | pm3=`0xA4C4C4A4` |
| +0x30  | status byte — `sub_3C1C0` returns `byte - 2` | |
| +0x38  | calibration slot (`dword_121D28 << 16`) | populated by `sub_3C53C` from am_ph cali |
| +0x3C/0x40/0x44 | calibration loop fields (`sub_3C53C` init writes `dword_121D24<<20 + dword_121D28<<16` to each) | |
| +0x58  | CFAR DMA result base = `0x1C100000` | |
| +0x5C  | CFAR DMA stride = `0x100000` | |
| +0x60  | sector count max = `0x3E7` (999) | `sub_3C230` reads |
| +0x64  | live CFAR write sector idx — advances ~0.3 Hz on bench | `sub_3C21C` reads |
| +0x68  | FPGA write pointer `(bank_bit<<10) \| sector_idx` | `sub_3C204` / `sub_3C244` read |
| +0x70..0x7C | CFAR thresholds (4 × u32). Defaults `0x999 / 0x3000 / 0xA000 / 0x21BD`. Rewritten by `set_rd_sens_parameter` | |
| +0xC8  | arm bit (init `\|=1`) | observed `0x1` |
| +0xCC  | sector window q10. **Always 0 at runtime by design** (see `sub_3C4C4` / `sub_3ADB8` analysis below) | |
| +0xD0/0xD4/0xD8/0xDC | phase cal ch1..4 (`ph_cal_ch1_L` …). STATIC, not motor angle | |
| +0x100/0x104/0x108/0x10C | amplitude cal ch1..4 | |
| +0x124 | frame metadata | `sub_3C1DC` reads; sampled by `fpga_mimo_proc_task` |
| +0x12C | low-nibble OR target (UNUSED, `sub_3CCFC`) | |
| +0x130 | grating switch mode byte. `sub_3C25C` reads bit0; `sub_3C288` writes `{2,0,0,0}` on recovery via `sub_3DA9C` | |
| +0x134 | (read, `sub_3C274`) | |
| +0x15C | **grat_ctrl_cfg main reg**. State-2 motor FSM writes `bit0=1` (`sub_32510`), `bit4=0` (`sub_3257C`) | observed `0x0058000B` |
| +0x17C | RF profile param (u16). `205` = 800 M, `1000` = 200 M, written by `set_rf_profile_param_380` on power-mode change | |
| +0x1C0 | par_ctrl_cfg (mapped alias only) | |
| +0x1D0/0x1D8 | status windows. `sub_3BD80` returns high16 of +0x1D0 and full +0x1D8 | |
| +0x1D4 | grat_ctrl_cfg status (`off_120490`) | |

CFAR record magic: `0xB562645A` ("FMCT sample key") at record `+0x7F4`
and `+0x7FC`. `fpga_mimo_proc_task` polls after every FFT IRQ; missing
value logs `"FMCT wrong sample key in magic_number_front, find: 0x0,
demand: 0xb562645a"`.

### 2.1 Why `ctrl+0xCC` is always zero

`sub_3C4C4 set_type2_sector_window_q10(a1, a2)` writes `(a1<<10)` to
low16 and `(a2<<10)` to high16 of `ctrl+0xCC`. Call sites:

- Init: `sub_3C53C` calls it with `(0, 0)`.
- Runtime: `loc_3CF30` registered as type-2 config handler — but it
  reads `a1+8 / a1+0xA` from the **caller's buffer**, not from the
  dispatch entry.

Runtime handler `sub_3ADB8 radar_manager_para_config` callers:
- `cmd_flight_pos_handler` passes `double s[2]` with only `s[0]` set,
  so `s[+8..+B] = 0`.
- `sub_3EBC8` is unreachable (no xrefs).

Therefore `ctrl+0xCC` is always `0` at runtime — by design, not a bug.

---

## 3. Kernel UIO mapping

| node          | subsystem        | SW behavior                                  |
|---------------|------------------|----------------------------------------------|
| `/dev/uio0`   | adc_sample0      |                                              |
| `/dev/uio1`   | fft_2d_proc      | `fpga_mimo_proc_task` opens this, waits IRQ; CFAR decode rides on the FFT IRQ |
| `/dev/uio2`   | 2dcfar_proc      | **app.elf does NOT open this** — it's only referenced in a cleanup `remove()` table at `0x11ef4c`. No separate CFAR IRQ in software |
| `/dev/uio3`   | fixed_height_proc|                                              |

All four share physical `0x43C30000` (single 4 KB page).

---

## 4. DMA / udmabuf

- `udmabuf0`: physical `0x18100000`, size 96 MB (`0x6000000`), sync_mode=5
  (uncached). App mmaps at vaddr **`0xB0400000`**.
- CFAR result ring: physical `0x1C100000` = `udmabuf + 0x4000000` =
  app vaddr **`0xB4400000`**.
- FFT complex output: physical `0x1C900000`. Live IQ, values vary
  smoothly. Pipeline up to FFT confirmed working (independent of
  encoder-feedback state).

---

## 5. Global memory map (`app.elf`, key writeable addresses)

### 5.1 Configuration / control state

| Addr        | Symbol                          | Purpose                                       |
|-------------|---------------------------------|-----------------------------------------------|
| `0x122090`  | `byte_122090`                   | flight_mode copy (written from 0x1E21)        |
| `0x17737C`  | `dword_17737C`                  | Kalman-fused height (updated by ground_points) |
| `0x177578`  | fixed_height CFAR tuning block  | see §10.1                                     |
| `0x17A3F8`  | `byte_17A3F8`                   | power_mode source (pm0..3)                    |
| `0x177578..0x177594` | fixed_height pipeline params | (see §10.1)                                |
| `0x17AC28`  | `byte_17AC28`                   | previous `rd_sens_level`                      |
| `0x181518`  | `unk_181518`                    | monotonic V1 seq counter                      |

### 5.2 Runtime state (BSS-like, written by handlers)

| Addr        | Symbol                          | Purpose                                       |
|-------------|---------------------------------|-----------------------------------------------|
| `0x282CD4`  | `unk_282CD4`                    | `2dcfar_map` ctrl-block pointer               |
| `0x282CD8`  | `unk_282CD8` = `0x1C100000`     | CFAR phys base                                |
| `0x282CDC`  | `unk_282CDC` = `0xB4400000`     | CFAR vaddr                                    |
| `0x282CE0`  | `unk_282CE0` = `0x100000`       | bank stride                                   |
| `0x282D74`  | `unk_282D74`                    | RPM setpoint (observed 900)                   |
| `0x282DA0`  | `unk_282DA0`                    | fix-mode request flag                         |
| `0x282DD0`  | `dword_282DD0`                  | debug_mode (from persisted fac/debug)         |
| `0x282DD8`  | `dword_282DD8`                  | fac_mode (from persisted fac/debug)           |
| `0x282DEC`  | motor_cmd gate                  | 0=uninit, 1=stop, 2=run-gate-open             |
| `0x283208..0x283234` | 0x1E25 flight_debug fields | see `cmd_flight_debug_handler`             |
| `0x283295`  | `g_flight_mode_from_fc`         | write 2 = activate side antenna               |
| `0x2836B8`  | `unk_2836B8`                    | band / session (from 0x1E22 sub_id=0)         |
| `0x284124`  | `unk_284124`                    | flight-mode latch                             |
| `0x28453C`  | `unk_28453C`                    | FC keepalive flag — cleared by watchdog, set by 0x1E21 |
| `0x284540`  | `unk_284540`                    | FC disconnect-seen flag                       |
| `0x2845E0..0x2845E4` | 1E62/1E32 scheduler counters | 6-tick modulo and alternating flags       |
| `0xC140DA9` | `dfa_mode` mirror                |                                               |
| `0xC140DD8` | first omni snapshot sector      |                                               |
| `0xC21DF31` | DFA state (copy of flight_mode) |                                               |
| `0xC21DF50` | first decoded CFAR sector       | header+6 = N_points                           |
| `0xC2CF7F0` | live power_mode                 | mirror of `byte_17A3F8`                       |
| `0xC2CF810..0xC2CF81D` | DFA condition bytes (16 B) | from `0x1E25[28..33]`                     |
| `0xC2CF824` | rot_angle + sectors_decoded     | 3× u16 rot_angle (bytes 0..1, 2..3, 6..7), sectors_decoded (bytes 4..5) |
| `0xC2D5C64` | CFAR error flag                 | 1 = last FMCT check failed                    |
| `0xC3DEE00` | firmware body_xyz (250 × 12 B)  | capped; biased; we ignore this in viewer     |
| `0xC442F90` | exported point count (u16)      |                                               |
| `0xCACBDB0` | `flt_CACBDB0`                   | Kalman-fused height in m (1.6 m gate reads)  |
| `0xCD37CC0` | fixed_height peak list (0xA8 B) | streamed by `radar_streamer top`              |
| `0xCD40330` | fixed_height 32×64 heatmap      | streamed by `radar_streamer top`              |
| `0xCD44330` | fixed_height snapshot (0x224 B) | streamed by `radar_streamer top`              |

---

## 6. Key functions by subsystem

### 6.1 FFT / CFAR / event dispatch

| Addr        | Name                               | Notes                                                 |
|-------------|------------------------------------|-------------------------------------------------------|
| `sub_3D07C` | `fpga_mimo_proc_task`              | Opens `/dev/uio1`, waits IRQ, computes cursor, checks magic, calls `radar_manager_dispatch_event`. See §7 |
| `sub_3AFD0` | `radar_manager_dispatch_event`     | Dispatch table @ `0x282778`, 136 B entries (34 dwords). Dispatches by `(a1[0], a1[4])` |
| `sub_4B10C` | type-2 event handler               | Reads `event[12]` (= `bank_base + bank_bit*stride`) as src and `event->u16[18]` (= sector_idx) as count; calls `decode_2dcfar_sector_batch` |
| `sub_3B25C` | `decode_2dcfar_sector_batch`       | memset scratch, loop `sector_count` calls to `decode_2dcfar_sector`, memcpy 0x7E4 B to dest, src += 2048; stops on first decode error |
| `sub_3B0DC` | `decode_2dcfar_sector`             | Record parser. See §8 |
| `sub_3C204` | `get_2dcfar_result_sector_idx`     | `ctrl[0x68] & 0x3FF` |
| `sub_3C244` | `get_2dcfar_result_bank_bit`       | `(ctrl[0x68] >> 10) & 1` |
| `sub_3C53C` | `init_2dcfar_map_and_defaults`     | Maps ctrl block, sets thresholds, `ctrl+0xC8 \|= 1`, `ctrl+0x08 \|= 1`, `set_type2_sector_window_q10(0, 0)` |
| `sub_3C3AC` | `enable_type2_result_lanes_0_3`    | `ctrl+0x10 \|= 0xF` |
| `sub_3C4C4` | `set_type2_sector_window_q10`      | Writes `(a1<<10)` low16 and `(a2<<10)` high16 of `ctrl+0xCC`. See §2.1 |
| `sub_3ADB8` | `radar_manager_para_config`        | Only real caller passes `s[+8..+B] = 0`, so ctrl+0xCC is always 0 at runtime |

### 6.2 Motor FSM

| Addr        | Name                               | Notes                                                 |
|-------------|------------------------------------|-------------------------------------------------------|
| `sub_3E1F8` | `motor_control_state_machine`      | 5-state. See §9 |
| `sub_3DD88` | `motor_fix_mode_state_machine`     | Also writes `ctrl+0x15C` bit0/bit4 at case transitions; reads motor angle via `get_bottom_data_pack` / `sub_31744` |
| `sub_3E764` | `set_motor_state_from_fc`          | Emits outbound DUML `cmd_set=0x1E cmd_id=0x83 (hms_switch)` with 500 ms ACK timeout |
| `sub_3E85C` | `cmd_flight_ctrl_handler`          | 0x1E22 handler. sub_id 1/3 → `set_motor_state_from_fc` |
| `sub_3EA50` | `hms_switch_ack_handler`           | On `ack_ret=0 && value!=0`: `configure_motor_runtime_target_and_clear_init(2, value, 3, 300)` → writes `unk_282DEC = 2` |
| `sub_3DB44` | `configure_motor_runtime_target_and_clear_init` | **Only** inbound-control writer of `unk_282DEC = 2`. Other writers: `sub_3D840` FAC_MANUAL, `sub_5DC7C` `radar_motor_init_state_machine` case 0 |
| `sub_31744` | running-mode motor angle read      | Reads FPGA register `off_120420+0x10` — the encoder line. Cannot be spoofed by DUML alone |
| `sub_33320` | `send_internal_motor_source_cmd`   | Emits `0x1E/0x81` motor_source_pos |
| `sub_33364` | `send_internal_motor_setpoint_cmd` | Emits `0x1E/0x80` motor_speed (fire-and-forget) |

### 6.3 Power mode / RF profile

| Addr        | Name                               | Notes                                                 |
|-------------|------------------------------------|-------------------------------------------------------|
| `sub_65F10` | `power_mode_state_machine`         | Selects `byte_17A3F8`: case1 (pm=1) AFE eq=1 / 800M; case2 (pm=2) AFE eq=2 / 800M low-PGA; case3 (pm=3) AFE eq=0 / 200M. Target depends on area_code |
| `sub_3D840` | FAC_MANUAL entry                   | Factory mode; writes `unk_282DEC=2` when `fac_mode<=6` |
| `set_rf_profile_param_380` | writes ctrl+0x17C: 205=800M, 1000=200M | |
| `set_rd_sens_parameter`    | writes ctrl+0x70..0x7C CFAR thresholds | |

### 6.4 Flight command handlers (50007 V1 ingress)

| Msg ID | Handler addr | Symbol                              | Payload len | Notes |
|--------|--------------|-------------------------------------|-------------|-------|
| 0x1E21 | `sub_3F044`  | `cmd_flight_pos_handler`            | 66 B        | Writes `g_flight_mode_from_fc=payload[38]`; sets `unk_28453C=1` (FC keepalive). Also updates height-fusion snapshot |
| 0x1E22 | `sub_3E85C`  | `cmd_flight_ctrl_handler`           | 7 B         | sub_id ∈ {1,3} → motor path |
| 0x1E25 | `sub_3E534`  | `cmd_flight_debug_handler`          | 38 B        | See §6.4.1 |
| 0x1E26 | —            | `cmd_sample_ctrl_handler`           | 18 B        | type 1=start, 2=stop, 10=mark |
| 0x1E27 | —            | `cmd_radar_time_sync_handler`       | 82 B        | Copies full payload to global sync state |
| 0x1E0B | —            | `cmd_radar_imu_inter_pack_handler`  | 26 B        |       |
| 0x1EFC | —            | `cmd_handler_req_or_terminate_data` | dynamic     | Drives `data_transfer_task` → 0x1EFD |
| 0x1E51 | —            | `cmd_get_bottom_radar_result`       | 88 B        |       |
| 0x1E52 | —            | `cmd_get_rear_radar_result`         | 62 B        |       |

#### 6.4.1 `cmd_flight_debug_handler` (0x1E25) field map

```
[0-3]   dword → dword_283208
[4-7]   dword → dword_28320C
[8-11]  dword → dword_283210
[12-15] dword → dword_283214
[16-19] dword → dword_283218
[20-23] dword → dword_28321C
[24-27] dword → dword_283220
[28]    byte  → byte_283224      ← DFA profile selector, exported to C2CF810
[29]    byte  → byte_283225      ← exported to C2CF811
[30]    byte  → byte_283226      ← exported to C2CF812
[31]    bitfield:
        b0 = byte_283227 / byte_283236   (alt)
        b1 = g_pitch_correction_enable
        b2 = byte_283229
        b3 = byte_28322A
        b4 = byte_28322B
        b5-7 = byte_28322C
[32]    bitfield:
        b0-2 = byte_28322D
        b3   = byte_28322E
        b4   = byte_28322F
        b5-6 = byte_283230
        b7   = partial byte_283231
[33]    b0 = partial byte_283231
[34]    gap
[35-36] u16   → word_283232
[37]    byte  → byte_283234
```

Side effect: sets `g_flight_debug_received = 1`. When
`g_flight_debug_received && g_pitch_correction_enable`,
`cmd_flight_pos_handler` applies a −3° pitch correction.

`sub_3FAB8` copies the `0x283224..0x283231` group each tick to
`unk_C2CF7F4`, which is what `sub_4B10C` reads for DFA profile
selection:
- `0xC2CF810 == 14 && 0xC2CF811 == 13` → fixed profile, `dfa_mode = 4`
- `0xC2CF818 == 2` → special profile, `dfa_mode = 3`
- otherwise → generic, `dfa_mode = 2`

Bit mapping `0x1E25[28..33] → 0xC2CF810..0xC2CF81D`:
- `0xC2CF810 ← 0x1E25[28]`
- `0xC2CF811 ← 0x1E25[29]`
- `0xC2CF812 ← 0x1E25[30]`
- `0xC2CF818 ← 0x1E25[31] >> 5`
- `0xC2CF81C ← (0x1E25[32] >> 5) & 0x3`
- `0xC2CF81D ← (2 * (0x1E25[33] & 1)) | (0x1E25[32] >> 7)`

Experimentally verified (on-bench): switching profiles via these bytes
does change `dfa_mode` but does **not** unblock empty raw 2D-CFAR on a
CAN-less bench — root cause is upstream (encoder feedback).

#### 6.4.2 `cmd_flight_pos_handler` (0x1E21) field map

```
[0..15]  4× f32 quaternion
[16..27] 6× int16 position+velocity (×0.001 m / m/s)
[28..33] 3× int16 aux (×0.01)
[38]     flight_mode → g_flight_mode_from_fc (byte_283295) and byte_122090
[39]     bitfield: b0 → unk_283294, b3 → unk_283297
[40..47] double latitude
[48..55] double longitude
[56]     GPS valid byte → unk_283296
[57..64] double timestamp
[65]     pose tag → unk_2832A0
```

At end: `unk_28453C = 1` (keepalive tick).

### 6.5 Fusion / avoidance periodic chain

| Addr        | Name                               | Notes                                                 |
|-------------|------------------------------------|-------------------------------------------------------|
| `sub_3FBAC` | `fusion_avoid_periodic_build_and_send_1E62_1E32` | Every 6th periodic tick. Payload @ `0x284548`, size 116 B. See §7 of `radar_link_protocols.md` for layout |
| `sub_50A54` | `avoid_entries_build_16_sectors`   | Fills 16 × 6 B sector entries. Default = (0x28, 9000 cdeg, 0, 10000 cm) |
| `sub_5BEB4` | `copy_avoidance_state_from_global` | memcpy 0x224 B from `unk_CD44330` |
| `sub_5494C` | 1.6 m height gate                  | Invoked from `fusion_periodic_callback` (10 s). See §10.2 |
| `sub_586A8` | `ground_points_height_fusion_periodic_publish` | Maintains Kalman filter `dword_17737C`; emits `count=0` 1E61 keepalives when both gates pass |
| `sub_583D8` | measurement input to height Kalman | Called from `pcl_avoidance_main_process` |
| `sub_565E4` | ground-plane fit                   | Produces `flt_CC627F8`; `fabsf` of this = measurement |
| `sub_58304` | loads `flt_CACBDB0` from `dword_17737C` | |

### 6.6 Perception senders (outbound)

| Msg     | Addr         | Name                                                | Destination                       |
|---------|--------------|-----------------------------------------------------|-----------------------------------|
| 0x1E61  | `sub_3086C`  | `perception_v2_send_msg_1E61`                       | V2 / 40204                        |
| 0x1E61.0| `sub_58428`  | `ground_points_build_and_send_perception_msg_1E61`  | (ground builder)                  |
| 0x1E61.1| `sub_5141C`  | `wire_candidates_build_and_send_perception_msg_1E61`| (wire builder)                    |
| 0x1E61.2| `sub_4E370`  | `pcla_build_and_send_perception_msg_1E61`           | (pcla builder → `sub_4DE18` entries) |
| 0x1E62  | `sub_30184`  | `v1_send_msg_1E62_to_fc`                            | receiver (type=6, id=3) = FC      |
| 0x1E32  | `sub_30284`  | `v1_send_msg_1E32_to_alt`                           | receiver (type=0, id=2) = alt     |
| 0xF000  | —            | `blackbox_v2_send_msg_F000`                         | V2 / 40204, via `task_pcl_blackbox_publish` |
| 0x1EFD  | `sub_62818`/`sub_62750` | data-transfer packets                    | V1, fragmented                    |

Handler dispatch table for **ESC DUML** (CAN side): `0xC2394`,
registered via `off_17789C`. Entry format:
`[cmd_id:1][cmd_set:1][0x00 0x01:2][mask:4][handler:4]`.

### 6.7 Watchdog / keepalive

| Addr        | Name                               | Notes                                                 |
|-------------|------------------------------------|-------------------------------------------------------|
| `sub_3F838` | `monitor_fc_uart_keepalive_and_disconnect` | 1700 ms budget; checks+clears `unk_28453C`; else sets `unk_284540=1` and logs `fc uart disconnect!` |
| `sub_4B10C` | `"FPGA output 2D-CFAR target Error!"` check | **Symptom**, not cause. Compares each sector's `record+0x7F8` against the previous; empty records give constant `0x007FFFFB` → adjacent pairs equal → always fires |

---

## 7. `fpga_mimo_proc_task` event layout

Stack event passed to `radar_manager_dispatch_event`:

```
event+0x00 = 2                                          // dispatch type
event+0x04 = byte from (a1+20)() or 0
event+0x10 = __int64 ticks/timestamp from (a1+12)()
event+0x18 = ctrl+0x48 snapshot
event+0x1C = WORD1(v37) | (HIDWORD(v37) << 16)          // packed field
event+0x20 = (a1+28)() base
event+0x24 = sector_idx = get_2dcfar_result_sector_idx()
event+0x28 = *(a1+52)
event+0x2C = bank_write_base + (sector_idx << 11)       // cursor
event+0x30 = bank_base + bank_bit      * 0x100000       // just-finished bank
event+0x34 = bank_base + (1 - bank_bit) * 0x100000      // next bank
event+0x38 = sector_idx << 11                           // cursor - bank_write_base
event+0x3C = (int)v37
```

Magic check at `sub_3D07C @ 0x3D26C`:

```
*(cursor - 12)     == 0xB562645A  // previous record +0x7F4
*(cursor -  4)     == 0xB562645A  // previous record +0x7FC
*(cursor - 0x800)                  // previous record [0..3], stored as event+0x66
```

Record size = `0x800` B. Magic at `+0x7F4` and `+0x7FC` of each record.

---

## 8. CFAR record format (0x800 B)

Parsed by `sub_3B0DC decode_2dcfar_sector`.

| Record offset | Field                                                               |
|---------------|---------------------------------------------------------------------|
| `+0..+3`      | qword0 low32. `WORD1 & 0x3FF` = sector idx; `BYTE1 & 0x7F` = N_points (≤125); `BYTE1 >> 7` = ready flag; bits 28–29 = aux |
| `+4..+7`      | frame counter (as WORD2 of qword0) |
| `+0..+16*N`   | `N` point slots, 16 B each (see below) |
| `+0x7E8`      | → output trailer +0x7D8 |
| `+0x7EC`      | → output trailer +0x7DC |
| `+0x7F4`      | magic `0xB562645A` |
| `+0x7F8`      | → output trailer +0x7E0 (constant `0x007FFFFB` when empty) |
| `+0x7FC`      | magic `0xB562645A` |

### 8.1 Output struct (`sub_3B0DC` writes 2020 B scratch, memcpy 0x7E4 B to dest)

```
out[0]   = 1
out[1..2] = WORD1(rec) & 0x3FF    // sector idx (scan-line)
out[3..4] = WORD2(rec)            // frame counter
out[5]    = BYTE1(rec) >> 7       // ready bit
out[6]    = BYTE1(rec) & 0x7F     // N_points (≤125)
out[7]    = bits 28..29 of rec[0..3]
out[8..]  = per-point records (layout TBD, see §8.2)
out+0x7D8 = record+0x7E8
out+0x7DC = record+0x7EC
out+0x7E0 = record+0x7F8
```

### 8.2 Per-point slot (16 B starting at `record + 16*i`)

- `+0`:   reserved / counter
- `+4`:   u32 `table_lookup` — passed into `sub_422C8` as the
  elevation-angle argument after XOR `0x80000000`. Client code treats
  it as sign-flipped float radians; some records decode to large /
  NaN values (encoding may differ by detection class).
- `+8`:   u64 (strength or composite range × speed)
- `+16`:  low32 of u64 field
- `+20`:  u32

`body_xyz.z` = `sin(el) × range`; only meaningful when the `+4`
elevation decode yields a value in `[-π/2, +π/2]`.

---

## 9. Motor state machine (`sub_3E1F8`)

Five states. Edge-triggers acquisition via `ctrl+0x08 bit1`.

```
state 0  MOTOR_START_RUN
           waits sub_B922C() == 50 tick
           calls sub_317E8  (ADC debug arm)
                 sub_3BF04  (ctrl+8 bit1 = 0)
                 sub_3BEE0  (ctrl+8 bit1 = 1)   ← edge
           branch on unk_282DA0: set → state 4 (FIX), else state 1

state 1  RAMP
           ramps rpm to unk_282D74 (nominal 900) over 50-tick steps

state 2  MOTOR_RUNING_NORMAL
           periodically:
             send_internal_motor_setpoint_cmd(rpm)
             sub_3257C(0)   → ctrl+0x15C  &= ~0x10
             sub_32510(1)   → ctrl+0x15C  |= 1
             sub_30208(249, …)  → emits 0x1E/0xF9 state-toggle
           entry gate: unk_282DEC == 2

state 3  MOTOR_UNDEFINE
           terminal; reached if unk_282DEC != 2

state 4  FIX_MODE
           runs motor_fix_mode_state_machine (sub_3DD88)
```

On a bench without CAN, `unk_282DEC` may stay 0 → state 3 reached,
`ctrl+0x15C` is never toggled, no grating switching occurs.

---

## 10. Tunable-parameter blocks

### 10.1 fixed_height 32×64 heatmap pipeline — `0x177578`

| addr      | type | default | purpose                                       |
| --------- | ---- | ------- | --------------------------------------------- |
| `0x177578`| u16  | 1       | range_bin_start                               |
| `0x17757A`| u16  | 12      | lateral_bin_start                             |
| `0x17757C`| u16  | 31      | range_bin_end                                 |
| `0x17757E`| u16  | 52      | lateral_bin_end                               |
| `0x177580`| u16  | 64      | CFAR neighborhood size                        |
| `0x177582`| u16  | 64      | DOA window                                    |
| `0x177584`| f32  | 7.0     | **global threshold multiplier** (↓ = more peaks) |
| `0x177588`| f32  | 0.73    | range m/bin (physical, DO NOT change)         |
| `0x17758C`| f32  | 0.2296  | lateral m/bin (physical, DO NOT change)       |
| `0x177590`| f32  | 3.0     | **local mean multiplier** (↓ = more peaks)    |
| `0x177594`| f32  | 0.1667  | order-statistic index (1/6)                   |

Active detection box: range bins [1..31] × lateral bins [12..52] =
0.73..22.63 m × ±4.59 m. Heatmap buffer covers 0..22.63 m × ±7.35 m.

### 10.2 1.6 m height gate (`sub_5494C @ 0x54C74`)

```
if (byte_1724C6 == 2) {                  // flight_mode == 2
    if (++word_CACDEB2 > 300)             // >300 → bypass
        word_CACDEB2 = 301;               // clamp
    else if (flt_CACBDB0 < 1.6)           // height < 1.6 m
        memset(avoidance_data, 0, ...);
        memset(sector_data, 0, ...);
        goto skip_processing;
}
```

Counter at `word_CACDEB2` increments each 10-s callback; bypass after
300 ticks (≈ 50 min continuous flight_mode=2). Counter resets when
flight_mode leaves {2, 4}.

Possible static patches (bench only):

| target          | addr        | original              | new           | effect                   |
| --------------- | ----------- | --------------------- | ------------- | ------------------------ |
| gate constant   | `0x54CDC`   | `CD CC CC 3F` (1.6 f) | `00 00 00 00` | gate disabled            |
| branch to skip  | `0x54C74`   | `5D FF FF 5A` (BPL)   | `5D FF FF EA` (B) | unconditional skip      |
| 2 m dist filter | `0x57BC0`   | `00 10 A0 D3` (MOVLE R1,#0) | `00 F0 20 E3` (NOP) | remove 2 m pre-filter |

Note: patching `0x54C74` to a NOP falls into the `memset` clear path —
use `B loc_549F0` (`0xEAFFFF5D`) or zero the constant at `0x54CDC`.

### 10.3 FPGA CFAR thresholds (side antenna)

`0x43C30000 + 0x70..0x7C`, written by `set_rd_sens_parameter`. Defaults
(T40, `RF board version == 2`, `rd_sens_level` ∈ {1, 2}):

| addr          | default | note |
|---------------|---------|------|
| `0x43C30070`  | `0x999` (2457)    | |
| `0x43C30074`  | `0x3000` (12288)  | |
| `0x43C30078`  | `0xA000` (40960)  | |
| `0x43C3007C`  | `0x21BD` (8637)   | |

Runtime sensitivity via `devmem`:
```
ssh -p 107 root@192.168.8.2 'devmem 0x43C30070 32 <val>'
```

---

## 11. 1E61 downstream filters (perception pipeline)

`pcl_avoidance_main_process @ 0x57958`, terrain_mode = 2 branch:

- **3D-distance filter**: drop points with `sqrt(x²+y²+z²) ≤ 2.0 m`.
  Code site: `0x57B8C` VMOV.F32 S14, #2.0; `0x57BC0` MOVLE R1,#0.
- **Elevation window**: `z / sqrt(x²+y²) ∈ [-1.1918, +0.8391]`
  (≈ −50°..+40°).
- **Minimum point count** into gridding: `v13 > 0x77` (>119).
- **Minimum grid cells** into height estimation: `v99 > 0x63` (>99).

Grid: azimuth ±120° at 1.5°, range 0..20 m (or 30 m when
`byte_CC82448 == 1`) at 0.5 m.

`pcla_build_and_send_perception_msg_1E61`: adds its own checks —
distance < 2.0 m dropped, angle / SNR / speed / ground-plane checks.

---

## 12. FC-side (`dji_perception`) function map

From `V01.03.1005_ag601_dji_system-out/…/system/bin/dji_perception`.
Separate binary on the flight-controller Linux AP.

| Addr        | Name                                        | Notes                                      |
|-------------|---------------------------------------------|--------------------------------------------|
| `0x4DCD0`   | `sub_4DCD0` (config init)                   | Registers `radar_data_{obstacle,ground,label,raw}` SHM topics; inits terrain mode enum (GROUND / MOUNTAIN / FRUITER) |
| `0x20077C`  | `radar_data_from_radarbody_to_navi`         | Quaternion + translation transform. Record layout: 32 B, xyz at `+4..+15` |
| `0x1FFC84`  | `radar_data_from_bodylevel_to_navi`         | Used for obstacle / ground topics         |
| `0x200AF0`  | `update_pointcloud_map`                     | Main LMAP entry                            |
| `0x210B6C`  | `sub_210B6C`                                | Build point cloud + pillar cell point table |
| `0x20EE10`  | `sub_20EE10`                                | Cell classify (obstacle / ground / free)  |
| `0x212380`  | `sub_212380`                                | Cell allocate                              |
| `0x2129E8`  | `sub_2129E8`                                | Pillar fusion (merge across topics)        |
| `0x214CC8`  | `sub_214CC8`                                | Pillar cell blanking (label only)         |
| `0x215084`  | `sub_215084`                                | Collect observe cell info                  |
| `0xE1070`   | `PayloadDataRadarOmdAvoidData_ctor`         | 0x1E28 output ctor                         |
| `0xE1238`   | `sub_E1238` @ vtable `0x8FC9A0+0x30`        | 0x1E28 serializer (28 B header + 3 × var-length 6 B arrays) |
| `0xE1A30`   | `PayloadDataRadarUpAvoidData_ctor`          | 0x1E50 output ctor                         |
| `0xE1BF8`   | `sub_E1BF8` @ vtable `0x8FCA10+0x30`        | 0x1E50 serializer (15 B flat)              |

Intra-process bus (abstract_devices): `/device/payload/2/data/*`.
Four radar topics: `radar_data_obstacle`, `radar_data_ground`,
`radar_data_label`, `radar_data_raw`.

Cross-machine radar→FC is UDP only (no SHM between machines). The
SHM topics are intra-FC only.

---

## 13. Unresolved / open items

- `table_lookup` field (per-detection record `+4`) — some records
  decode to large / NaN; encoding likely varies by detection class.
- Full 69 B header of `0x1E61` — fields beyond subtype not fully
  named.
- V1 `wire[8]` ack/flag bit semantics (partially mapped via
  `internal[10]`, `[11]`, `[18]`).
- How `radar_data_raw` is fed on the FC side (may not be on UDP).
- Exact UDP cmd_id → FC topic mapping inside `dji_perception`'s
  UDP receiver.
- `0xF000` blackbox fragment header and reassembly protocol.
