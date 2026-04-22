# DJI T40 Radar — Working Notes

**Read at session start. Update when facts change.**

Sibling docs (prefer these for their respective surface area):
- `re_notes.md` — `app.elf` reversal: memory map, function addresses,
  struct layouts, state machines
- `radar_link_protocols.md` — UDP (50007 V1 / 40204 V2) between radar
  and FC
- `DUML_V1_Protocol.md` — DUML wire format on ESC ↔ app.elf CAN

---

## Ground rules for shell commands on the radar

- **No gdb on target.** Use the on-target `radar_mem_poke` ELF
  (PTRACE_POKEDATA). Python wrapper: `radar_mem_poke.py` (deploys to
  `/tmp/radar_mem_poke`).
- **No Python on target.** Diagnostics run from a dev box over SSH,
  or pure shell on the radar.
- **Use `hexdump -C`**, not `od`. BusyBox `od` is awkward.
- **No Chinese characters in shell commands** — not in args, comments,
  or echo strings. The radar shell does not handle them.
- Can't SSH to the radar from this dev machine; the user runs every
  command. Give copy-pasteable blocks and wait for output.

## Toolchain

- ARM cross-compiler: `/tmp/arm-toolchain/bin/arm-buildroot-linux-musleabihf-gcc`
- Build helpers statically: `-O2 -static`.

---

## Hardware

- **Top antenna:** 4×4 physical array (fixed, upward). Driven by a
  **2R1T chip with only 1T + 1R wired** — other elements are parasitic
  / corporate feed. Effectively monostatic: no angular aperture, no
  MIMO.
  - 32×64 fixed_height heatmap: rows = range (×0.73 m/bin, 0..23 m,
    real); cols = Doppler-derived "y" assuming fixed target height
    and known forward velocity. On bench (v≈0) all energy collapses
    into col 32.
  - Spec: coarse upper-obstacle detection + AGL height, FOV 45°,
    1.5–30 m.
- **Side antenna:** 12 (vertical) × 6 physical array, rotates
  continuously in azimuth at **900 rpm** (15 rev/s). Driven by a
  **4R2T chip with only 1T + 4R wired**: 1T is the 12×2 middle-column
  strip (wide-beam TX), 4R are four 12-element vertical columns.
  - **Azimuth resolution:** rotation encoder (360 sectors/rev); 4-RX
    column spacing adds ~30° in-sector refinement.
  - **Elevation resolution:** 12-vertical beamforming per RX column
    (~10° beamwidth at λ/2). Produces `body_xyz.z`.
  - Sector-index → body-azimuth (viewer):
    `az_rad = -slot_idx × 1° × π/180 + offset_rad` (default offset
    180°). Corrections baked in: encoder zero is +180° from forward;
    motor spins CCW viewed from above (hence the negation).
- **Radar ↔ FC link is NETWORK ONLY.** No UART between radar and FC —
  everything nicknamed "fc uart" is DUML over UDP 50007.
- **Motor control path:** app.elf → DUML UDP → FC → CAN → lower-board
  motor MCU. app.elf does not drive the motor directly. Without the
  FC/CAN path the motor command ACKs never arrive and `unk_282DEC`
  stays 0.
- Upper/lower boards link by WiFi. AR1021X is the WiFi chip, not radar.
- SSH: `root@192.168.8.2` port 107 (ssh-dss), password `root`. UDP
  50007 (V1) / 40204 (V2) are the live data ports.

Reverse-engineered addresses and function details live in
`re_notes.md` — this doc references symbols by name only.

---

## Pipeline status

**Closed-loop healthy with CAN-attached ESC.** Through `--can-bridge`
in `radar_min_client.py` (or a real lower-board ESC), encoder feedback
reaches the FPGA, `record[1]` (N_points) becomes non-zero, `rot_angle`
sweeps, and body_xyz points appear at varied azimuths.

**Bench without CAN:** motor may spin autonomously (lower-board
default power) but the encoder → FPGA path is severed. Symptoms:
`rot_angle` pinned at 0x168 sentinel; `sectors_decoded` advances at
~0.7 Hz (timeout-based); `ctrl+0x68` stuck at 0x3E7; `record[1] == 0`
on every record; `[FMCT] wrong sample key` log spams 1 Hz.

Software ESC sim cannot fix this — the encoder is a physical wire
into the FPGA (read by `sub_31744` from an FPGA register), not a DUML
field. See `re_notes.md` §6.2.

---

## Artifacts on disk

- `radar_mem_poke.c` / `radar_mem_poke` — ptrace writer ELF, deployed
  to `/tmp/radar_mem_poke` on target.
- `radar_mem_poke.py` — Python CLI wrapper (activate / deactivate /
  pm0 / pm3 / poke / status / monitor). Handles deploy.
- `radar_activate_side.py` — one-shot: writes `g_flight_mode_from_fc=2`
  then monitors state. Uses radar_mem_poke.
- `radar_streamer.c` / `radar_streamer` — unified UDP streamer with a
  positional MODE arg:
  - `radar_streamer top  PID [PORT] [INTERVAL_US]` — port 9876, 8908 B
    frames. Streams the fixed_height heatmap + snapshot + peak list
    from app.elf via `/proc/PID/mem` (addresses in `re_notes.md` §5).
  - `radar_streamer side PID [PORT] [INTERVAL_US]` — port 9877, 60632 B
    frames. 32 B header (`OMNI` magic v2 + rot_angle + sectors_decoded
    + frame_token + point_count + cfar_error + export_busy) + 360 ×
    160 B `omni_snapshot` + 250 × 12 B firmware `body_xyz`.
- `radar_viewer.py` — unified TOP + SIDE viewer, flat CLI (no subcommand):
  - `--deploy` — auto-launches both streamer instances on the radar
    (ports 9876 + 9877).
  - `--udp HOST` — attach to already-running streamer pair.
  - `--dump` — print one frame as text and exit.
  - SIDE body_xyz is **computed client-side** from raw sectors;
    firmware body_xyz in memory is capped at 250 and biased, we ignore
    it.
  - Azimuth uses `slot_idx × 1°` with `--azimuth-offset` (default
    180°). Motor rotates CCW so the viewer negates `slot_idx` before
    adding the offset.
  - Persistence filter (CFAR flicker rejection): `--persist-window N`
    (default 16 ≈ 1.3 s), `--persist-min-hits N` (default 6; 0
    disables), `--persist-tol-az/tol-r` (default 10° × 1.0 m). Uses
    per-cell EMA position smoothing for stable output.
  - SNR encoded as dot size + edge brightness; z encoded as fill
    color (coolwarm).
  - Absolute filters: `--min-snr N`, `--min-range N`.
  - Layer toggles: `--show-sectors`, `--hide-body`, `--hide-filter`,
    `--ccw`.
  - Zoom: `+`/`-` cycles 10 / 20 / 30 / 50 m.
- `radar_min_client.py` — FC stand-in + live plot. Plots 1E61-derived
  content (wire + pcla + ground density). Terrain modes:
  `generic / ground / mountain / fruiter` (press `t` to cycle at
  runtime).

---

## FC-bridge modes in `radar_min_client.py`

On a real aircraft, app.elf's motor DUML (cmd_set=0x1E) is routed
UDP → FC → CAN → ESC. Three ways to stand in for the FC on bench:

- **`--can-bridge IFACE`** — transparent SocketCAN bridge. Forwards
  cmd_set=0x1E UDP DUML to CAN in 8 B chunks and relays reassembled
  CAN DUML back over UDP. Default CAN ids: `tx=0x0A2` (ESC rx),
  `rx=0x0A3` (ESC tx), from `decode_link_id(0x520518A2)`. Produces
  real perception data when a CAN-attached ESC is present.
- **`--sim-esc`** — pure software simulation. Emits a byte-precise
  replica of the real ESC push stream (bottom_data_pack,
  imu_inter_pack, keepalive, activation reply, diagnostic) plus
  hms_switch ACK synthesis. Unblocks state progression (motor_cmd=2,
  flight_mode=2, power_mode=2 with `--fm-ramp-sec 3`, log
  `Bandwidth switch to 800M`) but does **not** produce perception
  data — encoder is a physical wire, not a DUML field.
- **`--replay-esc PATH`** — replay a captured `can/duml.jsonl` from
  a prior `--can-bridge --capture-dir` session. Each loop shifts seq
  by +1000 and recomputes CRC16 so app.elf doesn't dedup. Mutually
  exclusive with `--can-bridge` and `--sim-esc`.

**Graceful shutdown:** Ctrl-C and SIGTERM go through
`RadarClient.close()`. If a CAN bridge is up, one final
`cmd_set=0x1E cmd_id=0x80` motor_speed DUML with payload `00 00` is
sent on the tx CAN id so the ESC doesn't hold the last-commanded
speed. 50 ms sleep afterward to let the TX queue flush.

**CAN bridge logging** (`--can-bridge` + `--capture-dir`):
- `can/raw_can.log` — candump-compatible, replayable with
  `canplayer -I raw_can.log`. Direction implicit via can_id.
- `can/duml.jsonl` — one record per reassembled DUML frame with
  `dir`, `can_id`, `sender`, `receiver`, `seq`, `flags`, `cmd_set`,
  `cmd_id`, `cmd_name`, `payload_hex`, `frame_hex`. This file is what
  `--replay-esc` consumes.

---

## 1E61 subtype observations

All three subtypes share layout `<HfffH>` (u16 id, f32 x/y/z, u16
metric) and come from the **side rotating 12×6 array**. Wire layout
is documented in `radar_link_protocols.md` §2.3; builder function
addresses are in `re_notes.md` §6.6.

Observed quantitative behaviors (cap1, 689 frames):

- **subtype 0 ground** (385 frames): all points `z < 0` (below-horizon
  beams); 435 distinct 0.5 m bins hit, only 87 hit in ≥10% of frames;
  37% of bins are 1–2 hit flicker. Correct rendering = time-decayed
  2D density heatmap over (x, y), not individual-dot scatter. `id`
  and `metric` are always 0 (never written by the builder).
- **subtype 1 wire** (41 frames): 20 ids, each in 6–54 frames (median
  33). **Same id jumps to unrelated coords across frames** — tracker
  reuses slots. Don't draw per-id trails; they fabricate fake
  trajectories. Show latest-frame scatter; encode metric via size.
- **subtype 2 pcla** (263 frames): id 1..22 is **azimuth sector**,
  not an object class. Multiple records in a frame share the same id
  (= range peaks inside that beam). 18–22 sectors × 100–160 points
  typical. Color by sector id to visualize angular coverage.

~14% of ground points are multipath/image-frequency outliers with
`z < -10 m` (observed z = −44.6 m). Must be filtered.

**Top 4×4 is NOT on UDP.** Exhaustive scan of `app.elf`:
perception-sending functions are exactly `v1_send_msg_1E62_to_fc`,
`v1_send_msg_1E32_to_alt`, `perception_v2_send_msg_1E61`,
`blackbox_v2_send_msg_F000`. The 32×64 fixed_height heatmap goes to
internal diag log `sub_216C4(12435)` only. Top-antenna network-visible
output is just the `height_kalman_cm / height_filtered_cm /
height_confidence` tail in `0x1E62` / `0x1E32`. `radar_viewer.py`'s
TOP panel reads the heatmap via `/proc/PID/mem` through
`radar_streamer top`.

---

## ESC ↔ app DUML observations

Reverse-engineered from app.elf, confirmed against a 241 s cap1
`--can-bridge` recording. Command dispatch and addresses live in
`re_notes.md` §6.2; `DUML_V1_Protocol.md` covers the ESC firmware's
handler table.

| cmd_id | dir       | len  | rate      | purpose                                                                 |
| ------ | --------- | ---- | --------- | ----------------------------------------------------------------------- |
| 0x09   | app→esc   | 1 B  | on demand | motor_enable                                                            |
| 0x0A   | app→esc   | 1 B  | on demand | motor_disable                                                           |
| 0x0B   | esc→app   | 26 B | 4.3 Hz    | radar_imu_inter_pack (1 B flag + 13 B zero + 3 × f32 IMU/vibration)     |
| 0x0F   | esc→app   | 44 B | 4.3 Hz    | bottom_data_pack. cap1: byte[25]=0x1F, byte[28]=1, bytes[26..27]=0 always (running-mode angle comes from FPGA, not this field) |
| 0x80   | app→esc   | 2 B  | 0.4 Hz    | motor_speed (cap1: constant 900 = rpm_set)                              |
| 0x81   | app→esc   | 6 B  | on demand | motor_source_pos                                                        |
| 0x82   | app→esc   | 0 B  | on demand | motor_stop                                                              |
| 0x83   | app→esc   | 1 B  | on demand | hms_switch (ACK required)                                               |
| 0xEA   | esc→app   | 2 B  | 0.5 Hz    | keepalive `aa aa`                                                       |
| 0xF9   | app→esc   | 1 B  | 0.4 Hz    | state-2 toggle (fire-and-forget, paired with motor_speed)               |
| 0xFE   | app↔esc   | —    | on demand | motor_state_query                                                       |

Non-0x1E frames the real ESC pushes:
- `cmd_set=0x00 cmd_id=0x32` activation reply, 18 B (ASCII SN
  `5F2BK5E10201EZ`), 0.5 Hz, broadcast to receiver ids 0x02 and 0x0A.
- `cmd_set=0x0E cmd_id=0x01` diagnostic, 1 B fixed `07`, 0.5 Hz,
  receiver 0xC3.

**hms_switch ACK** never appeared in cap1 — `unk_282DEC = 2` was
written via the FAC_MANUAL path. `motor_speed + 0xF9` keep state-2
locked without the ACK chain in normal operation.

---

## FC-side avoidance pipeline

`dji_perception` is an **FC-side binary**
(`V01.03.1005_ag601_dji_system-out/…/system/bin/dji_perception`), not
on the radar Zynq. Cross-machine radar → FC is UDP only — no shared
memory between the two boards.

Inside `dji_perception`, an `abstract_devices` pub/sub bus
(`/device/payload/2/data/*`) is **intra-process**, not cross-machine
SHM. Topics: `radar_data_obstacle`, `radar_data_ground`,
`radar_data_label`, `radar_data_raw`. `lmap_grid_task_entry` subscribes
and drives the LMAP pipeline, whose output is packaged into two DUML
frames to the MCU:

- **`0x1E28 radar_omd_avoid_data`** — 28 B header + 3 variable-length
  6 B record arrays. Header `+1` holds 8 direction-valid bits (one per
  `LMAP_DIRECTION_*`: DOWN/FRONT/BACK/RIGHT/LEFT/UP/DOWN-LEFT/DOWN-RIGHT).
- **`0x1E50 radar_up_avoid_data`** — 15 B flat (up-direction only).

Function addresses in `re_notes.md` §12.

**Terrain modes** (`sub_4DCD0`): `GROUND / MOUNTAIN / FRUITER`.
FRUITER (fruit-tree) is what makes the T40 different — in this mode
tree canopies are the spray target, not obstacles. Fusion rules and
obstacle classification differ by mode.

**Status enum:** `NORMAL / SLIGHT_ABNORMAL / MIDDLE_ABNORMAL /
SERIOUS_ABNORMAL` — feeds LMAP fusion weight.

**ESDF unknown policy:** `MAP_UNKNOWN_AS_FREE` vs
`MAP_UNKNOWN_AS_OCCUPY` — runtime-tunable per flight phase.

FC Linux binaries (context):
- `dji_perception` (9.5 MB) — the big one; LMAP + avoidance output.
- `dji_flight` (293 KB) — thin CLI.
- `dji_navigation`, `dji_autoflight` — mission / autoflight interface,
  also thin.
- `radar_agent` (60 KB) — power / HMS only, **not** a UDP ↔ SHM bridge.
- `dji_agent`, `dji_sys`, `dji_ppt` — device / transport daemons.

**The real flight-control loop runs on a separate MCU**, not on the
Linux AP. Linux binaries only prepare data and forward via DUML.

---

## Plot panel physical meaning

**radar_min_client.py (1E61-derived, DJI-post-CFAR)**:
All XY panels use forward-up PPI: screen-y = x_m (forward),
screen-x = y_m (right). Drone ▲ at origin points up.
- `ax_top` — side-antenna scope (wire + pcla), latest-frame only.
- `ax_zenith` — ground_points density heatmap (default 2 s window).
- `ax_side` — height vs horizontal range (wire + pcla + latest
  ground), no axis swap.
- `ax_height` — 1E62 tail `height_kalman_cm` / `height_filtered_cm`
  (Kalman-fused AGL; does NOT reflect "what is above the drone").
- `ax_counts` — 1E62 `btm_avoid_dis_cm` (FC's bottom-avoid distance).

**radar_viewer.py TOP panel (raw magnitude, pre-CFAR)**:
- Rows (32, y): range × 0.73 m/bin = 0..22.63 m.
- Cols (64, x): Doppler-derived "lateral", NOT real-world angle.
  On bench (v≈0) all energy is in col 32.
- Color = complex magnitude (10³–10⁶ range).

**1E61 data rate is inherently low** — subtype 1 wire ~0.17 Hz,
subtype 2 pcla ~1.1 Hz, subtype 0 ground ~1.6 Hz. For real-time
environment response use `radar_viewer.py`'s raw-sector path (50 Hz).

---

## Open items

- Persistent noise clusters sometimes appear a few seconds after
  client start (not frame 1). Could be firmware state ramp-up or
  persistence-filter cumulative-threshold effect.
- See `re_notes.md` §13 for the reversal-side open questions
  (`table_lookup` field decoding, V1 flag bit semantics, FC-side
  UDP → SHM routing, etc.).
