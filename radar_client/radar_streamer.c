/*
 * radar_streamer - unified /proc/PID/mem -> UDP streamer for DJI T40 radar
 *
 * Replaces radar_mem_streamer + radar_omni_streamer. Single binary with
 * a positional MODE arg selecting which memory regions to read and which
 * frame layout to emit.
 *
 * Usage:
 *   ./radar_streamer top  PID [UDP_PORT] [INTERVAL_US]
 *   ./radar_streamer side PID [UDP_PORT] [INTERVAL_US]
 *
 *   PID         - app.elf process ID
 *   UDP_PORT    - listen port (default: 9876 for top, 9877 for side)
 *   INTERVAL_US - sleep between frames in microseconds (default: 20000 = 50Hz)
 *
 * Protocol:
 *   Client sends any packet to UDP_PORT to "register".
 *   Streamer sends fixed-size frames back to the registered client.
 *   Lost frames are simply skipped - client always gets latest data.
 *
 * --- TOP mode (heatmap + snapshot + peaks, 8908 bytes) ---
 *   For the top 4x4 antenna fixed_height pipeline.
 *   [0..8191]      heatmap   32x64 float32 @ 0xCD40330
 *   [8192..8739]   snapshot  0x224 bytes   @ 0xCD44330
 *   [8740..8907]   peaks     0xA8  bytes   @ 0xCD37CC0
 *
 * --- SIDE mode (omni snapshot + body_xyz, 60632 bytes) ---
 *   For the side 12x6 rotating array 360 deg PPI.
 *   Header (32 B):
 *     [0..3]   magic 'O','M','N','I'
 *     [4]      version (=2)
 *     [5]      body_xyz count (u8, capped at 250)
 *     [6]      export_busy flag (u8 from 0xC3B2ED4)
 *     [7]      reserved
 *     [8..23]  rotation state (16 B from 0xC2CF824):
 *                bytes 0-1: rot_angle A (u16)
 *                bytes 2-3: rot_angle B (u16)
 *                bytes 4-5: sectors_decoded (u16)
 *                bytes 6-7: rot_angle C (u16)
 *                bytes 8-15: other state
 *     [24..27] count + frame_token (4 B from 0xC442F90)
 *                bytes 24-25: u16 point count (= number of valid body_xyz)
 *                bytes 26-27: u16 frame_token (advances each new frame)
 *     [28]     cfar_error (u8 from 0xC2D5C64)
 *     [29..31] padding
 *
 *   Body part A: per-sector polar (57600 B = 360 sectors x 160 B):
 *     For each sector i in [0..359], 160 bytes stride-read from
 *     0xC140DD8 + i * 2515:
 *       [0]       active flag (u8)
 *       [1]       point count (u8, max 125)
 *       [2]       mode (u8)
 *       [3..4]    angle (u16)
 *       [5..6]    field (u16, low 10 bits = sector angle, 1.3433 deg/unit)
 *       [7..10]   timestamp (u32)
 *       [11..14]  timestamp2 (u32)
 *       [15..154] up to 7 detection slots x 20 B each:
 *                   +0  range (float, m)
 *                   +4  table lookup value (u32)
 *                   +8  raw CFAR field A (u32)
 *                   +12 raw CFAR field B (u32)
 *                   +16 SNR (u16)
 *                   +18 velocity index (u16)
 *       [155..159] padding
 *     NOTE: the per-sector 'sector index' here is the rotating array
 *     encoder index; physical drone-forward direction is offset by
 *     +180 deg (sector 180 = drone-forward) - see viewer for the
 *     correction. This is why body_xyz below is the preferred source
 *     for spatial info: it has already been body-frame-transformed.
 *
 *   Body part B: body_xyz (3000 B = 250 points x 12 B):
 *     Stride-read from 0xC3DEE00 (the body-frame XYZ output of
 *     transform_omni_snapshot_to_point_set_exports). Each entry:
 *       +0 float x (forward)
 *       +4 float y (right)
 *       +8 float z (up)
 *     Only the first 'body_xyz_count' (header byte 5) entries are valid.
 *     point_count_total in header may be > 250, in which case slots
 *     beyond 250 are dropped (typical scans: 0-200 points, 250 is a
 *     comfortable headroom).
 *
 *     CAVEAT: this read is NOT guarded by export_busy_lock. Occasional
 *     torn data is possible; viewer filters NaN/inf and out-of-range
 *     points to compensate.
 *
 * Build (cross-compile for Zynq ARM):
 *   arm-buildroot-linux-musleabihf-gcc -O2 -static \
 *       -o radar_streamer radar_streamer.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* ─── TOP mode constants ─────────────────────────────────────────────── */
#define TOP_HEATMAP_ADDR  0x0CD40330UL
#define TOP_HEATMAP_SIZE  8192
#define TOP_SNAPSHOT_ADDR 0x0CD44330UL
#define TOP_SNAPSHOT_SIZE 0x224
#define TOP_PEAKS_ADDR    0x0CD37CC0UL
#define TOP_PEAKS_SIZE    0x0A8
#define TOP_FRAME_SIZE    (TOP_HEATMAP_SIZE + TOP_SNAPSHOT_SIZE + TOP_PEAKS_SIZE) /* 8908 */

/* ─── SIDE mode constants ────────────────────────────────────────────── */
#define SIDE_OMNI_BASE        0x0C140DD8UL
#define SIDE_OMNI_SECTOR_FULL 2515
#define SIDE_OMNI_SECTOR_DUMP 160
#define SIDE_N_SECTORS        360
#define SIDE_OMNI_BODY        (SIDE_N_SECTORS * SIDE_OMNI_SECTOR_DUMP) /* 57600 */

#define SIDE_BODY_XYZ_ADDR    0x0C3DEE00UL
#define SIDE_BODY_XYZ_STRIDE  12       /* 3 floats: x, y, z */
#define SIDE_BODY_XYZ_MAX     250      /* capped — fits in UDP datagram */
#define SIDE_BODY_XYZ_SIZE    (SIDE_BODY_XYZ_MAX * SIDE_BODY_XYZ_STRIDE) /* 3000 */

#define SIDE_ROTATION_ADDR    0x0C2CF824UL
#define SIDE_ROTATION_SIZE    16
#define SIDE_POINTCOUNT_ADDR  0x0C442F90UL
#define SIDE_POINTCOUNT_SIZE  4   /* u16 count + u16 frame_token */
#define SIDE_CFAR_ERR_ADDR    0x0C2D5C64UL
#define SIDE_EXPORT_BUSY_ADDR 0x0C3B2ED4UL  /* byte = 1 while producer writing */

#define SIDE_HEADER_SIZE  32
#define SIDE_FRAME_SIZE   (SIDE_HEADER_SIZE + SIDE_OMNI_BODY + SIDE_BODY_XYZ_SIZE) /* 60632 */

/* Largest of the two — buffer is statically allocated */
#define MAX_FRAME_SIZE (SIDE_FRAME_SIZE > TOP_FRAME_SIZE ? SIDE_FRAME_SIZE : TOP_FRAME_SIZE)

/* ─── Helpers ────────────────────────────────────────────────────────── */
static int read_at(int memfd, unsigned long addr, void *buf, size_t sz)
{
    if (lseek(memfd, (off_t)addr, SEEK_SET) == (off_t)-1)
        return -1;
    ssize_t n = read(memfd, buf, sz);
    return (n == (ssize_t)sz) ? 0 : -1;
}

/* ─── Mode: top ──────────────────────────────────────────────────────── */
static int build_top_frame(int memfd, unsigned char *buf)
{
    if (read_at(memfd, TOP_HEATMAP_ADDR,  buf,                                                    TOP_HEATMAP_SIZE)  < 0) return -1;
    if (read_at(memfd, TOP_SNAPSHOT_ADDR, buf + TOP_HEATMAP_SIZE,                                 TOP_SNAPSHOT_SIZE) < 0) return -1;
    if (read_at(memfd, TOP_PEAKS_ADDR,    buf + TOP_HEATMAP_SIZE + TOP_SNAPSHOT_SIZE,             TOP_PEAKS_SIZE)    < 0) return -1;
    return 0;
}

/* ─── Mode: side ─────────────────────────────────────────────────────── */
static int build_side_frame(int memfd, unsigned char *buf)
{
    /* Header */
    memset(buf, 0, SIDE_HEADER_SIZE);
    buf[0] = 'O'; buf[1] = 'M'; buf[2] = 'N'; buf[3] = 'I';
    buf[4] = 2;  /* version */

    if (read_at(memfd, SIDE_ROTATION_ADDR,   buf + 8,  SIDE_ROTATION_SIZE)   < 0) return -1;
    if (read_at(memfd, SIDE_POINTCOUNT_ADDR, buf + 24, SIDE_POINTCOUNT_SIZE) < 0) return -1;
    if (read_at(memfd, SIDE_CFAR_ERR_ADDR,   buf + 28, 1)                    < 0) return -1;

    /* export_busy snapshot — informational only */
    unsigned char busy = 0;
    read_at(memfd, SIDE_EXPORT_BUSY_ADDR, &busy, 1);  /* tolerate failure */
    buf[6] = busy;

    /* Per-sector polar (offset SIDE_HEADER_SIZE) */
    unsigned char *sec_body = buf + SIDE_HEADER_SIZE;
    int i;
    for (i = 0; i < SIDE_N_SECTORS; i++) {
        unsigned long addr = SIDE_OMNI_BASE + (unsigned long)i * SIDE_OMNI_SECTOR_FULL;
        if (read_at(memfd, addr, sec_body + i * SIDE_OMNI_SECTOR_DUMP,
                    SIDE_OMNI_SECTOR_DUMP) < 0)
            return -1;
    }

    /* body_xyz: read up to MAX entries. Use the count we just stored
     * in header bytes 24-25 to know how many are valid. */
    unsigned int point_count = (unsigned int)buf[24] | ((unsigned int)buf[25] << 8);
    unsigned int n_export = point_count > SIDE_BODY_XYZ_MAX
                          ? SIDE_BODY_XYZ_MAX : point_count;
    buf[5] = (unsigned char)n_export;

    unsigned char *body_xyz = buf + SIDE_HEADER_SIZE + SIDE_OMNI_BODY;
    /* Always read SIDE_BODY_XYZ_MAX worth; trailing slots will be junk
     * but viewer ignores them based on header byte 5 */
    if (read_at(memfd, SIDE_BODY_XYZ_ADDR, body_xyz, SIDE_BODY_XYZ_SIZE) < 0) {
        /* Tolerate read failure here — zero out */
        memset(body_xyz, 0, SIDE_BODY_XYZ_SIZE);
        buf[5] = 0;
    }

    return 0;
}

/* ─── Mode table ─────────────────────────────────────────────────────── */
struct mode_def {
    const char *name;
    int default_port;
    int frame_size;
    int (*build)(int memfd, unsigned char *buf);
};

static const struct mode_def MODES[] = {
    { "top",  9876, TOP_FRAME_SIZE,  build_top_frame  },
    { "side", 9877, SIDE_FRAME_SIZE, build_side_frame },
};
#define N_MODES (sizeof(MODES) / sizeof(MODES[0]))

/* ─── Main loop ──────────────────────────────────────────────────────── */
int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr,
            "Usage: %s MODE PID [UDP_PORT] [INTERVAL_US]\n"
            "  MODE         'top' (heatmap+peaks, %d B) or 'side' (omni+body_xyz, %d B)\n"
            "  PID          app.elf process ID\n"
            "  UDP_PORT     listen port (default: top=9876 side=9877)\n"
            "  INTERVAL_US  frame interval, default 20000 (50 Hz)\n",
            argv[0], TOP_FRAME_SIZE, SIDE_FRAME_SIZE);
        return 1;
    }

    /* Pick mode */
    const struct mode_def *mode = NULL;
    size_t mi;
    for (mi = 0; mi < N_MODES; mi++) {
        if (strcmp(argv[1], MODES[mi].name) == 0) {
            mode = &MODES[mi];
            break;
        }
    }
    if (!mode) {
        fprintf(stderr, "Unknown mode '%s' (expected 'top' or 'side')\n",
                argv[1]);
        return 1;
    }

    int pid          = atoi(argv[2]);
    int udp_port     = argc > 3 ? atoi(argv[3]) : mode->default_port;
    int interval_us  = argc > 4 ? atoi(argv[4]) : 20000;

    char path[64];
    snprintf(path, sizeof(path), "/proc/%d/mem", pid);
    int memfd = open(path, O_RDONLY);
    if (memfd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", path, strerror(errno));
        return 1;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }

    /* Big send buffer for ~60 KB datagrams */
    int sndbuf = mode->frame_size * 4;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons((unsigned short)udp_port);
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        perror("bind"); return 1;
    }

    fprintf(stderr, "[radar_streamer:%s] pid=%d frame=%d interval=%dus UDP:%d\n",
            mode->name, pid, mode->frame_size, interval_us, udp_port);

    static unsigned char buf[MAX_FRAME_SIZE];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int has_client = 0;
    unsigned long frame = 0;
    int read_errors = 0;

    for (;;) {
        /* Poll for client registration (non-blocking) */
        struct pollfd pfd = { .fd = sock, .events = POLLIN };
        while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
            char tmp[64];
            socklen_t cl = sizeof(client_addr);
            ssize_t r = recvfrom(sock, tmp, sizeof(tmp), 0,
                                 (struct sockaddr *)&client_addr, &cl);
            if (r >= 0) {
                if (!has_client) {
                    fprintf(stderr,
                            "[radar_streamer:%s] client registered: %s:%d\n",
                            mode->name,
                            inet_ntoa(client_addr.sin_addr),
                            ntohs(client_addr.sin_port));
                }
                client_len = cl;
                has_client = 1;
            }
        }

        if (mode->build(memfd, buf) < 0) {
            read_errors++;
            if (read_errors >= 50) {
                fprintf(stderr, "[radar_streamer:%s] too many read errors, exiting\n",
                        mode->name);
                break;
            }
            usleep(100000);
            continue;
        }
        read_errors = 0;

        if (has_client) {
            sendto(sock, buf, mode->frame_size, 0,
                   (struct sockaddr *)&client_addr, client_len);
            frame++;
        }

        if (interval_us > 0)
            usleep(interval_us);
    }

    fprintf(stderr, "[radar_streamer:%s] sent %lu frames\n", mode->name, frame);
    close(sock);
    close(memfd);
    return 0;
}
