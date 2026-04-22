/*
 * radar_mem_poke - ptrace-based byte writer for DJI T40 radar app.elf
 *
 * Writes raw bytes into a target process's memory using PTRACE_ATTACH +
 * PTRACE_POKEDATA. Works without gdb (which isn't on the radar rootfs).
 *
 * Usage:
 *   radar_mem_poke PID ADDR BYTE [BYTE ...]
 *
 * Example (set g_flight_mode_from_fc byte @ 0x283295 to 2):
 *   radar_mem_poke $(pidof app.elf) 0x283295 2
 *
 * Example (force byte_17A3F8 = 0 to exit power_mode=1):
 *   radar_mem_poke $(pidof app.elf) 0x17A3F8 0
 *
 * Build (cross-compile for Zynq ARM):
 *   arm-linux-gnueabihf-gcc -O2 -static -o radar_mem_poke radar_mem_poke.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
#include <sys/user.h>

static long parse_ulong(const char *s)
{
    char *end = NULL;
    long v = strtol(s, &end, 0);
    if (!end || *end != '\0') {
        fprintf(stderr, "bad number: %s\n", s);
        exit(1);
    }
    return v;
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr,
            "Usage: %s PID ADDR BYTE [BYTE ...]\n"
            "  PID   target process id\n"
            "  ADDR  absolute virtual address (e.g. 0x283295)\n"
            "  BYTE  one or more byte values (decimal or 0xNN)\n",
            argv[0]);
        return 1;
    }

    pid_t pid = (pid_t)parse_ulong(argv[1]);
    unsigned long addr = (unsigned long)parse_ulong(argv[2]);
    int nbytes = argc - 3;
    unsigned char *bytes = malloc(nbytes);
    if (!bytes) { perror("malloc"); return 1; }
    for (int i = 0; i < nbytes; i++)
        bytes[i] = (unsigned char)parse_ulong(argv[3 + i]);

    if (ptrace(PTRACE_ATTACH, pid, 0, 0) < 0) {
        fprintf(stderr, "PTRACE_ATTACH pid=%d: %s\n", pid, strerror(errno));
        free(bytes);
        return 1;
    }
    int status;
    if (waitpid(pid, &status, 0) < 0) {
        perror("waitpid");
        ptrace(PTRACE_DETACH, pid, 0, 0);
        free(bytes);
        return 1;
    }

    /* PEEK/POKE work on word (sizeof(long)) granularity; do RMW per word. */
    const unsigned long wsize = sizeof(long);
    int written = 0, failed = 0;

    for (int i = 0; i < nbytes; ) {
        unsigned long a = addr + i;
        unsigned long wa = a & ~(wsize - 1);
        errno = 0;
        long word = ptrace(PTRACE_PEEKDATA, pid, (void *)wa, 0);
        if (errno != 0) {
            fprintf(stderr, "PEEKDATA @0x%lx: %s\n", wa, strerror(errno));
            failed++;
            break;
        }
        unsigned char buf[sizeof(long)];
        memcpy(buf, &word, wsize);

        /* Fill as many bytes as fit in this word. */
        while (i < nbytes) {
            unsigned long cur = addr + i;
            if ((cur & ~(wsize - 1)) != wa) break;
            buf[cur - wa] = bytes[i];
            i++;
        }

        memcpy(&word, buf, wsize);
        if (ptrace(PTRACE_POKEDATA, pid, (void *)wa, (void *)word) < 0) {
            fprintf(stderr, "POKEDATA @0x%lx: %s\n", wa, strerror(errno));
            failed++;
            break;
        }
        written++;
    }

    if (ptrace(PTRACE_DETACH, pid, 0, 0) < 0)
        fprintf(stderr, "PTRACE_DETACH: %s\n", strerror(errno));

    if (failed) { free(bytes); return 2; }

    printf("ok: wrote %d byte(s) @ 0x%lx across %d word(s)\n",
           nbytes, addr, written);
    free(bytes);
    return 0;
}
