// Shared memory between fasttrackd (root, owns USB) and the Core Audio
// plug-in (inside coreaudiod). Mapped through xpc_shmem.
//
// Time base: "device sample time" = output frames handed to the USB engine
// since the last start. The daemon publishes a clock line (anchor sample,
// anchor host time, host ticks per sample) via a seqlock; the plug-in derives
// zero time stamps from it. Ring buffers are indexed by device sample time
// modulo FTP_RING_FRAMES and hold 24-bit packed little-endian integers
// (the HAL physical format); the daemon converts to the device's big-endian.
#pragma once

#include <stdatomic.h>
#include <stdint.h>

#define FTP_SERVICE_NAME   "be.jsix.fasttrackd"
#define FTP_SHM_MAGIC      0x46545031u /* 'FTP1' */
#define FTP_SHM_VERSION    1
#define FTP_RING_FRAMES    16384u
#define FTP_SHM_OUT_CH     4
#define FTP_SHM_IN_CH      2
#define FTP_SHM_BPS        3   /* bytes per sample */

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t ring_frames;
    uint32_t _pad0;

    _Atomic uint32_t device_present;
    _Atomic uint32_t running;
    _Atomic uint32_t rate;
    _Atomic uint32_t out_pairs;         // 2 at 44.1/48 kHz, 1 at 88.2/96 kHz
    _Atomic uint32_t output_queue_frames;
    _Atomic uint32_t xruns;
    _Atomic uint64_t generation;        // bumps on every (re)start: new time line

    // clock line, seqlock protected (odd = write in progress)
    _Atomic uint32_t clock_seq;
    uint32_t _pad1;
    uint64_t anchor_sample;
    uint64_t anchor_host;
    double   ticks_per_sample;

    _Atomic uint64_t in_newest;         // input written up to (exclusive), device sample time
    _Atomic uint64_t out_consumed;      // output read by the daemon up to (exclusive)

    uint8_t  _pad2[64];
    uint8_t  out_ring[FTP_RING_FRAMES * FTP_SHM_OUT_CH * FTP_SHM_BPS];
    uint8_t  in_ring[FTP_RING_FRAMES * FTP_SHM_IN_CH * FTP_SHM_BPS];
} ftp_shm;

static inline void ftp_shm_write_clock(ftp_shm *s, uint64_t sample, uint64_t host, double tps)
{
    uint32_t q = atomic_load_explicit(&s->clock_seq, memory_order_relaxed);
    atomic_store_explicit(&s->clock_seq, q + 1, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    s->anchor_sample = sample;
    s->anchor_host = host;
    s->ticks_per_sample = tps;
    atomic_store_explicit(&s->clock_seq, q + 2, memory_order_release);
}

// Returns 0 if no consistent clock could be read.
static inline int ftp_shm_read_clock(const ftp_shm *s, uint64_t *sample, uint64_t *host, double *tps)
{
    for (int tries = 0; tries < 100; tries++) {
        uint32_t a = atomic_load_explicit(&((ftp_shm *)s)->clock_seq, memory_order_acquire);
        if (a & 1) continue;
        uint64_t sm = s->anchor_sample, h = s->anchor_host;
        double t = s->ticks_per_sample;
        atomic_thread_fence(memory_order_acquire);
        uint32_t b = atomic_load_explicit(&((ftp_shm *)s)->clock_seq, memory_order_relaxed);
        if (a == b) { *sample = sm; *host = h; *tps = t; return t > 0; }
    }
    return 0;
}
