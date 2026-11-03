// fasttrackd: root daemon that owns the M-Audio FastTrack Pro over libusb
// and serves audio (shared memory) and MIDI (XPC) to the Core Audio plug-in
// and the MIDI agent.
#include "../common/ftp_usb.h"
#include "../common/ftp_shm.h"
#include "../common/ftp_proto.h"
#include "../common/ftp_format.h"

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/usb/IOUSBLib.h>
#include <dispatch/dispatch.h>
#include <mach/mach_time.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <xpc/xpc.h>

static dispatch_queue_t   g_q;          // serializes all daemon state
static ftp_dev           *g_dev;
static ftp_shm           *g_shm;
static size_t             g_shm_size;
static xpc_object_t       g_shm_obj;
static xpc_connection_t   g_audio_peer; // the peer that started streaming
static xpc_object_t       g_peers;      // array of all peers
static xpc_object_t       g_midi_peers; // array of MIDI subscribers
static uint32_t           g_rate = 48000;
static int                g_input = FTP_IF_IN_ANALOG;
static bool               g_want_running;
static bool               g_verbose;

// USB-thread-only diagnostics, reported every few seconds
static uint64_t           g_out_underruns, g_out_missing_frames;
static int64_t            g_margin_min = INT64_MAX;
static double             g_jitter_max_ms;
static double             g_ms_per_tick;
static int                g_xrun_logs;  // reasons logged since the last stream start

// USB-thread-only state for input alignment
static int64_t            g_in_offset;
static bool               g_in_aligned;
static uint64_t           g_last_out_pos;

static void logf_(const char *fmt, ...)
{
    char ts[32];
    time_t t = time(NULL);
    strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S", localtime(&t));
    fprintf(stderr, "%s fasttrackd: ", ts);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}
#define LOGF(...) logf_(__VA_ARGS__)

/* --------------------------------------------------------------- audio io */

static void on_render(void *ctx, uint64_t pos, float *buf, uint32_t frames)
{
    ftp_shm *s = g_shm;
    const uint32_t mask = FTP_RING_FRAMES - 1, stride = FTP_SHM_OUT_CH * FTP_SHM_BPS;
    uint64_t written = atomic_load_explicit(&s->out_written_end, memory_order_acquire);
    if (written) { // plug-in has written at least once this run
        int64_t margin = (int64_t)written - (int64_t)(pos + frames);
        if (margin < g_margin_min) g_margin_min = margin;
        if (margin < 0) { g_out_underruns++; g_out_missing_frames += (uint64_t)(margin < -(int64_t)frames ? frames : -margin); }
    }
    uint32_t done = 0;
    while (done < frames) {
        uint32_t idx = (uint32_t)((pos + done) & mask);
        uint32_t n = frames - done;
        if (n > FTP_RING_FRAMES - idx) n = FTP_RING_FRAMES - idx;
        uint8_t *src = s->out_ring + (size_t)idx * stride;
        ftp_i24le_to_f32(buf + (size_t)done * FTP_SHM_OUT_CH, src, (size_t)n * FTP_SHM_OUT_CH);
        memset(src, 0, (size_t)n * stride); // never replay stale audio
        done += n;
    }
    g_last_out_pos = pos + frames;
    atomic_store_explicit(&s->out_consumed, pos + frames, memory_order_release);
}

static void on_capture(void *ctx, uint64_t pos, const float *buf, uint32_t frames)
{
    ftp_shm *s = g_shm;
    // Map the input frame counter onto the device (output) time line so that
    // the newest captured frame lines up with "now" on the clock line.
    int64_t newest = (int64_t)(pos + frames) + g_in_offset;
    int64_t target = (int64_t)g_last_out_pos;
    // in/out completion order can swing the alignment by a whole transfer
    int64_t tol = (int64_t)atomic_load(&s->output_queue_frames) + (int64_t)atomic_load(&s->rate) * 2 / 1000;
    if (!g_in_aligned || newest - target > tol || target - newest > tol) {
        if (g_in_aligned) { atomic_fetch_add(&s->xruns, 1); if (g_xrun_logs < 5) { g_xrun_logs++; LOGF("xrun: input re-aligned by %lld frames", (long long)(target - newest)); } }
        g_in_offset = target - (int64_t)(pos + frames);
        g_in_aligned = true;
    }
    uint64_t start = (uint64_t)((int64_t)pos + g_in_offset);
    const uint32_t mask = FTP_RING_FRAMES - 1, stride = FTP_SHM_IN_CH * FTP_SHM_BPS;
    uint32_t done = 0;
    while (done < frames) {
        uint32_t idx = (uint32_t)((start + done) & mask);
        uint32_t n = frames - done;
        if (n > FTP_RING_FRAMES - idx) n = FTP_RING_FRAMES - idx;
        ftp_f32_to_i24le(s->in_ring + (size_t)idx * stride, buf + (size_t)done * FTP_SHM_IN_CH, (size_t)n * FTP_SHM_IN_CH);
        done += n;
    }
    atomic_store_explicit(&s->in_newest, start + frames, memory_order_release);
}

static void on_clock(void *ctx, uint64_t sample, uint64_t host, double tps)
{
    ftp_shm_write_clock(g_shm, sample, host, tps);
    double j = fabs((double)mach_absolute_time() - (double)host) * g_ms_per_tick;
    if (j > g_jitter_max_ms) g_jitter_max_ms = j;
}

static void report_stats(void)
{
    if (!atomic_load(&g_shm->running)) return;
    LOGF("stats: out underruns %llu (%llu frames), min out margin %.2f ms, in underruns %u, xruns %u, clock jitter max %.2f ms",
         (unsigned long long)g_out_underruns, (unsigned long long)g_out_missing_frames,
         g_margin_min == INT64_MAX ? 0.0 : g_margin_min * 1000.0 / atomic_load(&g_shm->rate),
         atomic_exchange(&g_shm->in_underruns, 0), atomic_exchange(&g_shm->xruns, 0), g_jitter_max_ms);
    g_out_underruns = g_out_missing_frames = 0;
    g_margin_min = INT64_MAX;
    g_jitter_max_ms = 0;
}

static void broadcast(xpc_object_t peers, xpc_object_t msg)
{
    xpc_array_apply(peers, ^bool(size_t i, xpc_object_t p) {
        xpc_connection_send_message((xpc_connection_t)p, msg);
        return true;
    });
}

static void on_midi_in(void *ctx, const uint8_t *pkts, size_t len)
{
    // drop empty (CIN 0) padding packets
    uint8_t tmp[64];
    size_t n = 0;
    for (size_t i = 0; i + 3 < len && n + 4 <= sizeof tmp; i += 4)
        if (pkts[i] & 0x0f) { memcpy(tmp + n, pkts + i, 4); n += 4; }
    if (!n) return;
    xpc_object_t msg = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_string(msg, FTP_KEY_OP, "midi_in");
    xpc_dictionary_set_data(msg, FTP_KEY_DATA, tmp, n);
    dispatch_async(g_q, ^{ broadcast(g_midi_peers, msg); xpc_release(msg); });
}

static void device_closed(void);

static void on_gone(void *ctx)
{
    dispatch_async(g_q, ^{ LOGF("device disconnected"); device_closed(); });
}

static void on_xrun(void *ctx, const char *what)
{
    atomic_fetch_add(&g_shm->xruns, 1);
    if (g_verbose || g_xrun_logs < 5) { g_xrun_logs++; LOGF("xrun: %s", what); }
}

/* ------------------------------------------------------------ device mgmt */

static void send_status(void)
{
    xpc_object_t msg = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_string(msg, FTP_KEY_OP, "status");
    xpc_dictionary_set_bool(msg, FTP_KEY_PRESENT, g_dev != NULL);
    broadcast(g_peers, msg);
    xpc_release(msg);
}

static int stream_start(void)
{
    if (!g_dev) return -1;
    ftp_shm *s = g_shm;
    atomic_store(&s->running, 0);
    ftp_stop(g_dev);
    memset(s->out_ring, 0, sizeof s->out_ring);
    memset(s->in_ring, 0, sizeof s->in_ring);
    atomic_store(&s->in_newest, 0);
    atomic_store(&s->out_consumed, 0);
    atomic_store(&s->out_written_end, 0);
    g_in_aligned = false;
    g_last_out_pos = 0;
    g_xrun_logs = 0;
    atomic_store(&s->rate, g_rate);
    atomic_store(&s->out_pairs, g_rate > 48000 ? 1 : 2);
    ftp_stream_config cfg = { .rate = g_rate, .input_iface = g_input };
    // seed the clock line at the nominal rate until the DLL locks
    mach_timebase_info_data_t tb;
    mach_timebase_info(&tb);
    double tps = 1e9 * tb.denom / tb.numer / g_rate;
    atomic_fetch_add(&s->generation, 1);
    int r = ftp_start(g_dev, &cfg);
    if (r) { LOGF("start %u Hz failed: %s", g_rate, ftp_strerror(r)); return r; }
    ftp_shm_write_clock(s, g_last_out_pos, mach_absolute_time(), tps);
    atomic_store(&s->output_queue_frames, ftp_output_queue_frames(g_dev));
    atomic_store(&s->running, 1);
    LOGF("streaming at %u Hz, input %s", g_rate, g_input == FTP_IF_IN_ANALOG ? "analog" : "S/PDIF");
    return 0;
}

static void stream_stop(void)
{
    atomic_store(&g_shm->running, 0);
    if (g_dev) ftp_stop(g_dev);
}

static void try_open(void)
{
    if (g_dev) return;
    ftp_callbacks cb = { .render = on_render, .capture = on_capture, .clock = on_clock,
                         .midi_in = on_midi_in, .gone = on_gone, .xrun = on_xrun };
    int r = ftp_open(&g_dev, &cb, g_verbose);
    if (r) {
        g_dev = NULL;
        if (r != -5 /* LIBUSB_ERROR_NOT_FOUND */) LOGF("cannot open device: %s", ftp_strerror(r));
        return;
    }
    LOGF("FastTrack Pro captured, configuration 2 active");
    atomic_store(&g_shm->device_present, 1);
    if (g_want_running) stream_start();
    send_status();
}

static void device_closed(void)
{
    if (!g_dev) return;
    atomic_store(&g_shm->running, 0);
    atomic_store(&g_shm->device_present, 0);
    ftp_close(g_dev);
    g_dev = NULL;
    send_status();
}

static void device_matched(void *refcon, io_iterator_t it)
{
    io_object_t o;
    bool any = false;
    while ((o = IOIteratorNext(it))) { any = true; IOObjectRelease(o); }
    if (any) {
        // give the USB stack a moment to settle after (re-)enumeration
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 500 * NSEC_PER_MSEC), g_q, ^{ try_open(); });
    }
}

static void watch_usb(void)
{
    IONotificationPortRef port = IONotificationPortCreate(kIOMainPortDefault);
    IONotificationPortSetDispatchQueue(port, g_q);
    CFMutableDictionaryRef match = IOServiceMatching("IOUSBHostDevice");
    int vid = FTP_VID, pid = FTP_PID;
    CFNumberRef v = CFNumberCreate(NULL, kCFNumberIntType, &vid), p = CFNumberCreate(NULL, kCFNumberIntType, &pid);
    CFDictionarySetValue(match, CFSTR(kUSBVendorID), v);
    CFDictionarySetValue(match, CFSTR(kUSBProductID), p);
    CFRelease(v); CFRelease(p);
    io_iterator_t it;
    if (IOServiceAddMatchingNotification(port, kIOMatchedNotification, match, device_matched, NULL, &it) == KERN_SUCCESS)
        device_matched(NULL, it); // arms the notification, handles already-present device
}

/* --------------------------------------------------------------------- xpc */

static void array_remove(xpc_object_t *arr, xpc_object_t obj)
{
    xpc_object_t n = xpc_array_create(NULL, 0);
    xpc_array_apply(*arr, ^bool(size_t i, xpc_object_t v) { if (v != obj) xpc_array_append_value(n, v); return true; });
    xpc_release(*arr);
    *arr = n;
}

static void handle_message(xpc_connection_t peer, xpc_object_t msg)
{
    const char *op = xpc_dictionary_get_string(msg, FTP_KEY_OP);
    if (!op) return;
    xpc_object_t reply = xpc_dictionary_create_reply(msg);

    if (!strcmp(op, "hello")) {
        if (reply) {
            xpc_dictionary_set_value(reply, FTP_KEY_SHM, g_shm_obj);
            xpc_dictionary_set_bool(reply, FTP_KEY_PRESENT, g_dev != NULL);
            xpc_dictionary_set_uint64(reply, FTP_KEY_RATE, g_rate);
        }
    } else if (!strcmp(op, "start")) {
        uint32_t rate = (uint32_t)xpc_dictionary_get_uint64(msg, FTP_KEY_RATE);
        int input = (int)xpc_dictionary_get_uint64(msg, FTP_KEY_INPUT);
        if (ftp_rate_supported(rate)) g_rate = rate;
        g_input = input == FTP_IF_IN_DIGITAL ? FTP_IF_IN_DIGITAL : FTP_IF_IN_ANALOG;
        g_want_running = true;
        g_audio_peer = peer;
        int r = g_dev ? stream_start() : -1;
        if (reply) {
            xpc_dictionary_set_bool(reply, FTP_KEY_OK, r == 0);
            if (r) xpc_dictionary_set_string(reply, FTP_KEY_ERROR, g_dev ? ftp_strerror(r) : "device not present");
        }
    } else if (!strcmp(op, "stop")) {
        g_want_running = false;
        g_audio_peer = NULL;
        stream_stop();
        LOGF("stopped");
        if (reply) xpc_dictionary_set_bool(reply, FTP_KEY_OK, true);
    } else if (!strcmp(op, "set_input")) {
        int input = (int)xpc_dictionary_get_uint64(msg, FTP_KEY_INPUT) == FTP_IF_IN_DIGITAL ? FTP_IF_IN_DIGITAL : FTP_IF_IN_ANALOG;
        if (input != g_input) {
            g_input = input;
            if (g_want_running && g_dev) stream_start();
        }
        if (reply) xpc_dictionary_set_bool(reply, FTP_KEY_OK, true);
    } else if (!strcmp(op, "subscribe_midi")) {
        xpc_array_append_value(g_midi_peers, peer);
    } else if (!strcmp(op, "midi_out")) {
        size_t len = 0;
        const void *data = xpc_dictionary_get_data(msg, FTP_KEY_DATA, &len);
        if (g_dev && data && len) ftp_midi_send(g_dev, data, len);
    }
    if (reply) {
        xpc_connection_send_message(peer, reply);
        xpc_release(reply);
    }
}

static void new_peer(xpc_connection_t peer)
{
    xpc_array_append_value(g_peers, peer);
    xpc_connection_set_target_queue(peer, g_q);
    xpc_connection_set_event_handler(peer, ^(xpc_object_t ev) {
        if (xpc_get_type(ev) == XPC_TYPE_DICTIONARY) { handle_message(peer, ev); return; }
        if (ev == XPC_ERROR_CONNECTION_INVALID) {
            array_remove(&g_peers, peer);
            array_remove(&g_midi_peers, peer);
            if (peer == g_audio_peer) {
                LOGF("audio client went away, stopping");
                g_audio_peer = NULL;
                g_want_running = false;
                stream_stop();
            }
        }
    });
    xpc_connection_resume(peer);
}

/* -------------------------------------------------------------------- main */

static void shutdown_and_exit(void)
{
    LOGF("shutting down, handing the device back to macOS");
    if (g_dev) { ftp_close(g_dev); g_dev = NULL; }
    exit(0);
}

int main(int argc, char **argv)
{
    g_verbose = getenv("FTP_VERBOSE") != NULL || (argc > 1 && !strcmp(argv[1], "-v"));
    setvbuf(stderr, NULL, _IOLBF, 0);
    if (geteuid() != 0) LOGF("warning: not running as root, capturing the device will fail");

    g_q = dispatch_queue_create("be.jsix.fasttrackd", DISPATCH_QUEUE_SERIAL);
    g_peers = xpc_array_create(NULL, 0);
    g_midi_peers = xpc_array_create(NULL, 0);

    long page = sysconf(_SC_PAGESIZE);
    g_shm_size = (sizeof(ftp_shm) + (size_t)page - 1) & ~((size_t)page - 1);
    g_shm = mmap(NULL, g_shm_size, PROT_READ | PROT_WRITE, MAP_ANON | MAP_SHARED, -1, 0);
    if (g_shm == MAP_FAILED) { perror("mmap"); return 1; }
    memset(g_shm, 0, g_shm_size);
    g_shm->magic = FTP_SHM_MAGIC;
    g_shm->version = FTP_SHM_VERSION;
    g_shm->ring_frames = FTP_RING_FRAMES;
    const char *e;
    g_shm->safety_out_us = (e = getenv("FTP_SAFETY_OUT_MS")) ? (uint32_t)(atof(e) * 1000) : 8000;
    g_shm->safety_in_us = (e = getenv("FTP_SAFETY_IN_MS")) ? (uint32_t)(atof(e) * 1000) : 12000;
    g_shm_obj = xpc_shmem_create(g_shm, g_shm_size);
    LOGF("safety offsets: output %.1f ms, input %.1f ms", g_shm->safety_out_us / 1000.0, g_shm->safety_in_us / 1000.0);

    xpc_connection_t listener = xpc_connection_create_mach_service(FTP_SERVICE_NAME, g_q, XPC_CONNECTION_MACH_SERVICE_LISTENER);
    xpc_connection_set_event_handler(listener, ^(xpc_object_t ev) {
        if (xpc_get_type(ev) == XPC_TYPE_CONNECTION) new_peer((xpc_connection_t)ev);
    });
    xpc_connection_resume(listener);

    signal(SIGTERM, SIG_IGN);
    signal(SIGINT, SIG_IGN);
    dispatch_source_t st = dispatch_source_create(DISPATCH_SOURCE_TYPE_SIGNAL, SIGTERM, 0, g_q);
    dispatch_source_set_event_handler(st, ^{ shutdown_and_exit(); });
    dispatch_resume(st);
    dispatch_source_t si = dispatch_source_create(DISPATCH_SOURCE_TYPE_SIGNAL, SIGINT, 0, g_q);
    dispatch_source_set_event_handler(si, ^{ shutdown_and_exit(); });
    dispatch_resume(si);

    mach_timebase_info_data_t tb;
    mach_timebase_info(&tb);
    g_ms_per_tick = (double)tb.numer / tb.denom / 1e6;
    dispatch_source_t stats = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, g_q);
    dispatch_source_set_timer(stats, dispatch_time(DISPATCH_TIME_NOW, 5 * NSEC_PER_SEC), 5 * NSEC_PER_SEC, NSEC_PER_SEC / 10);
    dispatch_source_set_event_handler(stats, ^{ report_stats(); });
    dispatch_resume(stats);

    dispatch_async(g_q, ^{ watch_usb(); });
    LOGF("started (service %s)", FTP_SERVICE_NAME);
    dispatch_main();
}
