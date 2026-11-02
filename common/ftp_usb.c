#include "ftp_usb.h"
#include "ftp_format.h"
#include "ftp_clock.h"

#include <libusb.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_XFERS 32
#define MAX_PKTS  16

typedef struct {
    struct libusb_transfer *xfer[2];   // [0] = EP 0x03 (out 1/2), [1] = EP 0x04 (out 3/4)
    uint32_t gen_sent[2];              // generation each endpoint has submitted
    uint32_t gen_rendered;             // generation currently in `staging`
    uint32_t sizes[MAX_PKTS];          // frames per packet for that generation
    uint32_t total;
    float   *staging;                  // 4ch float, MAX_PKTS * max frames
} out_slot;

struct ftp_dev {
    libusb_context       *ctx;
    libusb_device_handle *h;
    ftp_callbacks         cb;
    int                   verbose;

    pthread_t             thread;
    atomic_bool           quit;
    atomic_bool           gone;

    pthread_mutex_t       lock;
    pthread_cond_t        cond;
    int                   inflight;     // audio transfers owned by libusb
    atomic_bool           stopping;
    bool                  running;

    ftp_stream_config     cfg;
    uint32_t              max_frames;   // per packet
    int                   alt;          // streaming alt setting
    int                   max_packet;   // wMaxPacketSize of that alt
    int                   n_out;        // 1 = outputs 1/2 only, 2 = outputs 1-4
    bool                  use_in;
    ftp_sched             sched;
    uint64_t              out_pos;      // frames rendered so far
    uint64_t              in_pos;       // frames captured so far
    out_slot              slots[MAX_XFERS];
    struct libusb_transfer *in_xfer[MAX_XFERS];
    float                *in_tmp;

    ftp_dll               dll;
    bool                  dll_init;
    double                ticks_per_sec;
    double                last_completion;

    ftp_stats             stats;
    struct libusb_transfer *midi_in;
    uint8_t               midi_buf[64];
};

const char *ftp_strerror(int err) { return libusb_error_name(err); }

#define LOG(d, ...) do { if ((d)->verbose) { fprintf(stderr, "ftp: " __VA_ARGS__); fputc('\n', stderr); } } while (0)

static void report_xrun(ftp_dev *d, const char *what)
{
    if (d->cb.xrun) d->cb.xrun(d->cb.ctx, what);
}

static void mark_gone(ftp_dev *d)
{
    if (!atomic_exchange(&d->gone, true) && d->cb.gone) d->cb.gone(d->cb.ctx);
}

static void make_realtime(void)
{
    mach_timebase_info_data_t tb;
    mach_timebase_info(&tb);
    double ns_to_abs = (double)tb.denom / (double)tb.numer;
    thread_time_constraint_policy_data_t pol = {
        .period = (uint32_t)(1000000 * ns_to_abs),       // 1 ms
        .computation = (uint32_t)(250000 * ns_to_abs),   // 0.25 ms
        .constraint = (uint32_t)(1000000 * ns_to_abs),
        .preemptible = 1,
    };
    thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY,
                      (thread_policy_t)&pol, THREAD_TIME_CONSTRAINT_POLICY_COUNT);
}

static void *event_thread(void *arg)
{
    ftp_dev *d = arg;
    pthread_setname_np("fasttrack-usb");
    make_realtime();
    while (!atomic_load(&d->quit)) {
        struct timeval tv = { 0, 100000 };
        int r = libusb_handle_events_timeout_completed(d->ctx, &tv, NULL);
        if (r == LIBUSB_ERROR_NO_DEVICE) mark_gone(d);
    }
    return NULL;
}

/* ------------------------------------------------------------------ MIDI */

static void midi_in_cb(struct libusb_transfer *t)
{
    ftp_dev *d = t->user_data;
    if (t->status == LIBUSB_TRANSFER_NO_DEVICE) { mark_gone(d); return; }
    if (t->status == LIBUSB_TRANSFER_CANCELLED) return;
    if (t->status == LIBUSB_TRANSFER_COMPLETED && t->actual_length >= 4 && d->cb.midi_in)
        d->cb.midi_in(d->cb.ctx, t->buffer, (size_t)t->actual_length & ~(size_t)3);
    if (!atomic_load(&d->quit) && libusb_submit_transfer(t) != 0)
        LOG(d, "MIDI IN resubmit failed");
}

static void midi_out_cb(struct libusb_transfer *t)
{
    if (t->status == LIBUSB_TRANSFER_NO_DEVICE) mark_gone(t->user_data);
    free(t->buffer);
    libusb_free_transfer(t);
}

int ftp_midi_send(ftp_dev *d, const uint8_t *pkts, size_t len)
{
    if (!d || atomic_load(&d->gone)) return LIBUSB_ERROR_NO_DEVICE;
    if (len == 0) return 0;
    uint8_t *copy = malloc(len);
    struct libusb_transfer *t = libusb_alloc_transfer(0);
    if (!copy || !t) { free(copy); libusb_free_transfer(t); return LIBUSB_ERROR_NO_MEM; }
    memcpy(copy, pkts, len);
    libusb_fill_bulk_transfer(t, d->h, FTP_EP_MIDI_OUT, copy, (int)len, midi_out_cb, d, 1000);
    int r = libusb_submit_transfer(t);
    if (r) { free(copy); libusb_free_transfer(t); }
    return r;
}

/* ----------------------------------------------------------------- audio */

static void xfer_done(ftp_dev *d)
{
    pthread_mutex_lock(&d->lock);
    if (--d->inflight == 0) pthread_cond_broadcast(&d->cond);
    pthread_mutex_unlock(&d->lock);
}

static void clock_tick(ftp_dev *d)
{
    double now = (double)mach_absolute_time();
    double period = d->cfg.pkts * d->ticks_per_sec / 1000.0;

    if (d->dll_init) {
        double gap = now - d->last_completion;
        if (gap > period * (d->cfg.xfers - 1)) {
            report_xrun(d, "late completion: USB queue ran dry");
            d->dll_init = false; // the device clock line jumped; resync
        }
    }
    d->last_completion = now;

    if (!d->dll_init) {
        ftp_dll_init(&d->dll, now, period, d->cfg.pkts / 1000.0, 2.0);
        d->dll_init = true;
    } else {
        ftp_dll_update(&d->dll, now);
        if (d->dll.ticks == (uint64_t)(2000 / d->cfg.pkts)) ftp_dll_set_bw(&d->dll, 0.5);
    }
    if (d->cb.clock) {
        double frames_per_tick = (double)d->cfg.rate * d->cfg.pkts / 1000.0;
        d->cb.clock(d->cb.ctx, d->out_pos, (uint64_t)d->dll.t0, d->dll.e2 / frames_per_tick);
    }
}

static void render_slot(ftp_dev *d, out_slot *s, bool tick)
{
    s->total = 0;
    for (int i = 0; i < d->cfg.pkts; i++) {
        s->sizes[i] = ftp_sched_next(&d->sched);
        s->total += s->sizes[i];
    }
    if (d->cb.render) d->cb.render(d->cb.ctx, d->out_pos, s->staging, s->total);
    else memset(s->staging, 0, sizeof(float) * FTP_OUT_CHANNELS * s->total);
    d->out_pos += s->total;
    s->gen_rendered++;
    if (tick) clock_tick(d);
}

static void fill_out(ftp_dev *d, out_slot *s, int e)
{
    struct libusb_transfer *t = s->xfer[e];
    (d->cfg.little_endian ? ftp_pack_s24le : ftp_pack_s24be)(t->buffer, s->staging, s->total, FTP_OUT_CHANNELS, (unsigned)(2 * e));
    t->length = (int)(s->total * FTP_BYTES_PER_FRAME);
    for (int i = 0; i < d->cfg.pkts; i++)
        t->iso_packet_desc[i].length = s->sizes[i] * FTP_BYTES_PER_FRAME;
    s->gen_sent[e] = s->gen_rendered;
}

static void out_cb(struct libusb_transfer *t)
{
    ftp_dev *d = t->user_data;
    out_slot *s = NULL;
    int e = (t->endpoint == FTP_EP_OUT12) ? 0 : 1;
    for (int idx = 0; idx < d->cfg.xfers; idx++)
        if (d->slots[idx].xfer[e] == t) { s = &d->slots[idx]; break; }

    d->stats.out_completions++;
    d->stats.out_ep_completions[t->endpoint == FTP_EP_OUT12 ? 0 : 1]++;
    d->stats.out_last_status = t->status;
    if (t->status == LIBUSB_TRANSFER_NO_DEVICE) mark_gone(d);
    if (!s || atomic_load(&d->stopping) || atomic_load(&d->gone) ||
        (t->status != LIBUSB_TRANSFER_COMPLETED && t->status != LIBUSB_TRANSFER_ERROR)) {
        xfer_done(d);
        return;
    }
    for (int i = 0; i < t->num_iso_packets; i++)
        if (t->iso_packet_desc[i].status != LIBUSB_TRANSFER_COMPLETED) {
            d->stats.out_pkt_errors++;
            d->stats.out_last_pkt_status = t->iso_packet_desc[i].status;
        }

    // The first endpoint to complete renders the next generation for the slot;
    // the second one only packs its channel pair from the staging buffer.
    if (s->gen_rendered == s->gen_sent[e]) render_slot(d, s, true);
    fill_out(d, s, e);
    int sr = libusb_submit_transfer(t);
    if (sr != 0) {
        d->stats.submit_errors++;
        d->stats.last_submit_error = sr;
        report_xrun(d, "OUT resubmit failed");
        xfer_done(d);
    }
}

static void in_cb(struct libusb_transfer *t)
{
    ftp_dev *d = t->user_data;
    d->stats.in_completions++;
    d->stats.in_last_status = t->status;
    if (t->status == LIBUSB_TRANSFER_NO_DEVICE) mark_gone(d);
    if (atomic_load(&d->stopping) || atomic_load(&d->gone) ||
        (t->status != LIBUSB_TRANSFER_COMPLETED && t->status != LIBUSB_TRANSFER_ERROR)) {
        xfer_done(d);
        return;
    }
    uint32_t frames = 0;
    for (int i = 0; i < t->num_iso_packets; i++) {
        struct libusb_iso_packet_descriptor *p = &t->iso_packet_desc[i];
        if (p->status != LIBUSB_TRANSFER_COMPLETED) {
            d->stats.in_pkt_errors++;
            d->stats.in_last_pkt_status = p->status;
            continue;
        }
        d->stats.in_bytes += p->actual_length;
        if (p->actual_length) d->stats.in_last_len = p->actual_length;
        uint32_t n = p->actual_length / FTP_BYTES_PER_FRAME;
        const uint8_t *src = t->buffer + (size_t)i * (size_t)d->max_packet;
        (d->cfg.little_endian ? ftp_unpack_s24le : ftp_unpack_s24be)(d->in_tmp + (size_t)frames * FTP_IN_CHANNELS, src, n, FTP_IN_CHANNELS, 0);
        frames += n;
    }
    if (frames && d->cb.capture) d->cb.capture(d->cb.ctx, d->in_pos, d->in_tmp, frames);
    d->in_pos += frames;
    int sr = libusb_submit_transfer(t);
    if (sr != 0) {
        d->stats.submit_errors++;
        d->stats.last_submit_error = sr;
        report_xrun(d, "IN resubmit failed");
        xfer_done(d);
    }
}

/* ------------------------------------------------------------- control */

bool ftp_rate_supported(uint32_t rate)
{
    return rate == 44100 || rate == 48000 || rate == 88200 || rate == 96000;
}

static int set_rate(ftp_dev *d, uint8_t ep, uint32_t rate)
{
    // UAC1 SET_CUR SAMPLING_FREQ_CONTROL on the endpoint (Linux: set_sample_rate_v1)
    uint8_t data[3] = { (uint8_t)rate, (uint8_t)(rate >> 8), (uint8_t)(rate >> 16) };
    int r = libusb_control_transfer(d->h, 0x22, 0x01, 0x0100, ep, data, 3, 1000);
    if (r < 0) return r;
    uint8_t rb[3] = { 0 };
    if (libusb_control_transfer(d->h, 0xA2, 0x81, 0x0100, ep, rb, 3, 1000) == 3) {
        uint32_t got = rb[0] | (rb[1] << 8) | ((uint32_t)rb[2] << 16);
        if (got != rate) LOG(d, "EP 0x%02x: requested %u Hz, device reports %u Hz", ep, rate, got);
    }
    return 0;
}

int ftp_open(ftp_dev **out, const ftp_callbacks *cb, int verbose)
{
    ftp_dev *d = calloc(1, sizeof(*d));
    if (!d) return LIBUSB_ERROR_NO_MEM;
    d->verbose = verbose;
    if (cb) d->cb = *cb;
    pthread_mutex_init(&d->lock, NULL);
    pthread_cond_init(&d->cond, NULL);
    mach_timebase_info_data_t tb;
    mach_timebase_info(&tb);
    d->ticks_per_sec = 1e9 * (double)tb.denom / (double)tb.numer;

    int r = libusb_init_context(&d->ctx, NULL, 0);
    if (r) { free(d); return r; }

    d->h = libusb_open_device_with_vid_pid(d->ctx, FTP_VID, FTP_PID);
    if (!d->h) { r = LIBUSB_ERROR_NOT_FOUND; goto fail; }

    // Detach AppleUSBAudio. On macOS this "captures" the whole device (root).
    r = libusb_detach_kernel_driver(d->h, FTP_IF_CONTROL);
    if (r && r != LIBUSB_ERROR_NOT_FOUND) { LOG(d, "detach kernel driver: %s", libusb_error_name(r)); goto fail; }

    int cfg = 0;
    libusb_get_configuration(d->h, &cfg);
    if (cfg != FTP_CONFIG_FULL) {
        LOG(d, "switching from configuration %d to %d", cfg, FTP_CONFIG_FULL);
        r = libusb_set_configuration(d->h, FTP_CONFIG_FULL);
        if (r) { LOG(d, "set configuration: %s", libusb_error_name(r)); goto fail; }
    }
    const int ifs[] = { FTP_IF_MIDI, FTP_IF_OUT12, FTP_IF_OUT34, FTP_IF_IN_ANALOG, FTP_IF_IN_DIGITAL };
    for (size_t i = 0; i < sizeof ifs / sizeof ifs[0]; i++) {
        r = libusb_claim_interface(d->h, ifs[i]);
        if (r) { LOG(d, "claim interface %d: %s", ifs[i], libusb_error_name(r)); goto fail; }
    }

    d->midi_in = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(d->midi_in, d->h, FTP_EP_MIDI_IN, d->midi_buf, sizeof d->midi_buf, midi_in_cb, d, 0);
    if (libusb_submit_transfer(d->midi_in)) LOG(d, "MIDI IN submit failed");

    pthread_create(&d->thread, NULL, event_thread, d);
    *out = d;
    return 0;

fail:
    if (d->h) libusb_close(d->h);
    libusb_exit(d->ctx);
    free(d);
    return r;
}

bool ftp_running(const ftp_dev *d) { return d && d->running; }

void ftp_get_stats(const ftp_dev *d, ftp_stats *out) { *out = d->stats; }

uint32_t ftp_output_queue_frames(const ftp_dev *d)
{
    return (uint32_t)((uint64_t)d->cfg.rate * (uint64_t)(d->cfg.xfers * d->cfg.pkts) / 1000);
}

static void free_streams(ftp_dev *d)
{
    for (int i = 0; i < MAX_XFERS; i++) {
        out_slot *s = &d->slots[i];
        for (int e = 0; e < 2; e++) {
            if (s->xfer[e]) { free(s->xfer[e]->buffer); libusb_free_transfer(s->xfer[e]); s->xfer[e] = NULL; }
        }
        free(s->staging); s->staging = NULL;
        if (d->in_xfer[i]) { free(d->in_xfer[i]->buffer); libusb_free_transfer(d->in_xfer[i]); d->in_xfer[i] = NULL; }
    }
    free(d->in_tmp); d->in_tmp = NULL;
}

int ftp_start(ftp_dev *d, const ftp_stream_config *cfg)
{
    if (d->running) ftp_stop(d);
    if (atomic_load(&d->gone)) return LIBUSB_ERROR_NO_DEVICE;
    if (!ftp_rate_supported(cfg->rate)) return LIBUSB_ERROR_INVALID_PARAM;

    d->cfg = *cfg;
    if (d->cfg.xfers <= 1 || d->cfg.xfers > MAX_XFERS) d->cfg.xfers = 8;
    if (d->cfg.pkts <= 0 || d->cfg.pkts > MAX_PKTS) d->cfg.pkts = 2;
    if (d->cfg.input_iface != FTP_IF_IN_DIGITAL) d->cfg.input_iface = FTP_IF_IN_ANALOG;
    const char *e;
    if ((e = getenv("FTP_XFERS"))) d->cfg.xfers = atoi(e) > 1 && atoi(e) <= MAX_XFERS ? atoi(e) : d->cfg.xfers;
    if ((e = getenv("FTP_PKTS")))  d->cfg.pkts  = atoi(e) > 0 && atoi(e) <= MAX_PKTS ? atoi(e) : d->cfg.pkts;

    int in_if = d->cfg.input_iface, other_in = in_if == FTP_IF_IN_ANALOG ? FTP_IF_IN_DIGITAL : FTP_IF_IN_ANALOG;
    uint8_t in_ep = in_if == FTP_IF_IN_ANALOG ? FTP_EP_IN_ANALOG : FTP_EP_IN_DIGITAL;
    int r;
    memset(&d->stats, 0, sizeof d->stats);
    // Full-speed USB has ~1150 bytes/frame for periodic traffic. Alt 3
    // (584 B packets) only fits two streams, so 88.2/96 kHz is 2-in/2-out
    // (outputs 1/2); at 44.1/48 kHz alt 2 (296 B) carries all 4 outputs + input.
    bool high = d->cfg.rate > 48000;
    d->alt = high ? FTP_ALT_24BIT_96K : FTP_ALT_24BIT_48K;
    d->max_packet = high ? FTP_MAX_PACKET : FTP_MAX_PACKET_48K;
    d->n_out = (high || d->cfg.no_out34) ? 1 : 2;
    d->use_in = !d->cfg.no_input;
    if ((r = libusb_set_interface_alt_setting(d->h, other_in, 0))) { LOG(d, "IF%d alt 0: %s", other_in, libusb_error_name(r)); return r; }
    if ((r = libusb_set_interface_alt_setting(d->h, FTP_IF_OUT12, d->alt))) { LOG(d, "IF2 alt %d: %s", d->alt, libusb_error_name(r)); return r; }
    if ((r = libusb_set_interface_alt_setting(d->h, FTP_IF_OUT34, d->n_out == 2 ? d->alt : 0))) { LOG(d, "IF3: %s", libusb_error_name(r)); return r; }
    if ((r = libusb_set_interface_alt_setting(d->h, in_if, d->use_in ? d->alt : 0))) { LOG(d, "IF%d: %s", in_if, libusb_error_name(r)); return r; }
    if ((r = set_rate(d, FTP_EP_OUT12, d->cfg.rate))) { LOG(d, "set rate EP3: %s", libusb_error_name(r)); return r; }
    if (d->n_out == 2 && (r = set_rate(d, FTP_EP_OUT34, d->cfg.rate))) { LOG(d, "set rate EP4: %s", libusb_error_name(r)); return r; }

    d->max_frames = ftp_sched_max(d->cfg.rate);
    ftp_sched_init(&d->sched, d->cfg.rate);
    d->out_pos = d->in_pos = 0;
    d->dll_init = false;
    atomic_store(&d->stopping, false);

    size_t out_len = (size_t)d->cfg.pkts * d->max_frames * FTP_BYTES_PER_FRAME;
    size_t in_len = (size_t)d->cfg.pkts * (size_t)d->max_packet;
    d->in_tmp = calloc((size_t)d->cfg.pkts * (FTP_MAX_PACKET / FTP_BYTES_PER_FRAME) * FTP_IN_CHANNELS, sizeof(float));
    for (int i = 0; i < d->cfg.xfers; i++) {
        out_slot *s = &d->slots[i];
        memset(s, 0, sizeof *s);
        s->staging = calloc((size_t)d->cfg.pkts * d->max_frames * FTP_OUT_CHANNELS, sizeof(float));
        for (int ep = 0; ep < d->n_out; ep++) {
            s->xfer[ep] = libusb_alloc_transfer(d->cfg.pkts);
            libusb_fill_iso_transfer(s->xfer[ep], d->h, ep == 0 ? FTP_EP_OUT12 : FTP_EP_OUT34,
                                     calloc(1, out_len), (int)out_len, d->cfg.pkts, out_cb, d, 0);
        }
        if (!d->use_in) continue;
        d->in_xfer[i] = libusb_alloc_transfer(d->cfg.pkts);
        libusb_fill_iso_transfer(d->in_xfer[i], d->h, in_ep, calloc(1, in_len), (int)in_len, d->cfg.pkts, in_cb, d, 0);
        libusb_set_iso_packet_lengths(d->in_xfer[i], (unsigned)d->max_packet);
    }

    // Prime the queue: render every slot once and submit everything.
    for (int i = 0; i < d->cfg.xfers; i++) {
        out_slot *s = &d->slots[i];
        render_slot(d, s, false);
        for (int ep = 0; ep < d->n_out; ep++) {
            fill_out(d, s, ep);
            pthread_mutex_lock(&d->lock);
            int sr = libusb_submit_transfer(s->xfer[ep]);
            if (sr == 0) d->inflight++;
            pthread_mutex_unlock(&d->lock);
            if (sr) { d->stats.submit_errors++; d->stats.last_submit_error = sr; LOG(d, "submit EP 0x%02x: %s", s->xfer[ep]->endpoint, libusb_error_name(sr)); }
        }
        if (!d->use_in) continue;
        pthread_mutex_lock(&d->lock);
        int sr = libusb_submit_transfer(d->in_xfer[i]);
        if (sr == 0) d->inflight++;
        pthread_mutex_unlock(&d->lock);
        if (sr) { d->stats.submit_errors++; d->stats.last_submit_error = sr; LOG(d, "submit IN EP 0x%02x: %s", in_ep, libusb_error_name(sr)); }
    }
    d->running = true;
    LOG(d, "streaming %u Hz alt %d: outputs %s, input %s, %d x %d ms transfers", d->cfg.rate, d->alt,
        d->n_out == 2 ? "1-4" : "1/2", d->use_in ? (in_if == FTP_IF_IN_ANALOG ? "analog" : "digital") : "off",
        d->cfg.xfers, d->cfg.pkts);
    return 0;
}

void ftp_stop(ftp_dev *d)
{
    if (!d->running) return;
    atomic_store(&d->stopping, true);
    for (int i = 0; i < d->cfg.xfers; i++) {
        for (int e = 0; e < 2; e++) if (d->slots[i].xfer[e]) libusb_cancel_transfer(d->slots[i].xfer[e]);
        if (d->in_xfer[i]) libusb_cancel_transfer(d->in_xfer[i]);
    }
    pthread_mutex_lock(&d->lock);
    while (d->inflight > 0) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 2;
        if (pthread_cond_timedwait(&d->cond, &d->lock, &ts) != 0) { LOG(d, "stop: %d transfers stuck", d->inflight); break; }
    }
    bool clean = d->inflight == 0;
    pthread_mutex_unlock(&d->lock);
    if (!atomic_load(&d->gone)) {
        libusb_set_interface_alt_setting(d->h, FTP_IF_OUT12, 0);
        libusb_set_interface_alt_setting(d->h, FTP_IF_OUT34, 0);
        libusb_set_interface_alt_setting(d->h, d->cfg.input_iface, 0);
    }
    if (clean) free_streams(d); // else leak rather than free memory libusb still owns
    d->running = false;
}

void ftp_close(ftp_dev *d)
{
    if (!d) return;
    ftp_stop(d);
    atomic_store(&d->quit, true);
    if (d->midi_in) libusb_cancel_transfer(d->midi_in);
    libusb_interrupt_event_handler(d->ctx);
    pthread_join(d->thread, NULL);
    struct timeval tv = { 0, 100000 };
    libusb_handle_events_timeout_completed(d->ctx, &tv, NULL); // reap cancellations
    if (d->midi_in) libusb_free_transfer(d->midi_in);
    if (!atomic_load(&d->gone)) {
        const int ifs[] = { FTP_IF_MIDI, FTP_IF_OUT12, FTP_IF_OUT34, FTP_IF_IN_ANALOG, FTP_IF_IN_DIGITAL };
        for (size_t i = 0; i < sizeof ifs / sizeof ifs[0]; i++) libusb_release_interface(d->h, ifs[i]);
        libusb_attach_kernel_driver(d->h, FTP_IF_CONTROL); // re-enumerate -> AppleUSBAudio takes over
    }
    libusb_close(d->h);
    libusb_exit(d->ctx);
    pthread_mutex_destroy(&d->lock);
    pthread_cond_destroy(&d->cond);
    free(d);
}
