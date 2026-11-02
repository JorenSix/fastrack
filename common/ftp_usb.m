// USB engine for the M-Audio FastTrack Pro on IOUSBHost.framework.
//
// Replaces the libusb engine: buffers come from -ioDataWithCapacity: (kernel
// memory mapped once, no per-submit wiring), completions arrive directly on
// our dispatch queue, frames are scheduled explicitly, and the clock DLL is
// fed with the controller's per-transaction timestamps instead of
// user-space wake-up times.
#import <Foundation/Foundation.h>
#import <IOKit/IOKitLib.h>
#import <IOKit/IOMessage.h>
#import <IOUSBHost/IOUSBHost.h>
#include <mach/mach_time.h>
#include <pthread.h>
#include <stdatomic.h>

#include "ftp_usb.h"
#include "ftp_format.h"
#include "ftp_clock.h"

#define MAX_XFERS 32
#define MAX_PKTS  16

#define LOG(d, ...) do { if ((d)->verbose) { fprintf(stderr, "ftp: " __VA_ARGS__); fputc('\n', stderr); } } while (0)

typedef struct {
    IOUSBHostIsochronousTransaction tx[MAX_PKTS];
    uint64_t frame;          // first USB frame of this transfer
} iso_xfer;

typedef struct {
    iso_xfer  x[2];          // [0] EP 0x03 (out 1/2), [1] EP 0x04 (out 3/4)
    uint32_t  gen_sent[2];
    uint32_t  gen_rendered;
    uint32_t  sizes[MAX_PKTS];
    uint32_t  total;
    float    *staging;
} out_slot;

@interface FTPEngine : NSObject {
@public
    ftp_callbacks cb;
    int verbose;
    dispatch_queue_t q;

    IOUSBHostDevice *dev;
    IOUSBHostInterface *ifMidi, *ifOut12, *ifOut34, *ifInA, *ifInD;
    IOUSBHostPipe *pOut[2], *pIn, *pMidiIn, *pMidiOut;
    NSMutableData *outData[MAX_XFERS][2], *inData[MAX_XFERS], *midiInData;

    pthread_mutex_t lock;
    pthread_cond_t cond;
    int inflight;
    atomic_bool stopping, gone, closing;
    bool running;

    ftp_stream_config cfg;
    int alt, max_packet, n_out;
    bool use_in;
    uint32_t max_frames;
    ftp_sched sched;
    uint64_t out_pos, in_pos;
    out_slot slots[MAX_XFERS];
    iso_xfer in_x[MAX_XFERS];
    uint64_t next_frame[3];  // per stream: out12, out34, in
    float *in_tmp;

    ftp_dll dll;
    bool dll_init;
    double ticks_per_ms;
    ftp_stats stats;
}
@end
@implementation FTPEngine
@end

struct ftp_dev { void *engine; }; // opaque wrapper (holds a retained FTPEngine)

static inline FTPEngine *E(ftp_dev *d) { return (__bridge FTPEngine *)d->engine; }

const char *ftp_strerror(int err)
{
    switch (err) {
    case 0: return "success";
    case -1: return "I/O error";
    case -3: return "access denied (run as root)";
    case -4: return "device not present";
    case -5: return "device not found";
    case -6: return "device busy";
    case -12: return "not supported";
    }
    static __thread char buf[48];
    snprintf(buf, sizeof buf, "IOReturn 0x%08x", (unsigned)err);
    return buf;
}

static int err_from(NSError *e)
{
    if (!e) return -1;
    switch ((IOReturn)e.code) {
    case kIOReturnNotPrivileged: return -3;
    case kIOReturnExclusiveAccess: return -6;
    case kIOReturnNoDevice: return -4;
    }
    return (int)e.code;
}

bool ftp_rate_supported(uint32_t rate)
{
    return rate == 44100 || rate == 48000 || rate == 88200 || rate == 96000;
}

static void report_xrun(FTPEngine *e, const char *what) { if (e->cb.xrun) e->cb.xrun(e->cb.ctx, what); }

static void mark_gone(FTPEngine *e)
{
    if (!atomic_exchange(&e->gone, true) && e->cb.gone) e->cb.gone(e->cb.ctx);
}

static bool fatal(IOReturn r) { return r == kIOReturnNoDevice || r == kIOReturnNotAttached || r == kIOReturnNotResponding || r == kIOReturnOffline; }

static void xfer_done(FTPEngine *e)
{
    pthread_mutex_lock(&e->lock);
    if (--e->inflight == 0) pthread_cond_broadcast(&e->cond);
    pthread_mutex_unlock(&e->lock);
}

/* ------------------------------------------------------------------ MIDI */

static void midi_submit(FTPEngine *e);

static void midi_submit(FTPEngine *e)
{
    if (atomic_load(&e->closing) || atomic_load(&e->gone) || !e->pMidiIn) return;
    NSError *err = nil;
    BOOL ok = [e->pMidiIn enqueueIORequestWithData:e->midiInData completionTimeout:0 error:&err
                                 completionHandler:^(IOReturn st, NSUInteger n) {
        if (st == kIOReturnAborted || atomic_load(&e->closing)) return;
        if (fatal(st)) { mark_gone(e); return; }
        if (st == kIOReturnSuccess && n >= 4 && e->cb.midi_in)
            e->cb.midi_in(e->cb.ctx, e->midiInData.bytes, n & ~(NSUInteger)3);
        midi_submit(e);
    }];
    if (!ok) LOG(e, "MIDI IN submit failed: %s", err.localizedDescription.UTF8String);
}

int ftp_midi_send(ftp_dev *d, const uint8_t *pkts, size_t len)
{
    if (!d) return -4;
    FTPEngine *e = E(d);
    if (atomic_load(&e->gone)) return -4;
    if (!len || !e->pMidiOut) return 0;
    NSData *copy = [NSData dataWithBytes:pkts length:len];
    dispatch_async(e->q, ^{
        NSError *err = nil;
        NSMutableData *buf = [e->ifMidi ioDataWithCapacity:copy.length error:&err];
        if (!buf) return;
        memcpy(buf.mutableBytes, copy.bytes, copy.length);
        [e->pMidiOut enqueueIORequestWithData:buf completionTimeout:1.0 error:&err
                            completionHandler:^(IOReturn st, NSUInteger n) { (void)buf; if (fatal(st)) mark_gone(e); }];
    });
    return 0;
}

/* ----------------------------------------------------------------- audio */

static void clock_tick(FTPEngine *e, uint64_t hw_time)
{
    double now = (double)hw_time;
    double period = e->cfg.pkts * e->ticks_per_ms;
    if (!e->dll_init) {
        ftp_dll_init(&e->dll, now, period, e->cfg.pkts / 1000.0, 2.0);
        e->dll_init = true;
    } else {
        double err = now - e->dll.t1;
        if (fabs(err) > period * (e->cfg.xfers - 1)) {
            report_xrun(e, "clock jump: USB schedule slipped");
            ftp_dll_init(&e->dll, now, period, e->cfg.pkts / 1000.0, 2.0);
        } else {
            ftp_dll_update(&e->dll, now);
            if (e->dll.ticks == (uint64_t)(2000 / e->cfg.pkts)) ftp_dll_set_bw(&e->dll, 0.5);
        }
    }
    if (e->cb.clock) {
        double frames_per_tick = (double)e->cfg.rate * e->cfg.pkts / 1000.0;
        e->cb.clock(e->cb.ctx, e->out_pos, (uint64_t)e->dll.t0, e->dll.e2 / frames_per_tick);
    }
}

static void render_slot(FTPEngine *e, out_slot *s)
{
    s->total = 0;
    for (int i = 0; i < e->cfg.pkts; i++) { s->sizes[i] = ftp_sched_next(&e->sched); s->total += s->sizes[i]; }
    if (e->cb.render) e->cb.render(e->cb.ctx, e->out_pos, s->staging, s->total);
    else memset(s->staging, 0, sizeof(float) * FTP_OUT_CHANNELS * s->total);
    e->out_pos += s->total;
    s->gen_rendered++;
}

// Frame scheduling: contiguous per stream; if we fell behind, skip ahead.
static uint64_t take_frame(FTPEngine *e, int stream)
{
    uint64_t f = e->next_frame[stream];
    e->next_frame[stream] += (uint64_t)e->cfg.pkts;
    return f;
}

static void resync_frames(FTPEngine *e)
{
    uint64_t now = [e->ifOut12 frameNumberWithTime:NULL] + 3;
    for (int i = 0; i < 3; i++) if (e->next_frame[i] < now) e->next_frame[i] = now;
}

static bool submit_out(FTPEngine *e, int si, int ep);
static bool submit_in(FTPEngine *e, int si);

static void out_done(FTPEngine *e, int si, int ep, IOReturn st, IOUSBHostIsochronousTransaction *tl)
{
    e->stats.out_completions++;
    e->stats.out_ep_completions[ep]++;
    e->stats.out_last_status = st;
    if (fatal(st)) mark_gone(e);
    if (atomic_load(&e->stopping) || atomic_load(&e->gone) || st == kIOReturnAborted) { xfer_done(e); return; }
    out_slot *s = &e->slots[si];
    for (int i = 0; i < e->cfg.pkts; i++)
        if (tl[i].status != kIOReturnSuccess) { e->stats.out_pkt_errors++; e->stats.out_last_pkt_status = tl[i].status; }

    // completion arrived too late to keep the queue full -> reschedule
    uint64_t hw = tl[e->cfg.pkts - 1].timeStamp;
    if ((double)(mach_absolute_time() - hw) > e->ticks_per_ms * e->cfg.pkts * (e->cfg.xfers - 1)) {
        report_xrun(e, "late completion");
        resync_frames(e);
    }
    if (s->gen_rendered == s->gen_sent[ep]) {
        render_slot(e, s);
        clock_tick(e, hw);
    }
    if (!submit_out(e, si, ep)) xfer_done(e);
}

static bool submit_out(FTPEngine *e, int si, int ep)
{
    out_slot *s = &e->slots[si];
    iso_xfer *x = &s->x[ep];
    NSMutableData *buf = e->outData[si][ep];
    ftp_pack_s24be(buf.mutableBytes, s->staging, s->total, FTP_OUT_CHANNELS, (unsigned)(2 * ep));
    uint32_t off = 0;
    for (int i = 0; i < e->cfg.pkts; i++) {
        x->tx[i] = (IOUSBHostIsochronousTransaction){ .requestCount = s->sizes[i] * FTP_BYTES_PER_FRAME, .offset = off };
        off += s->sizes[i] * FTP_BYTES_PER_FRAME;
    }
    s->gen_sent[ep] = s->gen_rendered;
    x->frame = take_frame(e, ep);
    NSError *err = nil;
    BOOL ok = [e->pOut[ep] enqueueIORequestWithData:buf transactionList:x->tx transactionListCount:(NSUInteger)e->cfg.pkts
                                   firstFrameNumber:x->frame options:IOUSBHostIsochronousTransferOptionsNone error:&err
                                  completionHandler:^(IOReturn st, IOUSBHostIsochronousTransaction tl[]) { out_done(e, si, ep, st, tl); }];
    if (!ok) {
        e->stats.submit_errors++;
        e->stats.last_submit_error = (int)err.code;
        if (err.code == kIOReturnIsoTooOld || err.code == kIOReturnIsoTooNew) {
            resync_frames(e);
            x->frame = take_frame(e, ep);
            ok = [e->pOut[ep] enqueueIORequestWithData:buf transactionList:x->tx transactionListCount:(NSUInteger)e->cfg.pkts
                                      firstFrameNumber:x->frame options:IOUSBHostIsochronousTransferOptionsNone error:&err
                                     completionHandler:^(IOReturn st, IOUSBHostIsochronousTransaction tl[]) { out_done(e, si, ep, st, tl); }];
        }
        if (!ok) { report_xrun(e, "OUT submit failed"); LOG(e, "OUT submit: %s", err.localizedDescription.UTF8String); }
    }
    return ok;
}

static void in_done(FTPEngine *e, int si, IOReturn st, IOUSBHostIsochronousTransaction *tl)
{
    e->stats.in_completions++;
    e->stats.in_last_status = st;
    if (fatal(st)) mark_gone(e);
    if (atomic_load(&e->stopping) || atomic_load(&e->gone) || st == kIOReturnAborted) { xfer_done(e); return; }
    const uint8_t *base = e->inData[si].bytes;
    uint32_t frames = 0;
    for (int i = 0; i < e->cfg.pkts; i++) {
        if (tl[i].status != kIOReturnSuccess && tl[i].status != kIOReturnUnderrun) {
            e->stats.in_pkt_errors++;
            e->stats.in_last_pkt_status = tl[i].status;
            continue;
        }
        uint32_t n = tl[i].completeCount / FTP_BYTES_PER_FRAME;
        e->stats.in_bytes += tl[i].completeCount;
        if (tl[i].completeCount) e->stats.in_last_len = tl[i].completeCount;
        ftp_unpack_s24be(e->in_tmp + (size_t)frames * FTP_IN_CHANNELS, base + tl[i].offset, n, FTP_IN_CHANNELS, 0);
        frames += n;
    }
    if (frames && e->cb.capture) e->cb.capture(e->cb.ctx, e->in_pos, e->in_tmp, frames);
    e->in_pos += frames;
    if (!submit_in(e, si)) xfer_done(e);
}

static bool submit_in(FTPEngine *e, int si)
{
    iso_xfer *x = &e->in_x[si];
    for (int i = 0; i < e->cfg.pkts; i++)
        x->tx[i] = (IOUSBHostIsochronousTransaction){ .requestCount = (uint32_t)e->max_packet, .offset = (uint32_t)(i * e->max_packet) };
    x->frame = take_frame(e, 2);
    NSError *err = nil;
    BOOL ok = [e->pIn enqueueIORequestWithData:e->inData[si] transactionList:x->tx transactionListCount:(NSUInteger)e->cfg.pkts
                              firstFrameNumber:x->frame options:IOUSBHostIsochronousTransferOptionsNone error:&err
                             completionHandler:^(IOReturn st, IOUSBHostIsochronousTransaction tl[]) { in_done(e, si, st, tl); }];
    if (!ok) {
        e->stats.submit_errors++;
        e->stats.last_submit_error = (int)err.code;
        if (err.code == kIOReturnIsoTooOld || err.code == kIOReturnIsoTooNew) {
            resync_frames(e);
            x->frame = take_frame(e, 2);
            ok = [e->pIn enqueueIORequestWithData:e->inData[si] transactionList:x->tx transactionListCount:(NSUInteger)e->cfg.pkts
                                 firstFrameNumber:x->frame options:IOUSBHostIsochronousTransferOptionsNone error:&err
                                completionHandler:^(IOReturn st, IOUSBHostIsochronousTransaction tl[]) { in_done(e, si, st, tl); }];
        }
        if (!ok) { report_xrun(e, "IN submit failed"); LOG(e, "IN submit: %s", err.localizedDescription.UTF8String); }
    }
    return ok;
}

/* ------------------------------------------------------------- control */

static int set_rate(FTPEngine *e, uint8_t ep, uint32_t rate)
{
    // UAC1 SET_CUR SAMPLING_FREQ_CONTROL on the endpoint (Linux: set_sample_rate_v1)
    NSMutableData *d = [NSMutableData dataWithLength:3];
    uint8_t *b = d.mutableBytes;
    b[0] = (uint8_t)rate; b[1] = (uint8_t)(rate >> 8); b[2] = (uint8_t)(rate >> 16);
    IOUSBDeviceRequest req = { .bmRequestType = 0x22, .bRequest = 0x01, .wValue = 0x0100, .wIndex = ep, .wLength = 3 };
    NSError *err = nil;
    if (![e->dev sendDeviceRequest:req data:d bytesTransferred:NULL completionTimeout:1.0 error:&err]) {
        LOG(e, "SET_CUR rate on EP 0x%02x: %s", ep, err.localizedDescription.UTF8String);
        return err_from(err);
    }
    NSMutableData *rb = [NSMutableData dataWithLength:3];
    IOUSBDeviceRequest get = { .bmRequestType = 0xA2, .bRequest = 0x81, .wValue = 0x0100, .wIndex = ep, .wLength = 3 };
    NSUInteger n = 0;
    if ([e->dev sendDeviceRequest:get data:rb bytesTransferred:&n completionTimeout:1.0 error:nil] && n == 3) {
        const uint8_t *r = rb.bytes;
        uint32_t got = r[0] | (r[1] << 8) | ((uint32_t)r[2] << 16);
        if (got != rate) LOG(e, "EP 0x%02x: requested %u Hz, device reports %u Hz", ep, rate, got);
    }
    return 0;
}

static io_service_t find_device(void)
{
    CFMutableDictionaryRef m = [IOUSBHostDevice createMatchingDictionaryWithVendorID:@(FTP_VID) productID:@(FTP_PID)
                                                                           bcdDevice:nil deviceClass:nil deviceSubclass:nil
                                                                      deviceProtocol:nil speed:nil productIDArray:nil];
    return IOServiceGetMatchingService(kIOMainPortDefault, m); // consumes m
}

static io_service_t find_interface_child(io_service_t device, int number)
{
    io_iterator_t it;
    io_service_t found = IO_OBJECT_NULL;
    if (IORegistryEntryGetChildIterator(device, kIOServicePlane, &it) != KERN_SUCCESS) return IO_OBJECT_NULL;
    io_object_t o;
    while ((o = IOIteratorNext(it))) {
        if (!found && IOObjectConformsTo(o, "IOUSBHostInterface")) {
            CFTypeRef n = IORegistryEntryCreateCFProperty(o, CFSTR("bInterfaceNumber"), NULL, 0);
            int v = -1;
            if (n) { CFNumberGetValue(n, kCFNumberIntType, &v); CFRelease(n); }
            if (v == number) { found = o; continue; }
        }
        IOObjectRelease(o);
    }
    IOObjectRelease(it);
    return found;
}

static IOUSBHostInterface *open_interface(FTPEngine *e, io_service_t devsvc, int number)
{
    for (int attempt = 0; attempt < 40; attempt++) {
        io_service_t s = find_interface_child(devsvc, number);
        if (s) {
            NSError *err = nil;
            IOUSBHostInterface *i = [[IOUSBHostInterface alloc] initWithIOService:s options:IOUSBHostObjectInitOptionsNone
                                                                            queue:e->q error:&err interestHandler:nil];
            if (!i) i = [[IOUSBHostInterface alloc] initWithIOService:s options:IOUSBHostObjectInitOptionsDeviceSeize
                                                                queue:e->q error:&err interestHandler:nil];
            IOObjectRelease(s);
            if (i) return i;
            if (attempt == 39) LOG(e, "open interface %d: %s", number, err.localizedDescription.UTF8String);
        }
        usleep(50000);
    }
    return nil;
}

int ftp_open(ftp_dev **out, const ftp_callbacks *cb, int verbose)
{
    @autoreleasepool {
        FTPEngine *e = [FTPEngine new];
        e->verbose = verbose;
        if (cb) e->cb = *cb;
        pthread_mutex_init(&e->lock, NULL);
        pthread_cond_init(&e->cond, NULL);
        mach_timebase_info_data_t tb;
        mach_timebase_info(&tb);
        e->ticks_per_ms = 1e6 * (double)tb.denom / (double)tb.numer;
        dispatch_queue_attr_t attr = dispatch_queue_attr_make_with_qos_class(DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INTERACTIVE, 0);
        e->q = dispatch_queue_create("be.jsix.fasttrack.usb", attr);

        io_service_t svc = find_device();
        if (!svc) return -5;
        NSError *err = nil;
        // Capture terminates AppleUSBAudio and every other client (root only).
        e->dev = [[IOUSBHostDevice alloc] initWithIOService:svc options:IOUSBHostObjectInitOptionsDeviceCapture
                                                      queue:e->q error:&err
                                            interestHandler:^(IOUSBHostObject *o, uint32_t type, void *arg) {
            if (type == kIOMessageServiceIsTerminated) mark_gone(e);
        }];
        if (!e->dev) {
            IOObjectRelease(svc);
            LOG(e, "capture device: %s", err.localizedDescription.UTF8String);
            return err_from(err);
        }
        const IOUSBConfigurationDescriptor *cd = e->dev.configurationDescriptor;
        LOG(e, "switching from configuration %d to %d", cd ? cd->bConfigurationValue : 0, FTP_CONFIG_FULL);
        if (![e->dev configureWithValue:FTP_CONFIG_FULL matchInterfaces:NO error:&err]) {
            LOG(e, "set configuration: %s", err.localizedDescription.UTF8String);
            [e->dev destroy];
            IOObjectRelease(svc);
            return err_from(err);
        }
        e->ifOut12 = open_interface(e, svc, FTP_IF_OUT12);
        if (!e->ifOut12) {
            // unregistered interfaces could not be opened: let them register (we hold the capture)
            LOG(e, "retrying configuration with interface matching");
            [e->dev configureWithValue:FTP_CONFIG_FULL matchInterfaces:YES error:&err];
        } else {
            [e->ifOut12 destroy];
            e->ifOut12 = nil;
        }
        e->ifMidi = open_interface(e, svc, FTP_IF_MIDI);
        e->ifOut12 = open_interface(e, svc, FTP_IF_OUT12);
        e->ifOut34 = open_interface(e, svc, FTP_IF_OUT34);
        e->ifInA = open_interface(e, svc, FTP_IF_IN_ANALOG);
        e->ifInD = open_interface(e, svc, FTP_IF_IN_DIGITAL);
        IOObjectRelease(svc);
        if (!e->ifOut12 || !e->ifOut34 || !e->ifInA) {
            LOG(e, "could not open the streaming interfaces");
            [e->ifMidi destroy]; [e->ifOut12 destroy]; [e->ifOut34 destroy]; [e->ifInA destroy]; [e->ifInD destroy];
            [e->dev destroy];
            return -6;
        }
        if (e->ifMidi) {
            e->pMidiIn = [e->ifMidi copyPipeWithAddress:FTP_EP_MIDI_IN error:&err];
            e->pMidiOut = [e->ifMidi copyPipeWithAddress:FTP_EP_MIDI_OUT error:&err];
            e->midiInData = [e->ifMidi ioDataWithCapacity:64 error:&err];
            if (e->pMidiIn && e->midiInData) dispatch_async(e->q, ^{ midi_submit(e); });
        }
        ftp_dev *d = calloc(1, sizeof *d);
        d->engine = (__bridge_retained void *)e;
        *out = d;
        return 0;
    }
}

bool ftp_running(const ftp_dev *d) { return d && E((ftp_dev *)d)->running; }
void ftp_get_stats(const ftp_dev *d, ftp_stats *out) { *out = E((ftp_dev *)d)->stats; }

uint32_t ftp_output_queue_frames(const ftp_dev *d)
{
    FTPEngine *e = E((ftp_dev *)d);
    return (uint32_t)((uint64_t)e->cfg.rate * (uint64_t)(e->cfg.xfers * e->cfg.pkts) / 1000);
}

static void free_streams(FTPEngine *e)
{
    for (int i = 0; i < MAX_XFERS; i++) {
        free(e->slots[i].staging);
        e->slots[i].staging = NULL;
        e->outData[i][0] = e->outData[i][1] = nil;
        e->inData[i] = nil;
    }
    e->pOut[0] = e->pOut[1] = nil;
    e->pIn = nil;
    free(e->in_tmp);
    e->in_tmp = NULL;
}

int ftp_start(ftp_dev *d, const ftp_stream_config *cfg)
{
    FTPEngine *e = E(d);
    if (e->running) ftp_stop(d);
    if (atomic_load(&e->gone)) return -4;
    if (!ftp_rate_supported(cfg->rate)) return -2;

    @autoreleasepool {
        e->cfg = *cfg;
        if (e->cfg.xfers <= 1 || e->cfg.xfers > MAX_XFERS) e->cfg.xfers = 4;
        if (e->cfg.pkts <= 0 || e->cfg.pkts > MAX_PKTS) e->cfg.pkts = 4;
        if (e->cfg.input_iface != FTP_IF_IN_DIGITAL || !e->ifInD) e->cfg.input_iface = FTP_IF_IN_ANALOG;
        const char *env;
        if ((env = getenv("FTP_XFERS")) && atoi(env) > 1 && atoi(env) <= MAX_XFERS) e->cfg.xfers = atoi(env);
        if ((env = getenv("FTP_PKTS")) && atoi(env) > 0 && atoi(env) <= MAX_PKTS) e->cfg.pkts = atoi(env);

        // Full-speed USB has ~1150 bytes/frame for periodic traffic: alt 3 (584 B)
        // only fits two streams, so 88.2/96 kHz is 2-in/2-out; 44.1/48 kHz uses
        // alt 2 (296 B) for all 4 outputs + input.
        bool high = e->cfg.rate > 48000;
        e->alt = high ? FTP_ALT_24BIT_96K : FTP_ALT_24BIT_48K;
        e->max_packet = high ? FTP_MAX_PACKET : FTP_MAX_PACKET_48K;
        e->n_out = (high || e->cfg.no_out34) ? 1 : 2;
        e->use_in = !e->cfg.no_input;
        IOUSBHostInterface *inIf = e->cfg.input_iface == FTP_IF_IN_DIGITAL ? e->ifInD : e->ifInA;
        IOUSBHostInterface *otherIn = inIf == e->ifInA ? e->ifInD : e->ifInA;
        uint8_t inEp = inIf == e->ifInA ? FTP_EP_IN_ANALOG : FTP_EP_IN_DIGITAL;
        memset(&e->stats, 0, sizeof e->stats);

        NSError *err = nil;
        if (otherIn) [otherIn selectAlternateSetting:0 error:nil];
        if (![e->ifOut12 selectAlternateSetting:(NSUInteger)e->alt error:&err]) { LOG(e, "IF2 alt %d: %s", e->alt, err.localizedDescription.UTF8String); return err_from(err); }
        if (![e->ifOut34 selectAlternateSetting:(NSUInteger)(e->n_out == 2 ? e->alt : 0) error:&err]) { LOG(e, "IF3: %s", err.localizedDescription.UTF8String); return err_from(err); }
        if (![inIf selectAlternateSetting:(NSUInteger)(e->use_in ? e->alt : 0) error:&err]) { LOG(e, "IN IF: %s", err.localizedDescription.UTF8String); return err_from(err); }
        int r;
        if ((r = set_rate(e, FTP_EP_OUT12, e->cfg.rate))) return r;
        if (e->n_out == 2 && (r = set_rate(e, FTP_EP_OUT34, e->cfg.rate))) return r;

        e->pOut[0] = [e->ifOut12 copyPipeWithAddress:FTP_EP_OUT12 error:&err];
        e->pOut[1] = e->n_out == 2 ? [e->ifOut34 copyPipeWithAddress:FTP_EP_OUT34 error:&err] : nil;
        e->pIn = e->use_in ? [inIf copyPipeWithAddress:inEp error:&err] : nil;
        if (!e->pOut[0] || (e->n_out == 2 && !e->pOut[1]) || (e->use_in && !e->pIn)) {
            LOG(e, "copy pipes: %s", err.localizedDescription.UTF8String);
            return err_from(err);
        }

        e->max_frames = ftp_sched_max(e->cfg.rate);
        ftp_sched_init(&e->sched, e->cfg.rate);
        e->out_pos = e->in_pos = 0;
        e->dll_init = false;
        atomic_store(&e->stopping, false);

        size_t out_len = (size_t)e->cfg.pkts * e->max_frames * FTP_BYTES_PER_FRAME;
        size_t in_len = (size_t)e->cfg.pkts * (size_t)e->max_packet;
        e->in_tmp = calloc((size_t)e->cfg.pkts * (FTP_MAX_PACKET / FTP_BYTES_PER_FRAME) * FTP_IN_CHANNELS, sizeof(float));
        for (int i = 0; i < e->cfg.xfers; i++) {
            out_slot *s = &e->slots[i];
            memset(s, 0, sizeof *s);
            s->staging = calloc((size_t)e->cfg.pkts * e->max_frames * FTP_OUT_CHANNELS, sizeof(float));
            for (int ep = 0; ep < e->n_out; ep++) {
                e->outData[i][ep] = [(ep ? e->ifOut34 : e->ifOut12) ioDataWithCapacity:out_len error:&err];
                if (!e->outData[i][ep]) { LOG(e, "ioData: %s", err.localizedDescription.UTF8String); free_streams(e); return err_from(err); }
            }
            if (e->use_in) {
                e->inData[i] = [inIf ioDataWithCapacity:in_len error:&err];
                if (!e->inData[i]) { LOG(e, "ioData: %s", err.localizedDescription.UTF8String); free_streams(e); return err_from(err); }
            }
        }

        // Prime: all streams start on the same frame, a few ms in the future.
        __block int submitted = 0;
        dispatch_sync(e->q, ^{
            uint64_t f0 = [e->ifOut12 frameNumberWithTime:NULL] + 4;
            for (int i = 0; i < 3; i++) e->next_frame[i] = f0;
            for (int i = 0; i < e->cfg.xfers; i++) {
                render_slot(e, &e->slots[i]);
                for (int ep = 0; ep < e->n_out; ep++) if (submit_out(e, i, ep)) submitted++;
                if (e->use_in && submit_in(e, i)) submitted++;
            }
            pthread_mutex_lock(&e->lock);
            e->inflight += submitted;
            pthread_mutex_unlock(&e->lock);
        });
        e->running = true;
        LOG(e, "streaming %u Hz alt %d: outputs %s, input %s, %d x %d ms transfers (IOUSBHost)", e->cfg.rate, e->alt,
            e->n_out == 2 ? "1-4" : "1/2", e->use_in ? (inIf == e->ifInA ? "analog" : "digital") : "off", e->cfg.xfers, e->cfg.pkts);
        return 0;
    }
}

void ftp_stop(ftp_dev *d)
{
    FTPEngine *e = E(d);
    if (!e->running) return;
    atomic_store(&e->stopping, true);
    @autoreleasepool {
        for (int ep = 0; ep < 2; ep++) if (e->pOut[ep]) [e->pOut[ep] abortWithOption:IOUSBHostAbortOptionAsynchronous error:nil];
        if (e->pIn) [e->pIn abortWithOption:IOUSBHostAbortOptionAsynchronous error:nil];
    }
    pthread_mutex_lock(&e->lock);
    while (e->inflight > 0) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 2;
        if (pthread_cond_timedwait(&e->cond, &e->lock, &ts) != 0) { LOG(e, "stop: %d transfers stuck", e->inflight); break; }
    }
    bool clean = e->inflight == 0;
    e->inflight = 0;
    pthread_mutex_unlock(&e->lock);
    if (!atomic_load(&e->gone)) {
        [e->ifOut12 selectAlternateSetting:0 error:nil];
        [e->ifOut34 selectAlternateSetting:0 error:nil];
        [(e->cfg.input_iface == FTP_IF_IN_DIGITAL ? e->ifInD : e->ifInA) selectAlternateSetting:0 error:nil];
    }
    if (clean) free_streams(e); // otherwise keep buffers alive for the kernel
    e->running = false;
}

void ftp_close(ftp_dev *d)
{
    if (!d) return;
    FTPEngine *e = (__bridge_transfer FTPEngine *)d->engine;
    ftp_dev tmp = { (__bridge void *)e };
    ftp_stop(&tmp);
    atomic_store(&e->closing, true);
    @autoreleasepool {
        if (e->pMidiIn) [e->pMidiIn abortWithOption:IOUSBHostAbortOptionSynchronous error:nil];
        [e->ifMidi destroy]; [e->ifOut12 destroy]; [e->ifOut34 destroy]; [e->ifInA destroy]; [e->ifInD destroy];
        // destroy resets the device and re-registers drivers: AppleUSBAudio takes over again
        [e->dev destroy];
    }
    pthread_mutex_destroy(&e->lock);
    pthread_cond_destroy(&e->cond);
    free(d);
}
