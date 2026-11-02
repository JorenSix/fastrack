// ftp-probe: protocol test tool for the M-Audio FastTrack Pro.
//
//   ftp-probe --dump                       print all USB configuration descriptors (no root needed)
//   sudo ftp-probe --tone 48000 [-s 5] [--record in.wav] [--digital] [--channels 1234]
//                                          capture the device, switch to config 2, play test tones
//                                          on outputs 1-4 and optionally record the input to WAV.
#include "../common/ftp_usb.h"
#include "../common/ftp_format.h"

#include <libusb.h>
#include <math.h>
#include <mach/mach_time.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ---------------------------------------------------------- descriptor dump */

static const char *as_subtype(uint8_t t)
{
    switch (t) { case 1: return "AS_GENERAL"; case 2: return "FORMAT_TYPE"; case 3: return "FORMAT_SPECIFIC"; }
    return "?";
}

static const char *ac_subtype(uint8_t t)
{
    switch (t) {
    case 1: return "HEADER"; case 2: return "INPUT_TERMINAL"; case 3: return "OUTPUT_TERMINAL";
    case 4: return "MIXER_UNIT"; case 5: return "SELECTOR_UNIT"; case 6: return "FEATURE_UNIT";
    case 7: return "PROCESSING_UNIT"; case 8: return "EXTENSION_UNIT";
    }
    return "?";
}

static void dump_extra(const unsigned char *p, int len, int iface_class, int iface_sub)
{
    while (len >= 2) {
        int l = p[0];
        if (l < 2 || l > len) break;
        if (p[1] == 0x24 && iface_class == 1 && iface_sub == 1) {
            printf("        CS_INTERFACE %s", ac_subtype(p[2]));
            if (p[2] == 2) printf(" id=%u type=0x%04x ch=%u", p[3], p[4] | p[5] << 8, p[7]);
            if (p[2] == 3) printf(" id=%u type=0x%04x src=%u", p[3], p[4] | p[5] << 8, p[7]);
            if (p[2] == 6) printf(" id=%u src=%u controlsize=%u", p[3], p[4], p[5]);
            printf("\n");
        } else if (p[1] == 0x24 && iface_class == 1 && iface_sub == 2) {
            printf("        CS_INTERFACE %s", as_subtype(p[2]));
            if (p[2] == 1) printf(" terminal=%u delay=%u format=0x%04x", p[3], p[4], p[5] | p[6] << 8);
            if (p[2] == 2 && p[3] == 1) {
                printf(" TYPE_I ch=%u subframe=%u bits=%u rates:", p[4], p[5], p[6]);
                int n = p[7];
                const unsigned char *r = p + 8;
                if (n == 0) printf(" %u-%u (continuous)", r[0] | r[1] << 8 | r[2] << 16, r[3] | r[4] << 8 | r[5] << 16);
                for (int i = 0; i < n; i++) printf(" %u", r[3 * i] | r[3 * i + 1] << 8 | r[3 * i + 2] << 16);
            }
            printf("\n");
        } else if (p[1] == 0x25) {
            printf("        CS_ENDPOINT attrs=0x%02x (sampling-freq control: %s)\n", p[3], p[3] & 1 ? "yes" : "no");
        } else {
            printf("        desc type=0x%02x len=%d\n", p[1], l);
        }
        p += l;
        len -= l;
    }
}

static int dump(void)
{
    libusb_context *ctx;
    if (libusb_init_context(&ctx, NULL, 0)) return 1;
    libusb_device **list;
    ssize_t n = libusb_get_device_list(ctx, &list);
    int found = 0;
    for (ssize_t i = 0; i < n; i++) {
        struct libusb_device_descriptor dd;
        libusb_get_device_descriptor(list[i], &dd);
        if (dd.idVendor != FTP_VID || dd.idProduct != FTP_PID) continue;
        found = 1;
        printf("FastTrack Pro: bus %u addr %u bcdDevice %04x, %u configurations\n",
               libusb_get_bus_number(list[i]), libusb_get_device_address(list[i]), dd.bcdDevice, dd.bNumConfigurations);
        for (int c = 0; c < dd.bNumConfigurations; c++) {
            struct libusb_config_descriptor *cd;
            if (libusb_get_config_descriptor(list[i], (uint8_t)c, &cd)) continue;
            printf("Configuration %u: %u interfaces, wTotalLength %u\n", cd->bConfigurationValue, cd->bNumInterfaces, cd->wTotalLength);
            for (int f = 0; f < cd->bNumInterfaces; f++) {
                const struct libusb_interface *itf = &cd->interface[f];
                for (int a = 0; a < itf->num_altsetting; a++) {
                    const struct libusb_interface_descriptor *id = &itf->altsetting[a];
                    printf("  IF%u alt%u class %u/%u, %u endpoints\n", id->bInterfaceNumber, id->bAlternateSetting,
                           id->bInterfaceClass, id->bInterfaceSubClass, id->bNumEndpoints);
                    dump_extra(id->extra, id->extra_length, id->bInterfaceClass, id->bInterfaceSubClass);
                    for (int e = 0; e < id->bNumEndpoints; e++) {
                        const struct libusb_endpoint_descriptor *ep = &id->endpoint[e];
                        static const char *types[] = { "control", "iso", "bulk", "interrupt" };
                        static const char *sync[] = { "none", "async", "adaptive", "sync" };
                        printf("      EP 0x%02x %s %s maxpkt %u interval %u synch 0x%02x\n", ep->bEndpointAddress,
                               types[ep->bmAttributes & 3], sync[(ep->bmAttributes >> 2) & 3], ep->wMaxPacketSize,
                               ep->bInterval, ep->bSynchAddress);
                        dump_extra(ep->extra, ep->extra_length, id->bInterfaceClass, id->bInterfaceSubClass);
                    }
                }
            }
            libusb_free_config_descriptor(cd);
        }
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    if (!found) fprintf(stderr, "FastTrack Pro (0763:2012) not found\n");
    return found ? 0 : 1;
}

/* -------------------------------------------------------------- tone test */

typedef struct {
    uint32_t rate;
    unsigned channel_mask;
    FILE *wav;
    uint64_t wav_frames;
    atomic_uint_fast64_t xruns;
    double peak[2];
    uint64_t clock_sample, clock_host;
    double tps;
} probe;

static atomic_bool g_quit;
static double g_amp = 0.25;   // --amp
static bool   g_ramp;         // --ramp: 0.1, 0.2 ... 1.0 full scale, one step per second

static double amp_at(double t)
{
    if (!g_ramp) return g_amp;
    double a = 0.1 * (1.0 + floor(t));
    return a > 1.0 ? 1.0 : a;
}
static void on_sig(int s) { (void)s; atomic_store(&g_quit, true); }

static void render(void *ctx, uint64_t pos, float *buf, uint32_t frames)
{
    probe *p = ctx;
    static const double freq[4] = { 440.0, 554.37, 659.25, 880.0 };
    for (uint32_t i = 0; i < frames; i++) {
        double t = (double)(pos + i) / p->rate;
        for (int c = 0; c < 4; c++)
            buf[i * 4 + c] = (p->channel_mask & (1u << c)) ? (float)(amp_at(t) * sin(2 * M_PI * freq[c] * t)) : 0.0f;
    }
}

static void put_le(FILE *f, uint32_t v, int n) { for (int i = 0; i < n; i++) fputc((v >> (8 * i)) & 0xff, f); }

static void wav_header(FILE *f, uint32_t rate, uint64_t frames)
{
    uint32_t data = (uint32_t)(frames * 6);
    fseek(f, 0, SEEK_SET);
    fwrite("RIFF", 1, 4, f); put_le(f, 36 + data, 4); fwrite("WAVEfmt ", 1, 8, f);
    put_le(f, 16, 4); put_le(f, 1, 2); put_le(f, 2, 2); put_le(f, rate, 4); put_le(f, rate * 6, 4);
    put_le(f, 6, 2); put_le(f, 24, 2); fwrite("data", 1, 4, f); put_le(f, data, 4);
}

static void capture(void *ctx, uint64_t pos, const float *buf, uint32_t frames)
{
    (void)pos;
    probe *p = ctx;
    for (uint32_t i = 0; i < frames * 2; i++) {
        double a = fabs(buf[i]);
        if (a > p->peak[i & 1]) p->peak[i & 1] = a;
        if (p->wav) {
            int32_t s = (int32_t)lrint(fmax(-1.0, fmin(0.99999988, buf[i])) * 8388608.0);
            put_le(p->wav, (uint32_t)s, 3);
        }
    }
    p->wav_frames += frames;
}

static void on_clock(void *ctx, uint64_t sample, uint64_t host, double tps)
{
    probe *p = ctx;
    p->clock_sample = sample; p->clock_host = host; p->tps = tps;
}

static void on_xrun(void *ctx, const char *what)
{
    probe *p = ctx;
    atomic_fetch_add(&p->xruns, 1);
    if (atomic_load(&p->xruns) < 20) fprintf(stderr, "xrun: %s\n", what);
}

static void on_midi(void *ctx, const uint8_t *pkts, size_t len)
{
    (void)ctx;
    for (size_t i = 0; i + 3 < len; i += 4)
        if (pkts[i]) printf("MIDI in: %02x %02x %02x %02x\n", pkts[i], pkts[i + 1], pkts[i + 2], pkts[i + 3]);
}

static void on_gone(void *ctx) { (void)ctx; fprintf(stderr, "device gone\n"); atomic_store(&g_quit, true); }

static int tone(uint32_t rate, double seconds, const char *wav_path, int digital, unsigned mask, bool le, bool no34, bool noin)
{
    probe p = { .rate = rate, .channel_mask = mask };
    if (wav_path) {
        p.wav = fopen(wav_path, "wb");
        if (!p.wav) { perror(wav_path); return 1; }
        wav_header(p.wav, rate, 0);
    }
    ftp_callbacks cb = { .ctx = &p, .render = render, .capture = capture, .clock = on_clock,
                         .midi_in = on_midi, .gone = on_gone, .xrun = on_xrun };
    ftp_dev *d;
    int r = ftp_open(&d, &cb, 1);
    if (r) {
        fprintf(stderr, "open failed: %s%s\n", ftp_strerror(r), geteuid() ? " (run with sudo)" : "");
        return 1;
    }
    ftp_stream_config cfg = { .rate = rate, .input_iface = digital ? FTP_IF_IN_DIGITAL : FTP_IF_IN_ANALOG,
                              .little_endian = le, .no_out34 = no34, .no_input = noin };
    printf("byte order: 24-bit %s\n", le ? "little-endian" : "big-endian (Linux quirk)");
    r = ftp_start(d, &cfg);
    if (r) { fprintf(stderr, "start failed: %s\n", ftp_strerror(r)); ftp_close(d); return 1; }

    signal(SIGINT, on_sig);
    mach_timebase_info_data_t tb; mach_timebase_info(&tb);
    double tps_nominal = 1e9 * tb.denom / tb.numer / rate;
    uint64_t end = mach_absolute_time() + (uint64_t)(seconds * 1e9 * tb.denom / tb.numer);
    uint64_t first_sample = 0, first_host = 0;
    while (!atomic_load(&g_quit) && mach_absolute_time() < end) {
        sleep(1);
        if (!first_host && p.clock_host) { first_sample = p.clock_sample; first_host = p.clock_host; }
        double measured = 0;
        if (first_host && p.clock_host > first_host)
            measured = (double)(p.clock_sample - first_sample) / ((double)(p.clock_host - first_host) / (tps_nominal * rate));
        if (g_ramp || g_amp != 0.25)
            printf("out amplitude %.2f (%.1f dBFS)\n", amp_at((double)p.clock_sample / rate), 20 * log10(amp_at((double)p.clock_sample / rate)));
        printf("in peak L %.3f R %.3f | captured %llu frames | clock %.2f Hz (dll %.2f Hz) | xruns %llu\n",
               p.peak[0], p.peak[1], (unsigned long long)p.wav_frames, measured,
               p.tps > 0 ? tps_nominal * rate / p.tps : 0.0, (unsigned long long)atomic_load(&p.xruns));
        ftp_stats st;
        ftp_get_stats(d, &st);
        printf("  usb: EP3 %llu EP4 %llu | submit errors %llu (last %s)\n",
               (unsigned long long)st.out_ep_completions[0], (unsigned long long)st.out_ep_completions[1],
               (unsigned long long)st.submit_errors, st.submit_errors ? ftp_strerror(st.last_submit_error) : "-");
        printf("  usb: out xfers %llu (pkt err %llu, last st %d/%d) | in xfers %llu (pkt err %llu, last st %d/%d) bytes %llu last pkt %u\n",
               (unsigned long long)st.out_completions, (unsigned long long)st.out_pkt_errors, st.out_last_status, st.out_last_pkt_status,
               (unsigned long long)st.in_completions, (unsigned long long)st.in_pkt_errors, st.in_last_status, st.in_last_pkt_status,
               (unsigned long long)st.in_bytes, st.in_last_len);
        p.peak[0] = p.peak[1] = 0;
    }
    ftp_stop(d);
    ftp_close(d);
    if (p.wav) { wav_header(p.wav, rate, p.wav_frames); fclose(p.wav); printf("wrote %s\n", wav_path); }
    return 0;
}

int main(int argc, char **argv)
{
    uint32_t rate = 0;
    double seconds = 5;
    const char *wav = NULL;
    int digital = 0;
    unsigned mask = 0xf;
    bool le = false, no34 = false, noin = false;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--dump")) return dump();
        else if (!strcmp(argv[i], "--tone") && i + 1 < argc) rate = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "-s") && i + 1 < argc) seconds = atof(argv[++i]);
        else if (!strcmp(argv[i], "--record") && i + 1 < argc) wav = argv[++i];
        else if (!strcmp(argv[i], "--digital")) digital = 1;
        else if (!strcmp(argv[i], "--le")) le = true;
        else if (!strcmp(argv[i], "--no-out34")) no34 = true;
        else if (!strcmp(argv[i], "--no-input")) noin = true;
        else if (!strcmp(argv[i], "--ramp")) g_ramp = true;
        else if (!strcmp(argv[i], "--amp") && i + 1 < argc) g_amp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--channels") && i + 1 < argc) {
            mask = 0;
            for (const char *c = argv[++i]; *c; c++) if (*c >= '1' && *c <= '4') mask |= 1u << (*c - '1');
        } else { rate = 0; break; }
    }
    if (!rate || !ftp_rate_supported(rate)) {
        fprintf(stderr, "usage: %s --dump\n       sudo %s --tone 44100|48000|88200|96000 [-s secs] [--record in.wav] [--digital] [--le] [--no-out34] [--no-input] [--channels 1234] [--amp 0..1] [--ramp]\n", argv[0], argv[0]);
        return 2;
    }
    return tone(rate, seconds, wav, digital, mask, le, no34, noin);
}
