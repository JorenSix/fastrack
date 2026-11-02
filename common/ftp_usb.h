// User-space USB engine for the M-Audio FastTrack Pro (0763:2012).
//
// Mirrors what Linux snd-usb-audio does for this device:
//   * switch to USB configuration 2 (snd_usb_fasttrackpro_boot_quirk)
//   * use 24-bit big-endian alt settings (alt 3: 44.1/48/88.2/96 kHz)
//   * treat playback as adaptive (ignore feedback EPs), set the rate with a
//     UAC1 SET_CUR SAMPLING_FREQ request on the OUT endpoints
// plus USB-MIDI on interface 1.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define FTP_VID 0x0763
#define FTP_PID 0x2012

#define FTP_CONFIG_FULL      2
#define FTP_IF_CONTROL       0
#define FTP_IF_MIDI          1
#define FTP_IF_OUT12         2
#define FTP_IF_OUT34         3
#define FTP_IF_IN_ANALOG     4
#define FTP_IF_IN_DIGITAL    5
#define FTP_ALT_24BIT_48K    2
#define FTP_ALT_24BIT_96K    3

#define FTP_EP_OUT12         0x03
#define FTP_EP_OUT34         0x04
#define FTP_EP_IN_ANALOG     0x85
#define FTP_EP_IN_DIGITAL    0x86
#define FTP_EP_MIDI_OUT      0x01
#define FTP_EP_MIDI_IN       0x81
#define FTP_MAX_PACKET       584   /* alt 3 */
#define FTP_MAX_PACKET_48K   296   /* alt 2 */

#define FTP_OUT_CHANNELS     4
#define FTP_IN_CHANNELS      2

typedef struct ftp_dev ftp_dev;

typedef struct {
    void *ctx;
    // Produce `frames` interleaved 4-channel float frames for device output
    // sample position `pos`. Called on the USB event thread.
    void (*render)(void *ctx, uint64_t pos, float *buf, uint32_t frames);
    // Captured interleaved 2-channel float frames, `pos` = input frame counter.
    void (*capture)(void *ctx, uint64_t pos, const float *buf, uint32_t frames);
    // Clock line: output sample position `sample` is consumed at host time
    // `host` (mach_absolute_time units), advancing `ticks_per_sample`.
    void (*clock)(void *ctx, uint64_t sample, uint64_t host, double ticks_per_sample);
    // Raw USB-MIDI event packets (multiples of 4 bytes).
    void (*midi_in)(void *ctx, const uint8_t *pkts, size_t len);
    // Device unplugged / fatal error. Called at most once.
    void (*gone)(void *ctx);
    // Glitch report (late completion, packet error, ...).
    void (*xrun)(void *ctx, const char *what);
} ftp_callbacks;

typedef struct {
    uint32_t rate;          // 44100, 48000, 88200, 96000
    int      input_iface;   // FTP_IF_IN_ANALOG or FTP_IF_IN_DIGITAL
    int      xfers;         // transfers in flight per endpoint (default 8)
    int      pkts;          // 1 ms packets per transfer (default 2)
    bool     little_endian; // debug: 24-bit LE instead of the Linux-documented BE
    bool     no_out34;      // debug: only stream outputs 1/2
    bool     no_input;      // debug: don't stream the input
} ftp_stream_config;

typedef struct {
    uint64_t out_completions, in_completions;
    uint64_t out_ep_completions[2];
    uint64_t submit_errors;
    int last_submit_error;
    uint64_t out_pkt_errors, in_pkt_errors;
    uint64_t in_bytes;
    int out_last_status, in_last_status;          // libusb_transfer_status
    int out_last_pkt_status, in_last_pkt_status;
    unsigned in_last_len;
} ftp_stats;

// Find, capture (detach AppleUSBAudio - requires root), switch to config 2,
// claim interfaces and start the event thread + MIDI input.
int  ftp_open(ftp_dev **out, const ftp_callbacks *cb, int verbose);
bool ftp_rate_supported(uint32_t rate);
int  ftp_start(ftp_dev *d, const ftp_stream_config *cfg);
void ftp_stop(ftp_dev *d);
bool ftp_running(const ftp_dev *d);
void ftp_get_stats(const ftp_dev *d, ftp_stats *out);
// Output latency in frames introduced by the transfer queue.
uint32_t ftp_output_queue_frames(const ftp_dev *d);
int  ftp_midi_send(ftp_dev *d, const uint8_t *pkts, size_t len);
// Stop everything, release interfaces and hand the device back to macOS.
void ftp_close(ftp_dev *d);
const char *ftp_strerror(int err);
