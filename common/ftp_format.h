// Sample-format conversion and packet scheduling for the M-Audio FastTrack Pro.
//
// In USB configuration 2 the 24-bit alternate settings (2, 3, 5, 6) carry
// 3-byte packed *big-endian* samples (Linux: snd_usb_fasttrackpro quirk ->
// SNDRV_PCM_FORMAT_S24_3BE). Every stream is 2 channels.
#pragma once

#include <stdint.h>
#include <stddef.h>

#define FTP_BYTES_PER_SAMPLE 3
#define FTP_STREAM_CHANNELS  2
#define FTP_BYTES_PER_FRAME  (FTP_BYTES_PER_SAMPLE * FTP_STREAM_CHANNELS)

// Pack `frames` frames of 2 channels taken from an interleaved float buffer
// with `src_channels` channels, starting at channel `first_channel`.
// Samples are clipped to [-1, 1).
void ftp_pack_s24be(uint8_t *dst, const float *src, size_t frames,
                    unsigned src_channels, unsigned first_channel);

// Unpack `frames` frames of 2-channel S24_3BE into an interleaved float
// buffer with `dst_channels` channels, starting at channel `first_channel`.
void ftp_unpack_s24be(float *dst, const uint8_t *src, size_t frames,
                      unsigned dst_channels, unsigned first_channel);

// Little-endian variants (debug / firmware differences).
void ftp_pack_s24le(uint8_t *dst, const float *src, size_t frames,
                    unsigned src_channels, unsigned first_channel);
void ftp_unpack_s24le(float *dst, const uint8_t *src, size_t frames,
                      unsigned dst_channels, unsigned first_channel);

// Frames-per-USB-frame scheduler for adaptive OUT endpoints (1 ms frames).
// Distributes `rate` frames per second over 1000 packets per second exactly:
// 44100 Hz -> nine packets of 44 and one of 45 frames per 10 ms.
typedef struct {
    uint32_t rate;
    uint32_t acc;
} ftp_sched;

void     ftp_sched_init(ftp_sched *s, uint32_t rate);
uint32_t ftp_sched_next(ftp_sched *s);
// Max frames any packet can carry at `rate` (ceil(rate / 1000)).
uint32_t ftp_sched_max(uint32_t rate);

// Plain sample arrays: 24-bit packed little-endian <-> float (lossless).
void ftp_i24le_to_f32(float *dst, const uint8_t *src, size_t samples);
void ftp_f32_to_i24le(uint8_t *dst, const float *src, size_t samples);
