#include "ftp_format.h"

static inline int32_t to_s24(float f)
{
    if (!(f == f)) return 0; // NaN
    double v = (double)f * 8388608.0;
    if (v >= 8388607.0) return 8388607;
    if (v <= -8388608.0) return -8388608;
    return (int32_t)(v < 0 ? v - 0.5 : v + 0.5);
}

void ftp_pack_s24be(uint8_t *dst, const float *src, size_t frames,
                    unsigned src_channels, unsigned first_channel)
{
    for (size_t i = 0; i < frames; i++) {
        const float *f = src + i * src_channels + first_channel;
        for (unsigned c = 0; c < FTP_STREAM_CHANNELS; c++) {
            int32_t s = to_s24(f[c]);
            *dst++ = (uint8_t)(s >> 16);
            *dst++ = (uint8_t)(s >> 8);
            *dst++ = (uint8_t)s;
        }
    }
}

void ftp_unpack_s24be(float *dst, const uint8_t *src, size_t frames,
                      unsigned dst_channels, unsigned first_channel)
{
    for (size_t i = 0; i < frames; i++) {
        float *f = dst + i * dst_channels + first_channel;
        for (unsigned c = 0; c < FTP_STREAM_CHANNELS; c++) {
            int32_t s = ((int32_t)src[0] << 24) | ((int32_t)src[1] << 16) | ((int32_t)src[2] << 8);
            src += 3;
            f[c] = (float)((double)(s >> 8) / 8388608.0);
        }
    }
}

void ftp_pack_s24le(uint8_t *dst, const float *src, size_t frames,
                    unsigned src_channels, unsigned first_channel)
{
    for (size_t i = 0; i < frames; i++) {
        const float *f = src + i * src_channels + first_channel;
        for (unsigned c = 0; c < FTP_STREAM_CHANNELS; c++) {
            int32_t s = to_s24(f[c]);
            *dst++ = (uint8_t)s;
            *dst++ = (uint8_t)(s >> 8);
            *dst++ = (uint8_t)(s >> 16);
        }
    }
}

void ftp_unpack_s24le(float *dst, const uint8_t *src, size_t frames,
                      unsigned dst_channels, unsigned first_channel)
{
    for (size_t i = 0; i < frames; i++) {
        float *f = dst + i * dst_channels + first_channel;
        for (unsigned c = 0; c < FTP_STREAM_CHANNELS; c++) {
            int32_t s = ((int32_t)src[2] << 24) | ((int32_t)src[1] << 16) | ((int32_t)src[0] << 8);
            src += 3;
            f[c] = (float)((double)(s >> 8) / 8388608.0);
        }
    }
}

void ftp_sched_init(ftp_sched *s, uint32_t rate)
{
    s->rate = rate;
    s->acc = 0;
}

uint32_t ftp_sched_next(ftp_sched *s)
{
    s->acc += s->rate;
    uint32_t n = s->acc / 1000;
    s->acc -= n * 1000;
    return n;
}

uint32_t ftp_sched_max(uint32_t rate)
{
    return (rate + 999) / 1000;
}

void ftp_i24le_to_f32(float *dst, const uint8_t *src, size_t samples)
{
    for (size_t i = 0; i < samples; i++, src += 3) {
        int32_t s = ((int32_t)src[2] << 24) | ((int32_t)src[1] << 16) | ((int32_t)src[0] << 8);
        dst[i] = (float)((double)(s >> 8) / 8388608.0);
    }
}

void ftp_f32_to_i24le(uint8_t *dst, const float *src, size_t samples)
{
    for (size_t i = 0; i < samples; i++) {
        int32_t s = to_s24(src[i]);
        *dst++ = (uint8_t)s;
        *dst++ = (uint8_t)(s >> 8);
        *dst++ = (uint8_t)(s >> 16);
    }
}
