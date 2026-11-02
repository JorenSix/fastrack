#include "../common/ftp_format.h"
#include "../common/ftp_clock.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int failures;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static void test_pack(void)
{
    float src[4 * 3] = { 0.5f, -0.5f, 0.25f, -1.0f,  1.0f, 0.0f, -0.25f, 0.75f,  0.0f, 0.1f, 0.2f, 0.3f };
    uint8_t buf[3 * 6];
    ftp_pack_s24be(buf, src, 3, 4, 0);
    // 0.5 -> 0x400000 big-endian
    CHECK(buf[0] == 0x40 && buf[1] == 0x00 && buf[2] == 0x00);
    // -0.5 -> 0xC00000
    CHECK(buf[3] == 0xC0 && buf[4] == 0x00 && buf[5] == 0x00);
    // 1.0 clips to 0x7FFFFF
    CHECK(buf[6] == 0x7F && buf[7] == 0xFF && buf[8] == 0xFF);
    float out[2 * 3];
    ftp_unpack_s24be(out, buf, 3, 2, 0);
    CHECK(out[0] == 0.5f && out[1] == -0.5f && out[3] == 0.0f);
    CHECK(fabsf(out[5] - 0.1f) < 1e-6f);
    // channel offset (outputs 3/4)
    ftp_pack_s24be(buf, src, 3, 4, 2);
    CHECK(buf[0] == 0x20 && buf[3] == 0x80 && buf[4] == 0x00); // 0.25, -1.0
    // round trip of random values
    for (int i = 0; i < 10000; i++) {
        float v[2] = { (float)(rand() / (double)RAND_MAX * 2 - 1), (float)(rand() / (double)RAND_MAX * 2 - 1) }, w[2];
        uint8_t b[6];
        ftp_pack_s24be(b, v, 1, 2, 0);
        ftp_unpack_s24be(w, b, 1, 2, 0);
        CHECK(fabsf(v[0] - w[0]) < 1.5f / 8388608.0f && fabsf(v[1] - w[1]) < 1.5f / 8388608.0f);
    }
}

static void test_sched(void)
{
    const uint32_t rates[] = { 44100, 48000, 88200, 96000 };
    for (int r = 0; r < 4; r++) {
        ftp_sched s;
        ftp_sched_init(&s, rates[r]);
        uint64_t total = 0;
        uint32_t mx = 0, mn = ~0u;
        for (int i = 0; i < 1000; i++) {
            uint32_t n = ftp_sched_next(&s);
            total += n;
            if (n > mx) mx = n;
            if (n < mn) mn = n;
        }
        CHECK(total == rates[r]);
        CHECK(mx <= ftp_sched_max(rates[r]));
        CHECK(mx * 6 <= 584);
        CHECK(mx - mn <= 1);
    }
}

static void test_dll(void)
{
    ftp_dll d;
    double period = 48000.0; // 2 ms at 24 MHz
    ftp_dll_init(&d, 0, period * 1.001, 0.002, 2.0); // start 0.1% off
    srand(1);
    for (int i = 1; i < 5000; i++) {
        double jitter = (rand() / (double)RAND_MAX - 0.5) * 24000; // +-0.5 ms
        ftp_dll_update(&d, i * period + jitter);
    }
    CHECK(fabs(d.e2 - period) / period < 1e-3);
    CHECK(fabs(d.t0 - 4999 * period) < 6000);
}

int main(void)
{
    test_pack();
    test_sched();
    test_dll();
    if (failures) { printf("%d failure(s)\n", failures); return 1; }
    printf("all tests passed\n");
    return 0;
}
