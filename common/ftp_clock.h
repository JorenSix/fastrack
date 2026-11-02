// Delay-locked loop (F. Adriaensen, "Using a DLL to filter time") used to
// turn jittery USB completion times into a clean (tick, host-time) line.
#pragma once

#include <stdint.h>

typedef struct {
    double t0;      // filtered time of the current tick (host ticks)
    double t1;      // predicted time of the next tick
    double e2;      // filtered tick period (host ticks)
    double b, c;    // loop coefficients
    double tper_s;  // nominal tick period in seconds
    uint64_t ticks; // number of updates since init
} ftp_dll;

// now/period in host ticks; period_s = same period in seconds; bw_hz = loop bandwidth.
void ftp_dll_init(ftp_dll *d, double now, double period, double period_s, double bw_hz);
// Feed the measured time of the next tick. Returns the error (now - predicted).
double ftp_dll_update(ftp_dll *d, double now);
void ftp_dll_set_bw(ftp_dll *d, double bw_hz);
