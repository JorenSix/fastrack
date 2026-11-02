#include "ftp_clock.h"
#include <math.h>

void ftp_dll_set_bw(ftp_dll *d, double bw_hz)
{
    double omega = 2.0 * M_PI * bw_hz * d->tper_s;
    d->b = sqrt(2.0) * omega;
    d->c = omega * omega;
}

void ftp_dll_init(ftp_dll *d, double now, double period, double period_s, double bw_hz)
{
    d->tper_s = period_s;
    d->e2 = period;
    d->t0 = now;
    d->t1 = now + period;
    d->ticks = 0;
    ftp_dll_set_bw(d, bw_hz);
}

double ftp_dll_update(ftp_dll *d, double now)
{
    double e = now - d->t1;
    d->t0 = d->t1;
    d->t1 += d->b * e + d->e2;
    d->e2 += d->c * e;
    d->ticks++;
    return e;
}
