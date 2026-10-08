#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "flight.h"

typedef struct {
    uint32_t magic;
    int32_t  p0;
    int32_t  have_p0;
    double   last_agl;
    double   max_agl;
    uint32_t t_descent;
    uint32_t last_tx_ms;
    uint32_t seq;
    int32_t  phase;
    int32_t  servo_us;
    int32_t  cnt;
    int32_t  rej_run;
    int32_t  bad_run;
    int32_t  started;
} FlightPriv;

typedef char flight_priv_size_check[(sizeof(FlightPriv) <= 256) ? 1 : -1];

void flight_init(FlightState *st)
{
    FlightPriv *s = (FlightPriv *)st;
    memset(st, 0, 256);
    s->phase = 0;
    s->servo_us = 1000;
    s->last_agl = 0.0;
    s->max_agl = 0.0;
}

static void reset_counters(FlightPriv *s)
{
    s->cnt = 0;
}

void flight_step(FlightState *st, const Sample *in, Outputs *out)
{
    FlightPriv *s = (FlightPriv *)st;
    uint32_t t = (uint32_t)in->t_ms;
    int valid = (in->press_valid != 0) &&
                (in->press_pa >= 30000) && (in->press_pa <= 110000);
    int accepted = 0;
    double agl = 0.0;

    if (valid) {
        if (!s->have_p0) {
            s->p0 = (int32_t)in->press_pa;
            s->have_p0 = 1;
        }
        agl = 44330.0 * (1.0 - pow((double)in->press_pa / (double)s->p0, 0.19029));
        if (fabs(agl - s->last_agl) <= 50.0) {
            accepted = 1;
        } else {
            s->rej_run++;
            if (s->rej_run >= 100) {
                accepted = 1;
            }
        }
    }

    if (accepted) {
        s->rej_run = 0;
        s->bad_run = 0;
        s->last_agl = agl;

        if (s->phase == 0) {
            int32_t d = (int32_t)in->press_pa - s->p0;
            if (d >= -30 && d <= 30) {
                s->p0 += d / 64;
            }
            if (agl > 30.0) s->cnt++; else s->cnt = 0;
            if (s->cnt >= 20) {
                s->phase = 1;
                s->max_agl = -1.0e9;
                reset_counters(s);
            }
        } else if (s->phase == 1) {
            if (agl > s->max_agl) s->max_agl = agl;
            if (s->max_agl >= 500.0 && agl <= s->max_agl - 100.0) s->cnt++;
            else s->cnt = 0;
            if (s->cnt >= 30) {
                s->phase = 2;
                s->t_descent = t;
                reset_counters(s);
            }
        } else if (s->phase == 2) {
            if (agl <= 100.0) s->cnt++; else s->cnt = 0;
            if (s->cnt >= 25) {
                s->phase = 3;
                s->servo_us = 2000;
                reset_counters(s);
            }
        } else if (s->phase == 3) {
            int64_t ax = (int64_t)in->ax, ay = (int64_t)in->ay, az = (int64_t)in->az;
            int64_t m2 = ax * ax + ay * ay + az * az;
            int ok = (agl <= 15.0) && (m2 >= 800LL * 800LL) && (m2 <= 1200LL * 1200LL);
            if (ok) s->cnt++; else s->cnt = 0;
            if (s->cnt >= 100) {
                s->phase = 4;
                reset_counters(s);
            }
        }
    } else {
        s->bad_run++;
        if (s->bad_run == 30) {
            reset_counters(s);
        }
    }

    out->phase = (uint8_t)s->phase;
    out->servo_us = (uint16_t)s->servo_us;
    out->buzzer = (s->phase == 4) ? 1 : 0;
    out->tele_ready = 0;
    out->tele_len = 0;

    if (!s->started || (uint32_t)(t - s->last_tx_ms) >= 1000u) {
        uint32_t trel = 0;
        long aglm;
        int n;
        s->started = 1;
        s->last_tx_ms = t;
        if (s->phase >= 2) trel = (uint32_t)(t - s->t_descent);
        aglm = (long)floor(s->last_agl + 0.5);
        n = snprintf((char *)out->tele_buf, 80,
                     "SHD,%lu,%lu.%03lu,%ld,%ld,%ld,%ld,%ld,%d\r",
                     (unsigned long)s->seq,
                     (unsigned long)(trel / 1000u),
                     (unsigned long)(trel % 1000u),
                     aglm,
                     (long)in->press_pa,
                     (long)in->ax, (long)in->ay, (long)in->az,
                     (int)s->phase);
        if (n < 0) n = 0;
        if (n > 79) n = 79;
        out->tele_len = (uint8_t)n;
        out->tele_ready = 1;
        s->seq++;
    }
}
