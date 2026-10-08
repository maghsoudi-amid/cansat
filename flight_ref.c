#include "flight.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    int32_t p0;
    uint32_t t_desc, last_tx, seq;
    double last_agl, max_agl;
    uint16_t bad, rej, launch_cnt, apo_cnt, sep_cnt, land_cnt, servo_us;
    uint8_t phase, p0_init, tx_init;
} Priv;
_Static_assert(sizeof(Priv) <= sizeof(FlightState), "state too small");

void flight_init(FlightState *st) {
    Priv *p = (Priv *)st;
    memset(st, 0, sizeof *st);
    p->phase = PH_PAD;
    p->servo_us = 1000;
}

static void reset_counters(Priv *p) { p->launch_cnt = p->apo_cnt = p->sep_cnt = p->land_cnt = 0; }

void flight_step(const Sample *in, FlightState *st, Outputs *out) {
    Priv *p = (Priv *)st;
    int valid = in->press_valid && in->press_pa >= 30000 && in->press_pa <= 110000;
    int accepted = 0;
    double agl = 0.0;

    if (valid) {
        if (!p->p0_init) { p->p0 = in->press_pa; p->p0_init = 1; }
        agl = 44330.0 * (1.0 - pow((double)in->press_pa / (double)p->p0, 0.19029));
        if (fabs(agl - p->last_agl) <= 50.0 || p->rej >= 100) accepted = 1;
        else if (p->rej < 65535) p->rej++;
    }

    if (accepted) {
        p->bad = 0; p->rej = 0; p->last_agl = agl;
        switch (p->phase) {
        case PH_PAD: {
            int32_t d = in->press_pa - p->p0;
            if (d >= -30 && d <= 30) p->p0 += d / 64;
            p->launch_cnt = (agl > 30.0) ? p->launch_cnt + 1 : 0;
            if (p->launch_cnt >= 20) { p->phase = PH_ASCENT; p->max_agl = agl; p->apo_cnt = 0; }
            break; }
        case PH_ASCENT:
            if (agl > p->max_agl) p->max_agl = agl;
            if (p->max_agl >= 500.0) {
                p->apo_cnt = (agl <= p->max_agl - 100.0) ? p->apo_cnt + 1 : 0;
                if (p->apo_cnt >= 30) { p->phase = PH_DESCENT; p->t_desc = in->t_ms; p->sep_cnt = 0; }
            }
            break;
        case PH_DESCENT:
            p->sep_cnt = (agl <= 100.0) ? p->sep_cnt + 1 : 0;
            if (p->sep_cnt >= 25) { p->phase = PH_SEPARATED; p->servo_us = 2000; p->land_cnt = 0; }
            break;
        case PH_SEPARATED: {
            int64_t m2 = (int64_t)in->ax_mg * in->ax_mg + (int64_t)in->ay_mg * in->ay_mg + (int64_t)in->az_mg * in->az_mg;
            int still = (m2 >= 800LL * 800LL) && (m2 <= 1200LL * 1200LL);
            p->land_cnt = (agl <= 15.0 && still) ? p->land_cnt + 1 : 0;
            if (p->land_cnt >= 100) p->phase = PH_LANDED;
            break; }
        default: break;
        }
    } else {
        if (p->bad < 65535) p->bad++;
        if (p->bad == 30) reset_counters(p);
    }

    out->servo_us = p->servo_us;
    out->phase = p->phase;
    out->buzzer = (p->phase == PH_LANDED);
    out->tele_ready = 0; out->tele_len = 0;

    if (!p->tx_init || (uint32_t)(in->t_ms - p->last_tx) >= 1000u) {
        uint32_t trel = (p->phase >= PH_DESCENT) ? (uint32_t)(in->t_ms - p->t_desc) : 0u;
        int agl_i = (int)floor(p->last_agl + 0.5);
        int n = snprintf(out->tele, sizeof out->tele, "SHD,%lu,%lu.%03lu,%d,%ld,%d,%d,%d,%u\r",
                         (unsigned long)p->seq, (unsigned long)(trel / 1000u), (unsigned long)(trel % 1000u),
                         agl_i, (long)in->press_pa, in->ax_mg, in->ay_mg, in->az_mg, (unsigned)p->phase);
        if (n > 0 && n < (int)sizeof out->tele) { out->tele_len = (uint8_t)n; out->tele_ready = 1; }
        p->seq++; p->last_tx = in->t_ms; p->tx_init = 1;
    }
}
