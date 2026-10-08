#include "flight.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    double p0_sum, p0, last_agl, max_agl;
    uint32_t last_tx, seq;
    uint16_t p0_n, since, c_launch, c_sep, c_land, servo_us;
    uint8_t state, tx_init;
} Priv;
_Static_assert(sizeof(Priv) <= sizeof(FlightState), "state too small");

void flight_init(FlightState *st) {
    Priv *p = (Priv *)st;
    memset(st, 0, sizeof *st);
    p->servo_us = 1000;
}

void flight_step(const Sample *in, FlightState *st, Outputs *out) {
    Priv *p = (Priv *)st;
    int valid = in->press_valid && in->press_pa >= 30000 && in->press_pa <= 110000;

    if (p->p0_n < 200) {
        if (valid) {
            p->p0_sum += in->press_pa;
            if (++p->p0_n == 200) p->p0 = p->p0_sum / 200.0;
        }
    } else {
        int acc = 0;
        double agl = 0.0;
        if (valid) {
            agl = 44330.0 * (1.0 - pow((double)in->press_pa / p->p0, 0.19029));
            if (fabs(agl - p->last_agl) <= 20.0 + 0.2 * p->since) acc = 1;
        }
        if (!acc) {
            if (p->since < 60000) p->since++;
        } else {
            p->since = 0;
            p->last_agl = agl;
            if (p->state == 0) {
                p->c_launch = (agl > 25.0) ? p->c_launch + 1 : 0;
                if (p->c_launch >= 30) { p->state = 1; p->max_agl = agl; }
            } else if (p->state == 1) {
                if (agl > p->max_agl) p->max_agl = agl;
                if (p->max_agl >= 300.0) {
                    p->c_sep = (agl <= 100.0) ? p->c_sep + 1 : 0;
                    if (p->c_sep >= 100) { p->state = 2; p->servo_us = 2000; }
                }
            } else if (p->state == 2) {
                int64_t m2 = (int64_t)in->ax_mg * in->ax_mg + (int64_t)in->ay_mg * in->ay_mg + (int64_t)in->az_mg * in->az_mg;
                int still = (m2 >= 800LL * 800LL) && (m2 <= 1200LL * 1200LL);
                p->c_land = (agl <= 10.0 && still) ? p->c_land + 1 : 0;
                if (p->c_land >= 150) p->state = 3;
            }
        }
    }

    out->servo_us = p->servo_us;
    out->state = p->state;
    out->buzzer = (p->state == 3);
    out->tele_ready = 0; out->tele_len = 0;
    if (!p->tx_init || (uint32_t)(in->t_ms - p->last_tx) >= 1000u) {
        int n = snprintf(out->tele, sizeof out->tele, "SHD,%lu,%d,%u\r",
                         (unsigned long)p->seq, (int)floor(p->last_agl + 0.5), (unsigned)p->state);
        if (n > 0 && n < (int)sizeof out->tele) { out->tele_len = (uint8_t)n; out->tele_ready = 1; }
        p->seq++; p->last_tx = in->t_ms; p->tx_init = 1;
    }
}
