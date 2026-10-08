#include "flight.h"
#include <stdio.h>
#include <string.h>
static FlightState st;
int main(int argc, char **argv) {
    FILE *f = argc > 1 ? fopen(argv[1], "r") : stdin;
    char line[256]; Sample s; Outputs o;
    int idx = 0, prev_phase = -1, prev_servo = -1;
    if (!f) return 2;
    flight_init(&st);
    if (!fgets(line, sizeof line, f)) return 2;
    while (fgets(line, sizeof line, f)) {
        unsigned t; int pp, pv, ax, ay, az;
        if (sscanf(line, "%u,%d,%d,%d,%d,%d", &t, &pp, &pv, &ax, &ay, &az) != 6) continue;
        s.t_ms = t; s.press_pa = pp; s.press_valid = (uint8_t)pv;
        s.ax_mg = (int16_t)ax; s.ay_mg = (int16_t)ay; s.az_mg = (int16_t)az;
        memset(&o, 0, sizeof o);
        flight_step(&s, &st, &o);
        if (o.phase != prev_phase) { printf("P %d %d\n", idx, o.phase); prev_phase = o.phase; }
        if (o.servo_us != prev_servo) { printf("S %d %d\n", idx, o.servo_us); prev_servo = o.servo_us; }
        if (o.buzzer != (o.phase == PH_LANDED)) printf("BUZBAD %d\n", idx);
        if (o.tele_ready) {
            if (o.tele_len == 0 || o.tele[o.tele_len - 1] != '\r' || o.tele[o.tele_len] != '\0') printf("TELEBAD %d\n", idx);
            else { o.tele[o.tele_len - 1] = 0; printf("T %d %s\n", idx, o.tele); }
        }
        idx++;
    }
    printf("N %d\n", idx);
    return 0;
}
