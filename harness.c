#include "flight.h"
#include <stdio.h>
#include <string.h>
typedef struct { uint32_t t; int32_t p; int16_t ax, ay, az; uint8_t v, pad; } Rec;
static FlightState st;
int main(void) {
    Rec r; Outputs o; Sample s;
    int idx = 0, ps = -1, pv = -1, pb = -1;
    flight_init(&st);
    while (fread(&r, sizeof r, 1, stdin) == 1) {
        s.t_ms = r.t; s.press_pa = r.p; s.press_valid = r.v; s.ax_mg = r.ax; s.ay_mg = r.ay; s.az_mg = r.az;
        memset(&o, 0, sizeof o);
        flight_step(&s, &st, &o);
        if (o.state != ps) { printf("ST %d %d\n", idx, o.state); ps = o.state; }
        if (o.servo_us != pv) { printf("SV %d %d\n", idx, o.servo_us); pv = o.servo_us; }
        if (o.buzzer != pb) { printf("BZ %d %d\n", idx, o.buzzer); pb = o.buzzer; }
        if (o.tele_ready) {
            if (o.tele_len == 0 || o.tele_len >= sizeof o.tele || o.tele[o.tele_len - 1] != '\r' || o.tele[o.tele_len] != '\0') printf("TELEBAD %d\n", idx);
            else { o.tele[o.tele_len - 1] = 0; printf("T %d %s\n", idx, o.tele); }
        }
        idx++;
    }
    printf("N %d\n", idx);
    return 0;
}
