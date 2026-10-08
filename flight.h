#ifndef FLIGHT_H
#define FLIGHT_H
#include <stdint.h>

typedef enum { PH_PAD = 0, PH_ASCENT = 1, PH_DESCENT = 2, PH_SEPARATED = 3, PH_LANDED = 4 } Phase;

typedef struct {
    uint32_t t_ms;        /* free-running ms counter, wraps at 2^32 */
    int32_t  press_pa;    /* undefined when press_valid == 0 */
    uint8_t  press_valid;
    int16_t  ax_mg, ay_mg, az_mg;
} Sample;

typedef struct {
    uint16_t servo_us;    /* 1000 = locked, 2000 = released */
    uint8_t  buzzer;      /* 1 only in PH_LANDED */
    uint8_t  phase;
    uint8_t  tele_ready;  /* 1 when tele[] holds a new packet */
    uint8_t  tele_len;    /* bytes, excluding NUL */
    char     tele[80];
} Outputs;

typedef struct { uint64_t storage[32]; } FlightState;  /* 256 bytes, static only */

void flight_init(FlightState *st);
void flight_step(const Sample *in, FlightState *st, Outputs *out);  /* called every 10 ms */
#endif
