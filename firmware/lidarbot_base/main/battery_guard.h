// Low-battery guard: warns, then stops the lidar and wheels, from the INA228's pack readings.
// Pure logic (no hardware calls) so it can be tested on the host: test/test_battery_guard.c.
#pragma once
#include <stdbool.h>

typedef enum {
    BATT_NO_PACK,    // under NO_PACK_V: running from USB with the pack unplugged
    BATT_OK,
    BATT_LOW,        // warn: double beep every 30 s
    BATT_CRITICAL,   // long beep every 10 s, lidar and wheels stopped
} batt_state_t;

typedef struct {
    batt_state_t state;
    double rest_v;   // smoothed, sag-corrected pack voltage (V)
    bool beep;       // beep this tick ...
    int beep_ms;     // ... for this long
    bool stop_loads; // true while critical: stop the lidar motor and the wheels
} batt_guard_t;

#define BATT_TICK_S 0.2       // called on every INA228 read (5 Hz)
#define BATT_NO_PACK_V 6.0    // a 3S pack is never this low; the INA228 reads ~0 V without it
#define BATT_PACK_OHMS 0.27   // measured 2026-11-16 (0.6 m/s full-load test): V_rest = V + I * R
#define BATT_LOW_V 10.6       // about the last 10% (idle discharge curve, 2026-11-11)
#define BATT_CRITICAL_V 10.2  // Yahboom's alarm point; 3.4 V per cell
#define BATT_HYSTERESIS_V 0.15
#define BATT_SMOOTH_S 5.0     // time constant: a dip under load can't trip it

void batt_guard_init(batt_guard_t *g);
// One reading (bus volts, amps out of the pack). Updates state and this tick's actions.
void batt_guard_update(batt_guard_t *g, double bus_v, double current_a);
const char *batt_state_name(batt_state_t s);
