// Host test for main/battery_guard.c (no ESP-IDF needed):
//   cc -std=c11 -Wall -Wextra -fsanitize=address,undefined -I main test/test_battery_guard.c main/battery_guard.c -o /tmp/tbg && /tmp/tbg
#include <assert.h>
#include <stdio.h>
#include "battery_guard.h"

static int beeps(batt_guard_t *g, double v, double a, int ticks, int *long_ms) {
    int n = 0;
    for (int i = 0; i < ticks; i++) {
        batt_guard_update(g, v, a);
        if (g->beep) { n++; if (long_ms) *long_ms = g->beep_ms; }
    }
    return n;
}

int main(void) {
    batt_guard_t g;

    // USB only, pack unplugged: INA228 reads ~0 V. Never an alarm, never stops anything.
    batt_guard_init(&g);
    assert(beeps(&g, 0.02, 0.0, 600, NULL) == 0);
    assert(g.state == BATT_NO_PACK && !g.stop_loads);

    // Full pack: OK at once (no slow climb from 0 V through the alarm levels).
    batt_guard_update(&g, 12.4, 0.3);
    assert(g.state == BATT_OK && !g.beep);

    // Sag under load: 10.4 V at 1.1 A is 10.7 V resting -> still OK.
    batt_guard_init(&g);
    assert(beeps(&g, 10.4, 1.1, 300, NULL) == 0);
    assert(g.state == BATT_OK);

    // A short dip (2 s at 9.8 V) on a healthy pack doesn't trip it.
    batt_guard_init(&g);
    beeps(&g, 11.6, 0.4, 50, NULL);
    assert(beeps(&g, 9.8, 0.4, 10, NULL) == 0);
    assert(g.state == BATT_OK);

    // Slow discharge at 0.3 A idle: low (double beeps), then critical (long beeps, loads off).
    batt_guard_init(&g);
    beeps(&g, 11.0, 0.3, 100, NULL);
    assert(g.state == BATT_OK);
    int n = beeps(&g, 10.4, 0.3, 150 * 2, NULL);   // 60 s at 10.4 V (10.48 resting)
    assert(g.state == BATT_LOW && !g.stop_loads);
    assert(n >= 4 && n <= 6);                        // pairs every 30 s
    int ms = 0;
    n = beeps(&g, 10.0, 0.3, 50 * 3, &ms);           // 30 s at 10.0 V
    assert(g.state == BATT_CRITICAL && g.stop_loads);
    assert(ms == 700 && n >= 2);

    // Loads off: current drops, resting estimate stays critical (no flapping).
    assert(beeps(&g, 10.05, 0.15, 100, NULL) >= 2);
    assert(g.state == BATT_CRITICAL);

    // Charger on: voltage rises well above the thresholds -> back to OK, loads allowed again.
    beeps(&g, 12.2, 0.2, 200, NULL);
    assert(g.state == BATT_OK && !g.stop_loads);

    // Pack unplugged mid-alarm: back to NO_PACK, nothing stopped.
    batt_guard_init(&g);
    beeps(&g, 10.0, 0.3, 50, NULL);
    batt_guard_update(&g, 0.0, 0.0);
    assert(g.state == BATT_NO_PACK && !g.stop_loads);

    puts("battery_guard: all checks passed");
    return 0;
}
