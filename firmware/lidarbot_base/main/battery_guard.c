#include "battery_guard.h"
#include <stdlib.h>
#include <string.h>

#define LOW_PERIOD_TICKS 150       // 30 s
#define CRITICAL_PERIOD_TICKS 50   // 10 s
#define DOUBLE_GAP_TICKS 2         // second beep of the pair 0.4 s after the first

static unsigned s_tick;

void batt_guard_init(batt_guard_t *g) {
    g->state = BATT_NO_PACK;
    g->rest_v = 0;
    g->beep = false;
    g->beep_ms = 0;
    g->stop_loads = false;
    s_tick = 0;
}

static batt_state_t next_state(batt_state_t s, double v) {
    // Hysteresis: getting out of a state needs the voltage clearly back above its threshold.
    double h = BATT_HYSTERESIS_V;
    switch (s) {
    case BATT_CRITICAL:
        return v > BATT_CRITICAL_V + h ? (v > BATT_LOW_V + h ? BATT_OK : BATT_LOW) : BATT_CRITICAL;
    case BATT_LOW:
        if (v < BATT_CRITICAL_V) return BATT_CRITICAL;
        return v > BATT_LOW_V + h ? BATT_OK : BATT_LOW;
    default:
        if (v < BATT_CRITICAL_V) return BATT_CRITICAL;
        return v < BATT_LOW_V ? BATT_LOW : BATT_OK;
    }
}

void batt_guard_update(batt_guard_t *g, double bus_v, double current_a) {
    g->beep = false;
    g->beep_ms = 0;
    if (bus_v < BATT_NO_PACK_V) {
        g->state = BATT_NO_PACK;   // forget the history: a pack plugged in later starts fresh
        g->rest_v = 0;
        g->stop_loads = false;
        return;
    }
    double v = bus_v + (current_a > 0 ? current_a : 0) * BATT_PACK_OHMS;
    if (g->state == BATT_NO_PACK) {
        g->rest_v = v;             // first reading with a pack: start from it, not from 0
        g->state = BATT_OK;
        s_tick = 0;
    } else {
        g->rest_v += (BATT_TICK_S / BATT_SMOOTH_S) * (v - g->rest_v);
    }
    batt_state_t s = next_state(g->state, g->rest_v);
    if (s != g->state) s_tick = 0;  // alarm at once when a state is entered
    g->state = s;
    g->stop_loads = (s == BATT_CRITICAL);

    if (s == BATT_LOW) {
        unsigned t = s_tick % LOW_PERIOD_TICKS;
        if (t == 0 || t == DOUBLE_GAP_TICKS) { g->beep = true; g->beep_ms = 80; }
    } else if (s == BATT_CRITICAL) {
        if (s_tick % CRITICAL_PERIOD_TICKS == 0) { g->beep = true; g->beep_ms = 700; }
    }
    s_tick++;
}

const char *batt_state_name(batt_state_t s) {
    switch (s) {
    case BATT_NO_PACK: return "NO PACK";
    case BATT_OK: return "OK";
    case BATT_LOW: return "LOW";
    case BATT_CRITICAL: return "CRITICAL";
    }
    return "?";
}

batt_cmd_t batt_parse_cmd(const char *line, double *volts) {
    static const char prefix[] = "batt sim ";
    while (*line == ' ') line++;
    if (strncmp(line, prefix, sizeof prefix - 1) != 0) return BATT_CMD_NONE;
    const char *arg = line + sizeof prefix - 1;
    while (*arg == ' ') arg++;
    size_t n = strcspn(arg, " \r\n");
    if (n == 3 && strncmp(arg, "off", 3) == 0) return BATT_CMD_SIM_OFF;
    char *end;
    double v = strtod(arg, &end);
    if (end == arg || end != arg + n || !(v > BATT_SIM_MIN_V && v < BATT_SIM_MAX_V)) return BATT_CMD_NONE;
    *volts = v;
    return BATT_CMD_SIM;
}

void rail_init(rail_t *r) {
    r->on = true;
    r->low_ticks = 0;
}

void rail_update(rail_t *r, double volts) {
    if (!(volts < RAIL_OFF_V)) r->low_ticks = 0;   // NaN (no reading) counts as on
    else if (r->low_ticks < RAIL_OFF_TICKS) r->low_ticks++;
    if (r->low_ticks >= RAIL_OFF_TICKS) r->on = false;
    else if (volts > RAIL_ON_V) r->on = true;
}
