#include "ina228.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "ina228";

#define REG_CONFIG 0x00
#define REG_ADC_CONFIG 0x01
#define REG_SHUNT_CAL 0x02
#define REG_VBUS 0x05
#define REG_DIETEMP 0x06
#define REG_CURRENT 0x07
#define REG_POWER 0x08
#define REG_ENERGY 0x09
#define REG_CHARGE 0x0A
#define REG_MANUFACTURER 0x3E
#define REG_DEVICE 0x3F

#define MAX_CURRENT_A 10.0   // sets the current resolution: 10 A / 2^19 = 19 uA per bit

static i2c_master_dev_handle_t s_dev;
static double s_current_lsb;

static bool rd(uint8_t reg, uint8_t *b, size_t n) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, b, n, 20) == ESP_OK;
}
static bool wr16(uint8_t reg, uint16_t v) {
    uint8_t b[3] = {reg, v >> 8, v & 0xFF};
    return i2c_master_transmit(s_dev, b, 3, 20) == ESP_OK;
}
static int32_t s20(const uint8_t *b) {   // 20-bit two's complement in bits 23:4
    int32_t v = (int32_t)((b[0] << 16) | (b[1] << 8) | b[2]) >> 4;
    return (v & 0x80000) ? v - 0x100000 : v;
}

bool ina228_init(i2c_master_bus_handle_t bus, uint8_t addr, double shunt_ohms) {
    if (i2c_master_probe(bus, addr, 50) != ESP_OK) return false;
    i2c_device_config_t dc = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = 400000};
    if (i2c_master_bus_add_device(bus, &dc, &s_dev) != ESP_OK) return false;
    uint8_t b[2];
    if (!rd(REG_MANUFACTURER, b, 2) || (b[0] << 8 | b[1]) != 0x5449) return false;
    if (!rd(REG_DEVICE, b, 2) || ((b[0] << 8 | b[1]) >> 4) != 0x228) return false;

    s_current_lsb = MAX_CURRENT_A / 524288.0;
    uint16_t cal = (uint16_t)lround(13107.2e6 * s_current_lsb * shunt_ohms);   // datasheet eq. 2, ADCRANGE 0
    // ADC_CONFIG: continuous bus + shunt + temperature (0xF), 1052 us conversions, 16-sample
    // averaging: a new result about every 50 ms.
    bool ok = wr16(REG_ADC_CONFIG, (0xF << 12) | (5 << 9) | (5 << 6) | (5 << 3) | 2) &&
              wr16(REG_SHUNT_CAL, cal) &&
              wr16(REG_CONFIG, 1 << 14);   // RSTACC: zero the charge and energy accumulators
    ESP_LOGI(TAG, "at 0x%02X, shunt %.4f ohm, SHUNT_CAL %u", addr, shunt_ohms, cal);
    return ok;
}

bool ina228_read(ina228_reading_t *out) {
    uint8_t v[3], c[3], p[3], t[2], e[5], q[5];
    if (!rd(REG_VBUS, v, 3) || !rd(REG_CURRENT, c, 3) || !rd(REG_POWER, p, 3) ||
        !rd(REG_DIETEMP, t, 2) || !rd(REG_ENERGY, e, 5) || !rd(REG_CHARGE, q, 5)) return false;
    out->bus_v = s20(v) * 195.3125e-6;
    out->current_a = s20(c) * s_current_lsb;
    out->power_w = (double)((p[0] << 16) | (p[1] << 8) | p[2]) * 3.2 * s_current_lsb;
    out->die_c = (int16_t)(t[0] << 8 | t[1]) * 7.8125e-3;
    uint64_t energy = 0;
    for (int i = 0; i < 5; i++) energy = energy << 8 | e[i];
    int64_t charge = 0;
    for (int i = 0; i < 5; i++) charge = charge << 8 | q[i];
    if (charge & (1LL << 39)) charge -= 1LL << 40;   // 40-bit two's complement
    out->energy_wh = energy * 16.0 * 3.2 * s_current_lsb / 3600.0;
    out->charge_ah = charge * s_current_lsb / 3600.0;
    return true;
}
