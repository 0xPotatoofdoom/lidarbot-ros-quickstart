#include "qmi8658.h"
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "qmi8658";

#define ADDR 0x6B
#define G 9.80665
#define ACCEL_LSB_PER_G 8192.0    // +-4 g
#define GYRO_LSB_PER_DPS 64.0     // +-512 deg/s
#define TILT_GAIN 0.02            // how fast gravity corrects roll/pitch drift, per sample

static i2c_master_dev_handle_t s_dev;
static double s_bias[3];
static double s_roll, s_pitch, s_yaw;

static bool wr(uint8_t reg, uint8_t v) {
    uint8_t b[2] = {reg, v};
    return i2c_master_transmit(s_dev, b, 2, 20) == ESP_OK;
}
static bool rd(uint8_t reg, uint8_t *b, size_t n) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, b, n, 20) == ESP_OK;
}

// Raw sample in the robot frame. The chip is mounted turned 180 deg about z
// (its x points back, y right), so x and y change sign.
static bool raw(double a[3], double g[3], double *temp) {
    uint8_t b[14];
    if (!rd(0x33, b, 14)) return false;
    int16_t v[7];
    for (int i = 0; i < 7; i++) v[i] = (int16_t)(b[2 * i + 1] << 8 | b[2 * i]);
    *temp = v[0] / 256.0;
    const double sign[3] = {-1, -1, 1};
    for (int k = 0; k < 3; k++) {
        a[k] = sign[k] * v[1 + k] / ACCEL_LSB_PER_G * G;
        g[k] = sign[k] * v[4 + k] / GYRO_LSB_PER_DPS * M_PI / 180.0;
    }
    return true;
}

bool qmi8658_init(i2c_master_bus_handle_t bus) {
    if (i2c_master_probe(bus, ADDR, 50) != ESP_OK) return false;
    i2c_device_config_t dc = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = ADDR, .scl_speed_hz = 400000};
    if (i2c_master_bus_add_device(bus, &dc, &s_dev) != ESP_OK) return false;
    uint8_t id = 0;
    if (!rd(0x00, &id, 1) || id != 0x05) return false;
    bool ok = wr(0x02, 0x40) &&   // CTRL1: register auto-increment, little endian
              wr(0x03, 0x15) &&   // CTRL2: accel +-4 g, 250 Hz
              wr(0x04, 0x55) &&   // CTRL3: gyro +-512 dps, 250 Hz
              wr(0x08, 0x03);     // CTRL7: accel and gyro on
    vTaskDelay(pdMS_TO_TICKS(50));
    return ok;
}

bool qmi8658_calibrate(int ms) {
    double a[3], g[3], t, sum[3] = {0}, asum[3] = {0};
    int n = 0;
    for (int i = 0; i < ms / 5; i++) {
        if (raw(a, g, &t)) {
            for (int k = 0; k < 3; k++) { sum[k] += g[k]; asum[k] += a[k]; }
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (n < 10) return false;
    for (int k = 0; k < 3; k++) s_bias[k] = sum[k] / n;
    s_roll = atan2(asum[1], asum[2]);
    s_pitch = atan2(-asum[0], sqrt(asum[1] * asum[1] + asum[2] * asum[2]));
    s_yaw = 0;
    ESP_LOGI(TAG, "gyro bias %.2f %.2f %.2f deg/s from %d samples", s_bias[0] * 180 / M_PI,
             s_bias[1] * 180 / M_PI, s_bias[2] * 180 / M_PI, n);
    return true;
}

bool qmi8658_update(double dt, imu_state_t *out) {
    double a[3], g[3];
    if (!raw(a, g, &out->temp_c)) return false;
    for (int k = 0; k < 3; k++) { g[k] -= s_bias[k]; out->accel[k] = a[k]; out->gyro[k] = g[k]; }

    // Euler-rate integration, then pull roll/pitch toward gravity when the accelerometer reads ~1 g.
    double sr = sin(s_roll), cr = cos(s_roll), cp = cos(s_pitch), tp = tan(s_pitch);
    s_roll += dt * (g[0] + sr * tp * g[1] + cr * tp * g[2]);
    s_pitch += dt * (cr * g[1] - sr * g[2]);
    s_yaw += dt * (sr / cp * g[1] + cr / cp * g[2]);
    double norm = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (fabs(norm - G) < 0.15 * G) {
        double roll_acc = atan2(a[1], a[2]);
        double pitch_acc = atan2(-a[0], sqrt(a[1] * a[1] + a[2] * a[2]));
        s_roll += TILT_GAIN * (roll_acc - s_roll);
        s_pitch += TILT_GAIN * (pitch_acc - s_pitch);
    }
    s_yaw = atan2(sin(s_yaw), cos(s_yaw));

    double cy = cos(s_yaw / 2), sy = sin(s_yaw / 2), cpp = cos(s_pitch / 2), sp = sin(s_pitch / 2);
    double crr = cos(s_roll / 2), srr = sin(s_roll / 2);
    out->q[0] = srr * cpp * cy - crr * sp * sy;
    out->q[1] = crr * sp * cy + srr * cpp * sy;
    out->q[2] = crr * cpp * sy - srr * sp * cy;
    out->q[3] = crr * cpp * cy + srr * sp * sy;
    return true;
}
