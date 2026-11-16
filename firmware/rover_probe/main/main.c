// rover_probe: hardware check for the Yahboom MicroROS V2 control board (ESP32-S3).
// Pin map from Yahboom's V2 board page (study/MicroROS-V2/20031); their ESP-IDF samples are for
// an older 4-motor board and use different pins.
//
// Boots with motors held off, no servo pulses, buzzer off. Prints the I2C scan, then waits for a
// one-letter command on the USB serial port (115200), so a reset never moves anything:
//   i  IMU (QMI8658): 1 s of accel/gyro, gravity magnitude and gyro bias
//   p  power: 3 s of INA228 readings
//   b  buzzer + LEDs
//   e  encoders: 5 s count while you turn the wheels by hand
//   m  motors: each motor 50% then 70%, forward and reverse, 1.5 s each; encoder counts + current
//   s  servos: pan 0 -> -30 -> +30 -> 0, tilt -45 -> -30 -> -55 -> -45 (degrees, same scale as
//      /cmd_ptz_angle); then holds pan 0 / tilt -45
//   L  left motor only: +70% for 2 s (positive command; watch which way it turns)
//   R  right motor only: +70% for 2 s
//   l  lidar: start scan, count bytes and 0xAA55 packet headers for 3 s, stop scan
//   k  key: 8 s window between beeps, counts presses
//   o  OLED: all on 2 s, blank 1 s, then a test pattern that stays up
//   a  IMU axes: 12 s window between beeps; tip nose down, then roll left
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- V2 pin map ---
#define PIN_IMU_SCL 39
#define PIN_IMU_SDA 40
#define PIN_OLED_SCL 2
#define PIN_OLED_SDA 1
#define PIN_M1A 13  // left motor
#define PIN_M1B 14
#define PIN_M2A 11  // right motor
#define PIN_M2B 12
#define PIN_H1A 5   // left encoder
#define PIN_H1B 4
#define PIN_H2A 7   // right encoder
#define PIN_H2B 6
#define PIN_SERVO_PAN 8
#define PIN_SERVO_TILT 16
#define PIN_BUZZER 47
#define PIN_LED_BLUE 48
#define PIN_LED_GREEN 45
#define PIN_KEY 42
#define PIN_LIDAR_TX 17
#define PIN_LIDAR_RX 18

#define IMU_PORT I2C_NUM_0
#define AUX_PORT I2C_NUM_1   // OLED header bus: OLED, INA228, later BME280
#define QMI_ADDR 0x6B
#define SHUNT_OHMS 0.002
#define MOTOR_HZ 25000
#define MOTOR_BITS LEDC_TIMER_10_BIT
#define SERVO_BITS LEDC_TIMER_14_BIT

static int s_ina = -1;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cal;
static pcnt_unit_handle_t s_enc[2];

// ---------- I2C helpers ----------
static esp_err_t rd(i2c_port_t p, uint8_t a, uint8_t reg, uint8_t *b, size_t n) {
    return i2c_master_write_read_device(p, a, &reg, 1, b, n, pdMS_TO_TICKS(20));
}
static esp_err_t wr(i2c_port_t p, uint8_t a, uint8_t reg, uint8_t v) {
    uint8_t b[2] = {reg, v};
    return i2c_master_write_to_device(p, a, b, 2, pdMS_TO_TICKS(20));
}
static void bus_init(i2c_port_t p, int scl, int sda) {
    i2c_config_t c = {.mode = I2C_MODE_MASTER, .sda_io_num = sda, .scl_io_num = scl,
                      .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE,
                      .master.clk_speed = 100000};
    ESP_ERROR_CHECK(i2c_param_config(p, &c));
    ESP_ERROR_CHECK(i2c_driver_install(p, I2C_MODE_MASTER, 0, 0, 0));
}

static void identify(i2c_port_t p, uint8_t a) {
    uint8_t b[2] = {0};
    if (a >= 0x40 && a <= 0x4F && rd(p, a, 0x3E, b, 2) == ESP_OK) {
        uint16_t mfr = b[0] << 8 | b[1];
        rd(p, a, 0x3F, b, 2);
        uint16_t dev = b[0] << 8 | b[1];
        bool ina = mfr == 0x5449 && (dev >> 4) == 0x228;
        printf("  0x%02X: manufacturer 0x%04X device 0x%04X%s\n", a, mfr, dev, ina ? "  -> INA228" : "");
        if (ina && s_ina < 0) s_ina = a;
    } else if ((a == 0x6A || a == 0x6B) && rd(p, a, 0x00, b, 1) == ESP_OK) {
        printf("  0x%02X: reg 0x00 = 0x%02X%s\n", a, b[0], b[0] == 0x05 ? "  -> QMI8658" : "");
    } else if ((a == 0x76 || a == 0x77) && rd(p, a, 0xD0, b, 1) == ESP_OK) {
        printf("  0x%02X: chip id 0x%02X%s\n", a, b[0],
               b[0] == 0x60 ? "  -> BME280" : b[0] == 0x58 ? "  -> BMP280 (no humidity)" : "");
    } else {
        printf("  0x%02X: present%s\n", a, a == 0x3C ? " (OLED, SSD1306 per Yahboom)" : "");
    }
}
static void scan(i2c_port_t p, const char *name, int scl, int sda) {
    printf("I2C %s (SCL %d, SDA %d):\n", name, scl, sda);
    int n = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        uint8_t d;
        if (rd(p, a, 0x00, &d, 1) == ESP_OK) { identify(p, a); n++; }
    }
    if (!n) printf("  nothing\n");
}

// ---------- power ----------
static int32_t s20(const uint8_t *b) {
    int32_t v = (int32_t)((b[0] << 16) | (b[1] << 8) | b[2]) >> 4;
    return (v & 0x80000) ? v - 0x100000 : v;
}
static bool ina_read(double *v, double *a) {
    if (s_ina < 0) return false;
    uint8_t sh[3], vb[3];
    if (rd(AUX_PORT, s_ina, 0x04, sh, 3) != ESP_OK || rd(AUX_PORT, s_ina, 0x05, vb, 3) != ESP_OK) return false;
    *a = s20(sh) * 312.5e-9 / SHUNT_OHMS;
    *v = s20(vb) * 195.3125e-6;
    return true;
}
static double battery_adc(void) {
    int raw = 0, mv = 0;
    adc_oneshot_read(s_adc, ADC_CHANNEL_2, &raw);
    if (s_cal) adc_cali_raw_to_voltage(s_cal, raw, &mv);
    return mv / 1000.0 * 4.03;   // divider from Yahboom's sample
}

// ---------- outputs ----------
static void out_low(int pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}
static void pwm_channel(int ch, int pin, ledc_timer_t t) {
    ledc_channel_config_t c = {.gpio_num = pin, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = ch,
                               .timer_sel = t, .duty = 0, .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}
static void pwm_set(int ch, uint32_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}
// motor: m 0 = left (ch 0/1), 1 = right (ch 2/3); frac -1..1, 0 = coast (both low)
static void motor(int m, double frac) {
    uint32_t max = (1 << MOTOR_BITS) - 1, d = (uint32_t)(fabs(frac) * max);
    pwm_set(m * 2, frac > 0 ? d : 0);
    pwm_set(m * 2 + 1, frac < 0 ? d : 0);
}
static void motors_init(void) {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = MOTOR_BITS,
                             .timer_num = LEDC_TIMER_0, .freq_hz = MOTOR_HZ, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    pwm_channel(0, PIN_M1A, LEDC_TIMER_0); pwm_channel(1, PIN_M1B, LEDC_TIMER_0);
    pwm_channel(2, PIN_M2A, LEDC_TIMER_0); pwm_channel(3, PIN_M2B, LEDC_TIMER_0);
}
static bool s_servo_on = false;
static void servo(int ch, double deg) {   // -90..90 -> 500..2500 us at 50 Hz
    if (!s_servo_on) {
        ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = SERVO_BITS,
                                 .timer_num = LEDC_TIMER_1, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK};
        ESP_ERROR_CHECK(ledc_timer_config(&t));
        pwm_channel(4, PIN_SERVO_PAN, LEDC_TIMER_1);
        pwm_channel(5, PIN_SERVO_TILT, LEDC_TIMER_1);
        s_servo_on = true;
    }
    double us = 1500.0 + deg / 90.0 * 1000.0;
    pwm_set(ch, (uint32_t)(us / 20000.0 * ((1 << SERVO_BITS) - 1)));
}

// ---------- encoders ----------
static pcnt_unit_handle_t encoder(int a, int b) {
    pcnt_unit_config_t uc = {.high_limit = 30000, .low_limit = -30000, .flags.accum_count = 1};
    pcnt_unit_handle_t u;
    ESP_ERROR_CHECK(pcnt_new_unit(&uc, &u));
    pcnt_glitch_filter_config_t f = {.max_glitch_ns = 1000};
    pcnt_unit_set_glitch_filter(u, &f);
    pcnt_chan_config_t ca = {.edge_gpio_num = a, .level_gpio_num = b};
    pcnt_chan_config_t cb = {.edge_gpio_num = b, .level_gpio_num = a};
    pcnt_channel_handle_t A, B;
    ESP_ERROR_CHECK(pcnt_new_channel(u, &ca, &A));
    ESP_ERROR_CHECK(pcnt_new_channel(u, &cb, &B));
    pcnt_channel_set_edge_action(A, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(A, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(B, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(B, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_add_watch_point(u, 30000);
    pcnt_unit_add_watch_point(u, -30000);
    pcnt_unit_enable(u); pcnt_unit_clear_count(u); pcnt_unit_start(u);
    return u;
}
static int count(int i) { int c = 0; pcnt_unit_get_count(s_enc[i], &c); return c; }

// ---------- tests ----------
static void t_imu(void) {
    uint8_t id = 0;
    if (rd(IMU_PORT, QMI_ADDR, 0x00, &id, 1) != ESP_OK || id != 0x05) { printf("IMU: no QMI8658\n"); return; }
    wr(IMU_PORT, QMI_ADDR, 0x02, 0x40);   // CTRL1: address auto-increment, little endian
    wr(IMU_PORT, QMI_ADDR, 0x03, 0x15);   // CTRL2: accel +-4 g, 250 Hz
    wr(IMU_PORT, QMI_ADDR, 0x04, 0x55);   // CTRL3: gyro +-512 dps, 250 Hz
    wr(IMU_PORT, QMI_ADDR, 0x08, 0x03);   // CTRL7: accel + gyro on
    vTaskDelay(pdMS_TO_TICKS(100));
    double s[6] = {0}, t = 0; int n = 0;
    for (int i = 0; i < 100; i++) {
        uint8_t b[14];
        if (rd(IMU_PORT, QMI_ADDR, 0x33, b, 14) == ESP_OK) {
            t += (int16_t)(b[1] << 8 | b[0]) / 256.0;
            for (int k = 0; k < 6; k++) s[k] += (int16_t)(b[3 + 2 * k] << 8 | b[2 + 2 * k]);
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!n) { printf("IMU: read errors\n"); return; }
    double ax = s[0] / n / 8192, ay = s[1] / n / 8192, az = s[2] / n / 8192;
    printf("IMU (%d samples): accel %+.3f %+.3f %+.3f g, |a| = %.3f g; gyro bias %+.2f %+.2f %+.2f deg/s; temp %.1f C\n",
           n, ax, ay, az, sqrt(ax * ax + ay * ay + az * az), s[3] / n / 64, s[4] / n / 64, s[5] / n / 64, t / n);
}
static void t_power(void) {
    for (int i = 0; i < 6; i++) {
        double v, a;
        if (ina_read(&v, &a)) printf("power: %.3f V  %+.4f A  %.2f W  |  board ADC %.2f V\n", v, a, v * a, battery_adc());
        else printf("power: no INA228  |  board ADC %.2f V\n", battery_adc());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
static void t_buzz(void) {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
                             .timer_num = LEDC_TIMER_2, .freq_hz = 2700, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    pwm_channel(6, PIN_BUZZER, LEDC_TIMER_2);
    printf("buzzer: two short beeps (passive buzzer, 2.7 kHz); blue then green LED\n");
    for (int i = 0; i < 2; i++) { pwm_set(6, 512); vTaskDelay(pdMS_TO_TICKS(120)); pwm_set(6, 0); vTaskDelay(pdMS_TO_TICKS(120)); }
    ledc_stop(LEDC_LOW_SPEED_MODE, 6, 0);
    gpio_set_level(PIN_LED_BLUE, 1); vTaskDelay(pdMS_TO_TICKS(700)); gpio_set_level(PIN_LED_BLUE, 0);
    gpio_set_level(PIN_LED_GREEN, 1); vTaskDelay(pdMS_TO_TICKS(700)); gpio_set_level(PIN_LED_GREEN, 0);
}
static void beeps(int n) {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
                             .timer_num = LEDC_TIMER_2, .freq_hz = 2700, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    pwm_channel(6, PIN_BUZZER, LEDC_TIMER_2);
    for (int i = 0; i < n; i++) { pwm_set(6, 512); vTaskDelay(pdMS_TO_TICKS(120)); pwm_set(6, 0); vTaskDelay(pdMS_TO_TICKS(120)); }
    ledc_stop(LEDC_LOW_SPEED_MODE, 6, 0);
}
static void t_encoders(void) {
    printf("encoders: one beep = start turning, two beeps = stop (10 s)\n");
    beeps(1);
    pcnt_unit_clear_count(s_enc[0]); pcnt_unit_clear_count(s_enc[1]);
    for (int i = 0; i < 10; i++) { vTaskDelay(pdMS_TO_TICKS(1000)); printf("  left %6d  right %6d\n", count(0), count(1)); }
    beeps(2);
}
static void run_motor(int m, double frac, int ms) {
    pcnt_unit_clear_count(s_enc[m]);
    double sum = 0, peak = 0, vmin = 99; int n = 0;
    int64_t end = esp_timer_get_time() + ms * 1000LL;
    motor(m, frac);
    while (esp_timer_get_time() < end) {
        double v, a;
        if (ina_read(&v, &a)) { sum += a; n++; if (a > peak) peak = a; if (v < vmin) vmin = v; }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    int c = count(m);
    motor(m, 0);
    printf("  %s %+3.0f%%: %6d counts in %.1f s = %+6.0f counts/s; current mean %.3f A peak %.3f A, min %.2f V\n",
           m ? "right" : "left ", frac * 100, c, ms / 1000.0, c * 1000.0 / ms, n ? sum / n : 0, peak, vmin);
    vTaskDelay(pdMS_TO_TICKS(1000));   // coast to a stop
}
static void t_motors(void) {
    printf("motors (wheels must be off the ground):\n");
    static const double duty[] = {0.5, 0.7};
    for (int m = 0; m < 2; m++)
        for (int d = 0; d < 2; d++) { run_motor(m, duty[d], 1500); run_motor(m, -duty[d], 1500); }
}
static void t_servos(void) {
    static const double pan[] = {0, -30, 30, 0}, tilt[] = {-45, -30, -55, -45};
    printf("servos: pan 0 -30 +30 0, then tilt -45 -30 -55 -45 (1 s each)\n");
    servo(5, -45);
    for (int i = 0; i < 4; i++) { servo(4, pan[i]); printf("  pan %+.0f\n", pan[i]); vTaskDelay(pdMS_TO_TICKS(1000)); }
    for (int i = 0; i < 4; i++) { servo(5, tilt[i]); printf("  tilt %+.0f\n", tilt[i]); vTaskDelay(pdMS_TO_TICKS(1000)); }
    printf("  holding pan 0 / tilt -45\n");
}
static void t_lidar(void) {
    static bool init = false;
    if (!init) {
        uart_config_t c = {.baud_rate = 230400, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                           .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 4096, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &c));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, PIN_LIDAR_TX, PIN_LIDAR_RX, -1, -1));
        init = true;
    }
    const uint8_t start[] = {0xA5, 0x60}, stop[] = {0xA5, 0x65};
    uart_flush_input(UART_NUM_1);
    uart_write_bytes(UART_NUM_1, start, 2);
    vTaskDelay(pdMS_TO_TICKS(1500));   // spin-up
    uart_flush_input(UART_NUM_1);
    uint8_t buf[512], prev = 0; long bytes = 0, heads = 0;
    int64_t end = esp_timer_get_time() + 3000000;
    while (esp_timer_get_time() < end) {
        int n = uart_read_bytes(UART_NUM_1, buf, sizeof buf, pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) { if (prev == 0xAA && buf[i] == 0x55) heads++; prev = buf[i]; }
        if (n > 0) bytes += n;
    }
    uart_write_bytes(UART_NUM_1, stop, 2);
    printf("lidar: %ld bytes, %ld packet headers in 3 s (%.0f B/s, %.0f packets/s); stop sent\n",
           bytes, heads, bytes / 3.0, heads / 3.0);
}
// SSD1306 128x32 on the OLED header bus at 0x3C
#define OLED_ADDR 0x3C
static esp_err_t oled_cmds(const uint8_t *c, size_t n) {
    uint8_t b[32] = {0x00};
    memcpy(b + 1, c, n);
    return i2c_master_write_to_device(AUX_PORT, OLED_ADDR, b, n + 1, pdMS_TO_TICKS(50));
}
static void oled_show(const uint8_t *fb) {   // 512 bytes: 4 pages x 128 columns
    const uint8_t win[] = {0x21, 0, 127, 0x22, 0, 3};
    oled_cmds(win, sizeof win);
    for (int i = 0; i < 512; i += 16) {
        uint8_t b[17] = {0x40};
        memcpy(b + 1, fb + i, 16);
        i2c_master_write_to_device(AUX_PORT, OLED_ADDR, b, 17, pdMS_TO_TICKS(50));
    }
}
static void px(uint8_t *fb, int x, int y) {
    if (x >= 0 && x < 128 && y >= 0 && y < 32) fb[(y / 8) * 128 + x] |= 1 << (y % 8);
}
static void t_oled(void) {
    const uint8_t init[] = {0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
                            0xA1, 0xC8, 0xDA, 0x02, 0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF};
    if (oled_cmds(init, sizeof init) != ESP_OK) { printf("OLED: no answer at 0x3C\n"); return; }
    static uint8_t fb[512];
    printf("OLED: all pixels on for 2 s\n");
    memset(fb, 0xFF, sizeof fb); oled_show(fb); vTaskDelay(pdMS_TO_TICKS(2000));
    printf("OLED: blank for 1 s\n");
    memset(fb, 0, sizeof fb); oled_show(fb); vTaskDelay(pdMS_TO_TICKS(1000));
    // Test pattern: border, solid 10x10 square top-left, small 4x4 dot top-right,
    // a diagonal from top-left to bottom-right, and bars along the bottom.
    for (int x = 0; x < 128; x++) { px(fb, x, 0); px(fb, x, 31); }
    for (int y = 0; y < 32; y++) { px(fb, 0, y); px(fb, 127, y); }
    for (int x = 2; x < 12; x++) for (int y = 2; y < 12; y++) px(fb, x, y);
    for (int x = 120; x < 124; x++) for (int y = 3; y < 7; y++) px(fb, x, y);
    for (int x = 0; x < 128; x++) px(fb, x, x / 4);
    for (int x = 20; x < 108; x += 8) for (int y = 24; y < 30; y++) { px(fb, x, y); px(fb, x + 1, y); }
    oled_show(fb);
    printf("OLED: test pattern on (border, big square top-left, small dot top-right, diagonal, bars at the bottom)\n");
}
static void t_key(void) {
    printf("key: one beep = start, press KEY1 a few times, two beeps = stop (8 s)\n");
    beeps(1);
    int presses = 0, prev = 1;
    for (int i = 0; i < 400; i++) {
        int lv = gpio_get_level(PIN_KEY);
        if (prev && !lv) { presses++; printf("  pressed at %.1f s\n", i * 0.02); }
        prev = lv;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    beeps(2);
    printf("key: %d press(es)\n", presses);
}
static void imu_start(void) {
    wr(IMU_PORT, QMI_ADDR, 0x02, 0x40);
    wr(IMU_PORT, QMI_ADDR, 0x03, 0x15);
    wr(IMU_PORT, QMI_ADDR, 0x04, 0x55);
    wr(IMU_PORT, QMI_ADDR, 0x08, 0x03);
    vTaskDelay(pdMS_TO_TICKS(100));
}
static void t_imu_axes(void) {
    imu_start();
    printf("IMU axes: one beep = start; tip the NOSE DOWN, hold, then back level; then ROLL onto the LEFT side, hold, back; two beeps = stop (12 s)\n");
    beeps(1);
    for (int i = 0; i < 24; i++) {
        uint8_t b[12];
        if (rd(IMU_PORT, QMI_ADDR, 0x35, b, 12) == ESP_OK) {
            double a[3], g[3];
            for (int k = 0; k < 3; k++) { a[k] = (int16_t)(b[2 * k + 1] << 8 | b[2 * k]) / 8192.0;
                                          g[k] = (int16_t)(b[2 * k + 7] << 8 | b[2 * k + 6]) / 64.0; }
            printf("  %4.1f s  accel x %+.2f y %+.2f z %+.2f g   gyro x %+6.1f y %+6.1f z %+6.1f deg/s\n",
                   i * 0.5, a[0], a[1], a[2], g[0], g[1], g[2]);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    beeps(2);
}

void app_main(void) {
    out_low(PIN_M1A); out_low(PIN_M1B); out_low(PIN_M2A); out_low(PIN_M2B);
    out_low(PIN_BUZZER); out_low(PIN_LED_BLUE); out_low(PIN_LED_GREEN);
    gpio_reset_pin(PIN_KEY); gpio_set_direction(PIN_KEY, GPIO_MODE_INPUT); gpio_set_pull_mode(PIN_KEY, GPIO_PULLUP_ONLY);
    motors_init();
    s_enc[0] = encoder(PIN_H1A, PIN_H1B);
    s_enc[1] = encoder(PIN_H2A, PIN_H2B);

    adc_oneshot_unit_init_cfg_t u = {.unit_id = ADC_UNIT_1};
    adc_oneshot_new_unit(&u, &s_adc);
    adc_oneshot_chan_cfg_t ch = {.atten = ADC_ATTEN_DB_11, .bitwidth = ADC_BITWIDTH_DEFAULT};
    adc_oneshot_config_channel(s_adc, ADC_CHANNEL_2, &ch);
    adc_cali_curve_fitting_config_t cc = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_11, .bitwidth = ADC_BITWIDTH_DEFAULT};
    adc_cali_create_scheme_curve_fitting(&cc, &s_cal);

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\nrover_probe 0.2 (V2 pin map). Motors off, no servo pulses.\n");
    bus_init(IMU_PORT, PIN_IMU_SCL, PIN_IMU_SDA);
    bus_init(AUX_PORT, PIN_OLED_SCL, PIN_OLED_SDA);
    scan(IMU_PORT, "IMU bus", PIN_IMU_SCL, PIN_IMU_SDA);
    scan(AUX_PORT, "OLED header", PIN_OLED_SCL, PIN_OLED_SDA);

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    printf("ready: i p b e m L R s l k a o\n");
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(100)) != 1) continue;
        switch (c) {
            case 'i': t_imu(); break;
            case 'p': t_power(); break;
            case 'b': t_buzz(); break;
            case 'e': t_encoders(); break;
            case 'm': t_motors(); break;
            case 'L': run_motor(0, 0.7, 2000); break;
            case 'R': run_motor(1, 0.7, 2000); break;
            case 's': t_servos(); break;
            case 'l': t_lidar(); break;
            case 'k': t_key(); break;
            case 'a': t_imu_axes(); break;
            case 'o': t_oled(); break;
            default: continue;
        }
        printf("done %c\n", c);
    }
}
