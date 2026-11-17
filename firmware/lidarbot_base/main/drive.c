#include "drive.h"
#include <math.h>
#include "board.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "drive";

#define CONTROL_HZ 100
#define PWM_HZ 25000
#define PWM_BITS LEDC_TIMER_10_BIT
#define PWM_MAX ((1 << 10) - 1)
// Feedforward from the stand test (rover_probe, wheels in the air): nothing below ~50% duty,
// about 0.43 m/s at 70%. The PI terms make up the rest under load.
#define DUTY_START 0.50
#define DUTY_PER_MPS 0.46
#define KP 0.6                    // duty per m/s of error
#define KI 3.0                    // duty per m of accumulated error
#define SPEED_FILTER 0.4          // new-sample weight for the measured wheel speed
// With a zero target, short the motor (both inputs high: brake on this kind of H-bridge) while
// the wheel still turns faster than this, then let go (both low: coast). Coasting alone rolled
// 28 cm from 0.6 m/s (2026-11-16).
#define BRAKE_ABOVE_MPS 0.02

static const double M_PER_COUNT = M_PI * WHEEL_DIAMETER_M / COUNTS_PER_REV;

typedef struct {
    int pin_a, pin_b, ch_a, ch_b;
    int drive_sign;               // +1 if positive duty on A drives the wheel forward
    int count_sign;               // +1 if the encoder counts up going forward
    pcnt_unit_handle_t pcnt;
    int last_count;
    double speed, target, integral, duty;
    bool brake;
} wheel_t;

static wheel_t s_wheel[2] = {
    // Directions measured with firmware/rover_probe (2026-11-15).
    // left: positive on A drives it backward; its encoder counts down going forward
    {.pin_a = PIN_M1A, .pin_b = PIN_M1B, .ch_a = 0, .ch_b = 1, .drive_sign = -1, .count_sign = -1},
    {.pin_a = PIN_M2A, .pin_b = PIN_M2B, .ch_a = 2, .ch_b = 3, .drive_sign = +1, .count_sign = +1},
};
static const int ENC_PINS[2][2] = {{PIN_H1A, PIN_H1B}, {PIN_H2A, PIN_H2B}};

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static double s_cmd_v, s_cmd_w;
static int64_t s_cmd_time;
static drive_state_t s_state;
static volatile bool s_inhibit;

static void pwm_set(int ch, double frac) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, (uint32_t)(fmin(fmax(frac, 0.0), 1.0) * PWM_MAX));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

// duty -1..1 in the wheel's forward sense; 0 = coast (both inputs low); brake = both high
static void wheel_output(wheel_t *w, double duty) {
    if (w->brake) {
        pwm_set(w->ch_a, 1.0);
        pwm_set(w->ch_b, 1.0);
        return;
    }
    double d = duty * w->drive_sign;
    pwm_set(w->ch_a, d > 0 ? d : 0);
    pwm_set(w->ch_b, d < 0 ? -d : 0);
}

static pcnt_unit_handle_t encoder_new(int a, int b) {
    pcnt_unit_config_t uc = {.high_limit = 30000, .low_limit = -30000, .flags.accum_count = 1};
    pcnt_unit_handle_t u;
    ESP_ERROR_CHECK(pcnt_new_unit(&uc, &u));
    pcnt_glitch_filter_config_t f = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(u, &f));
    pcnt_chan_config_t ca = {.edge_gpio_num = a, .level_gpio_num = b};
    pcnt_chan_config_t cb = {.edge_gpio_num = b, .level_gpio_num = a};
    pcnt_channel_handle_t A, B;
    ESP_ERROR_CHECK(pcnt_new_channel(u, &ca, &A));
    ESP_ERROR_CHECK(pcnt_new_channel(u, &cb, &B));
    pcnt_channel_set_edge_action(A, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(A, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(B, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(B, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(u, 30000));    // accum_count needs the limits watched
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(u, -30000));
    ESP_ERROR_CHECK(pcnt_unit_enable(u));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(u));
    ESP_ERROR_CHECK(pcnt_unit_start(u));
    return u;
}

static double wheel_control(wheel_t *w, double dt) {
    int raw = 0;
    pcnt_unit_get_count(w->pcnt, &raw);
    int delta = (raw - w->last_count) * w->count_sign;
    w->last_count = raw;
    double dist = delta * M_PER_COUNT;
    w->speed += SPEED_FILTER * (dist / dt - w->speed);

    if (w->target == 0.0) {
        w->integral = 0;
        w->duty = 0;
        w->brake = fabs(w->speed) > BRAKE_ABOVE_MPS;
    } else {
        w->brake = false;
        double err = w->target - w->speed;
        double ff = copysign(DUTY_START + DUTY_PER_MPS * fabs(w->target), w->target);
        double out = ff + KP * err + KI * (w->integral + err * dt);
        if (fabs(out) < 1.0 || (out > 0) != (err > 0)) w->integral += err * dt;   // anti-windup
        w->duty = fmax(-1.0, fmin(1.0, ff + KP * err + KI * w->integral));
    }
    wheel_output(w, w->duty);
    return dist;
}

static void control_task(void *arg) {
    TickType_t wake = xTaskGetTickCount();
    const double dt = 1.0 / CONTROL_HZ;
    double x = 0, y = 0, th = 0;
    for (;;) {
        int64_t now = esp_timer_get_time();
        double v, w;
        taskENTER_CRITICAL(&s_lock);
        bool fresh = !s_inhibit && (now - s_cmd_time) < CMD_TIMEOUT_MS * 1000LL;
        v = fresh ? s_cmd_v : 0;
        w = fresh ? s_cmd_w : 0;
        taskEXIT_CRITICAL(&s_lock);

        s_wheel[0].target = v - w * TRACK_M / 2;
        s_wheel[1].target = v + w * TRACK_M / 2;
        double dl = wheel_control(&s_wheel[0], dt);
        double dr = wheel_control(&s_wheel[1], dt);

        double d = (dl + dr) / 2, dth = (dr - dl) / TRACK_M;
        x += d * cos(th + dth / 2);
        y += d * sin(th + dth / 2);
        th = atan2(sin(th + dth), cos(th + dth));

        drive_state_t st = {
            .x = x, .y = y, .heading = th,
            .wheel_l = s_wheel[0].speed, .wheel_r = s_wheel[1].speed,
            .v = (s_wheel[0].speed + s_wheel[1].speed) / 2,
            .w = (s_wheel[1].speed - s_wheel[0].speed) / TRACK_M,
            .duty_l = s_wheel[0].duty, .duty_r = s_wheel[1].duty,
        };
        taskENTER_CRITICAL(&s_lock);
        s_state = st;
        taskEXIT_CRITICAL(&s_lock);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / CONTROL_HZ));
    }
}

void drive_start(void) {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = PWM_BITS,
                             .timer_num = LEDC_TIMER_0, .freq_hz = PWM_HZ, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    for (int i = 0; i < 2; i++) {
        wheel_t *w = &s_wheel[i];
        const int pins[2] = {w->pin_a, w->pin_b}, chs[2] = {w->ch_a, w->ch_b};
        for (int k = 0; k < 2; k++) {
            ledc_channel_config_t c = {.gpio_num = pins[k], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = chs[k],
                                       .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0};
            ESP_ERROR_CHECK(ledc_channel_config(&c));
        }
        w->pcnt = encoder_new(ENC_PINS[i][0], ENC_PINS[i][1]);
    }
    xTaskCreatePinnedToCore(control_task, "drive", 4096, NULL, 10, NULL, 1);
    ESP_LOGI(TAG, "%.3f mm per count, track %.1f mm, %d Hz control", M_PER_COUNT * 1000, TRACK_M * 1000, CONTROL_HZ);
}

void drive_command(double v, double w) {
    // Scale both down together if either wheel would exceed the limit, keeping the curvature.
    double l = fabs(v - w * TRACK_M / 2), r = fabs(v + w * TRACK_M / 2), m = fmax(l, r);
    if (m > MAX_WHEEL_SPEED) { v *= MAX_WHEEL_SPEED / m; w *= MAX_WHEEL_SPEED / m; }
    taskENTER_CRITICAL(&s_lock);
    s_cmd_v = v;
    s_cmd_w = w;
    s_cmd_time = esp_timer_get_time();
    taskEXIT_CRITICAL(&s_lock);
}

void drive_inhibit(bool on) { s_inhibit = on; }

drive_state_t drive_state(void) {
    taskENTER_CRITICAL(&s_lock);
    drive_state_t st = s_state;
    taskEXIT_CRITICAL(&s_lock);
    return st;
}
