// lidarbot_base: open firmware for the Yahboom MicroROS V2 control board (ESP32-S3).
//
// Connects to the micro-ROS agent over Wi-Fi with the settings Yahboom's firmware saved (network,
// agent IP/port, ROS domain). Same topics and frames as Yahboom's firmware, plus battery current:
//   publishes   /imu            sensor_msgs/Imu           50 Hz  imu_frame, x forward, y left, z up
//               /odom_raw       nav_msgs/Odometry         25 Hz  odom -> base_footprint, from the encoders
//               /scan           sensor_msgs/LaserScan     ~6 Hz  laser_frame, 667 points, 0 = no echo
//               /ptz_angle      geometry_msgs/Vector3     10 Hz  pan (x), tilt (y) in degrees, as commanded
//               /battery        std_msgs/Float32           5 Hz  pack volts
//               /power          sensor_msgs/BatteryState   5 Hz  INA228: voltage, current, temperature
//               /power_used_wh  std_msgs/Float32           5 Hz  energy drawn from the pack since boot
//   subscribes  /cmd_vel        geometry_msgs/Twist              stops after 0.5 s without a command
//               /cmd_ptz_angle  geometry_msgs/Vector3            pan -90..90, tilt -90..20 degrees
//               /beep           std_msgs/UInt16                  milliseconds
// OLED: volts and amps on top, connection state below. LEDs: blue blinks while looking for the
// agent, green while connected. One beep when the agent connects. Keep the robot still for the
// first 2 s after power-on (gyro bias).
// Serial console (115200): "batt sim <volts>" / "batt sim off" tests the low-battery guard.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "drive.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ina228.h"
#include "lidar.h"
#include "net.h"
#include "nvs_flash.h"
#include "oled.h"
#include "qmi8658.h"

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/battery_state.h>
#include "battery_guard.h"
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int16.h>

static const char *TAG = "base";

#define SHUNT_OHMS 0.002
#define IMU_HZ 200
#define LOOP_MS 20             // ROS loop: /imu every loop (50 Hz)
#define ODOM_DIV 2             // /odom_raw every 2nd loop (25 Hz)
#define PTZ_DIV 5              // /ptz_angle every 5th loop (10 Hz)
#define POWER_PERIOD_MS 200    // 5 Hz
#define RESYNC_PERIOD_US (60LL * 1000000)
#define SCAN_TIMEOUT_MS 200    // don't let a reliable /scan publish stall the loop
#define TILT_MAX 20.0          // Yahboom's limit for S2

// ---------- shared sensor state (written by the sensor task, read by the ROS task) ----------
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static imu_state_t s_imu;
static bool s_imu_ok, s_imu_fresh;
static ina228_reading_t s_power;
static bool s_power_ok;
static double s_adc_v;
static bool s_has_ina;
static volatile bool s_ros_up;
static uint32_t s_pub_fail;   // failed publishes, logged with the status
static volatile batt_state_t s_batt_state = BATT_NO_PACK;   // for the OLED and log
static double s_pan = 0, s_tilt = -40;      // -40 = camera level (measured 2026-11-12)

// ---------- outputs ----------
static void out_low(int pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}

static void servo_set(int ch, double deg) {   // 50 Hz, 14-bit; -90..90 deg = 500..2500 us
    double us = 1500.0 + deg / 90.0 * 1000.0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, (uint32_t)(us / 20000.0 * 16383));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void servos_start(void) {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_14_BIT,
                             .timer_num = LEDC_TIMER_1, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    const int pins[2] = {PIN_SERVO_PAN, PIN_SERVO_TILT};
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t c = {.gpio_num = pins[i], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = 4 + i,
                                   .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
    servo_set(4, s_pan);
    servo_set(5, s_tilt);
}

static esp_timer_handle_t s_beep_timer;
static void beep_off(void *arg) { ledc_stop(LEDC_LOW_SPEED_MODE, 6, 0); }

static void beep_start(void) {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
                             .timer_num = LEDC_TIMER_2, .freq_hz = 2700, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ledc_channel_config_t c = {.gpio_num = PIN_BUZZER, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = 6,
                               .timer_sel = LEDC_TIMER_2, .duty = 0, .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&c));
    esp_timer_create_args_t a = {.callback = beep_off, .name = "beep"};
    ESP_ERROR_CHECK(esp_timer_create(&a, &s_beep_timer));
}

static void beep(int ms) {   // non-blocking: a timer switches it off
    if (ms <= 0) return;
    if (ms > 5000) ms = 5000;
    esp_timer_stop(s_beep_timer);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, 6, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, 6);
    esp_timer_start_once(s_beep_timer, ms * 1000LL);
}

// ---------- sensor task: IMU at 200 Hz, power every 200 ms ----------
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cal;

static double battery_adc(void) {
    int raw = 0, mv = 0;
    if (adc_oneshot_read(s_adc, ADC_CHANNEL_2, &raw) != ESP_OK) return NAN;
    if (s_cal) adc_cali_raw_to_voltage(s_cal, raw, &mv);
    return mv / 1000.0 * 4.03;   // divider from Yahboom's sample, checked against the INA228
}

// Low-battery guard (battery_guard.c): beeps, and below the critical level stops the lidar motor
// and the wheels. Runs here, not in the ROS task, so it works without Wi-Fi or the agent.
static void battery_check(batt_guard_t *g, double volts, double amps, bool *loads_stopped) {
    batt_state_t before = g->state;
    batt_guard_update(g, volts, amps);
    s_batt_state = g->state;
    if (g->state != before)
        ESP_LOGW(TAG, "battery %s -> %s (%.2f V at %.2f A, %.2f V resting)", batt_state_name(before),
                 batt_state_name(g->state), volts, amps, g->rest_v);
    if (g->beep) beep(g->beep_ms);
    if (g->stop_loads != *loads_stopped) {
        *loads_stopped = g->stop_loads;
        drive_inhibit(g->stop_loads);
        lidar_motor(!g->stop_loads);
    }
}

// ---------- battery test mode: "batt sim <volts>" / "batt sim off" on the serial console ----------
static portMUX_TYPE s_sim_lock = portMUX_INITIALIZER_UNLOCKED;
static double s_sim_v;
static int64_t s_sim_until_us;   // 0: off

static bool sim_volts(double *v) {
    bool on;
    taskENTER_CRITICAL(&s_sim_lock);
    on = s_sim_until_us != 0 && esp_timer_get_time() < s_sim_until_us;
    if (on) *v = s_sim_v;
    taskEXIT_CRITICAL(&s_sim_lock);
    return on;
}

static void sim_set(bool on, double v) {
    taskENTER_CRITICAL(&s_sim_lock);
    s_sim_v = v;
    s_sim_until_us = on ? esp_timer_get_time() + (int64_t)BATT_SIM_MAX_S * 1000000 : 0;
    taskEXIT_CRITICAL(&s_sim_lock);
    if (on) ESP_LOGW(TAG, "battery test mode: %.2f V for up to %d s", v, BATT_SIM_MAX_S);
    else ESP_LOGW(TAG, "battery test mode off");
}

static void console_task(void *arg) {
    char line[48];
    size_t n = 0;
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) != 1) continue;
        if (c != '\n' && c != '\r') {
            if (n < sizeof line - 1) line[n++] = (char)c;
            else n = sizeof line;               // too long: drop the whole line
            continue;
        }
        if (n > 0 && n < sizeof line) {
            line[n] = 0;
            double v;
            batt_cmd_t cmd = batt_parse_cmd(line, &v);
            if (cmd == BATT_CMD_SIM) sim_set(true, v);
            else if (cmd == BATT_CMD_SIM_OFF) sim_set(false, 0);
        }
        n = 0;
    }
}

static void console_start(void) {
    // RX only: logging keeps writing to UART0 directly. A failure just means no test mode.
    esp_err_t err = uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    if (err != ESP_OK) { ESP_LOGW(TAG, "console input off: %s", esp_err_to_name(err)); return; }
    xTaskCreatePinnedToCore(console_task, "console", 3072, NULL, 1, NULL, 1);
}

static void sensor_task(void *arg) {
    bool imu = (bool)(intptr_t)arg;
    TickType_t last = xTaskGetTickCount();
    int64_t prev = esp_timer_get_time(), next_power = 0;
    batt_guard_t batt;
    batt_guard_init(&batt);
    bool loads_stopped = false;
    for (;;) {
        int64_t now = esp_timer_get_time();
        if (imu) {
            imu_state_t st;
            bool ok = qmi8658_update((now - prev) / 1e6, &st);
            taskENTER_CRITICAL(&s_lock);
            if (ok) { s_imu = st; s_imu_fresh = true; }
            s_imu_ok = ok;
            taskEXIT_CRITICAL(&s_lock);
        }
        prev = now;
        if (now >= next_power) {
            next_power = now + POWER_PERIOD_MS * 1000;
            ina228_reading_t p;
            bool ok = s_has_ina && ina228_read(&p);
            double adc = battery_adc();
            taskENTER_CRITICAL(&s_lock);
            if (ok) s_power = p;
            s_power_ok = ok;
            s_adc_v = adc;
            taskEXIT_CRITICAL(&s_lock);
            double sim;
            if (sim_volts(&sim)) battery_check(&batt, sim, 0.0, &loads_stopped);
            else battery_check(&batt, ok ? p.bus_v : adc, ok ? p.current_a : 0.0, &loads_stopped);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / IMU_HZ));
    }
}

// ---------- status task: OLED, LEDs, a log line every 10 s ----------
static uint32_t s_scans;
static lidar_scan_t s_scan_status;   // last scan's counters, for the log

static void status_task(void *arg) {
    char top[24], bottom[24];
    for (int i = 0;; i++) {
        ina228_reading_t p;
        bool pok;
        double adc;
        taskENTER_CRITICAL(&s_lock);
        p = s_power; pok = s_power_ok; adc = s_adc_v;
        taskEXIT_CRITICAL(&s_lock);
        double sim;
        if (sim_volts(&sim)) snprintf(top, sizeof top, "SIM %.2fV", sim);
        else if (pok) snprintf(top, sizeof top, "%.2fV %.2fA", p.bus_v, p.current_a);
        else snprintf(top, sizeof top, "%.2fV", adc);
        if (s_batt_state == BATT_CRITICAL) snprintf(bottom, sizeof bottom, "BATT CRITICAL");
        else if (s_batt_state == BATT_LOW) snprintf(bottom, sizeof bottom, "BATT LOW");
        else if (s_ros_up) snprintf(bottom, sizeof bottom, "ROS OK %.1fW", pok ? p.power_w : 0.0);
        else if (net_wifi_up()) snprintf(bottom, sizeof bottom, "AGENT...");
        else snprintf(bottom, sizeof bottom, "WIFI...");
        oled_lines(top, bottom);
        if (i % 20 == 0) {
            drive_state_t d = drive_state();
            ESP_LOGI(TAG, "%s | %s | wifi %d dBm | ros %s, %lu failed publishes | wheels %.2f %.2f m/s | "
                     "lidar %.1f Hz, %lu scans, %lu/%lu bad packets",
                     top, bottom, net_wifi_rssi(), s_ros_up ? "up" : "down", (unsigned long)s_pub_fail,
                     d.wheel_l, d.wheel_r, s_scan_status.scan_hz, (unsigned long)s_scans,
                     (unsigned long)s_scan_status.bad_packets,
                     (unsigned long)(s_scan_status.good_packets + s_scan_status.bad_packets));
        }
        gpio_set_level(PIN_LED_GREEN, s_ros_up);
        gpio_set_level(PIN_LED_BLUE, !s_ros_up && (i & 1));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------- micro-ROS ----------
#define DISCARD(x) do { rcl_ret_t discarded_ = (x); (void)discarded_; } while (0)
#define CHECK(x) do { rcl_ret_t rc = (x); if (rc != RCL_RET_OK) { ESP_LOGW(TAG, "%s failed: %d", #x, (int)rc); return false; } } while (0)
#define STR(field, buf) do { (field).data = (buf); (field).size = strlen(buf); (field).capacity = sizeof(buf); } while (0)

static rcl_allocator_t s_alloc;
static rcl_init_options_t s_init_opts;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rclc_executor_t s_exec;
static rcl_publisher_t s_pub_imu, s_pub_battery, s_pub_power, s_pub_used, s_pub_odom, s_pub_scan, s_pub_ptz;
static rcl_subscription_t s_sub_cmd, s_sub_ptz, s_sub_beep;
static sensor_msgs__msg__Imu s_imu_msg;
static sensor_msgs__msg__BatteryState s_power_msg;
static nav_msgs__msg__Odometry s_odom_msg;
static sensor_msgs__msg__LaserScan s_scan_msg;
static std_msgs__msg__Float32 s_f32;
static geometry_msgs__msg__Vector3 s_ptz_msg;
static geometry_msgs__msg__Twist s_cmd_in;
static geometry_msgs__msg__Vector3 s_ptz_in;
static std_msgs__msg__UInt16 s_beep_in;
static float s_ranges[SCAN_POINTS];
static lidar_scan_t s_scan;
static uint32_t s_scan_seq;
static char s_imu_frame[] = "imu_frame";
static char s_odom_frame[] = "odom";
static char s_base_frame[] = "base_footprint";
static char s_laser_frame[] = "laser_frame";
static char s_location[] = "main board INA228, battery output";

static void on_cmd_vel(const void *m) {
    const geometry_msgs__msg__Twist *t = m;
    drive_command(t->linear.x, t->angular.z);
}

static void on_ptz(const void *m) {
    const geometry_msgs__msg__Vector3 *v = m;
    s_pan = fmax(-90.0, fmin(90.0, v->x));
    s_tilt = fmax(-90.0, fmin(TILT_MAX, v->y));
    servo_set(4, s_pan);
    servo_set(5, s_tilt);
}

static void on_beep(const void *m) { beep(((const std_msgs__msg__UInt16 *)m)->data); }

static bool ros_create(void) {
    memset(&s_support, 0, sizeof s_support);
    s_node = rcl_get_zero_initialized_node();
    rcl_publisher_t *pubs[] = {&s_pub_imu, &s_pub_battery, &s_pub_power, &s_pub_used, &s_pub_odom, &s_pub_scan, &s_pub_ptz};
    for (size_t i = 0; i < sizeof pubs / sizeof pubs[0]; i++) *pubs[i] = rcl_get_zero_initialized_publisher();
    s_sub_cmd = rcl_get_zero_initialized_subscription();
    s_sub_ptz = rcl_get_zero_initialized_subscription();
    s_sub_beep = rcl_get_zero_initialized_subscription();
    s_exec = rclc_executor_get_zero_initialized_executor();

    CHECK(rclc_support_init_with_options(&s_support, 0, NULL, &s_init_opts, &s_alloc));
    CHECK(rclc_node_init_default(&s_node, "lidarbot_base", "", &s_support));
    CHECK(rclc_publisher_init_best_effort(&s_pub_imu, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));
    CHECK(rclc_publisher_init_best_effort(&s_pub_odom, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom_raw"));
    // A scan (about 2.7 KB) is bigger than one packet; only reliable streams can split it up.
    CHECK(rclc_publisher_init_default(&s_pub_scan, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));
    if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&s_pub_scan), SCAN_TIMEOUT_MS) != RMW_RET_OK)
        ESP_LOGW(TAG, "could not set the /scan publish timeout");
    CHECK(rclc_publisher_init_best_effort(&s_pub_ptz, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "ptz_angle"));
    CHECK(rclc_publisher_init_best_effort(&s_pub_battery, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "battery"));
    CHECK(rclc_publisher_init_best_effort(&s_pub_power, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "power"));
    CHECK(rclc_publisher_init_best_effort(&s_pub_used, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "power_used_wh"));
    // Best-effort subscriptions match both reliable and best-effort publishers.
    CHECK(rclc_subscription_init_best_effort(&s_sub_cmd, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    CHECK(rclc_subscription_init_best_effort(&s_sub_ptz, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "cmd_ptz_angle"));
    CHECK(rclc_subscription_init_best_effort(&s_sub_beep, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "beep"));
    CHECK(rclc_executor_init(&s_exec, &s_support.context, 3, &s_alloc));
    CHECK(rclc_executor_add_subscription(&s_exec, &s_sub_cmd, &s_cmd_in, on_cmd_vel, ON_NEW_DATA));
    CHECK(rclc_executor_add_subscription(&s_exec, &s_sub_ptz, &s_ptz_in, on_ptz, ON_NEW_DATA));
    CHECK(rclc_executor_add_subscription(&s_exec, &s_sub_beep, &s_beep_in, on_beep, ON_NEW_DATA));
    if (rmw_uros_sync_session(1000) != RMW_RET_OK) ESP_LOGW(TAG, "time sync failed; stamps will be off until the next try");
    return true;
}

static void ros_destroy(void) {
    drive_command(0, 0);
    if (!rcl_context_is_valid(&s_support.context)) return;   // init never got that far
    rmw_context_t *ctx = rcl_context_get_rmw_context(&s_support.context);
    if (ctx) (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
    DISCARD(rclc_executor_fini(&s_exec));
    DISCARD(rcl_subscription_fini(&s_sub_cmd, &s_node));
    DISCARD(rcl_subscription_fini(&s_sub_ptz, &s_node));
    DISCARD(rcl_subscription_fini(&s_sub_beep, &s_node));
    rcl_publisher_t *pubs[] = {&s_pub_imu, &s_pub_battery, &s_pub_power, &s_pub_used, &s_pub_odom, &s_pub_scan, &s_pub_ptz};
    for (size_t i = 0; i < sizeof pubs / sizeof pubs[0]; i++) DISCARD(rcl_publisher_fini(pubs[i], &s_node));
    DISCARD(rcl_node_fini(&s_node));
    DISCARD(rclc_support_fini(&s_support));
}

static void stamp(builtin_interfaces__msg__Time *t) {
    int64_t ns = rmw_uros_epoch_nanos();
    t->sec = (int32_t)(ns / 1000000000LL);
    t->nanosec = (uint32_t)(ns % 1000000000LL);
}

static void msgs_init(void) {
    memset(&s_imu_msg, 0, sizeof s_imu_msg);
    STR(s_imu_msg.header.frame_id, s_imu_frame);
    const double ori[3] = {0.0025, 0.0025, 0.01}, gyr = 2.5e-5, acc = 0.01;
    for (int k = 0; k < 3; k++) {
        s_imu_msg.orientation_covariance[k * 4] = ori[k];
        s_imu_msg.angular_velocity_covariance[k * 4] = gyr;
        s_imu_msg.linear_acceleration_covariance[k * 4] = acc;
    }

    memset(&s_odom_msg, 0, sizeof s_odom_msg);
    STR(s_odom_msg.header.frame_id, s_odom_frame);
    STR(s_odom_msg.child_frame_id, s_base_frame);
    // Same covariances as Yahboom's firmware (read from a recorded /odom_raw).
    s_odom_msg.pose.covariance[0] = s_odom_msg.pose.covariance[7] = s_odom_msg.pose.covariance[35] = 0.001;
    s_odom_msg.twist.covariance[0] = s_odom_msg.twist.covariance[7] = s_odom_msg.twist.covariance[35] = 0.0001;

    memset(&s_scan_msg, 0, sizeof s_scan_msg);
    STR(s_scan_msg.header.frame_id, s_laser_frame);
    s_scan_msg.angle_increment = 2.0f * (float)M_PI / SCAN_POINTS;
    s_scan_msg.angle_min = -(float)M_PI;
    s_scan_msg.angle_max = s_scan_msg.angle_min + (SCAN_POINTS - 1) * s_scan_msg.angle_increment;
    s_scan_msg.range_min = SCAN_RANGE_MIN;
    s_scan_msg.range_max = SCAN_RANGE_MAX;
    s_scan_msg.ranges.data = s_ranges;
    s_scan_msg.ranges.size = s_scan_msg.ranges.capacity = SCAN_POINTS;

    memset(&s_power_msg, 0, sizeof s_power_msg);
    STR(s_power_msg.location, s_location);
    s_power_msg.design_capacity = 2.2f;   // label: 12 V 2200 mAh, 3 cells
    s_power_msg.charge = NAN;
    s_power_msg.capacity = NAN;
    s_power_msg.percentage = NAN;         // tools/battery_monitor.py estimates it from the curve
    s_power_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    s_power_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
    s_power_msg.present = true;
}

static void publish_imu(void) {
    imu_state_t imu;
    bool fresh;
    taskENTER_CRITICAL(&s_lock);
    imu = s_imu; fresh = s_imu_fresh; s_imu_fresh = false;
    taskEXIT_CRITICAL(&s_lock);
    if (!fresh) return;
    stamp(&s_imu_msg.header.stamp);
    s_imu_msg.orientation.x = imu.q[0]; s_imu_msg.orientation.y = imu.q[1];
    s_imu_msg.orientation.z = imu.q[2]; s_imu_msg.orientation.w = imu.q[3];
    s_imu_msg.angular_velocity.x = imu.gyro[0]; s_imu_msg.angular_velocity.y = imu.gyro[1];
    s_imu_msg.angular_velocity.z = imu.gyro[2];
    s_imu_msg.linear_acceleration.x = imu.accel[0]; s_imu_msg.linear_acceleration.y = imu.accel[1];
    s_imu_msg.linear_acceleration.z = imu.accel[2];
    if (rcl_publish(&s_pub_imu, &s_imu_msg, NULL) != RCL_RET_OK) s_pub_fail++;
}

static void publish_odom(void) {
    drive_state_t d = drive_state();
    stamp(&s_odom_msg.header.stamp);
    s_odom_msg.pose.pose.position.x = d.x;
    s_odom_msg.pose.pose.position.y = d.y;
    s_odom_msg.pose.pose.orientation.z = sin(d.heading / 2);
    s_odom_msg.pose.pose.orientation.w = cos(d.heading / 2);
    s_odom_msg.twist.twist.linear.x = d.v;
    s_odom_msg.twist.twist.angular.z = d.w;
    if (rcl_publish(&s_pub_odom, &s_odom_msg, NULL) != RCL_RET_OK) s_pub_fail++;
}

static void publish_scan(void) {
    if (!lidar_take(&s_scan, &s_scan_seq)) return;
    s_scans++;
    s_scan_status = s_scan;
    memcpy(s_ranges, s_scan.ranges, sizeof s_ranges);
    stamp(&s_scan_msg.header.stamp);
    s_scan_msg.scan_time = s_scan.scan_hz > 0 ? 1.0f / s_scan.scan_hz : 0;
    if (rcl_publish(&s_pub_scan, &s_scan_msg, NULL) != RCL_RET_OK) s_pub_fail++;
}

static void publish_power(void) {
    ina228_reading_t p;
    bool pok;
    double adc;
    taskENTER_CRITICAL(&s_lock);
    p = s_power; pok = s_power_ok; adc = s_adc_v;
    taskEXIT_CRITICAL(&s_lock);
    s_f32.data = pok ? p.bus_v : adc;
    if (rcl_publish(&s_pub_battery, &s_f32, NULL) != RCL_RET_OK) s_pub_fail++;
    if (!pok) return;
    stamp(&s_power_msg.header.stamp);
    s_power_msg.voltage = p.bus_v;
    s_power_msg.current = -p.current_a;   // BatteryState: negative while discharging
    s_power_msg.temperature = p.die_c;    // the INA228's die, next to the battery lead
    if (rcl_publish(&s_pub_power, &s_power_msg, NULL) != RCL_RET_OK) s_pub_fail++;
    s_f32.data = p.energy_wh;
    if (rcl_publish(&s_pub_used, &s_f32, NULL) != RCL_RET_OK) s_pub_fail++;
}

static void ros_task(void *arg) {
    net_config_t cfg;
    net_load_config(&cfg);
    s_alloc = rcl_get_default_allocator();
    s_init_opts = rcl_get_zero_initialized_init_options();
    ESP_ERROR_CHECK(rcl_init_options_init(&s_init_opts, s_alloc));
    ESP_ERROR_CHECK(rcl_init_options_set_domain_id(&s_init_opts, cfg.domain_id));
    rmw_init_options_t *rmw_opts = rcl_init_options_get_rmw_init_options(&s_init_opts);
    ESP_ERROR_CHECK(rmw_uros_options_set_udp_address(cfg.agent_ip, cfg.agent_port, rmw_opts));
    msgs_init();

    int64_t next_ping = 0, next_power = 0, next_sync = 0;
    TickType_t wake = xTaskGetTickCount();
    for (uint32_t tick = 0;; tick++) {
        int64_t now = esp_timer_get_time();
        if (!s_ros_up) {
            if (net_wifi_up() && rmw_uros_ping_agent_options(200, 1, rmw_opts) == RMW_RET_OK) {
                if (ros_create()) {
                    s_ros_up = true;
                    next_ping = now + 1000000;
                    next_sync = now + RESYNC_PERIOD_US;
                    ESP_LOGI(TAG, "connected to agent %s:%s", cfg.agent_ip, cfg.agent_port);
                    beep(80);
                    wake = xTaskGetTickCount();
                } else {
                    ros_destroy();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (now >= next_ping) {
            next_ping = now + 1000000;
            if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                ESP_LOGW(TAG, "agent lost");
                s_ros_up = false;
                ros_destroy();
                continue;
            }
        }
        if (now >= next_sync) {
            next_sync = now + RESYNC_PERIOD_US;
            if (rmw_uros_sync_session(200) != RMW_RET_OK) ESP_LOGW(TAG, "time resync failed");
        }
        DISCARD(rclc_executor_spin_some(&s_exec, 0));
        publish_imu();
        if (tick % ODOM_DIV == 0) publish_odom();
        if (tick % PTZ_DIV == 0) {
            s_ptz_msg.x = s_pan;
            s_ptz_msg.y = s_tilt;
            if (rcl_publish(&s_pub_ptz, &s_ptz_msg, NULL) != RCL_RET_OK) s_pub_fail++;
        }
        if (now >= next_power) {
            next_power = now + POWER_PERIOD_MS * 1000;
            publish_power();
        }
        publish_scan();
        // Fixed schedule (not a fixed sleep after the work), so /imu holds 50 Hz.
        if (!xTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS))) wake = xTaskGetTickCount();
    }
}

void app_main(void) {
    // Motors off, buzzer quiet, LEDs off, before anything else.
    out_low(PIN_M1A); out_low(PIN_M1B); out_low(PIN_M2A); out_low(PIN_M2B);
    out_low(PIN_BUZZER); out_low(PIN_LED_BLUE); out_low(PIN_LED_GREEN);
    servos_start();
    beep_start();
    ESP_LOGI(TAG, "lidarbot_base phase 2");

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) ESP_LOGE(TAG, "nvs init failed: %s (not erasing: it holds the settings)", esp_err_to_name(err));
    net_wifi_start();

    i2c_master_bus_handle_t imu_bus, aux_bus;
    i2c_master_bus_config_t bc = {.i2c_port = I2C_NUM_0, .sda_io_num = PIN_IMU_SDA, .scl_io_num = PIN_IMU_SCL,
                                  .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
                                  .flags.enable_internal_pullup = true};
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &imu_bus));
    bc.i2c_port = I2C_NUM_1; bc.sda_io_num = PIN_AUX_SDA; bc.scl_io_num = PIN_AUX_SCL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &aux_bus));

    bool oled = oled_init(aux_bus);
    oled_lines("LIDARBOT", "STARTING");
    bool ina = s_has_ina = ina228_init(aux_bus, 0x40, SHUNT_OHMS);
    bool imu = qmi8658_init(imu_bus) && qmi8658_calibrate(2000);   // robot must be still
    ESP_LOGI(TAG, "oled %s, ina228 %s, imu %s", oled ? "ok" : "missing", ina ? "ok" : "missing", imu ? "ok" : "missing");

    adc_oneshot_unit_init_cfg_t u = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&u, &s_adc));
    adc_oneshot_chan_cfg_t ch = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_2, &ch));
    adc_cali_curve_fitting_config_t cc = {.unit_id = ADC_UNIT_1, .chan = ADC_CHANNEL_2,
                                          .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    if (adc_cali_create_scheme_curve_fitting(&cc, &s_cal) != ESP_OK) s_cal = NULL;

    drive_start();
    lidar_start();
    xTaskCreatePinnedToCore(sensor_task, "sensors", 4096, (void *)(intptr_t)imu, 6, NULL, 1);
    xTaskCreatePinnedToCore(status_task, "status", 4096, NULL, 2, NULL, 1);
    console_start();
    xTaskCreatePinnedToCore(ros_task, "ros", 16384, NULL, 5, NULL, 0);
}
