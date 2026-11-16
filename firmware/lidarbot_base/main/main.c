// lidarbot_base: open firmware for the Yahboom MicroROS V2 control board (ESP32-S3).
//
// Phase 1: sensors only. Connects to the micro-ROS agent over Wi-Fi with the settings Yahboom's
// firmware saved (network, agent IP/port, ROS domain) and publishes
//   /imu            sensor_msgs/Imu           50 Hz  robot frame (x forward, y left, z up), imu_frame
//   /battery        std_msgs/Float32           5 Hz  pack volts (same topic as Yahboom's firmware)
//   /power          sensor_msgs/BatteryState   5 Hz  INA228: voltage, current, die temperature
//   /power_used_wh  std_msgs/Float32           5 Hz  energy drawn from the pack since boot
// Motors are held off; the pan/tilt servos hold the power-on position (pan 0, tilt -45).
// OLED: volts and amps on top, connection state below. LEDs: blue blinks while looking for the
// agent, green while connected. One beep when the agent connects.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ina228.h"
#include "net.h"
#include "nvs_flash.h"
#include "oled.h"
#include "qmi8658.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>

static const char *TAG = "base";

#define SHUNT_OHMS 0.002
#define IMU_HZ 200
#define IMU_PUB_DIV 4          // publish every 4th sample: 50 Hz
#define POWER_PERIOD_MS 200    // 5 Hz
#define RESYNC_PERIOD_US (60LL * 1000000)

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

// ---------- outputs held safe ----------
static void out_low(int pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}

static void servo_hold(void) {   // 50 Hz, 14-bit; -90..90 deg = 500..2500 us
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_14_BIT,
                             .timer_num = LEDC_TIMER_1, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    const struct { int pin, ch; double deg; } s[] = {{PIN_SERVO_PAN, 4, 0}, {PIN_SERVO_TILT, 5, -45}};
    for (int i = 0; i < 2; i++) {
        double us = 1500.0 + s[i].deg / 90.0 * 1000.0;
        ledc_channel_config_t c = {.gpio_num = s[i].pin, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = s[i].ch,
                                   .timer_sel = LEDC_TIMER_1, .duty = (uint32_t)(us / 20000.0 * 16383), .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
}

static void beep(int ms) {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
                             .timer_num = LEDC_TIMER_2, .freq_hz = 2700, .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&t);
    ledc_channel_config_t c = {.gpio_num = PIN_BUZZER, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = 6,
                               .timer_sel = LEDC_TIMER_2, .duty = 512, .hpoint = 0};
    ledc_channel_config(&c);
    vTaskDelay(pdMS_TO_TICKS(ms));
    ledc_stop(LEDC_LOW_SPEED_MODE, 6, 0);
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

static void sensor_task(void *arg) {
    bool imu = (bool)(intptr_t)arg;
    TickType_t last = xTaskGetTickCount();
    int64_t prev = esp_timer_get_time(), next_power = 0;
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
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / IMU_HZ));
    }
}

// ---------- status task: OLED and LEDs ----------
static void status_task(void *arg) {
    char top[24], bottom[24];
    for (int i = 0;; i++) {
        ina228_reading_t p;
        bool pok;
        double adc;
        taskENTER_CRITICAL(&s_lock);
        p = s_power; pok = s_power_ok; adc = s_adc_v;
        taskEXIT_CRITICAL(&s_lock);
        if (pok) snprintf(top, sizeof top, "%.2fV %.2fA", p.bus_v, p.current_a);
        else snprintf(top, sizeof top, "%.2fV", adc);
        if (s_ros_up) snprintf(bottom, sizeof bottom, "ROS OK %.1fW", pok ? p.power_w : 0.0);
        else if (net_wifi_up()) snprintf(bottom, sizeof bottom, "AGENT...");
        else snprintf(bottom, sizeof bottom, "WIFI...");
        oled_lines(top, bottom);
        if (i % 20 == 0)
            ESP_LOGI(TAG, "%s | %s | wifi %d dBm | ros %s, %lu failed publishes", top, bottom, net_wifi_rssi(),
                     s_ros_up ? "up" : "down", (unsigned long)s_pub_fail);
        gpio_set_level(PIN_LED_GREEN, s_ros_up);
        gpio_set_level(PIN_LED_BLUE, !s_ros_up && (i & 1));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------- micro-ROS ----------
#define DISCARD(x) do { rcl_ret_t discarded_ = (x); (void)discarded_; } while (0)
#define CHECK(x) do { rcl_ret_t rc = (x); if (rc != RCL_RET_OK) { ESP_LOGW(TAG, "%s failed: %d", #x, (int)rc); return false; } } while (0)

static rcl_allocator_t s_alloc;
static rcl_init_options_t s_init_opts;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rcl_publisher_t s_pub_imu, s_pub_battery, s_pub_power, s_pub_used;
static sensor_msgs__msg__Imu s_imu_msg;
static sensor_msgs__msg__BatteryState s_power_msg;
static std_msgs__msg__Float32 s_f32;
static char s_imu_frame[] = "imu_frame";
static char s_location[] = "main board INA228, battery output";

static bool ros_create(void) {
    memset(&s_support, 0, sizeof s_support);
    s_node = rcl_get_zero_initialized_node();
    s_pub_imu = rcl_get_zero_initialized_publisher();
    s_pub_battery = rcl_get_zero_initialized_publisher();
    s_pub_power = rcl_get_zero_initialized_publisher();
    s_pub_used = rcl_get_zero_initialized_publisher();
    CHECK(rclc_support_init_with_options(&s_support, 0, NULL, &s_init_opts, &s_alloc));
    CHECK(rclc_node_init_default(&s_node, "lidarbot_base", "", &s_support));
    CHECK(rclc_publisher_init_best_effort(&s_pub_imu, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));
    CHECK(rclc_publisher_init_best_effort(&s_pub_battery, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "battery"));
    CHECK(rclc_publisher_init_best_effort(&s_pub_power, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "power"));
    CHECK(rclc_publisher_init_best_effort(&s_pub_used, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "power_used_wh"));
    if (rmw_uros_sync_session(1000) != RMW_RET_OK) ESP_LOGW(TAG, "time sync failed; stamps will be off until the next try");
    return true;
}

static void ros_destroy(void) {
    if (!rcl_context_is_valid(&s_support.context)) return;   // init never got that far
    rmw_context_t *ctx = rcl_context_get_rmw_context(&s_support.context);
    if (ctx) (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
    DISCARD(rcl_publisher_fini(&s_pub_imu, &s_node));
    DISCARD(rcl_publisher_fini(&s_pub_battery, &s_node));
    DISCARD(rcl_publisher_fini(&s_pub_power, &s_node));
    DISCARD(rcl_publisher_fini(&s_pub_used, &s_node));
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
    s_imu_msg.header.frame_id.data = s_imu_frame;
    s_imu_msg.header.frame_id.size = strlen(s_imu_frame);
    s_imu_msg.header.frame_id.capacity = sizeof s_imu_frame;
    const double ori[3] = {0.0025, 0.0025, 0.01}, gyr = 2.5e-5, acc = 0.01;
    for (int k = 0; k < 3; k++) {
        s_imu_msg.orientation_covariance[k * 4] = ori[k];
        s_imu_msg.angular_velocity_covariance[k * 4] = gyr;
        s_imu_msg.linear_acceleration_covariance[k * 4] = acc;
    }
    memset(&s_power_msg, 0, sizeof s_power_msg);
    s_power_msg.location.data = s_location;
    s_power_msg.location.size = strlen(s_location);
    s_power_msg.location.capacity = sizeof s_location;
    s_power_msg.design_capacity = 2.2f;   // label: 12 V 2200 mAh, 3 cells
    s_power_msg.charge = NAN;
    s_power_msg.capacity = NAN;
    s_power_msg.percentage = NAN;         // tools/battery_monitor.py estimates it from the curve
    s_power_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    s_power_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
    s_power_msg.present = true;
}

static void publish(int64_t now, int64_t *next_power) {
    imu_state_t imu;
    bool fresh;
    taskENTER_CRITICAL(&s_lock);
    imu = s_imu; fresh = s_imu_fresh; s_imu_fresh = false;
    taskEXIT_CRITICAL(&s_lock);
    if (fresh) {
        stamp(&s_imu_msg.header.stamp);
        s_imu_msg.orientation.x = imu.q[0]; s_imu_msg.orientation.y = imu.q[1];
        s_imu_msg.orientation.z = imu.q[2]; s_imu_msg.orientation.w = imu.q[3];
        s_imu_msg.angular_velocity.x = imu.gyro[0]; s_imu_msg.angular_velocity.y = imu.gyro[1];
        s_imu_msg.angular_velocity.z = imu.gyro[2];
        s_imu_msg.linear_acceleration.x = imu.accel[0]; s_imu_msg.linear_acceleration.y = imu.accel[1];
        s_imu_msg.linear_acceleration.z = imu.accel[2];
        if (rcl_publish(&s_pub_imu, &s_imu_msg, NULL) != RCL_RET_OK) s_pub_fail++;
    }
    if (now < *next_power) return;
    *next_power = now + POWER_PERIOD_MS * 1000;
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
    for (;;) {
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
        publish(now, &next_power);
        // Fixed schedule (not a fixed sleep after the work), so /imu holds 50 Hz.
        if (!xTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / IMU_HZ * IMU_PUB_DIV))) wake = xTaskGetTickCount();
    }
}

void app_main(void) {
    // Motors off, buzzer quiet, LEDs off, before anything else.
    out_low(PIN_M1A); out_low(PIN_M1B); out_low(PIN_M2A); out_low(PIN_M2B);
    out_low(PIN_BUZZER); out_low(PIN_LED_BLUE); out_low(PIN_LED_GREEN);
    servo_hold();
    ESP_LOGI(TAG, "lidarbot_base phase 1 (sensors only, motors off)");

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

    xTaskCreatePinnedToCore(sensor_task, "sensors", 4096, (void *)(intptr_t)imu, 6, NULL, 1);
    xTaskCreatePinnedToCore(status_task, "status", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(ros_task, "ros", 16384, NULL, 5, NULL, 0);
}
