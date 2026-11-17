#include "lidar.h"
#include <math.h>
#include <string.h>
#include "board.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lidar";

#define UART UART_NUM_1
#define BAUD 230400
#define SAMPLE_BYTES 3            // T-mini Plus: intensity byte + 2 distance bytes
#define MAX_LSN 80

// The lidar reports angles clockwise seen from above, 0 at its own reference mark. ROS angles
// are counter-clockwise with 0 straight ahead: ros = MOUNT_OFFSET_DEG + LIDAR_DIRECTION * lidar.
// Checked on the robot (2026-11-16): a box straight ahead read about -4 deg, and a person behind-left
// (8 o'clock) read +122..+158 deg, so the direction is right and the offset is within a few degrees.
#define LIDAR_DIRECTION (-1)
#define MOUNT_OFFSET_DEG 0.0

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static lidar_scan_t s_building, s_ready;
static uint32_t s_good, s_bad;
static float s_hz;
static volatile bool s_started, s_motor_off;
static const uint8_t CMD_STOP[] = {0xA5, 0x65}, CMD_START[] = {0xA5, 0x60};

static void put_point(float lidar_deg, float range_m) {
    float deg = MOUNT_OFFSET_DEG + LIDAR_DIRECTION * lidar_deg;
    float rad = deg * (float)M_PI / 180.0f;
    rad = atan2f(sinf(rad), cosf(rad));                       // -pi .. pi
    int i = (int)lroundf((rad + (float)M_PI) / (2.0f * (float)M_PI / SCAN_POINTS)) % SCAN_POINTS;
    float *slot = &s_building.ranges[i];
    if (range_m > 0 && (*slot == 0 || range_m < *slot)) *slot = range_m;   // keep the nearest
}

static void finish_scan(void) {
    s_building.scan_hz = s_hz;
    s_building.good_packets = s_good;
    s_building.bad_packets = s_bad;
    taskENTER_CRITICAL(&s_lock);
    s_building.seq = s_ready.seq + 1;
    s_ready = s_building;
    taskEXIT_CRITICAL(&s_lock);
    memset(s_building.ranges, 0, sizeof s_building.ranges);
}

// One packet after the AA 55 header: CT, LSN, FSA(2), LSA(2), CS(2), LSN samples.
static void handle_packet(const uint8_t *h, const uint8_t *samples) {
    uint8_t ct = h[0], lsn = h[1];
    uint16_t fsa = h[2] | h[3] << 8, lsa = h[4] | h[5] << 8, cs = h[6] | h[7] << 8;
    uint16_t sum = 0x55AA ^ (ct | lsn << 8) ^ fsa ^ lsa;
    for (int i = 0; i < lsn; i++) {
        const uint8_t *s = samples + i * SAMPLE_BYTES;
        sum ^= s[0];
        sum ^= (uint16_t)(s[1] | s[2] << 8);
    }
    if (sum != cs) { s_bad++; return; }
    s_good++;
    if (ct & 0x01) {                                          // start of a new turn
        s_hz = (ct >> 1) / 10.0f;
        finish_scan();
    }
    float a0 = (fsa >> 1) / 64.0f, a1 = (lsa >> 1) / 64.0f;
    float span = a1 - a0;
    if (span < 0) span += 360.0f;
    for (int i = 0; i < lsn; i++) {
        const uint8_t *s = samples + i * SAMPLE_BYTES;
        uint16_t mm = (uint16_t)(s[1] | s[2] << 8) >> 2;
        float deg = a0 + (lsn > 1 ? span * i / (lsn - 1) : 0);
        put_point(fmodf(deg, 360.0f), mm / 1000.0f);
    }
}

static void lidar_task(void *arg) {
    static uint8_t buf[1024];
    size_t len = 0;
    for (;;) {
        int n = uart_read_bytes(UART, buf + len, sizeof buf - len, pdMS_TO_TICKS(20));
        if (n > 0) len += n;
        size_t i = 0;
        while (len - i >= 10) {
            if (buf[i] != 0xAA || buf[i + 1] != 0x55) { i++; continue; }
            uint8_t lsn = buf[i + 3];
            if (lsn == 0 || lsn > MAX_LSN) { i++; s_bad++; continue; }
            size_t total = 10 + (size_t)lsn * SAMPLE_BYTES;
            if (len - i < total) break;                       // wait for the rest
            handle_packet(buf + i + 2, buf + i + 10);
            i += total;
        }
        memmove(buf, buf + i, len - i);
        len -= i;
        if (len == sizeof buf) len = 0;                       // garbage: start over
    }
}

void lidar_start(void) {
    uart_config_t c = {.baud_rate = BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                       .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                       .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(UART, 8192, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART, &c));
    ESP_ERROR_CHECK(uart_set_pin(UART, PIN_LIDAR_TX, PIN_LIDAR_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_write_bytes(UART, CMD_STOP, 2);
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_flush_input(UART);
    if (!s_motor_off) uart_write_bytes(UART, CMD_START, 2);   // stays off if the battery is critical
    s_started = true;
    xTaskCreatePinnedToCore(lidar_task, "lidar", 4096, NULL, 7, NULL, 1);
    ESP_LOGI(TAG, "started");
}

void lidar_motor(bool on) {
    s_motor_off = !on;
    if (s_started) uart_write_bytes(UART, on ? CMD_START : CMD_STOP, 2);
}

bool lidar_take(lidar_scan_t *out, uint32_t *seq) {
    bool fresh = false;
    taskENTER_CRITICAL(&s_lock);
    if (s_ready.seq != *seq) { *out = s_ready; *seq = s_ready.seq; fresh = true; }
    taskEXIT_CRITICAL(&s_lock);
    return fresh;
}
