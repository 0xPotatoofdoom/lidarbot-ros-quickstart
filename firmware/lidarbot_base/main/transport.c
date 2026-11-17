#include "transport.h"
#include <stdlib.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

static const char *TAG = "transport";

#define NS "LIDARBOT"
#define KEY "transport"
#define UART UART_NUM_0
#define UART_BUF 4096            // a scan is about 2.7 KB; both directions buffered

// ---------- settings ----------

transport_kind_t transport_load(void) {
    nvs_handle_t h;
    uint8_t v = TRANSPORT_UDP;
    if (nvs_open(NS, NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_u8(h, KEY, &v) != ESP_OK) v = TRANSPORT_UDP;
        nvs_close(h);
    }
    return v == TRANSPORT_SERIAL ? TRANSPORT_SERIAL : TRANSPORT_UDP;
}

esp_err_t transport_save(transport_kind_t kind) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(h, KEY, (uint8_t)kind);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

const char *transport_name(transport_kind_t kind) {
    return kind == TRANSPORT_SERIAL ? "serial" : "udp";
}

// ---------- UDP: one socket to the agent, packet framing ----------

static int s_sock = -1;
static struct sockaddr_in s_agent;

static bool udp_open(struct uxrCustomTransport *t) {
    (void)t;
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    return s_sock >= 0;
}

static bool udp_close(struct uxrCustomTransport *t) {
    (void)t;
    if (s_sock >= 0) close(s_sock);
    s_sock = -1;
    return true;
}

static size_t udp_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err) {
    (void)t;
    int n = sendto(s_sock, buf, len, 0, (struct sockaddr *)&s_agent, sizeof s_agent);
    if (n < 0) { *err = 1; return 0; }
    return (size_t)n;
}

static size_t udp_read(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout_ms, uint8_t *err) {
    (void)t;
    fd_set rd;
    FD_ZERO(&rd);
    FD_SET(s_sock, &rd);
    struct timeval tv = {.tv_sec = timeout_ms / 1000, .tv_usec = (timeout_ms % 1000) * 1000};
    int ready = select(s_sock + 1, &rd, NULL, NULL, &tv);
    if (ready <= 0) return 0;                     // timeout (or error: the session will notice)
    int n = recv(s_sock, buf, len, 0);
    if (n < 0) { *err = 1; return 0; }
    return (size_t)n;
}

// ---------- serial: UART0 at TRANSPORT_SERIAL_BAUD, stream framing ----------

static bool serial_open(struct uxrCustomTransport *t) {
    (void)t;
    // Open and close run around every ping too: install once and keep it.
    if (!uart_is_driver_installed(UART) && uart_driver_install(UART, UART_BUF, UART_BUF, 0, NULL, 0) != ESP_OK)
        return false;
    return uart_set_baudrate(UART, TRANSPORT_SERIAL_BAUD) == ESP_OK;
}

static bool serial_close(struct uxrCustomTransport *t) {
    (void)t;
    return true;
}

static size_t serial_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err) {
    (void)t;
    int n = uart_write_bytes(UART, buf, len);
    if (n < 0) { *err = 1; return 0; }
    return (size_t)n;
}

// Return what has arrived as soon as there is something, rather than waiting for len bytes.
static size_t serial_read(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout_ms, uint8_t *err) {
    (void)t;
    if (len == 0) return 0;
    size_t avail = 0;
    uart_get_buffered_data_len(UART, &avail);
    size_t got = 0;
    if (avail == 0) {
        TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
        int n = uart_read_bytes(UART, buf, 1, ticks > 0 ? ticks : 1);
        if (n < 0) { *err = 1; return 0; }
        if (n == 0) return 0;
        got = 1;
        uart_get_buffered_data_len(UART, &avail);
    }
    size_t want = avail < len - got ? avail : len - got;
    if (want > 0) {
        int n = uart_read_bytes(UART, buf + got, want, 0);
        if (n > 0) got += (size_t)n;
    }
    return got;
}

bool transport_setup(transport_kind_t kind, const net_config_t *cfg) {
    rmw_ret_t rc;
    if (kind == TRANSPORT_SERIAL) {
        rc = rmw_uros_set_custom_transport(true, NULL, serial_open, serial_close, serial_write, serial_read);
        ESP_LOGI(TAG, "micro-ROS over serial (UART0, %d baud)", TRANSPORT_SERIAL_BAUD);
    } else {
        memset(&s_agent, 0, sizeof s_agent);
        s_agent.sin_family = AF_INET;
        s_agent.sin_port = htons((uint16_t)atoi(cfg->agent_port));
        if (inet_aton(cfg->agent_ip, &s_agent.sin_addr) == 0) {
            ESP_LOGE(TAG, "bad agent address %s", cfg->agent_ip);
            return false;
        }
        rc = rmw_uros_set_custom_transport(false, NULL, udp_open, udp_close, udp_write, udp_read);
        ESP_LOGI(TAG, "micro-ROS over UDP to %s:%s", cfg->agent_ip, cfg->agent_port);
    }
    return rc == RMW_RET_OK;
}
