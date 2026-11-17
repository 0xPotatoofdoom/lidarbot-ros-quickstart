#include "net.h"
#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs.h"

static const char *TAG = "net";
static volatile bool s_up;

// Yahboom's namespace and keys (seen in our board's nvs; set by tools/config_robot.py).
#define NS "YAHBOOM"

void net_load_config(net_config_t *cfg) {
    snprintf(cfg->agent_ip, sizeof cfg->agent_ip, "192.168.1.100");
    snprintf(cfg->agent_port, sizeof cfg->agent_port, "8090");
    cfg->domain_id = 0;
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "no %s settings in nvs, using defaults", NS);
        return;
    }
    uint8_t ip[4];
    size_t n = sizeof ip;
    if (nvs_get_blob(h, "CFG_AGENT_IP", ip, &n) == ESP_OK && n == 4)
        snprintf(cfg->agent_ip, sizeof cfg->agent_ip, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    uint16_t port;
    if (nvs_get_u16(h, "CFG_AGENT_PORT", &port) == ESP_OK)
        snprintf(cfg->agent_port, sizeof cfg->agent_port, "%u", port);
    int32_t domain;
    if (nvs_get_i32(h, "CFG_DOMAIN_ID", &domain) == ESP_OK) cfg->domain_id = domain;
    nvs_close(h);
    ESP_LOGI(TAG, "agent %s:%s, domain %d", cfg->agent_ip, cfg->agent_port, cfg->domain_id);
}

esp_err_t net_save_agent(const uint8_t ip[4], uint16_t port) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(h, "CFG_AGENT_IP", ip, 4);
    if (err == ESP_OK) err = nvs_set_u16(h, "CFG_AGENT_PORT", port);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_up = false;
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        esp_ip4_addr_t ip = ((ip_event_got_ip_t *)data)->ip_info.ip;
        ESP_LOGI(TAG, "network up, ip " IPSTR, IP2STR(&ip));
        s_up = true;
    }
}

// If the Wi-Fi driver has no saved network, use the one in Yahboom's settings.
static void load_yahboom_network(wifi_config_t *wc) {
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t n = sizeof wc->sta.ssid;
    if (nvs_get_str(h, "CFG_SSID", (char *)wc->sta.ssid, &n) == ESP_OK) {
        n = sizeof wc->sta.password;
        nvs_get_str(h, "CFG_PASSWD", (char *)wc->sta.password, &n);
    }
    nvs_close(h);
}

void net_wifi_start(void) {
    esp_log_level_set("wifi", ESP_LOG_WARN);   // the driver logs the network name
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(sta, "lidarbot-base");
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t wc = {0};
    esp_wifi_get_config(WIFI_IF_STA, &wc);
    if (wc.sta.ssid[0] == 0) {
        load_yahboom_network(&wc);
        if (wc.sta.ssid[0]) esp_wifi_set_config(WIFI_IF_STA, &wc);
    }
    ESP_LOGI(TAG, "saved network: %s", wc.sta.ssid[0] ? "yes" : "none");
    memset(&wc, 0, sizeof wc);
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);   // power save adds 100+ ms of latency
}

bool net_wifi_up(void) { return s_up; }

int net_wifi_rssi(void) {
    wifi_ap_record_t ap;
    return esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
}
