// Wi-Fi station and the micro-ROS agent settings, both read from the settings that Yahboom's
// firmware (or tools/config_robot.py) saved in the nvs partition. Nothing is in the source.
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    char agent_ip[16];
    char agent_port[6];
    int domain_id;
} net_config_t;

void net_load_config(net_config_t *cfg);
// Saves the UDP agent address in Yahboom's keys (the same ones net_load_config reads).
esp_err_t net_save_agent(const uint8_t ip[4], uint16_t port);
void net_wifi_start(void);
bool net_wifi_up(void);
int net_wifi_rssi(void);
