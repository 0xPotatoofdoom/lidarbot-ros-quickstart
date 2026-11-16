// Wi-Fi station and the micro-ROS agent settings, both read from the settings that Yahboom's
// firmware (or tools/config_robot.py) saved in the nvs partition. Nothing is in the source.
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    char agent_ip[16];
    char agent_port[6];
    int domain_id;
} net_config_t;

void net_load_config(net_config_t *cfg);
void net_wifi_start(void);
bool net_wifi_up(void);
int net_wifi_rssi(void);
