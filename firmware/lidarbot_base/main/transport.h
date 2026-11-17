// micro-ROS link: UDP over Wi-Fi to an agent (as before), or the USB serial cable (UART0, the
// board's CP2102) to the agent on the Orange Pi. Chosen at boot from nvs ("transport serial|udp"
// on the console); holding the KEY at power-on forces UDP for that boot.
// Serial mode owns UART0: logs go quiet and console commands are off (see main.c).
#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "net.h"

typedef enum { TRANSPORT_UDP, TRANSPORT_SERIAL } transport_kind_t;

#define TRANSPORT_SERIAL_BAUD 921600

transport_kind_t transport_load(void);              // saved choice, UDP if none
esp_err_t transport_save(transport_kind_t kind);
const char *transport_name(transport_kind_t kind);
// Registers the custom transport. Call before rcl_init_options_init (the options copy it).
bool transport_setup(transport_kind_t kind, const net_config_t *cfg);
