// Settings commands typed on the serial console (only while micro-ROS isn't using it):
//   transport serial | transport udp     micro-ROS link, saved, then the board restarts
//   agent 192.168.1.140 8090             UDP agent address, saved (used from the next restart)
// Pure parsing (no hardware calls) so it can be tested on the host: test/test_commands.c.
#pragma once
#include <stdint.h>

typedef enum { CMD_NONE, CMD_TRANSPORT_UDP, CMD_TRANSPORT_SERIAL, CMD_AGENT } cmd_kind_t;

typedef struct {
    cmd_kind_t kind;
    uint8_t ip[4];     // CMD_AGENT only
    uint16_t port;     // CMD_AGENT only
} cmd_t;

// One console line (trailing CR/LF allowed). Anything malformed is CMD_NONE.
cmd_t cmd_parse(const char *line);
