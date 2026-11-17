// Host test for main/commands.c (no ESP-IDF needed):
//   cc -std=c11 -Wall -Wextra -fsanitize=address,undefined -I main test/test_commands.c main/commands.c -o /tmp/tc && /tmp/tc
#include <assert.h>
#include <stdio.h>
#include "commands.h"

int main(void) {
    assert(cmd_parse("transport serial").kind == CMD_TRANSPORT_SERIAL);
    assert(cmd_parse("transport udp\r\n").kind == CMD_TRANSPORT_UDP);
    assert(cmd_parse("  transport   udp  ").kind == CMD_TRANSPORT_UDP);
    assert(cmd_parse("transport serialx").kind == CMD_NONE);
    assert(cmd_parse("transport").kind == CMD_NONE);
    assert(cmd_parse("transport tcp").kind == CMD_NONE);

    cmd_t c = cmd_parse("agent 192.168.1.140 8090\r");
    assert(c.kind == CMD_AGENT && c.ip[0] == 192 && c.ip[1] == 168 && c.ip[2] == 1 && c.ip[3] == 140);
    assert(c.port == 8090);
    assert(cmd_parse("agent 10.0.0.1 1").kind == CMD_AGENT);
    assert(cmd_parse("agent 256.0.0.1 8090").kind == CMD_NONE);
    assert(cmd_parse("agent 1.2.3 8090").kind == CMD_NONE);
    assert(cmd_parse("agent 1.2.3.4").kind == CMD_NONE);
    assert(cmd_parse("agent 1.2.3.4 0").kind == CMD_NONE);
    assert(cmd_parse("agent 1.2.3.4 70000").kind == CMD_NONE);
    assert(cmd_parse("agent 1.2.3.4 8090 extra").kind == CMD_NONE);
    assert(cmd_parse("agent -1.2.3.4 8090").kind == CMD_NONE);
    assert(cmd_parse("agent 1.2.3.4 +80").kind == CMD_NONE);

    assert(cmd_parse("").kind == CMD_NONE);
    assert(cmd_parse("batt sim 10.4").kind == CMD_NONE);   // battery_guard.c handles those
    puts("commands: all checks passed");
    return 0;
}
