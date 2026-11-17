#include "commands.h"
#include <stdio.h>
#include <string.h>

static const char *skip_spaces(const char *s) {
    while (*s == ' ') s++;
    return s;
}

// True if only spaces and line endings are left.
static int at_end(const char *s) {
    return s[strspn(s, " \r\n")] == '\0';
}

static cmd_t parse_agent(const char *arg) {
    cmd_t none = {.kind = CMD_NONE};
    unsigned a, b, c, d, port;
    int used = 0;
    if (sscanf(arg, "%3u.%3u.%3u.%3u %5u%n", &a, &b, &c, &d, &port, &used) != 5 || !at_end(arg + used))
        return none;
    if (a > 255 || b > 255 || c > 255 || d > 255 || port == 0 || port > 65535) return none;
    // sscanf's %u also takes a sign; reject anything that isn't plain digits, dots and spaces.
    if (strspn(arg, "0123456789. \r\n") != strlen(arg)) return none;
    cmd_t cmd = {.kind = CMD_AGENT, .ip = {(uint8_t)a, (uint8_t)b, (uint8_t)c, (uint8_t)d}, .port = (uint16_t)port};
    return cmd;
}

cmd_t cmd_parse(const char *line) {
    cmd_t none = {.kind = CMD_NONE};
    const char *s = skip_spaces(line);
    if (strncmp(s, "transport ", 10) == 0) {
        const char *arg = skip_spaces(s + 10);
        if (strncmp(arg, "serial", 6) == 0 && at_end(arg + 6)) return (cmd_t){.kind = CMD_TRANSPORT_SERIAL};
        if (strncmp(arg, "udp", 3) == 0 && at_end(arg + 3)) return (cmd_t){.kind = CMD_TRANSPORT_UDP};
        return none;
    }
    if (strncmp(s, "agent ", 6) == 0) return parse_agent(skip_spaces(s + 6));
    return none;
}
