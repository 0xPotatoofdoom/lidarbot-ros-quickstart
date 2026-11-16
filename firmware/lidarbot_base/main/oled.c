#include "oled.h"
#include <ctype.h>
#include <string.h>

#define ADDR 0x3C
#define SCALE 2
#define CHAR_W 8   // 3 px glyph x2 + 2 px gap

static i2c_master_dev_handle_t s_dev;
static uint8_t s_fb[512];   // 4 pages x 128 columns

// 3x5 glyphs, 15 bits row-major from the top-left (bit 14 = row 0, left column).
#define G(a, b, c, d, e) ((a) << 12 | (b) << 9 | (c) << 6 | (d) << 3 | (e))
static uint16_t glyph(char ch) {
    switch (toupper((unsigned char)ch)) {
        case '0': return G(7, 5, 5, 5, 7);  case '1': return G(2, 6, 2, 2, 7);
        case '2': return G(7, 1, 7, 4, 7);  case '3': return G(7, 1, 7, 1, 7);
        case '4': return G(5, 5, 7, 1, 1);  case '5': return G(7, 4, 7, 1, 7);
        case '6': return G(7, 4, 7, 5, 7);  case '7': return G(7, 1, 1, 1, 1);
        case '8': return G(7, 5, 7, 5, 7);  case '9': return G(7, 5, 7, 1, 7);
        case '.': return G(0, 0, 0, 0, 2);  case '-': return G(0, 0, 7, 0, 0);
        case ':': return G(0, 2, 0, 2, 0);  case '%': return G(5, 1, 2, 4, 5);
        case '/': return G(1, 1, 2, 4, 4);
        case 'A': return G(2, 5, 7, 5, 5);  case 'B': return G(6, 5, 6, 5, 6);
        case 'C': return G(3, 4, 4, 4, 3);  case 'D': return G(6, 5, 5, 5, 6);
        case 'E': return G(7, 4, 6, 4, 7);  case 'F': return G(7, 4, 6, 4, 4);
        case 'G': return G(3, 4, 5, 5, 3);  case 'H': return G(5, 5, 7, 5, 5);
        case 'I': return G(7, 2, 2, 2, 7);  case 'K': return G(5, 5, 6, 5, 5);
        case 'L': return G(4, 4, 4, 4, 7);  case 'M': return G(5, 7, 7, 5, 5);
        case 'N': return G(6, 5, 5, 5, 5);  case 'O': return G(2, 5, 5, 5, 2);
        case 'P': return G(6, 5, 6, 4, 4);  case 'R': return G(6, 5, 6, 5, 5);
        case 'S': return G(3, 4, 2, 1, 6);  case 'T': return G(7, 2, 2, 2, 2);
        case 'U': return G(5, 5, 5, 5, 7);  case 'V': return G(5, 5, 5, 5, 2);
        case 'W': return G(5, 5, 7, 7, 5);  case 'X': return G(5, 5, 2, 5, 5);
        case 'Y': return G(5, 5, 2, 2, 2);
        default: return 0;
    }
}

static bool cmds(const uint8_t *c, size_t n) {
    uint8_t b[32] = {0x00};
    memcpy(b + 1, c, n);
    return i2c_master_transmit(s_dev, b, n + 1, 50) == ESP_OK;
}

static void px(int x, int y) {
    if (x >= 0 && x < 128 && y >= 0 && y < 32) s_fb[(y / 8) * 128 + x] |= 1 << (y % 8);
}

static void text(int y0, const char *s) {
    for (int i = 0; s[i] && i < 16; i++) {
        uint16_t g = glyph(s[i]);
        for (int r = 0; r < 5; r++)
            for (int c = 0; c < 3; c++)
                if (g & (1 << (14 - (r * 3 + c))))
                    for (int dy = 0; dy < SCALE; dy++)
                        for (int dx = 0; dx < SCALE; dx++) px(i * CHAR_W + c * SCALE + dx, y0 + r * SCALE + dy);
    }
}

bool oled_init(i2c_master_bus_handle_t bus) {
    if (i2c_master_probe(bus, ADDR, 50) != ESP_OK) return false;
    i2c_device_config_t dc = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = ADDR, .scl_speed_hz = 400000};
    if (i2c_master_bus_add_device(bus, &dc, &s_dev) != ESP_OK) return false;
    // 0xA1 + 0xC8 show it the right way up on our board (checked with rover_probe).
    const uint8_t init[] = {0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
                            0xA1, 0xC8, 0xDA, 0x02, 0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF};
    return cmds(init, sizeof init);
}

void oled_lines(const char *top, const char *bottom) {
    if (!s_dev) return;
    memset(s_fb, 0, sizeof s_fb);
    text(3, top);
    text(19, bottom);
    const uint8_t win[] = {0x21, 0, 127, 0x22, 0, 3};
    cmds(win, sizeof win);
    for (int i = 0; i < 512; i += 32) {
        uint8_t b[33] = {0x40};
        memcpy(b + 1, s_fb + i, 32);
        i2c_master_transmit(s_dev, b, 33, 50);
    }
}
