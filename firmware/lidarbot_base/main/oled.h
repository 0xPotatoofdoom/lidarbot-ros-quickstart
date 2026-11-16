// SSD1306 128x32 status display: two lines of up to 16 characters (3x5 font drawn at 2x).
#pragma once
#include <stdbool.h>
#include "driver/i2c_master.h"

bool oled_init(i2c_master_bus_handle_t bus);
// Characters without a glyph show as blanks. Lower case is drawn as upper case.
void oled_lines(const char *top, const char *bottom);
