// TI INA228 power monitor (20-bit), battery output line.
#pragma once
#include <stdbool.h>
#include "driver/i2c_master.h"

typedef struct {
    double bus_v;       // battery voltage (VBUS jumpered to VIN-)
    double current_a;   // positive = flowing out of the battery into the robot
    double power_w;
    double charge_ah;   // used since boot, from the chip's own accumulator
    double energy_wh;   // used since boot
    double die_c;
} ina228_reading_t;

// Probes the address, checks the TI manufacturer ID and INA228 device ID, configures
// averaging and the shunt calibration, and clears the accumulators.
bool ina228_init(i2c_master_bus_handle_t bus, uint8_t addr, double shunt_ohms);
bool ina228_read(ina228_reading_t *out);
