// YDLIDAR T-mini Plus on UART1 (230400 baud): packet decoding and scan assembly.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define SCAN_POINTS 667           // same layout as Yahboom's /scan: -pi .. pi in 2*pi/667 steps
#define SCAN_RANGE_MIN 0.12f
#define SCAN_RANGE_MAX 8.0f

typedef struct {
    float ranges[SCAN_POINTS];    // m; 0 = no echo (Yahboom's convention)
    uint32_t seq;
    float scan_hz;                // the lidar's own rotation rate report
    uint32_t good_packets, bad_packets;
} lidar_scan_t;

void lidar_start(void);
// Copies the latest complete scan if it is newer than *seq; returns false otherwise.
bool lidar_take(lidar_scan_t *out, uint32_t *seq);
