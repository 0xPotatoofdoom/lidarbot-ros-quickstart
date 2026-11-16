// Differential drive: wheel speed control (PI + feedforward) from the encoders at 100 Hz,
// and odometry integrated from the encoder counts.
#pragma once
#include <stdbool.h>

#define WHEEL_DIAMETER_M 0.0675
#define TRACK_M 0.1714            // wheel centre to wheel centre
#define COUNTS_PER_REV 1320.0     // quadrature (x4) counts per wheel turn
#define MAX_WHEEL_SPEED 0.6       // m/s per wheel
#define CMD_TIMEOUT_MS 500        // stop if /cmd_vel goes quiet this long

typedef struct {
    double x, y, heading;         // m, m, rad: integrated since boot (odom frame)
    double v, w;                  // m/s, rad/s: measured, robot frame
    double wheel_l, wheel_r;      // m/s measured
    double duty_l, duty_r;        // -1..1 applied
} drive_state_t;

void drive_start(void);
// Velocity command from /cmd_vel (m/s, rad/s). Clamped so neither wheel exceeds MAX_WHEEL_SPEED.
void drive_command(double v, double w);
drive_state_t drive_state(void);
