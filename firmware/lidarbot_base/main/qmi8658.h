// QMI8658 6-axis IMU, converted to the robot's ROS frame (x forward, y left, z up) and SI units,
// with a complementary filter for orientation (roll/pitch from gravity, yaw from the gyro).
#pragma once
#include <stdbool.h>
#include "driver/i2c_master.h"

typedef struct {
    double accel[3];   // m/s^2
    double gyro[3];    // rad/s, bias removed
    double q[4];       // orientation quaternion x, y, z, w
    double temp_c;
} imu_state_t;

bool qmi8658_init(i2c_master_bus_handle_t bus);
// Averages the gyro for `ms` milliseconds; the robot must be still.
bool qmi8658_calibrate(int ms);
// Reads one sample and advances the filter by dt seconds.
bool qmi8658_update(double dt, imu_state_t *out);
