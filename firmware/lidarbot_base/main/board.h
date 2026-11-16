// Yahboom MicroROS V2 control board (ESP32-S3) pin map.
// From Yahboom's V2 board page (study/MicroROS-V2/20031), every pin checked with
// firmware/rover_probe on 2026-11-15. Yahboom's ESP-IDF samples are for an older
// 4-motor board: don't take pins from them.
#pragma once

#define PIN_IMU_SCL 39        // QMI8658 at 0x6B, mounted turned 180 deg (x back, y right, z up)
#define PIN_IMU_SDA 40
#define PIN_AUX_SCL 2         // OLED header: SSD1306 0x3C, INA228 0x40
#define PIN_AUX_SDA 1

#define PIN_M1A 13            // left motor; positive on A turns the wheel backward
#define PIN_M1B 14
#define PIN_M2A 11            // right motor; positive on A turns the wheel forward
#define PIN_M2B 12
#define PIN_H1A 5             // left encoder; counts down going forward
#define PIN_H1B 4
#define PIN_H2A 7             // right encoder; counts up going forward
#define PIN_H2B 6

#define PIN_SERVO_PAN 8       // 500-2500 us = -90..+90 deg; pan -30 turns right
#define PIN_SERVO_TILT 16     // larger = looks up
#define PIN_BUZZER 47         // passive
#define PIN_LED_BLUE 48
#define PIN_LED_GREEN 45
#define PIN_KEY 42            // active low
#define PIN_BATTERY_ADC 3     // ADC1 channel 2, divider x4.03
#define PIN_LIDAR_TX 17       // UART1, 230400
#define PIN_LIDAR_RX 18
