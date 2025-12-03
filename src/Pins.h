/*
 * Pins.h
 * Hardware pin definitions for ESP32 38-pin with 8-channel RLS-08 sensor array
 * Optimized for IIT Bombay Mesmerize Competition
 * 
 * UPGRADE: 5-channel → 8-channel HYBRID sensor array
 * - Center 6 sensors: ANALOG (precise line following)
 * - Outer 2 sensors: DIGITAL (junction detection)
 * - All ADC1 pins (WiFi compatible)
 */

#pragma once
#include <cstdint>

// === RLS-08 8-CHANNEL HYBRID SENSOR ARRAY ===
// Hybrid: 6 Analog (ADC1) + 2 Digital (GPIO)
#define SENSOR_PIN_1  25  // Digital - Rightmost (outer) - Safe GPIO
#define SENSOR_PIN_2  39  // ADC1_CH3 - Analog
#define SENSOR_PIN_3  34  // ADC1_CH6 - Analog
#define SENSOR_PIN_4  35  // ADC1_CH7 - Analog (Center-Right)
#define SENSOR_PIN_5  32  // ADC1_CH4 - Analog (Center-Left)
#define SENSOR_PIN_6  33  // ADC1_CH5 - Analog
#define SENSOR_PIN_7  36  // ADC1_CH0 - Analog
#define SENSOR_PIN_8  26  // Digital - Leftmost (outer) - Safe GPIO

const uint8_t SensorCount = 8;

// Track which pins are analog vs digital
const bool isAnalogPin[SensorCount] = {
    false,  // S1 (GPIO 25) - Digital
    true,   // S2 (GPIO 39) - Analog
    true,   // S3 (GPIO 34) - Analog
    true,   // S4 (GPIO 35) - Analog
    true,   // S5 (GPIO 32) - Analog
    true,   // S6 (GPIO 33) - Analog
    true,   // S7 (GPIO 36) - Analog
    false   // S8 (GPIO 26) - Digital
};

// Analog sensor configuration
#define LINE_THRESHOLD 2000           // Analog threshold (0-4095) for analog pins
#define MIN_DETECTION_RATIO 0.4       // 40% of calibrated range
#define CALIBRATION_TIME_MS 3000      // 3 seconds for auto-calibration

// === MOTOR CONTROL (L298N) ===
// Left Motor (Motor A)
#define MOTOR_L_IN1 15   // Direction control 1
#define MOTOR_L_IN2 4    // Direction control 2
#define MOTOR_L_ENA 16   // Speed control (PWM)

// Right Motor (Motor B)
#define MOTOR_R_IN3 17   // Direction control 1
#define MOTOR_R_IN4 18   // Direction control 2
#define MOTOR_R_ENB 19   // Speed control (PWM)

// === ENCODERS ===
#define ENCODER_L_A 21   // Left encoder channel A
#define ENCODER_L_B 22   // Left encoder channel B
#define ENCODER_R_A 23   // Right encoder channel A
#define ENCODER_R_B 5    // Right encoder channel B

// === USER INTERFACE ===
#define ONBOARD_LED 2    // ESP32 onboard LED
#define USER_BUTTON 0    // Boot button (also used for control)

// === WIFI CONFIGURATION ===
#define SSID "Readmi A2"
#define PASSWORD "kvsandkks"
#define TELNET_PORT 23

// === OPTIONAL: BATTERY MONITORING ===
// Uncomment if you add voltage divider for battery monitoring
// #define BATTERY_PIN 13           // ADC pin for battery voltage
// #define BATTERY_NOMINAL 8.4      // 2S LiPo nominal voltage