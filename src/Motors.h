/*
 * Motors.h
 * Enhanced motor control with WiFi-tunable parameters
 */

#pragma once
#include "Pins.h"
#include <ESP32Encoder.h>

// WiFi-tunable motor parameters (defined as extern, initialized in Motors.cpp)
extern int TICKS_FOR_90_DEG;
extern int TICKS_FOR_180_DEG;
extern int TICKS_TO_CENTER;
extern int BASE_SPEED;
extern int TURN_SPEED;
extern int MAX_SPEED;

class Motors {
public:
    Motors();
    void setup();
    
    void setSpeeds(int leftSpeed, int rightSpeed);
    void stopBrake();
    
    void turn_90_left();
    void turn_90_right();
    void turn_180_back();
    void rotate();
    void moveForward(int ticks);
    
    long getLeftCount();
    long getRightCount();
    long getAverageCount();
    void clearEncoders();
    
    // WiFi tuning methods
    static void updateTurnTicks(int ticks90);
    static void updateCenterTicks(int ticks);
    static void updateSpeeds(int base, int turn, int max);

private:
    const int pwm_channel_left = 0;
    const int pwm_channel_right = 1;
    const int pwm_frequency = 5000;
    const int pwm_resolution = 8;
    
    ESP32Encoder leftEncoder;
    ESP32Encoder rightEncoder;
};