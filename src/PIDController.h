/*
 * PIDController.h
 * Simplified direct PID implementation
 */

#pragma once
#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    
    // Main compute function - call this every loop
    float compute(float error);
    
    // Tuning
    void setTunings(float kp, float ki, float kd);
    
    // Reset PID state
    void reset();
    
    // Set output constraints
    void setOutputLimits(float min, float max);
    
    // Getters for WiFi monitoring
    float getKp() { return Kp; }
    float getKi() { return Ki; }
    float getKd() { return Kd; }
    float getOutput() { return output; }

private:
    // PID gains
    float Kp, Ki, Kd;
    
    // Output limits
    float outputMin, outputMax;
    
    // Current output
    float output;
    
    // PID state variables
    float integral;
    float lastError;
    unsigned long lastTime;
};