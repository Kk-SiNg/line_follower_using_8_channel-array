/*
 * PIDController.cpp
 * Direct PID implementation - no dependencies
 */

#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) 
    : Kp(kp), Ki(ki), Kd(kd), 
      output(0),
      outputMin(-255), outputMax(255),
      integral(0), 
      lastError(0), 
      lastTime(0) {
}

float PIDController::compute(float error) {
    unsigned long now = millis();
    
    // First call initialization
    if (lastTime == 0) {
        lastTime = now;
        lastError = error;
        return 0;
    }
    
    // Calculate time delta (in seconds)
    float dt = (now - lastTime) / 1000.0;
    
    // Prevent computation if called too quickly
    if (dt < 0.001) {  // Less than 1ms
        return output;  // Return last output
    }
    
    // === PROPORTIONAL TERM ===
    float P = Kp * error;
    
    // === INTEGRAL TERM (with anti-windup) ===
    integral += error * dt;
    
    // Anti-windup: Clamp integral to prevent overflow
    float maxIntegral = 100.0;  // Reasonable limit
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;
    
    float I = Ki * integral;
    
    // === DERIVATIVE TERM ===
    // Use derivative on measurement to avoid derivative kick
    float derivative = (error - lastError) / dt;
    float D = Kd * derivative;
    
    // === COMPUTE OUTPUT ===
    output = P + I + D;
    
    // Clamp output to limits
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;
    
    // Save state for next iteration
    lastError = error;
    lastTime = now;
    
    return output;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::reset() {
    integral = 0;
    lastError = 0;
    output = 0;
    lastTime = 0;
}

void PIDController::setOutputLimits(float min, float max) {
    outputMin = min;
    outputMax = max;
    
    // Clamp current output if needed
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;
}