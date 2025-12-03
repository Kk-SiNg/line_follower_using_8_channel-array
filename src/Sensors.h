/*
 * Sensors.h
 * 8-Channel RLS-08 HYBRID Sensor Array
 * UPGRADE: 5 sensors → 8 sensors with analog precision
 */

#pragma once
#include "Pins.h"
#include <Arduino.h>

// Path availability at junction
struct PathOptions {
    bool left = false;
    bool straight = false;
    bool right = false;
};

// Junction type classification (NEW FEATURE)
enum JunctionType {
    JUNCTION_NONE,
    JUNCTION_T_LEFT,        // ├ (left + straight)
    JUNCTION_T_RIGHT,       // ┤ (right + straight)
    JUNCTION_T_BOTH,        // ┬ (left + right + straight)
    JUNCTION_CROSS,         // ┼ (all paths)
    JUNCTION_90_LEFT,       // └ (only left)
    JUNCTION_90_RIGHT,      // ┘ (only right)
    JUNCTION_DEAD_END       // End of line
};

class Sensors {
public:
    Sensors();
    void setup();
    
    // Get line position error for PID (-7.0 to +7.0 with decimals)
    float getLineError();
    
    // Check which paths are available (for LSRB at junctions)
    PathOptions getAvailablePaths();
    
    // Classify junction type (NEW)
    JunctionType classifyJunction(PathOptions paths);
    
    // Detect if we've completely lost the line (dead end)
    bool isLineEnd();
    
    bool isEndPoint();
    // Read raw sensor values
    void readRaw(uint16_t* values);       // Analog values (0-4095)
    void readDigital(bool* values);       // Convert to digital
    
    // Get weighted position (-7.0 to +7.0 with sub-sensor precision)
    float getPosition();
    
    // Get sensor array for WiFi monitoring
    void getSensorArray(bool* arr);           // Digital representation
    void getAnalogArray(uint16_t* arr);       // Raw analog values (NEW)
    
    // Check if currently on a line (any sensor active)
    bool onLine();
    
    // Calibration
    void calibrate();  // Manual recalibration
    void printCalibration();  // Display calibration values (NEW)
    
    // Sensor confidence (NEW FEATURE)
    float getSensorConfidence(uint8_t sensorIndex);
    
    // Get active sensor count
    int getActiveSensorCount();

private:
    uint16_t sensorValues[SensorCount];           // Current analog readings
    uint16_t minValues[SensorCount];              // Calibration: min (black surface)
    uint16_t maxValues[SensorCount];              // Calibration: max (white line)
    
    // Weights for 8 sensors (increased resolution)
    const int8_t weights[SensorCount] = {-7, -5, -3, -1, 1, 3, 5, 7};
    const int8_t setpoint = 0;
    float lastPosition;
    
    // Convert analog to digital based on threshold
    bool isLineDetected(uint16_t analogValue, uint8_t sensorIndex);
    
    // Normalize analog value to 0.0-1.0 based on calibration
    float normalizeValue(uint16_t rawValue, uint8_t sensorIndex);
};
