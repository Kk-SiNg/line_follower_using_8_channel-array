/*
 * Sensors.cpp
 * 8-Channel RLS-08 HYBRID implementation
 * 6 Analog (center) + 2 Digital (outer) sensors
 */

#include "Sensors.h"

Sensors::Sensors() {
    lastPosition = 0.0;
    
    // Initialize calibration arrays with extremes
    for(int i = 0; i < SensorCount; i++) {
        minValues[i] = 4095;  // Start with max possible
        maxValues[i] = 0;      // Start with min possible
    }
}

void Sensors::setup() {
    // Configure ESP32 ADC
    analogReadResolution(12);           // 12-bit resolution (0-4095)
    analogSetAttenuation(ADC_11db);     // Full 0-3.3V range
    
    // Configure digital pins (analog pins auto-configure)
    pinMode(SENSOR_PIN_1, INPUT);  // Digital outer right
    pinMode(SENSOR_PIN_8, INPUT);  // Digital outer left
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  RLS-08 HYBRID Sensor Calibration    ║");
    Serial.println("║  8-Channel: 6 Analog + 2 Digital      ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println("Auto-calibrating analog sensors...");
    Serial.println("Move sensor over BLACK and WHITE surfaces");
    Serial.print("Calibrating for ");
    Serial.print(CALIBRATION_TIME_MS / 1000);
    Serial.println(" seconds...\n");
    
    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, HIGH);
    
    // === AUTO-CALIBRATION ROUTINE ===
    unsigned long startTime = millis();
    int sampleCount = 0;
    
    while (millis() - startTime < CALIBRATION_TIME_MS) {
        readRaw(sensorValues);
        
        // Track min and max for each ANALOG sensor
        for (uint8_t i = 0; i < SensorCount; i++) {
            if (isAnalogPin[i]) {
                if (sensorValues[i] < minValues[i]) {
                    minValues[i] = sensorValues[i];
                }
                if (sensorValues[i] > maxValues[i]) {
                    maxValues[i] = sensorValues[i];
                }
            }
        }
        
        sampleCount++;
        
        // Visual feedback (blink LED)
        if (sampleCount % 50 == 0) {
            digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
        }
        
        delay(10);
    }
    
    digitalWrite(ONBOARD_LED, LOW);
    Serial.println("✓ Calibration Complete!\n");
    
    // Display calibration results
    printCalibration();
}

void Sensors::printCalibration() {
    Serial.println("Sensor Calibration Results:");
    Serial. println("╔═══╦═════════╦═══════╦═══════╦═══════╦═══════════╗");
    Serial.println("║ # ║  Type   ║  Min  ║  Max  ║ Range ║  Status   ║");
    Serial.println("╠═══╬═════════╬═══════╬═══════╬═══════╬═══════════╣");
    
    for (uint8_t i = 0; i < SensorCount; i++) {
        // Sensor number
        Serial.printf("║ %d ║ ", i + 1);
        
        if (isAnalogPin[i]) {
            uint16_t range = maxValues[i] - minValues[i];
            
            // Print values with fixed width (4 characters each)
            Serial.printf("Analog  ║ %4d  ║ %4d  ║ %4d  ║ ", 
                          minValues[i], maxValues[i], range);
            
            // Status based on range quality
            if (range > 1500) {
                Serial.println("Excellent ║");
            } else if (range > 1000) {
                Serial.println("Good      ║");
            } else if (range > 500) {
                Serial.println("Fair      ║");
            } else {
                Serial.println("⚠ Poor    ║");
            }
        } else {
            // Digital sensors don't need calibration
            Serial.println("Digital ║  N/A  ║  N/A  ║  N/A  ║    N/A    ║");
        }
    }
    
    Serial.println("╚═══╩═════════╩═══════╩═══════╩═══════╩═══════════╝\n");
}

void Sensors::readRaw(uint16_t* values) {
    // Read sensors based on type (analog vs digital)
    
    // S1: Digital (rightmost outer)
    values[0] = digitalRead(SENSOR_PIN_1) ? 4095 : 0;
    
    // S2-S7: Analog (center 6 sensors)
    values[1] = analogRead(SENSOR_PIN_2);
    values[2] = analogRead(SENSOR_PIN_3);
    values[3] = analogRead(SENSOR_PIN_4);
    values[4] = analogRead(SENSOR_PIN_5);
    values[5] = analogRead(SENSOR_PIN_6);
    values[6] = analogRead(SENSOR_PIN_7);
    
    // S8: Digital (leftmost outer)
    values[7] = digitalRead(SENSOR_PIN_8) ? 4095 : 0;
}

float Sensors::normalizeValue(uint16_t rawValue, uint8_t sensorIndex) {
    if (!isAnalogPin[sensorIndex]) {
        // Digital sensor - return 0.0 or 1.0
        return (rawValue > 2000) ? 1.0 : 0.0;
    }
    
    // Analog sensor - normalize to 0.0-1.0 based on calibration
    uint16_t range = maxValues[sensorIndex] - minValues[sensorIndex];
    
    if (range < 100) {
        // Poor calibration - use raw threshold
        return (rawValue > LINE_THRESHOLD) ? 1.0 : 0.0;
    }
    
    // Normalize
    float normalized = (float)(rawValue - minValues[sensorIndex]) / (float)range;
    return constrain(normalized, 0.0, 1.0);
}

bool Sensors::isLineDetected(uint16_t analogValue, uint8_t sensorIndex) {
    if (isAnalogPin[sensorIndex]) {
        // Analog sensor - use calibrated threshold
        float normalized = normalizeValue(analogValue, sensorIndex);
        return normalized > MIN_DETECTION_RATIO;
    } else {
        // Digital sensor - simple threshold
        return analogValue > LINE_THRESHOLD;
    }
}

void Sensors::readDigital(bool* values) {
    readRaw(sensorValues);
    
    for (uint8_t i = 0; i < SensorCount; i++) {
        values[i] = isLineDetected(sensorValues[i], i);
    }
}

float Sensors::getPosition() {
    readRaw(sensorValues);
    
    // === ANALOG PRECISION WEIGHTED AVERAGE ===
    // Uses normalized values for smoother calculation
    
    float weightedSum = 0.0;
    float totalWeight = 0.0;
    
    for (uint8_t i = 0; i < SensorCount; i++) {
        // Normalize value (0.0 to 1.0)
        float normalized = normalizeValue(sensorValues[i], i);
        
        // Get confidence (NEW FEATURE)
        float confidence = getSensorConfidence(i);
        
        // Only use sensors detecting line (above threshold)
        if (normalized > 0.3) {  // 30% threshold to include in calculation
            // Weight by both normalized value AND confidence
            weightedSum += weights[i] * normalized * confidence;
            totalWeight += normalized * confidence;
        }
    }
    
    // Calculate position
    if (totalWeight > 0.1) {  // At least some detection
        lastPosition = weightedSum / totalWeight;
    }
    // else: keep last position (line lost momentarily - use memory)
    
    return lastPosition;
}

float Sensors::getLineError() {
    float position = getPosition();
    return position - (float)setpoint;
}

bool Sensors::onLine() {
    readRaw(sensorValues);
    
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (isLineDetected(sensorValues[i], i)) {
            return true;
        }
    }
    
    return false;
}

int Sensors::getActiveSensorCount() {
    readRaw(sensorValues);
    
    int count = 0;
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (isLineDetected(sensorValues[i], i)) {
            count++;
        }
    }
    
    return count;
}

PathOptions Sensors::getAvailablePaths() {
    readRaw(sensorValues);
    
    PathOptions paths;
    
    // Convert to digital for junction detection
    bool digital[SensorCount];
    for(int i = 0; i < SensorCount; i++) {
        digital[i] = isLineDetected(sensorValues[i], i);
    }
    
    // Count total active sensors
    int activeCount = 0;
    for(int i = 0; i < SensorCount; i++) {
        if(digital[i]) activeCount++;
    }
    
    // === IMPROVED JUNCTION DETECTION ===
    // Key insight: Normal line following = 2-4 sensors active (center)
    //              Junction = 5+ sensors active (line spreads to outer sensors)
    
    if(activeCount >= 5) {
        // Potential junction - check outer sensors
        
        // LEFT path: Leftmost sensors active (S7 or S8)
        paths.left = (digital[6] || digital[7]);
        
        // RIGHT path: Rightmost sensors active (S1 or S2)
        paths.right = (digital[0] || digital[1]);
        
        // STRAIGHT: Center sensors active (S4 or S5)
        paths.straight = (digital[3] || digital[4]);
    } else {
        // Normal line following - not a junction
        paths.left = false;
        paths.right = false;
        paths.straight = (activeCount > 0);  // Straight if any sensor active
    }
    
    return paths;
}

JunctionType Sensors::classifyJunction(PathOptions paths) {
    // Count available paths
    int pathCount = 0;
    if (paths.left) pathCount++;
    if (paths.straight) pathCount++;
    if (paths.right) pathCount++;
    
    // Classify based on pattern
    if (pathCount == 0) {
        return JUNCTION_DEAD_END;
    } else if (pathCount == 1) {
        if (paths.left && !paths.straight) return JUNCTION_90_LEFT;
        if (paths.right && !paths.straight) return JUNCTION_90_RIGHT;
        return JUNCTION_NONE;  // Just straight (normal line)
    } else if (pathCount == 2) {
        if (paths.left && paths.straight) return JUNCTION_T_LEFT;
        if (paths.right && paths.straight) return JUNCTION_T_RIGHT;
        if (paths.left && paths.right) return JUNCTION_T_BOTH;
        return JUNCTION_NONE;
    } else if (pathCount == 3) {
        return JUNCTION_CROSS;
    }
    
    return JUNCTION_NONE;
}

bool Sensors::isLineEnd() {
    readRaw(sensorValues);
    
    // Check if ALL sensors see black (no line)
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (isLineDetected(sensorValues[i], i)) {
            return false;  // Found a sensor that sees line → NOT line end
        }
    }
    
    // All sensors see black → LINE END
    return true;
}

void Sensors::getSensorArray(bool* arr) {
    readDigital(arr);
}

void Sensors::getAnalogArray(uint16_t* arr) {
    readRaw(arr);
}

void Sensors::calibrate() {
    // Manual recalibration - just call setup again
    setup();
}

float Sensors::getSensorConfidence(uint8_t sensorIndex) {
    // === NEW FEATURE: SENSOR CONFIDENCE SCORING ===
    // Returns 0.0-1.0 based on how reliable this sensor is
    
    if (!isAnalogPin[sensorIndex]) {
        return 1.0;  // Digital sensors are binary - always "confident"
    }
    
    // For analog sensors, confidence = calibration range quality
    uint16_t range = maxValues[sensorIndex] - minValues[sensorIndex];
    
    if (range > 2000) {
        return 1.0;  // Excellent contrast
    } else if (range > 1500) {
        return 0.9;  // Very good
    } else if (range > 1000) {
        return 0.8;  // Good
    } else if (range > 500) {
        return 0.6;  // Fair - usable but unreliable
    } else {
        return 0.3;  // Poor - barely usable
    }
}
