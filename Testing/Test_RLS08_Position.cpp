#include <Arduino.h>
/*
 * Test_RLS08_Position.ino
 * Test weighted position calculation with analog precision
 * Verify smooth position values from -7.0 to +7.0
 */

#define SENSOR_PIN_1  25
#define SENSOR_PIN_2  39
#define SENSOR_PIN_3  34
#define SENSOR_PIN_4  35
#define SENSOR_PIN_5  32
#define SENSOR_PIN_6  33
#define SENSOR_PIN_7  36
#define SENSOR_PIN_8  26

const bool isAnalog[8] = {false, true, true, true, true, true, true, false};
const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

uint16_t minVals[8] = {0, 500, 500, 500, 500, 500, 500, 0};
uint16_t maxVals[8] = {4095, 3500, 3500, 3500, 3500, 3500, 3500, 4095};

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    pinMode(SENSOR_PIN_1, INPUT);
    pinMode(SENSOR_PIN_8, INPUT);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  RLS-08 Position Calculation Test    ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    Serial.println("Instructions:");
    Serial.println("1. Place 3cm black line under sensor");
    Serial.println("2. Slowly move line from RIGHT to LEFT");
    Serial.println("3. Watch position change from -7.0 to +7.0");
    Serial.println("4. Verify smooth transitions\n");
    
    delay(3000);
}

float normalizeValue(uint16_t raw, uint8_t index) {
    if(!isAnalog[index]) {
        return (raw > 2000) ? 1.0 : 0.0;
    }
    
    uint16_t range = maxVals[index] - minVals[index];
    if(range < 100) return (raw > 2000) ? 1.0 : 0.0;
    
    float normalized = (float)(raw - minVals[index]) / (float)range;
    return constrain(normalized, 0.0, 1.0);
}

void loop() {
    uint16_t values[8];
    
    // Read sensors
    values[0] = digitalRead(SENSOR_PIN_1) ? 4095 : 0;
    values[1] = analogRead(SENSOR_PIN_2);
    values[2] = analogRead(SENSOR_PIN_3);
    values[3] = analogRead(SENSOR_PIN_4);
    values[4] = analogRead(SENSOR_PIN_5);
    values[5] = analogRead(SENSOR_PIN_6);
    values[6] = analogRead(SENSOR_PIN_7);
    values[7] = digitalRead(SENSOR_PIN_8) ? 4095 : 0;
    
    // Calculate position with analog precision
    float weightedSum = 0.0;
    float totalWeight = 0.0;
    
    for(int i = 0; i < 8; i++) {
        float normalized = normalizeValue(values[i], i);
        
        if(normalized > 0.3) {
            weightedSum += weights[i] * normalized;
            totalWeight += normalized;
        }
    }
    
    float position = (totalWeight > 0.1) ? (weightedSum / totalWeight) : 0.0;
    
    // Visual display
    Serial.println("╔═══════════════════════════════════════════════════╗");
    
    // Sensor visualization
    Serial.print("║ Sensors: [");
    for(int i = 0; i < 8; i++) {
        if(values[i] > 2000) {
            Serial.print("█");
        } else if(values[i] > 1000) {
            Serial.print("▓");
        } else if(values[i] > 500) {
            Serial.print("░");
        } else {
            Serial.print("·");
        }
        if(i < 7) Serial.print("][");
    }
    Serial.println("]          ║");
    
    // Position bar graph
    Serial.print("║ Position: ");
    
    // Draw position indicator
    int posDisplay = (int)((position + 7.0) * 3);  // Scale to 0-42
    for(int i = 0; i < 42; i++) {
        if(i == 21) {
            Serial.print("|");  // Center mark
        } else if(abs(i - posDisplay) < 2) {
            Serial.print("●");  // Position indicator
        } else {
            Serial.print("─");
        }
    }
    Serial.println(" ║");
    
    // Numeric position
    Serial.print("║           ");
    
    // Direction arrows
    if(position < -5.0) {
        Serial.print("<<<<<<< ");
    } else if(position < -2.0) {
        Serial.print("<<<< ");
    } else if(position < -0.5) {
        Serial.print("<< ");
    }
    
    // Numeric value
    Serial.printf("%.2f", position);
    
    // Direction arrows
    if(position > 5.0) {
        Serial.print(" >>>>>>>");
    } else if(position > 2.0) {
        Serial.print(" >>>>");
    } else if(position > 0.5) {
        Serial.print(" >>");
    }
    
    // Padding
    for(int i = 0; i < 30; i++) Serial.print(" ");
    Serial.println("║");
    
    // Active sensors
    int active = 0;
    for(int i = 0; i < 8; i++) {
        if(values[i] > 2000) active++;
    }
    
    Serial.print("║ Active: ");
    Serial.print(active);
    Serial.print("/8    Error: ");
    Serial.printf("%.2f", position);  // Error = position (when setpoint = 0)
    Serial.print("                        ║");
    Serial.println();
    
    Serial.println("╚═══════════════════════════════════════════════════╝\n");
    
    delay(200);
}