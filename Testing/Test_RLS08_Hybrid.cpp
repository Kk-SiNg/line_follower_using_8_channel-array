#include <Arduino.h>

/*
 * Test_RLS08_Hybrid.ino
 * Complete test of 6 analog + 2 digital configuration
 * Simulates full line following behavior
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

uint16_t minVals[8];
uint16_t maxVals[8];
bool calibrated = false;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    pinMode(SENSOR_PIN_1, INPUT);
    pinMode(SENSOR_PIN_8, INPUT);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  RLS-08 Complete Hybrid Test         ║");
    Serial.println("║  6 Analog + 2 Digital                 ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    // Initialize calibration
    for(int i = 0; i < 8; i++) {
        minVals[i] = 4095;
        maxVals[i] = 0;
    }
    
    Serial.println("AUTO-CALIBRATION:");
    Serial.println("Move sensor over BLACK and WHITE for 5 seconds...");
    Serial.println();
    
    unsigned long startTime = millis();
    while(millis() - startTime < 5000) {
        uint16_t values[8];
        
        values[0] = digitalRead(SENSOR_PIN_1) ? 4095 : 0;
        values[1] = analogRead(SENSOR_PIN_2);
        values[2] = analogRead(SENSOR_PIN_3);
        values[3] = analogRead(SENSOR_PIN_4);
        values[4] = analogRead(SENSOR_PIN_5);
        values[5] = analogRead(SENSOR_PIN_6);
        values[6] = analogRead(SENSOR_PIN_7);
        values[7] = digitalRead(SENSOR_PIN_8) ? 4095 : 0;
        
        for(int i = 0; i < 8; i++) {
            if(isAnalog[i]) {
                if(values[i] < minVals[i]) minVals[i] = values[i];
                if(values[i] > maxVals[i]) maxVals[i] = values[i];
            }
        }
        
        if((millis() - startTime) % 1000 < 50) {
            Serial.print(".");
        }
        
        delay(50);
    }
    
    Serial.println("\n\n✓ Calibration Complete!\n");
    
    // Display calibration results
    Serial.println("Calibration Results:");
    Serial.println("S# | Type    | Min  | Max  | Range | Quality");
    Serial.println("---|---------|------|------|-------|----------");
    
    for(int i = 0; i < 8; i++) {
        Serial.printf("%d  | ", i+1);
        
        if(isAnalog[i]) {
            uint16_t range = maxVals[i] - minVals[i];
            Serial.printf("Analog  | %4d | %4d | %4d  | ", minVals[i], maxVals[i], range);
            
            if(range > 1500) Serial.println("Excellent");
            else if(range > 1000) Serial.println("Good");
            else if(range > 500) Serial.println("Fair");
            else Serial.println("⚠ Poor");
        } else {
            Serial.println("Digital | N/A  | N/A  | N/A   | N/A");
        }
    }
    
    calibrated = true;
    
    Serial.println("\n✓ Starting continuous monitoring...\n");
    delay(2000);
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
    bool digital[8];
    
    // Read sensors
    values[0] = digitalRead(SENSOR_PIN_1) ? 4095 : 0;
    values[1] = analogRead(SENSOR_PIN_2);
    values[2] = analogRead(SENSOR_PIN_3);
    values[3] = analogRead(SENSOR_PIN_4);
    values[4] = analogRead(SENSOR_PIN_5);
    values[5] = analogRead(SENSOR_PIN_6);
    values[6] = analogRead(SENSOR_PIN_7);
    values[7] = digitalRead(SENSOR_PIN_8) ? 4095 : 0;
    
    // Convert to digital
    for(int i = 0; i < 8; i++) {
        digital[i] = (values[i] > 2000);
    }
    
    // Calculate position
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
    float error = position;  // Error when setpoint = 0
    
    // Count active
    int activeCount = 0;
    for(int i = 0; i < 8; i++) {
        if(digital[i]) activeCount++;
    }
    
    // Detect junction
    bool isJunction = false;
    bool leftPath = false;
    bool straightPath = false;
    bool rightPath = false;
    
    if(activeCount >= 5) {
        leftPath = (digital[6] || digital[7]);
        rightPath = (digital[0] || digital[1]);
        straightPath = (digital[3] || digital[4]);
        isJunction = true;
    }
    
    // === COMPREHENSIVE DISPLAY ===
    Serial.println("\n╔═══════════════════════════════════════════════════════════╗");
    Serial.println("║              RLS-08 REAL-TIME MONITORING                  ║");
    Serial.println("╠═══════════════════════════════════════════════════════════╣");
    
    // Sensor type header
    Serial.print("║ Type:    [");
    for(int i = 0; i < 8; i++) {
        Serial.print(isAnalog[i] ? " A " : " D ");
        if(i < 7) Serial.print("][");
    }
    Serial.println("]                    ║");
    
    // Analog values
    Serial.print("║ Analog:  [");
    for(int i = 0; i < 8; i++) {
        if(isAnalog[i]) {
            Serial.printf("%4d", values[i]);
        } else {
            Serial.print(values[i] > 2000 ? " ON " : " OFF");
        }
        if(i < 7) Serial.print("][");
    }
    Serial.println("] ║");
    
    // Digital representation
    Serial.print("║ Digital: [");
    for(int i = 0; i < 8; i++) {
        Serial.print(digital[i] ? " █ " : " · ");
        if(i < 7) Serial.print("][");
    }
    Serial.println("]                    ║");
    
    Serial.println("╠═══════════════════════════════════════════════════════════╣");
    
    // Position and error
    Serial.printf("║ Position: %.2f", position);
    Serial.print("    Error: ");
    Serial.printf("%.2f", error);
    
    // Direction indicator
    Serial.print("    ");
    if(error < -3.0) Serial.print("<<<<<<");
    else if(error < -1.0) Serial.print("<<<");
    else if(error < -0.3) Serial.print("<");
    else if(error > 3.0) Serial.print(">>>>>>");
    else if(error > 1.0) Serial.print(">>>");
    else if(error > 0.3) Serial.print(">");
    else Serial.print("CENTER");
    
    Serial.println("         ║");
    
    // Active sensors
    Serial.printf("║ Active: %d/8    ", activeCount);
    Serial.print("[");
    for(int i = 0; i < 8; i++) {
        Serial.print(i < activeCount ? "●" : "○");
    }
    Serial.println("]                       ║");
    
    Serial.println("╠═══════════════════════════════════════════════════════════╣");
    
    // Junction status
    Serial.print("║ Junction: ");
    if(isJunction) {
        Serial.print("YES    Paths: ");
        if(leftPath) Serial.print("[L]");
        if(straightPath) Serial.print("[S]");
        if(rightPath) Serial.print("[R]");
    } else {
        Serial.print("NO     (Normal line following)");
    }
    
    // Padding
    for(int i = 0; i < 30; i++) Serial.print(" ");
    Serial.println("║");
    
    // Simulated PID output
    float Kp = 45.0;
    float pidOutput = Kp * error;
    int baseSpeed = 150;
    int leftMotor = baseSpeed + (int)pidOutput;
    int rightMotor = baseSpeed - (int)pidOutput;
    
    leftMotor = constrain(leftMotor, -255, 255);
    rightMotor = constrain(rightMotor, -255, 255);
    
    Serial.printf("║ PID Output: %.1f", pidOutput);
    Serial.print("    Motors: L=");
    Serial.printf("%3d", leftMotor);
    Serial.print(" R=");
    Serial.printf("%3d", rightMotor);
    Serial.println("                ║");
    
    Serial.println("╚═══════════════════════════════════════════════════════════╝");
    
    delay(300);
}