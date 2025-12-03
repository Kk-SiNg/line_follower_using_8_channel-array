#include <Arduino.h>

/*
 * Test_RLS08_Calibration.ino
 * Calibration test for analog sensors
 * Find optimal potentiometer settings
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

uint16_t minVals[8];
uint16_t maxVals[8];

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    pinMode(SENSOR_PIN_1, INPUT);
    pinMode(SENSOR_PIN_8, INPUT);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  RLS-08 Calibration Test              ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    // Initialize calibration
    for(int i = 0; i < 8; i++) {
        minVals[i] = 4095;
        maxVals[i] = 0;
    }
    
    Serial.println("Instructions:");
    Serial.println("1. Slowly move sensor over BLACK and WHITE");
    Serial.println("2. Watch Min/Max values stabilize");
    Serial.println("3. Range should be > 1000 for good contrast");
    Serial.println("4. Adjust potentiometers if needed\n");
    
    delay(3000);
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
    
    // Update min/max for analog sensors
    for(int i = 0; i < 8; i++) {
        if(isAnalog[i]) {
            if(values[i] < minVals[i]) minVals[i] = values[i];
            if(values[i] > maxVals[i]) maxVals[i] = values[i];
        }
    }
    
    // Display
    Serial.println("╔═══╦════════╦═══════╦═══════╦═══════╦═══════════╗");
    Serial.println("║ # ║ Current║  Min  ║  Max  ║ Range ║  Quality  ║");
    Serial.println("╠═══╬════════╬═══════╬═══════╬═══════╬═══════════╣");
    
    for(int i = 0; i < 8; i++) {
        Serial.print("║ ");
        Serial.print(i+1);
        Serial.print(" ║ ");
        
        if(isAnalog[i]) {
            // Current
            Serial.printf("%4d   ║ ", values[i]);
            // Min
            Serial.printf("%4d  ║ ", minVals[i]);
            // Max
            Serial.printf("%4d  ║ ", maxVals[i]);
            // Range
            uint16_t range = maxVals[i] - minVals[i];
            Serial.printf("%4d  ║ ", range);
            
            // Quality
            if(range > 2000) Serial.println("Excellent ║");
            else if(range > 1500) Serial.println("Very Good ║");
            else if(range > 1000) Serial.println("Good      ║");
            else if(range > 500) Serial.println("Fair      ║");
            else Serial.println("⚠ Poor    ║");
        } 
        else {
            Serial.println("  N/A  ║  N/A  ║  N/A  ║  N/A  ║ Digital   ║");
        }
    }
    
    Serial.println("╚═══╩════════╩═══════╩═══════╩═══════╩═══════════╝\n");
    
    delay(500);
}