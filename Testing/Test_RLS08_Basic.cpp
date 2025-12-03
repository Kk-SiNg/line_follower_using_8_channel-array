#include <Arduino.h>

/*
 * Test_RLS08_Basic.ino
 * Basic test for 8-channel RLS-08 sensor array
 * Tests all 8 sensors individually
 */

// Pin definitions (match your main code)
#define SENSOR_PIN_1  25  // Digital
#define SENSOR_PIN_2  39  // Analog
#define SENSOR_PIN_3  34  // Analog
#define SENSOR_PIN_4  35  // Analog
#define SENSOR_PIN_5  32  // Analog
#define SENSOR_PIN_6  33  // Analog
#define SENSOR_PIN_7  36  // Analog
#define SENSOR_PIN_8  26  // Digital

const bool isAnalog[8] = {false, true, true, true, true, true, true, false};

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  RLS-08 Basic Sensor Test            ║");
    Serial.println("║  8-Channel Array                      ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    // Configure ADC for analog sensors
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    // Configure digital sensors
    pinMode(SENSOR_PIN_1, INPUT);
    pinMode(SENSOR_PIN_8, INPUT);
    
    Serial.println("✓ Pins configured");
    Serial.println("\nTest Instructions:");
    Serial.println("1. Place sensor over WHITE surface (line)");
    Serial.println("2. Observe HIGH readings");
    Serial.println("3. Place sensor over BLACK surface");
    Serial.println("4. Observe LOW readings\n");
    
    delay(2000);
}

void loop() {
    uint16_t values[8];
    
    // Read all sensors
    values[0] = digitalRead(SENSOR_PIN_1) ? 4095 : 0;
    values[1] = analogRead(SENSOR_PIN_2);
    values[2] = analogRead(SENSOR_PIN_3);
    values[3] = analogRead(SENSOR_PIN_4);
    values[4] = analogRead(SENSOR_PIN_5);
    values[5] = analogRead(SENSOR_PIN_6);
    values[6] = analogRead(SENSOR_PIN_7);
    values[7] = digitalRead(SENSOR_PIN_8) ? 4095 : 0;
    
    // Display sensor types
    Serial.println("╔═══╦═══╦═══╦═══╦═══╦═══╦═══╦═══╗");
    Serial.println("║ 1 ║ 2 ║ 3 ║ 4 ║ 5 ║ 6 ║ 7 ║ 8 ║");
    Serial.println("╠═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╣");
    
    // Display type
    Serial.print("║ D ║ A ║ A ║ A ║ A ║ A ║ A ║ D ║\n");
    Serial.println("╠═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╣");
    
    // Display values
    for(int i = 0; i < 8; i++) {
        Serial.print("║");
        if(isAnalog[i]) {
            // Show 4-digit analog value
            if(values[i] < 1000) Serial.print(" ");
            if(values[i] < 100) Serial.print(" ");
            if(values[i] < 10) Serial.print(" ");
            Serial.print(values[i]);
        } else {
            // Show ON/OFF for digital
            Serial.print(values[i] > 2000 ? " ON " : " OFF");
        }
    }
    Serial.println("║");
    Serial.println("╚═══╩═══╩═══╩═══╩═══╩═══╩═══╩═══╝");
    
    // Visual bar
    Serial.print("Visual: [");
    for(int i = 0; i < 8; i++) {
        Serial.print(values[i] > 2000 ? "█" : "·");
        if(i < 7) Serial.print("][");
    }
    Serial.println("]");
    
    // Count active
    int active = 0;
    for(int i = 0; i < 8; i++) {
        if(values[i] > 2000) active++;
    }
    Serial.print("Active sensors: ");
    Serial.print(active);
    Serial.println("/8");
    
    // Status
    if(active == 8) {
        Serial.println("✓ All sensors detecting (WHITE surface)");
    } else if(active == 0) {
        Serial.println("✓ No sensors detecting (BLACK surface)");
    } else {
        Serial.println("○ Partial detection (on LINE or edge)");
    }
    
    Serial.println("\n");
    delay(500);
}