#include <Arduino.h>
/*
 * Test_RLS08_Junction.ino
 * Test junction detection and classification
 * Verify no false positives on straight lines
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

enum JunctionType {
    JUNCTION_NONE,
    JUNCTION_T_LEFT,
    JUNCTION_T_RIGHT,
    JUNCTION_T_BOTH,
    JUNCTION_CROSS,
    JUNCTION_90_LEFT,
    JUNCTION_90_RIGHT,
    JUNCTION_DEAD_END
};

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    pinMode(SENSOR_PIN_1, INPUT);
    pinMode(SENSOR_PIN_8, INPUT);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  RLS-08 Junction Detection Test      ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    Serial.println("Test different configurations:");
    Serial.println("✓ Straight line (should NOT detect junction)");
    Serial.println("✓ T-junction left  (├)");
    Serial.println("✓ T-junction right (┤)");
    Serial.println("✓ Cross junction   (┼)");
    Serial.println("✓ 90° turns        (└ ┘)");
    Serial.println("✓ Dead end\n");
    
    delay(3000);
}

String junctionTypeToString(JunctionType type) {
    switch(type) {
        case JUNCTION_T_LEFT: return "T-Left ├";
        case JUNCTION_T_RIGHT: return "T-Right ┤";
        case JUNCTION_T_BOTH: return "T-Both ┬";
        case JUNCTION_CROSS: return "Cross ┼";
        case JUNCTION_90_LEFT: return "90° Left └";
        case JUNCTION_90_RIGHT: return "90° Right ┘";
        case JUNCTION_DEAD_END: return "Dead End";
        case JUNCTION_NONE: return "Straight Line";
        default: return "Unknown";
    }
}

JunctionType classifyJunction(bool left, bool straight, bool right) {
    int pathCount = (left ? 1 : 0) + (straight ? 1 : 0) + (right ? 1 : 0);
    
    if(pathCount == 0) return JUNCTION_DEAD_END;
    if(pathCount == 1) {
        if(left && !straight) return JUNCTION_90_LEFT;
        if(right && !straight) return JUNCTION_90_RIGHT;
        return JUNCTION_NONE;
    }
    if(pathCount == 2) {
        if(left && straight) return JUNCTION_T_LEFT;
        if(right && straight) return JUNCTION_T_RIGHT;
        if(left && right) return JUNCTION_T_BOTH;
    }
    if(pathCount == 3) return JUNCTION_CROSS;
    
    return JUNCTION_NONE;
}

void loop() {
    uint16_t values[8];
    bool digital[8];
    
    // Read all sensors
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
    
    // Count active sensors
    int activeCount = 0;
    for(int i = 0; i < 8; i++) {
        if(digital[i]) activeCount++;
    }
    
    // Detect paths (using improved logic)
    bool leftPath = false;
    bool straightPath = false;
    bool rightPath = false;
    
    if(activeCount >= 5) {
        // Junction detection
        leftPath = (digital[6] || digital[7]);      // S7 or S8
        rightPath = (digital[0] || digital[1]);      // S1 or S2
        straightPath = (digital[3] || digital[4]);   // S4 or S5
    } else {
        // Normal line following
        straightPath = (activeCount > 0);
    }
    
    // Classify junction
    JunctionType jType = classifyJunction(leftPath, straightPath, rightPath);
    
    // Display
    Serial.println("╔═════════════════════════════════════════════════════╗");
    
    // Sensor visualization
    Serial.print("║ Sensors: [");
    for(int i = 0; i < 8; i++) {
        Serial.print(digital[i] ? "█" : "·");
        if(i < 7) Serial.print("][");
    }
    Serial.println("]                    ║");
    
    // Raw values (for debugging)
    Serial.print("║ Raw:     [");
    for(int i = 0; i < 8; i++) {
        Serial.printf("%4d", values[i]);
        if(i < 7) Serial.print("][");
    }
    Serial.println("] ║");
    
    // Active count
    Serial.print("║ Active: ");
    Serial.print(activeCount);
    Serial.print("/8");
    
    // Progress bar
    Serial.print("  [");
    for(int i = 0; i < 8; i++) {
        Serial.print(i < activeCount ? "●" : "○");
    }
    Serial.println("]                       ║");
    
    // Path detection
    Serial.print("║ Paths: ");
    if(leftPath) Serial.print("[LEFT] ");
    if(straightPath) Serial.print("[STRAIGHT] ");
    if(rightPath) Serial.print("[RIGHT] ");
    if(!leftPath && !straightPath && !rightPath) Serial.print("[NONE]");
    
    // Padding
    for(int i = 0; i < 30; i++) Serial.print(" ");
    Serial.println("║");
    
    // Junction classification
    Serial.print("║ Type: ");
    String typeStr = junctionTypeToString(jType);
    Serial.print(typeStr);
    for(int i = typeStr.length(); i < 45; i++) Serial.print(" ");
    Serial.println("║");
    
    // LSRB Decision
    Serial.print("║ LSRB Decision: ");
    if(jType == JUNCTION_NONE || jType == JUNCTION_DEAD_END) {
        if(leftPath) Serial.print("Turn LEFT");
        else if(straightPath) Serial.print("Go STRAIGHT");
        else if(rightPath) Serial.print("Turn RIGHT");
        else Serial.print("Turn BACK (B)");
    } else {
        if(leftPath) Serial.print("Turn LEFT (L)");
        else if(straightPath) Serial.print("Go STRAIGHT (S)");
        else if(rightPath) Serial.print("Turn RIGHT (R)");
        else Serial.print("Turn BACK (B)");
    }
    
    // Padding
    for(int i = 0; i < 25; i++) Serial.print(" ");
    Serial.println("║");
    
    // Warning if false junction detected
    if(activeCount < 5 && jType != JUNCTION_NONE) {
        Serial.println("║ ⚠️  WARNING: Junction detected with <5 sensors!     ║");
        Serial.println("║     This might be a FALSE POSITIVE                  ║");
    }
    
    if(activeCount >= 5 && jType == JUNCTION_NONE) {
        Serial.println("║ ⚠️  WARNING: Many sensors active but no junction    ║");
        Serial.println("║     Check junction detection logic                  ║");
    }
    
    Serial.println("╚═════════════════════════════════════════════════════╝\n");
    
    delay(400);
}