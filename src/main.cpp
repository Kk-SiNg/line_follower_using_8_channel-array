#include <Arduino.h>
/*
 * main.cpp
 * PID Line Follower with Junction Navigation & WiFi Tuning
 * 8-Channel RLS-08 Hybrid Sensor Array
 * 
 * FEATURES:
 * - PID line following
 * - Junction detection and LSRB navigation
 * - Real-time WiFi tuning of Kp, Ki, Kd
 * - Single continuous run (no path optimization)
 * - Live telemetry over WiFi
 */

#include <WiFi.h>
#include "Pins.h"
#include "Sensors.h"
#include "Motors.h"
#include "PIDController.h"

// === WiFi Server ===
WiFiServer server(TELNET_PORT);
WiFiClient client;

// === Global Objects ===
Sensors sensors;
Motors motors;
PIDController pid(45.0, 0.5, 25.0);  // Initial Kp, Ki, Kd values

// === Robot State Machine ===
enum RobotState {
    CALIBRATING,
    IDLE,
    RUNNING,
    FINISHED
};
RobotState currentState = CALIBRATING;

// === Control Variables ===
bool robotRunning = false;
int currentSpeed = BASE_SPEED;
unsigned long lastWiFiUpdate = 0;
unsigned long lastDebugPrint = 0;

// === Junction Detection Variables ===
unsigned long lastJunctionTime = 0;
unsigned long junctionDebounce = 300;  // Debounce time between junctions
int junctionCount = 0;  // Just for counting, not storing

// === Line End Detection ===
unsigned long lineEndStartTime = 0;
const unsigned long LINE_END_CONFIRM_TIME = 150;  // 150ms confirmation

// === Emergency Stop ===
unsigned long buttonPressStart = 0;
const unsigned long LONG_PRESS_TIME = 2000;

// === Performance Tracking ===
unsigned long runStartTime = 0;

// === Function Declarations ===
void setupWiFi();
void handleWiFiClient();
void processCommand(String cmd);
void printMenu();
void printStatus();
void printPIDValues();
void runPID(int baseSpeed);
String junctionTypeToString(JunctionType type);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  PID Maze Solver with WiFi Tuning     ║");
    Serial.println("║  8-Channel RLS-08 Hybrid Array        ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    pinMode(ONBOARD_LED, OUTPUT);
    pinMode(USER_BUTTON, INPUT_PULLUP);
    
    motors.setup();
    setupWiFi();
    
    // Calibrate sensors while rotating
    Serial.println("Starting sensor calibration...");
    currentState = CALIBRATING;
    motors.rotate();
    sensors.setup();
    motors.stopBrake();
    
    pid.setOutputLimits(-BASE_SPEED, BASE_SPEED);
    
    // Serial.println("\n╔════════════════════════════════════════╗");
    // Serial.println("║  Configuration                         ║");
    // Serial.println("╠════════════════════════════════════════╣");
    // Serial.print("║  Kp = ");
    // Serial.print(pid.getKp(), 2);
    // Serial.print("  Ki = ");
    // Serial.print(pid.getKi(), 2);
    // Serial.print("  Kd = ");
    // Serial.print(pid.getKd(), 2);
    // Serial.println("     ║");
    // Serial.print("║  BASE_SPEED = ");
    // Serial.print(BASE_SPEED);
    // Serial.print("                      ║");
    // Serial.println("\n║  Junction Debounce = ");
    // Serial.print(junctionDebounce);
    // Serial.println("ms        ║");
    // Serial.println("╚════════════════════════════════════════╝\n");
    
    currentState = IDLE;
    // Serial.println("✓ Ready!  Press button or type START via WiFi\n");
}

void loop() {
    // Give WiFi time to process
    yield();
    
    handleWiFiClient();
    
    // === WiFi Telemetry (every 500ms) ===
    if (millis() - lastWiFiUpdate > 500 && client && client.connected()) {
        if (client.availableForWrite() > 100) {
            bool sensorVals[8];
            sensors.getSensorArray(sensorVals);
            
            client.print("S:[");
            for (int i = 0; i < 8; i++) {
                client.print(sensorVals[i] ? "█" : "·");
            }
            client.print("] Err:");
            client.print(sensors.getLineError(), 2);
            client.print(" Speed:");
            client.print(currentSpeed);
            client.print(" | PID: Kp=");
            client.print(pid.getKp(), 1);
            client.print(" Ki=");
            client.print(pid.getKi(), 2);
            client.print(" Kd=");
            client.print(pid.getKd(), 1);
            client.print(" | State:");
            
            switch(currentState) {
                case IDLE: client.print("IDLE"); break;
                case RUNNING: 
                    client.print("RUNNING");
                    client.print(" Junc:");
                    client.print(junctionCount);
                    break;
                case FINISHED: client.print("FINISHED"); break;
                default: client.print("CALIBRATING");
            }
            
            client.println();
            client.flush();
        }
        
        lastWiFiUpdate = millis();
    }
    
    // === Emergency Stop (long press button) ===
    if (digitalRead(USER_BUTTON) == LOW) {
        if (buttonPressStart == 0) {
            buttonPressStart = millis();
        } else if (millis() - buttonPressStart > LONG_PRESS_TIME) {
            motors.stopBrake();
            robotRunning = false;
            currentState = FINISHED;
            Serial.println("\n⚠️ EMERGENCY STOP!");
            if (client && client.connected()) {
                client.println("⚠️ EMERGENCY STOP!");
            }
            buttonPressStart = 0;
        }
    } 
    else {
        buttonPressStart = 0;
    }
    
    // === Main State Machine ===
    switch (currentState) {
        
        case IDLE:
            // Wait for start command or button press
            if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                delay(50);
                if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                    Serial.println("\n╔════════════════════════════════════════╗");
                    Serial.println("║     MAZE SOLVING STARTED              ║");
                    Serial.println("╚════════════════════════════════════════╝\n");
                    
                    if (client && client.connected()) {
                        client.println("\n>>> MAZE SOLVING STARTED!");
                    }
                    
                    currentState = RUNNING;
                    robotRunning = true;
                    junctionCount = 0;
                    lastJunctionTime = 0;
                    lineEndStartTime = 0;
                    runStartTime = millis();
                    pid.reset();
                    motors.clearEncoders();
                    
                    // Wait for button release
                    while(digitalRead(USER_BUTTON) == LOW) delay(10);
                }
            }
            break;
            
        case RUNNING:
        {
            if (! robotRunning) {
                motors.stopBrake();
                currentState = FINISHED;
                Serial.println("\n>>> STOPPED");
                if (client && client.connected()) {
                    client.println("\n>>> STOPPED");
                }
                break;
            }
            
            // === Check for FINISH WHITE SQUARE FIRST (Priority!) ===
            if (sensors. isEndPoint()) {
                if (lineEndStartTime == 0) {
                    lineEndStartTime = millis();
                    Serial. println("⚠️ Finish square detected - confirming.. .");
                    // Keep moving slowly to center on square
                } 
                else if (millis() - lineEndStartTime > LINE_END_CONFIRM_TIME) {
                    // Confirmed finish square! 
                    motors.stopBrake();
                    robotRunning = false;
                    
                    unsigned long runTime = (millis() - runStartTime) / 1000;
                    
                    Serial.println("\n╔════════════════════════════════════════╗");
                    Serial.println("║                                        ║");
                    Serial.println("║      🏆  MAZE COMPLETE!  🏆            ║");
                    Serial.println("║                                        ║");
                    Serial.println("║   IIT Bombay Mesmerize Complete!     ║");
                    Serial. println("║                                        ║");
                    Serial.println("╚════════════════════════════════════════╝");
                    Serial.print("\n⏱️  Time: ");
                    Serial.print(runTime);
                    Serial.println(" seconds");
                    Serial.print("🔀 Junctions: ");
                    Serial.println(junctionCount);
                    
                    if (client && client.connected()) {
                        client.println("\n╔════════════════════════════════════════╗");
                        client. println("║      🏆  MAZE COMPLETE!  🏆            ║");
                        client.println("║   IIT Bombay Mesmerize Complete!     ║");
                        client.println("╚════════════════════════════════════════╝");
                        client.print("Time: ");
                        client.print(runTime);
                        client.println("s");
                        client.print("Junctions: ");
                        client.println(junctionCount);
                    }
                    
                    currentState = FINISHED;
                    lineEndStartTime = 0;
                    break;
                }
            } else {
                // Not on finish square - reset timer
                lineEndStartTime = 0;
            }
            
            // === Optional: Check for line loss (dead end) ===
            // Uncomment if you want to handle complete line loss
            /*
            if (sensors.isLineEnd()) {
                Serial.println("⚠️ Line completely lost - possible dead end");
                // Could implement recovery or turn-back logic here
            }
            */
            
            // Run PID control
            runPID(currentSpeed);
            
            // === Check for Line End FIRST ===
            if (sensors.isLineEnd()) {
                if (lineEndStartTime == 0) {
                    lineEndStartTime = millis();
                    motors.stopBrake();  // Stop immediately
                } 
                else if (millis() - lineEndStartTime > LINE_END_CONFIRM_TIME) {
                    // Confirmed line end - maze complete! 
                    motors.stopBrake();
                    robotRunning = false;
                    
                    unsigned long runTime = (millis() - runStartTime) / 1000;
                    
                    // Serial.println("\n╔════════════════════════════════════════╗");
                    // Serial.println("║                                        ║");
                    // Serial.println("║      🏆  MAZE COMPLETE!  🏆            ║");
                    // Serial.println("║                                        ║");
                    // Serial.println("╚════════════════════════════════════════╝");
                    // Serial.print("Time: ");
                    // Serial.print(runTime);
                    // Serial.println(" seconds");
                    // Serial.print("Junctions navigated: ");
                    // Serial.println(junctionCount);
                    
                    if (client && client.connected()) {
                        client.println("\n╔════════════════════════════════════════╗");
                        client.println("║      🏆  MAZE COMPLETE!  🏆            ║");
                        client.println("╚════════════════════════════════════════╝");
                        client.print("Time: ");
                        client.print(runTime);
                        client.println("s");
                        client.print("Junctions: ");
                        client.println(junctionCount);
                    }
                    
                    currentState = FINISHED;
                    lineEndStartTime = 0;
                    break;
                }
            } else {
                // Line detected - reset timer
                lineEndStartTime = 0;
            }
            
            // Run PID control
            runPID(currentSpeed);
            
            // === Detailed Debug Output (every 1 second) ===
            if (millis() - lastDebugPrint > 1000) {
                uint16_t analogVals[8];
                sensors.getAnalogArray(analogVals);
                
                Serial.print("Analog: ");
                for(int i = 0; i < 8; i++) {
                    Serial.print(analogVals[i]);
                    Serial.print("\t");
                }
                
                float pos = sensors.getPosition();
                float err = sensors.getLineError();
                
                Serial.print("| Pos:");
                Serial.print(pos, 2);
                Serial.print(" Err:");
                Serial.print(err, 2);
                Serial.print(" Speed:");
                Serial.println(currentSpeed);
                
                lastDebugPrint = millis();
            }
            
            // === Junction Detection ===
            if (millis() - lastJunctionTime > junctionDebounce) {
                PathOptions paths = sensors.getAvailablePaths();
                
                // Count available paths
                int pathCount = 0;
                if (paths.left) pathCount++;
                if (paths.right) pathCount++;
                if (paths.straight) pathCount++;
                
                // Junction = more than just straight, OR only left/right (90° turn)
                bool isJunction = false;
                if (pathCount > 1 || (pathCount == 1 && ! paths.straight)) {
                    isJunction = true;
                }
                
                if (isJunction) {
                    motors.stopBrake();
                    
                    junctionCount++;
                    
                    // Classify junction type
                    JunctionType jType = sensors.classifyJunction(paths);
                    
                    Serial.print("Junction ");
                    Serial.print(junctionCount);
                    Serial.print(": ");
                    Serial.print(junctionTypeToString(jType));
                    Serial.print(" (");
                    if(paths.left) Serial.print("L");
                    if(paths.straight) Serial.print("S");
                    if(paths.right) Serial.print("R");
                    Serial.print(") Ticks:");
                    Serial.println(motors.getAverageCount());
                    
                    if (client && client.connected()) {
                        client.print("Junction ");
                        client.print(junctionCount);
                        client.print(": ");
                        if(paths.left) client.print("L");
                        if(paths.straight) client.print("S");
                        if(paths.right) client.print("R");
                        client.println();
                    }
                    
                    // Move to center of junction
                    motors.moveForward(TICKS_TO_CENTER);
                    delay(100);
                    
                    // === LSRB Logic (Left > Straight > Right > Back) ===
                    if (paths.left) {
                        Serial.println("  → Taking LEFT");
                        if (client && client.connected()) {
                            client.println("  → LEFT");
                        }
                        motors.turn_90_left();
                    }
                    else if (paths.straight) {
                        Serial.println("  → Going STRAIGHT");
                        if (client && client.connected()) {
                            client.println("  → STRAIGHT");
                        }
                        // No turn needed
                    }
                    else if (paths.right) {
                        Serial.println("  → Taking RIGHT");
                        if (client && client.connected()) {
                            client.println("  → RIGHT");
                        }
                        motors.turn_90_right();
                    }
                    else {
                        // Dead end - turn back
                        Serial.println("  → DEAD END - Turning back");
                        if (client && client.connected()) {
                            client.println("  → DEAD END - BACK");
                        }
                        motors.turn_180_back();
                    }
                    
                    motors.clearEncoders();
                    pid.reset();
                    lastJunctionTime = millis();
                    delay(100);
                }
            }
            
            break;
        }
            
        case FINISHED:
            // Victory blink
            digitalWrite(ONBOARD_LED, ! digitalRead(ONBOARD_LED));
            delay(200);
            break;
            
        case CALIBRATING:
            break;
    }
}

void runPID(int baseSpeed) {
    float error = sensors.getLineError();
    
    // Compute PID correction
    float correction = pid.compute(error);
    
    // Apply correction to motor speeds
    int leftSpeed = baseSpeed + (int)correction;
    int rightSpeed = baseSpeed - (int)correction;
    
    motors.setSpeeds(leftSpeed, rightSpeed);
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
        default: return "Unknown";
    }
}

void setupWiFi() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  WiFi Connection Setup                ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.print("SSID: ");
    Serial.println(SSID);
    Serial.print("Connecting");
    
    WiFi.mode(WIFI_STA);  // Station mode
    WiFi.begin(SSID, PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {  // 20 seconds
        delay(500);
        Serial.print(".");
        attempts++;
        
        // Print connection status
        if (attempts % 10 == 0) {
            Serial.println();
            Serial.print("Status: ");
            switch(WiFi.status()) {
                case WL_IDLE_STATUS:
                    Serial. print("IDLE");
                    break;
                case WL_NO_SSID_AVAIL:
                    Serial.print("NO SSID AVAILABLE");
                    break;
                case WL_CONNECT_FAILED:
                    Serial.print("CONNECT FAILED");
                    break;
                case WL_CONNECTION_LOST:
                    Serial.print("CONNECTION LOST");
                    break;
                case WL_DISCONNECTED:
                    Serial.print("DISCONNECTED");
                    break;
                default:
                    Serial.print("UNKNOWN");
            }
            Serial.print(" - Retrying");
        }
    }
    
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✓ WiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal Strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        Serial.print("Connect via: telnet ");
        Serial.println(WiFi.localIP());
        Serial.println();
        server.begin();
    } else {
        Serial.println("\n❌ WiFi Connection Failed!");
        Serial.print("Final Status: ");
        switch(WiFi.status()) {
            case WL_NO_SSID_AVAIL:
                Serial.println("Network not found");
                Serial.println("→ Check SSID spelling");
                Serial.println("→ Make sure it's 2.4GHz WiFi");
                break;
            case WL_CONNECT_FAILED:
                Serial.println("Connection failed");
                Serial.println("→ Check password");
                break;
            default:
                Serial.println("Unknown error");
        }
        Serial.println("Continuing without WiFi...\n");
    }
}

void handleWiFiClient() {
    if (! client || !client.connected()) {
        client = server.available();
        if (client) {
            Serial.println("Client connected via WiFi");
            client.println("╔════════════════════════════════════════╗");
            client.println("║  PID Maze Solver WiFi Console         ║");
            client.println("╚════════════════════════════════════════╝");
            printMenu();
        }
    }
    
    if (client && client.connected() && client.available()) {
        String cmd = client.readStringUntil('\n');
        cmd.trim();
        processCommand(cmd);
    }
}

void processCommand(String cmd) {
    cmd.toUpperCase();
    
    if (cmd == "START" || cmd == "ST") {
        if (currentState == IDLE || currentState == FINISHED) {
            robotRunning = true;
            client.println("✓ START command received");
        } else {
            client.println("❌ Cannot start - already running");
        }
    }
    else if (cmd == "S" || cmd == "STOP") {
        robotRunning = false;
        motors.stopBrake();
        currentState = FINISHED;
        client.println("✓ STOP command received");
    }
    else if (cmd == "RESET") {
        currentState = IDLE;
        robotRunning = false;
        motors.stopBrake();
        pid.reset();
        junctionCount = 0;
        client.println("✓ System RESET - Ready to start");
    }
    else if (cmd == "STATUS") {
        printStatus();
    }
    else if (cmd == "GETPID" || cmd == "PID") {
        printPIDValues();
    }
    else if (cmd == "CAL") {
        client.println("Recalibrating sensors...");
        sensors.calibrate();
        client.println("✓ Calibration complete");
    }
    else if (cmd == "HELP") {
        printMenu();
    }
    // === PID TUNING COMMANDS ===
    else if (cmd.startsWith("KP ")) {
        float value = cmd.substring(3).toFloat();
        if (value >= 0 && value <= 200) {
            pid.setTunings(value, pid.getKi(), pid.getKd());
            client.print("✓ Kp set to ");
            client.println(value, 2);
            Serial.print("WiFi: Kp = ");
            Serial.println(value, 2);
        } else {
            client.println("❌ Invalid Kp (range: 0-200)");
        }
    }
    else if (cmd.startsWith("KI ")) {
        float value = cmd.substring(3).toFloat();
        if (value >= 0 && value <= 50) {
            pid.setTunings(pid.getKp(), value, pid.getKd());
            client.print("✓ Ki set to ");
            client.println(value, 2);
            Serial.print("WiFi: Ki = ");
            Serial.println(value, 2);
        } else {
            client.println("❌ Invalid Ki (range: 0-50)");
        }
    }
    else if (cmd.startsWith("KD ")) {
        float value = cmd.substring(3).toFloat();
        if (value >= 0 && value <= 200) {
            pid.setTunings(pid.getKp(), pid.getKi(), value);
            client.print("✓ Kd set to ");
            client.println(value, 2);
            Serial.print("WiFi: Kd = ");
            Serial.println(value, 2);
        } else {
            client.println("❌ Invalid Kd (range: 0-200)");
        }
    }
    else if (cmd.startsWith("TUNE ")) {
        // Parse "TUNE kp ki kd"
        int firstSpace = cmd.indexOf(' ', 5);
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        
        if (firstSpace > 0 && secondSpace > 0) {
            float kp = cmd.substring(5, firstSpace).toFloat();
            float ki = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            float kd = cmd.substring(secondSpace + 1).toFloat();
            
            if (kp >= 0 && kp <= 200 && ki >= 0 && ki <= 50 && kd >= 0 && kd <= 200) {
                pid.setTunings(kp, ki, kd);
                client.println("✓ PID tuned:");
                client.print("  Kp = ");
                client.println(kp, 2);
                client.print("  Ki = ");
                client.println(ki, 2);
                client.print("  Kd = ");
                client.println(kd, 2);
                Serial.print("WiFi: PID = ");
                Serial.print(kp, 2);
                Serial.print(", ");
                Serial.print(ki, 2);
                Serial.print(", ");
                Serial.println(kd, 2);
            } else {
                client.println("❌ Invalid values");
            }
        } else {
            client.println("❌ Usage: TUNE <kp> <ki> <kd>");
        }
    }
    else if (cmd.startsWith("SPEED ")) {
        int value = cmd.substring(6).toInt();
        if (value >= 50 && value <= 255) {
            currentSpeed = value;
            client.print("✓ Base speed set to ");
            client.println(value);
            Serial.print("WiFi: Speed = ");
            Serial.println(value);
        } else {
            client.println("❌ Invalid speed (range: 50-255)");
        }
    }
    else if (cmd.startsWith("DEBOUNCE ")) {
        int value = cmd.substring(9).toInt();
        if (value >= 100 && value <= 1000) {
            junctionDebounce = value;
            client.print("✓ Junction debounce set to ");
            client.print(value);
            client.println("ms");
            Serial.print("WiFi: Debounce = ");
            Serial.println(value);
        } else {
            client.println("❌ Invalid debounce (range: 100-1000ms)");
        }
    }
    else {
        client.println("❌ Unknown command. Type HELP for commands.");
    }
}

void printMenu() {
    client.println("\n=== Commands ===");
    client.println("START           - Start maze solving");
    client.println("STOP            - Stop robot");
    client.println("RESET           - Reset to idle state");
    client.println("STATUS          - Show current status");
    client.println("GETPID          - Display PID values");
    client.println("CAL             - Recalibrate sensors");
    client.println("");
    client.println("=== PID Tuning ===");
    client.println("KP <value>      - Set Kp (0-200)");
    client.println("KI <value>      - Set Ki (0-50)");
    client.println("KD <value>      - Set Kd (0-200)");
    client.println("TUNE <kp> <ki> <kd> - Set all three");
    client.println("SPEED <value>   - Set base speed (50-255)");
    client.println("DEBOUNCE <ms>   - Junction debounce (100-1000)");
    client.println("");
    client.println("HELP            - This menu");
    client.println("================\n");
}

void printStatus() {
    client.println("\n=== Status ===");
    client.print("State: ");
    switch(currentState) {
        case IDLE: client.println("IDLE"); break;
        case RUNNING: client.println("RUNNING"); break;
        case FINISHED: client.println("FINISHED"); break;
        case CALIBRATING: client.println("CALIBRATING"); break;
    }
    client.print("Error: ");
    client.println(sensors.getLineError(), 2);
    client.print("Position: ");
    client.println(sensors.getPosition(), 2);
    client.print("On Line: ");
    client.println(sensors.onLine() ? "YES" : "NO");
    client.print("Active Sensors: ");
    client.println(sensors.getActiveSensorCount());
    client.print("Base Speed: ");
    client.println(currentSpeed);
    client.print("Junctions: ");
    client.println(junctionCount);
    client.print("Debounce: ");
    client.print(junctionDebounce);
    client.println("ms");
    client.println("==============\n");
}

void printPIDValues() {
    client.println("\n=== PID Values ===");
    client.print("Kp = ");
    client.println(pid.getKp(), 3);
    client.print("Ki = ");
    client.println(pid.getKi(), 3);
    client.print("Kd = ");
    client.println(pid.getKd(), 3);
    client.print("Output = ");
    client.println(pid.getOutput(), 2);
    client.println("==================\n");
}