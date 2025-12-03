#include <Arduino.h>
/*
 * main.cpp
 * Simplified PID Line Follower with WiFi Tuning
 * 8-Channel RLS-08 Hybrid Sensor Array
 * 
 * FEATURES:
 * - Pure PID line following
 * - Real-time WiFi tuning of Kp, Ki, Kd
 * - Live telemetry over WiFi
 * - Simple start/stop control
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
    STOPPED
};
RobotState currentState = CALIBRATING;

// === Control Variables ===
bool robotRunning = false;
int currentSpeed = BASE_SPEED;
unsigned long lastWiFiUpdate = 0;
unsigned long lastDebugPrint = 0;

// === Emergency Stop ===
unsigned long buttonPressStart = 0;
const unsigned long LONG_PRESS_TIME = 2000;

// === Function Declarations ===
void setupWiFi();
void handleWiFiClient();
void processCommand(String cmd);
void printMenu();
void printStatus();
void printPIDValues();
void runPID(int baseSpeed);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  PID Line Follower with WiFi Tuning  ║");
    Serial. println("║  8-Channel RLS-08 Hybrid Array       ║");
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
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  Configuration                         ║");
    Serial.println("╠════════════════════════════════════════╣");
    Serial.print("║  Kp = ");
    Serial.print(pid.getKp(), 2);
    Serial.print("  Ki = ");
    Serial.print(pid.getKi(), 2);
    Serial.print("  Kd = ");
    Serial.print(pid.getKd(), 2);
    Serial.println("     ║");
    Serial.print("║  BASE_SPEED = ");
    Serial.print(BASE_SPEED);
    Serial. print("                      ║");
    Serial.println("\n╚════════════════════════════════════════╝\n");
    
    currentState = IDLE;
    Serial.println("✓ Ready!  Press button or type START via WiFi\n");
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
            client. print("] Err:");
            client.print(sensors.getLineError(), 2);
            client.print(" Speed:");
            client.print(currentSpeed);
            client.print(" | PID: Kp=");
            client. print(pid.getKp(), 1);
            client.print(" Ki=");
            client.print(pid.getKi(), 2);
            client.print(" Kd=");
            client. print(pid.getKd(), 1);
            client.print(" | State:");
            
            switch(currentState) {
                case IDLE: client.print("IDLE"); break;
                case RUNNING: client.print("RUNNING"); break;
                case STOPPED: client.print("STOPPED"); break;
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
            currentState = STOPPED;
            Serial.println("\n⚠️ EMERGENCY STOP!");
            if (client && client.connected()) {
                client.println("⚠️ EMERGENCY STOP!");
            }
            buttonPressStart = 0;
        }
    } else {
        buttonPressStart = 0;
    }
    
    // === Main State Machine ===
    switch (currentState) {
        
        case IDLE:
            // Wait for start command or button press
            if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                delay(50);
                if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                    Serial.println("\n>>> LINE FOLLOWING STARTED!");
                    if (client && client.connected()) {
                        client.println("\n>>> LINE FOLLOWING STARTED!");
                    }
                    
                    currentState = RUNNING;
                    robotRunning = true;
                    pid.reset();
                    motors.clearEncoders();
                    
                    // Wait for button release
                    while(digitalRead(USER_BUTTON) == LOW) delay(10);
                }
            }
            break;
            
        case RUNNING:
            if (! robotRunning) {
                motors.stopBrake();
                currentState = STOPPED;
                Serial. println("\n>>> STOPPED");
                if (client && client.connected()) {
                    client.println("\n>>> STOPPED");
                }
                break;
            }
            
            // Check if robot is on the line
            if (! sensors.onLine()) {
                Serial.println("⚠️ Line lost!");
                // Keep running with last known error for a bit
            }
            
            // Run PID control
            runPID(currentSpeed);
            
            // Detailed debug output (every 1 second)
            if (millis() - lastDebugPrint > 1000) {
                uint16_t analogVals[8];
                sensors. getAnalogArray(analogVals);
                
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
            
            break;
            
        case STOPPED:
            motors.stopBrake();
            // Blink LED
            digitalWrite(ONBOARD_LED, ! digitalRead(ONBOARD_LED));
            delay(500);
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

void setupWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(SSID, PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi. status() == WL_CONNECTED) {
        Serial.println(" Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        server.begin();
    } else {
        Serial.println(" Failed!");
        Serial.println("Continuing without WiFi...");
    }
}

void handleWiFiClient() {
    if (! client || !client.connected()) {
        client = server.available();
        if (client) {
            Serial.println("Client connected via WiFi");
            client.println("╔════════════════════════════════════════╗");
            client.println("║  PID Line Follower WiFi Console      ║");
            client. println("╚════════════════════════════════════════╝");
            printMenu();
        }
    }
    
    if (client && client.connected() && client.available()) {
        String cmd = client.readStringUntil('\n');
        cmd. trim();
        processCommand(cmd);
    }
}

void processCommand(String cmd) {
    cmd.toUpperCase();
    
    if (cmd == "START") {
        if (currentState == IDLE || currentState == STOPPED) {
            robotRunning = true;
            client.println("✓ START command received");
        } else {
            client.println("❌ Cannot start - already running");
        }
    }
    else if (cmd == "STOP") {
        robotRunning = false;
        motors.stopBrake();
        currentState = STOPPED;
        client.println("✓ STOP command received");
    }
    else if (cmd == "RESET") {
        currentState = IDLE;
        robotRunning = false;
        motors.stopBrake();
        pid.reset();
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
            Serial. print("WiFi: Kd = ");
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
            float ki = cmd.substring(firstSpace + 1, secondSpace). toFloat();
            float kd = cmd.substring(secondSpace + 1).toFloat();
            
            if (kp >= 0 && kp <= 200 && ki >= 0 && ki <= 50 && kd >= 0 && kd <= 200) {
                pid.setTunings(kp, ki, kd);
                client.println("✓ PID tuned:");
                client.print("  Kp = ");
                client. println(kp, 2);
                client.print("  Ki = ");
                client.println(ki, 2);
                client. print("  Kd = ");
                client.println(kd, 2);
                Serial.print("WiFi: PID = ");
                Serial.print(kp, 2);
                Serial.print(", ");
                Serial.print(ki, 2);
                Serial.print(", ");
                Serial.println(kd, 2);
            } else {
                client. println("❌ Invalid values");
            }
        } else {
            client.println("❌ Usage: TUNE <kp> <ki> <kd>");
        }
    }
    else if (cmd. startsWith("SPEED ")) {
        int value = cmd.substring(6).toInt();
        if (value >= 50 && value <= 255) {
            currentSpeed = value;
            client.print("✓ Base speed set to ");
            client. println(value);
            Serial.print("WiFi: Speed = ");
            Serial.println(value);
        } else {
            client.println("❌ Invalid speed (range: 50-255)");
        }
    }
    else {
        client.println("❌ Unknown command.  Type HELP for commands.");
    }
}

void printMenu() {
    client.println("\n=== Commands ===");
    client.println("START           - Start line following");
    client.println("STOP            - Stop robot");
    client.println("RESET           - Reset to idle state");
    client.println("STATUS          - Show current status");
    client.println("GETPID          - Display PID values");
    client. println("CAL             - Recalibrate sensors");
    client.println("");
    client.println("=== PID Tuning ===");
    client.println("KP <value>      - Set Kp (0-200)");
    client.println("KI <value>      - Set Ki (0-50)");
    client.println("KD <value>      - Set Kd (0-200)");
    client.println("TUNE <kp> <ki> <kd> - Set all three");
    client.println("SPEED <value>   - Set base speed (50-255)");
    client.println("");
    client. println("HELP            - This menu");
    client.println("================\n");
}

void printStatus() {
    client.println("\n=== Status ===");
    client.print("State: ");
    switch(currentState) {
        case IDLE: client.println("IDLE"); break;
        case RUNNING: client.println("RUNNING"); break;
        case STOPPED: client.println("STOPPED"); break;
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
    client. println("==============\n");
}

void printPIDValues() {
    client.println("\n=== PID Values ===");
    client. print("Kp = ");
    client.println(pid.getKp(), 3);
    client.print("Ki = ");
    client.println(pid.getKi(), 3);
    client.print("Kd = ");
    client. println(pid.getKd(), 3);
    client.print("Output = ");
    client.println(pid.getOutput(), 2);
    client.println("==================\n");
}