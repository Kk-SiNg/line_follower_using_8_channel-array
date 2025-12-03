/*
 * Motors.cpp
 * Motor control with WiFi-tunable parameters
 */

#include "Motors.h"
#include <Arduino.h>

// Define global tunable parameters can be changed externally via wifi
int TICKS_FOR_90_DEG = 450;
int TICKS_FOR_180_DEG = 900;
int TICKS_TO_CENTER = 120;
int BASE_SPEED = 150;
int TURN_SPEED = 190;
int MAX_SPEED = 220;

Motors::Motors() {}

void Motors::setup() {
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN3, OUTPUT);
    pinMode(MOTOR_R_IN4, OUTPUT);
    
    ledcSetup(pwm_channel_left, pwm_frequency, pwm_resolution);
    ledcSetup(pwm_channel_right, pwm_frequency, pwm_resolution);
    
    ledcAttachPin(MOTOR_L_ENA, pwm_channel_left);
    ledcAttachPin(MOTOR_R_ENB, pwm_channel_right);
    
    puType pu_type = puType::up;
    ESP32Encoder::useInternalWeakPullResistors = pu_type;
    leftEncoder.attachHalfQuad(ENCODER_L_A, ENCODER_L_B);
    rightEncoder.attachHalfQuad(ENCODER_R_A, ENCODER_R_B);
    
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    
    stopBrake();
}

void Motors::setSpeeds(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Left Motor
    if (leftSpeed > 0) {
        digitalWrite(MOTOR_L_IN1, HIGH);
        digitalWrite(MOTOR_L_IN2, LOW);
    } 
    else if (leftSpeed < 0) {
        digitalWrite(MOTOR_L_IN1, LOW);
        digitalWrite(MOTOR_L_IN2, HIGH);
    } 
    else {
        digitalWrite(MOTOR_L_IN1, LOW);
        digitalWrite(MOTOR_L_IN2, LOW);
    }
    ledcWrite(pwm_channel_left, abs(leftSpeed));
    
    // Right Motor
    if (rightSpeed > 0) {
        digitalWrite(MOTOR_R_IN3, HIGH);
        digitalWrite(MOTOR_R_IN4, LOW);
    } else if (rightSpeed < 0) {
        digitalWrite(MOTOR_R_IN3, LOW);
        digitalWrite(MOTOR_R_IN4, HIGH);
    } else {
        digitalWrite(MOTOR_R_IN3, LOW);
        digitalWrite(MOTOR_R_IN4, LOW);
    }
    ledcWrite(pwm_channel_right, abs(rightSpeed));
}

void Motors::stopBrake() {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, HIGH);
    digitalWrite(MOTOR_R_IN3, HIGH);
    digitalWrite(MOTOR_R_IN4, HIGH);
    ledcWrite(pwm_channel_left, 0);
    ledcWrite(pwm_channel_right, 0);
}

void Motors::turn_90_left() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(-TURN_SPEED, TURN_SPEED);
    while (rightEncoder.getCount() < TICKS_FOR_90_DEG) delay(1);
    stopBrake();
}

void Motors::turn_90_right() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(TURN_SPEED, -TURN_SPEED);
    while (leftEncoder.getCount() < TICKS_FOR_90_DEG) delay(1);
    stopBrake();
}

void Motors::turn_180_back() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(TURN_SPEED, -TURN_SPEED);     //pivot from right
    while (leftEncoder.getCount() < TICKS_FOR_180_DEG) delay(1);
    stopBrake();
}

void Motors::rotate() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(-180, 180);
}

void Motors::moveForward(int ticks) {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(BASE_SPEED, BASE_SPEED);
    while ((leftEncoder.getCount() + rightEncoder.getCount()) / 2 < ticks) delay(1);
    stopBrake();
}

long Motors::getLeftCount() {
    return leftEncoder.getCount();
}

long Motors::getRightCount() {
    return rightEncoder.getCount();
}

long Motors::getAverageCount() {
    return (long)(leftEncoder.getCount() + rightEncoder.getCount()) / 2;
}

void Motors::clearEncoders() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}

void Motors::updateTurnTicks(int ticks90) {
    TICKS_FOR_90_DEG = constrain(ticks90, 50, 2000);
    TICKS_FOR_180_DEG = TICKS_FOR_90_DEG * 2;
}

void Motors::updateCenterTicks(int ticks) {
    TICKS_TO_CENTER = constrain(ticks, 10, 500);
}

void Motors::updateSpeeds(int base, int turn, int max) {
    BASE_SPEED = constrain(base, 50, 255);
    TURN_SPEED = constrain(turn, 50, 255);
    MAX_SPEED = constrain(max, 100, 255);
}