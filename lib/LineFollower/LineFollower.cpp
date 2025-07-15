// LineFollower.cpp
#include "LineFollower.h"

LineFollower::LineFollower(InfraredSensors* irSensors) {
    sensors = irSensors;
    isActive = false;
    lineFollowingMode = false;
    lastLineTime = 0;
    baseSpeed = 200; // Default speed
    turnSpeed = 150; // Default turn speed
}

void LineFollower::activate() {
    isActive = true;
    lineFollowingMode = true;
    lastLineTime = millis();
    Serial.println("Line following activated");
}

void LineFollower::deactivate() {
    isActive = false;
    lineFollowingMode = false;
    Serial.println("Line following deactivated");
}

bool LineFollower::isLineFollowingActive() {
    return isActive && lineFollowingMode;
}

void LineFollower::setSpeed(int base, int turn) {
    baseSpeed = base;
    turnSpeed = turn;
}

void LineFollower::followLine(void (*moveForwardFunc)(), void (*turnLeftFunc)(), void (*turnRightFunc)(), void (*stopFunc)()) {
    if (!isActive) return;
    
    // Add small delay to prevent ADC reading conflicts
    delayMicroseconds(100);
    
    // Check for falling first (safety)
    if (sensors->isFalling()) {
        Serial.println("EMERGENCY: Robot detected falling edge!");
        emergencyStop(moveForwardFunc, stopFunc);
        return;
    }
    
    bool leftLine = sensors->isLineDetectedLeft();
    bool rightLine = sensors->isLineDetectedRight();
    
    // Debug output
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        Serial.print("Line following - Left: ");
        Serial.print(leftLine ? "YES" : "NO");
        Serial.print(", Right: ");
        Serial.println(rightLine ? "YES" : "NO");
        lastDebug = millis();
    }
    
    if (leftLine && rightLine) {
        // Both sensors detect line - go forward
        Serial.println("Following line forward");
        moveForwardFunc();
        lastLineTime = millis();
    } else if (leftLine && !rightLine) {
        // Left sensor detects line - turn left to follow
        Serial.println("Correcting left");
        turnLeftFunc();
        lastLineTime = millis();
    } else if (!leftLine && rightLine) {
        // Right sensor detects line - turn right to follow
        Serial.println("Correcting right");
        turnRightFunc();
        lastLineTime = millis();
    } else {
        // No line detected
        if (millis() - lastLineTime > 3000) {
            // Lost line for too long - stop
            Serial.println("Line lost for too long - stopping line following");
            stopFunc();
            deactivate();
        } else {
            // Continue previous movement briefly to find line
            Serial.println("Searching for line...");
            moveForwardFunc();
        }
    }
    
    // Small delay for stability
    delay(50);
}

void LineFollower::emergencyStop(void (*moveForwardFunc)(), void (*stopFunc)()) {
    Serial.println("EMERGENCY: Robot is approaching edge! Emergency maneuver!");
    
    // Stop first
    stopFunc();
    delay(100);
    
    // Move forward briefly to get away from edge
    moveForwardFunc();
    delay(300);
    
    // Stop again
    stopFunc();
    
    // Deactivate line following for safety
    deactivate();
    Serial.println("Line following deactivated due to emergency stop");
}