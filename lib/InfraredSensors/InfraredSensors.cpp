// InfraredSensors.cpp - Fixed ADC Driver Conflict
#include "InfraredSensors.h"

InfraredSensors::InfraredSensors(int frontLeft, int frontRight, int back) {
    frontLeftPin = frontLeft;
    frontRightPin = frontRight;
    backPin = back;
    lineThreshold = 2000;
    fallThreshold = 1000;
    
    // Safer ADC initialization without deprecated functions
    // The ESP32 should automatically handle ADC configuration
    Serial.println("ADC initialized for infrared sensors");
}

void InfraredSensors::setThresholds(int lineThresh, int fallThresh) {
    lineThreshold = lineThresh;
    fallThreshold = fallThresh;
    Serial.print("IR Thresholds set - Line: ");
    Serial.print(lineThreshold);
    Serial.print(", Fall: ");
    Serial.println(fallThreshold);
}

bool InfraredSensors::isLineDetectedLeft() {
    // Add small delay to prevent ADC reading conflicts
    delay(1);
    int reading = analogRead(frontLeftPin);
    return reading > lineThreshold;
}

bool InfraredSensors::isLineDetectedRight() {
    // Add small delay to prevent ADC reading conflicts  
    delay(1);
    int reading = analogRead(frontRightPin);
    return reading > lineThreshold;
}

bool InfraredSensors::isFalling() {
    // Add small delay to prevent ADC reading conflicts
    delay(1);
    int reading = analogRead(backPin);
    return reading < fallThreshold;
}

int InfraredSensors::getFrontLeftReading() {
    delay(1);
    return analogRead(frontLeftPin);
}

int InfraredSensors::getFrontRightReading() {
    delay(1);
    return analogRead(frontRightPin);
}

int InfraredSensors::getBackReading() {
    delay(1);
    return analogRead(backPin);
}

void InfraredSensors::printReadings() {
    int leftReading = getFrontLeftReading();
    delay(2); // Small delay between readings
    int rightReading = getFrontRightReading();
    delay(2);
    int backReading = getBackReading();
    
    Serial.print("IR Sensors - Left: ");
    Serial.print(leftReading);
    Serial.print(" (");
    Serial.print(leftReading > lineThreshold ? "LINE" : "NO LINE");
    Serial.print("), Right: ");
    Serial.print(rightReading);
    Serial.print(" (");
    Serial.print(rightReading > lineThreshold ? "LINE" : "NO LINE");
    Serial.print("), Back: ");
    Serial.print(backReading);
    Serial.print(" (");
    Serial.print(backReading < fallThreshold ? "FALLING" : "SAFE");
    Serial.println(")");
}