// GyroscopeControl.cpp
#include "GyroscopeControl.h"

GyroscopeControl::GyroscopeControl() {
    calibrationOffset = 0;
    currentYaw = 0;
    isCalibrated = false;
}

bool GyroscopeControl::initialize() {
    // Initialize I2C with specific pins for ESP32
    Wire.begin(17, 16); // SDA=21, SCL=22 for ESP32
    Wire.setClock(400000); // Set I2C frequency to 400kHz
    
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    
    // Test connection with retry
    int retries = 3;
    while (retries > 0 && !mpu.testConnection()) {
        Serial.print("MPU6050 connection attempt failed, retries left: ");
        Serial.println(retries - 1);
        delay(1000);
        retries--;
    }
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed - gyroscope features disabled");
        return false;
    }
    
    // Configure MPU6050 settings
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // ±250°/s
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g
    mpu.setDLPFMode(MPU6050_DLPF_BW_20); // Low pass filter
    
    Serial.println("MPU6050 connected and configured successfully");
    calibrate();
    return true;
}

void GyroscopeControl::calibrate() {
    Serial.println("Calibrating gyroscope... Keep robot still!");
    float sum = 0;
    int samples = 200; // Increased samples for better calibration
    
    // Wait a bit for sensor to stabilize
    delay(500);
    
    for (int i = 0; i < samples; i++) {
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz);
        sum += gz;
        delay(5);
        
        // Show progress
        if (i % 50 == 0) {
            Serial.print("Calibration progress: ");
            Serial.print((i * 100) / samples);
            Serial.println("%");
        }
    }
    
    calibrationOffset = sum / samples;
    isCalibrated = true;
    resetYaw();
    
    Serial.print("Gyroscope calibrated. Offset: ");
    Serial.println(calibrationOffset);
}

void GyroscopeControl::resetYaw() {
    currentYaw = 0;
    Serial.println("Yaw reset to 0°");
}

float GyroscopeControl::getYaw() {
    return currentYaw;
}

void GyroscopeControl::update() {
    if (!isCalibrated) return;
    
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    
    // Convert to degrees/second (MPU6050 gyro sensitivity at ±250°/s is 131 LSB/°/s)
    // Note: Inverted sign to match robot's coordinate system
    float gyroZ = -(gz - calibrationOffset) / 131.0;
    
    // Integrate to get angle (assuming update is called every ~10ms)
    static unsigned long lastUpdateTime = 0;
    unsigned long currentTime = millis();
    
    if (lastUpdateTime > 0) {
        float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
        currentYaw += gyroZ * deltaTime;
    }
    
    lastUpdateTime = currentTime;
    
    // Keep yaw in -180 to 180 range
    while (currentYaw > 180) currentYaw -= 360;
    while (currentYaw < -180) currentYaw += 360;
}

bool GyroscopeControl::turnToAngle(float targetAngle, void (*motorLeftFunc)(), void (*motorRightFunc)(), void (*stopFunc)()) {
    if (!isCalibrated) {
        Serial.println("Gyroscope not calibrated, cannot perform precision turn");
        return false;
    }
    
    const float tolerance = 10.0; // Increased tolerance to prevent endless wiggling
    const unsigned long timeout = 5000; // Reduced timeout
    const unsigned long minTurnTime = 200; // Increased minimum turn time for smoother turns
    
    unsigned long startTime = millis();
    unsigned long lastTurnTime = 0;
    
    Serial.print("Starting gyro turn to ");
    Serial.print(targetAngle);
    Serial.println(" degrees");
    
    resetYaw(); // Reset yaw before turning
    
    while (millis() - startTime < timeout) {
        update();
        float error = targetAngle - currentYaw;
        
        // Normalize error to -180 to 180
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        Serial.print("Current yaw: ");
        Serial.print(currentYaw);
        Serial.print("°, Error: ");
        Serial.print(error);
        Serial.println("°");
        
        if (abs(error) < tolerance) {
            stopFunc();
            Serial.println("Target angle reached!");
            return true;
        }
        
        // Only turn if enough time has passed since last turn command AND error is significant
        if (millis() - lastTurnTime > minTurnTime && abs(error) > tolerance) {
            if (error > 0) {
                Serial.println("Turning right..."); // Fixed: positive error means we need to turn right
                motorRightFunc();
            } else {
                Serial.println("Turning left..."); // Fixed: negative error means we need to turn left  
                motorLeftFunc();
            }
            lastTurnTime = millis();
        } else if (abs(error) <= tolerance) {
            // Stop if we're close enough to prevent overshoot
            stopFunc();
        }
        
        delay(150); // Increased delay for better stability
    }
    
    stopFunc();
    Serial.println("Gyroscope turn timeout - target not reached");
    return false; // Timeout
}