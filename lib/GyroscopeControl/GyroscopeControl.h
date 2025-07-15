// GyroscopeControl.h
#ifndef GYROSCOPE_CONTROL_H
#define GYROSCOPE_CONTROL_H

#include <Wire.h>
#include <MPU6050.h>

class GyroscopeControl {
private:
    MPU6050 mpu;
    float calibrationOffset;
    float currentYaw;
    bool isCalibrated;
    
public:
    GyroscopeControl();
    bool initialize();
    void calibrate();
    void resetYaw();
    float getYaw();
    void update();
    bool turnToAngle(float targetAngle, void (*motorLeftFunc)(), void (*motorRightFunc)(), void (*stopFunc)());
};

#endif
