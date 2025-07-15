// InfraredSensors.h
#ifndef INFRARED_SENSORS_H
#define INFRARED_SENSORS_H

#include <Arduino.h>

class InfraredSensors {
private:
    int frontLeftPin;
    int frontRightPin;
    int backPin;
    int lineThreshold;
    int fallThreshold;
    
public:
    InfraredSensors(int frontLeft, int frontRight, int back);
    void setThresholds(int lineThresh, int fallThresh);
    bool isLineDetectedLeft();
    bool isLineDetectedRight();
    bool isFalling();
    int getFrontLeftReading();
    int getFrontRightReading();
    int getBackReading();
    void printReadings();
};

#endif
