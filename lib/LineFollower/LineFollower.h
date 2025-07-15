// LineFollower.h
#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "InfraredSensors.h"

class LineFollower {
private:
    InfraredSensors* sensors;
    bool isActive;
    bool lineFollowingMode;
    unsigned long lastLineTime;
    int baseSpeed;
    int turnSpeed;
    
public:
    LineFollower(InfraredSensors* irSensors);
    void activate();
    void deactivate();
    bool isLineFollowingActive();
    void setSpeed(int base, int turn);
    void followLine(void (*moveForwardFunc)(), void (*turnLeftFunc)(), void (*turnRightFunc)(), void (*stopFunc)());
    void emergencyStop(void (*moveForwardFunc)(), void (*stopFunc)());
};

#endif

