// EmotionSystem.h
// This file should be located at: Kubit-PlatformIO-Project/lib/EmotionSystem/EmotionSystem.h

#ifndef EMOTION_SYSTEM_H
#define EMOTION_SYSTEM_H

#include <Arduino.h>
#include <functional>

enum EmotionType {
    EMOTION_NEUTRAL,
    EMOTION_HAPPY,
    EMOTION_EXCITED,
    EMOTION_WIGGLE,
    EMOTION_CONFUSED
};

class EmotionSystem {
public:
    EmotionSystem();
    
    bool addButtonToSequence(int buttonNumber);
    void updateEmotion(std::function<void()> moveForward, std::function<void()> moveBackward, std::function<void()> turnLeft, std::function<void()> turnRight, std::function<void()> stopMotors, std::function<void(const char*)> playAudio);
    bool isEmotionActive();
    void forceStopEmotion();
    EmotionType getCurrentEmotion();
    bool shouldActivateLineFollowing();
    void resetLineFollowingTrigger();

private:
    enum EmotionState {
        STATE_IDLE,
        STATE_ACTIVE,
        STATE_COOLDOWN
    };

    static const int MAX_SEQUENCE = 16;
    static const unsigned long SEQUENCE_TIMEOUT = 5000; // 5 seconds
    
    int buttonSequence[MAX_SEQUENCE];
    int sequenceIndex;
    unsigned long lastButtonTime;
    
    EmotionType currentEmotion;
    unsigned long emotionStartTime;
    unsigned long emotionDuration;
    bool emotionActive;
    bool emotionSoundPlayed;
    
    bool triggerLineFollowing;

    // Callbacks
    std::function<void()> m_stopMotorsCallback;
    std::function<void(const char*)> m_playAudioCallback;
    
    void clearSequence();
    bool checkEmotionTriggers();
    void triggerEmotion(EmotionType emotion, unsigned long duration);
    void printSequence();
    void updateCurrentState(EmotionState newState);

    EmotionState currentState;
    unsigned long stateStartTime;
    unsigned long cooldownDuration;
};

#endif