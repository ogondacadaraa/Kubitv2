// EmotionSystem.cpp
// This file should be located at: Kubit-PlatformIO-Project/lib/EmotionSystem/EmotionSystem.cpp

#include "EmotionSystem.h"

EmotionSystem::EmotionSystem() {
    currentEmotion = EMOTION_NEUTRAL;
    emotionStartTime = 0;
    emotionDuration = 0;
    emotionActive = false;
    sequenceIndex = 0;
    lastButtonTime = 0;
    triggerLineFollowing = false;
    emotionSoundPlayed = false;
    clearSequence();
}

bool EmotionSystem::addButtonToSequence(int buttonNumber) {
    unsigned long currentTime = millis();
    if (currentTime - lastButtonTime > SEQUENCE_TIMEOUT) {
        sequenceIndex = 0; // Timeout, reset sequence
        Serial.println("[Emotion] Sequence timed out, starting new sequence.");
    }
    
    if (sequenceIndex < MAX_SEQUENCE) {
        buttonSequence[sequenceIndex++] = buttonNumber;
        lastButtonTime = currentTime;
        printSequence(); // Log the current sequence
        return checkEmotionTriggers();
    }
    return false;
}

void EmotionSystem::updateEmotion(std::function<void()> moveForward, std::function<void()> moveBackward, std::function<void()> turnLeft, std::function<void()> turnRight, std::function<void()> stopMotors, std::function<void(const char*)> playAudio) {
    m_stopMotorsCallback = stopMotors;
    m_playAudioCallback = playAudio;

    if (emotionActive && millis() - emotionStartTime > emotionDuration) {
        Serial.printf("[Emotion] Emotion '%d' finished.\n", currentEmotion);
        forceStopEmotion();
    }
}

void EmotionSystem::forceStopEmotion() {
    if (isEmotionActive())
    {
        emotionActive = false;
        emotionSoundPlayed = false;
        currentEmotion = EMOTION_NEUTRAL;
        clearSequence();
        if (m_stopMotorsCallback) {
            m_stopMotorsCallback();
        }
        Serial.println("[Emotion] Emotion forced to stop.");
    }
}

void EmotionSystem::clearSequence() {
    sequenceIndex = 0;
    for (int i = 0; i < MAX_SEQUENCE; ++i) {
        buttonSequence[i] = 0;
    }
    Serial.println("Button sequence cleared");
}

bool EmotionSystem::checkEmotionTriggers() {
    // Simple check for 3 identical presses
    if (sequenceIndex == 3 && buttonSequence[0] == buttonSequence[1] && buttonSequence[1] == buttonSequence[2]) {
        switch (buttonSequence[0]) {
            case 1: // Forward button
                Serial.println("[Emotion] Happy trigger detected.");
                triggerEmotion(EMOTION_HAPPY, 3000);
                return true;
            case 2: // Backward button
                Serial.println("[Emotion] Wiggle trigger detected.");
                triggerEmotion(EMOTION_WIGGLE, 2500);
                return true;
        }
    }

    // Example: Confused (Left, Right, Left)
    if (sequenceIndex == 3 && buttonSequence[0] == 3 && buttonSequence[1] == 4 && buttonSequence[2] == 3) {
        Serial.println("[Emotion] Confused trigger detected.");
        triggerEmotion(EMOTION_CONFUSED, 4000);
        return true;
    }

    return false;
}

void EmotionSystem::triggerEmotion(EmotionType emotion, unsigned long duration) {
    Serial.printf("[Emotion] Triggering emotion '%d' for %lu ms.\n", emotion, duration);
    currentEmotion = emotion;
    emotionDuration = duration;
    emotionStartTime = millis();
    emotionActive = true;
    emotionSoundPlayed = false; // Reset sound flag for the new emotion
    clearSequence();
}

void EmotionSystem::printSequence() {
    String seq = "[Emotion] Current sequence: ";
    for (int i = 0; i < sequenceIndex; ++i) {
        seq += String(buttonSequence[i]);
        if (i < sequenceIndex - 1) {
            seq += ", ";
        }
    }
    Serial.println(seq);
}

EmotionType EmotionSystem::getCurrentEmotion() {
    return currentEmotion;
}

bool EmotionSystem::isEmotionActive() {
    return emotionActive;
}

void EmotionSystem::updateCurrentState(EmotionState newState) {
    currentState = newState;
    Serial.printf("[Emotion] Current state updated: %d\n", newState);
}