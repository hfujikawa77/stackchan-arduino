#pragma once

#include <Arduino.h>

typedef bool (*StackChanSpeechHandler)(const String& text);

class StackChanSpeech {
public:
    void begin(StackChanSpeechHandler handler = nullptr);
    bool speak(const String& text);

private:
    StackChanSpeechHandler handler_ = nullptr;
};
