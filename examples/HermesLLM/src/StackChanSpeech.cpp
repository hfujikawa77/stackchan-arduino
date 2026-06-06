#include "StackChanSpeech.h"

void StackChanSpeech::begin(StackChanSpeechHandler handler) {
    handler_ = handler;
}

bool StackChanSpeech::speak(const String& text) {
    Serial.println("[StackChanSpeech] " + text);
    if (!handler_) return true;

    const bool ok = handler_(text);
    if (!ok) {
        Serial.println("[StackChanSpeech] speech handler failed");
    }
    return ok;
}
