#pragma once

#include <Arduino.h>

// 挨拶の WAV（SPIFFS 上の /audio/*.wav）を非同期で再生する。ループを止めない。
// 再生中は音声の読み込みに使ったバッファを保持し、終わったら解放する。
class Greeting {
public:
    // 再生を始める。既に再生中、またはファイルが無ければ false。
    bool start(const char* path, uint8_t volume);
    // 定期的に呼ぶ。再生が終わったらバッファを解放する。
    void tick();
    bool playing() const { return buf_ != nullptr; }
    uint32_t durationMs() const { return duration_ms_; }  // 直近に開始した音声の長さ

private:
    uint8_t* buf_ = nullptr;
    uint32_t duration_ms_ = 0;
    uint32_t done_ms_ = 0;  // この時刻までは再生中とみなし、バッファを保持する
};
