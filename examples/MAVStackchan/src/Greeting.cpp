#include "Greeting.h"

#include <M5Unified.h>
#include <SPIFFS.h>
#include <esp_heap_caps.h>

bool Greeting::start(const char* path, uint8_t volume) {
    if (buf_ || !SPIFFS.exists(path)) return false;

    File file = SPIFFS.open(path, "r");
    if (!file) return false;
    const size_t size = file.size();
    if (size < 44) {
        file.close();
        return false;
    }
    uint8_t* buf = static_cast<uint8_t*>(heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!buf) {
        file.close();
        return false;
    }
    const size_t got = file.read(buf, size);
    file.close();
    if (got != size) {
        heap_caps_free(buf);
        return false;
    }

    // WAV ヘッダから、データの開始位置とサンプルレートを取る（HermesLLM の play_wav と同じ）
    uint32_t data_offset = 44;
    uint32_t sample_rate = 24000;
    if (memcmp(buf, "RIFF", 4) == 0) {
        sample_rate = *reinterpret_cast<uint32_t*>(buf + 24);
        uint32_t pos = 12;
        while (pos + 8 <= size) {
            if (memcmp(buf + pos, "data", 4) == 0) {
                data_offset = pos + 8;
                break;
            }
            const uint32_t chunk = *reinterpret_cast<uint32_t*>(buf + pos + 4);
            pos += 8 + ((chunk + 1) & ~1u);
        }
    }
    if (data_offset >= size || sample_rate == 0) {
        heap_caps_free(buf);
        return false;
    }

    M5.Speaker.setVolume(static_cast<uint8_t>(volume * 255 / 100));  // 設定は 0〜100、スピーカーは 0〜255
    const size_t samples = (size - data_offset) / 2;
    const bool ok = M5.Speaker.playRaw(reinterpret_cast<int16_t*>(buf + data_offset), samples, sample_rate, false, 1, 0);
    Serial.printf("[SLIDE] playRaw=%d samples=%u rate=%u\n", ok ? 1 : 0, (unsigned)samples, (unsigned)sample_rate);
    if (!ok) {
        heap_caps_free(buf);
        return false;
    }
    // 音声の長さ + 余裕。isPlaying() が直後は false を返すこともあるため、時間でも保持する
    duration_ms_ = static_cast<uint32_t>(samples * 1000ULL / sample_rate);
    done_ms_ = millis() + duration_ms_ + 300;
    buf_ = buf;
    return true;
}

void Greeting::tick() {
    if (!buf_) return;
    if (millis() < done_ms_ || M5.Speaker.isPlaying()) return;
    heap_caps_free(buf_);
    buf_ = nullptr;
}
