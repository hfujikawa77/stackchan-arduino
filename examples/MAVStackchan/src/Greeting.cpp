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

    M5.Speaker.setVolume(volume);
    const size_t samples = (size - data_offset) / 2;
    if (!M5.Speaker.playRaw(reinterpret_cast<int16_t*>(buf + data_offset), samples, sample_rate, false, 1, 0)) {
        heap_caps_free(buf);
        return false;
    }
    buf_ = buf;  // 再生が終わるまで保持する
    return true;
}

void Greeting::tick() {
    if (!buf_) return;
    if (M5.Speaker.isPlaying()) return;
    heap_caps_free(buf_);
    buf_ = nullptr;
}
