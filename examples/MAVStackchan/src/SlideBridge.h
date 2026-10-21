#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>

// reveal.js からの接続を受ける WebSocket（GET /servo-ws）。HermesLLM の /servo-ws と同じ仕様。
//   受信: {"type":"slide",...} / {"type":"ping"}（ブラウザ → 機体）
//   送信: {"type":"control","action":"next"|"prev"}（頭のスワイプ → ブラウザ）
// 同時に接続できるのは 1 台。90 秒無通信で切断する。
class SlideBridge {
public:
    using Handler = void (*)(const char* json, void* ctx);

    void begin(uint16_t port);
    // 接続の受け付けと受信を行う。受信した JSON テキストは handler に渡す。
    void tick(uint32_t now_ms, Handler handler, void* ctx);
    // テキストを 1 フレームで送る（125 バイト以内）。接続が無ければ false。
    bool sendText(const char* text);

    bool connected();
    void stop();

private:
    bool handshake(WiFiClient& client);
    bool readExact(uint8_t* out, size_t len, uint32_t timeout_ms = 20);

    WiFiServer server_{80};
    WiFiClient client_;
    bool active_ = false;
    uint32_t last_ms_ = 0;
};
