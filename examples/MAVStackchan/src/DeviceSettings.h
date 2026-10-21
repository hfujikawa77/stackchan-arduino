#pragma once

#include <Arduino.h>
#include <stdint.h>

// 本体の設定画面で変える項目。SPIFFS の /settings.json に保存する。
// 秘密情報（SSID・トークン）は含まない。SC_SecConfig.yaml の値を既定値として使い、
// 設定画面で変えた値がそれを上書きする。
struct DeviceSettings {
    bool slide_mode = false;   // スライドモード
    uint8_t volume = 80;       // 挨拶などの音量（0〜100）
    uint8_t brightness = 200;  // 画面の明るさ（0〜255）
    uint8_t wifi_index = 0;    // 接続に使う wifi_list の番号
};

// /settings.json を読む。無ければ false（既定値のまま）。
bool loadDeviceSettings(DeviceSettings& out);
// /settings.json に保存する。
bool saveDeviceSettings(const DeviceSettings& in);
