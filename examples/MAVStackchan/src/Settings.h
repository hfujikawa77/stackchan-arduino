#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// 設定は SPIFFS の /yaml/SC_SecConfig.yaml から読む（HermesLLM と同じ場所・同じファイル名）。
// 秘密情報を含むため、このファイル名はルートの .gitignore（**/SC_SecConfig.yaml）で除外される。

constexpr size_t kWifiMaxEntries = 8;

struct WifiEntry {
    String ssid;
    String password;
};

struct MavSettings {
    WifiEntry wifi[kWifiMaxEntries];
    size_t wifi_count = 0;     // wifi_list: の件数（先頭から順に接続を試す）
    uint8_t system_id = 1;     // mav: system_id（指示書 4.1 の既定値 1）
    uint16_t udp_port = 14550; // mav: udp_port（MAVLink 2 の標準 UDP ポート）
};

// YAML 本文を解析して out に反映する。
// 対応書式（HermesLLM と同じ 2 段インデント）:
//   wifi_list:
//     - ssid: "xxx"
//       password: "yyy"
//   mav:
//     system_id: 1
//     udp_port: 14550
// 値の中の # は、" で囲まれていればコメントとして扱わない。
void parseSettingsYaml(const String& yaml, MavSettings& out);
