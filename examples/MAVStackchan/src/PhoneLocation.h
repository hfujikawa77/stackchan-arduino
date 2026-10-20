#pragma once

#include <Arduino.h>

// スマホの最後の位置（Cloudflare Worker の GET /location から取得）。
// 機体の GPS ではない。鮮度の判定はしない（方針として考慮不要）。
struct PhoneLocation {
    bool valid = false;
    double lat = 0.0;
    double lon = 0.0;
};

// 位置の取得タスクを起動する。Wi-Fi 接続中に 60 秒ごとに取得する。
// base_url / token が空なら何もしない。
void startPhoneLocationTask(const String& base_url, const String& token);

// 最新の位置をコピーして返す。取得前・失敗が続いた場合は valid = false（前の値は残す）。
bool latestPhoneLocation(PhoneLocation& out);
