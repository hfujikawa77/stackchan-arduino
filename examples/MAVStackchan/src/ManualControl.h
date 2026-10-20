#pragma once

#include <stdint.h>

// QGC のバーチャルスティック（MANUAL_CONTROL の x / y）を、頭部サーボの目標角に変換する。
// Arduino 非依存。
//
// 割り当て（仮定。実機で QGC から動かして確認し、違えば入れ替える）:
//   MANUAL_CONTROL.x（前後スティック）→ 頭部の上下（tilt, Y 軸）
//   MANUAL_CONTROL.y（左右スティック）→ 頭部の左右（pan,  X 軸）
//
// 角度は config の center / lower_limit / upper_limit を超えない。

struct HeadLimits {
    float center;  // 中立角（度）
    float lower;   // 可動範囲の下限（度）
    float upper;   // 可動範囲の上限（度）
};

struct HeadTarget {
    float pan;   // X 軸（度）
    float tilt;  // Y 軸（度）
};

// スティック入力の有効範囲（MAVLink の MANUAL_CONTROL は -1000〜1000）
constexpr int16_t kStickMax = 1000;
// 中央付近の不感帯（正規化値）
constexpr float kStickDeadzone = 0.05f;

// 入力値を -1.0〜1.0 に正規化する。範囲外（INT16_MAX などの未定義値を含む）は 0 にする。
float normalizeStick(int16_t raw);

// 正規化値を角度に変換する。中立から上限側・下限側へ、それぞれの範囲で線形に変換する。
float stickToDegrees(float normalized, const HeadLimits& limits);

// スティックの x / y から頭部の目標角を求める。
HeadTarget manualControlToHead(int16_t stick_x, int16_t stick_y,
                               const HeadLimits& pan_limits, const HeadLimits& tilt_limits);
