#pragma once

#include <stdint.h>

// 機体の状態を、Stack-chan の表情・吹き出し・LED の色に変換する。Arduino 非依存。
//
// 表示の対応（最初の案。実機で見て調整する）:
//   キャリブレーション中  : 無表情   / 「キャリブレーション中」 / LED 白（暗め）
//   GCS 未接続            : 眠い     / 「接続待ち」             / LED 消灯
//   接続・DISARM・HOLD    : 無表情   / 「接続」                 / LED 緑（暗め）
//   接続・DISARM・MANUAL  : 無表情   / 「マニュアル」           / LED 緑（暗め）
//   ARM・HOLD             : 警戒     / 「アーム」               / LED 赤
//   ARM・MANUAL           : 嬉しい   / 「マニュアル操作」       / LED 青

enum class FaceExpression : uint8_t {
    Neutral,
    Happy,
    Angry,
    Sad,
    Doubt,
    Sleepy,
};

struct LedColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct StatusView {
    FaceExpression expression;
    const char* speech;  // 吹き出しの文字（UTF-8）
    LedColor led;        // 全 LED の色（消灯は 0,0,0）
};

StatusView statusViewFor(bool calibrating, bool gcs_linked, bool armed, bool manual_mode);
