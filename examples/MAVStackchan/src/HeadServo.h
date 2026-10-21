#pragma once

#include <stdint.h>

#include "ManualControl.h"
#include "Stackchan_servo.h"

// Stack-chan 内蔵の頭部サーボ（パン・チルト）。HermesLLM と同じ StackchanSERVO を使う。
// 設定（ピン・中立角・可動範囲）は system_config（SC_BasicConfig.yaml）から読む。
// 新しいサーボは追加しない。
class HeadServo {
public:
    // system_config を読み込んだ後に呼ぶ。サーボが無い構成では false を返す。
    bool begin();
    bool ready() const { return ready_; }

    // 内蔵 LED の制御用（頭部サーボと同じ IO 拡張）。サーボ未使用なら nullptr。
    m5::PY32IOExpander_Class* ioExpander() { return ready_ ? servo_.getIOExpander() : nullptr; }

    HeadLimits panLimits() const;
    HeadLimits tiltLimits() const;

    // 目標角を設定する（度）。範囲外の値は可動範囲に収める。
    void setTarget(float pan, float tilt);

    // 定期的に呼ぶ。目標が動いたときだけ、一定の間隔でサーボへ送る。
    void update(uint32_t now_ms);

    // スライドの動き用: 指定の角度へ、指定の時間で 1 回だけ動かす（範囲内に収める）。
    void moveTo(float pan, float tilt, uint32_t ms);
    // スライドの回転（スピン）。開始と終了。終了で中立へ戻す。
    void spinBegin();
    void spinEnd();

private:
    StackchanSERVO servo_;
    bool ready_ = false;
    float target_pan_ = 0.0f;
    float target_tilt_ = 0.0f;
    float last_pan_ = 0.0f;
    float last_tilt_ = 0.0f;
    uint32_t next_send_ms_ = 0;
};
