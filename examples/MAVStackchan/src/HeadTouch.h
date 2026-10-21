#pragma once

#include <Arduino.h>

#include "Si12T.h"

// 頭上の静電タッチ（Si12T）から、スワイプ（前後）と 2 回タップを判定する。
// HermesLLM の HeadTouchSensor と同じ判定。ループから定期的に poll() を呼ぶ。
class HeadTouch {
public:
    void begin();
    void poll(uint32_t now_ms);

    // 取り出し型: 1 回読むと消える
    bool takeSwipeForward();
    bool takeSwipeBackward();
    bool takeDoubleTap();

private:
    Si12T sensor_ = Si12T(SI12T_Type_High, SI12T_Sensitivity_Level_4);
    uint8_t intensities_[3] = {0, 0, 0};
    uint32_t touch_start_ms_[3] = {0, 0, 0};
    bool touched_[3] = {false, false, false};
    bool in_gesture_ = false;
    bool swipe_fwd_ = false;
    bool swipe_back_ = false;
    bool prev_any_touched_ = false;
    uint8_t tap_pending_ = 0;
    uint8_t tap_count_result_ = 0;
    bool tap_finalized_ = false;
    bool swipe_seen_in_seq_ = false;
    uint32_t last_release_ms_ = 0;

    static constexpr uint32_t kTapQuietMs = 400;
    static constexpr int32_t kSwipeMinIntervalMs = 30;
    static constexpr int32_t kSwipeMaxIntervalMs = 400;
};
