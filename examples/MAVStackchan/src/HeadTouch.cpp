#include "HeadTouch.h"

void HeadTouch::begin() {
    sensor_.begin();
    swipe_fwd_ = false;
    swipe_back_ = false;
    in_gesture_ = false;
    for (int i = 0; i < 3; ++i) {
        intensities_[i] = 0;
        touched_[i] = false;
        touch_start_ms_[i] = 0;
    }
    prev_any_touched_ = false;
    tap_pending_ = 0;
    tap_count_result_ = 0;
    tap_finalized_ = false;
    swipe_seen_in_seq_ = false;
    last_release_ms_ = 0;
}

void HeadTouch::poll(uint32_t now) {
    sensor_.read_touch_result();
    sensor_.parse_touch_result();

    bool any_touched = false;
    for (int i = 0; i < 3; ++i) {
        intensities_[(3 - 1) - i] = sensor_.point_type[i];
        if (intensities_[(3 - 1) - i] > 0) any_touched = true;
    }
    for (int i = 0; i < 3; ++i) {
        if (intensities_[i] > 0 && !touched_[i]) {
            touched_[i] = true;
            touch_start_ms_[i] = now;
        }
    }

    // タップの数え方（立ち下がりを数える）。戻る前に処理する
    const bool falling = prev_any_touched_ && !any_touched;
    if (falling) {
        if (!swipe_seen_in_seq_) {
            tap_pending_++;
            last_release_ms_ = now;
        } else {
            swipe_seen_in_seq_ = false;
        }
    }
    if (tap_pending_ > 0 && !any_touched &&
        static_cast<int32_t>(now - last_release_ms_) >= static_cast<int32_t>(kTapQuietMs)) {
        tap_count_result_ = tap_pending_;
        tap_finalized_ = true;
        tap_pending_ = 0;
    }
    prev_any_touched_ = any_touched;

    if (!any_touched) {
        in_gesture_ = false;
        for (int i = 0; i < 3; ++i) touched_[i] = false;
        return;
    }

    // 3 か所が触れて、その順番で前後に動いたらスワイプ
    if (touched_[0] && touched_[1] && touched_[2] && !in_gesture_) {
        const int32_t t1_0 = static_cast<int32_t>(touch_start_ms_[1]) - static_cast<int32_t>(touch_start_ms_[0]);
        const int32_t t2_1 = static_cast<int32_t>(touch_start_ms_[2]) - static_cast<int32_t>(touch_start_ms_[1]);
        const int32_t t1_2 = static_cast<int32_t>(touch_start_ms_[1]) - static_cast<int32_t>(touch_start_ms_[2]);
        const int32_t t0_1 = static_cast<int32_t>(touch_start_ms_[0]) - static_cast<int32_t>(touch_start_ms_[1]);

        if (t1_0 > kSwipeMinIntervalMs && t2_1 > kSwipeMinIntervalMs &&
            t1_0 < kSwipeMaxIntervalMs && t2_1 < kSwipeMaxIntervalMs) {
            swipe_fwd_ = true;
            in_gesture_ = true;
            swipe_seen_in_seq_ = true;
            tap_pending_ = 0;
        } else if (t1_2 > kSwipeMinIntervalMs && t0_1 > kSwipeMinIntervalMs &&
                   t1_2 < kSwipeMaxIntervalMs && t0_1 < kSwipeMaxIntervalMs) {
            swipe_back_ = true;
            in_gesture_ = true;
            swipe_seen_in_seq_ = true;
            tap_pending_ = 0;
        }
    }
}

bool HeadTouch::takeSwipeForward() {
    const bool v = swipe_fwd_;
    swipe_fwd_ = false;
    return v;
}

bool HeadTouch::takeSwipeBackward() {
    const bool v = swipe_back_;
    swipe_back_ = false;
    return v;
}

uint8_t HeadTouch::takeTapCount() {
    if (!tap_finalized_) return 0;
    tap_finalized_ = false;
    const uint8_t n = tap_count_result_;
    tap_count_result_ = 0;
    return n;
}
