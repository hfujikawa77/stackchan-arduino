#include "HeadServo.h"

#include <math.h>

#include "Stackchan_system_config.h"

// 設定本体は main.cpp で定義する（HermesLLM と同じ構成）
extern StackchanSystemConfig system_config;

namespace {

constexpr uint32_t kSendIntervalMs = 50;   // 最大 20Hz で送る
constexpr uint32_t kMoveMs = 60;           // 1 回の移動時間（連続追従用）
constexpr float kMinChangeDeg = 0.5f;      // これより小さい変化は送らない

}  // namespace

bool HeadServo::begin() {
    auto* sx = system_config.getServoInfo(AXIS_X);
    auto* sy = system_config.getServoInfo(AXIS_Y);
    if (sx->pin == 0) return false;  // 設定にサーボが無い

    servo_.begin(sx->pin, sx->start_degree, sx->offset,
                 sy->pin, sy->start_degree, sy->offset,
                 (ServoType)system_config.getServoType());

    // 起動直後は中立角にする
    target_pan_ = last_pan_ = sx->start_degree;
    target_tilt_ = last_tilt_ = sy->start_degree;
    ready_ = true;
    return true;
}

HeadLimits HeadServo::panLimits() const {
    auto* sx = system_config.getServoInfo(AXIS_X);
    return HeadLimits{(float)sx->start_degree, (float)sx->lower_limit, (float)sx->upper_limit};
}

HeadLimits HeadServo::tiltLimits() const {
    auto* sy = system_config.getServoInfo(AXIS_Y);
    return HeadLimits{(float)sy->start_degree, (float)sy->lower_limit, (float)sy->upper_limit};
}

void HeadServo::setTarget(float pan, float tilt) {
    const HeadLimits p = panLimits();
    const HeadLimits t = tiltLimits();
    target_pan_ = fminf(fmaxf(pan, p.lower), p.upper);
    target_tilt_ = fminf(fmaxf(tilt, t.lower), t.upper);
}

void HeadServo::moveTo(float pan, float tilt, uint32_t ms) {
    if (!ready_) return;
    const HeadLimits p = panLimits();
    const HeadLimits t = tiltLimits();
    const float x = fminf(fmaxf(pan, p.lower), p.upper);
    const float y = fminf(fmaxf(tilt, t.lower), t.upper);
    servo_.moveXYContinuous(x, y, ms);  // スティックと同じ経路（ループを止めない）
    last_pan_ = x;
    last_tilt_ = y;
    target_pan_ = x;
    target_tilt_ = y;
}

void HeadServo::spinBegin() {
    if (!ready_) return;
    servo_.turnXBegin(500, false);
}

void HeadServo::spinEnd() {
    if (!ready_) return;
    servo_.turnXEnd();
    moveTo(panLimits().center, tiltLimits().center, 500);
}

void HeadServo::update(uint32_t now_ms) {
    if (!ready_ || now_ms < next_send_ms_) return;

    const bool moved = fabsf(target_pan_ - last_pan_) >= kMinChangeDeg
                    || fabsf(target_tilt_ - last_tilt_) >= kMinChangeDeg;
    if (!moved) return;

    servo_.moveXYContinuous(target_pan_, target_tilt_, kMoveMs);
    last_pan_ = target_pan_;
    last_tilt_ = target_tilt_;
    next_send_ms_ = now_ms + kSendIntervalMs;
}
