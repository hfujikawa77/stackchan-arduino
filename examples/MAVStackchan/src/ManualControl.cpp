#include "ManualControl.h"

namespace {

float clampFloat(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

}  // namespace

float normalizeStick(int16_t raw) {
    if (raw < -kStickMax || raw > kStickMax) return 0.0f;  // 未定義値・異常値は中立扱い
    const float n = static_cast<float>(raw) / static_cast<float>(kStickMax);
    if (n > -kStickDeadzone && n < kStickDeadzone) return 0.0f;
    // 不感帯の外側を 0〜1 に引き伸ばし、入力の段差をなくす
    const float sign = n < 0.0f ? -1.0f : 1.0f;
    const float magnitude = (sign * n - kStickDeadzone) / (1.0f - kStickDeadzone);
    return sign * clampFloat(magnitude, 0.0f, 1.0f);
}

float stickToDegrees(float normalized, const HeadLimits& limits) {
    const float n = clampFloat(normalized, -1.0f, 1.0f);
    float deg;
    if (n >= 0.0f) {
        deg = limits.center + n * (limits.upper - limits.center);
    } else {
        deg = limits.center + n * (limits.center - limits.lower);
    }
    return clampFloat(deg, limits.lower, limits.upper);
}

HeadTarget manualControlToHead(int16_t stick_x, int16_t stick_y,
                               const HeadLimits& pan_limits, const HeadLimits& tilt_limits) {
    HeadTarget t;
    // 実機で確認し、左右・上下ともに頭部の向きと逆だったため符号を反転している
    t.pan = stickToDegrees(-normalizeStick(stick_y), pan_limits);
    t.tilt = stickToDegrees(-normalizeStick(stick_x), tilt_limits);
    return t;
}
