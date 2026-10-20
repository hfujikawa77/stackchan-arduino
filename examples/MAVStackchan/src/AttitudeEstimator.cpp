#include "AttitudeEstimator.h"

#include <math.h>

namespace {

// センサ座標系 → body 座標系。各要素は {センサの軸番号, 符号}。
// 実機の静止データから決定（水平: センサー y が +1G、左に倒す: x が負、右に倒す: x が正）。
//   body z（上下）← −センサー y（水平で重力が y に乗るため）
//   body y（左右）← −センサー x（左に倒すと x が負になるため）
//   body x（前後）← +センサー z（右手系を保つための符号。前後の確認は未実施）
// 3 軸の変換行列の行列式は +1（右手系を保つ）。
struct AxisSpec {
    uint8_t source;  // 0: x, 1: y, 2: z
    float sign;      // +1 or -1
};
constexpr AxisSpec kAxisMap[3] = {{2, 1.0f}, {0, -1.0f}, {1, -1.0f}};

// 相補フィルタの加速度側の重み（ジャイロ側 0.98）
constexpr float kAccelWeight = 0.02f;

// 加速度の大きさが 1G からこの範囲を超えたら、加速度による補正を行わない
constexpr float kAccelNormTolerance = 0.3f;

// キャリブレーションに必要な最小サンプル数
constexpr uint32_t kMinCalibrationSamples = 50;

// 有効な dt の上限（秒）。これを超えると積分を飛ばす（再起動直後などの異常値対策）
constexpr float kMaxDtSeconds = 0.5f;

float wrapPi(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

}  // namespace

void AttitudeEstimator::toBody(const float in[3], float out[3]) {
    for (int i = 0; i < 3; ++i) {
        out[i] = kAxisMap[i].sign * in[kAxisMap[i].source];
    }
}

void AttitudeEstimator::update(const float accel_g[3], const float gyro_rad[3], float dt_s) {
    float a[3];
    float g[3];
    toBody(accel_g, a);
    toBody(gyro_rad, g);
    rate_[0] = g[0];
    rate_[1] = g[1];
    rate_[2] = g[2];

    if (calibrating_) {
        for (int i = 0; i < 3; ++i) cal_sum_[i] += a[i];
        ++cal_count_;
    }

    if (dt_s <= 0.0f || dt_s > kMaxDtSeconds) return;

    // 静止時の加速度計は重力の反力を測るため、重力方向（下向き）は -a
    const float down_x = -a[0];
    const float down_y = -a[1];
    const float down_z = -a[2];
    const float roll_acc = atan2f(down_y, down_z);
    const float pitch_acc = atan2f(-down_x, sqrtf(down_y * down_y + down_z * down_z));

    if (!initialized_) {
        roll_ = roll_acc;
        pitch_ = pitch_acc;
        initialized_ = true;
    }

    // ジャイロ積分（小角度近似で Euler 角の変化率 = body 角速度）
    roll_ += rate_[0] * dt_s;
    pitch_ += rate_[1] * dt_s;
    yaw_ = wrapPi(yaw_ + rate_[2] * dt_s);

    // 加速度の大きさが 1G 付近のときだけ重力方向で補正する（動いているときは信用しない）
    const float norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (fabsf(norm - 1.0f) < kAccelNormTolerance) {
        roll_ = (1.0f - kAccelWeight) * roll_ + kAccelWeight * roll_acc;
        pitch_ = (1.0f - kAccelWeight) * pitch_ + kAccelWeight * pitch_acc;
    }
}

void AttitudeEstimator::beginCalibration() {
    for (int i = 0; i < 3; ++i) cal_sum_[i] = 0.0f;
    cal_count_ = 0;
    calibrating_ = true;
    calibrated_ = false;
}

bool AttitudeEstimator::finishCalibration() {
    calibrating_ = false;
    if (cal_count_ < kMinCalibrationSamples) return false;

    const float ax = cal_sum_[0] / cal_count_;
    const float ay = cal_sum_[1] / cal_count_;
    const float az = cal_sum_[2] / cal_count_;
    const float down_x = -ax;
    const float down_y = -ay;
    const float down_z = -az;
    roll_ref_ = atan2f(down_y, down_z);
    pitch_ref_ = atan2f(-down_x, sqrtf(down_y * down_y + down_z * down_z));
    calibrated_ = true;
    return true;
}

AttitudeEstimator::Sample AttitudeEstimator::sample() const {
    // ロールは出力の符号を反転する（実機の HUD で左右が逆だったため）。
    // 内部の推定（ジャイロと加速度の融合）はそのままで、角度と角速度の両方を反転する。
    Sample s;
    s.roll = -(roll_ - roll_ref_);
    s.pitch = pitch_ - pitch_ref_;
    s.yaw = yaw_;
    s.rollspeed = -rate_[0];
    s.pitchspeed = rate_[1];
    s.yawspeed = rate_[2];
    return s;
}
