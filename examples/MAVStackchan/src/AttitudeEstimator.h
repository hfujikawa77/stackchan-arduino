#pragma once

#include <stdint.h>

// IMU（加速度 + ジャイロ）から Roll / Pitch / Yaw を推定する。Arduino 非依存。
//
// 座標系（MAVLink の機体座標系 = body FRD）:
//   x: 前方, y: 右, z: 下。角度はラジアン、角速度は rad/s。
//   センサ座標系から body への軸割り当ては AttitudeEstimator.cpp の kAxisMap で決める（実機確認で調整する）。
//
// 推定方法:
//   - Roll / Pitch: ジャイロ積分を加速度からの重力方向で補正（相補フィルタ）
//   - Yaw: ジャイロ積分のみ（磁気補正なし）。ドリフトするため絶対方位は保証しない
//   - 静止時の加速度の大きさが 1G から大きく外れるときは、加速度による補正を止める
//
// キャリブレーション:
//   起動直後に一定時間、機体を動かさずに水平に置く。その時点の姿勢を 0 度とみなす。
class AttitudeEstimator {
public:
    struct Sample {
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        float rollspeed = 0.0f;
        float pitchspeed = 0.0f;
        float yawspeed = 0.0f;
    };

    // 加速度（G 単位）とジャイロ（rad/s）を入力する。センサ座標系のまま渡す。
    void update(const float accel_g[3], const float gyro_rad[3], float dt_s);

    // キャリブレーション: beginCalibration → update() を繰り返す → finishCalibration
    // （update() 中の加速度を集計する。サンプルが少なすぎると finishCalibration は false）
    void beginCalibration();
    bool finishCalibration();
    bool calibrated() const { return calibrated_; }

    Sample sample() const;

private:
    static void toBody(const float in[3], float out[3]);

    float roll_ = 0.0f;   // 生の推定値（基準補正前）
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;
    float roll_ref_ = 0.0f;
    float pitch_ref_ = 0.0f;
    float rate_[3] = {0.0f, 0.0f, 0.0f};  // body の角速度 rad/s
    bool initialized_ = false;
    bool calibrated_ = false;

    // キャリブレーション中の加速度の合計
    float cal_sum_[3] = {0.0f, 0.0f, 0.0f};
    uint32_t cal_count_ = 0;
    bool calibrating_ = false;
};
