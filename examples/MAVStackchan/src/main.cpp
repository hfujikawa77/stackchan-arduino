// MAVStackchan (Phase 1)
//   Stack-chan(CoreS3) を MAVLink 機体として QGroundControl から見えるようにする。
//   - 設定は SPIFFS の /yaml/SC_SecConfig.yaml から読む（Wi-Fi 候補・System ID・UDP ポート）
//   - Wi-Fi(STA) に接続し、UDP で HEARTBEAT を送信する
//   - QGC の HEARTBEAT を受けたら、その送信元へ以後の応答を返す
//   - ARM / モード変更 / サーボ制御は未実装（Phase 3 以降）

#include <Arduino.h>
#include <M5Unified.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <Avatar.h>

#include "AttitudeEstimator.h"
#include "FaceStatus.h"
#include "HeadServo.h"
#include "ManualControl.h"
#include "MavlinkVehicle.h"
#include "PhoneLocation.h"
#include "Settings.h"
#include "Stackchan_system_config.h"

// サーボ設定（SC_BasicConfig.yaml）。HeadServo.cpp から extern で参照する。
StackchanSystemConfig system_config;

namespace {

constexpr char kSettingsPath[] = "/yaml/SC_SecConfig.yaml";
constexpr uint32_t kWifiRetryMs = 5000;
constexpr uint32_t kImuIntervalMs = 10;          // IMU 読み取り 100Hz
constexpr uint32_t kCalibrationMs = 2000;        // 起動直後に機体を動かさない時間
constexpr float kDegToRad = 0.0174532925f;
constexpr size_t kRxBufferSize = 320;  // MAVLink 2 の最大パケット長以上

MavSettings settings;
MavlinkVehicle* vehicle = nullptr;
bool settings_loaded = false;

HeadServo head_servo;
bool head_ready = false;

m5avatar::Avatar avatar;                    // 顔（表情・吹き出し）
m5::PY32IOExpander_Class* led_io = nullptr;  // 内蔵 LED（頭部サーボと同じ IO 拡張）
constexpr int kLedCount = 12;               // HermesLLM と同じ LED 数
constexpr uint32_t kFaceIntervalMs = 200;

AttitudeEstimator attitude;
bool imu_ready = false;
bool calibration_started = false;
bool qgc_calibration_pending = false;  // QGC からの水平キャリブレーション中（結果を返す必要がある）
uint32_t calibration_end_ms = 0;
uint32_t last_imu_ms = 0;

WiFiUDP udp;
bool udp_open = false;
IPAddress broadcast_ip;
IPAddress peer_ip;
uint16_t peer_port = 0;
bool peer_known = false;

size_t wifi_index = 0;  // 次に試す wifi_list の番号（失敗ごとに循環）
uint32_t next_wifi_try_ms = 0;
bool last_linked = false;

// 送信先: QGC の送信元が分かっていればユニキャスト、なければサブネットのブロードキャスト
void sendToNetwork(void* /*ctx*/, const uint8_t* data, size_t len) {
    if (!udp_open) return;
    const IPAddress dst = peer_known ? peer_ip : broadcast_ip;
    const uint16_t port = peer_known ? peer_port : settings.udp_port;
    if (udp.beginPacket(dst, port) == 0) return;
    udp.write(data, len);
    udp.endPacket();
}

bool loadSettings() {
    if (!SPIFFS.begin(true)) {
        Serial.println("[MAV] SPIFFS mount failed");
        return false;
    }
    File f = SPIFFS.open(kSettingsPath, "r");
    if (!f) {
        Serial.printf("[MAV] %s not found. Copy data/yaml/SC_SecConfig.example.yaml and run uploadfs.\n",
                      kSettingsPath);
        return false;
    }
    const String yaml = f.readString();
    f.close();

    parseSettingsYaml(yaml, settings);
    Serial.printf("[MAV] settings: wifi=%u system_id=%u udp_port=%u\n",
                  static_cast<unsigned>(settings.wifi_count),
                  settings.system_id,
                  settings.udp_port);
    return true;
}

void ensureNetwork(uint32_t now_ms) {
    if (WiFi.status() == WL_CONNECTED) {
        if (!udp_open && udp.begin(settings.udp_port)) {
            udp_open = true;
            const uint32_t ip = static_cast<uint32_t>(WiFi.localIP());
            const uint32_t mask = static_cast<uint32_t>(WiFi.subnetMask());
            broadcast_ip = IPAddress(ip | ~mask);
            Serial.printf("[MAV] UDP %u open, local=%s broadcast=%s\n",
                          settings.udp_port,
                          WiFi.localIP().toString().c_str(),
                          broadcast_ip.toString().c_str());
        }
        return;
    }

    // 切断時は UDP を閉じ、学習した送信先も捨てる（再接続後に古い宛先へ送らない）
    if (udp_open) {
        udp.stop();
        udp_open = false;
        peer_known = false;
        Serial.println("[MAV] Wi-Fi lost, UDP closed");
    }

    if (settings.wifi_count == 0 || now_ms < next_wifi_try_ms) return;
    next_wifi_try_ms = now_ms + kWifiRetryMs;

    const WifiEntry& ap = settings.wifi[wifi_index];
    WiFi.mode(WIFI_STA);
    WiFi.begin(ap.ssid.c_str(), ap.password.c_str());
    Serial.printf("[MAV] Wi-Fi connecting to %s\n", ap.ssid.c_str());
    wifi_index = (wifi_index + 1) % settings.wifi_count;
}

void pumpNetwork(uint32_t now_ms) {
    if (!udp_open) return;
    if (udp.parsePacket() <= 0) return;

    uint8_t buf[kRxBufferSize];
    const int len = udp.read(buf, sizeof(buf));
    if (len <= 0) return;

    const IPAddress from_ip = udp.remoteIP();
    const uint16_t from_port = udp.remotePort();

    vehicle->onBytes(buf, static_cast<size_t>(len), now_ms);

    // GCS の HEARTBEAT を受けた送信元だけを宛先として学習する
    if (vehicle->takeGcsHeartbeat()) {
        if (!peer_known || peer_ip != from_ip || peer_port != from_port) {
            Serial.printf("[MAV] GCS peer %s:%u\n", from_ip.toString().c_str(), from_port);
        }
        peer_ip = from_ip;
        peer_port = from_port;
        peer_known = true;
    }
}

// バーチャルスティックで頭部を動かす。ARM かつ MANUAL かつ入力が新しいときだけ追従し、
// それ以外（DISARM・HOLD・入力途絶え・GCS 切断）は中立に戻す。
void updateHead(uint32_t now_ms) {
    if (!head_ready) return;

    const MavlinkVehicle::ManualInput in = vehicle->manualInput(now_ms);
    if (in.active) {
        const HeadTarget t = manualControlToHead(in.x, in.y, head_servo.panLimits(), head_servo.tiltLimits());
        head_servo.setTarget(t.pan, t.tilt);
    } else {
        const HeadLimits pan = head_servo.panLimits();
        const HeadLimits tilt = head_servo.tiltLimits();
        head_servo.setTarget(pan.center, tilt.center);
    }
    head_servo.update(now_ms);
}

// スマホの最後の位置（Worker 経由）を機体の位置として渡す。取得は別タスクで行う。
void updatePosition(uint32_t now_ms) {
    static uint32_t next_ms = 0;
    if (now_ms < next_ms) return;
    next_ms = now_ms + 1000;

    PhoneLocation loc;
    if (latestPhoneLocation(loc) && loc.valid) {
        vehicle->setPosition(loc.lat, loc.lon);
    }
}

// 電池と IMU の状態を 1 秒ごとに SYS_STATUS 用に渡す（M5.Power の値。残量は推定値）。
void updateSystemReport(uint32_t now_ms) {
    static uint32_t next_report_ms = 0;
    if (now_ms < next_report_ms) return;
    next_report_ms = now_ms + 1000;

    MavlinkVehicle::SystemReport report;
    report.imu_ok = imu_ready;
    const int16_t mv = M5.Power.getBatteryVoltage();
    const int32_t pct = M5.Power.getBatteryLevel();
    report.battery_valid = mv > 0;
    report.battery_mv = mv > 0 ? static_cast<uint16_t>(mv) : 0;
    report.battery_pct = (pct >= 0 && pct <= 100) ? static_cast<int8_t>(pct) : -1;

    report.wifi_valid = WiFi.status() == WL_CONNECTED;
    if (report.wifi_valid) {
        const int dbm = WiFi.RSSI();
        report.wifi_rssi_dbm = static_cast<int8_t>(dbm < -128 ? -128 : (dbm > 127 ? 127 : dbm));
    }
    vehicle->setSystemReport(report);
}

// QGC からの水平キャリブレーション要求を受けて、kCalibrationMs の間の姿勢を基準にする。
void updateLevelCalibration(uint32_t now_ms) {
    if (!vehicle->takeLevelCalibrationRequest()) return;
    if (!imu_ready) {
        vehicle->finishLevelCalibration(false);
        return;
    }
    attitude.beginCalibration();
    calibration_started = true;
    calibration_end_ms = now_ms + kCalibrationMs;
    qgc_calibration_pending = true;
    Serial.println("[MAV] level calibration requested by QGC: keep still");
}

// IMU を読んで姿勢を推定する。起動後 kCalibrationMs の間は水平に置いて基準を取る。
void updateImu(uint32_t now_ms) {
    if (!imu_ready || now_ms - last_imu_ms < kImuIntervalMs) return;
    const float dt_s = (now_ms - last_imu_ms) / 1000.0f;
    last_imu_ms = now_ms;

    M5.Imu.update();
    m5::imu_data_t d;
    M5.Imu.getImuData(&d);

    const float accel_g[3] = {d.accel.x, d.accel.y, d.accel.z};             // G
    const float gyro_rad[3] = {d.gyro.x * kDegToRad, d.gyro.y * kDegToRad,  // deg/s → rad/s
                               d.gyro.z * kDegToRad};
    attitude.update(accel_g, gyro_rad, dt_s);

    if (calibration_started && now_ms >= calibration_end_ms) {
        calibration_started = false;
        const bool ok = attitude.finishCalibration();
        const float norm = sqrtf(accel_g[0] * accel_g[0] + accel_g[1] * accel_g[1] + accel_g[2] * accel_g[2]);
        Serial.printf("[MAV] IMU calibration %s (|accel|=%.3fG, expect ~1.0G at rest)\n",
                      ok ? "OK" : "FAILED", norm);
        if (qgc_calibration_pending) {
            qgc_calibration_pending = false;
            vehicle->finishLevelCalibration(ok);
        }
    }

    if (!attitude.calibrated()) return;

    const AttitudeEstimator::Sample s = attitude.sample();
    MavlinkVehicle::AttitudeReport report;
    report.roll = s.roll;
    report.pitch = s.pitch;
    report.yaw = s.yaw;
    report.rollspeed = s.rollspeed;
    report.pitchspeed = s.pitchspeed;
    report.yawspeed = s.yawspeed;
    vehicle->setAttitude(report);
}

void reportLinkChange() {
    const bool linked = vehicle->gcsLinked();
    if (linked == last_linked) return;
    last_linked = linked;
    Serial.printf("[MAV] GCS link %s\n", linked ? "UP" : "DOWN");
    if (!linked) peer_known = false;  // 切断後は次の GCS HEARTBEAT まで宛先を固定しない
}

// 機体の状態を、顔（表情・吹き出し）と LED に反映する。状態が変わったときだけ更新する。
m5avatar::Expression toAvatarExpression(FaceExpression e) {
    switch (e) {
        case FaceExpression::Happy: return m5avatar::Expression::Happy;
        case FaceExpression::Angry: return m5avatar::Expression::Angry;
        case FaceExpression::Sad: return m5avatar::Expression::Sad;
        case FaceExpression::Doubt: return m5avatar::Expression::Doubt;
        case FaceExpression::Sleepy: return m5avatar::Expression::Sleepy;
        case FaceExpression::Neutral:
        default: return m5avatar::Expression::Neutral;
    }
}

void setLeds(const LedColor& c) {
    if (!led_io) return;
    for (int i = 0; i < kLedCount; ++i) led_io->setLedColor(i, c.r, c.g, c.b);
    led_io->refreshLeds();
}

void updateFace(uint32_t now_ms) {
    static uint32_t next_ms = 0;
    static bool has_last = false;
    static FaceExpression last_expression = FaceExpression::Neutral;
    static const char* last_speech = nullptr;
    static LedColor last_led = {0, 0, 0};

    if (now_ms < next_ms) return;
    next_ms = now_ms + kFaceIntervalMs;

    const StatusView v = statusViewFor(calibration_started, vehicle->gcsLinked(),
                                       vehicle->armed(), vehicle->mode() == VehicleMode::Manual);
    if (has_last && v.expression == last_expression && v.speech == last_speech
        && v.led.r == last_led.r && v.led.g == last_led.g && v.led.b == last_led.b) {
        return;
    }
    has_last = true;
    last_expression = v.expression;
    last_speech = v.speech;
    last_led = v.led;

    avatar.setExpression(toAvatarExpression(v.expression));
    avatar.setSpeechText(settings_loaded ? v.speech : "設定なし");
    setLeds(v.led);
}

}  // namespace

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    Serial.println("[MAV] MAVStackchan Phase 1 boot");

    settings_loaded = loadSettings();
    if (settings_loaded) startPhoneLocationTask(settings.digest_url, settings.digest_token);

    // 頭部サーボの設定（HermesLLM と同じ SC_BasicConfig.yaml）
    // 基本設定は約 3KB あるため、バッファを 8KB にする（既定の 2KB では解析中に落ちる）。
    // シークレット設定・アプリ用設定はこのアプリでは使わないので読まない（サイズ 0）。
    system_config.loadConfig(SPIFFS, "/yaml/SC_BasicConfig.yaml", 0,
                             "/yaml/SC_SecConfig.yaml", 0,
                             "/yaml/SC_BasicConfig.yaml", 8192);
    head_ready = head_servo.begin();
    Serial.printf("[MAV] head servo %s\n", head_ready ? "ready" : "not available");

    // 顔と内蔵 LED（LED は頭部サーボの IO 拡張を使うので、サーボが無ければ消灯のまま）
    avatar.init();
    avatar.setSpeechFont(&fonts::efontJA_12);
    led_io = head_servo.ioExpander();
    if (led_io) {
        led_io->setLedCount(kLedCount);
        setLeds(LedColor{0, 0, 0});
    }

    MavlinkVehicle::Config vehicle_cfg;
    vehicle_cfg.system_id = settings.system_id;
    vehicle = new MavlinkVehicle(vehicle_cfg, sendToNetwork, nullptr);

    imu_ready = M5.Imu.getType() != m5::imu_none;
    if (imu_ready) {
        // 起動直後は機体を水平に置いて動かさない。その時点の姿勢を 0 度とみなす。
        attitude.beginCalibration();
        calibration_started = true;
        calibration_end_ms = millis() + kCalibrationMs;
        Serial.println("[MAV] IMU found. Keep the device level and still for 2s (calibration).");
    } else {
        Serial.println("[MAV] IMU not found: ATTITUDE will not be sent");
    }
}

void loop() {
    M5.update();
    const uint32_t now_ms = millis();

    ensureNetwork(now_ms);
    pumpNetwork(now_ms);
    updateSystemReport(now_ms);
    updatePosition(now_ms);
    updateLevelCalibration(now_ms);
    updateImu(now_ms);
    updateHead(now_ms);
    vehicle->tick(now_ms);
    reportLinkChange();
    updateFace(now_ms);

    delay(1);
}
