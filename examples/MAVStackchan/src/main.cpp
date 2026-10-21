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
#include "DeviceSettings.h"
#include "FaceStatus.h"
#include "Greeting.h"
#include "HeadServo.h"
#include "HeadTouch.h"
#include "ManualControl.h"
#include "MavlinkVehicle.h"
#include "PhoneLocation.h"
#include "Settings.h"
#include "SlideBridge.h"
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
// --- スライドモード（reveal.js 連携）の共有状態 ---
SlideBridge slide_ws;     // GET /servo-ws
HeadTouch head_touch;     // 頭上タッチ（スワイプ・2 回タップ）
Greeting greeting;        // 挨拶の WAV（intro / outro）
bool slide_mode_on = false;     // 設定（既定 OFF）
bool slide_active = false;      // スライドの表示を適用中（状態表示より優先）
bool face_refresh_needed = false;
bool slide_server_started = false;
bool slideMotionBusy();         // 首の動き（ステップ・回転）中か。updateHead が書き込みを止めるのに使う

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
        if (!slide_server_started) {
            slide_ws.begin(80);
            slide_server_started = true;
        }
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
    if (slideMotionBusy()) return;  // スライドの首の動きが優先（終わるまで手動追従を止める）

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

// --- スライドモード ----------------------------------------------------------
// 頭上のスワイプでスライドを前後させ（{"type":"control"}）、2 回タップで挨拶。
// reveal.js からの {"type":"slide"} で表情・吹き出し・LED・首の動きを切り替える。
// 首の動きは、手動（ARM かつ MANUAL）で動かしている間は行わない（安全側）。

struct MotionStep { int8_t dpan; int8_t dtilt; uint16_t ms; uint16_t after_ms; };  // 中立からの相対角
const MotionStep kSlideNod[] = {{0, -18, 180, 260}, {0, 0, 220, 0}};
const MotionStep kSlideShake[] = {{22, 0, 150, 190}, {-22, 0, 150, 190}, {0, 0, 150, 0}};
const MotionStep* motion_steps = nullptr;
uint8_t motion_n = 0;
uint8_t motion_i = 0;
uint32_t motion_next_ms = 0;
bool spinning = false;
uint32_t spin_end_ms = 0;

struct SlidePending {
    bool pending = false;
    int h = 0;
    int v = 0;
    String title;
    String motion = "nod";
    String expr;
    String led;  // 空 = LED は変えない
};
SlidePending slide_pending;
int slide_applied_h = -1;
int slide_applied_v = -1;
uint32_t slide_bubble_clear_ms = 0;
uint32_t slide_next_control_ms = 0;
bool slide_greeting_request = false;
bool slide_greeting_outro = false;
String slide_bubble_text;  // avatar に渡す文字列（参照が残るので保持する）

bool manualDriveActive() {
    return vehicle->armed() && vehicle->mode() == VehicleMode::Manual;
}

bool slideMotionBusy() {
    return motion_steps != nullptr || spinning;
}

void stopSlideMotion() {
    motion_steps = nullptr;
    if (spinning) {
        head_servo.spinEnd();
        spinning = false;
    }
}

void runMotionStep(uint32_t now) {
    if (!motion_steps) return;
    const MotionStep& st = motion_steps[motion_i++];
    const HeadLimits pan = head_servo.panLimits();
    const HeadLimits tilt = head_servo.tiltLimits();
    head_servo.moveTo(pan.center + st.dpan, tilt.center + st.dtilt, st.ms);
    if (motion_i >= motion_n) {
        motion_steps = nullptr;
    } else {
        motion_next_ms = now + st.after_ms;
    }
}

void startMotion(const MotionStep* steps, uint8_t n, uint32_t now) {
    motion_steps = steps;
    motion_n = n;
    motion_i = 0;
    runMotionStep(now);
}

// 挨拶: 1 回目 intro、次 outro、交互。非同期なので、ループは止まらない。
void playGreeting() {
    const char* path = slide_greeting_outro ? "/audio/outro.wav" : "/audio/intro.wav";
    if (greeting.start(path, settings.volume)) {
        slide_greeting_outro = !slide_greeting_outro;
        Serial.printf("[SLIDE] greeting: %s\n", path);
    } else {
        Serial.printf("[SLIDE] greeting not played: %s\n", path);
    }
}

// reveal.js からの JSON を受ける（SlideBridge のハンドラ）
void onSlideMessage(const char* json, void* /*ctx*/) {
    JsonDocument doc;
    if (deserializeJson(doc, json)) return;
    const char* type = doc["type"] | "";
    if (strcmp(type, "slide") != 0) return;  // ping など
    if (!slide_mode_on) return;
    slide_pending.h = doc["indexh"] | 0;
    slide_pending.v = doc["indexv"] | 0;
    slide_pending.title = doc["title"] | "";
    slide_pending.motion = doc["motion"] | "nod";
    slide_pending.expr = doc["expression"] | "";
    slide_pending.led = "";
    if (doc["led"].is<JsonObject>()) serializeJson(doc["led"], slide_pending.led);
    slide_pending.pending = true;
}

// LED 指定（data-stackchan-led）。色だけ反映する（パターンは未対応）
void applySlideLed(const String& json) {
    JsonDocument doc;
    if (deserializeJson(doc, json)) return;
    if (doc["off"] | false) {
        setLeds(LedColor{0, 0, 0});
        return;
    }
    const String color = doc["color"] | "";
    if (color.length() == 7 && color[0] == '#') {
        const uint8_t r = static_cast<uint8_t>(strtol(color.substring(1, 3).c_str(), nullptr, 16));
        const uint8_t g = static_cast<uint8_t>(strtol(color.substring(3, 5).c_str(), nullptr, 16));
        const uint8_t b = static_cast<uint8_t>(strtol(color.substring(5, 7).c_str(), nullptr, 16));
        setLeds(LedColor{r, g, b});
    }
}

void applySlideExpression(const String& expr, int h) {
    static const m5avatar::Expression table[] = {
        m5avatar::Expression::Neutral, m5avatar::Expression::Happy, m5avatar::Expression::Doubt,
        m5avatar::Expression::Sad, m5avatar::Expression::Angry, m5avatar::Expression::Sleepy,
    };
    m5avatar::Expression e = table[(h < 0 ? 0 : h) % 6];  // 6 枚ごとに循環（HermesLLM と同じ）
    if (expr == "neutral") e = m5avatar::Expression::Neutral;
    else if (expr == "happy") e = m5avatar::Expression::Happy;
    else if (expr == "sad") e = m5avatar::Expression::Sad;
    else if (expr == "angry") e = m5avatar::Expression::Angry;
    else if (expr == "doubt") e = m5avatar::Expression::Doubt;
    else if (expr == "sleepy") e = m5avatar::Expression::Sleepy;
    avatar.setExpression(e);
}

// スライドの表示を適用する（表情・吹き出し・LED・首の動き）
void applySlide(uint32_t now) {
    slide_applied_h = slide_pending.h;
    slide_applied_v = slide_pending.v;
    slide_active = true;

    applySlideExpression(slide_pending.expr, slide_applied_h);

    String bubble = String(slide_applied_h + 1) + ". " + slide_pending.title;
    if (slide_pending.title.isEmpty()) bubble = String(slide_applied_h + 1) + "ページ目";
    slide_bubble_text = bubble;
    avatar.setSpeechText(slide_bubble_text.c_str());
    slide_bubble_clear_ms = now + 4000;

    if (!slide_pending.led.isEmpty()) applySlideLed(slide_pending.led);

    // 首の動き: MANUAL で操作中は動かさない
    if (manualDriveActive() || !head_ready) return;
    if (slide_pending.motion == "none") return;
    if (slide_pending.motion == "shake") {
        startMotion(kSlideShake, sizeof(kSlideShake) / sizeof(kSlideShake[0]), now);
    } else if (slide_pending.motion == "spin") {
        head_servo.spinBegin();
        spinning = true;
        spin_end_ms = now + 2000;
    } else {
        startMotion(kSlideNod, sizeof(kSlideNod) / sizeof(kSlideNod[0]), now);
    }
}

// 毎ループ呼ぶ。接続・タッチ・挨拶・スライド適用・首の動きを進める。
void updateSlides(uint32_t now) {
    static uint32_t next_touch_ms = 0;

    slide_ws.tick(now, onSlideMessage, nullptr);

    // スライドモードが無効、または reveal.js が切れたら、状態表示に戻す
    const bool want = slide_mode_on && slide_ws.connected();
    if (!want) {
        if (slide_active) {
            slide_active = false;
            stopSlideMotion();
            avatar.setSpeechText("");
            face_refresh_needed = true;
        }
        return;
    }

    // MANUAL で操作中は、スライドの首の動きを止める（安全側）
    if (manualDriveActive()) stopSlideMotion();

    // 頭上タッチ（20ms ごと）
    if (now >= next_touch_ms) {
        next_touch_ms = now + 20;
        head_touch.poll(now);
    }
    const bool fwd = head_touch.takeSwipeForward();
    const bool back = head_touch.takeSwipeBackward();
    if ((fwd || back) && now >= slide_next_control_ms) {
        slide_next_control_ms = now + 500;  // 1 回のスワイプで 1 回だけ送る
        slide_ws.sendText(fwd ? "{\"type\":\"control\",\"action\":\"next\"}"
                              : "{\"type\":\"control\",\"action\":\"prev\"}");
    }
    if (head_touch.takeDoubleTap()) slide_greeting_request = true;

    if (slide_greeting_request && !greeting.playing()) {
        slide_greeting_request = false;
        playGreeting();
    }
    greeting.tick();

    if (slide_pending.pending) {
        slide_pending.pending = false;
        if (!(slide_pending.h == slide_applied_h && slide_pending.v == slide_applied_v)) {
            applySlide(now);
        }
    }

    if (slide_bubble_clear_ms && now >= slide_bubble_clear_ms) {
        slide_bubble_clear_ms = 0;
        avatar.setSpeechText("");
    }
    if (motion_steps && now >= motion_next_ms) runMotionStep(now);
    if (spinning && now >= spin_end_ms) {
        head_servo.spinEnd();
        spinning = false;
    }
}

// --- 本体の設定画面 --------------------------------------------------------
// 画面の右上隅をタップで開く。項目: スライドモード / 音量 / 明るさ / Wi-Fi 候補。
// 閉じると /settings.json に保存する。開いている間は顔とスライドを止める。

DeviceSettings dev;
bool settings_open = false;

constexpr int kSettingsRowY[4] = {44, 84, 124, 164};  // スライド / 音量 / 明るさ / Wi-Fi
constexpr int kSettingsRowH = 36;
constexpr int kSettingsCloseY = 206;
constexpr int kSettingsCloseH = 30;

void drawSettingsScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setFont(&fonts::efontJA_16);
    M5.Display.setTextSize(1);

    M5.Display.setCursor(8, 12);
    M5.Display.print("設定");

    char line[64];
    M5.Display.setCursor(12, kSettingsRowY[0] + 10);
    M5.Display.print(dev.slide_mode ? "スライドモード: ON" : "スライドモード: OFF");

    M5.Display.setCursor(12, kSettingsRowY[1] + 10);
    snprintf(line, sizeof(line), "音量: %u", (unsigned)dev.volume);
    M5.Display.print(line);
    M5.Display.fillRoundRect(200, kSettingsRowY[1] + 4, 50, kSettingsRowH - 8, 4, TFT_DARKGREY);
    M5.Display.fillRoundRect(256, kSettingsRowY[1] + 4, 50, kSettingsRowH - 8, 4, TFT_DARKGREY);
    M5.Display.setCursor(218, kSettingsRowY[1] + 10);
    M5.Display.print("-");
    M5.Display.setCursor(274, kSettingsRowY[1] + 10);
    M5.Display.print("+");

    M5.Display.setCursor(12, kSettingsRowY[2] + 10);
    snprintf(line, sizeof(line), "明るさ: %u", (unsigned)dev.brightness);
    M5.Display.print(line);
    M5.Display.fillRoundRect(200, kSettingsRowY[2] + 4, 50, kSettingsRowH - 8, 4, TFT_DARKGREY);
    M5.Display.fillRoundRect(256, kSettingsRowY[2] + 4, 50, kSettingsRowH - 8, 4, TFT_DARKGREY);
    M5.Display.setCursor(218, kSettingsRowY[2] + 10);
    M5.Display.print("-");
    M5.Display.setCursor(274, kSettingsRowY[2] + 10);
    M5.Display.print("+");

    M5.Display.setCursor(12, kSettingsRowY[3] + 10);
    if (settings.wifi_count > 0 && dev.wifi_index < settings.wifi_count) {
        snprintf(line, sizeof(line), "Wi-Fi: %s", settings.wifi[dev.wifi_index].ssid.c_str());
    } else {
        snprintf(line, sizeof(line), "Wi-Fi: 未設定");
    }
    M5.Display.print(line);

    M5.Display.fillRoundRect(8, kSettingsCloseY, 304, kSettingsCloseH, 4, TFT_DARKGREEN);
    M5.Display.setCursor(120, kSettingsCloseY + 7);
    M5.Display.print("閉じる（保存）");
}

void openSettingsScreen() {
    settings_open = true;
    avatar.suspend();
    drawSettingsScreen();
    Serial.println("[SET] settings opened");
}

void closeSettingsScreen() {
    saveDeviceSettings(dev);
    settings_open = false;
    avatar.resume();
    face_refresh_needed = true;
    M5.Display.setFont(&fonts::Font0);
    M5.Display.fillScreen(TFT_BLACK);
    Serial.println("[SET] settings saved and closed");
}

bool inRect(int x, int y, int rx, int ry, int rw, int rh) {
    return x >= rx && x < rx + rw && y >= ry && y < ry + rh;
}

// 設定画面のタッチ処理。設定画面を開いている間は true を返す。
bool updateSettingsScreen(uint32_t now) {
    (void)now;
    auto t = M5.Touch.getDetail(0);
    if (!t.wasPressed()) return settings_open;

    if (!settings_open) {
        // 右上隅（画面の幅の右端 60px、上端 60px）で開く
        if (t.x > M5.Display.width() - 60 && t.y < 60) openSettingsScreen();
        return settings_open;
    }

    const int x = t.x;
    const int y = t.y;
    if (inRect(x, y, 8, kSettingsCloseY, 304, kSettingsCloseH)) {
        closeSettingsScreen();
        return false;
    }
    if (inRect(x, y, 0, kSettingsRowY[0], 320, kSettingsRowH)) {
        dev.slide_mode = !dev.slide_mode;
        slide_mode_on = dev.slide_mode;
    } else if (inRect(x, y, 0, kSettingsRowY[1], 320, kSettingsRowH)) {
        if (inRect(x, y, 200, kSettingsRowY[1], 50, kSettingsRowH)) {
            dev.volume = dev.volume >= 10 ? dev.volume - 10 : 0;
        } else if (inRect(x, y, 256, kSettingsRowY[1], 50, kSettingsRowH)) {
            dev.volume = dev.volume <= 90 ? dev.volume + 10 : 100;
        }
        settings.volume = dev.volume;
        M5.Speaker.setVolume(dev.volume);
    } else if (inRect(x, y, 0, kSettingsRowY[2], 320, kSettingsRowH)) {
        if (inRect(x, y, 200, kSettingsRowY[2], 50, kSettingsRowH)) {
            dev.brightness = dev.brightness >= 50 ? dev.brightness - 25 : 25;
        } else if (inRect(x, y, 256, kSettingsRowY[2], 50, kSettingsRowH)) {
            dev.brightness = dev.brightness <= 230 ? dev.brightness + 25 : 255;
        }
        M5.Display.setBrightness(dev.brightness);
    } else if (inRect(x, y, 0, kSettingsRowY[3], 320, kSettingsRowH)) {
        // 次の候補へ切り替える。接続し直して、新しい候補から接続する
        if (settings.wifi_count > 0) {
            dev.wifi_index = (dev.wifi_index + 1) % settings.wifi_count;
            wifi_index = dev.wifi_index;
            WiFi.disconnect();
            next_wifi_try_ms = 0;
        }
    }
    drawSettingsScreen();
    return true;
}

void updateFace(uint32_t now_ms) {
    static uint32_t next_ms = 0;
    static bool has_last = false;
    static FaceExpression last_expression = FaceExpression::Neutral;
    static const char* last_speech = nullptr;
    static LedColor last_led = {0, 0, 0};

    if (now_ms < next_ms) return;
    next_ms = now_ms + kFaceIntervalMs;
    if (settings_open) return;  // 設定画面の表示中は顔を出さない
    if (slide_active) return;  // スライドの表示中は状態表示を出さない
    if (face_refresh_needed) {
        face_refresh_needed = false;
        has_last = false;
    }

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
    // 設定画面の値（/settings.json）。無ければ YAML の既定値のまま
    dev.slide_mode = settings.slide_mode;
    dev.volume = settings.volume;
    loadDeviceSettings(dev);
    settings.volume = dev.volume;
    if (dev.wifi_index >= settings.wifi_count) dev.wifi_index = 0;
    wifi_index = dev.wifi_index;
    M5.Display.setBrightness(dev.brightness);
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

    slide_mode_on = dev.slide_mode;
    if (slide_mode_on) {
        head_touch.begin();
        Serial.println("[SLIDE] slide mode ON");
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
    const bool in_settings = updateSettingsScreen(now_ms);  // 右上隅のタップで設定画面
    if (!in_settings) updateSlides(now_ms);
    updateHead(now_ms);
    vehicle->tick(now_ms);
    reportLinkChange();
    updateFace(now_ms);

    delay(1);
}
