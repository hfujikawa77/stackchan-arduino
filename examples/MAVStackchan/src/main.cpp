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

// ループのスタックは既定の 8KB では足りない（設定画面・スライドの処理で溢れて再起動する）。
// HermesLLM と同じ 16KB にする。
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

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
DeviceSettings dev;       // 本体の設定画面の値（/settings.json）
bool settings_open = false;  // 設定画面を開いているか
bool mav_enabled = true;  // MAVLink（QGC との通信）。設定画面から再起動なしで切り替える
uint32_t ip_bubble_until_ms = 0;  // 接続直後に IP を吹き出しで出す期限（0 = 出していない）
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
        if (!mav_enabled && udp_open) {
            udp.stop();
            udp_open = false;
            peer_known = false;
        }
        if (mav_enabled && !udp_open && udp.begin(settings.udp_port)) {
            udp_open = true;
            const uint32_t ip = static_cast<uint32_t>(WiFi.localIP());
            const uint32_t mask = static_cast<uint32_t>(WiFi.subnetMask());
            broadcast_ip = IPAddress(ip | ~mask);
            // 機体の IP を吹き出しで 15 秒表示する（ブラウザの ?sc=<IP> に使う）
            ip_bubble_until_ms = millis() + 15000;
            avatar.setSpeechText(("IP " + WiFi.localIP().toString()).c_str());
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

// 吹き出しの文字を流す（HermesLLM と同じ: 12 文字ずつ、130ms ごとに 1 文字ずらす）
struct BubbleScroll {
    String text;
    uint32_t started_ms = 0;
    uint32_t duration_ms = 0;
    uint32_t last_ms = 0;
    bool active = false;
    static constexpr int kVisibleChars = 12;
    static constexpr uint32_t kStepMs = 130;

    static int nextIndex(const String& t, int index) {
        if (index >= (int)t.length()) return t.length();
        const uint8_t c = (uint8_t)t[index];
        int step = 1;
        if ((c & 0x80) == 0x00) step = 1;
        else if ((c & 0xE0) == 0xC0) step = 2;
        else if ((c & 0xF0) == 0xE0) step = 3;
        else if ((c & 0xF8) == 0xF0) step = 4;
        return min((int)t.length(), index + step);
    }
    static int charCount(const String& t) {
        int n = 0;
        for (int i = 0; i < (int)t.length(); i = nextIndex(t, i)) n++;
        return n;
    }
    static String window(const String& t, int start_char) {
        int start = 0;
        for (int i = 0; i < start_char && start < (int)t.length(); ++i) start = nextIndex(t, start);
        int end = start;
        for (int i = 0; i < kVisibleChars && end < (int)t.length(); ++i) end = nextIndex(t, end);
        return t.substring(start, end);
    }
    void start(const String& s, uint32_t dur, uint32_t now) {
        text = s;
        active = !s.isEmpty();
        started_ms = now;
        duration_ms = dur;
        last_ms = 0;
        tick(now);
    }
    void stop() {
        active = false;
        text = "";
    }
    void tick(uint32_t now) {
        if (!active) return;
        if (last_ms && now - last_ms < kStepMs) return;
        last_ms = now;
        const int total = charCount(text);
        String visible = text;
        if (total > kVisibleChars) {
            const int span = total - kVisibleChars + 1;
            const int start = duration_ms > 0 ? (int)(((now - started_ms) / kStepMs) % span) : 0;
            visible = window(text, start);
        }
        avatar.setSpeechText(visible.c_str());
    }
};
BubbleScroll bubble_scroll;

bool manualDriveActive() {
    return vehicle->armed() && vehicle->mode() == VehicleMode::Manual;
}

bool speech_active = false;  // 挨拶の音声に合わせて、口と首を動かしている
bool cue_playing = false;    // いま鳴っているのが状態音（接続・ARM など）。口と首は動かさない
const char* cue_pending = nullptr;  // 再生待ちの状態音（挨拶が鳴っている間は待つ。新しい要求で上書き）

bool slideMotionBusy() {
    return motion_steps != nullptr || spinning || speech_active;
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
void playGreeting(uint32_t now) {
    const char* path = slide_greeting_outro ? "/audio/outro.wav" : "/audio/intro.wav";
    const char* text = slide_greeting_outro ? "Thank you, everyone!" : "Hiii, everyone! I'm Stack-chan!";
    if (greeting.start(path, settings.volume)) {
        cue_playing = false;
        slide_greeting_outro = !slide_greeting_outro;
        Serial.printf("[SLIDE] greeting: %s\n", path);
        // しゃべる内容を吹き出しに出す。音声が終わるまで表示する
        bubble_scroll.start(text, greeting.durationMs(), now);
        slide_bubble_clear_ms = now + greeting.durationMs() + 500;
    } else {
        Serial.printf("[SLIDE] greeting not played: %s\n", path);
    }
}

// --- 状態音（QGC の接続・ARM・モード変更に合わせて鳴らす）-------------------
// 挨拶と同じ WAV 再生を使う。挨拶や別の状態音が鳴っている間は待ち、終わってから鳴らす。
// 音だけ鳴らし、吹き出し・口・首の動きは付けない。
constexpr const char* kCueConnect = "/audio/connect.wav";
constexpr const char* kCueDisconnect = "/audio/disconnect.wav";
constexpr const char* kCueArmed = "/audio/armed.wav";
constexpr const char* kCueDisarmed = "/audio/disarmed.wav";
constexpr const char* kCueModeManual = "/audio/mode_manual.wav";
constexpr const char* kCueModeHold = "/audio/mode_hold.wav";
constexpr const char* kCueBeep = "/audio/beep.wav";  // 設定「ビープ」のときは全部この音

// 状態が変わったときに 1 回だけ要求を出す。同時に複数変わったときは 接続 → ARM → モード の順で 1 つだけ。
// （接続が切れると GCS タイムアウトで DISARM もされるため、切断音を優先する）
void reportCueEvents() {
    static bool prev_linked = false;
    static bool prev_armed = false;
    static VehicleMode prev_mode = VehicleMode::Hold;

    const bool linked = vehicle->gcsLinked();
    const bool armed = vehicle->armed();
    const VehicleMode mode = vehicle->mode();
    if (linked != prev_linked) {
        cue_pending = linked ? kCueConnect : kCueDisconnect;
    } else if (armed != prev_armed) {
        cue_pending = armed ? kCueArmed : kCueDisarmed;
    } else if (mode != prev_mode) {
        cue_pending = mode == VehicleMode::Manual ? kCueModeManual : kCueModeHold;
    }
    prev_linked = linked;
    prev_armed = armed;
    prev_mode = mode;
}

// 挨拶と状態音の再生を進める（毎ループ）。スライドモードの外でも鳴るように、ここで tick する。
void updateCues() {
    greeting.tick();
    if (!greeting.playing()) cue_playing = false;
    if (!cue_pending || greeting.playing()) return;

    // 設定が「ビープ」なら、どの状態でも同じ音にする
    const char* path = dev.cue_beep ? kCueBeep : cue_pending;
    cue_pending = nullptr;
    if (greeting.start(path, settings.volume)) {
        cue_playing = true;
        Serial.printf("[MAV] cue: %s\n", path);
    } else {
        Serial.printf("[MAV] cue not played: %s\n", path);
    }
}

// 状態音の再生中は、口だけを開け閉めする（頭は動かさない）。設定画面の表示中は顔に触らない。
void updateCueMouth(uint32_t now) {
    static uint32_t next_ms = 0;
    static bool mouth_open = false;

    if (!cue_playing || settings_open) {
        if (mouth_open) {
            avatar.setMouthOpenRatio(0.0f);
            mouth_open = false;
        }
        return;
    }
    if (now < next_ms) return;
    next_ms = now + 120;
    mouth_open = !mouth_open;
    avatar.setMouthOpenRatio(mouth_open ? random(30, 65) / 100.0f : 0.05f);
}

// しゃべっている間の動き（HermesLLM の再生中の動きと同じ 7 ステップ）
// 0〜4: うなずき（上下）と口の開閉 / 5: 左右に振る / 6: 中央へ戻す
void updateSpeechMotion(uint32_t now) {
    static uint32_t next_ms = 0;
    static uint8_t step = 0;
    static bool up = false;

    if (!greeting.playing() || cue_playing) {
        if (speech_active) {
            speech_active = false;
            avatar.setMouthOpenRatio(0.0f);
            if (head_ready && !manualDriveActive()) {
                const HeadLimits pan = head_servo.panLimits();
                const HeadLimits tilt = head_servo.tiltLimits();
                head_servo.moveTo(pan.center, tilt.center, 500);
            }
        }
        return;
    }
    if (!speech_active) {
        speech_active = true;
        step = 0;
        next_ms = now;
    }
    if (now < next_ms) return;
    next_ms = now + 700;

    const bool may_move = head_ready && !manualDriveActive();
    const HeadLimits pan = head_servo.panLimits();
    const HeadLimits tilt = head_servo.tiltLimits();
    const uint8_t phase = step % 7;
    Serial.printf("[SLIDE] speech step=%u phase=%u may_move=%d\n", (unsigned)step, (unsigned)phase, may_move ? 1 : 0);
    if (phase == 5) {
        const int dir = random(0, 2) == 0 ? -1 : 1;
        if (may_move) head_servo.moveTo(pan.center + dir * 22, tilt.center - 4, 450);
    } else if (phase == 6) {
        if (may_move) head_servo.moveTo(pan.center, tilt.center, 450);
    } else {
        up = !up;
        if (may_move) head_servo.moveTo(pan.center, tilt.center + (up ? -16 : 4), 500);
        avatar.setMouthOpenRatio(up ? random(30, 65) / 100.0f : 0.05f);
    }
    step++;
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

// LED 指定（data-stackchan-led）。6 種類のパターン（HermesLLM と同じ見た目）:
//   blink / wave / split_chase / split_breath / split_blink / split_wave
// それ以外の pattern と solid は、color の単色になる。時間から毎回計算する（状態を持たない）。
enum class LedPat : uint8_t { Solid, Blink, Wave, SplitChase, SplitBreath, SplitBlink, SplitWave };
struct LedSpec {
    LedPat pat = LedPat::Solid;
    LedColor c1 = {0, 0, 0};
    LedColor c2 = {0, 0, 0};
    uint8_t speed = 5;  // 1〜10
};
LedSpec slide_led;
bool slide_led_active = false;
uint32_t slide_led_next_ms = 0;

LedColor scaleLed(LedColor c, uint8_t br) {
    return LedColor{static_cast<uint8_t>(c.r * br / 255), static_cast<uint8_t>(c.g * br / 255),
                    static_cast<uint8_t>(c.b * br / 255)};
}

// 0〜255 の位相から、三角波の明るさ（0〜254）
uint8_t triangleBright(uint32_t phase) {
    phase &= 0xFF;
    return phase < 128 ? static_cast<uint8_t>(phase * 2) : static_cast<uint8_t>((255 - phase) * 2);
}

LedColor parseHexColor(const String& text, LedColor fallback) {
    if (text.length() != 7 || text[0] != '#') return fallback;
    return LedColor{static_cast<uint8_t>(strtol(text.substring(1, 3).c_str(), nullptr, 16)),
                    static_cast<uint8_t>(strtol(text.substring(3, 5).c_str(), nullptr, 16)),
                    static_cast<uint8_t>(strtol(text.substring(5, 7).c_str(), nullptr, 16))};
}

// 現在時刻のフレーム（kLedCount 個の色）を作る
void renderSlideLed(const LedSpec& s, uint32_t now, LedColor* out) {
    const int speed = s.speed < 1 ? 1 : (s.speed > 10 ? 10 : s.speed);
    const uint32_t delay_base = 11 - speed;
    const bool split = s.pat == LedPat::SplitChase || s.pat == LedPat::SplitBreath ||
                       s.pat == LedPat::SplitBlink || s.pat == LedPat::SplitWave;
    const int half = kLedCount / 2;
    auto colorAt = [&](int i) { return (split && i >= half) ? s.c2 : s.c1; };

    switch (s.pat) {
        case LedPat::Blink:
        case LedPat::SplitBlink: {
            const uint32_t period = 2 * (60 + delay_base * 40);
            const bool on = (now % period) < period / 2;
            for (int i = 0; i < kLedCount; ++i) {
                const LedColor c = colorAt(i);
                out[i] = on ? c : LedColor{0, 0, 0};
            }
            break;
        }
        case LedPat::Wave:
        case LedPat::SplitWave: {
            const uint32_t step_ms = 20 + delay_base * 4;
            const uint32_t step = now / step_ms;
            for (int i = 0; i < kLedCount; ++i) {
                const uint32_t phase = step * 4 + static_cast<uint32_t>(i) * 255 / kLedCount;
                out[i] = scaleLed(colorAt(i), triangleBright(phase));
            }
            break;
        }
        case LedPat::SplitBreath: {
            const uint32_t step_ms = 15 + delay_base * 3;
            const uint32_t p = ((now / step_ms) * 6) % 500;
            uint8_t anim = static_cast<uint8_t>(p < 250 ? p : 499 - p);
            if (anim > 249) anim = 249;
            for (int i = 0; i < kLedCount; ++i) out[i] = scaleLed(colorAt(i), anim);
            break;
        }
        case LedPat::SplitChase: {
            const uint32_t step_ms = 30 + delay_base * 6;
            const int pos = static_cast<int>((now / step_ms) % kLedCount);
            for (int i = 0; i < kLedCount; ++i) out[i] = LedColor{0, 0, 0};
            for (int j = 0; j < 3; ++j) {
                const int idx = (pos + j * 4) % kLedCount;
                out[idx] = scaleLed(colorAt(idx), j == 0 ? 255 : 100);
            }
            break;
        }
        case LedPat::Solid:
        default:
            for (int i = 0; i < kLedCount; ++i) out[i] = s.c1;
            break;
    }
}

// スライドの LED 指定を適用する。空の指定は LED を消す（HermesLLM と同じ）
void applySlideLed(const String& json) {
    slide_led_active = false;
    if (json.isEmpty()) {
        setLeds(LedColor{0, 0, 0});
        return;
    }
    JsonDocument doc;
    if (deserializeJson(doc, json)) {
        setLeds(LedColor{0, 0, 0});
        return;
    }
    if (doc["off"] | false) {
        setLeds(LedColor{0, 0, 0});
        return;
    }

    LedSpec spec;
    const String name = doc["pattern"] | "solid";
    if (name == "blink") spec.pat = LedPat::Blink;
    else if (name == "wave") spec.pat = LedPat::Wave;
    else if (name == "split_chase") spec.pat = LedPat::SplitChase;
    else if (name == "split_breath") spec.pat = LedPat::SplitBreath;
    else if (name == "split_blink") spec.pat = LedPat::SplitBlink;
    else if (name == "split_wave") spec.pat = LedPat::SplitWave;
    else spec.pat = LedPat::Solid;

    spec.c1 = parseHexColor(String(doc["color"] | "#FFFFFF"), LedColor{255, 255, 255});
    spec.c2 = parseHexColor(String(doc["color2"] | ""), spec.c1);
    spec.speed = doc["speed"] | 5;
    Serial.printf("[SLIDE] led pattern=%s speed=%u\n", name.c_str(), (unsigned)spec.speed);

    if (spec.pat == LedPat::Solid) {
        setLeds(spec.c1);  // 動かない
        return;
    }
    slide_led = spec;
    slide_led_active = true;
    slide_led_next_ms = 0;
}

// パターンのフレームを 30ms ごとに出す（ループ内で呼ぶ）
void updateSlideLed(uint32_t now) {
    if (!slide_led_active || now < slide_led_next_ms || !led_io) return;
    slide_led_next_ms = now + 30;
    LedColor frame[kLedCount];
    renderSlideLed(slide_led, now, frame);
    for (int i = 0; i < kLedCount; ++i) {
        led_io->setLedColor(i, frame[i].r, frame[i].g, frame[i].b);
    }
    led_io->refreshLeds();
    static uint32_t next_log_ms = 0;
    if (now >= next_log_ms) {  // デバッグ: 1 秒ごとに送っている色を出す
        next_log_ms = now + 1000;
        Serial.printf("[SLIDE] led frame led0=(%u,%u,%u) led6=(%u,%u,%u)\n",
                      frame[0].r, frame[0].g, frame[0].b, frame[6].r, frame[6].g, frame[6].b);
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
    bubble_scroll.start(bubble, 4000, now);
    slide_bubble_clear_ms = now + 4000;

    applySlideLed(slide_pending.led);  // 空なら LED を消す（HermesLLM と同じ）

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
        slide_led_active = false;
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
    // 頭上タップ（HermesLLM と同じ）: 2 回 = 挨拶、3 回 = うなずき、4 回 = 一回転
    const uint8_t taps = head_touch.takeTapCount();
    if (taps == 2 && dev.greeting_on) slide_greeting_request = true;  // 挨拶 OFF のときは鳴らさない
    if ((taps == 3 || taps == 4) && head_ready && !manualDriveActive() && !slideMotionBusy()) {
        if (taps == 3) {
            startMotion(kSlideNod, sizeof(kSlideNod) / sizeof(kSlideNod[0]), now);
        } else {
            head_servo.spinBegin();
            spinning = true;
            spin_end_ms = now + 2000;
        }
    }

    if (slide_greeting_request && !greeting.playing()) {
        slide_greeting_request = false;
        playGreeting(now);
    }
    updateSpeechMotion(now);

    if (slide_pending.pending) {
        slide_pending.pending = false;
        if (!(slide_pending.h == slide_applied_h && slide_pending.v == slide_applied_v)) {
            applySlide(now);
        }
    }

    bubble_scroll.tick(now);
    if (slide_bubble_clear_ms && now >= slide_bubble_clear_ms) {
        slide_bubble_clear_ms = 0;
        bubble_scroll.stop();
        avatar.setSpeechText("");
    }
    updateSlideLed(now);
    if (motion_steps && now >= motion_next_ms) runMotionStep(now);
    if (spinning && now >= spin_end_ms) {
        head_servo.spinEnd();
        spinning = false;
    }
}

// --- 本体の設定画面 --------------------------------------------------------
// 画面の右上隅をタップで開く。項目: スライドモード / MAVLink / 音量（メーター）/ 明るさ（メーター）/ Wi-Fi 候補 / 再キャリブレーション。
// メーターはバーをタップ、またはドラッグして値を決める。閉じると /settings.json に保存する。
// 開いている間は顔とスライドを止める。

constexpr int kSettingsRowY[5] = {36, 68, 100, 132, 164};  // スライド / MAVLink・状態音 / 音量 / 明るさ / Wi-Fi
constexpr int kSettingsRowH = 30;
constexpr int kSettingsCueX = 168;  // 2 行目の右半分（状態音）の左端。左半分は MAVLink
constexpr int kSettingsGreetX = 176;  // 1 行目の右半分（挨拶）の左端。左半分はスライドモード
constexpr int kSettingsCalibY = 200;  // 再キャリブレーションのボタン
constexpr int kSettingsCalibH = 30;
constexpr int kMeterX = 120;   // メーターの左端
constexpr int kMeterW = 188;   // メーターの幅
constexpr int kMeterH = 16;

// メーターの 1 行を描く（ラベルと値、バー）
void drawMeter(int row_y, const char* label, int value, int vmin, int vmax) {
    char text[32];
    snprintf(text, sizeof(text), "%s %d", label, value);
    M5.Display.setCursor(12, row_y + 7);
    M5.Display.print(text);

    M5.Display.fillRect(kMeterX, row_y + 7, kMeterW, kMeterH, TFT_DARKGREY);
    const int span = vmax - vmin;
    int fill = span > 0 ? (value - vmin) * kMeterW / span : 0;
    if (fill < 0) fill = 0;
    if (fill > kMeterW) fill = kMeterW;
    M5.Display.fillRect(kMeterX, row_y + 7, fill, kMeterH, TFT_GREEN);
}

void drawSettingsScreen() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setFont(&fonts::efontJA_16);
    M5.Display.setTextSize(1);

    M5.Display.setCursor(8, 10);
    M5.Display.print("設定");
    if (WiFi.status() == WL_CONNECTED) {
        M5.Display.setCursor(60, 10);
        char ip_line[32];
        snprintf(ip_line, sizeof(ip_line), "IP %s", WiFi.localIP().toString().c_str());
        M5.Display.print(ip_line);
    }

    // 1 行目は左半分がスライドモード、右半分が挨拶（頭上 2 回タップの intro / outro）
    M5.Display.setCursor(12, kSettingsRowY[0] + 7);
    M5.Display.print(dev.slide_mode ? "スライドモード: ON" : "スライドモード: OFF");
    M5.Display.setCursor(kSettingsGreetX, kSettingsRowY[0] + 7);
    M5.Display.print(dev.greeting_on ? "挨拶: ON" : "挨拶: OFF");

    // 2 行目は左半分が MAVLink、右半分が状態音（声 / ビープ）
    M5.Display.setCursor(12, kSettingsRowY[1] + 7);
    M5.Display.print(dev.mav_enabled ? "MAVLink: ON" : "MAVLink: OFF");
    M5.Display.setCursor(kSettingsCueX, kSettingsRowY[1] + 7);
    M5.Display.print(dev.cue_beep ? "状態音:ビープ" : "状態音:声");

    drawMeter(kSettingsRowY[2], "音量", dev.volume, 0, 100);
    drawMeter(kSettingsRowY[3], "明るさ", dev.brightness, 25, 255);

    char line[64];
    M5.Display.setCursor(12, kSettingsRowY[4] + 7);
    if (settings.wifi_count > 0 && dev.wifi_index < settings.wifi_count) {
        snprintf(line, sizeof(line), "Wi-Fi: %s", settings.wifi[dev.wifi_index].ssid.c_str());
    } else {
        snprintf(line, sizeof(line), "Wi-Fi: 未設定");
    }
    M5.Display.print(line);

    const int back_x = M5.Display.width() - 84;  // 右上隅（入るときと同じ位置）
    M5.Display.fillRoundRect(back_x, 4, 80, 26, 4, TFT_DARKGREEN);
    M5.Display.setCursor(back_x + 8, 8);
    M5.Display.print("戻る >");

    // 再キャリブレーション（水平の基準を取り直す）
    const char* calib_label = "再キャリブレーション";
    if (calibration_started) calib_label = "キャリブレーション中...";
    else if (vehicle->armed()) calib_label = "ARM中は実行できません";
    else if (!imu_ready) calib_label = "IMU がありません";
    M5.Display.fillRoundRect(8, kSettingsCalibY, 304, kSettingsCalibH, 4, TFT_DARKGREY);
    M5.Display.setCursor(20, kSettingsCalibY + 7);
    M5.Display.print(calib_label);
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
    M5.Display.setFont(&fonts::Font0);
    M5.Display.fillScreen(TFT_BLACK);
    avatar.resume();  // HermesLLM と同じ順番（画面を消してから再開する）
    face_refresh_needed = true;
    Serial.println("[SET] settings saved and closed");
}

bool inRect(int x, int y, int rx, int ry, int rw, int rh) {
    return x >= rx && x < rx + rw && y >= ry && y < ry + rh;
}

// バーの x 位置から値を決める（5 刻み）
int meterValueAt(int x, int vmin, int vmax) {
    int v = vmin + (x - kMeterX) * (vmax - vmin) / kMeterW;
    if (v < vmin) v = vmin;
    if (v > vmax) v = vmax;
    return (v / 5) * 5 < vmin ? vmin : (v / 5) * 5;
}

// MAVLink の ON/OFF を切り替える（再起動なし）。
// OFF: 機体を DISARM・HOLD に戻し、通信を止める。ON: 次の接続で UDP を開き直す（ensureNetwork）。
void setMavlinkEnabled(bool on) {
    dev.mav_enabled = on;
    mav_enabled = on;
    if (!on) {
        vehicle->resetState();
        if (udp_open) {
            udp.stop();
            udp_open = false;
            peer_known = false;
        }
    }
    Serial.printf("[SET] MAVLink %s\n", on ? "ON" : "OFF");
}

// 設定画面のタッチ処理。設定画面を開いている間は true を返す。
bool updateSettingsScreen(uint32_t now) {
    (void)now;
    auto t = M5.Touch.getDetail(0);

    if (!settings_open) {
        // 右上隅（画面の幅の右端 60px、上端 60px）で開く
        if (t.wasPressed() && t.x > M5.Display.width() - 60 && t.y < 60) openSettingsScreen();
        return settings_open;
    }

    // キャリブレーションの開始・終了に合わせて、ボタンの表示を更新する
    static bool last_calib_state = false;
    if (calibration_started != last_calib_state) {
        last_calib_state = calibration_started;
        drawSettingsScreen();
    }

    if (!t.wasPressed() && !t.isPressed()) return true;

    const int x = t.x;
    const int y = t.y;

    // メーターはタップとドラッグの両方で値を変える
    const bool on_volume = inRect(x, y, kMeterX - 8, kSettingsRowY[2], kMeterW + 16, kSettingsRowH);
    const bool on_bright = inRect(x, y, kMeterX - 8, kSettingsRowY[3], kMeterW + 16, kSettingsRowH);
    if (on_volume) {
        const int v = meterValueAt(x, 0, 100);
        if (v != dev.volume) {
            dev.volume = static_cast<uint8_t>(v);
            settings.volume = dev.volume;
            M5.Speaker.setVolume(static_cast<uint8_t>(dev.volume * 255 / 100));  // 設定は 0〜100、スピーカーは 0〜255
            drawSettingsScreen();
        }
        return true;
    }
    if (on_bright) {
        const int v = meterValueAt(x, 25, 255);
        if (v != dev.brightness) {
            dev.brightness = static_cast<uint8_t>(v);
            M5.Display.setBrightness(dev.brightness);
            drawSettingsScreen();
        }
        return true;
    }

    if (!t.wasPressed()) return true;  // 以下はタップだけ

    if (inRect(x, y, M5.Display.width() - 90, 0, 90, 40)) {  // 右上隅: 通常画面へ戻る（保存する）
        closeSettingsScreen();
        return false;
    }
    if (inRect(x, y, 8, kSettingsCalibY, 304, kSettingsCalibH)) {
        // 水平の基準を取り直す（ARM 中とセンサ無しは実行しない）
        if (imu_ready && !vehicle->armed() && !calibration_started) {
            attitude.beginCalibration();
            calibration_started = true;
            calibration_end_ms = millis() + kCalibrationMs;
            Serial.println("[SET] re-calibration started from settings: keep still");
        }
        drawSettingsScreen();
        return true;
    }
    if (inRect(x, y, 0, kSettingsRowY[0], kSettingsGreetX, kSettingsRowH)) {
        dev.slide_mode = !dev.slide_mode;
        slide_mode_on = dev.slide_mode;
    } else if (inRect(x, y, kSettingsGreetX, kSettingsRowY[0], 320 - kSettingsGreetX, kSettingsRowH)) {
        dev.greeting_on = !dev.greeting_on;
    } else if (inRect(x, y, 0, kSettingsRowY[1], kSettingsCueX, kSettingsRowH)) {
        setMavlinkEnabled(!dev.mav_enabled);
    } else if (inRect(x, y, kSettingsCueX, kSettingsRowY[1], 320 - kSettingsCueX, kSettingsRowH)) {
        dev.cue_beep = !dev.cue_beep;
    } else if (inRect(x, y, 0, kSettingsRowY[4], 320, kSettingsRowH)) {
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
    M5.Speaker.begin();  // 挨拶の WAV 再生に必要（HermesLLM の setup と同じ）
    Serial.println("[MAV] MAVStackchan Phase 1 boot");

    settings_loaded = loadSettings();
    // 設定画面の値（/settings.json）。無ければ YAML の既定値のまま
    dev.slide_mode = settings.slide_mode;
    dev.volume = settings.volume;
    loadDeviceSettings(dev);
    mav_enabled = dev.mav_enabled;
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
    if (ip_bubble_until_ms && now_ms >= ip_bubble_until_ms) {
        ip_bubble_until_ms = 0;
        if (!settings_open) avatar.setSpeechText("");  // 設定画面の表示中は顔に触らない
    }
    const bool in_settings = updateSettingsScreen(now_ms);  // 右上隅のタップで設定画面
    if (!in_settings) updateSlides(now_ms);
    updateHead(now_ms);
    if (mav_enabled) vehicle->tick(now_ms);
    reportLinkChange();
    reportCueEvents();
    updateCues();
    updateCueMouth(now_ms);
    updateFace(now_ms);

    delay(1);
}
