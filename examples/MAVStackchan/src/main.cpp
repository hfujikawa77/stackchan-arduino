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

#include "AttitudeEstimator.h"
#include "MavlinkVehicle.h"
#include "Settings.h"

namespace {

constexpr char kSettingsPath[] = "/yaml/SC_SecConfig.yaml";
constexpr uint32_t kWifiRetryMs = 5000;
constexpr uint32_t kDisplayIntervalMs = 500;
constexpr uint32_t kImuIntervalMs = 10;          // IMU 読み取り 100Hz
constexpr uint32_t kCalibrationMs = 2000;        // 起動直後に機体を動かさない時間
constexpr float kDegToRad = 0.0174532925f;
constexpr size_t kRxBufferSize = 320;  // MAVLink 2 の最大パケット長以上

MavSettings settings;
MavlinkVehicle* vehicle = nullptr;
bool settings_loaded = false;

AttitudeEstimator attitude;
bool imu_ready = false;
bool calibration_started = false;
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
uint32_t next_display_ms = 0;
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

void drawStatus(uint32_t now_ms) {
    if (now_ms < next_display_ms) return;
    next_display_ms = now_ms + kDisplayIntervalMs;

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(8, 8);
    M5.Display.println("MAVStackchan");

    if (!settings_loaded) {
        M5.Display.println("No settings");
        M5.Display.println("uploadfs needed");
        return;
    }
    if (settings.wifi_count == 0) {
        M5.Display.println("No wifi_list");
        return;
    }
    if (WiFi.status() == WL_CONNECTED) {
        M5.Display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        M5.Display.println("WiFi: connecting");
    }
    M5.Display.printf("QGC: %s\n", vehicle->gcsLinked() ? "LINKED" : "waiting");
    M5.Display.printf("%s / %s\n", vehicle->armed() ? "ARMED" : "DISARMED",
                      vehicle->mode() == VehicleMode::Manual ? "MANUAL" : "HOLD");
    M5.Display.printf("HB tx: %lu\n", static_cast<unsigned long>(vehicle->heartbeatsSent()));
    M5.Display.printf("ATT tx: %lu\n", static_cast<unsigned long>(vehicle->attitudesSent()));
    if (!imu_ready) {
        M5.Display.println("IMU: not found");
    } else if (attitude.calibrated()) {
        const AttitudeEstimator::Sample s = attitude.sample();
        M5.Display.printf("R:%.0f P:%.0f\n", s.roll * 57.2957795f, s.pitch * 57.2957795f);
    } else {
        M5.Display.println("IMU: calibrating");
    }
    M5.Display.printf("Unsupp: %lu\n", static_cast<unsigned long>(vehicle->commandsUnsupported()));
}

}  // namespace

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    Serial.println("[MAV] MAVStackchan Phase 1 boot");

    settings_loaded = loadSettings();

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
    updateImu(now_ms);
    vehicle->tick(now_ms);
    reportLinkChange();
    drawStatus(now_ms);

    delay(1);
}
