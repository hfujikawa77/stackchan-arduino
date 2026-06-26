// HeadTracker firmware for M5StickC Plus
// Reads MPU6886 IMU, sends pan/tilt angles to Stack-chan via ESP-NOW at 50 Hz.
//
// Mounting: M5StickC Plus fixed on back of head, screen facing out (any rotation).
// Axis mapping is mounting-independent:
//   - Head yaw  (L/R turn) = gyro component about gravity axis   → servo X (pan)
//   - Head pitch (nod)     = gravity tilt toward screen normal Z → servo Y (tilt)
//
// First-time setup:
//   1. Flash Stack-chan (CoreS3) with HermesLLM firmware that includes ESP-NOW receiver.
//   2. Open Serial monitor for Stack-chan, copy the MAC address printed as "ESP-NOW MAC: XX:XX:XX:XX:XX:XX".
//   3. Set STACKCHAN_MAC below to that address and reflash this HeadTracker firmware.
//
// Button operation:
//   BtnA (front): recalibrate center (hold still, press once)
//   BtnB (side):  toggle tracking on / off

#include <M5Unified.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ── Peer MAC address ─────────────────────────────────────────────────────────
// Set to Stack-chan CoreS3's MAC address after checking its Serial output.
// 0xFF*6 = broadcast (works without specific pairing, convenient for initial test).
static uint8_t STACKCHAN_MAC[6] = {0x1C, 0xDB, 0xD4, 0xBA, 0x55, 0x74};

// ── Servo range [degree] ─────────────────────────────────────────────────────
static constexpr float PAN_MIN  = 45.0f;
static constexpr float PAN_MAX  = 135.0f;
static constexpr float TILT_MIN = 65.0f;
static constexpr float TILT_MAX = 115.0f;

// ── Axis gain / inversion ─────────────────────────────────────────────────────
// If the servo moves opposite to your head, flip the sign here.
static constexpr float PAN_GAIN  =  1.0f;  // +1 or -1
static constexpr float TILT_GAIN = -1.0f;  // nod forward → servo looks down

// ── Complementary filter coefficient ─────────────────────────────────────────
static constexpr float CF_ALPHA = 0.96f;   // higher = trust gyro more

// ── Packet sent to Stack-chan ─────────────────────────────────────────────────
typedef struct {
    float pan;   // horizontal servo target [degree]
    float tilt;  // vertical servo target [degree]
} HeadTrackPacket;

// ── IMU state ─────────────────────────────────────────────────────────────────
static float pitch_deg   = 0.0f;
static float yaw_deg     = 0.0f;
static float ref_pitch   = 0.0f;
static float ref_yaw     = 0.0f;
static uint32_t last_imu_ms = 0;
static float glp_x = 0.0f, glp_y = 1.0f, glp_z = 0.0f;  // low-passed accel (gravity estimate)
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

// ── App state ─────────────────────────────────────────────────────────────────
static bool    tracking_enabled = true;
static uint8_t espnow_channel   = 1;
static volatile bool tx_ok      = false;  // last ESP-NOW send was ACKed by Stack-chan
static volatile uint16_t tx_fail_streak = 0;
static bool calibrated       = false;
static uint32_t next_send_ms = 0;
static uint32_t next_disp_ms = 0;

// ─────────────────────────────────────────────────────────────────────────────

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void on_send_status(const uint8_t* mac, esp_now_send_status_t status) {
    tx_ok = (status == ESP_NOW_SEND_SUCCESS);
    if (tx_ok) {
        tx_fail_streak = 0;
    } else if (tx_fail_streak < 0xFFFF) {
        tx_fail_streak = tx_fail_streak + 1;
    }
}

static void calibrate() {
    // Measure gyro bias while held still (~400ms)
    float bx = 0.0f, by = 0.0f, bz = 0.0f;
    for (int i = 0; i < 40; i++) {
        float x, y, z;
        M5.Imu.getGyroData(&x, &y, &z);
        bx += x; by += y; bz += z;
        delay(10);
    }
    gyro_bias_x = bx / 40.0f;
    gyro_bias_y = by / 40.0f;
    gyro_bias_z = bz / 40.0f;

    ref_pitch  = pitch_deg;
    ref_yaw    = yaw_deg;
    calibrated = true;
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(GREEN);
    M5.Display.setCursor(4, 24);
    M5.Display.println("Calibrated");
    delay(400);
}

static void draw_ui(float servo_x, float servo_y, float dp, float dt) {
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(4, 4);
    M5.Display.printf("pan  %5.1f\n", servo_x);
    M5.Display.printf("tilt %5.1f\n", servo_y);
    M5.Display.setTextColor(tracking_enabled ? GREEN : RED);
    M5.Display.printf("dP %+5.1f\n", dp);
    M5.Display.printf("dT %+5.1f\n", dt);
    M5.Display.setTextColor(DARKGREY);
    M5.Display.printf(tracking_enabled ? "ON  ch:%d " : "OFF ch:%d ", espnow_channel);
    M5.Display.setTextColor(tx_ok ? GREEN : RED);
    M5.Display.print(tx_ok ? "TX" : "NG");
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);

    M5.Display.setRotation(1);  // landscape
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.println("HeadTracker");
    M5.Display.println("Starting...");

    // Print own MAC for pairing reference
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.printf("HeadTracker MAC: %s\n", WiFi.macAddress().c_str());

    // Scan to detect WiFi channel used by the nearest AP (must match CoreS3's WiFi channel)
    M5.Display.setCursor(4, 40);
    M5.Display.setTextColor(YELLOW);
    M5.Display.print("Scanning ch...");
    {
        int n = WiFi.scanNetworks(false, true);
        int best_rssi = -100;
        for (int i = 0; i < n; i++) {
            if (WiFi.RSSI(i) > best_rssi) {
                best_rssi    = WiFi.RSSI(i);
                espnow_channel = (uint8_t)WiFi.channel(i);
            }
        }
        WiFi.scanDelete();
        esp_wifi_set_channel(espnow_channel, WIFI_SECOND_CHAN_NONE);
        Serial.printf("ESP-NOW channel: %d\n", espnow_channel);
    }

    if (esp_now_init() != ESP_OK) {
        M5.Display.println("ESP-NOW ERR");
        Serial.println("ESP-NOW init failed");
        while (true) delay(1000);
    }
    esp_now_register_send_cb(on_send_status);

    // Register Stack-chan as peer
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, STACKCHAN_MAC, 6);
    peer_info.channel = 0;  // follow the radio's current channel (we hop on TX failure)
    peer_info.encrypt = false;
    esp_now_add_peer(&peer_info);

    M5.Imu.begin();
    delay(300);

    // Bootstrap gravity estimate and pitch from accelerometer
    float ax, ay, az;
    M5.Imu.getAccelData(&ax, &ay, &az);
    {
        float n = sqrtf(ax * ax + ay * ay + az * az);
        if (n > 0.1f) { glp_x = ax / n; glp_y = ay / n; glp_z = az / n; }
        pitch_deg = asinf(clampf(glp_z, -1.0f, 1.0f)) * 57.2958f;
    }
    yaw_deg    = 0.0f;
    last_imu_ms = millis();

    calibrate();  // treat startup position as center
    Serial.println("HeadTracker ready. BtnA=recalib BtnB=toggle");
}

void loop() {
    M5.update();

    // BtnA: recalibrate center
    if (M5.BtnA.wasPressed()) {
        calibrate();
    }

    // BtnB (side): toggle on/off
    if (M5.BtnB.wasPressed()) {
        tracking_enabled = !tracking_enabled;
    }

    uint32_t now = millis();
    float dt = (now - last_imu_ms) * 0.001f;
    last_imu_ms = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;  // clamp on first tick or hiccup

    // Read IMU
    float ax, ay, az, gx, gy, gz;
    M5.Imu.getAccelData(&ax, &ay, &az);
    M5.Imu.getGyroData(&gx, &gy, &gz);
    gx -= gyro_bias_x;
    gy -= gyro_bias_y;
    gz -= gyro_bias_z;

    // Low-pass accel to track gravity direction (unit vector, device frame)
    glp_x += (ax - glp_x) * 0.10f;
    glp_y += (ay - glp_y) * 0.10f;
    glp_z += (az - glp_z) * 0.10f;
    float gn = sqrtf(glp_x * glp_x + glp_y * glp_y + glp_z * glp_z);
    if (gn < 0.1f) gn = 1.0f;
    const float ux = glp_x / gn, uy = glp_y / gn, uz = glp_z / gn;

    // Yaw: gyro component about the gravity (vertical) axis — mounting-independent.
    // No magnetometer → drifts over time; re-center with BtnA.
    yaw_deg += (gx * ux + gy * uy + gz * uz) * dt;

    // Pitch: gravity tilt toward the screen normal (device Z).
    // acc_pitch = asin(ĝ·ẑ); gyro rate about ê = (ĝ×ẑ)/|ĝ×ẑ| matches d(acc_pitch)/dt.
    float acc_pitch = asinf(clampf(uz, -1.0f, 1.0f)) * 57.2958f;
    float s = sqrtf(ux * ux + uy * uy);
    if (s > 0.05f) {
        const float ex = uy / s, ey = -ux / s;
        const float pitch_rate = -(gx * ex + gy * ey);
        pitch_deg = CF_ALPHA * (pitch_deg + pitch_rate * dt) + (1.0f - CF_ALPHA) * acc_pitch;
    } else {
        pitch_deg = acc_pitch;  // screen nearly horizontal; nod axis ill-defined
    }

    // 50 Hz transmission
    if (now < next_send_ms) {
        delay(2);
        return;
    }
    next_send_ms = now + 20;

    if (!tracking_enabled || !calibrated) {
        if (now >= next_disp_ms) {
            draw_ui(90.0f, 90.0f, 0.0f, 0.0f);
            next_disp_ms = now + 200;
        }
        return;
    }

    float delta_pan  = (yaw_deg   - ref_yaw)   * PAN_GAIN;
    float delta_tilt = (pitch_deg - ref_pitch)  * TILT_GAIN;

    float servo_x = clampf(90.0f + delta_pan,  PAN_MIN,  PAN_MAX);
    float servo_y = clampf(90.0f + delta_tilt, TILT_MIN, TILT_MAX);

    HeadTrackPacket pkt = { servo_x, servo_y };
    esp_now_send(STACKCHAN_MAC, (uint8_t*)&pkt, sizeof(pkt));

    // Auto channel hop: if Stack-chan stops ACKing (e.g. its WiFi moved to a
    // different channel after reboot), cycle through 1-13 until ACKs resume.
    if (tx_fail_streak > 50) {
        espnow_channel = espnow_channel % 13 + 1;
        esp_wifi_set_channel(espnow_channel, WIFI_SECOND_CHAN_NONE);
        tx_fail_streak = 25;  // retry next channel after ~0.5s if still failing
        Serial.printf("TX failing, hopping to ch %d\n", espnow_channel);
    }

    if (now >= next_disp_ms) {
        draw_ui(servo_x, servo_y, delta_pan, delta_tilt);
        next_disp_ms = now + 200;
    }
}
