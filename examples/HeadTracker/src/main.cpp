// HeadTracker firmware for M5StickC Plus
// Reads MPU6886 IMU, sends pan/tilt angles to Stack-chan via ESP-NOW at 50 Hz.
//
// Mounting: M5StickC Plus fixed horizontally on back of head (landscape).
//   - Head yaw  (L/R turn) → gyro Z → servo X (pan)
//   - Head pitch (nod)     → gyro X → servo Y (tilt)
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

// ── Peer MAC address ─────────────────────────────────────────────────────────
// Set to Stack-chan CoreS3's MAC address after checking its Serial output.
// 0xFF*6 = broadcast (works without specific pairing, convenient for initial test).
static uint8_t STACKCHAN_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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

// ── App state ─────────────────────────────────────────────────────────────────
static bool tracking_enabled = true;
static bool calibrated       = false;
static uint32_t next_send_ms = 0;
static uint32_t next_disp_ms = 0;

// ─────────────────────────────────────────────────────────────────────────────

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void calibrate() {
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
    M5.Display.printf(tracking_enabled ? "ON " : "OFF");
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

    if (esp_now_init() != ESP_OK) {
        M5.Display.println("ESP-NOW ERR");
        Serial.println("ESP-NOW init failed");
        while (true) delay(1000);
    }

    // Register Stack-chan as peer
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, STACKCHAN_MAC, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;
    esp_now_add_peer(&peer_info);

    M5.Imu.begin();
    delay(300);

    // Bootstrap pitch from accelerometer
    float ax, ay, az;
    M5.Imu.getAccelData(&ax, &ay, &az);
    pitch_deg  = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
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

    // Accelerometer pitch (complementary filter reference)
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;

    // Complementary filter: pitch (nod)
    pitch_deg = CF_ALPHA * (pitch_deg + gx * dt) + (1.0f - CF_ALPHA) * acc_pitch;

    // Gyro-only yaw (no magnetometer → drift over time; re-center with BtnA)
    yaw_deg += gz * dt;

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

    if (now >= next_disp_ms) {
        draw_ui(servo_x, servo_y, delta_pan, delta_tilt);
        next_disp_ms = now + 200;
    }
}
