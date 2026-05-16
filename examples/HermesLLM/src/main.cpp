#include <Arduino.h>
#include <M5Unified.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include <math.h>
#include <array>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Stackchan_system_config.h>
#include <Stackchan_servo.h>
#include "Si12T.h"
#include <Avatar.h>
using namespace m5avatar;
bool touchedZone(int zone);
void show(const String& text);
void handle_speak_server();
static void led_force_clear();
static const char B64_TABLE[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static size_t base64_encode(uint8_t* out, const uint8_t* in, size_t in_len) {
    size_t j = 0;
    for (size_t i = 0; i < in_len; i += 3) {
        uint8_t a = in[i];
        uint8_t b = (i + 1 < in_len) ? in[i + 1] : 0;
        uint8_t c = (i + 2 < in_len) ? in[i + 2] : 0;
        uint32_t t = ((uint32_t)a << 16) | ((uint32_t)b << 8) | c;
        out[j++] = B64_TABLE[(t >> 18) & 0x3F];
        out[j++] = B64_TABLE[(t >> 12) & 0x3F];
        out[j++] = (i + 1 < in_len) ? B64_TABLE[(t >> 6) & 0x3F] : '=';
        out[j++] = (i + 2 < in_len) ? B64_TABLE[t & 0x3F] : '=';
    }
    out[j] = '\0';
    return j;
}

#define MIC_SAMPLE_RATE 16000
#define MIC_RECORD_SEC  5
#define MIC_BUF_SAMPLES (MIC_SAMPLE_RATE * MIC_RECORD_SEC)
#define MIC_BUF_BYTES   (MIC_BUF_SAMPLES * 2)

StackchanSystemConfig system_config;

// --- Avatar ---
Avatar avatar;

// --- Servo ---
StackchanSERVO sc_servo;
static bool servo_ready = false;
static volatile bool servo_idle_enabled = true;
static int16_t srv_cx = 150, srv_cy = 90;
static uint32_t servo_idle_next_ms = 0;
static uint32_t head_pat_cooldown_until_ms = 0;
static uint32_t head_touch_next_poll_ms = 0;
static volatile bool led_effect_active = false;
static SemaphoreHandle_t io_mutex = nullptr;

// --- Settings ---
enum AppMode { MODE_NORMAL, MODE_SETTINGS, MODE_BPM_DETECT, MODE_BPM_PLAY };
static AppMode app_mode = MODE_NORMAL;
static int setting_volume     = 80;
static int setting_brightness = 200;
static int brightness_val     = 200;

static constexpr size_t BPM_RECORD_LENGTH = 256;
static constexpr uint32_t BPM_SAMPLE_RATE = 16000;
static int16_t bpm_buf[BPM_RECORD_LENGTH];
static bool bpm_audio_active = false;
static uint32_t bpm_mode_started_ms = 0;
static uint32_t bpm_toggle_cooldown_until_ms = 0;
static float bpm_noise_floor = 0.0f;
static uint8_t bpm_calibration_frames = 0;
static float bpm_env = 0.0f;
static float bpm_avg_env = 0.0f;
static uint32_t bpm_last_peak_ms = 0;
static uint32_t bpm_peak_intervals[8] = {0};
static uint8_t bpm_peak_interval_count = 0;
static uint8_t bpm_peak_interval_index = 0;
static uint16_t detected_bpm = 0;
static uint16_t play_bpm = 120;
static uint32_t bpm_next_log_ms = 0;
static uint32_t bpm_play_next_step_ms = 0;
static uint32_t bpm_play_hold_until_ms = 0;
static bool bpm_play_swing_right = true;

class ScopedLock {
public:
    explicit ScopedLock(SemaphoreHandle_t mutex) : mutex_(mutex), locked_(false) {
        if (mutex_) locked_ = xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE;
    }
    ~ScopedLock() {
        if (locked_) xSemaphoreGive(mutex_);
    }
private:
    SemaphoreHandle_t mutex_;
    bool locked_;
};

class HeadTouchSensor {
public:
    void begin() {
        ScopedLock lock(io_mutex);
        sensor.begin();
        swipe_forward = false;
        swipe_backward = false;
        in_gesture = false;
        for (int i = 0; i < 3; ++i) {
            intensities[i] = 0;
            touched_flag[i] = false;
            touch_start_time[i] = 0;
        }
    }

    void update() {
        {
            ScopedLock lock(io_mutex);
            sensor.read_touch_result();
            sensor.parse_touch_result();
        }
        swipe_forward = false;
        swipe_backward = false;

        bool any_touched = false;
        uint32_t now = millis();
        for (int i = 0; i < 3; ++i) {
            intensities[(3 - 1) - i] = sensor.point_type[i];
            if (intensities[(3 - 1) - i] > 0) any_touched = true;
        }
        if (intensities != last_logged_intensities) {
            Serial.printf("Head touch: f=%u m=%u b=%u\n", intensities[0], intensities[1], intensities[2]);
            last_logged_intensities = intensities;
        }
        for (int i = 0; i < 3; ++i) {
            if (intensities[i] > 0) {
                if (!touched_flag[i]) {
                    touched_flag[i] = true;
                    touch_start_time[i] = now;
                }
            }
        }

        if (!any_touched) {
            in_gesture = false;
            for (int i = 0; i < 3; ++i) touched_flag[i] = false;
            return;
        }

        if (touched_flag[0] && touched_flag[1] && touched_flag[2] && !in_gesture) {
            int32_t t1_0 = static_cast<int32_t>(touch_start_time[1]) - static_cast<int32_t>(touch_start_time[0]);
            int32_t t2_1 = static_cast<int32_t>(touch_start_time[2]) - static_cast<int32_t>(touch_start_time[1]);
            int32_t t1_2 = static_cast<int32_t>(touch_start_time[1]) - static_cast<int32_t>(touch_start_time[2]);
            int32_t t0_1 = static_cast<int32_t>(touch_start_time[0]) - static_cast<int32_t>(touch_start_time[1]);
            const int32_t max_swipe_interval = 400;
            const int32_t min_swipe_interval = 30;

            if (t1_0 > min_swipe_interval && t2_1 > min_swipe_interval &&
                t1_0 < max_swipe_interval && t2_1 < max_swipe_interval) {
                swipe_forward = true;
                in_gesture = true;
                Serial.println("Head pat swipe forward");
            } else if (t1_2 > min_swipe_interval && t0_1 > min_swipe_interval &&
                       t1_2 < max_swipe_interval && t0_1 < max_swipe_interval) {
                swipe_backward = true;
                in_gesture = true;
                Serial.println("Head pat swipe backward");
            }
        }
    }

    bool was_swiped_forward() const { return swipe_forward; }
    bool was_swiped_backward() const { return swipe_backward; }

private:
    Si12T sensor = Si12T(SI12T_Type_High, SI12T_Sensitivity_Level_4);
    std::array<uint8_t, 3> intensities = {0, 0, 0};
    std::array<uint8_t, 3> last_logged_intensities = {255, 255, 255};
    uint32_t touch_start_time[3] = {0, 0, 0};
    bool touched_flag[3] = {false, false, false};
    bool in_gesture = false;
    bool swipe_forward = false;
    bool swipe_backward = false;
};

static HeadTouchSensor head_touch_sensor;

// --- HTTP Server ---
WiFiServer speak_server(80);
static volatile bool busy = false;
static uint32_t error_clear_at_ms = 0;

// --- LED ---
#define LED_NUM 12
static m5::PY32IOExpander_Class* led_io = nullptr;
enum LedMode { LED_OFF, LED_HEARING, LED_THINKING, LED_SPEAKING, LED_ERROR, LED_HAPPY };
static volatile LedMode led_mode = LED_OFF;

static constexpr uint32_t LED_IDLE_INTERVAL_MS = 180000;
static constexpr uint32_t LED_IDLE_DURATION_MS = 20000;
static constexpr uint8_t LED_IDLE_BRIGHTNESS_MIN = 6;
static constexpr uint8_t LED_IDLE_BRIGHTNESS_MAX = 72;
static constexpr uint32_t HEAD_PAT_REACTION_MS = 1800;
static constexpr uint32_t HEAD_PAT_COOLDOWN_MS = 4000;

enum LedIdleLogEvent : uint8_t {
    LED_IDLE_LOG_NONE = 0,
    LED_IDLE_LOG_START,
};

struct LedColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

static bool time_reached(uint32_t now, uint32_t target) {
    return static_cast<int32_t>(now - target) >= 0;
}

static bool led_idle_base_allowed() {
    return led_mode == LED_OFF && !busy;
}

static volatile LedIdleLogEvent led_idle_log_event = LED_IDLE_LOG_NONE;
static volatile uint8_t led_idle_log_r = 0;
static volatile uint8_t led_idle_log_g = 0;
static volatile uint8_t led_idle_log_b = 0;
static volatile bool led_idle_once_requested = false;
static volatile bool led_idle_timer_reset_requested = false;
static volatile bool led_force_clear_requested = false;

static void queue_led_idle_log(LedIdleLogEvent event, LedColor color = {0, 0, 0}) {
    led_idle_log_r = color.r;
    led_idle_log_g = color.g;
    led_idle_log_b = color.b;
    led_idle_log_event = event;
}

static void led_idle_kick_once() {
    led_idle_timer_reset_requested = true;
    led_idle_once_requested = true;
}

static void led_force_clear() {
    led_force_clear_requested = true;
}

static LedColor random_idle_color() {
    static constexpr LedColor palette[] = {
        {110, 78, 20},
        {80, 96, 22},
        {92, 44, 26},
        {62, 88, 34},
        {96, 58, 12},
        {76, 34, 84},
    };
    return palette[random(0, static_cast<long>(sizeof(palette) / sizeof(palette[0])))];
}

static void led_all(uint8_t r, uint8_t g, uint8_t b) {
    ScopedLock lock(io_mutex);
    for (int i = 0; i < LED_NUM; i++) led_io->setLedColor(i, r, g, b);
    led_io->refreshLeds();
}

static void led_task(void*) {
    uint8_t anim = 0;
    int8_t  anim_dir = 1;
    uint8_t idle_anim = LED_IDLE_BRIGHTNESS_MIN;
    int8_t  idle_anim_dir = 1;
    uint8_t chase_pos = 0;
    bool    blink_state = false;
    bool    idle_active = false;
    uint32_t idle_next_ms = millis() + LED_IDLE_INTERVAL_MS;
    uint32_t idle_started_ms = 0;
    uint32_t idle_end_ms = 0;
    LedColor idle_color = {0, 0, 0};
    while (true) {
        if (!led_io) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        uint32_t now = millis();
        bool idle_base_allowed = led_idle_base_allowed();
        bool idle_timer_reset_requested = led_idle_timer_reset_requested;
        bool idle_once_requested = led_idle_once_requested;
        bool force_clear_requested = led_force_clear_requested;

        if (force_clear_requested) {
            led_force_clear_requested = false;
            idle_active = false;
            idle_next_ms = now + LED_IDLE_INTERVAL_MS;
            idle_anim = LED_IDLE_BRIGHTNESS_MIN;
            idle_anim_dir = 1;
            anim = 0;
            anim_dir = 1;
            blink_state = false;
            led_effect_active = false;
            led_all(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (idle_timer_reset_requested) {
            led_idle_timer_reset_requested = false;
            idle_next_ms = now + LED_IDLE_INTERVAL_MS;
        }

        if (!idle_base_allowed) {
            if (idle_active) led_all(0, 0, 0);
            idle_active = false;
            idle_next_ms = now + LED_IDLE_INTERVAL_MS;
            led_idle_once_requested = false;
            idle_anim = LED_IDLE_BRIGHTNESS_MIN;
            idle_anim_dir = 1;
        } else if (idle_once_requested) {
            led_idle_once_requested = false;
            idle_active = true;
            idle_started_ms = now;
            idle_end_ms = now + LED_IDLE_DURATION_MS;
            idle_color = random_idle_color();
            idle_anim = LED_IDLE_BRIGHTNESS_MIN;
            idle_anim_dir = 1;
            queue_led_idle_log(LED_IDLE_LOG_START, idle_color);
        } else if (!idle_active && servo_idle_enabled && time_reached(now, idle_next_ms)) {
            idle_active = true;
            idle_started_ms = now;
            idle_end_ms = now + LED_IDLE_DURATION_MS;
            idle_color = random_idle_color();
            idle_anim = LED_IDLE_BRIGHTNESS_MIN;
            idle_anim_dir = 1;
            queue_led_idle_log(LED_IDLE_LOG_START, idle_color);
        } else if (idle_active && time_reached(now, idle_end_ms)) {
            idle_active = false;
            idle_next_ms = now + LED_IDLE_INTERVAL_MS;
            idle_anim = LED_IDLE_BRIGHTNESS_MIN;
            idle_anim_dir = 1;
            led_all(0, 0, 0);
        }

        switch (led_mode) {
            case LED_OFF:
                if (!idle_active) {
                    led_effect_active = false;
                    led_all(0, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    break;
                }
                {
                    led_effect_active = true;
                    if (idle_anim_dir > 0) {
                        if (idle_anim < LED_IDLE_BRIGHTNESS_MAX - 2) idle_anim += 2;
                        else idle_anim_dir = -1;
                    } else {
                        if (idle_anim > LED_IDLE_BRIGHTNESS_MIN + 2) idle_anim -= 2;
                        else idle_anim_dir = 1;
                    }
                    uint8_t brightness = idle_anim;
                    led_all((idle_color.r * brightness) / LED_IDLE_BRIGHTNESS_MAX,
                            (idle_color.g * brightness) / LED_IDLE_BRIGHTNESS_MAX,
                            (idle_color.b * brightness) / LED_IDLE_BRIGHTNESS_MAX);
                    vTaskDelay(pdMS_TO_TICKS(45));
                }
                break;
            case LED_HEARING:
                led_effect_active = true;
                if (anim_dir > 0) { if (anim < 252) anim += 4; else anim_dir = -1; }
                else              { if (anim > 4)   anim -= 4; else anim_dir =  1; }
                led_all(0, 0, anim);  // blue breath
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            case LED_THINKING:
                led_effect_active = true;
                {
                ScopedLock lock(io_mutex);
                for (int i = 0; i < LED_NUM; i++) led_io->setLedColor(i, 0, 0, 0);
                for (int i = 0; i < 3; i++) led_io->setLedColor((chase_pos + i * 4) % LED_NUM, 180, 160, 0);
                led_io->refreshLeds();
                }
                chase_pos = (chase_pos + 1) % LED_NUM;
                vTaskDelay(pdMS_TO_TICKS(55));
                break;
            case LED_SPEAKING:
                led_effect_active = true;
                if (anim_dir > 0) { if (anim < 249) anim += 6; else anim_dir = -1; }
                else              { if (anim > 6)   anim -= 6; else anim_dir =  1; }
                led_all(0, anim, 0);  // green breath
                vTaskDelay(pdMS_TO_TICKS(25));
                break;
            case LED_ERROR:
                led_effect_active = true;
                blink_state = !blink_state;
                led_all(blink_state ? 200 : 0, 0, 0);  // red blink
                vTaskDelay(pdMS_TO_TICKS(300));
                break;
            case LED_HAPPY:
                led_effect_active = true;
                if (anim_dir > 0) { if (anim < 249) anim += 6; else anim_dir = -1; }
                else              { if (anim > 6)   anim -= 6; else anim_dir =  1; }
                led_all(anim, (anim * 3) / 5, anim / 6);  // warm happy breath
                vTaskDelay(pdMS_TO_TICKS(25));
                break;
        }
    }
}

void led_init() {
    led_io = sc_servo.getIOExpander();
    if (!led_io) { M5_LOGE("LED: no ioexpander"); return; }
    if (!io_mutex) io_mutex = xSemaphoreCreateMutex();
    led_io->setLedCount(LED_NUM);
    led_all(0, 0, 0);
    xTaskCreatePinnedToCore(led_task, "led", 2048, nullptr, 1, nullptr, PRO_CPU_NUM);
}

void led_set(LedMode mode) { led_mode = mode; }

static void clear_error_state() {
    error_clear_at_ms = 0;
    if (led_mode == LED_ERROR) {
        led_set(LED_OFF);
        led_force_clear();
    }
    avatar.setExpression(Expression::Neutral);
}

static void show_error_state(const String& text, uint32_t hold_ms = 3000) {
    avatar.setExpression(Expression::Sad);
    led_set(LED_ERROR);
    show(text);
    error_clear_at_ms = millis() + hold_ms;
}

static void bpm_stop_audio() {
    if (!bpm_audio_active) return;
    while (M5.Mic.isRecording()) { delay(1); }
    M5.Mic.end();
    M5.Speaker.begin();
    bpm_audio_active = false;
}

static void bpm_start_audio() {
    if (bpm_audio_active) return;
    M5.Speaker.end();
    auto mic_cfg = M5.Mic.config();
    mic_cfg.sample_rate = BPM_SAMPLE_RATE;
    mic_cfg.over_sampling = 1;
    mic_cfg.magnification = 2;
    mic_cfg.noise_filter_level = 2;
    M5.Mic.config(mic_cfg);
    M5.Mic.begin();
    bpm_audio_active = true;
}

void led_idle_log_tick() {
    LedIdleLogEvent event = led_idle_log_event;
    if (event == LED_IDLE_LOG_NONE) return;
    uint8_t r = led_idle_log_r;
    uint8_t g = led_idle_log_g;
    uint8_t b = led_idle_log_b;
    led_idle_log_event = LED_IDLE_LOG_NONE;

    switch (event) {
        case LED_IDLE_LOG_START:
            Serial.printf("LED idle start rgb=(%u,%u,%u)\n", r, g, b);
            break;
        default:
            break;
    }
}

static bool head_pat_detected() {
    head_touch_sensor.update();
    return head_touch_sensor.was_swiped_forward() || head_touch_sensor.was_swiped_backward();
}

static void head_pat_reaction() {
    if (busy) {
        Serial.println("Head pat ignored: busy");
        return;
    }
    if (!servo_idle_enabled) {
        Serial.println("Head pat ignored: servo off");
        return;
    }
    uint32_t now = millis();
    if (!time_reached(now, head_pat_cooldown_until_ms)) {
        Serial.println("Head pat ignored: cooldown");
        return;
    }

    head_pat_cooldown_until_ms = now + HEAD_PAT_COOLDOWN_MS;
    Serial.println("Head pat reaction start");
    busy = true;
    led_set(LED_HAPPY);
    avatar.setExpression(Expression::Happy);
    avatar.setLeftGaze(-0.18f, 0.10f);
    avatar.setRightGaze(-0.18f, 0.10f);
    show("Pat pat");

    uint32_t reaction_started_ms = millis();
    uint32_t next_motion_ms = reaction_started_ms;
    bool motion_phase = false;
    while (!time_reached(millis(), reaction_started_ms + HEAD_PAT_REACTION_MS)) {
        M5.update();
        handle_speak_server();
        led_idle_log_tick();

        uint32_t loop_now = millis();
        if (servo_ready && servo_idle_enabled && time_reached(loop_now, next_motion_ms)) {
            motion_phase = !motion_phase;
            sc_servo.moveXY(srv_cx + (motion_phase ? 12 : -12), srv_cy - 8, 220);
            next_motion_ms = loop_now + 320;
        }
        delay(10);
    }

    if (servo_ready) sc_servo.moveXY(srv_cx, srv_cy, 350);
    avatar.setLeftGaze(0.0f, 0.0f);
    avatar.setRightGaze(0.0f, 0.0f);
    avatar.setExpression(Expression::Neutral);
    led_set(LED_OFF);
    show("");
    busy = false;
}

void servo_begin() {
    auto* sx = system_config.getServoInfo(AXIS_X);
    auto* sy = system_config.getServoInfo(AXIS_Y);
    if (sx->pin == 0) return;
    sc_servo.begin(sx->pin, sx->start_degree, sx->offset,
                   sy->pin, sy->start_degree, sy->offset,
                   (ServoType)system_config.getServoType());
    srv_cx = sx->start_degree;
    srv_cy = sy->start_degree;
    servo_ready = true;
    auto* si = system_config.getServoInterval(AvatarMode::NORMAL);
    servo_idle_next_ms = millis() + si->interval_min;
    M5_LOGI("Servo ready cx=%d cy=%d type=%d", srv_cx, srv_cy, system_config.getServoType());
}

void servo_idle_tick() {
    if (!servo_ready || !servo_idle_enabled) return;
    uint32_t now = millis();
    if (now < servo_idle_next_ms) return;
    // If we've been blocked for >5 sec (STT/Hermes/TTS), defer idle move
    if (servo_idle_next_ms < now - 5000) {
        servo_idle_next_ms = now + 2000;
        return;
    }
    auto* si = system_config.getServoInterval(AvatarMode::NORMAL);
    int tx = srv_cx + random(-20, 21);
    int ty = srv_cy + random(-12, 1);  // nod range: center to center-12
    uint32_t move_ms = random(si->move_min, si->move_max + 1);
    sc_servo.moveXY(tx, ty, move_ms);
    // Random gaze shift matching servo direction
    float gv = random(-20, 21) / 100.0f;
    float gh = random(-40, 41) / 100.0f;
    avatar.setLeftGaze(gv, gh);
    avatar.setRightGaze(gv, gh);
    servo_idle_next_ms = millis() + random(si->interval_min, si->interval_max + 1);
}

String hermes_endpoint = "";
String hermes_model    = "";
String hermes_api_key  = "";
int    tts_volume          = 100;
int    hermes_max_tokens   = 80;
int    hermes_timeout_ms   = 60000;
String voicevox_host    = "";  // e.g. "192.168.1.100:50021" — use local VOICEVOX if set
int    voicevox_speaker = 1;

static bool ensure_wifi_connected(uint32_t timeout_ms = 15000) {
    if (WiFi.status() == WL_CONNECTED) return true;

    M5_LOGW("WiFi disconnected (status=%d), reconnecting...", WiFi.status());
    show("WiFi reconnect...");

    wifi_s* wifi = system_config.getWiFiSetting();
    uint32_t started_ms = millis();
    bool begin_called = false;
    while (WiFi.status() != WL_CONNECTED && (millis() - started_ms) < timeout_ms) {
        if (!begin_called) {
            WiFi.disconnect(false, true);
            delay(100);
            WiFi.begin(wifi->ssid.c_str(), wifi->password.c_str());
            begin_called = true;
        } else {
            WiFi.reconnect();
        }
        delay(500);
        M5.update();
    }

    if (WiFi.status() == WL_CONNECTED) {
        M5_LOGI("WiFi reconnected: %s", WiFi.localIP().toString().c_str());
        show("IP: " + WiFi.localIP().toString());
        return true;
    }

    M5_LOGE("WiFi reconnect failed (status=%d)", WiFi.status());
    show("WiFi FAILED");
    return false;
}

void load_hermes_config(fs::FS& fs) {
    File f = fs.open("/yaml/SC_SecConfig.yaml", "r");
    if (!f) { M5_LOGE("SC_SecConfig.yaml not found"); return; }
    String yaml = f.readString();
    f.close();

    String normalized_yaml = "";
    int line_start = 0;
    while (line_start < (int)yaml.length()) {
        int line_end = yaml.indexOf('\n', line_start);
        if (line_end < 0) line_end = yaml.length();
        String line = yaml.substring(line_start, line_end);
        int comment_pos = line.indexOf('#');
        if (comment_pos >= 0) line = line.substring(0, comment_pos);
        normalized_yaml += line + "\n";
        line_start = line_end + 1;
    }

    int hermes_pos = normalized_yaml.indexOf("hermes:");
    if (hermes_pos >= 0) {
        auto extract = [&](const char* key) -> String {
            String search = String("  ") + key + ": \"";
            int pos = normalized_yaml.indexOf(search, hermes_pos);
            if (pos < 0) return "";
            pos += search.length();
            return normalized_yaml.substring(pos, normalized_yaml.indexOf("\"", pos));
        };
        hermes_endpoint = extract("endpoint");
        hermes_model    = extract("model");
        hermes_api_key  = extract("api_key");
        M5_LOGI("Hermes endpoint: %s", hermes_endpoint.c_str());
    }

    auto extractInt = [&](const char* key, int defaultVal) -> int {
        String search = String(key) + ":";
        int pos = normalized_yaml.indexOf(search);
        if (pos < 0) return defaultVal;
        pos += search.length();
        while (pos < (int)normalized_yaml.length() && normalized_yaml[pos] == ' ') pos++;
        return normalized_yaml.substring(pos).toInt();
    };
    tts_volume        = extractInt("tts_volume", tts_volume);
    hermes_max_tokens = extractInt("hermes_max_tokens", hermes_max_tokens);
    hermes_timeout_ms = extractInt("hermes_timeout_ms", hermes_timeout_ms);
    voicevox_speaker  = extractInt("voicevox_speaker", voicevox_speaker);
    M5_LOGI("tts_volume: %d, hermes_max_tokens: %d, hermes_timeout_ms: %d", tts_volume, hermes_max_tokens, hermes_timeout_ms);

    auto extractStr = [&](const char* key) -> String {
        String search = String(key) + ": \"";
        int pos = normalized_yaml.indexOf(search);
        if (pos < 0) return "";
        pos += search.length();
        return normalized_yaml.substring(pos, normalized_yaml.indexOf("\"", pos));
    };
    voicevox_host = extractStr("voicevox_host");
    if (!voicevox_host.isEmpty()) M5_LOGI("Local VOICEVOX: %s speaker=%d", voicevox_host.c_str(), voicevox_speaker);
    brightness_val = extractInt("brightness", brightness_val);
}

String url_encode(const String& text) {
    String encoded;
    for (int i = 0; i < (int)text.length(); i++) {
        uint8_t c = (uint8_t)text[i];
        if (isAlphaNumeric(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            encoded += (char)c;
        } else {
            char buf[4];
            snprintf(buf, sizeof(buf), "%%%02X", c);
            encoded += buf;
        }
    }
    return encoded;
}

String call_hermes(const String& user_message) {
    if (hermes_endpoint.isEmpty()) return "Hermes not configured.";
    if (!ensure_wifi_connected()) return "WiFi reconnect failed.";

    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure();
    http.begin(client, hermes_endpoint);
    http.setTimeout(hermes_timeout_ms);
    http.addHeader("Content-Type", "application/json");
    if (!hermes_api_key.isEmpty())
        http.addHeader("Authorization", "Bearer " + hermes_api_key);

    JsonDocument doc;
    doc["model"] = hermes_model;
    JsonArray messages = doc["messages"].to<JsonArray>();
    JsonObject msg = messages.add<JsonObject>();
    msg["role"] = "user"; msg["content"] = user_message;
    doc["max_tokens"] = hermes_max_tokens; doc["stream"] = false;

    String body; serializeJson(doc, body);
    int status = http.POST(body);
    if (status != 200) { http.end(); return "Error: HTTP " + String(status); }

    String response = http.getString();
    http.end();

    JsonDocument resp;
    if (deserializeJson(resp, response)) return "Parse error";
    const char* content = resp["choices"][0]["message"]["content"];
    return content ? String(content) : "No content";
}

static void play_wav(uint8_t* wav_buf, int read_len) {
    uint32_t data_offset = 44;
    uint32_t sample_rate = 24000;
    if (read_len >= 44 && memcmp(wav_buf, "RIFF", 4) == 0) {
        sample_rate = *(uint32_t*)(wav_buf + 24);
        uint32_t pos = 12;
        while (pos + 8 <= (uint32_t)read_len) {
            if (memcmp(wav_buf + pos, "data", 4) == 0) { data_offset = pos + 8; break; }
            uint32_t chunk_size = *(uint32_t*)(wav_buf + pos + 4);
            pos += 8 + ((chunk_size + 1) & ~1u);
        }
    }
    uint32_t speak_next_ms = millis() + 900;
    bool speak_phase = false;
    M5.Speaker.setVolume(tts_volume);
    M5.Speaker.playRaw((int16_t*)(wav_buf + data_offset), (read_len - data_offset) / 2, sample_rate, false, 1, 0);
    while (M5.Speaker.isPlaying()) {
        M5.update();
        delay(10);
        if (touchedZone(2)) { M5.Speaker.stop(); break; }
        if (servo_ready) {
            uint32_t now = millis();
            if (now >= speak_next_ms) {
                speak_phase = !speak_phase;
                sc_servo.moveY(speak_phase ? srv_cy - 8 : srv_cy, 500, false);
                avatar.setMouthOpenRatio(speak_phase ? random(30, 65) / 100.0f : 0.05f);
                speak_next_ms = now + 900;
            }
        }
    }
    avatar.setMouthOpenRatio(0.0f);
    if (servo_ready) sc_servo.moveXY(srv_cx, srv_cy, 500);
}

bool call_tts_local(const String& text) {
    bpm_stop_audio();
    if (!ensure_wifi_connected()) return false;

    String base = "http://" + voicevox_host;
    String speaker_str = String(voicevox_speaker);

    // Step1: audio_query (synchronous POST)
    HTTPClient http1;
    WiFiClient c1;
    http1.begin(c1, base + "/audio_query?text=" + url_encode(text) + "&speaker=" + speaker_str);
    http1.setTimeout(10000);
    int st1 = http1.POST("");
    if (st1 != 200) { M5_LOGE("audio_query error: %d", st1); http1.end(); return false; }
    String query_json = http1.getString();
    http1.end();
    M5_LOGI("audio_query OK (%d bytes)", query_json.length());

    // Step2: synthesis (synchronous POST → WAV bytes)
    HTTPClient http2;
    WiFiClient c2;
    http2.begin(c2, base + "/synthesis?speaker=" + speaker_str);
    http2.setTimeout(15000);
    http2.addHeader("Content-Type", "application/json");
    int st2 = http2.POST(query_json);
    if (st2 != 200) { M5_LOGE("synthesis error: %d", st2); http2.end(); return false; }

    int wav_size = http2.getSize();
    if (wav_size <= 0) wav_size = 512 * 1024;
    uint8_t* wav_buf = (uint8_t*)heap_caps_malloc(wav_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!wav_buf) { M5_LOGE("malloc failed"); http2.end(); return false; }
    int read_len = http2.getStream().readBytes(wav_buf, wav_size);
    http2.end();
    M5_LOGI("synthesis WAV: %d bytes", read_len);

    play_wav(wav_buf, read_len);
    heap_caps_free(wav_buf);
    return true;
}

bool call_tts(const String& text) {
    bpm_stop_audio();
    if (!ensure_wifi_connected()) return false;
    if (!voicevox_host.isEmpty()) return call_tts_local(text);
    String api_key = system_config.getAPISetting()->tts;
    if (api_key.isEmpty()) { M5_LOGE("TTS API key not set"); return false; }

    // Step1: 合成リクエスト → wavDownloadUrl / audioStatusUrl を取得
    String synth_url = "https://api.tts.quest/v3/voicevox/synthesis?text="
                       + url_encode(text) + "&speaker=1&key=" + api_key;
    M5_LOGI("TTS synth: %s", synth_url.c_str());

    HTTPClient http1;
    WiFiClientSecure c1; c1.setInsecure();
    http1.begin(c1, synth_url);
    http1.setTimeout(15000);
    int st1 = http1.GET();
    if (st1 != 200) { M5_LOGE("TTS synth error: %d", st1); http1.end(); return false; }

    String json_resp = http1.getString();
    http1.end();
    M5_LOGI("TTS synth resp: %s", json_resp.c_str());

    JsonDocument jdoc;
    if (deserializeJson(jdoc, json_resp)) { M5_LOGE("TTS JSON parse error"); return false; }
    if (!jdoc["success"].as<bool>()) {
        M5_LOGE("TTS API success=false: %s", jdoc["errorMessage"].as<const char*>());
        return false;
    }
    String wav_url    = jdoc["wavDownloadUrl"].as<String>();
    String status_url = jdoc["audioStatusUrl"].as<String>();
    if (wav_url.isEmpty()) { M5_LOGE("No wavDownloadUrl"); return false; }
    M5_LOGI("WAV URL: %s", wav_url.c_str());

    // Step1.5: audioStatusUrl をポーリングして WAV 生成完了を待つ
    if (!status_url.isEmpty()) {
        M5_LOGI("Polling audio status...");
        bool audio_ready = false;
        for (int attempt = 0; attempt < 40; attempt++) {
            delay(500);
            HTTPClient hStat;
            WiFiClientSecure cStat; cStat.setInsecure();
            hStat.begin(cStat, status_url);
            hStat.setTimeout(5000);
            int stStat = hStat.GET();
            if (stStat == 200) {
                String statResp = hStat.getString();
                hStat.end();
                JsonDocument statDoc;
                if (!deserializeJson(statDoc, statResp) && statDoc["isAudioReady"].as<bool>()) {
                    M5_LOGI("Audio ready (attempt %d)", attempt + 1);
                    audio_ready = true;
                    break;
                }
            } else {
                hStat.end();
            }
        }
        if (!audio_ready) { M5_LOGE("Audio not ready after polling"); return false; }
    }

    // Step2: WAVダウンロード
    HTTPClient http2;
    WiFiClientSecure c2; c2.setInsecure();
    http2.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http2.begin(c2, wav_url);
    http2.setTimeout(15000);
    int st2 = http2.GET();
    if (st2 != 200) { M5_LOGE("WAV download error: %d", st2); http2.end(); return false; }

    int wav_size = http2.getSize();
    if (wav_size <= 0) wav_size = 512 * 1024; // 不明な場合は512KB確保
    uint8_t* wav_buf = (uint8_t*)heap_caps_malloc(wav_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!wav_buf) { M5_LOGE("malloc failed"); http2.end(); return false; }

    int read_len = http2.getStream().readBytes(wav_buf, wav_size);
    http2.end();
    M5_LOGI("WAV downloaded: %d bytes", read_len);

    // Step3: WAV ヘッダー解析 → playRaw で再生 (spk_task スタック節約)
    uint32_t data_offset = 44;
    uint32_t sample_rate = 24000;
    if (read_len >= 44 && memcmp(wav_buf, "RIFF", 4) == 0) {
        sample_rate = *(uint32_t*)(wav_buf + 24);
        uint32_t pos = 12;
        while (pos + 8 <= (uint32_t)read_len) {
            if (memcmp(wav_buf + pos, "data", 4) == 0) { data_offset = pos + 8; break; }
            uint32_t chunk_size = *(uint32_t*)(wav_buf + pos + 4);
            pos += 8 + ((chunk_size + 1) & ~1u);
        }
    }
    play_wav(wav_buf, read_len);
    heap_caps_free(wav_buf);
    return true;
}

String call_stt() {
    bpm_stop_audio();
    if (!ensure_wifi_connected()) return "";

    String api_key = system_config.getAPISetting()->stt;
    if (api_key.isEmpty()) { M5_LOGE("STT API key not set"); return ""; }

    int16_t* rec_buf = (int16_t*)heap_caps_malloc(MIC_BUF_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rec_buf) { M5_LOGE("rec_buf malloc failed"); return ""; }

    M5.Speaker.end();
    M5.Mic.begin();
    if (servo_ready) sc_servo.moveXY(srv_cx + 15, srv_cy - 5, 500);  // listening tilt
    led_set(LED_HEARING);
    M5_LOGI("Recording %d sec...", MIC_RECORD_SEC);
    M5.Mic.record(rec_buf, MIC_BUF_SAMPLES, MIC_SAMPLE_RATE, false);
    bool cancelled = false;
    for (int i = 0; i < MIC_RECORD_SEC * 100; i++) {
        delay(10);
        M5.update();
        if (touchedZone(2)) { cancelled = true; break; }
    }
    M5.Mic.end();
    M5.Speaker.begin();
    if (servo_ready) sc_servo.moveXY(srv_cx, srv_cy, 400);  // return to center
    led_set(LED_OFF);
    M5_LOGI("Recording done");
    if (cancelled) { heap_caps_free(rec_buf); show("Cancelled"); return ""; }
    if (!ensure_wifi_connected()) { heap_caps_free(rec_buf); return ""; }

    // Base64 エンコード
    size_t b64_len = ((MIC_BUF_BYTES + 2) / 3) * 4 + 1;
    uint8_t* b64_buf = (uint8_t*)heap_caps_malloc(b64_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!b64_buf) { heap_caps_free(rec_buf); M5_LOGE("b64 malloc failed"); return ""; }
    size_t out_len = base64_encode(b64_buf, (uint8_t*)rec_buf, MIC_BUF_BYTES);
    heap_caps_free(rec_buf);

    // JSON body 構築（大きいので PSRAM に確保）
    const char* prefix = "{\"config\":{\"encoding\":\"LINEAR16\",\"sampleRateHertz\":16000,\"languageCode\":\"ja-JP\"},\"audio\":{\"content\":\"";
    const char* suffix = "\"}}";
    size_t prefix_len = strlen(prefix);
    size_t suffix_len = strlen(suffix);
    size_t body_size  = prefix_len + out_len + suffix_len + 1;
    char* body_buf = (char*)heap_caps_malloc(body_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!body_buf) { heap_caps_free(b64_buf); M5_LOGE("body malloc failed"); return ""; }
    memcpy(body_buf, prefix, prefix_len);
    memcpy(body_buf + prefix_len, b64_buf, out_len);
    memcpy(body_buf + prefix_len + out_len, suffix, suffix_len + 1);
    heap_caps_free(b64_buf);

    // Google STT REST API 呼び出し
    String url = "https://speech.googleapis.com/v1/speech:recognize?key=" + api_key;
    HTTPClient http;
    WiFiClientSecure client; client.setInsecure();
    http.begin(client, url);
    http.setTimeout(30000);
    http.addHeader("Content-Type", "application/json");
    int status = http.POST((uint8_t*)body_buf, body_size - 1);
    heap_caps_free(body_buf);

    if (status != 200) { M5_LOGE("STT error: %d", status); http.end(); return ""; }
    String response = http.getString();
    http.end();
    M5_LOGI("STT resp: %s", response.c_str());

    JsonDocument resp;
    if (deserializeJson(resp, response)) { M5_LOGE("STT JSON parse error"); return ""; }
    const char* transcript = resp["results"][0]["alternatives"][0]["transcript"];
    return transcript ? String(transcript) : "";
}

void show(const String& text) {
    String oneline = text;
    oneline.replace("\n", " ");
    avatar.setSpeechText(oneline.c_str());
    Serial.println(text);
}

void handle_speak_server() {
    WiFiClient client = speak_server.available();
    if (!client) return;

    String req_line = client.readStringUntil('\n');
    bool is_post_speak = req_line.startsWith("POST /speak");

    int content_length = 0;
    while (client.connected()) {
        String line = client.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) break;
        String lower = line; lower.toLowerCase();
        if (lower.startsWith("content-length:"))
            content_length = line.substring(line.indexOf(':') + 1).toInt();
    }

    if (!is_post_speak) {
        client.print("HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        client.stop();
        return;
    }

    if (busy) {
        client.print("HTTP/1.1 503 Service Unavailable\r\nContent-Type: application/json\r\nContent-Length: 13\r\nConnection: close\r\n\r\n{\"busy\":true}");
        client.stop();
        return;
    }

    String body = "";
    uint32_t deadline = millis() + 2000;
    while ((int)body.length() < content_length && millis() < deadline) {
        if (client.available()) body += (char)client.read();
    }

    JsonDocument doc;
    String speak_text = "";
    if (!deserializeJson(doc, body)) speak_text = doc["text"] | "";
    if (speak_text.isEmpty()) {
        client.print("HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        client.stop();
        return;
    }

    client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 10\r\nConnection: close\r\n\r\n{\"ok\":true}");
    client.flush();
    client.stop();

    busy = true;
    avatar.setExpression(Expression::Happy);
    led_set(LED_SPEAKING);
    show("[Speaking...]");
    if (call_tts(speak_text)) {
        avatar.setExpression(Expression::Neutral);
        led_set(LED_OFF);
        show("");
    } else {
        avatar.setExpression(Expression::Sad);
        led_set(LED_ERROR);
        show("TTS failed");
    }
    busy = false;
}

// ---- Settings UI ----

static void update_yaml_int_value(String& yaml, const char* key, int value) {
    String search = String(key) + ":";
    int pos = yaml.indexOf(search);
    if (pos < 0) { yaml += "\n" + search + " " + String(value); return; }
    int start = pos + search.length();
    while (start < (int)yaml.length() && yaml[start] == ' ') start++;
    int end = start;
    while (end < (int)yaml.length() && (isDigit(yaml[end]) || yaml[end] == '-')) end++;
    yaml = yaml.substring(0, start) + String(value) + yaml.substring(end);
}

static void save_settings() {
    File f = SPIFFS.open("/yaml/SC_SecConfig.yaml", "r");
    if (!f) { M5_LOGE("save_settings: open failed"); return; }
    String yaml = f.readString();
    f.close();
    update_yaml_int_value(yaml, "tts_volume", setting_volume);
    update_yaml_int_value(yaml, "brightness", setting_brightness);
    File fw = SPIFFS.open("/yaml/SC_SecConfig.yaml", "w");
    if (!fw) { M5_LOGE("save_settings: write failed"); return; }
    fw.print(yaml);
    fw.close();
    M5_LOGI("Settings saved: vol=%d bright=%d", setting_volume, setting_brightness);
}

static void draw_slider_row(int y, const char* label, int val) {
    const int bar_x = 8, bar_w = 200, bar_h = 12;
    const int bar_y = y + 22;
    const int btn_y = y + 16, btn_h = 26;
    int filled = val * bar_w / 255;

    M5.Display.fillRect(0, y, 320, 48, TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(8, y + 2);
    M5.Display.print(label);
    char buf[8];
    snprintf(buf, sizeof(buf), "%3d%%", val * 100 / 255);
    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Display.setCursor(168, y + 2);
    M5.Display.print(buf);

    M5.Display.fillRect(bar_x, bar_y, filled, bar_h, TFT_CYAN);
    M5.Display.fillRect(bar_x + filled, bar_y, bar_w - filled, bar_h, TFT_DARKGREY);
    M5.Display.drawRect(bar_x, bar_y, bar_w, bar_h, TFT_WHITE);

    M5.Display.fillRoundRect(218, btn_y, 38, btn_h, 4, TFT_NAVY);
    M5.Display.setTextColor(TFT_WHITE, TFT_NAVY);
    M5.Display.setCursor(229, btn_y + 6);
    M5.Display.print("-");

    M5.Display.fillRoundRect(264, btn_y, 38, btn_h, 4, TFT_NAVY);
    M5.Display.setTextColor(TFT_WHITE, TFT_NAVY);
    M5.Display.setCursor(273, btn_y + 6);
    M5.Display.print("+");
}

static void draw_settings_ui() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Display.setCursor(8, 8);
    M5.Display.print("Settings");
    M5.Display.fillRoundRect(280, 4, 36, 28, 4, TFT_DARKGREY);
    M5.Display.setTextColor(TFT_WHITE, TFT_DARKGREY);
    M5.Display.setCursor(288, 10);
    M5.Display.print("X");
    M5.Display.drawLine(0, 36, 320, 36, TFT_DARKGREY);
    draw_slider_row(40, "Volume  ", setting_volume);
    M5.Display.drawLine(0, 88, 320, 88, TFT_DARKGREY);
    draw_slider_row(92, "Bright  ", setting_brightness);
    M5.Display.drawLine(0, 140, 320, 140, TFT_DARKGREY);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    M5.Display.setCursor(8, 150);
    M5.Display.print("BPM Mode: hold right zone Detect/Play/Normal");
    M5.Display.fillRoundRect(10, 192, 135, 36, 6, TFT_DARKGREY);
    M5.Display.setTextColor(TFT_WHITE, TFT_DARKGREY);
    M5.Display.setCursor(34, 203);
    M5.Display.print("Cancel");
    M5.Display.fillRoundRect(165, 192, 145, 36, 6, TFT_DARKGREEN);
    M5.Display.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    M5.Display.setCursor(206, 203);
    M5.Display.print("Save");
}

static void settings_enter() {
    if (busy) return;
    bpm_stop_audio();
    app_mode = MODE_SETTINGS;
    setting_volume = tts_volume;
    setting_brightness = brightness_val;
    avatar.suspend();
    draw_settings_ui();
}

static void settings_exit(bool save) {
    if (save) {
        tts_volume = setting_volume;
        brightness_val = setting_brightness;
        M5.Display.setBrightness(brightness_val);
        save_settings();
    }
    app_mode = MODE_NORMAL;
    M5.Display.fillScreen(TFT_BLACK);
    avatar.resume();
}

static void handle_settings_touch() {
    auto touch = M5.Touch.getDetail();
    if (!touch.wasPressed()) return;
    int tx = touch.x, ty = touch.y;

    // [X] or Cancel
    if (tx >= 280 && ty >= 4 && ty <= 32) { settings_exit(false); return; }
    if (tx >= 10 && tx <= 145 && ty >= 192 && ty <= 228) { settings_exit(false); return; }
    // Save
    if (tx >= 165 && tx <= 310 && ty >= 192 && ty <= 228) { settings_exit(true); return; }

    // Volume [-][+]
    if (ty >= 56 && ty <= 82) {
        if (tx >= 218 && tx <= 256) {
            setting_volume = max(0, setting_volume - 20);
            draw_slider_row(40, "Volume  ", setting_volume);
        } else if (tx >= 264 && tx <= 302) {
            setting_volume = min(255, setting_volume + 20);
            draw_slider_row(40, "Volume  ", setting_volume);
        }
    }
    // Brightness [-][+]
    if (ty >= 108 && ty <= 134) {
        if (tx >= 218 && tx <= 256) {
            setting_brightness = max(20, setting_brightness - 20);
            M5.Display.setBrightness(setting_brightness);
            draw_slider_row(92, "Bright  ", setting_brightness);
        } else if (tx >= 264 && tx <= 302) {
            setting_brightness = min(255, setting_brightness + 20);
            M5.Display.setBrightness(setting_brightness);
            draw_slider_row(92, "Bright  ", setting_brightness);
        }
    }
}

static void bpm_reset_state() {
    bpm_noise_floor = 0.0f;
    bpm_calibration_frames = 0;
    bpm_env = 0.0f;
    bpm_avg_env = 0.0f;
    bpm_last_peak_ms = 0;
    bpm_peak_interval_count = 0;
    bpm_peak_interval_index = 0;
    bpm_next_log_ms = 0;
    bpm_play_next_step_ms = 0;
    bpm_play_hold_until_ms = 0;
    bpm_play_swing_right = true;
    for (size_t i = 0; i < (sizeof(bpm_peak_intervals) / sizeof(bpm_peak_intervals[0])); ++i) {
        bpm_peak_intervals[i] = 0;
    }
    avatar.setMouthOpenRatio(0.0f);
    if (servo_ready) sc_servo.moveXY(srv_cx, srv_cy, 250);
}

static void bpm_store_peak_interval(uint32_t interval_ms) {
    if (interval_ms < 250 || interval_ms > 1200) return;
    bpm_peak_intervals[bpm_peak_interval_index] = interval_ms;
    bpm_peak_interval_index = (bpm_peak_interval_index + 1) % (sizeof(bpm_peak_intervals) / sizeof(bpm_peak_intervals[0]));
    if (bpm_peak_interval_count < (sizeof(bpm_peak_intervals) / sizeof(bpm_peak_intervals[0]))) {
        bpm_peak_interval_count++;
    }
}

static uint16_t bpm_estimate_from_intervals() {
    if (bpm_peak_interval_count < 3) return 0;
    uint32_t intervals[8] = {0};
    for (uint8_t i = 0; i < bpm_peak_interval_count; ++i) {
        intervals[i] = bpm_peak_intervals[i];
    }
    for (uint8_t i = 0; i < bpm_peak_interval_count; ++i) {
        for (uint8_t j = i + 1; j < bpm_peak_interval_count; ++j) {
            if (intervals[j] < intervals[i]) {
                uint32_t tmp = intervals[i];
                intervals[i] = intervals[j];
                intervals[j] = tmp;
            }
        }
    }

    uint8_t start = bpm_peak_interval_count > 4 ? 1 : 0;
    uint8_t end = bpm_peak_interval_count > 4 ? bpm_peak_interval_count - 1 : bpm_peak_interval_count;
    uint32_t sum = 0;
    uint8_t used = 0;
    for (uint8_t i = start; i < end; ++i) {
        sum += intervals[i];
        used++;
    }
    if (!used || !sum) return 0;

    uint32_t avg_interval = sum / used;
    if (!avg_interval) return 0;
    uint16_t bpm = static_cast<uint16_t>(60000UL / avg_interval);
    while (bpm < 60) bpm *= 2;
    while (bpm > 180) bpm /= 2;
    return bpm;
}

static void bpm_exit_modes() {
    bpm_stop_audio();
    bpm_reset_state();
    app_mode = MODE_NORMAL;
    servo_idle_enabled = true;
    servo_idle_next_ms = millis() + 2000;
    bpm_mode_started_ms = 0;
    bpm_toggle_cooldown_until_ms = millis() + 800;
    avatar.setExpression(Expression::Neutral);
    show("");
}

static void bpm_enter_detect_mode() {
    bpm_stop_audio();
    detected_bpm = 0;
    bpm_reset_state();
    app_mode = MODE_BPM_DETECT;
    servo_idle_enabled = false;
    bpm_mode_started_ms = millis();
    bpm_toggle_cooldown_until_ms = bpm_mode_started_ms + 800;
    avatar.setExpression(Expression::Neutral);
    show("Detecting BPM...");
}

static void bpm_enter_play_mode() {
    bpm_stop_audio();
    bpm_reset_state();
    app_mode = MODE_BPM_PLAY;
    servo_idle_enabled = false;
    bpm_mode_started_ms = millis();
    bpm_toggle_cooldown_until_ms = bpm_mode_started_ms + 800;
    if (detected_bpm >= 60 && detected_bpm <= 180) play_bpm = detected_bpm;
    if (play_bpm < 60 || play_bpm > 180) play_bpm = 120;
    avatar.setExpression(Expression::Happy);
    show("Play " + String(play_bpm) + " BPM");
}

static void bpm_detect_tick() {
    uint32_t now = millis();
    if (!servo_ready) return;

    if (!bpm_audio_active) {
        bpm_start_audio();
        bpm_next_log_ms = now;
        return;
    }
    if (!M5.Mic.isEnabled()) return;
    if (!M5.Mic.record(bpm_buf, BPM_RECORD_LENGTH, BPM_SAMPLE_RATE, false)) return;

    int32_t sample_sum = 0;
    for (size_t i = 0; i < BPM_RECORD_LENGTH; ++i) {
        sample_sum += bpm_buf[i];
    }
    int32_t dc_offset = sample_sum / static_cast<int32_t>(BPM_RECORD_LENGTH);

    uint32_t accum = 0;
    int16_t peak = 0;
    for (size_t i = 0; i < BPM_RECORD_LENGTH; ++i) {
        int32_t centered = static_cast<int32_t>(bpm_buf[i]) - dc_offset;
        int16_t amp = abs(centered);
        accum += amp;
        if (amp > peak) peak = amp;
    }

    float avg = static_cast<float>(accum) / BPM_RECORD_LENGTH;
    float raw_level = avg * 0.85f + peak * 0.15f;
    if (bpm_calibration_frames < 24) {
        bpm_noise_floor = (bpm_calibration_frames == 0) ? raw_level : (bpm_noise_floor * 0.82f + raw_level * 0.18f);
        bpm_calibration_frames++;
        bpm_env = 0.0f;
        bpm_avg_env = 0.0f;
        if (servo_ready) sc_servo.moveXY(srv_cx, srv_cy, 220);
        avatar.setMouthOpenRatio(0.0f);
        return;
    }

    if (raw_level < bpm_noise_floor + 80.0f) {
        if (raw_level > bpm_noise_floor) bpm_noise_floor = bpm_noise_floor * 0.97f + raw_level * 0.03f;
        else bpm_noise_floor = bpm_noise_floor * 0.90f + raw_level * 0.10f;
    }

    float active_level = raw_level - bpm_noise_floor;
    bpm_env = bpm_env * 0.55f + active_level * 0.45f;
    if (bpm_env < 0.0f) bpm_env = 0.0f;
    bpm_avg_env = bpm_avg_env * 0.92f + bpm_env * 0.08f;

    bool beat_candidate = peak >= 170
                       && bpm_env > (bpm_avg_env * 1.45f + 18.0f)
                       && (bpm_last_peak_ms == 0 || (now - bpm_last_peak_ms) >= 280);
    if (beat_candidate) {
        if (bpm_last_peak_ms != 0) {
            bpm_store_peak_interval(now - bpm_last_peak_ms);
            uint16_t estimated = bpm_estimate_from_intervals();
            if (estimated) detected_bpm = estimated;
        }
        bpm_last_peak_ms = now;
    }

    if (time_reached(now, bpm_next_log_ms)) {
        Serial.printf("BPMDetect ms=%lu avg=%.1f peak=%d raw=%.1f floor=%.1f active=%.1f env=%.1f avgEnv=%.1f bpm=%u calib=%u\n",
                      bpm_mode_started_ms ? (unsigned long)(now - bpm_mode_started_ms) : 0UL,
                      avg, peak, raw_level, bpm_noise_floor, active_level, bpm_env, bpm_avg_env,
                      detected_bpm, bpm_calibration_frames);
        bpm_next_log_ms = now + 1000;
        if (detected_bpm) show("Detecting... " + String(detected_bpm) + " BPM");
        else show("Detecting BPM...");
    }
}

static void bpm_play_tick() {
    uint32_t now = millis();
    if (bpm_audio_active) bpm_stop_audio();
    if (!servo_ready) return;

    const int play_center_y = srv_cy - 10;
    uint16_t bpm = play_bpm;
    if (bpm < 60 || bpm > 180) bpm = 120;
    uint32_t step_interval = 60000UL / bpm;
    if (step_interval < 320) step_interval = 320;
    if (step_interval > 1200) step_interval = 1200;

    if (!bpm_play_next_step_ms) bpm_play_next_step_ms = now;
    if (time_reached(now, bpm_play_next_step_ms)) {
        bpm_play_swing_right = !bpm_play_swing_right;
        int swing = 12;
        uint32_t move_ms = step_interval > 120 ? (step_interval - 120) : (step_interval - 40);
        sc_servo.moveXY(srv_cx + (bpm_play_swing_right ? swing : -swing), play_center_y, move_ms);
        avatar.setMouthOpenRatio(0.10f);
        bpm_play_next_step_ms = now + step_interval;
        bpm_play_hold_until_ms = now + (step_interval * 3 / 5);
    } else if (bpm_play_hold_until_ms && time_reached(now, bpm_play_hold_until_ms)) {
        sc_servo.moveXY(srv_cx, play_center_y, 120);
        avatar.setMouthOpenRatio(0.0f);
        bpm_play_hold_until_ms = 0;
    }

    if (time_reached(now, bpm_next_log_ms)) {
        Serial.printf("BPMPlay ms=%lu bpm=%u step=%lu right=%u\n",
                      bpm_mode_started_ms ? (unsigned long)(now - bpm_mode_started_ms) : 0UL,
                      bpm, (unsigned long)step_interval, bpm_play_swing_right ? 1 : 0);
        bpm_next_log_ms = now + 1000;
    }
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    randomSeed(esp_random());
    head_touch_sensor.begin();
    auto spk_cfg = M5.Speaker.config();
    spk_cfg.dma_buf_len = 1024;  // spk_task stack = 1280 + dma_buf_len*4 = 5376 bytes
    M5.Speaker.config(spk_cfg);
    M5.Speaker.begin();

    if (!SPIFFS.begin(true)) { M5.Display.println("SPIFFS ERROR"); return; }

    system_config.loadConfig(SPIFFS, "/yaml/SC_BasicConfig.yaml");
    servo_begin();
    load_hermes_config(SPIFFS);
    M5.Display.setBrightness(brightness_val);  // set before LED task starts
    led_init();  // must be after servo_begin() — needs ioexpander initialized

    avatar.init();
    avatar.setSpeechFont(&fonts::efontJA_16);

    show("Connecting WiFi...");
    wifi_s* wifi = system_config.getWiFiSetting();
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(wifi->ssid.c_str(), wifi->password.c_str());
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) { delay(500); retry++; }

    if (WiFi.status() == WL_CONNECTED) {
        speak_server.begin();
        M5_LOGI("WiFi: %s", WiFi.localIP().toString().c_str());
        show("IP: " + WiFi.localIP().toString());
        delay(3000);
        show("M:Voice R:Servo TR:Settings");
    } else {
        show("WiFi FAILED");
    }
}

bool touchedZone(int zone) {
    auto touch = M5.Touch.getDetail();
    if (!touch.wasPressed() || touch.y < 192) return false;
    return (touch.x / (320 / 3)) == zone;
}

bool touchHoldZone(int zone) {
    auto touch = M5.Touch.getDetail();
    if (!touch.wasHold() || touch.y < 192) return false;
    return (touch.x / (320 / 3)) == zone;
}

bool touchClickZone(int zone) {
    auto touch = M5.Touch.getDetail();
    if (!touch.wasClicked() || touch.y < 192) return false;
    return (touch.x / (320 / 3)) == zone;
}

void loop() {
    M5.update();

    if (M5.BtnPWR.wasHold()) {
        bpm_stop_audio();
        show("Power off");
        delay(50);
        M5.Power.powerOff();
        return;
    }

    if (app_mode == MODE_SETTINGS) {
        handle_settings_touch();
        delay(10);
        return;
    }

    if (app_mode == MODE_BPM_DETECT) {
        led_idle_log_tick();
        if (error_clear_at_ms && time_reached(millis(), error_clear_at_ms)) {
            clear_error_state();
            show(detected_bpm ? "Detecting... " + String(detected_bpm) + " BPM" : "Detecting BPM...");
        }
        if (time_reached(millis(), bpm_toggle_cooldown_until_ms) && touchHoldZone(2)) {
            bpm_enter_play_mode();
            delay(10);
            return;
        }
        bpm_detect_tick();
        delay(10);
        return;
    }

    if (app_mode == MODE_BPM_PLAY) {
        led_idle_log_tick();
        if (error_clear_at_ms && time_reached(millis(), error_clear_at_ms)) {
            clear_error_state();
            show("Play " + String(play_bpm) + " BPM");
        }
        if (time_reached(millis(), bpm_toggle_cooldown_until_ms) && touchHoldZone(2)) {
            bpm_exit_modes();
            delay(10);
            return;
        }
        bpm_play_tick();
        delay(10);
        return;
    }

    led_idle_log_tick();
    if (error_clear_at_ms && time_reached(millis(), error_clear_at_ms)) {
        clear_error_state();
        show("");
    }
    handle_speak_server();
    servo_idle_tick();
    uint32_t now = millis();
    if (!led_effect_active && time_reached(now, head_touch_next_poll_ms)) {
        head_touch_next_poll_ms = now + 50;
        if (head_pat_detected()) head_pat_reaction();
    }

    // 設定トリガー: 画面右上コーナー（x>260, y<50）タップ
    if (!busy) {
        auto t = M5.Touch.getDetail();
        if (t.wasPressed() && t.x > 260 && t.y < 50) {
            settings_enter();
            return;
        }
    }

    // 左タッチ: テスト発話
    if (!busy && time_reached(millis(), bpm_toggle_cooldown_until_ms) && touchHoldZone(2)) {
        clear_error_state();
        bpm_enter_detect_mode();
        return;
    }

    if (touchedZone(0) && !busy) {
        clear_error_state();
        busy = true;
        avatar.setExpression(Expression::Doubt);
        led_set(LED_THINKING);
        show("Thinking...");
        String reply = call_hermes("こんにちは！一言で自己紹介してください。");
        if (reply.startsWith("Error:") || reply.startsWith("Parse error") || reply.startsWith("No content")) {
            show_error_state(reply);
            busy = false;
            return;
        }
        avatar.setExpression(Expression::Happy);
        led_set(LED_SPEAKING);
        Serial.println("Reply: " + reply);
        show("[Speaking...]");
        if (call_tts(reply)) {
            avatar.setExpression(Expression::Neutral);
            led_set(LED_OFF);
            show("");
        } else {
            show_error_state("TTS failed");
        }
        busy = false;
    }

    // 右タッチ: アイドルサーボ停止/再開トグル
    if (touchedZone(2)) {
        clear_error_state();
        show("");
        servo_idle_enabled = !servo_idle_enabled;
        led_idle_kick_once();
        if (!servo_idle_enabled && servo_ready) {
            sc_servo.moveXY(srv_cx, srv_cy, 500);
            show("Servo: OFF");
        } else {
            servo_idle_next_ms = millis() + 2000;
            show("Servo: ON");
        }
    }

    // 中央タッチ: Push-to-Talk (STT → Hermes → TTS)
    if (touchedZone(1) && !busy) {
        clear_error_state();
        busy = true;
        servo_idle_enabled = true;  // 話しかけたらサーボ再開
        servo_idle_next_ms = millis() + 2000;
        avatar.setExpression(Expression::Happy);
        show("Speak now!");
        String text = call_stt();
        if (text.isEmpty()) {
            show_error_state("STT failed");
            busy = false;
            return;
        }
        avatar.setExpression(Expression::Doubt);
        led_set(LED_THINKING);
        show("Thinking...");
        String reply = call_hermes(text);
        if (reply.startsWith("Error:") || reply.startsWith("Parse error") || reply.startsWith("No content")) {
            show_error_state(reply);
            busy = false;
            return;
        }
        avatar.setExpression(Expression::Happy);
        led_set(LED_SPEAKING);
        Serial.println("Reply: " + reply);
        show("[Speaking...]");
        if (call_tts(reply)) {
            avatar.setExpression(Expression::Neutral);
            led_set(LED_OFF);
            show("");
        } else {
            show_error_state("TTS failed");
        }
        busy = false;
    }

    delay(10);
}
