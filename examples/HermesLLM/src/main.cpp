#include <Arduino.h>
#include <M5Unified.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include <mbedtls/sha1.h>
#include <math.h>
#include <array>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Stackchan_system_config.h>
#include <Stackchan_servo.h>
#include "Si12T.h"
#include "MavlinkReceiver.h"
#include "FlightStateMonitor.h"
#include "StackChanSpeech.h"
#include <Avatar.h>
using namespace m5avatar;
struct WifiCredential;
bool touchedZone(int zone);
void show(const String& text);
void handle_speak_server();
static bool speak_mavlink_notification(const String& text);
static void mavlink_notification_tick();
static void mavlink_receive_tick(bool allow_speech);
static void load_wifi_credentials_from_yaml(const String& normalized_yaml);
static void load_llm_models_from_yaml(const String& normalized_yaml);
static void replace_wifi_yaml_sections(String& yaml, WifiCredential entries[], size_t entry_count);
static void led_force_clear();
static void chat_add(bool is_user, const String& text);
static void speech_scroll_start(const String& text);
static void speech_scroll_tick();
static void speech_scroll_run_to_end(uint8_t repeat_count = 3);
static void speech_scroll_stop();
static bool play_wav_file(const char* path);
static bool call_tts_cache_wav(const String& text, const char* cache_path);
static String mavlink_notification_audio_path(const String& text);
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
static volatile bool servo_idle_enabled = false;
static bool servo_on_boot = false;

#define PERIODIC_MAX 5
struct PeriodicEntry {
    bool     enabled     = false;
    bool     is_llm      = false;
    String   message;
    uint32_t interval_ms = 1800000UL;
    uint32_t next_fire_ms = 0;
};
static PeriodicEntry periodic_entries[PERIODIC_MAX];
static bool     periodic_mode_enabled   = false;
static uint32_t periodic_hold_cooldown_ms = 0;
static bool btn_llm_tap       = false;
static bool btn_periodic_hold = true;
static bool btn_stt_tap       = true;
static bool btn_servo_tap     = true;
static bool btn_bpm_hold      = true;
static int16_t srv_cx = 150, srv_cy = 90;
static uint32_t servo_idle_next_ms = 0;
static uint32_t head_pat_cooldown_until_ms = 0;
static uint32_t head_touch_next_poll_ms = 0;
static int16_t servo_web_x = 150;
static int16_t servo_web_y = 90;
static WiFiClient servo_ws_client;
static bool servo_ws_active = false;
static uint32_t servo_ws_last_ms = 0;
static volatile bool led_effect_active = false;
static SemaphoreHandle_t io_mutex = nullptr;

// --- Settings ---
enum AppMode { MODE_NORMAL, MODE_SETTINGS, MODE_BPM_DETECT, MODE_BPM_PLAY };
static AppMode app_mode = MODE_NORMAL;
static int setting_volume     = 80;
static int setting_brightness = 200;
static int setting_wifi_index = 0;
static bool setting_attitude_servo_enabled = false;
static bool setting_rc_follow_enabled = false;
static bool setting_attitude_yaw_source_roll = false;
static int setting_attitude_scale = 50;
static uint8_t setting_page = 0;   // 0=Main, 1=Flight, 2=LLM
// Selectable LLM models — loaded from YAML `llm_models:` (falls back to built-in defaults).
// All entries share the same hermes: endpoint/api_key; only `model` changes.
struct LlmModelPreset { String label; String slug; };
static constexpr size_t LLM_MODEL_MAX = 8;
static LlmModelPreset llm_models[LLM_MODEL_MAX];
static size_t llm_model_count = 0;
static int setting_model_index = -1;   // -1 = current model is not in the list (custom)
static int brightness_val     = 200;
static bool mavlink_attitude_servo_enabled = false;
static bool mavlink_rc_follow_enabled = false;
static bool mavlink_attitude_yaw_source_roll = false;
static int mavlink_attitude_scale = 50;
static uint32_t mavlink_attitude_next_servo_ms = 0;
static bool mavlink_attitude_filter_ready = false;
static bool mavlink_attitude_target_ready = false;
static bool mavlink_attitude_latest_ready = false;
static float mavlink_attitude_latest_yaw = 0.0f;
static float mavlink_attitude_latest_roll = 0.0f;
static float mavlink_attitude_latest_pitch = 0.0f;
static float mavlink_attitude_zero_yaw_source = 0.0f;
static float mavlink_attitude_zero_pitch = 0.0f;
static float mavlink_attitude_target_x = 0.0f;
static float mavlink_attitude_target_y = 0.0f;
static float mavlink_attitude_filtered_x = 0.0f;
static float mavlink_attitude_filtered_y = 0.0f;
static float mavlink_attitude_last_x = 0.0f;
static float mavlink_attitude_last_y = 0.0f;
static uint32_t mavlink_attitude_servo_commands = 0;
static uint32_t mavlink_attitude_servo_commands_last = 0;
static uint32_t mavlink_attitude_servo_skips = 0;
static uint32_t mavlink_attitude_servo_skips_last = 0;
static uint32_t mavlink_attitude_servo_next_log_ms = 0;
static uint32_t normal_touch_guard_until_ms = 0;
static bool normal_touch_guard_wait_release = false;

// ── Head tracking via ESP-NOW ─────────────────────────────────────────────────
typedef struct { float pan; float tilt; } HeadTrackPacket;

// Shared between WiFi task (callback) and main loop — protected by portMUX spinlock.
// portMUX emits MEMW barriers on ESP32, guaranteeing cache-coherent visibility
// between Core 0 (WiFi task) and Core 1 (Arduino loop).
static portMUX_TYPE   head_track_mux          = portMUX_INITIALIZER_UNLOCKED;
static bool           head_track_recv_ready   = false;
static float          head_track_recv_x       = 0.0f;
static float          head_track_recv_y       = 0.0f;
static uint32_t       head_track_last_recv_ms = 0;  // millis of most recent packet
// Main-loop-only state (no locking needed):
static float          head_track_filtered_x  = 0.0f;
static float          head_track_filtered_y  = 0.0f;
static float          head_track_last_x      = 0.0f;
static float          head_track_last_y      = 0.0f;
static bool           head_track_filter_ready = false;
static uint32_t       head_track_next_servo_ms = 0;
static constexpr size_t WIFI_MAX = 3;
struct WifiCredential {
    String ssid;
    String password;
};
static WifiCredential wifi_credentials[WIFI_MAX];
static size_t wifi_credential_count = 0;

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
static uint32_t bpm_play_last_move_ms = 0;
static bool http_beat_pending = false;
static bool http_beat_return_pending = false;
static int http_beat_target_x = 0;
static int http_beat_target_y = 0;
static uint32_t http_beat_return_at_ms = 0;
static bool http_beat_motion_vertical = false;
static m5::touch_detail_t current_touch_detail;

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

enum HeadGesture { GESTURE_NONE, GESTURE_PAT, GESTURE_DOUBLE_TAP, GESTURE_TRIPLE_TAP };

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
        prev_any_touched_ = false;
        tap_pending_ = 0;
        tap_count_result_ = 0;
        tap_finalized_ = false;
        swipe_seen_in_seq_ = false;
        last_release_ms_ = 0;
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

        // Tap counting — must run before the early return to capture falling edges
        bool falling = prev_any_touched_ && !any_touched;
        if (falling) {
            if (!swipe_seen_in_seq_) {
                tap_pending_++;
                last_release_ms_ = now;
            } else {
                swipe_seen_in_seq_ = false;
            }
        }
        if (tap_pending_ > 0 && !any_touched &&
                static_cast<int32_t>(now - last_release_ms_) >= TAP_QUIET_MS) {
            tap_count_result_ = tap_pending_;
            tap_finalized_ = true;
            tap_pending_ = 0;
            Serial.printf("Head tap count: %u\n", tap_count_result_);
        }
        prev_any_touched_ = any_touched;

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
                swipe_seen_in_seq_ = true;
                tap_pending_ = 0;
                Serial.println("Head pat swipe forward");
            } else if (t1_2 > min_swipe_interval && t0_1 > min_swipe_interval &&
                       t1_2 < max_swipe_interval && t0_1 < max_swipe_interval) {
                swipe_backward = true;
                in_gesture = true;
                swipe_seen_in_seq_ = true;
                tap_pending_ = 0;
                Serial.println("Head pat swipe backward");
            }
        }
    }

    bool was_swiped_forward() const { return swipe_forward; }
    bool was_swiped_backward() const { return swipe_backward; }
    bool tap_ready() const { return tap_finalized_; }
    uint8_t consume_tap_count() { tap_finalized_ = false; return tap_count_result_; }

private:
    Si12T sensor = Si12T(SI12T_Type_High, SI12T_Sensitivity_Level_4);
    std::array<uint8_t, 3> intensities = {0, 0, 0};
    std::array<uint8_t, 3> last_logged_intensities = {255, 255, 255};
    uint32_t touch_start_time[3] = {0, 0, 0};
    bool touched_flag[3] = {false, false, false};
    bool in_gesture = false;
    bool swipe_forward = false;
    bool swipe_backward = false;
    bool prev_any_touched_ = false;
    uint8_t tap_pending_ = 0;
    uint8_t tap_count_result_ = 0;
    bool tap_finalized_ = false;
    bool swipe_seen_in_seq_ = false;
    uint32_t last_release_ms_ = 0;
    static constexpr uint32_t TAP_QUIET_MS = 400;
};

static HeadTouchSensor head_touch_sensor;

// --- HTTP Server ---
WiFiServer speak_server(80);

// --- UDP Beat Receiver ---
static WiFiUDP udp_beat;
static constexpr uint16_t UDP_BEAT_PORT = 8888;
static volatile bool busy = false;
static volatile bool web_tts_pending = false;
static String web_tts_text = "";
static uint32_t error_clear_at_ms = 0;
static MavlinkReceiver mavlink_receiver;
static FlightStateMonitor flight_state_monitor;
static StackChanSpeech stackchan_speech;
static String mavlink_pending_notification = "";
static bool rc_ch5_greeting_active = false;
static bool rc_ch6_school_greeting_active = false;
static int8_t rc_ch9_posture_state = -1;

// --- Conversation history ---
static constexpr int CHAT_HISTORY_MAX = 8;
struct ChatMessage {
    bool is_user = false;
    String text;
};
static ChatMessage chat_messages[CHAT_HISTORY_MAX];
static size_t chat_message_count = 0;
static bool avatar_ready = false;
static bool speech_scroll_active = false;
static String speech_scroll_text = "";
static uint32_t speech_scroll_started_ms = 0;
static uint32_t speech_scroll_last_ms = 0;
static uint32_t speech_scroll_duration_ms = 0;
static constexpr int SPEECH_SCROLL_VISIBLE_CHARS = 12;
static constexpr uint32_t SPEECH_SCROLL_STEP_MS = 130;

// --- LED ---
#define LED_NUM 12
static m5::PY32IOExpander_Class* led_io = nullptr;
enum LedMode { LED_OFF, LED_HEARING, LED_THINKING, LED_SPEAKING, LED_ERROR, LED_HAPPY, LED_MANUAL };
static volatile LedMode led_mode = LED_OFF;

enum ManualPattern : uint8_t { PAT_SOLID, PAT_BLINK, PAT_BREATH, PAT_CHASE, PAT_RAINBOW, PAT_WAVE, PAT_SPLIT, PAT_ALTERNATE, PAT_SPLIT_BLINK, PAT_SPLIT_BREATH, PAT_SPLIT_WAVE, PAT_SPLIT_CHASE };
static volatile ManualPattern manual_pattern = PAT_SOLID;
static volatile uint8_t manual_r = 0, manual_g = 0, manual_b = 0;
static volatile uint8_t manual_r2 = 255, manual_g2 = 255, manual_b2 = 255;
static volatile uint8_t manual_speed = 5;  // 1-10
static volatile uint16_t manual_on_ms = 300, manual_off_ms = 300;

static constexpr uint32_t LED_IDLE_INTERVAL_MS = 180000;
static constexpr uint32_t LED_IDLE_DURATION_MS = 20000;
static constexpr uint8_t LED_IDLE_BRIGHTNESS_MIN = 6;
static constexpr uint8_t LED_IDLE_BRIGHTNESS_MAX = 72;
static constexpr uint32_t HEAD_PAT_REACTION_MS = 1800;
static constexpr uint32_t HEAD_PAT_COOLDOWN_MS = 2000;

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

static uint16_t clamp_bpm_value(int bpm) {
    if (bpm < 60) return 60;
    if (bpm > 180) return 180;
    return static_cast<uint16_t>(bpm);
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
static volatile bool led_beat_requested = false;
static volatile uint16_t led_beat_step_index = 0;
static volatile bool led_beat_accent = false;
static volatile bool led_beat_vertical = false;

static constexpr uint32_t LED_BEAT_DURATION_MS = 180;

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

static LedColor beat_theme_color(uint8_t bar_index, bool vertical) {
    static constexpr LedColor lr_palette[] = {
        {0, 180, 120},
        {0, 140, 200},
        {90, 200, 90},
        {120, 120, 220},
    };
    static constexpr LedColor ud_palette[] = {
        {220, 140, 0},
        {180, 90, 180},
        {200, 120, 40},
        {140, 80, 220},
    };
    uint8_t idx = bar_index % 4;
    return vertical ? ud_palette[idx] : lr_palette[idx];
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
    bool    beat_active = false;
    uint32_t idle_next_ms = millis() + LED_IDLE_INTERVAL_MS;
    uint32_t idle_started_ms = 0;
    uint32_t idle_end_ms = 0;
    uint32_t beat_started_ms = 0;
    uint32_t beat_end_ms = 0;
    LedColor idle_color = {0, 0, 0};
    LedColor beat_color = {0, 0, 0};
    uint16_t beat_step_index = 0;
    bool beat_accent = false;
    bool beat_vertical = false;
    while (true) {
        if (!led_io) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        uint32_t now = millis();
        bool idle_base_allowed = led_idle_base_allowed();
        bool idle_timer_reset_requested = led_idle_timer_reset_requested;
        bool idle_once_requested = led_idle_once_requested;
        bool force_clear_requested = led_force_clear_requested;
        bool beat_requested = led_beat_requested;

        if (force_clear_requested) {
            led_force_clear_requested = false;
            idle_active = false;
            beat_active = false;
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

        if (beat_requested) {
            led_beat_requested = false;
            beat_active = true;
            beat_started_ms = now;
            beat_end_ms = now + LED_BEAT_DURATION_MS;
            beat_step_index = led_beat_step_index;
            beat_accent = led_beat_accent;
            beat_vertical = led_beat_vertical;
            beat_color = beat_theme_color((beat_step_index / 4) % 4, beat_vertical);
            idle_active = false;
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

        if (beat_active && time_reached(now, beat_end_ms)) {
            beat_active = false;
            if (led_mode == LED_OFF && !idle_active) {
                led_all(0, 0, 0);
            }
        }

        switch (led_mode) {
            case LED_OFF:
                if (beat_active) {
                    led_effect_active = true;
                    uint8_t beat_in_bar = beat_step_index % 4;
                    uint8_t brightness = beat_accent ? 220 : 150;
                    uint8_t tail_brightness = beat_accent ? 120 : 72;
                    uint8_t progress = (uint8_t)(((now - beat_started_ms) * LED_NUM) / max<uint32_t>(1, LED_BEAT_DURATION_MS));
                    if (progress >= LED_NUM) progress = LED_NUM - 1;
                    ScopedLock lock(io_mutex);
                    for (int i = 0; i < LED_NUM; i++) led_io->setLedColor(i, 0, 0, 0);
                    if (beat_vertical) {
                        bool upper_first = (beat_in_bar == 0 || beat_in_bar == 2);
                        bool both_halves = (beat_in_bar == 2);
                        bool full_ring = (beat_in_bar == 3);
                        for (int i = 0; i < LED_NUM; ++i) {
                            bool upper_half = i < (LED_NUM / 2);
                            bool on = full_ring || both_halves || (upper_first ? upper_half : !upper_half);
                            if (!on) continue;
                            uint8_t local = full_ring ? brightness : (upper_half == upper_first ? brightness : tail_brightness);
                            led_io->setLedColor(i,
                                (beat_color.r * local) / 255,
                                (beat_color.g * local) / 255,
                                (beat_color.b * local) / 255);
                        }
                    } else {
                        bool forward = (beat_step_index % 2) == 0;
                        uint8_t head = forward ? progress : (LED_NUM - 1 - progress);
                        uint8_t chase_len = 3 + (beat_in_bar == 2 ? 1 : 0);
                        for (uint8_t j = 0; j < chase_len; ++j) {
                            int idx = forward ? (head - j) : (head + j);
                            while (idx < 0) idx += LED_NUM;
                            idx %= LED_NUM;
                            uint8_t local = (j == 0) ? brightness : tail_brightness;
                            led_io->setLedColor(idx,
                                (beat_color.r * local) / 255,
                                (beat_color.g * local) / 255,
                                (beat_color.b * local) / 255);
                        }
                    }
                    led_io->refreshLeds();
                    vTaskDelay(pdMS_TO_TICKS(18));
                    break;
                }
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
            case LED_MANUAL:
                led_effect_active = true;
                {
                    uint8_t mr = manual_r, mg = manual_g, mb = manual_b;
                    uint8_t spd = manual_speed;
                    uint8_t delay_base = 11 - (spd > 10 ? 10 : (spd < 1 ? 1 : spd));
                    switch (manual_pattern) {
                        case PAT_SOLID:
                            led_all(mr, mg, mb);
                            vTaskDelay(pdMS_TO_TICKS(100));
                            break;
                        case PAT_BLINK:
                            blink_state = !blink_state;
                            led_all(blink_state ? mr : 0, blink_state ? mg : 0, blink_state ? mb : 0);
                            vTaskDelay(pdMS_TO_TICKS(blink_state ? manual_on_ms : manual_off_ms));
                            break;
                        case PAT_BREATH:
                            if (anim_dir > 0) { if (anim < 249) anim += 6; else anim_dir = -1; }
                            else              { if (anim > 6)   anim -= 6; else anim_dir =  1; }
                            led_all((mr * anim) / 255, (mg * anim) / 255, (mb * anim) / 255);
                            vTaskDelay(pdMS_TO_TICKS(15 + delay_base * 3));
                            break;
                        case PAT_CHASE: {
                            ScopedLock lock(io_mutex);
                            for (int i = 0; i < LED_NUM; i++) led_io->setLedColor(i, 0, 0, 0);
                            for (int i = 0; i < 3; i++) {
                                int idx = (chase_pos + i * 4) % LED_NUM;
                                uint8_t br = (i == 0) ? 255 : 100;
                                led_io->setLedColor(idx, (mr * br) / 255, (mg * br) / 255, (mb * br) / 255);
                            }
                            led_io->refreshLeds();
                            chase_pos = (chase_pos + 1) % LED_NUM;
                            vTaskDelay(pdMS_TO_TICKS(30 + delay_base * 6));
                            break;
                        }
                        case PAT_RAINBOW: {
                            ScopedLock lock(io_mutex);
                            for (int i = 0; i < LED_NUM; i++) {
                                uint8_t hue = (uint8_t)(anim + (i * 255 / LED_NUM));
                                uint8_t rr, gg, bb;
                                uint8_t region = hue / 43;
                                uint8_t remainder = (hue - region * 43) * 6;
                                switch (region) {
                                    case 0: rr = 255; gg = remainder; bb = 0; break;
                                    case 1: rr = 255 - remainder; gg = 255; bb = 0; break;
                                    case 2: rr = 0; gg = 255; bb = remainder; break;
                                    case 3: rr = 0; gg = 255 - remainder; bb = 255; break;
                                    case 4: rr = remainder; gg = 0; bb = 255; break;
                                    default: rr = 255; gg = 0; bb = 255 - remainder; break;
                                }
                                led_io->setLedColor(i, rr, gg, bb);
                            }
                            led_io->refreshLeds();
                            anim += 3;
                            vTaskDelay(pdMS_TO_TICKS(20 + delay_base * 4));
                            break;
                        }
                        case PAT_WAVE: {
                            ScopedLock lock(io_mutex);
                            for (int i = 0; i < LED_NUM; i++) {
                                // simple sine-like wave using triangle
                                int phase = ((int)anim + i * 255 / LED_NUM) & 0xFF;
                                uint8_t br = (phase < 128) ? (phase * 2) : ((255 - phase) * 2);
                                led_io->setLedColor(i, (mr * br) / 255, (mg * br) / 255, (mb * br) / 255);
                            }
                            led_io->refreshLeds();
                            anim += 4;
                            vTaskDelay(pdMS_TO_TICKS(20 + delay_base * 4));
                            break;
                        }
                        case PAT_SPLIT: {
                            uint8_t mr2 = manual_r2, mg2 = manual_g2, mb2 = manual_b2;
                            ScopedLock lock(io_mutex);
                            int half = LED_NUM / 2;
                            for (int i = 0; i < LED_NUM; i++) {
                                if (i < half)
                                    led_io->setLedColor(i, mr, mg, mb);
                                else
                                    led_io->setLedColor(i, mr2, mg2, mb2);
                            }
                            led_io->refreshLeds();
                            vTaskDelay(pdMS_TO_TICKS(100));
                            break;
                        }
                        case PAT_ALTERNATE: {
                            uint8_t mr2 = manual_r2, mg2 = manual_g2, mb2 = manual_b2;
                            ScopedLock lock(io_mutex);
                            for (int i = 0; i < LED_NUM; i++) {
                                int idx = (i + chase_pos) % LED_NUM;
                                if (idx % 2 == 0)
                                    led_io->setLedColor(i, mr, mg, mb);
                                else
                                    led_io->setLedColor(i, mr2, mg2, mb2);
                            }
                            led_io->refreshLeds();
                            chase_pos = (chase_pos + 1) % LED_NUM;
                            vTaskDelay(pdMS_TO_TICKS(50 + delay_base * 8));
                            break;
                        }
                        case PAT_SPLIT_BLINK: {
                            uint8_t mr2 = manual_r2, mg2 = manual_g2, mb2 = manual_b2;
                            blink_state = !blink_state;
                            ScopedLock lock(io_mutex);
                            int half = LED_NUM / 2;
                            for (int i = 0; i < LED_NUM; i++) {
                                if (i < half)
                                    led_io->setLedColor(i, blink_state ? mr : 0, blink_state ? mg : 0, blink_state ? mb : 0);
                                else
                                    led_io->setLedColor(i, blink_state ? mr2 : 0, blink_state ? mg2 : 0, blink_state ? mb2 : 0);
                            }
                            led_io->refreshLeds();
                            vTaskDelay(pdMS_TO_TICKS(blink_state ? manual_on_ms : manual_off_ms));
                            break;
                        }
                        case PAT_SPLIT_BREATH: {
                            uint8_t mr2 = manual_r2, mg2 = manual_g2, mb2 = manual_b2;
                            if (anim_dir > 0) { if (anim < 249) anim += 6; else anim_dir = -1; }
                            else              { if (anim > 6)   anim -= 6; else anim_dir =  1; }
                            ScopedLock lock(io_mutex);
                            int half = LED_NUM / 2;
                            for (int i = 0; i < LED_NUM; i++) {
                                if (i < half)
                                    led_io->setLedColor(i, (mr * anim) / 255, (mg * anim) / 255, (mb * anim) / 255);
                                else
                                    led_io->setLedColor(i, (mr2 * anim) / 255, (mg2 * anim) / 255, (mb2 * anim) / 255);
                            }
                            led_io->refreshLeds();
                            vTaskDelay(pdMS_TO_TICKS(15 + delay_base * 3));
                            break;
                        }
                        case PAT_SPLIT_WAVE: {
                            uint8_t mr2 = manual_r2, mg2 = manual_g2, mb2 = manual_b2;
                            ScopedLock lock(io_mutex);
                            int half = LED_NUM / 2;
                            for (int i = 0; i < LED_NUM; i++) {
                                int phase = ((int)anim + i * 255 / LED_NUM) & 0xFF;
                                uint8_t br = (phase < 128) ? (phase * 2) : ((255 - phase) * 2);
                                if (i < half)
                                    led_io->setLedColor(i, (mr * br) / 255, (mg * br) / 255, (mb * br) / 255);
                                else
                                    led_io->setLedColor(i, (mr2 * br) / 255, (mg2 * br) / 255, (mb2 * br) / 255);
                            }
                            led_io->refreshLeds();
                            anim += 4;
                            vTaskDelay(pdMS_TO_TICKS(20 + delay_base * 4));
                            break;
                        }
                        case PAT_SPLIT_CHASE: {
                            uint8_t mr2 = manual_r2, mg2 = manual_g2, mb2 = manual_b2;
                            ScopedLock lock(io_mutex);
                            int half = LED_NUM / 2;
                            for (int i = 0; i < LED_NUM; i++) led_io->setLedColor(i, 0, 0, 0);
                            for (int j = 0; j < 3; j++) {
                                int idx = (chase_pos + j * 4) % LED_NUM;
                                uint8_t br = (j == 0) ? 255 : 100;
                                if (idx < half)
                                    led_io->setLedColor(idx, (mr * br) / 255, (mg * br) / 255, (mb * br) / 255);
                                else
                                    led_io->setLedColor(idx, (mr2 * br) / 255, (mg2 * br) / 255, (mb2 * br) / 255);
                            }
                            led_io->refreshLeds();
                            chase_pos = (chase_pos + 1) % LED_NUM;
                            vTaskDelay(pdMS_TO_TICKS(30 + delay_base * 6));
                            break;
                        }
                    }
                }
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

static HeadGesture head_pat_detected() {
    head_touch_sensor.update();
    if (head_touch_sensor.was_swiped_forward() || head_touch_sensor.was_swiped_backward())
        return GESTURE_PAT;
    if (head_touch_sensor.tap_ready()) {
        uint8_t n = head_touch_sensor.consume_tap_count();
        if (n == 2) return GESTURE_DOUBLE_TAP;
        if (n >= 3) return GESTURE_TRIPLE_TAP;
    }
    return GESTURE_NONE;
}

static void head_pat_reaction(HeadGesture gesture) {
    if (busy) {
        Serial.println("Head pat ignored: busy");
        return;
    }
    if (!servo_ready) {
        Serial.println("Head pat ignored: servo not ready");
        return;
    }
    uint32_t now = millis();
    if (!time_reached(now, head_pat_cooldown_until_ms)) {
        Serial.println("Head pat ignored: cooldown");
        return;
    }

    head_pat_cooldown_until_ms = now + HEAD_PAT_COOLDOWN_MS;
    busy = true;
    led_set(LED_HAPPY);
    avatar.setExpression(Expression::Happy);
    avatar.setLeftGaze(-0.18f, 0.10f);
    avatar.setRightGaze(-0.18f, 0.10f);

    uint32_t reaction_started_ms = millis();

    if (gesture == GESTURE_PAT) {
        Serial.println("Head pat reaction: pat (left-right)");
        show("Pat pat");
        uint32_t next_motion_ms = reaction_started_ms;
        bool motion_phase = false;
        while (!time_reached(millis(), reaction_started_ms + HEAD_PAT_REACTION_MS)) {
            M5.update();
            handle_speak_server();
            led_idle_log_tick();
            uint32_t loop_now = millis();
            if (servo_ready && time_reached(loop_now, next_motion_ms)) {
                motion_phase = !motion_phase;
                sc_servo.moveXY(srv_cx + (motion_phase ? 12 : -12), srv_cy - 8, 220);
                next_motion_ms = loop_now + 320;
            }
            delay(10);
        }
    } else if (gesture == GESTURE_DOUBLE_TAP) {
        // 2 taps: vertical nod
        Serial.println("Head pat reaction: double tap (vertical nod)");
        show("Yes yes");
        uint32_t next_motion_ms = reaction_started_ms;
        bool nod_down = false;
        while (!time_reached(millis(), reaction_started_ms + 2000)) {
            M5.update();
            handle_speak_server();
            led_idle_log_tick();
            uint32_t loop_now = millis();
            if (servo_ready && time_reached(loop_now, next_motion_ms)) {
                nod_down = !nod_down;
                sc_servo.moveXY(srv_cx, nod_down ? srv_cy - 18 : srv_cy, 260);
                next_motion_ms = loop_now + 280;
            }
            delay(10);
        }
    } else {
        // 3+ taps: full X-axis spin (lower_limit → upper_limit → center)
        Serial.println("Head pat reaction: triple tap (full spin)");
        show("Yay!");
        if (servo_ready) {
            auto* sx = system_config.getServoInfo(AXIS_X);
            int x_min = sx ? sx->lower_limit : 0;
            int x_max = sx ? sx->upper_limit : 300;
            sc_servo.moveXY(x_min, srv_cy - 5, 300);
            sc_servo.moveXY(x_max, srv_cy - 5, 1200);
            sc_servo.moveXY(srv_cx, srv_cy,     400);
        } else {
            delay(2000);
        }
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
    servo_web_x = srv_cx;
    servo_web_y = srv_cy;
    servo_ready = true;
    auto* si = system_config.getServoInterval(AvatarMode::NORMAL);
    servo_idle_next_ms = millis() + si->interval_min;
    M5_LOGI("Servo ready cx=%d cy=%d type=%d", srv_cx, srv_cy, system_config.getServoType());
}

static int clamp_servo_axis(int axis, int value) {
    auto* info = system_config.getServoInfo(axis == AXIS_X ? AXIS_X : AXIS_Y);
    if (!info) return value;
    return constrain(value, info->lower_limit, info->upper_limit);
}

static float clamp_servo_axis_float(int axis, float value) {
    auto* info = system_config.getServoInfo(axis == AXIS_X ? AXIS_X : AXIS_Y);
    if (!info) return value;
    return constrain(value, (float)info->lower_limit, (float)info->upper_limit);
}

static float attitude_rad_to_servo_deg(float radians) {
    const float degrees = radians * 180.0f / PI;
    return degrees * (float)mavlink_attitude_scale / 100.0f;
}

static void servo_attitude_set_target(const MavlinkAttitude& attitude) {
    float yaw = attitude_rad_to_servo_deg(attitude.yaw);
    float roll = attitude_rad_to_servo_deg(attitude.roll);
    float yaw_source = mavlink_attitude_yaw_source_roll ? roll : yaw;
    float pitch = attitude_rad_to_servo_deg(attitude.pitch);
    mavlink_attitude_latest_yaw = yaw;
    mavlink_attitude_latest_roll = roll;
    mavlink_attitude_latest_pitch = pitch;
    mavlink_attitude_latest_ready = true;

    if (!servo_ready || !mavlink_attitude_servo_enabled || busy || app_mode != MODE_NORMAL) {
        if (!mavlink_rc_follow_enabled) {
            mavlink_attitude_filter_ready = false;
            mavlink_attitude_target_ready = false;
        }
        return;
    }

    const float yaw_delta = yaw_source - mavlink_attitude_zero_yaw_source;
    const float pitch_delta = pitch - mavlink_attitude_zero_pitch;
    float target_x = mavlink_attitude_yaw_source_roll ? srv_cx + yaw_delta : srv_cx - yaw_delta;
    float target_y = srv_cy - pitch_delta;

    mavlink_attitude_target_x = clamp_servo_axis_float(AXIS_X, target_x);
    mavlink_attitude_target_y = clamp_servo_axis_float(AXIS_Y, target_y);
    mavlink_attitude_target_ready = true;
}

static float rc_channel_to_servo_axis(uint16_t raw, int axis) {
    auto* info = system_config.getServoInfo(axis == AXIS_X ? AXIS_X : AXIS_Y);
    if (!info) return axis == AXIS_X ? srv_cx : srv_cy;

    const float center = axis == AXIS_X ? srv_cx : srv_cy;
    const float value = constrain((float)raw, 1000.0f, 2000.0f);
    if (value >= 1500.0f) {
        return center + (value - 1500.0f) * ((float)info->upper_limit - center) / 500.0f;
    }
    return center - (1500.0f - value) * (center - (float)info->lower_limit) / 500.0f;
}

static void servo_rc_follow_set_target(const MavlinkRcChannels& rc_channels) {
    if (!servo_ready || !mavlink_rc_follow_enabled || busy || app_mode != MODE_NORMAL
        || rc_channels.ch1_raw == UINT16_MAX || rc_channels.ch2_raw == UINT16_MAX) {
        return;
    }

    mavlink_attitude_target_x = clamp_servo_axis_float(AXIS_X, rc_channel_to_servo_axis(rc_channels.ch1_raw, AXIS_X));
    mavlink_attitude_target_y = clamp_servo_axis_float(AXIS_Y, rc_channel_to_servo_axis(rc_channels.ch2_raw, AXIS_Y));
    mavlink_attitude_target_ready = true;
}

static void servo_attitude_reset_front() {
    if (mavlink_attitude_latest_ready) {
        mavlink_attitude_zero_yaw_source = setting_attitude_yaw_source_roll
                                         ? mavlink_attitude_latest_roll
                                         : mavlink_attitude_latest_yaw;
        mavlink_attitude_zero_pitch = mavlink_attitude_latest_pitch;
    } else {
        mavlink_attitude_zero_yaw_source = 0.0f;
        mavlink_attitude_zero_pitch = 0.0f;
    }

    mavlink_attitude_target_x = srv_cx;
    mavlink_attitude_target_y = srv_cy;
    mavlink_attitude_filtered_x = srv_cx;
    mavlink_attitude_filtered_y = srv_cy;
    mavlink_attitude_last_x = srv_cx;
    mavlink_attitude_last_y = srv_cy;
    mavlink_attitude_filter_ready = true;
    mavlink_attitude_target_ready = true;
    mavlink_attitude_next_servo_ms = millis();
    if (servo_ready) {
        sc_servo.moveXYContinuous((float)srv_cx, (float)srv_cy, 180);
    }
}

static void servo_attitude_tick() {
    const uint32_t now = millis();
    if (!servo_ready || (!mavlink_attitude_servo_enabled && !mavlink_rc_follow_enabled)
        || busy || app_mode != MODE_NORMAL || !mavlink_attitude_target_ready) {
        mavlink_attitude_filter_ready = false;
        return;
    }
    if (!time_reached(now, mavlink_attitude_next_servo_ms)) return;

    if (!mavlink_attitude_filter_ready) {
        mavlink_attitude_filtered_x = srv_cx;
        mavlink_attitude_filtered_y = srv_cy;
        mavlink_attitude_last_x = srv_cx;
        mavlink_attitude_last_y = srv_cy;
        mavlink_attitude_filter_ready = true;
    } else {
        constexpr float alpha = 0.32f;
        mavlink_attitude_filtered_x += (mavlink_attitude_target_x - mavlink_attitude_filtered_x) * alpha;
        mavlink_attitude_filtered_y += (mavlink_attitude_target_y - mavlink_attitude_filtered_y) * alpha;
    }

    const float smooth_x = clamp_servo_axis_float(AXIS_X, mavlink_attitude_filtered_x);
    const float smooth_y = clamp_servo_axis_float(AXIS_Y, mavlink_attitude_filtered_y);
    if (fabsf(smooth_x - mavlink_attitude_last_x) < 0.05f && fabsf(smooth_y - mavlink_attitude_last_y) < 0.05f) {
        mavlink_attitude_servo_skips++;
        mavlink_attitude_next_servo_ms = now + 10;
        return;
    }

    servo_idle_enabled = false;
    sc_servo.moveXYContinuous(smooth_x, smooth_y, 60);
    mavlink_attitude_last_x = smooth_x;
    mavlink_attitude_last_y = smooth_y;
    mavlink_attitude_servo_commands++;
    mavlink_attitude_next_servo_ms = millis() + 10;

    if (mavlink_attitude_servo_next_log_ms == 0 || time_reached(now, mavlink_attitude_servo_next_log_ms)) {
        Serial.printf("[MAVLink] servo target=(%.2f,%.2f) filtered=(%.2f,%.2f) cmd_delta=%lu skip_delta=%lu\n",
                      mavlink_attitude_target_x,
                      mavlink_attitude_target_y,
                      smooth_x,
                      smooth_y,
                      static_cast<unsigned long>(mavlink_attitude_servo_commands - mavlink_attitude_servo_commands_last),
                      static_cast<unsigned long>(mavlink_attitude_servo_skips - mavlink_attitude_servo_skips_last));
        mavlink_attitude_servo_commands_last = mavlink_attitude_servo_commands;
        mavlink_attitude_servo_skips_last = mavlink_attitude_servo_skips;
        mavlink_attitude_servo_next_log_ms = now + 2000;
    }
}

// Called from ESP-NOW WiFi task (Core 0). portMUX spinlock makes writes
// atomically visible to Core 1 (Arduino loop) with full cache coherency.
static void on_head_track_recv(const uint8_t* mac, const uint8_t* data, int len) {
    if (len != sizeof(HeadTrackPacket)) return;
    HeadTrackPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    portENTER_CRITICAL_ISR(&head_track_mux);
    head_track_recv_x     = pkt.pan;
    head_track_recv_y     = pkt.tilt;
    head_track_recv_ready = true;
    portEXIT_CRITICAL_ISR(&head_track_mux);
}

static void head_track_tick() {
    const uint32_t now = millis();

    // Snapshot and consume the latest received packet under the spinlock.
    float rx, ry;
    bool ready;
    portENTER_CRITICAL(&head_track_mux);
    ready = head_track_recv_ready;
    rx    = head_track_recv_x;
    ry    = head_track_recv_y;
    if (ready) {
        head_track_recv_ready   = false;
        head_track_last_recv_ms = now;
    }
    portEXIT_CRITICAL(&head_track_mux);

    // If no packet for 500 ms, re-enable idle animation and reset filter.
    if (head_track_last_recv_ms != 0 && (now - head_track_last_recv_ms) > 500) {
        if (!servo_idle_enabled) {
            servo_idle_enabled    = true;
            head_track_filter_ready = false;
        }
        return;
    }

    if (!servo_ready || !ready || busy || app_mode != MODE_NORMAL) {
        if (!ready) head_track_filter_ready = false;
        return;
    }
    if (!time_reached(now, head_track_next_servo_ms)) return;

    if (!head_track_filter_ready) {
        head_track_filtered_x = rx;
        head_track_filtered_y = ry;
        head_track_last_x     = clamp_servo_axis_float(AXIS_X, srv_cx + (rx - 90.0f));
        head_track_last_y     = clamp_servo_axis_float(AXIS_Y, srv_cy - (ry - 90.0f));
        head_track_filter_ready = true;
    } else {
        constexpr float alpha = 0.32f;
        head_track_filtered_x += (rx - head_track_filtered_x) * alpha;
        head_track_filtered_y += (ry - head_track_filtered_y) * alpha;
    }

    // Packet angles are 90-centered (HeadTracker convention); remap onto this
    // unit's own servo centers (e.g. SCS X center=150). Tilt sign flipped here
    // so HeadTracker's TILT_GAIN stays untouched.
    const float sx = clamp_servo_axis_float(AXIS_X, srv_cx + (head_track_filtered_x - 90.0f));
    const float sy = clamp_servo_axis_float(AXIS_Y, srv_cy - (head_track_filtered_y - 90.0f));

    if (fabsf(sx - head_track_last_x) < 0.05f && fabsf(sy - head_track_last_y) < 0.05f) {
        head_track_next_servo_ms = now + 10;
        return;
    }

    servo_idle_enabled = false;
    sc_servo.moveXYContinuous(sx, sy, 60);
    head_track_last_x = sx;
    head_track_last_y = sy;
    head_track_next_servo_ms = now + 10;
}

static void servo_web_move(int x, int y, int move_ms) {
    if (!servo_ready) return;
    servo_idle_enabled = false;
    servo_web_x = clamp_servo_axis(AXIS_X, x);
    servo_web_y = clamp_servo_axis(AXIS_Y, y);
    sc_servo.moveXY(servo_web_x, servo_web_y, constrain(move_ms, 20, 1000));
}

static void servo_web_center(int move_ms = 300) {
    servo_web_move(srv_cx, srv_cy, move_ms);
}

static String websocket_accept_key(const String& key) {
    static const char WS_GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    String src = key + WS_GUID;
    uint8_t sha[20];
    uint8_t b64[32];
    mbedtls_sha1_ret(reinterpret_cast<const unsigned char*>(src.c_str()), src.length(), sha);
    size_t n = base64_encode(b64, sha, sizeof(sha));
    b64[n] = '\0';
    return String(reinterpret_cast<char*>(b64));
}

static void servo_ws_stop() {
    if (servo_ws_client) servo_ws_client.stop();
    servo_ws_active = false;
}

static void servo_ws_apply_text(const String& msg) {
    if (!servo_ready || busy) return;
    int first = msg.indexOf(',');
    if (first < 0) return;
    String head = msg.substring(0, first);
    int move_ms = 90;
    if (head == "c") {
        int ms = msg.substring(first + 1).toInt();
        servo_web_center(ms > 0 ? ms : 180);
        return;
    }
    int second = msg.indexOf(',', first + 1);
    if (second < 0) return;
    int yaw = constrain(head.toInt(), -45, 45);
    int roll = constrain(msg.substring(first + 1, second).toInt(), -35, 35);
    int ms = msg.substring(second + 1).toInt();
    if (ms > 0) move_ms = ms;
    servo_web_move(srv_cx + yaw, srv_cy - roll, move_ms);
}

static bool servo_ws_read_exact(uint8_t* out, size_t len, uint32_t timeout_ms = 20) {
    size_t pos = 0;
    uint32_t deadline = millis() + timeout_ms;
    while (pos < len && millis() < deadline) {
        while (pos < len && servo_ws_client.available()) {
            out[pos++] = static_cast<uint8_t>(servo_ws_client.read());
            deadline = millis() + timeout_ms;
        }
        if (pos < len) delay(1);
    }
    return pos == len;
}

static void servo_ws_tick() {
    if (!servo_ws_active) return;
    if (!servo_ws_client.connected()) {
        servo_ws_stop();
        return;
    }
    String latest_text = "";
    while (servo_ws_client.available() >= 2) {
        uint8_t hdr[2];
        if (!servo_ws_read_exact(hdr, 2)) return;
        uint8_t opcode = hdr[0] & 0x0F;
        bool masked = (hdr[1] & 0x80) != 0;
        uint64_t len = hdr[1] & 0x7F;
        if (len == 126) {
            uint8_t ext[2];
            if (!servo_ws_read_exact(ext, 2)) return;
            len = ((uint16_t)ext[0] << 8) | ext[1];
        } else if (len == 127) {
            servo_ws_stop();
            return;
        }
        if (!masked || len > 96) {
            servo_ws_stop();
            return;
        }
        uint8_t mask[4];
        if (!servo_ws_read_exact(mask, 4)) return;
        char payload[97];
        if (!servo_ws_read_exact(reinterpret_cast<uint8_t*>(payload), len)) return;
        for (uint64_t i = 0; i < len; ++i) payload[i] ^= mask[i & 3];
        payload[len] = '\0';
        if (opcode == 0x8) {
            servo_ws_stop();
            return;
        }
        if (opcode == 0x1) {
            servo_ws_last_ms = millis();
            latest_text = String(payload);
        }
    }
    if (latest_text.length()) servo_ws_apply_text(latest_text);
    if (servo_ws_last_ms && time_reached(millis(), servo_ws_last_ms + 30000)) {
        servo_ws_stop();
    }
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
// System instruction sent with every LLM call to keep replies short.
// Editable via YAML `hermes: system: "..."` (no # or " chars). Falls back to this default.
String hermes_system   = "あなたはStack-chan。返答は日本語で必ず1文・30文字以内。簡潔に。";
int    tts_volume          = 100;
int    hermes_max_tokens   = 60;
int    hermes_timeout_ms   = 180000;
String voicevox_host    = "";  // e.g. "192.168.1.100:50021" - use local VOICEVOX if set
int    voicevox_speaker = 1;

static bool ensure_wifi_connected(uint32_t timeout_ms = 15000) {
    if (WiFi.status() == WL_CONNECTED) return true;

    M5_LOGW("WiFi disconnected (status=%d), reconnecting...", WiFi.status());
    show("WiFi reconnect...");

    if (wifi_credential_count == 0) {
        wifi_s* wifi = system_config.getWiFiSetting();
        if (wifi && !wifi->ssid.isEmpty()) {
            wifi_credentials[0].ssid = wifi->ssid;
            wifi_credentials[0].password = wifi->password;
            wifi_credential_count = 1;
        }
    }

    for (size_t i = 0; i < wifi_credential_count; ++i) {
        if (wifi_credentials[i].ssid.isEmpty()) continue;
        M5_LOGI("WiFi try %d/%d: %s", (int)(i + 1), (int)wifi_credential_count, wifi_credentials[i].ssid.c_str());
        WiFi.disconnect(false, true);
        delay(100);
        WiFi.begin(wifi_credentials[i].ssid.c_str(), wifi_credentials[i].password.c_str());
        uint32_t started_ms = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - started_ms) < timeout_ms) {
            mavlink_receive_tick(false);
            delay(500);
            M5.update();
        }
        if (WiFi.status() == WL_CONNECTED) {
            M5_LOGI("WiFi connected via entry %d: %s", (int)(i + 1), WiFi.localIP().toString().c_str());
            show("IP: " + WiFi.localIP().toString());
            return true;
        }
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
        String sys_override = extract("system");
        if (!sys_override.isEmpty()) hermes_system = sys_override;
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
    load_wifi_credentials_from_yaml(normalized_yaml);
    load_llm_models_from_yaml(normalized_yaml);
    brightness_val = extractInt("brightness", brightness_val);
    servo_on_boot = extractInt("servo_on_boot", servo_on_boot ? 1 : 0) != 0;
    mavlink_attitude_servo_enabled = extractInt("mavlink_attitude_servo", mavlink_attitude_servo_enabled ? 1 : 0) != 0;
    mavlink_rc_follow_enabled = extractInt("mavlink_rc_follow", mavlink_rc_follow_enabled ? 1 : 0) != 0;
    if (mavlink_rc_follow_enabled) mavlink_attitude_servo_enabled = false;
    mavlink_attitude_yaw_source_roll = extractInt("mavlink_yaw_source_roll", mavlink_attitude_yaw_source_roll ? 1 : 0) != 0;
    mavlink_attitude_scale = constrain(extractInt("mavlink_attitude_scale", mavlink_attitude_scale), 0, 100);
    play_bpm = clamp_bpm_value(extractInt("play_bpm", play_bpm));

    periodic_mode_enabled = extractInt("periodic_enabled", 0) != 0;
    uint32_t boot_ms = millis();
    for (int i = 0; i < PERIODIC_MAX; i++) {
        String pre = "p" + String(i) + "_";
        periodic_entries[i].enabled  = extractInt((pre + "enabled").c_str(), 0) != 0;
        String t = extractStr((pre + "type").c_str());
        periodic_entries[i].is_llm   = (t == "llm");
        periodic_entries[i].message  = extractStr((pre + "msg").c_str());
        int mins = extractInt((pre + "min").c_str(), 30);
        if (mins < 1) mins = 1;
        periodic_entries[i].interval_ms  = (uint32_t)mins * 60000UL;
        periodic_entries[i].next_fire_ms = boot_ms + periodic_entries[i].interval_ms;
    }
    btn_llm_tap       = extractInt("btn_llm_tap",       0) != 0;
    btn_periodic_hold = extractInt("btn_periodic_hold",  1) != 0;
    btn_stt_tap       = extractInt("btn_stt_tap",        1) != 0;
    btn_servo_tap     = extractInt("btn_servo_tap",      1) != 0;
    btn_bpm_hold      = extractInt("btn_bpm_hold",       1) != 0;
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
    WiFiClient plain_client;
    WiFiClientSecure secure_client;
    if (hermes_endpoint.startsWith("https://")) {
        secure_client.setInsecure();
        http.begin(secure_client, hermes_endpoint);
    } else {
        http.begin(plain_client, hermes_endpoint);
    }
    http.setTimeout(hermes_timeout_ms);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Client-Source", "stackchan");
    if (!hermes_api_key.isEmpty())
        http.addHeader("Authorization", "Bearer " + hermes_api_key);

    JsonDocument doc;
    doc["model"] = hermes_model;
    JsonArray messages = doc["messages"].to<JsonArray>();
    if (!hermes_system.isEmpty()) {
        JsonObject sys = messages.add<JsonObject>();
        sys["role"] = "system"; sys["content"] = hermes_system;
    }
    JsonObject msg = messages.add<JsonObject>();
    msg["role"] = "user"; msg["content"] = user_message;
    doc["max_tokens"] = hermes_max_tokens; doc["stream"] = false;

    String body; serializeJson(doc, body);
    int status = http.POST(body);
    if (status != 200) {
        String error_text = "Error: HTTP " + String(status);
        if (status == -11) error_text += " (read timeout)";
        http.end();
        return error_text;
    }

    String response = http.getString();
    http.end();

    JsonDocument resp;
    if (deserializeJson(resp, response)) return "Parse error";
    const char* content = resp["choices"][0]["message"]["content"];
    return content ? String(content) : "No content";
}

static String build_llm_chat_prompt(JsonVariant history, const String& user_message) {
    String prompt = "Conversation so far:\n";
    if (history.is<JsonArray>()) {
        JsonArray arr = history.as<JsonArray>();
        for (JsonVariant item : arr) {
            String role = item["role"] | "";
            String content = item["content"] | "";
            if (content.isEmpty()) continue;
            if (role == "assistant") prompt += "Assistant: ";
            else prompt += "User: ";
            prompt += content + "\n";
        }
    }
    prompt += "Latest user message: " + user_message + "\n";
    prompt += "Reply naturally in Japanese, briefly, while keeping the conversation context.";
    return prompt;
}

static String chat_display_text(const String& text) {
    String one_line = text;
    one_line.replace("\r", " ");
    one_line.replace("\n", " ");
    one_line.trim();
    if (one_line.length() > 240) one_line = one_line.substring(0, 239) + ".";
    return one_line;
}

static void chat_add(bool is_user, const String& text) {
    String display = chat_display_text(text);
    if (display.isEmpty()) return;
    if (chat_message_count < CHAT_HISTORY_MAX) {
        chat_messages[chat_message_count++] = {is_user, display};
    } else {
        for (size_t i = 1; i < CHAT_HISTORY_MAX; ++i) chat_messages[i - 1] = chat_messages[i];
        chat_messages[CHAT_HISTORY_MAX - 1] = {is_user, display};
    }
}

static int utf8_next_index(const String& text, int index) {
    if (index >= (int)text.length()) return text.length();
    uint8_t c = (uint8_t)text[index];
    int step = 1;
    if ((c & 0x80) == 0x00) step = 1;
    else if ((c & 0xE0) == 0xC0) step = 2;
    else if ((c & 0xF0) == 0xE0) step = 3;
    else if ((c & 0xF8) == 0xF0) step = 4;
    return min((int)text.length(), index + step);
}

static int utf8_index_after_chars(const String& text, int start, int count) {
    int index = start;
    for (int i = 0; i < count && index < (int)text.length(); ++i) {
        index = utf8_next_index(text, index);
    }
    return index;
}

static int utf8_char_count(const String& text) {
    int count = 0;
    for (int i = 0; i < (int)text.length(); i = utf8_next_index(text, i)) count++;
    return count;
}

static String utf8_window(const String& text, int start_char, int width_chars) {
    int start = utf8_index_after_chars(text, 0, start_char);
    int end = utf8_index_after_chars(text, start, width_chars);
    return text.substring(start, end);
}

static void set_avatar_speech_text_safe(const String& text) {
    avatar.setSpeechText(text.c_str());
}

static void speech_scroll_start(const String& text) {
    speech_scroll_text = chat_display_text(text);
    speech_scroll_active = !speech_scroll_text.isEmpty();
    speech_scroll_started_ms = millis();
    speech_scroll_last_ms = 0;
    speech_scroll_duration_ms = 0;
    speech_scroll_tick();
}

static void speech_scroll_set_duration(uint32_t duration_ms) {
    if (!speech_scroll_active) return;
    speech_scroll_duration_ms = max<uint32_t>(duration_ms, 1000);
    speech_scroll_started_ms = millis();
    speech_scroll_last_ms = 0;
    speech_scroll_tick();
}

static void speech_scroll_tick() {
    if (!speech_scroll_active || speech_scroll_text.isEmpty() || app_mode != MODE_NORMAL) return;
    uint32_t now = millis();
    if (speech_scroll_last_ms && now - speech_scroll_last_ms < SPEECH_SCROLL_STEP_MS) return;
    speech_scroll_last_ms = now;

    int total_chars = utf8_char_count(speech_scroll_text);
    String visible = speech_scroll_text;
    if (total_chars > SPEECH_SCROLL_VISIBLE_CHARS) {
        int max_start = total_chars - SPEECH_SCROLL_VISIBLE_CHARS;
        int start_char = 0;
        if (speech_scroll_duration_ms > 0) {
            uint32_t elapsed_ms = now - speech_scroll_started_ms;
            int span = max_start + 1;
            start_char = (elapsed_ms / SPEECH_SCROLL_STEP_MS) % span;
        }
        visible = utf8_window(speech_scroll_text, start_char, SPEECH_SCROLL_VISIBLE_CHARS);
    }
    set_avatar_speech_text_safe(visible);
}

static void speech_scroll_run_to_end(uint8_t repeat_count) {
    if (!speech_scroll_active || speech_scroll_text.isEmpty() || app_mode != MODE_NORMAL) return;

    int total_chars = utf8_char_count(speech_scroll_text);
    repeat_count = max<uint8_t>(1, repeat_count);

    for (uint8_t repeat = 0; repeat < repeat_count; ++repeat) {
        if (total_chars <= SPEECH_SCROLL_VISIBLE_CHARS) {
            set_avatar_speech_text_safe(speech_scroll_text);
            M5.update();
            delay(900);
            continue;
        }

        int max_start = total_chars - SPEECH_SCROLL_VISIBLE_CHARS;
        for (int start_char = 0; start_char <= max_start; ++start_char) {
            set_avatar_speech_text_safe(utf8_window(speech_scroll_text, start_char, SPEECH_SCROLL_VISIBLE_CHARS));
            M5.update();
            delay(SPEECH_SCROLL_STEP_MS);
        }
        delay(900);
    }
}

static void speech_scroll_stop() {
    speech_scroll_active = false;
    speech_scroll_text = "";
    speech_scroll_duration_ms = 0;
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
    uint32_t speak_next_ms = millis() + 700;
    bool speak_phase = false;
    uint8_t speak_motion_step = 0;
    int speak_horizontal_direction = 1;
    M5.Speaker.setVolume(tts_volume);
    uint32_t audio_ms = 0;
    if (read_len > (int)data_offset && sample_rate > 0) {
        uint32_t sample_count = (read_len - data_offset) / 2;
        audio_ms = (uint32_t)(((uint64_t)sample_count * 1000) / sample_rate);
    }
    if (audio_ms > 0) speech_scroll_set_duration(audio_ms);
    M5.Speaker.playRaw((int16_t*)(wav_buf + data_offset), (read_len - data_offset) / 2, sample_rate, false, 1, 0);
    while (M5.Speaker.isPlaying()) {
        M5.update();
        speech_scroll_tick();
        delay(10);
        if (touchedZone(2)) { M5.Speaker.stop(); break; }
        if (servo_ready) {
            uint32_t now = millis();
            if (now >= speak_next_ms) {
                const uint8_t motion_phase = speak_motion_step % 7;
                if (motion_phase == 5) {
                    speak_horizontal_direction = random(0, 2) == 0 ? -1 : 1;
                    const int target_x = clamp_servo_axis(AXIS_X, srv_cx + speak_horizontal_direction * 22);
                    const int target_y = clamp_servo_axis(AXIS_Y, srv_cy - 4);
                    sc_servo.moveXYContinuous((float)target_x, (float)target_y, 450);
                } else if (motion_phase == 6) {
                    sc_servo.moveXYContinuous((float)srv_cx, (float)srv_cy, 450);
                } else {
                    speak_phase = !speak_phase;
                    const int target_y = clamp_servo_axis(AXIS_Y, speak_phase ? srv_cy - 16 : srv_cy + 4);
                    sc_servo.moveY(target_y, 500, false);
                }
                avatar.setMouthOpenRatio(speak_phase ? random(30, 65) / 100.0f : 0.05f);
                speak_motion_step++;
                speak_next_ms = now + 700;
            }
        }
    }
    avatar.setMouthOpenRatio(0.0f);
    if (servo_ready) sc_servo.moveXY(srv_cx, srv_cy, 500);
}

static bool play_wav_file(const char* path) {
    File file = SPIFFS.open(path, "r");
    if (!file) {
        return false;
    }

    const size_t file_size = file.size();
    if (file_size < 44) {
        M5_LOGE("WAV file too small: %s", path);
        file.close();
        return false;
    }

    uint8_t* wav_buf = (uint8_t*)heap_caps_malloc(file_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!wav_buf) {
        M5_LOGE("WAV malloc failed: %s (%u bytes)", path, (unsigned)file_size);
        file.close();
        return false;
    }

    const size_t read_len = file.read(wav_buf, file_size);
    file.close();
    if (read_len != file_size) {
        M5_LOGE("WAV read failed: %s (%u/%u bytes)", path, (unsigned)read_len, (unsigned)file_size);
        heap_caps_free(wav_buf);
        return false;
    }

    M5_LOGI("Play cached WAV: %s (%u bytes)", path, (unsigned)file_size);
    play_wav(wav_buf, (int)read_len);
    heap_caps_free(wav_buf);
    return true;
}

static bool save_and_play_wav(uint8_t* wav_buf, int read_len, const char* cache_path) {
    if (cache_path && cache_path[0]) {
        File out = SPIFFS.open(cache_path, "w");
        if (out) {
            const size_t written = out.write(wav_buf, read_len);
            out.close();
            if (written == (size_t)read_len) {
                M5_LOGI("Cached WAV: %s (%d bytes)", cache_path, read_len);
            } else {
                M5_LOGE("Cache WAV write incomplete: %s (%u/%d)", cache_path, (unsigned)written, read_len);
            }
        } else {
            M5_LOGE("Cache WAV open failed: %s", cache_path);
        }
    }

    play_wav(wav_buf, read_len);
    return true;
}

static bool call_tts_cache_wav(const String& text, const char* cache_path) {
    bpm_stop_audio();
    if (!ensure_wifi_connected()) return false;

    if (!voicevox_host.isEmpty()) {
        String base = "http://" + voicevox_host;
        String speaker_str = String(voicevox_speaker);

        HTTPClient http1;
        WiFiClient c1;
        http1.begin(c1, base + "/audio_query?text=" + url_encode(text) + "&speaker=" + speaker_str);
        http1.setTimeout(10000);
        int st1 = http1.POST("");
        if (st1 != 200) { M5_LOGE("audio_query error: %d", st1); http1.end(); return false; }
        String query_json = http1.getString();
        http1.end();

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

        bool ok = read_len > 44 && save_and_play_wav(wav_buf, read_len, cache_path);
        heap_caps_free(wav_buf);
        return ok;
    }

    String api_key = system_config.getAPISetting()->tts;
    if (api_key.isEmpty()) { M5_LOGE("TTS API key not set"); return false; }

    String synth_url = String("https://api.tts.quest/v3/voicevox/synthesis?text=")
                     + url_encode(text) + "&speaker=1&key=" + api_key;
    HTTPClient http1;
    WiFiClientSecure c1; c1.setInsecure();
    http1.begin(c1, synth_url);
    http1.setTimeout(15000);
    int st1 = http1.GET();
    if (st1 != 200) { M5_LOGE("TTS synth error: %d", st1); http1.end(); return false; }

    String json_resp = http1.getString();
    http1.end();

    JsonDocument jdoc;
    if (deserializeJson(jdoc, json_resp)) { M5_LOGE("TTS JSON parse error"); return false; }
    if (!jdoc["success"].as<bool>()) {
        M5_LOGE("TTS API success=false: %s", jdoc["errorMessage"].as<const char*>());
        return false;
    }

    String wav_url = jdoc["wavDownloadUrl"].as<String>();
    if (wav_url.isEmpty()) return false;
    for (int i = 0; i < 10; ++i) {
        HTTPClient http2;
        WiFiClientSecure c2; c2.setInsecure();
        http2.begin(c2, wav_url);
        http2.setTimeout(15000);
        int st2 = http2.GET();
        if (st2 == 200) {
            int wav_size = http2.getSize();
            if (wav_size <= 0) wav_size = 512 * 1024;
            uint8_t* wav_buf = (uint8_t*)heap_caps_malloc(wav_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!wav_buf) { M5_LOGE("malloc failed"); http2.end(); return false; }
            int read_len = http2.getStream().readBytes(wav_buf, wav_size);
            http2.end();
            bool ok = read_len > 44 && save_and_play_wav(wav_buf, read_len, cache_path);
            heap_caps_free(wav_buf);
            return ok;
        }
        http2.end();
        delay(1000);
    }
    return false;
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

    // Step2: synthesis (synchronous POST 竊・WAV bytes)
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

    // Step1: request synthesis and receive wavDownloadUrl / audioStatusUrl.
    String synth_url = String("https://api.tts.quest/v3/voicevox/synthesis?text=")
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

    // Step1.5: poll audioStatusUrl until the WAV becomes ready.
    if (!status_url.isEmpty()) {
        M5_LOGI("Polling audio status...");
        bool audio_ready = false;
        for (int attempt = 0; attempt < 40; attempt++) {
            delay(500);
            speech_scroll_tick();
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

    // Step2: download the WAV data.
    HTTPClient http2;
    WiFiClientSecure c2; c2.setInsecure();
    http2.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http2.begin(c2, wav_url);
    http2.setTimeout(15000);
    int st2 = http2.GET();
    if (st2 != 200) { M5_LOGE("WAV download error: %d", st2); http2.end(); return false; }

    int wav_size = http2.getSize();
    if (wav_size <= 0) wav_size = 512 * 1024;  // fallback if content-length is absent
    uint8_t* wav_buf = (uint8_t*)heap_caps_malloc(wav_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!wav_buf) { M5_LOGE("malloc failed"); http2.end(); return false; }

    int read_len = http2.getStream().readBytes(wav_buf, wav_size);
    http2.end();
    M5_LOGI("WAV downloaded: %d bytes", read_len);

    // Step3: parse and play the WAV.
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

    // Base64 encode the recorded PCM.
    size_t b64_len = ((MIC_BUF_BYTES + 2) / 3) * 4 + 1;
    uint8_t* b64_buf = (uint8_t*)heap_caps_malloc(b64_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!b64_buf) { heap_caps_free(rec_buf); M5_LOGE("b64 malloc failed"); return ""; }
    size_t out_len = base64_encode(b64_buf, (uint8_t*)rec_buf, MIC_BUF_BYTES);
    heap_caps_free(rec_buf);

    // Build the Google STT request body in PSRAM.
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

    // Call the Google STT REST API.
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

static void upsert_yaml_quoted(String& yaml, const String& key, const String& value) {
    String search = key + ": \"";
    String esc = value; esc.replace("\\", "\\\\"); esc.replace("\"", "\\\"");
    int pos = yaml.indexOf(search);
    if (pos < 0) { yaml += "\n" + key + ": \"" + esc + "\""; return; }
    int start = pos + search.length();
    int end = yaml.indexOf('"', start);
    if (end < 0) return;
    yaml = yaml.substring(0, start) + esc + yaml.substring(end);
}

static void periodic_tick() {
    if (!periodic_mode_enabled || busy || app_mode != MODE_NORMAL) return;
    uint32_t now = millis();
    for (int i = 0; i < PERIODIC_MAX; i++) {
        PeriodicEntry& e = periodic_entries[i];
        if (!e.enabled || e.interval_ms == 0 || e.message.isEmpty()) continue;
        if (!time_reached(now, e.next_fire_ms)) continue;
        e.next_fire_ms = now + e.interval_ms;
        busy = true;
        String text;
        if (e.is_llm) {
            avatar.setExpression(Expression::Doubt);
            led_set(LED_THINKING);
            show("Thinking...");
            text = call_hermes(e.message);
            if (text.startsWith("Error:") || text.startsWith("Parse error") || text.startsWith("No content")) {
                show_error_state(text);
                busy = false;
                return;
            }
        } else {
            text = e.message;
        }
        avatar.setExpression(Expression::Happy);
        led_set(LED_SPEAKING);
        show("[Speaking...]");
        chat_add(false, text);
        speech_scroll_start(text);
        speech_scroll_run_to_end();
        call_tts(text);
        speech_scroll_stop();
        avatar.setExpression(Expression::Neutral);
        led_set(LED_OFF);
        show("");
        busy = false;
        break;
    }
}

void show(const String& text) {
    String oneline = text;
    oneline.replace("\n", " ");
    set_avatar_speech_text_safe(oneline);
    Serial.println(text);
}

static void queue_http_beat(int step_index, bool accent, int bpm_hint, const String& motion) {
    if (!servo_ready) return;

    auto* sx = system_config.getServoInfo(AXIS_X);
    auto* sy = system_config.getServoInfo(AXIS_Y);
    int play_center_y = srv_cy - 10;
    if (sy) {
        play_center_y = max<int>(sy->lower_limit, min<int>(sy->upper_limit, play_center_y));
    }

    String motion_mode = motion;
    motion_mode.toLowerCase();
    bool vertical_motion = motion_mode == "ud";

    int target_x = srv_cx;
    int target_y = play_center_y;
    if (vertical_motion) {
        int nod = accent ? 26 : 7;   // accent: deep nod, off-beat: small bob
        target_y = play_center_y - nod;
        if (sy) {
            target_y = max<int>(sy->lower_limit, min<int>(sy->upper_limit, target_y));
        }
    } else {
        int swing = accent ? 32 : 8;  // accent: big swing, off-beat: small swing
        bool swing_right = (step_index % 2) == 0;
        target_x = srv_cx + (swing_right ? swing : -swing);
        if (sx) {
            target_x = max<int>(sx->lower_limit, min<int>(sx->upper_limit, target_x));
        }
    }

    if (bpm_hint >= 60 && bpm_hint <= 180) {
        play_bpm = clamp_bpm_value(bpm_hint);
    }

    http_beat_target_x = target_x;
    http_beat_target_y = target_y;
    http_beat_motion_vertical = vertical_motion;
    http_beat_pending = true;
    led_beat_step_index = step_index < 0 ? 0 : static_cast<uint16_t>(step_index);
    led_beat_accent = accent;
    led_beat_vertical = vertical_motion;
    led_beat_requested = true;
}

static void http_beat_tick() {
    if (!servo_ready) return;

    uint32_t now = millis();
    if (http_beat_pending) {
        sc_servo.moveXY(http_beat_target_x, http_beat_target_y, 140);
        avatar.setMouthOpenRatio(0.10f);
        http_beat_pending = false;
        http_beat_return_pending = true;
        http_beat_return_at_ms = now + 220;
        return;
    }

    if (http_beat_return_pending && time_reached(now, http_beat_return_at_ms)) {
        if (http_beat_motion_vertical) {
            sc_servo.moveXY(srv_cx, srv_cy, 120);
        } else {
            sc_servo.moveXY(srv_cx, http_beat_target_y, 120);
        }
        avatar.setMouthOpenRatio(0.0f);
        http_beat_return_pending = false;
    }
}

// ---- Web Config Server ----

static String html_escape(const String& s) {
    String out;
    out.reserve(s.length() + 8);
    for (int i = 0; i < (int)s.length(); i++) {
        char c = s[i];
        if      (c == '&')  out += "&amp;";
        else if (c == '<')  out += "&lt;";
        else if (c == '>')  out += "&gt;";
        else if (c == '"')  out += "&quot;";
        else                out += c;
    }
    return out;
}

static String json_escape(const String& s) {
    String out;
    out.reserve(s.length() + 8);
    for (int i = 0; i < (int)s.length(); i++) {
        char c = s[i];
        if (c == '\\' || c == '"') {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else if (c == '\t') {
            out += "\\t";
        } else {
            out += c;
        }
    }
    return out;
}

static String url_decode(const String& src) {
    String out;
    out.reserve(src.length());
    for (int i = 0; i < (int)src.length(); i++) {
        if (src[i] == '%' && i + 2 < (int)src.length()) {
            char h[3] = {src[i+1], src[i+2], 0};
            out += (char)strtol(h, nullptr, 16);
            i += 2;
        } else if (src[i] == '+') {
            out += ' ';
        } else {
            out += src[i];
        }
    }
    return out;
}

static String form_field(const String& body, const char* key) {
    String search = String(key) + "=";
    int pos = body.indexOf(search);
    if (pos < 0) return "";
    pos += search.length();
    int end = body.indexOf('&', pos);
    return url_decode(end < 0 ? body.substring(pos) : body.substring(pos, end));
}

static bool form_has_checked(const String& body, const char* key) {
    String search = String(key) + "=";
    return body.indexOf(search) >= 0;
}

static void update_yaml_quoted_str(String& yaml, const char* key_with_quote, const String& value) {
    String search = String(key_with_quote);
    int pos = yaml.indexOf(search);
    if (pos < 0) return;
    int start = pos + search.length();
    int end = yaml.indexOf('"', start);
    if (end < 0) return;
    String esc = value; esc.replace("\\", "\\\\"); esc.replace("\"", "\\\"");
    yaml = yaml.substring(0, start) + esc + yaml.substring(end);
}

static void update_yaml_quoted_in_section(String& yaml, const char* section, const char* key_with_quote, const String& value) {
    int section_pos = yaml.indexOf(String(section) + ":");
    if (section_pos < 0) return;
    String search = String(key_with_quote);
    int pos = yaml.indexOf(search, section_pos);
    if (pos < 0) return;
    int start = pos + search.length();
    int end = yaml.indexOf('"', start);
    if (end < 0) return;
    String esc = value; esc.replace("\\", "\\\\"); esc.replace("\"", "\\\"");
    yaml = yaml.substring(0, start) + esc + yaml.substring(end);
}

static void load_wifi_credentials_from_yaml(const String& normalized_yaml) {
    wifi_credential_count = 0;
    int list_pos = normalized_yaml.indexOf("wifi_list:");
    if (list_pos >= 0) {
        int scan = list_pos;
        while (wifi_credential_count < WIFI_MAX) {
            int ssid_pos = normalized_yaml.indexOf("  - ssid: \"", scan);
            if (ssid_pos < 0) break;
            int ssid_start = ssid_pos + String("  - ssid: \"").length();
            int ssid_end = normalized_yaml.indexOf('"', ssid_start);
            if (ssid_end < 0) break;
            String ssid = normalized_yaml.substring(ssid_start, ssid_end);

            int pass_pos = normalized_yaml.indexOf("    password: \"", ssid_end);
            if (pass_pos < 0) break;
            int pass_start = pass_pos + String("    password: \"").length();
            int pass_end = normalized_yaml.indexOf('"', pass_start);
            if (pass_end < 0) break;
            String password = normalized_yaml.substring(pass_start, pass_end);

            if (!ssid.isEmpty()) {
                wifi_credentials[wifi_credential_count].ssid = ssid;
                wifi_credentials[wifi_credential_count].password = password;
                wifi_credential_count++;
            }
            scan = pass_end;
        }
    }

    if (wifi_credential_count == 0) {
        wifi_s* wifi = system_config.getWiFiSetting();
        if (wifi && !wifi->ssid.isEmpty()) {
            wifi_credentials[0].ssid = wifi->ssid;
            wifi_credentials[0].password = wifi->password;
            wifi_credential_count = 1;
        }
    }
}

// Built-in fallback used when YAML has no `llm_models:` list.
static void load_llm_default_models() {
    static const struct { const char* label; const char* slug; } defaults[] = {
        {"Gemini Flash", "google/gemini-2.5-flash"},
        {"GPT-4o mini",  "openai/gpt-4o-mini"},
        {"Claude Haiku", "anthropic/claude-3.5-haiku"},
    };
    llm_model_count = 0;
    for (auto& d : defaults) {
        llm_models[llm_model_count].label = d.label;
        llm_models[llm_model_count].slug  = d.slug;
        llm_model_count++;
    }
}

// Parse `llm_models:` list (same style as wifi_list:) into llm_models[].
//   llm_models:
//     - label: "Gemini Flash"
//       model: "google/gemini-2.5-flash"
static void load_llm_models_from_yaml(const String& normalized_yaml) {
    llm_model_count = 0;
    int list_pos = normalized_yaml.indexOf("llm_models:");
    if (list_pos >= 0) {
        int scan = list_pos;
        while (llm_model_count < LLM_MODEL_MAX) {
            int label_pos = normalized_yaml.indexOf("  - label: \"", scan);
            if (label_pos < 0) break;
            int label_start = label_pos + String("  - label: \"").length();
            int label_end = normalized_yaml.indexOf('"', label_start);
            if (label_end < 0) break;
            String label = normalized_yaml.substring(label_start, label_end);

            int model_pos = normalized_yaml.indexOf("    model: \"", label_end);
            if (model_pos < 0) break;
            int model_start = model_pos + String("    model: \"").length();
            int model_end = normalized_yaml.indexOf('"', model_start);
            if (model_end < 0) break;
            String slug = normalized_yaml.substring(model_start, model_end);

            if (!slug.isEmpty()) {
                llm_models[llm_model_count].label = label.isEmpty() ? slug : label;
                llm_models[llm_model_count].slug  = slug;
                llm_model_count++;
            }
            scan = model_end;
        }
    }
    if (llm_model_count == 0) load_llm_default_models();
    M5_LOGI("LLM models loaded: %u", (unsigned)llm_model_count);
}

static void replace_wifi_yaml_sections(String& yaml, WifiCredential entries[], size_t entry_count) {
    String first_ssid = entry_count > 0 ? entries[0].ssid : "";
    String first_pass = entry_count > 0 ? entries[0].password : "";
    first_ssid.replace("\\", "\\\\"); first_ssid.replace("\"", "\\\"");
    first_pass.replace("\\", "\\\\"); first_pass.replace("\"", "\\\"");

    String wifi_section = "wifi:\n";
    wifi_section += "  ssid: \"" + first_ssid + "\"\n";
    wifi_section += "  password: \"" + first_pass + "\"\n";
    wifi_section += "wifi_list:\n";
    for (size_t i = 0; i < entry_count; ++i) {
        if (entries[i].ssid.isEmpty()) continue;
        String ssid = entries[i].ssid;
        String pass = entries[i].password;
        ssid.replace("\\", "\\\\"); ssid.replace("\"", "\\\"");
        pass.replace("\\", "\\\\"); pass.replace("\"", "\\\"");
        wifi_section += "  - ssid: \"" + ssid + "\"\n";
        wifi_section += "    password: \"" + pass + "\"\n";
    }

    int wifi_pos = yaml.indexOf("wifi:");
    int apikey_pos = yaml.indexOf("apikey:");
    if (wifi_pos >= 0 && apikey_pos > wifi_pos) {
        yaml = yaml.substring(0, wifi_pos) + wifi_section + yaml.substring(apikey_pos);
    }
}

static void handle_config_get(WiFiClient& client) {
    auto* api    = system_config.getAPISetting();
    String sttk  = api      ? html_escape(api->stt)       : "";
    String ttsk  = api      ? html_escape(api->tts)       : "";
    String current_ssid = WiFi.status() == WL_CONNECTED ? html_escape(WiFi.SSID()) : "";
    String current_ip = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "not connected";
    if (wifi_credential_count == 0) {
        wifi_s* wifi = system_config.getWiFiSetting();
        if (wifi && !wifi->ssid.isEmpty()) {
            wifi_credentials[0].ssid = wifi->ssid;
            wifi_credentials[0].password = wifi->password;
            wifi_credential_count = 1;
        }
    }

    String html;
    html.reserve(11000);
    html  = F("<!DOCTYPE html><html><head>"
              "<meta charset=UTF-8>"
              "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
              "<title>StackChan Settings</title>"
              "<style>"
              "*{box-sizing:border-box}"
              "body{font:14px sans-serif;max-width:560px;margin:16px auto;padding:8px}"
              "h2{color:#333;margin-bottom:4px}"
              "h3{color:#555;border-bottom:1px solid #ddd;padding-bottom:4px;margin:0 0 8px}"
              "label{display:block;margin-top:8px;font-weight:bold}"
              "input{width:100%;padding:6px;border:1px solid #ccc;border-radius:3px;margin-top:2px;font-size:13px}"
              "input[type=number]{width:130px}"
              "input[type=checkbox],input[type=radio]{width:auto;margin-right:6px}"
              "input[type=checkbox]{transform:scale(1.2)}"
              "input[type=range]{padding:0;border:none;cursor:pointer;accent-color:#4CAF50}"
              ".range-val{display:inline-block;min-width:2em;text-align:right;font-weight:normal;color:#555}"
              ".sec{margin:10px 0;padding:10px 12px;border:1px solid #ddd;border-radius:6px}"
              ".net-info{margin:0 0 10px;padding:8px;border-radius:6px;background:#f6f8fa;color:#333;line-height:1.5}"
              ".net-info b{display:inline-block;min-width:4.5em}"
              ".tabs{display:flex;gap:8px;margin:10px 0 14px;flex-wrap:wrap}"
              ".tab-btn{padding:10px 12px;border:1px solid #bbb;border-radius:999px;background:#f4f4f4;cursor:pointer;font-size:13px}"
              ".tab-btn.active{background:#1f6feb;color:#fff;border-color:#1f6feb}"
              ".tab{display:none}"
              ".tab.active{display:block}"
              ".check{display:flex;align-items:center;gap:8px;margin-top:12px;font-weight:bold}"
              ".chat-log{height:280px;overflow:auto;border:1px solid #ddd;border-radius:8px;padding:10px;background:#fafafa}"
              ".bubble{max-width:88%;padding:8px 10px;border-radius:12px;margin:8px 0;white-space:pre-wrap;word-break:break-word}"
              ".bubble.user{margin-left:auto;background:#dbeafe}"
              ".bubble.assistant{margin-right:auto;background:#e8f5e9}"
              ".speak-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}"
              "textarea{width:100%;min-height:88px;padding:8px;border:1px solid #ccc;border-radius:6px;font-size:13px}"
              ".mini-btn{padding:10px 12px;border:1px solid #1f6feb;border-radius:8px;background:#1f6feb;color:#fff;cursor:pointer}"
              ".servo-grid{display:grid;grid-template-columns:1fr;gap:12px}"
              ".jog-pad{position:relative;width:min(78vw,320px);aspect-ratio:1;margin:8px auto;border-radius:8px;"
              "background:linear-gradient(135deg,#f8fbff,#dbeafe);"
              "border:1px solid #93c5fd;touch-action:none;user-select:none}"
              ".jog-axis{position:absolute;background:rgba(30,64,175,.22)}"
              ".jog-axis.h{left:8%;right:8%;top:50%;height:1px}"
              ".jog-axis.v{top:8%;bottom:8%;left:50%;width:1px}"
              ".jog-dot{position:absolute;width:34px;height:34px;margin:-17px 0 0 -17px;border-radius:50%;"
              "background:#1f6feb;border:3px solid #fff;box-shadow:0 2px 8px rgba(0,0,0,.28);left:50%;top:50%}"
              ".servo-readout{display:grid;grid-template-columns:1fr 1fr;gap:8px;color:#333}"
              ".servo-readout div{background:#f6f8fa;border:1px solid #ddd;border-radius:6px;padding:8px}"
              ".btn{display:block;width:100%;padding:12px;background:#4CAF50;color:#fff;"
              "border:none;border-radius:4px;cursor:pointer;font-size:15px;margin-top:14px}"
              ".btn:hover{background:#45a049}"
              "</style>"
              "<script>"
              "function openTab(id){document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));"
              "document.querySelectorAll('.tab-btn').forEach(t=>t.classList.remove('active'));"
              "document.getElementById(id).classList.add('active');"
              "document.querySelector('[data-tab=\"'+id+'\"]').classList.add('active');"
              "const saveBtn=document.getElementById('save-btn');"
              "if(saveBtn)saveBtn.style.display=(id==='tab-speak'||id==='tab-servo'||id==='tab-led')?'none':'block';}"
              "function appendChat(role,text){const log=document.getElementById('chat-log');"
              "if(!log)return;"
              "const div=document.createElement('div');div.className='bubble '+role;div.textContent=text;"
              "log.appendChild(div);log.scrollTop=log.scrollHeight;}"
              "function setSpeakStatus(text){const el=document.getElementById('speak-status');if(el)el.textContent=text;}"
              "function formatSpeakStatus(data){"
              "const parts=[];"
              "if(data.mode==='llm'&&typeof data.llm_ms==='number')parts.push('LLM '+data.llm_ms+'ms');"
              "if(typeof data.tts_ms==='number')parts.push('TTS '+data.tts_ms+'ms');"
              "if(typeof data.total_ms==='number')parts.push('Total '+data.total_ms+'ms');"
              "parts.push(data.speaking?'Speaking...':(data.spoken?'Spoken':'Done'));"
              "return parts.join(' / ');}"
              "function speakHistAdd(text,mode){"
              "const hist=document.getElementById('speak-history');if(!hist)return;"
              "for(const c of hist.children){if(c._text===text&&c._mode===mode)return;}"
              "const div=document.createElement('div');"
              "div.style.cssText='display:flex;align-items:center;gap:8px;padding:6px 8px;margin:3px 0;background:#f6f8fa;border:1px solid #ddd;border-radius:6px;cursor:pointer;font-size:13px';"
              "div._text=text;div._mode=mode;"
              "const label=mode==='llm'?'LLM':'Fixed';"
              "div.innerHTML='<span style=\"color:#888;flex-shrink:0;font-size:11px\">'+label+'</span>'"
              "+'<span style=\"flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap\">'+text.replace(/</g,'&lt;')+'</span>';"
              "div.onclick=()=>{document.getElementById('speak-text').value=div._text;"
              "document.querySelector('input[name=speak_mode][value='+div._mode+']').checked=true;};"
              "hist.insertBefore(div,hist.firstChild);"
              "while(hist.children.length>10)hist.removeChild(hist.lastChild);}"
              "async function sendSpeak(){"
              "const mode=document.querySelector('input[name=\"speak_mode\"]:checked').value;"
              "const input=document.getElementById('speak-text');"
              "const text=input.value.trim();"
              "if(!text){setSpeakStatus('テキストを入力してください');return;}"
              "window.stackChanChat=window.stackChanChat||[];"
              "appendChat('user',text);"
              "if(mode==='llm'){window.stackChanChat.push({role:'user',content:text});}"
              "input.value='';"
              "setSpeakStatus(mode==='llm'?'Thinking...':'Speaking...');"
              "try{"
              "const payload={mode:mode,text:text,history:window.stackChanChat};"
              "const resp=await fetch('/speak',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});"
              "const raw=await resp.text();"
              "if(!raw){throw new Error('Empty response from StackChan');}"
              "const data=JSON.parse(raw);"
              "if(!resp.ok||!data.ok){throw new Error(data.error||('HTTP '+resp.status));}"
              "if(data.reply){appendChat('assistant',data.reply);window.stackChanChat.push({role:'assistant',content:data.reply});}"
              "setSpeakStatus(formatSpeakStatus(data));"
              "speakHistAdd(text,mode);"
              "}catch(err){setSpeakStatus('Error: '+err.message);}}"
              "let servoWs=null,servoWsReady=false;"
              "function setServoStatus(text){const el=document.getElementById('servo-status');if(el)el.textContent=text;}"
              "function connectServoWs(){"
              "if(servoWsReady||servoWs)return;"
              "const proto=location.protocol==='https:'?'wss':'ws';"
              "servoWs=new WebSocket(proto+'://'+location.host+'/servo-ws');"
              "servoWs.onopen=()=>{servoWsReady=true;setServoStatus('WebSocket');};"
              "servoWs.onclose=()=>{servoWsReady=false;servoWs=null;setServoStatus('HTTP fallback');};"
              "servoWs.onerror=()=>{servoWsReady=false;};"
              "}"
              "async function sendServo(x,y,center){"
              "const ms=Number(document.getElementById('servo-ms').value)||40;"
              "if(servoWsReady&&servoWs){"
              "if(servoWs.bufferedAmount>64)return {ok:false,skipped:true};"
              "servoWs.send(center?('c,'+ms):(x+','+y+','+ms));"
              "document.getElementById('servo-yaw').textContent=center?0:x;"
              "document.getElementById('servo-roll').textContent=center?0:y;"
              "return {ok:true,x:center?0:x,y:center?0:y};"
              "}"
              "const payload=center?{center:true,ms:ms}:{x:x,y:y,ms:ms};"
              "const resp=await fetch('/servo',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});"
              "const data=await resp.json();"
              "if(!resp.ok||!data.ok){throw new Error(data.error||('HTTP '+resp.status));}"
              "document.getElementById('servo-yaw').textContent=data.x;"
              "document.getElementById('servo-roll').textContent=data.y;"
              "return data;}"
              "function setJogDot(nx,ny){const dot=document.getElementById('jog-dot');if(dot){dot.style.left=(50+nx*46)+'%';dot.style.top=(50+ny*46)+'%';}}"
              "function initServoPad(){"
              "const pad=document.getElementById('jog-pad');if(!pad)return;"
              "connectServoWs();"
              "let active=false,last=0,lastYaw=null,lastRoll=null;"
              "const update=e=>{if(!active)return;const r=pad.getBoundingClientRect();"
              "const nx=Math.max(-1,Math.min(1,((e.clientX-r.left)/r.width-.5)*2));"
              "const ny=Math.max(-1,Math.min(1,((e.clientY-r.top)/r.height-.5)*2));"
              "setJogDot(nx,ny);"
              "const yaw=Math.round(nx*30);const roll=Math.round(-ny*22);"
              "document.getElementById('servo-yaw').textContent=yaw;"
              "document.getElementById('servo-roll').textContent=roll;"
              "const now=Date.now();if(now-last<20||yaw===lastYaw&&roll===lastRoll)return;last=now;lastYaw=yaw;lastRoll=roll;"
              "sendServo(yaw,roll,false).catch(err=>setServoStatus('Error: '+err.message));};"
              "pad.addEventListener('pointerdown',e=>{active=true;pad.setPointerCapture(e.pointerId);update(e);});"
              "pad.addEventListener('pointermove',update);"
              "pad.addEventListener('pointerup',()=>{active=false;lastYaw=null;lastRoll=null;});"
              "pad.addEventListener('pointercancel',()=>{active=false;lastYaw=null;lastRoll=null;});"
              "}"
              "function setLedStatus(t){const e=document.getElementById('led-status');if(e)e.textContent=t;}"
              "async function sendLed(preset){"
              "setLedStatus('Applying...');"
              "try{const r=await fetch('/led',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({preset:preset})});"
              "const d=await r.json();if(d.ok)setLedStatus(d.pattern+' '+d.color);else setLedStatus('Error');"
              "}catch(e){setLedStatus('Error: '+e.message);}}"
              "function ledPatChg(){const p=document.getElementById('led-pattern').value;"
              "const w=document.getElementById('led-c2-wrap');"
              "if(w)w.style.display=(p.startsWith('split')||p==='alternate')?'block':'none';}"
              "async function applyLed(){"
              "setLedStatus('Applying...');"
              "const pat=document.getElementById('led-pattern').value;"
              "const col=document.getElementById('led-color').value;"
              "const spd=Number(document.getElementById('led-speed').value);"
              "const payload={pattern:pat,color:col,speed:spd};"
              "if(pat.startsWith('split')||pat==='alternate')payload.color2=document.getElementById('led-color2').value;"
              "try{const r=await fetch('/led',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});"
              "const d=await r.json();if(d.ok)setLedStatus(d.pattern+' '+d.color);else setLedStatus('Error');"
              "}catch(e){setLedStatus('Error: '+e.message);}}"
              "function initColorMap(){"
              "const cv=document.getElementById('led-cmap');if(!cv)return;"
              "const ctx=cv.getContext('2d');"
              "const w=cv.width,h=cv.height;"
              "for(let x=0;x<w;x++){"
              "const hue=x*360/w;"
              "for(let y=0;y<h;y++){"
              "const sat=100;"
              "const lit=100-y*100/h;"
              "ctx.fillStyle='hsl('+hue+','+sat+'%,'+lit+'%)';"
              "ctx.fillRect(x,y,1,1);}}"
              "function pick(e){"
              "const r=cv.getBoundingClientRect();"
              "const ex=e.touches?e.touches[0].clientX:e.clientX;"
              "const ey=e.touches?e.touches[0].clientY:e.clientY;"
              "const px=Math.max(0,Math.min(w-1,Math.round((ex-r.left)*w/r.width)));"
              "const py=Math.max(0,Math.min(h-1,Math.round((ey-r.top)*h/r.height)));"
              "const d=ctx.getImageData(px,py,1,1).data;"
              "const hex='#'+((1<<24)+(d[0]<<16)+(d[1]<<8)+d[2]).toString(16).slice(1).toUpperCase();"
              "document.getElementById('led-color').value=hex;"
              "document.getElementById('led-cval').textContent=hex;"
              "document.getElementById('led-cswatch').style.background=hex;ledRtSend();}"
              "cv.addEventListener('pointerdown',e=>{e.preventDefault();pick(e);cv._picking=true;});"
              "cv.addEventListener('pointermove',e=>{if(cv._picking){e.preventDefault();pick(e);}});"
              "cv.addEventListener('pointerup',()=>{cv._picking=false;});"
              "cv.addEventListener('pointercancel',()=>{cv._picking=false;});"
              "document.getElementById('led-cswatch').style.background='#FF8800';"
              "const cv2=document.getElementById('led-cmap2');if(cv2){"
              "const ctx2=cv2.getContext('2d');"
              "const w2=cv2.width,h2=cv2.height;"
              "for(let x=0;x<w2;x++){const hue=x*360/w2;"
              "for(let y=0;y<h2;y++){ctx2.fillStyle='hsl('+hue+',100%,'+(100-y*100/h2)+'%)';ctx2.fillRect(x,y,1,1);}}"
              "function pick2(e){"
              "const r2=cv2.getBoundingClientRect();"
              "const ex=e.touches?e.touches[0].clientX:e.clientX;"
              "const ey=e.touches?e.touches[0].clientY:e.clientY;"
              "const px=Math.max(0,Math.min(w2-1,Math.round((ex-r2.left)*w2/r2.width)));"
              "const py=Math.max(0,Math.min(h2-1,Math.round((ey-r2.top)*h2/r2.height)));"
              "const d2=ctx2.getImageData(px,py,1,1).data;"
              "const hex='#'+((1<<24)+(d2[0]<<16)+(d2[1]<<8)+d2[2]).toString(16).slice(1).toUpperCase();"
              "document.getElementById('led-color2').value=hex;"
              "document.getElementById('led-cval2').textContent=hex;"
              "document.getElementById('led-cswatch2').style.background=hex;ledRtSend();}"
              "cv2.addEventListener('pointerdown',e=>{e.preventDefault();pick2(e);cv2._picking=true;});"
              "cv2.addEventListener('pointermove',e=>{if(cv2._picking){e.preventDefault();pick2(e);}});"
              "cv2.addEventListener('pointerup',()=>{cv2._picking=false;});"
              "cv2.addEventListener('pointercancel',()=>{cv2._picking=false;});"
              "document.getElementById('led-cswatch2').style.background='#FFFFFF';}}"
              "async function ledGenApply(data,el){"
              "setLedStatus('Applying...');"
              "const p={pattern:data.pattern,color:data.color,speed:data.speed};"
              "if(data.color2)p.color2=data.color2;"
              "try{const r=await fetch('/led',{method:'POST',headers:{'Content-Type':'application/json'},"
              "body:JSON.stringify(p)});"
              "const d=await r.json();if(d.ok)setLedStatus(d.pattern+' '+d.color);else setLedStatus('Error');"
              "document.querySelectorAll('.gen-hist-item').forEach(e=>e.style.outline='none');"
              "if(el)el.style.outline='2px solid #4CAF50';"
              "}catch(e){setLedStatus('Error: '+e.message);}}"
              "function ledGenAddHistory(d){"
              "const hist=document.getElementById('led-gen-history');if(!hist)return null;"
              "const dual=d.color2&&(d.pattern.startsWith('split')||d.pattern==='alternate');"
              "const swatch=dual?'linear-gradient(90deg,'+d.color+' 50%,'+d.color2+' 50%)':d.color;"
              "const div=document.createElement('div');div.className='gen-hist-item';"
              "div.style.cssText='display:flex;align-items:center;gap:8px;padding:6px 8px;margin:3px 0;background:#f6f8fa;border:1px solid #ddd;border-radius:6px;cursor:pointer;font-size:13px';"
              "div._data=d;"
              "div.innerHTML='<span style=\"display:inline-block;width:22px;height:22px;border-radius:3px;border:1px solid #999;flex-shrink:0;background:'+swatch+'\"></span>'"
              "+'<span style=\"flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap\">'+d.pattern+(d.description?' — '+d.description:'')+'</span>'"
              "+'<span style=\"color:#888;flex-shrink:0\">spd:'+d.speed+'</span>';"
              "div.onclick=()=>ledGenApply(div._data,div);"
              "hist.insertBefore(div,hist.firstChild);"
              "while(hist.children.length>5)hist.removeChild(hist.lastChild);"
              "return div;}"
              "async function ledGen(){"
              "document.getElementById('led-gen-status').textContent='Generating...';"
              "const theme=(document.getElementById('led-gen-theme').value||'').trim();"
              "try{const r=await fetch('/led-gen',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(theme?{theme:theme}:{})});"
              "const d=await r.json();"
              "if(!d.ok){document.getElementById('led-gen-status').textContent='Error: '+(d.error||'unknown');return;}"
              "document.getElementById('led-gen-status').textContent='';"
              "const el=ledGenAddHistory(d);"
              "await ledGenApply(d,el);"
              "}catch(e){document.getElementById('led-gen-status').textContent='Error: '+e.message;}}"
              "let _ledRtTimer=null;"
              "function ledRtSend(){if(!document.getElementById('led-rt').checked)return;"
              "if(_ledRtTimer)clearTimeout(_ledRtTimer);"
              "_ledRtTimer=setTimeout(()=>{_ledRtTimer=null;applyLed();},80);}"
              "let _walkTimer=null,_walkCdTimer=null,_walkActive=false,_walkSec=0;"
              "const _walkPresets=['warm_white','cool_blue','alert_red','calm_breath','christmas','ocean','rainbow','party'];"
              "function walkCdUpdate(){"
              "const ws=document.getElementById('led-walk-status');"
              "if(!ws||!_walkActive)return;"
              "const cur=ws.getAttribute('data-label')||'';"
              "ws.textContent=cur+' ('+_walkSec+'s)';}"
              "function walkCdStart(){"
              "_walkSec=30;walkCdUpdate();"
              "if(_walkCdTimer)clearInterval(_walkCdTimer);"
              "_walkCdTimer=setInterval(()=>{if(_walkSec>0)_walkSec--;walkCdUpdate();},1000);}"
              "async function ledWalkTick(){"
              "const hist=document.getElementById('led-gen-history');"
              "const histItems=hist?[...hist.querySelectorAll('.gen-hist-item')].filter(e=>e._data):[];"
              "const total=_walkPresets.length+histItems.length;"
              "if(total===0)return;"
              "const idx=Math.floor(Math.random()*total);"
              "const ws=document.getElementById('led-walk-status');"
              "if(idx<_walkPresets.length){"
              "const p=_walkPresets[idx];"
              "if(ws)ws.setAttribute('data-label',p);"
              "await sendLed(p);"
              "}else{"
              "const item=histItems[idx-_walkPresets.length];"
              "const lbl=item._data.pattern+(item._data.description?' — '+item._data.description:'');"
              "if(ws)ws.setAttribute('data-label',lbl);"
              "await ledGenApply(item._data,item);}"
              "walkCdStart();}"
              "function ledWalkToggle(){"
              "const btn=document.getElementById('led-walk-btn');"
              "_walkActive=!_walkActive;"
              "if(_walkActive){"
              "btn.textContent='Walk Mode: ON';btn.style.background='#e44';"
              "ledWalkTick();"
              "_walkTimer=setInterval(ledWalkTick,30000);"
              "}else{"
              "if(_walkTimer){clearInterval(_walkTimer);_walkTimer=null;}"
              "if(_walkCdTimer){clearInterval(_walkCdTimer);_walkCdTimer=null;}"
              "btn.textContent='Walk Mode: OFF';btn.style.background='';"
              "const ws=document.getElementById('led-walk-status');"
              "ws.textContent='';ws.removeAttribute('data-label');}}"
              "async function ledOff(){"
              "setLedStatus('OFF');"
              "try{await fetch('/led',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({off:true})});"
              "}catch(e){setLedStatus('Error: '+e.message);}}"
              "window.addEventListener('DOMContentLoaded',()=>{"
              "openTab('tab-security');"
              "initServoPad();"
              "initColorMap();"
              "const speakText=document.getElementById('speak-text');"
              "if(speakText){speakText.addEventListener('keydown',e=>{"
              "if(e.key==='Enter'&&!e.shiftKey&&!e.isComposing){e.preventDefault();sendSpeak();}"
              "});}"
              "});"
              "</script></head><body>"
              "<h2>StackChan Settings</h2>"
              "<div class=tabs>"
              "<button type=button class=tab-btn data-tab=tab-security onclick=\"openTab('tab-security')\">Security</button>"
              "<button type=button class=tab-btn data-tab=tab-speak onclick=\"openTab('tab-speak')\">Speak</button>"
              "<button type=button class=tab-btn data-tab=tab-servo onclick=\"openTab('tab-servo')\">Servo</button>"
              "<button type=button class=tab-btn data-tab=tab-bpm onclick=\"openTab('tab-bpm')\">BPM-Dance</button>"
              "<button type=button class=tab-btn data-tab=tab-led onclick=\"openTab('tab-led')\">LED</button>"
              "<button type=button class=tab-btn data-tab=tab-general onclick=\"openTab('tab-general')\">General</button>"
              "<button type=button class=tab-btn data-tab=tab-periodic onclick=\"openTab('tab-periodic')\">Periodic</button>"
              "</div>"
              "<form method=post action=/config>");
    html += F("<div id=tab-security class=\"tab active\">");
    html += F("<div class=sec><h3>WiFi</h3>");
    html += F("<div class=net-info><div><b>Connected</b>");
    html += current_ssid.isEmpty() ? F("not connected") : current_ssid;
    html += F("</div><div><b>IP</b>");
    html += html_escape(current_ip);
    html += F("</div></div><p>Try entries from top to bottom.</p>");
    for (size_t i = 0; i < WIFI_MAX; ++i) {
        String idx = String(i + 1);
        String ssid = i < wifi_credential_count ? html_escape(wifi_credentials[i].ssid) : "";
        String wpass = i < wifi_credential_count ? html_escape(wifi_credentials[i].password) : "";
        html += "<label>SSID " + idx + "<input type=text name=wifi" + idx + "_ssid value=\"";
        html += ssid;
        html += "\"></label>";
        html += "<label>Password " + idx + "<input type=password name=wifi" + idx + "_pass value=\"";
        html += wpass;
        html += "\"></label>";
    }
    html += F("</div>");
    html += F("<div class=sec><h3>API Keys</h3>"
              "<label>STT (Google)<input type=text name=stt_key value=\""); html += sttk; html += F("\"></label>"
              "<label>TTS (tts.quest)<input type=text name=tts_key value=\""); html += ttsk; html += F("\"></label>"
              "</div>");
    html += F("<div class=sec><h3>VOICEVOX (optional)</h3>"
              "<label>Host (IP:port)<input type=text name=vv_host value=\""); html += html_escape(voicevox_host); html += F("\"></label>"
              "<label>Speaker ID<input type=number name=vv_spk value=\""); html += String(voicevox_speaker); html += F("\"></label>"
              "</div>");
    html += F("<div class=sec><h3>Hermes (LLM)</h3>"
              "<label>Endpoint<input type=text name=h_ep value=\""); html += html_escape(hermes_endpoint); html += F("\"></label>"
              "<label>Model<input type=text name=h_model value=\""); html += html_escape(hermes_model); html += F("\"></label>"
              "<label>API Key<input type=text name=h_key value=\""); html += html_escape(hermes_api_key); html += F("\"></label>"
              "<label>Max Tokens<input type=number name=h_tokens value=\""); html += String(hermes_max_tokens); html += F("\"></label>"
              "<label>Timeout (ms)<input type=number name=h_timeout value=\""); html += String(hermes_timeout_ms); html += F("\"></label>"
              "</div>");
    html += F("</div>");
    html += F("<div id=tab-speak class=tab>");
    html += F("<div class=sec><h3>Speak</h3>"
              "<div class=speak-row>"
              "<label><input type=radio name=speak_mode value=fixed checked> Fixed</label>"
              "<label><input type=radio name=speak_mode value=llm> LLM</label>"
              "</div>"
              "<div id=chat-log class=chat-log></div>"
              "<label>Text / Prompt<textarea id=speak-text placeholder=\"Text to speak or a prompt for LLM\"></textarea></label>"
              "<div class=speak-row><button type=button class=mini-btn onclick=sendSpeak()>Send</button><span id=speak-status></span></div>"
              "<p>LLM mode sends the chat history shown in this tab as context.</p>"
              "<h3 style=\"margin-top:14px\">History</h3>"
              "<div id=speak-history></div>"
              "</div>");
    html += F("</div>");
    html += F("<div id=tab-servo class=tab>");
    html += F("<div class=sec><h3>Servo Jog</h3>"
              "<div class=servo-grid>"
              "<div id=jog-pad class=jog-pad>"
              "<div class=\"jog-axis h\"></div><div class=\"jog-axis v\"></div><div id=jog-dot class=jog-dot></div>"
              "</div>"
              "<div class=servo-readout>"
              "<div>Yaw <b><span id=servo-yaw>0</span></b></div>"
              "<div>Roll <b><span id=servo-roll>0</span></b></div>"
              "</div>"
              "<label>Move ms<input id=servo-ms type=number min=20 max=1000 value=40></label>"
              "<div class=speak-row>"
              "<button type=button class=mini-btn onclick=\"sendServo(0,0,true).then(()=>{setJogDot(0,0);document.getElementById('servo-status').textContent='Centered';}).catch(err=>document.getElementById('servo-status').textContent='Error: '+err.message)\">Center</button>"
              "<span id=servo-status></span>"
              "</div>"
              "</div></div>");
    html += F("</div>");
    html += F("<div id=tab-bpm class=tab>");
    html += F("<div class=sec><h3>BPM-Dance</h3>"
              "<label>Play BPM<input type=number min=60 max=180 name=play_bpm value=\""); html += String(play_bpm); html += F("\"></label>"
              "</div>");
    html += F("</div>");
    // --- LED tab ---
    html += F("<div id=tab-led class=tab>"
              "<div class=sec>"
              "<div style=\"display:flex;align-items:center;gap:10px\">"
              "<button type=button class=mini-btn id=led-walk-btn onclick=ledWalkToggle()>Walk Mode: OFF</button>"
              "<span id=led-walk-status style=\"font-size:13px;color:#888\"></span>"
              "</div></div>"
              "<div class=sec><h3>Presets</h3>"
              "<div style=\"display:flex;flex-wrap:wrap;gap:6px\">"
              "<button type=button class=mini-btn onclick=\"sendLed('warm_white')\">Warm White</button>"
              "<button type=button class=mini-btn onclick=\"sendLed('cool_blue')\">Cool Blue</button>"
              "<button type=button class=mini-btn onclick=\"sendLed('alert_red')\">Alert Red</button>"
              "<button type=button class=mini-btn onclick=\"sendLed('calm_breath')\">Calm Breath</button>"
              "<button type=button class=mini-btn onclick=\"sendLed('christmas')\">Christmas</button>"
              "<button type=button class=mini-btn onclick=\"sendLed('ocean')\">Ocean</button>"
              "<button type=button class=mini-btn onclick=\"sendLed('rainbow')\">Rainbow</button>"
              "<button type=button class=mini-btn onclick=\"sendLed('party')\">Party</button>"
              "<button type=button class=mini-btn onclick=ledOff() style=\"background:#888\">OFF</button>"
              "</div></div>"
              "<div class=sec><h3>LLM Generate</h3>"
              "<div style=\"display:flex;gap:8px;align-items:center;flex-wrap:wrap\">"
              "<input type=text id=led-gen-theme placeholder=\"例: うれしい気持ち\" style=\"flex:1;min-width:120px;padding:6px 8px;border:1px solid #ccc;border-radius:4px;font-size:14px\">"
              "<button type=button class=mini-btn onclick=ledGen()>Gen</button>"
              "<span id=led-gen-status></span>"
              "</div>"
              "<div id=led-gen-history style=\"margin:6px 0\"></div>"
              "</div>"
              "<div class=sec><h3>Custom</h3>"
              "<label>Pattern<select id=led-pattern onchange=\"ledPatChg();ledRtSend()\">"
              "<option value=solid>Solid</option>"
              "<option value=blink>Blink</option>"
              "<option value=breath>Breath</option>"
              "<option value=chase>Chase</option>"
              "<option value=rainbow>Rainbow</option>"
              "<option value=wave>Wave</option>"
              "<option value=split>Split (2-color)</option>"
              "<option value=split_blink>Split + Blink</option>"
              "<option value=split_breath>Split + Breath</option>"
              "<option value=split_wave>Split + Wave</option>"
              "<option value=split_chase>Split + Chase</option>"
              "<option value=alternate>Alternate (2-color)</option>"
              "</select></label>"
              "<div style=\"margin:8px 0\"><b>Color 1</b>"
              "<canvas id=led-cmap width=280 height=120 style=\"display:block;margin:6px 0;border-radius:6px;border:1px solid #ccc;cursor:crosshair;touch-action:none\"></canvas>"
              "<div style=\"display:flex;align-items:center;gap:8px\"><span style=\"display:inline-block;width:28px;height:28px;border-radius:4px;border:1px solid #999\" id=led-cswatch></span>"
              "<span id=led-cval>#FF8800</span></div>"
              "<input type=hidden id=led-color value=\"#FF8800\">"
              "</div>"
              "<div id=led-c2-wrap style=\"margin:8px 0;display:none\"><b>Color 2</b>"
              "<canvas id=led-cmap2 width=280 height=120 style=\"display:block;margin:6px 0;border-radius:6px;border:1px solid #ccc;cursor:crosshair;touch-action:none\"></canvas>"
              "<div style=\"display:flex;align-items:center;gap:8px\"><span style=\"display:inline-block;width:28px;height:28px;border-radius:4px;border:1px solid #999\" id=led-cswatch2></span>"
              "<span id=led-cval2>#FFFFFF</span></div>"
              "<input type=hidden id=led-color2 value=\"#FFFFFF\">"
              "</div>"
              "<label>Speed &nbsp;<span class=range-val id=led-spd-lbl>5</span>"
              "<input type=range min=1 max=10 value=5 id=led-speed "
              "oninput=\"document.getElementById('led-spd-lbl').textContent=this.value;ledRtSend()\"></label>"
              "<div style=\"display:flex;gap:8px;align-items:center;margin:8px 0\">"
              "<button type=button class=mini-btn onclick=applyLed()>Apply</button>"
              "<label style=\"display:flex;align-items:center;gap:6px;margin:0;cursor:pointer\">"
              "<input type=checkbox id=led-rt onchange=\"this.parentElement.style.color=this.checked?'#e44':'#333'\"> Realtime</label>"
              "</div>"
              "</div>"
              "<div class=sec><span id=led-status></span></div>"
              "</div>");
    html += F("<div id=tab-general class=tab>");
    html += F("<div class=sec><h3>General</h3>"
              "<label>TTS Volume &nbsp;<span class=range-val id=vol_lbl>");
    html += String(tts_volume);
    html += F("</span><input type=range min=0 max=255 name=tts_volume value=\"");
    html += String(tts_volume);
    html += F("\" oninput=\"document.getElementById('vol_lbl').textContent=this.value\"></label>"
              "<label>Brightness &nbsp;<span class=range-val id=bri_lbl>");
    html += String(brightness_val);
    html += F("</span><input type=range min=20 max=255 name=brightness value=\"");
    html += String(brightness_val);
    html += F("\" oninput=\"document.getElementById('bri_lbl').textContent=this.value\"></label>"
              "<label class=check><input type=checkbox name=servo_on_boot value=1");
    if (servo_on_boot) html += F(" checked");
    html += F(">Servo ON at boot</label>"
              "</div>");
    html += F("<div class=sec><h3>Button Assignments</h3>");
    html += F("<label class=check><input type=checkbox name=btn_llm_tap value=1");
    if (btn_llm_tap)       html += F(" checked");
    html += F(">Left tap 窶・LLM test speak</label>");
    html += F("<label class=check><input type=checkbox name=btn_periodic_hold value=1");
    if (btn_periodic_hold) html += F(" checked");
    html += F(">Left hold 窶・Periodic mode toggle</label>");
    html += F("<label class=check><input type=checkbox name=btn_stt_tap value=1");
    if (btn_stt_tap)       html += F(" checked");
    html += F(">Center tap 窶・Push-to-Talk (STT)</label>");
    html += F("<label class=check><input type=checkbox name=btn_servo_tap value=1");
    if (btn_servo_tap)     html += F(" checked");
    html += F(">Right tap 窶・Servo idle toggle</label>");
    html += F("<label class=check><input type=checkbox name=btn_bpm_hold value=1");
    if (btn_bpm_hold)      html += F(" checked");
    html += F(">Right hold 窶・BPM dance mode</label>");
    html += F("</div>");
    html += F("</div>");
    // --- Periodic tab ---
    html += F("<div id=tab-periodic class=tab>");
    html += F("<div class=sec><h3>Periodic Speech</h3>"
              "<label class=check><input type=checkbox name=periodic_enabled value=1");
    if (periodic_mode_enabled) html += F(" checked");
    html += F(">Global Enable (Left hold to toggle on device)</label></div>");
    html.reserve(html.length() + 2048);
    for (int i = 0; i < PERIODIC_MAX; i++) {
        String pi = String(i);
        html += "<div class=sec><h3>Entry " + String(i + 1) + "</h3>";
        html += "<label class=check><input type=checkbox name=p" + pi + "_enabled value=1";
        if (periodic_entries[i].enabled) html += " checked";
        html += ">Enabled</label>";
        html += "<label style=\"margin-top:8px;font-weight:bold\">Type&nbsp;&nbsp;"
                "<input type=radio name=p" + pi + "_type value=fixed";
        if (!periodic_entries[i].is_llm) html += " checked";
        html += "> Fixed&nbsp;&nbsp;"
                "<input type=radio name=p" + pi + "_type value=llm";
        if (periodic_entries[i].is_llm) html += " checked";
        html += "> LLM</label>";
        html += "<label>Message / Prompt<input type=text name=p" + pi + "_msg value=\"";
        html += html_escape(periodic_entries[i].message);
        html += "\"></label>";
        html += "<label>Interval (min)<input type=number min=1 max=1440 name=p" + pi + "_min value=\"";
        html += String((int)(periodic_entries[i].interval_ms / 60000UL));
        html += "\"></label></div>";
    }
    html += F("</div>");
    html += F("<button id=save-btn class=btn type=submit>Save &amp; Restart</button>"
              "</form></body></html>");

    client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: ");
    client.print(html.length());
    client.print("\r\nConnection: close\r\n\r\n");
    client.print(html);
    client.flush();
    client.stop();
}

static void handle_config_post(WiFiClient& client, int content_length) {
    String body = "";
    uint32_t deadline = millis() + 3000;
    while ((int)body.length() < content_length && millis() < deadline) {
        if (client.available()) body += (char)client.read();
    }

    String msg = F("<html><body><h2>Saved! Restarting...</h2><p>Reconnect in a few seconds.</p></body></html>");
    client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: ");
    client.print(msg.length());
    client.print("\r\nConnection: close\r\n\r\n");
    client.print(msg);
    client.flush();
    client.stop();
    delay(200);

    String stt_key   = form_field(body, "stt_key");
    String tts_key   = form_field(body, "tts_key");
    String vv_host   = form_field(body, "vv_host");
    String vv_spk    = form_field(body, "vv_spk");
    String h_ep      = form_field(body, "h_ep");
    String h_model   = form_field(body, "h_model");
    String h_key     = form_field(body, "h_key");
    String h_tokens  = form_field(body, "h_tokens");
    String h_timeout = form_field(body, "h_timeout");
    String bpm_play  = form_field(body, "play_bpm");
    String volume    = form_field(body, "tts_volume");
    String bright    = form_field(body, "brightness");
    bool servo_boot  = form_has_checked(body, "servo_on_boot");

    File f = SPIFFS.open("/yaml/SC_SecConfig.yaml", "r");
    if (!f) { M5_LOGE("config_post: open failed"); ESP.restart(); return; }
    String yaml = f.readString();
    f.close();

    WifiCredential new_entries[WIFI_MAX];
    size_t new_entry_count = 0;
    for (size_t i = 0; i < WIFI_MAX; ++i) {
        String idx = String(i + 1);
        String ssid = form_field(body, ("wifi" + idx + "_ssid").c_str());
        String pass = form_field(body, ("wifi" + idx + "_pass").c_str());
        ssid.trim();
        if (ssid.isEmpty()) continue;
        new_entries[new_entry_count].ssid = ssid;
        new_entries[new_entry_count].password = pass;
        new_entry_count++;
    }
    replace_wifi_yaml_sections(yaml, new_entries, new_entry_count);
    if (!stt_key.isEmpty())   update_yaml_quoted_in_section(yaml, "apikey",  "    stt: \"",     stt_key);
    if (!tts_key.isEmpty())   update_yaml_quoted_in_section(yaml, "apikey",  "    tts: \"",     tts_key);
    update_yaml_quoted_str(yaml,                                             "voicevox_host: \"", vv_host);
    if (!h_ep.isEmpty())      update_yaml_quoted_in_section(yaml, "hermes",  "  endpoint: \"",  h_ep);
    if (!h_model.isEmpty())   update_yaml_quoted_in_section(yaml, "hermes",  "  model: \"",     h_model);
    if (!h_key.isEmpty())     update_yaml_quoted_in_section(yaml, "hermes",  "  api_key: \"",   h_key);
    if (!vv_spk.isEmpty())    update_yaml_int_value(yaml, "voicevox_speaker", vv_spk.toInt());
    if (!h_tokens.isEmpty())  update_yaml_int_value(yaml, "hermes_max_tokens", h_tokens.toInt());
    if (!h_timeout.isEmpty()) update_yaml_int_value(yaml, "hermes_timeout_ms", h_timeout.toInt());
    if (!bpm_play.isEmpty())  update_yaml_int_value(yaml, "play_bpm", clamp_bpm_value(bpm_play.toInt()));
    if (!volume.isEmpty())    update_yaml_int_value(yaml, "tts_volume", constrain(volume.toInt(), 0, 255));
    if (!bright.isEmpty())    update_yaml_int_value(yaml, "brightness", constrain(bright.toInt(), 20, 255));
    update_yaml_int_value(yaml, "servo_on_boot", servo_boot ? 1 : 0);

    update_yaml_int_value(yaml, "btn_llm_tap",       form_has_checked(body, "btn_llm_tap")       ? 1 : 0);
    update_yaml_int_value(yaml, "btn_periodic_hold",  form_has_checked(body, "btn_periodic_hold")  ? 1 : 0);
    update_yaml_int_value(yaml, "btn_stt_tap",        form_has_checked(body, "btn_stt_tap")        ? 1 : 0);
    update_yaml_int_value(yaml, "btn_servo_tap",      form_has_checked(body, "btn_servo_tap")      ? 1 : 0);
    update_yaml_int_value(yaml, "btn_bpm_hold",       form_has_checked(body, "btn_bpm_hold")       ? 1 : 0);

    bool periodic_en = form_has_checked(body, "periodic_enabled");
    update_yaml_int_value(yaml, "periodic_enabled", periodic_en ? 1 : 0);
    for (int i = 0; i < PERIODIC_MAX; i++) {
        String pre = "p" + String(i) + "_";
        bool pen   = form_has_checked(body, (pre + "enabled").c_str());
        String ptype = form_field(body, (pre + "type").c_str());
        String pmsg  = form_field(body, (pre + "msg").c_str());
        String pmin  = form_field(body, (pre + "min").c_str());
        update_yaml_int_value(yaml, (pre + "enabled").c_str(), pen ? 1 : 0);
        if (!ptype.isEmpty()) upsert_yaml_quoted(yaml, pre + "type", ptype);
        upsert_yaml_quoted(yaml, pre + "msg", pmsg);
        if (!pmin.isEmpty()) update_yaml_int_value(yaml, (pre + "min").c_str(), constrain(pmin.toInt(), 1, 1440));
    }

    File fw = SPIFFS.open("/yaml/SC_SecConfig.yaml", "w");
    if (fw) { fw.print(yaml); fw.close(); M5_LOGI("Config saved, restarting"); }
    delay(300);
    ESP.restart();
}

static void handle_udp_beat() {
    int size = udp_beat.parsePacket();
    if (size <= 0) return;
    char buf[256];
    int len = udp_beat.read(buf, sizeof(buf) - 1);
    if (len <= 0) return;
    buf[len] = '\0';
    if (app_mode == MODE_SETTINGS || busy) return;
    JsonDocument doc;
    if (deserializeJson(doc, buf)) return;
    queue_http_beat(doc["step"] | 0, doc["accent"] | false, doc["bpm"] | 0, doc["motion"] | "lr");
}

// Beat commands over USB serial (Web Serial), jitter-free alternative to /beat.
// Line protocol (one command per line):
//   B <step> <accent 0|1> <motion lr|ud> <bpm>   e.g. "B 12 1 lr 117"
//   C                                            center servo to front
static void serial_beat_tick() {
    static char line[96];
    static uint8_t pos = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (pos == 0) continue;
            line[pos] = '\0';
            pos = 0;
            if (app_mode == MODE_SETTINGS || busy) continue;
            if (line[0] == 'B') {
                int step = 0, accent = 0, bpm = 0;
                char motion[4] = "lr";
                if (sscanf(line, "B %d %d %3s %d", &step, &accent, motion, &bpm) >= 3) {
                    queue_http_beat(step, accent != 0, bpm, String(motion));
                }
            } else if (line[0] == 'C') {
                if (servo_ready) servo_web_center(300);
            }
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = c;
        } else {
            pos = 0;  // overflow: drop the line
        }
    }
}

void handle_speak_server() {
    servo_ws_tick();

    WiFiClient client = speak_server.available();
    if (!client) return;

    String req_line = client.readStringUntil('\n');
    req_line.trim();
    bool is_get_root    = req_line.startsWith("GET / ");
    bool is_get_servo_ws = req_line.startsWith("GET /servo-ws");
    bool is_post_beat   = req_line.startsWith("POST /beat");
    bool is_post_servo  = req_line.startsWith("POST /servo");
    bool is_post_speak  = req_line.startsWith("POST /speak");
    bool is_post_config = req_line.startsWith("POST /config");
    bool is_post_led_gen = req_line.startsWith("POST /led-gen");
    bool is_post_led    = !is_post_led_gen && req_line.startsWith("POST /led");

    int content_length = 0;
    String ws_key = "";
    while (client.connected()) {
        String line = client.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) break;
        String lower = line; lower.toLowerCase();
        if (lower.startsWith("content-length:"))
            content_length = line.substring(line.indexOf(':') + 1).toInt();
        if (lower.startsWith("sec-websocket-key:"))
            ws_key = line.substring(line.indexOf(':') + 1);
    }
    ws_key.trim();

    if (is_get_root)    { handle_config_get(client);            return; }
    if (is_get_servo_ws) {
        if (ws_key.isEmpty()) {
            client.print("HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
            client.stop();
            return;
        }
        servo_ws_stop();
        String accept = websocket_accept_key(ws_key);
        client.print("HTTP/1.1 101 Switching Protocols\r\n");
        client.print("Upgrade: websocket\r\n");
        client.print("Connection: Upgrade\r\n");
        client.print("Sec-WebSocket-Accept: ");
        client.print(accept);
        client.print("\r\n\r\n");
        servo_ws_client = client;
        servo_ws_active = true;
        servo_ws_last_ms = millis();
        return;
    }
    if (is_post_config) { handle_config_post(client, content_length); return; }
    if (is_post_servo) {
        String body = "";
        body.reserve(content_length > 0 ? content_length + 1 : 128);
        uint32_t deadline = millis() + 1000;
        while ((int)body.length() < content_length && millis() < deadline) {
            while (client.available() && (int)body.length() < content_length) {
                body += (char)client.read();
                deadline = millis() + 200;
            }
            delay(1);
        }

        if (!servo_ready) {
            String err = "{\"ok\":false,\"error\":\"servo\"}";
            client.print("HTTP/1.1 503 Service Unavailable\r\nContent-Type: application/json\r\nContent-Length: ");
            client.print(err.length());
            client.print("\r\nConnection: close\r\n\r\n");
            client.print(err);
            client.stop();
            return;
        }
        if (busy) {
            client.print("HTTP/1.1 503 Service Unavailable\r\nContent-Type: application/json\r\nContent-Length: 13\r\nConnection: close\r\n\r\n{\"busy\":true}");
            client.stop();
            return;
        }

        JsonDocument doc;
        DeserializationError json_error = deserializeJson(doc, body);
        if (json_error) {
            String err = "{\"ok\":false,\"error\":\"json\"}";
            client.print("HTTP/1.1 400 Bad Request\r\nContent-Type: application/json\r\nContent-Length: ");
            client.print(err.length());
            client.print("\r\nConnection: close\r\n\r\n");
            client.print(err);
            client.stop();
            return;
        }

        int move_ms = doc["ms"] | 180;
        bool center = doc["center"] | false;
        if (center) {
            servo_web_center(move_ms);
        } else {
            int yaw = constrain((int)(doc["x"] | 0), -45, 45);
            int roll = constrain((int)(doc["y"] | 0), -35, 35);
            servo_web_move(srv_cx + yaw, srv_cy - roll, move_ms);
        }

        String res = "{\"ok\":true,\"x\":";
        res += String(servo_web_x - srv_cx);
        res += ",\"y\":";
        res += String(srv_cy - servo_web_y);
        res += ",\"target_x\":";
        res += String(servo_web_x);
        res += ",\"target_y\":";
        res += String(servo_web_y);
        res += "}";
        client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: ");
        client.print(res.length());
        client.print("\r\nConnection: close\r\n\r\n");
        client.print(res);
        client.stop();
        return;
    }
    if (is_post_beat) {
        String body = "";
        uint32_t deadline = millis() + 1000;
        while ((int)body.length() < content_length && millis() < deadline) {
            if (client.available()) body += (char)client.read();
        }

        JsonDocument doc;
        int step_index = 0;
        bool accent = false;
        int bpm_hint = 0;
        String motion = "lr";
        if (!deserializeJson(doc, body)) {
            step_index = doc["step"] | 0;
            accent = doc["accent"] | false;
            bpm_hint = doc["bpm"] | 0;
            motion = doc["motion"] | "lr";
        }

        if (app_mode == MODE_SETTINGS || busy) {
            client.print("HTTP/1.1 503 Service Unavailable\r\nContent-Type: application/json\r\nContent-Length: 13\r\nConnection: close\r\n\r\n{\"busy\":true}");
            client.stop();
            return;
        }

        queue_http_beat(step_index, accent, bpm_hint, motion);
        client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 42\r\nConnection: close\r\n\r\n{\"ok\":true,\"queued\":true,\"mode\":\"beat\"}");
        client.stop();
        return;
    }

    if (is_post_led_gen) {
        // Read request body for optional theme
        String led_gen_body = "";
        uint32_t lg_deadline = millis() + 1000;
        while ((int)led_gen_body.length() < content_length && millis() < lg_deadline) {
            if (client.available()) led_gen_body += (char)client.read();
        }
        String led_gen_theme = "";
        {
            JsonDocument lg_doc;
            if (!deserializeJson(lg_doc, led_gen_body)) {
                led_gen_theme = lg_doc["theme"] | "";
            }
        }
        // Call LLM to generate a random LED pattern
        if (hermes_endpoint.isEmpty()) {
            String err = "{\"ok\":false,\"error\":\"LLM not configured\"}";
            client.print("HTTP/1.1 503 Service Unavailable\r\nContent-Type: application/json\r\nContent-Length: ");
            client.print(err.length());
            client.print("\r\nConnection: close\r\n\r\n");
            client.print(err);
            client.stop();
            return;
        }
        String prompt = "Generate a creative LED lighting pattern for a small robot with a 12-LED ring. "
            "Reply ONLY with a JSON object (no markdown, no explanation outside JSON). Format:\n"
            "{\"pattern\":\"<name>\",\"color\":\"#RRGGBB\",\"color2\":\"#RRGGBB\",\"speed\":1-10,"
            "\"description\":\"one-line description in Japanese\"}\n\n"
            "== Single-color patterns (use color only, do NOT include color2) ==\n"
            "  solid: all LEDs lit with one color\n"
            "  blink: all LEDs blink on/off\n"
            "  breath: all LEDs fade in and out smoothly\n"
            "  chase: a dot runs around the ring\n"
            "  rainbow: full rainbow cycles around the ring (color is ignored)\n"
            "  wave: brightness ripples around the ring\n\n"
            "== Two-color patterns (MUST include color2) ==\n"
            "  split: static, left half=color, right half=color2\n"
            "  split_blink: split halves blinking on/off\n"
            "  split_breath: split halves breathing\n"
            "  split_wave: split halves with wave animation\n"
            "  split_chase: split halves with chase dots\n"
            "  alternate: even LEDs=color, odd=color2, rotating\n\n"
            "IMPORTANT RULES:\n"
            "- You MUST vary the pattern. Do NOT always pick the same one.\n"
            "- Pick randomly from ALL 12 patterns above, not just two-color ones.\n"
            "- Single-color patterns (solid/blink/breath/chase/rainbow/wave) are equally valid choices.\n"
            "- Only use two-color patterns when the theme specifically calls for multiple colors.\n";
        if (!led_gen_theme.isEmpty()) {
            prompt += "Theme/mood requested by user: " + led_gen_theme + "\n"
                      "Choose colors and pattern that match this theme.\n";
        }
        prompt += "Be creative and varied. Pick interesting color combinations and patterns.";
        String llm_reply = call_hermes(prompt);
        // Extract JSON from reply (LLM may wrap in markdown code block)
        int json_start = llm_reply.indexOf('{');
        int json_end = llm_reply.lastIndexOf('}');
        if (json_start < 0 || json_end <= json_start) {
            String err = "{\"ok\":false,\"error\":\"LLM returned no JSON\",\"raw\":\"" + llm_reply.substring(0, 100) + "\"}";
            client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: ");
            client.print(err.length());
            client.print("\r\nConnection: close\r\n\r\n");
            client.print(err);
            client.stop();
            return;
        }
        String json_str = llm_reply.substring(json_start, json_end + 1);
        // Validate it parses
        JsonDocument gen_doc;
        if (deserializeJson(gen_doc, json_str)) {
            String err = "{\"ok\":false,\"error\":\"parse\",\"raw\":\"" + json_str.substring(0, 100) + "\"}";
            client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: ");
            client.print(err.length());
            client.print("\r\nConnection: close\r\n\r\n");
            client.print(err);
            client.stop();
            return;
        }
        // Build response with ok:true merged
        String res = "{\"ok\":true,\"pattern\":\"";
        res += gen_doc["pattern"] | "solid";
        res += "\",\"color\":\"";
        res += gen_doc["color"] | "#FFFFFF";
        res += "\",\"speed\":";
        res += String((int)(gen_doc["speed"] | 5));
        const char* gen_color2 = gen_doc["color2"];
        if (gen_color2) {
            res += ",\"color2\":\"";
            res += gen_color2;
            res += "\"";
        }
        res += ",\"description\":\"";
        String desc = gen_doc["description"] | "";
        desc.replace("\"", "'");
        desc.replace("\\", "");
        res += desc;
        res += "\"}";
        client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: ");
        client.print(res.length());
        client.print("\r\nConnection: close\r\n\r\n");
        client.print(res);
        client.stop();
        return;
    }

    if (is_post_led) {
        String body = "";
        uint32_t deadline = millis() + 1000;
        while ((int)body.length() < content_length && millis() < deadline) {
            if (client.available()) body += (char)client.read();
        }
        JsonDocument doc;
        if (deserializeJson(doc, body)) {
            client.print("HTTP/1.1 400 Bad Request\r\nContent-Type: application/json\r\nContent-Length: 25\r\nConnection: close\r\n\r\n{\"ok\":false,\"error\":\"json\"}");
            client.stop();
            return;
        }
        String res_pattern = "off";
        String res_color = "#000000";
        if (doc["off"] | false) {
            led_set(LED_OFF);
            led_force_clear();
        } else if (doc["preset"].is<const char*>()) {
            String preset = doc["preset"].as<String>();
            if (preset == "warm_white")     { manual_pattern = PAT_SOLID; manual_r = 255; manual_g = 200; manual_b = 120; }
            else if (preset == "cool_blue") { manual_pattern = PAT_SOLID; manual_r = 0; manual_g = 100; manual_b = 255; }
            else if (preset == "alert_red") { manual_pattern = PAT_BLINK; manual_r = 255; manual_g = 0; manual_b = 0; manual_on_ms = 300; manual_off_ms = 300; }
            else if (preset == "calm_breath") { manual_pattern = PAT_BREATH; manual_r = 0; manual_g = 120; manual_b = 200; manual_speed = 5; }
            else if (preset == "christmas") { manual_pattern = PAT_ALTERNATE; manual_r = 255; manual_g = 0; manual_b = 0; manual_r2 = 0; manual_g2 = 200; manual_b2 = 0; manual_speed = 7; }
            else if (preset == "ocean")     { manual_pattern = PAT_WAVE; manual_r = 0; manual_g = 80; manual_b = 200; manual_speed = 5; }
            else if (preset == "rainbow")   { manual_pattern = PAT_RAINBOW; manual_speed = 5; }
            else if (preset == "party")     { manual_pattern = PAT_CHASE; manual_r = (uint8_t)random(100, 256); manual_g = (uint8_t)random(100, 256); manual_b = (uint8_t)random(100, 256); manual_speed = 9; }
            led_set(LED_MANUAL);
            const char* pat_names[] = {"solid","blink","breath","chase","rainbow","wave","split","alternate","split_blink","split_breath","split_wave","split_chase"};
            res_pattern = pat_names[manual_pattern];
            char hex[8]; snprintf(hex, sizeof(hex), "#%02X%02X%02X", (int)manual_r, (int)manual_g, (int)manual_b);
            res_color = hex;
        } else {
            String pat = doc["pattern"] | "solid";
            if (pat == "solid")      manual_pattern = PAT_SOLID;
            else if (pat == "blink") manual_pattern = PAT_BLINK;
            else if (pat == "breath") manual_pattern = PAT_BREATH;
            else if (pat == "chase") manual_pattern = PAT_CHASE;
            else if (pat == "rainbow") manual_pattern = PAT_RAINBOW;
            else if (pat == "wave")  manual_pattern = PAT_WAVE;
            else if (pat == "split") manual_pattern = PAT_SPLIT;
            else if (pat == "alternate") manual_pattern = PAT_ALTERNATE;
            else if (pat == "split_blink") manual_pattern = PAT_SPLIT_BLINK;
            else if (pat == "split_breath") manual_pattern = PAT_SPLIT_BREATH;
            else if (pat == "split_wave") manual_pattern = PAT_SPLIT_WAVE;
            else if (pat == "split_chase") manual_pattern = PAT_SPLIT_CHASE;
            String color_str = doc["color"] | "#FFFFFF";
            if (color_str.length() == 7 && color_str[0] == '#') {
                manual_r = (uint8_t)strtol(color_str.substring(1, 3).c_str(), nullptr, 16);
                manual_g = (uint8_t)strtol(color_str.substring(3, 5).c_str(), nullptr, 16);
                manual_b = (uint8_t)strtol(color_str.substring(5, 7).c_str(), nullptr, 16);
            }
            String color2_str = doc["color2"] | "";
            if (color2_str.length() == 7 && color2_str[0] == '#') {
                manual_r2 = (uint8_t)strtol(color2_str.substring(1, 3).c_str(), nullptr, 16);
                manual_g2 = (uint8_t)strtol(color2_str.substring(3, 5).c_str(), nullptr, 16);
                manual_b2 = (uint8_t)strtol(color2_str.substring(5, 7).c_str(), nullptr, 16);
            }
            manual_speed = doc["speed"] | 5;
            if (manual_speed < 1) manual_speed = 1;
            if (manual_speed > 10) manual_speed = 10;
            led_set(LED_MANUAL);
            res_pattern = pat;
            res_color = color_str;
        }
        if (res_pattern != "off" && !busy && avatar_ready) {
            avatar.setExpression(Expression::Happy);
            avatar.setMouthOpenRatio(0.4f);
            delay(1500);
            avatar.setMouthOpenRatio(0.0f);
            avatar.setExpression(Expression::Neutral);
        }
        String res = "{\"ok\":true,\"pattern\":\"" + res_pattern + "\",\"color\":\"" + res_color + "\"}";
        client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: ");
        client.print(res.length());
        client.print("\r\nConnection: close\r\n\r\n");
        client.print(res);
        client.stop();
        return;
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
    body.reserve(content_length > 0 ? content_length + 1 : 256);
    uint32_t deadline = millis() + 5000;
    while ((content_length <= 0 || (int)body.length() < content_length) && millis() < deadline) {
        while (client.available() && (content_length <= 0 || (int)body.length() < content_length)) {
            body += (char)client.read();
            deadline = millis() + 500;
        }
        if (content_length <= 0 && !client.connected()) break;
        delay(1);
    }

    JsonDocument doc;
    String mode = "fixed";
    String speak_text = "";
    String reply_text = "";
    DeserializationError json_error = deserializeJson(doc, body);
    if (!json_error) {
        mode = doc["mode"] | "fixed";
        speak_text = doc["text"] | "";
    } else {
        String err = "{\"ok\":false,\"error\":\"json:";
        err += json_error.c_str();
        err += "\",\"bytes\":";
        err += String(body.length());
        err += ",\"content_length\":";
        err += String(content_length);
        err += "}";
        client.print("HTTP/1.1 400 Bad Request\r\nContent-Type: application/json\r\nContent-Length: ");
        client.print(err.length());
        client.print("\r\nConnection: close\r\n\r\n");
        client.print(err);
        client.flush();
        client.stop();
        return;
    }
    if (speak_text.isEmpty()) {
        String err = "{\"ok\":false,\"error\":\"text\",\"bytes\":";
        err += String(body.length());
        err += ",\"content_length\":";
        err += String(content_length);
        err += "}";
        client.print("HTTP/1.1 400 Bad Request\r\nContent-Type: application/json\r\nContent-Length: ");
        client.print(err.length());
        client.print("\r\nConnection: close\r\n\r\n");
        client.print(err);
        client.flush();
        client.stop();
        return;
    }

    uint32_t request_started_ms = millis();
    uint32_t llm_elapsed_ms = 0;
    uint32_t tts_elapsed_ms = 0;

    busy = true;
    bool spoken = false;
    bool llm_mode = mode == "llm";
    if (llm_mode) {
        chat_add(true, speak_text);
        avatar.setExpression(Expression::Doubt);
        led_set(LED_THINKING);
        show("Thinking...");
        uint32_t llm_started_ms = millis();
        reply_text = call_hermes(build_llm_chat_prompt(doc["history"], speak_text));
        llm_elapsed_ms = millis() - llm_started_ms;
        if (reply_text.startsWith("Error:") || reply_text.startsWith("Parse error") || reply_text.startsWith("No content")) {
            busy = false;
            String err = "{\"ok\":false,\"error\":\"" + json_escape(reply_text) + "\",\"llm_ms\":";
            err += String(llm_elapsed_ms);
            err += ",\"total_ms\":";
            err += String(millis() - request_started_ms);
            err += "}";
            client.print("HTTP/1.1 500 Internal Server Error\r\nContent-Type: application/json\r\nContent-Length: ");
            client.print(err.length());
            client.print("\r\nConnection: close\r\n\r\n");
            client.print(err);
            client.stop();
            show_error_state(reply_text);
            return;
        }
    } else {
        reply_text = speak_text;
    }

    String resp = "{\"ok\":true,\"mode\":\"" + json_escape(mode) + "\",\"reply\":\"" + json_escape(reply_text) + "\",\"spoken\":";
    resp += "false";
    resp += ",\"speaking\":true";
    resp += ",\"llm_ms\":";
    resp += String(llm_elapsed_ms);
    resp += ",\"tts_ms\":";
    resp += "0";
    resp += ",\"total_ms\":";
    resp += String(millis() - request_started_ms);
    resp += "}";
    client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: ");
    client.print(resp.length());
    client.print("\r\nConnection: close\r\n\r\n");
    client.print(resp);
    client.flush();
    client.stop();

    web_tts_text = reply_text;
    web_tts_pending = true;
}

static void process_web_tts_pending() {
    if (!web_tts_pending) return;
    String reply_text = web_tts_text;
    web_tts_text = "";
    web_tts_pending = false;

    avatar.setExpression(Expression::Happy);
    led_set(LED_SPEAKING);
    show("[Speaking...]");
    chat_add(false, reply_text);
    speech_scroll_start(reply_text);
    speech_scroll_run_to_end();
    uint32_t tts_started_ms = millis();
    bool spoken = call_tts(reply_text);
    speech_scroll_stop();
    uint32_t tts_elapsed_ms = millis() - tts_started_ms;
    M5_LOGI("Speak TTS done: spoken=%d tts_ms=%lu", spoken ? 1 : 0, (unsigned long)tts_elapsed_ms);
    if (spoken) {
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

static int current_wifi_credential_index() {
    if (wifi_credential_count == 0) return 0;
    if (WiFi.status() == WL_CONNECTED) {
        String current = WiFi.SSID();
        for (size_t i = 0; i < wifi_credential_count; ++i) {
            if (wifi_credentials[i].ssid == current) return (int)i;
        }
    }
    return 0;
}

static void move_selected_wifi_to_front() {
    if (setting_wifi_index <= 0 || (size_t)setting_wifi_index >= wifi_credential_count) return;
    WifiCredential selected = wifi_credentials[setting_wifi_index];
    for (int i = setting_wifi_index; i > 0; --i) {
        wifi_credentials[i] = wifi_credentials[i - 1];
    }
    wifi_credentials[0] = selected;
    setting_wifi_index = 0;
}

static void save_settings() {
    File f = SPIFFS.open("/yaml/SC_SecConfig.yaml", "r");
    if (!f) { M5_LOGE("save_settings: open failed"); return; }
    String yaml = f.readString();
    f.close();
    move_selected_wifi_to_front();
    replace_wifi_yaml_sections(yaml, wifi_credentials, wifi_credential_count);
    update_yaml_int_value(yaml, "tts_volume", setting_volume);
    update_yaml_int_value(yaml, "brightness", setting_brightness);
    update_yaml_int_value(yaml, "mavlink_attitude_servo", setting_attitude_servo_enabled ? 1 : 0);
    update_yaml_int_value(yaml, "mavlink_rc_follow", setting_rc_follow_enabled ? 1 : 0);
    update_yaml_int_value(yaml, "mavlink_yaw_source_roll", setting_attitude_yaw_source_roll ? 1 : 0);
    update_yaml_int_value(yaml, "mavlink_attitude_scale", setting_attitude_scale);
    if (setting_model_index >= 0 && setting_model_index < (int)llm_model_count) {
        update_yaml_quoted_in_section(yaml, "hermes", "  model: \"", llm_models[setting_model_index].slug);
    }
    File fw = SPIFFS.open("/yaml/SC_SecConfig.yaml", "w");
    if (!fw) { M5_LOGE("save_settings: write failed"); return; }
    fw.print(yaml);
    fw.close();
    M5_LOGI("Settings saved: vol=%d bright=%d wifi=%s", setting_volume, setting_brightness,
            wifi_credential_count > 0 ? wifi_credentials[0].ssid.c_str() : "");
}

static void draw_wifi_list() {
    M5.Display.fillRoundRect(6, 24, 308, 56, 5, TFT_NAVY);
    M5.Display.drawRoundRect(6, 24, 308, 56, 5, TFT_CYAN);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_CYAN, TFT_NAVY);
    M5.Display.setCursor(14, 29);
    M5.Display.print("WiFi SSID");
    M5.Display.setTextColor(TFT_LIGHTGREY, TFT_NAVY);
    M5.Display.setCursor(178, 29);
    M5.Display.print(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "not connected");

    if (wifi_credential_count == 0) {
        M5.Display.setTextColor(TFT_WHITE, TFT_NAVY);
        M5.Display.setCursor(14, 52);
        M5.Display.print("No saved SSID");
        return;
    }

    for (size_t i = 0; i < wifi_credential_count; ++i) {
        int row_y = 41 + (int)i * 12;
        bool selected = (int)i == setting_wifi_index;
        uint16_t bg = selected ? TFT_DARKGREEN : TFT_NAVY;
        uint16_t fg = selected ? TFT_WHITE : TFT_LIGHTGREY;
        M5.Display.fillRoundRect(12, row_y, 292, 12, 3, bg);
        M5.Display.setTextColor(fg, bg);
        M5.Display.setCursor(18, row_y + 2);
        M5.Display.print(selected ? "> " : "  ");
        String ssid = wifi_credentials[i].ssid;
        if (ssid.length() > 30) ssid = ssid.substring(0, 29) + ".";
        M5.Display.print(ssid);
    }
}

static void draw_slider_row(int y, const char* label, int val) {
    const int bar_x = 8, bar_w = 304, bar_h = 14;
    const int bar_y = y + 16;
    int filled = val * bar_w / 255;

    M5.Display.fillRect(0, y, 320, 40, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(8, y + 2);
    M5.Display.print(label);
    char buf[8];
    snprintf(buf, sizeof(buf), "%3d%%", val * 100 / 255);
    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Display.setCursor(280, y + 2);
    M5.Display.print(buf);

    M5.Display.fillRect(bar_x, bar_y, filled, bar_h, TFT_CYAN);
    M5.Display.fillRect(bar_x + filled, bar_y, bar_w - filled, bar_h, TFT_DARKGREY);
    M5.Display.drawRect(bar_x, bar_y, bar_w, bar_h, TFT_WHITE);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
}

static void draw_attitude_settings_row() {
    M5.Display.fillRect(0, 84, 320, 114, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(8, 86);
    M5.Display.print("Servo follow mode");

    M5.Display.fillRoundRect(8, 103, 144, 24, 4, setting_attitude_servo_enabled ? TFT_DARKGREEN : TFT_DARKGREY);
    M5.Display.setTextColor(TFT_WHITE, setting_attitude_servo_enabled ? TFT_DARKGREEN : TFT_DARKGREY);
    M5.Display.setCursor(48, 111);
    M5.Display.print(setting_attitude_servo_enabled ? "IMU ON" : "IMU OFF");

    M5.Display.fillRoundRect(160, 103, 152, 24, 4, setting_rc_follow_enabled ? TFT_DARKGREEN : TFT_DARKGREY);
    M5.Display.setTextColor(TFT_WHITE, setting_rc_follow_enabled ? TFT_DARKGREEN : TFT_DARKGREY);
    M5.Display.setCursor(206, 111);
    M5.Display.print(setting_rc_follow_enabled ? "RC ON" : "RC OFF");

    M5.Display.fillRoundRect(8, 133, 144, 24, 4, TFT_NAVY);
    M5.Display.setTextColor(TFT_WHITE, TFT_NAVY);
    M5.Display.setCursor(25, 141);
    M5.Display.print(setting_attitude_yaw_source_roll ? "Yaw: FC Roll" : "Yaw: FC Yaw");

    M5.Display.fillRoundRect(160, 133, 152, 24, 4, TFT_DARKGREY);
    M5.Display.setTextColor(TFT_WHITE, TFT_DARKGREY);
    M5.Display.setCursor(202, 141);
    M5.Display.print("Scale ");
    M5.Display.print(setting_attitude_scale);
    M5.Display.print("%");

    M5.Display.fillRoundRect(8, 163, 304, 28, 4, TFT_MAROON);
    M5.Display.setTextColor(TFT_WHITE, TFT_MAROON);
    M5.Display.setCursor(120, 173);
    M5.Display.print("Reset Front");
}

static void draw_llm_model_list() {
    M5.Display.fillRect(0, 34, 320, 166, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    M5.Display.setCursor(8, 40);
    M5.Display.print("Select LLM model (OpenRouter)");
    for (size_t i = 0; i < llm_model_count; ++i) {
        int row_y = 58 + (int)i * 34;
        bool selected = ((int)i == setting_model_index);
        uint16_t bg = selected ? TFT_DARKGREEN : TFT_NAVY;
        uint16_t fg = selected ? TFT_WHITE : TFT_LIGHTGREY;
        M5.Display.fillRoundRect(12, row_y, 296, 30, 5, bg);
        M5.Display.drawRoundRect(12, row_y, 296, 30, 5, selected ? TFT_GREEN : TFT_DARKGREY);
        M5.Display.setTextColor(fg, bg);
        M5.Display.setCursor(22, row_y + 4);
        M5.Display.print(selected ? "> " : "  ");
        M5.Display.print(llm_models[i].label);
        M5.Display.setTextColor(selected ? TFT_GREENYELLOW : TFT_CYAN, bg);
        M5.Display.setCursor(28, row_y + 17);
        M5.Display.print(llm_models[i].slug);
    }
    if (setting_model_index < 0) {
        M5.Display.setTextColor(TFT_ORANGE, TFT_BLACK);
        M5.Display.setCursor(8, 58 + (int)llm_model_count * 34 + 2);
        M5.Display.print("Current: custom (");
        M5.Display.print(hermes_model);
        M5.Display.print(")");
    }
}

static void draw_settings_ui() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Display.setCursor(8, 8);
    M5.Display.print("Settings");
    M5.Display.fillRoundRect(90, 4, 54, 22, 4, setting_page == 0 ? TFT_NAVY : TFT_DARKGREY);
    M5.Display.setTextColor(TFT_WHITE, setting_page == 0 ? TFT_NAVY : TFT_DARKGREY);
    M5.Display.setCursor(108, 11);
    M5.Display.print("Main");
    M5.Display.fillRoundRect(150, 4, 86, 22, 4, setting_page == 1 ? TFT_NAVY : TFT_DARKGREY);
    M5.Display.setTextColor(TFT_WHITE, setting_page == 1 ? TFT_NAVY : TFT_DARKGREY);
    M5.Display.setCursor(166, 11);
    M5.Display.print("Flight");
    M5.Display.fillRoundRect(240, 4, 34, 22, 4, setting_page == 2 ? TFT_NAVY : TFT_DARKGREY);
    M5.Display.setTextColor(TFT_WHITE, setting_page == 2 ? TFT_NAVY : TFT_DARKGREY);
    M5.Display.setCursor(247, 11);
    M5.Display.print("LLM");
    M5.Display.fillRoundRect(280, 4, 36, 28, 4, TFT_DARKGREY);
    M5.Display.setTextColor(TFT_WHITE, TFT_DARKGREY);
    M5.Display.setCursor(288, 10);
    M5.Display.print("X");
    if (setting_page == 0) {
        draw_wifi_list();
        M5.Display.drawLine(0, 82, 320, 82, TFT_DARKGREY);
        draw_slider_row(84, "Volume", setting_volume);
        M5.Display.drawLine(0, 124, 320, 124, TFT_DARKGREY);
        draw_slider_row(126, "Brightness", setting_brightness);
        M5.Display.drawLine(0, 168, 320, 168, TFT_DARKGREY);
        M5.Display.setTextSize(1);
        M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5.Display.setCursor(8, 176);
        M5.Display.print("ESP-NOW MAC: ");
        M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
        M5.Display.print(WiFi.macAddress().c_str());
    } else if (setting_page == 1) {
        M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        M5.Display.setCursor(8, 42);
        M5.Display.print("Select IMU or RC follow mode.");
        draw_attitude_settings_row();
    } else {
        draw_llm_model_list();
    }
    M5.Display.drawLine(0, 200, 320, 200, TFT_DARKGREY);
    M5.Display.fillRoundRect(10, 204, 135, 30, 6, TFT_DARKGREY);
    M5.Display.setTextColor(TFT_WHITE, TFT_DARKGREY);
    M5.Display.setCursor(34, 214);
    M5.Display.print("Cancel");
    M5.Display.fillRoundRect(165, 204, 145, 30, 6, TFT_DARKGREEN);
    M5.Display.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    M5.Display.setCursor(206, 214);
    M5.Display.print("Save");
}

static void settings_enter() {
    if (busy) return;
    bpm_stop_audio();
    app_mode = MODE_SETTINGS;
    setting_volume = tts_volume;
    setting_brightness = brightness_val;
    setting_wifi_index = current_wifi_credential_index();
    setting_attitude_servo_enabled = mavlink_attitude_servo_enabled;
    setting_rc_follow_enabled = mavlink_rc_follow_enabled;
    setting_attitude_yaw_source_roll = mavlink_attitude_yaw_source_roll;
    setting_attitude_scale = mavlink_attitude_scale;
    setting_model_index = -1;
    for (size_t i = 0; i < llm_model_count; ++i) {
        if (hermes_model == llm_models[i].slug) { setting_model_index = (int)i; break; }
    }
    setting_page = 0;
    avatar.suspend();
    draw_settings_ui();
}

static void settings_exit(bool save) {
    if (save) {
        tts_volume = setting_volume;
        brightness_val = setting_brightness;
        mavlink_attitude_servo_enabled = setting_attitude_servo_enabled;
        mavlink_rc_follow_enabled = setting_rc_follow_enabled;
        if (mavlink_rc_follow_enabled) mavlink_attitude_servo_enabled = false;
        mavlink_attitude_filter_ready = false;
        mavlink_attitude_target_ready = false;
        mavlink_attitude_yaw_source_roll = setting_attitude_yaw_source_roll;
        mavlink_attitude_scale = setting_attitude_scale;
        if (setting_model_index >= 0 && setting_model_index < (int)llm_model_count) {
            hermes_model = llm_models[setting_model_index].slug;
        }
        M5.Display.setBrightness(brightness_val);
        save_settings();
        WiFi.disconnect(false, false);
        if (wifi_credential_count > 0) {
            WiFi.begin(wifi_credentials[0].ssid.c_str(), wifi_credentials[0].password.c_str());
        }
    }
    app_mode = MODE_NORMAL;
    normal_touch_guard_until_ms = millis() + 350;
    normal_touch_guard_wait_release = true;
    M5.Display.fillScreen(TFT_BLACK);
    avatar.resume();
}

static void handle_settings_touch() {
    auto touch = M5.Touch.getDetail();
    if (!(touch.wasPressed() || touch.isDragging() || touch.isPressed())) return;
    int tx = touch.x, ty = touch.y;

    // [X] or Cancel
    if (touch.wasPressed()) {
        if (tx >= 280 && ty >= 4 && ty <= 32) { settings_exit(false); return; }
        if (tx >= 10 && tx <= 145 && ty >= 204 && ty <= 234) { settings_exit(false); return; }
        if (tx >= 90 && tx <= 144 && ty >= 4 && ty <= 26) {
            setting_page = 0;
            draw_settings_ui();
            return;
        }
        if (tx >= 150 && tx <= 236 && ty >= 4 && ty <= 26) {
            setting_page = 1;
            draw_settings_ui();
            return;
        }
        if (tx >= 240 && tx <= 274 && ty >= 4 && ty <= 26) {
            setting_page = 2;
            draw_settings_ui();
            return;
        }
    }
    // Save
    if (touch.wasPressed()) {
        if (tx >= 165 && tx <= 310 && ty >= 204 && ty <= 234) { settings_exit(true); return; }
    }

    if (setting_page == 1) {
        if (touch.wasPressed() && tx >= 8 && tx <= 152 && ty >= 103 && ty <= 127) {
            setting_attitude_servo_enabled = !setting_attitude_servo_enabled;
            if (setting_attitude_servo_enabled) setting_rc_follow_enabled = false;
            draw_attitude_settings_row();
            return;
        }
        if (touch.wasPressed() && tx >= 160 && tx <= 312 && ty >= 103 && ty <= 127) {
            setting_rc_follow_enabled = !setting_rc_follow_enabled;
            if (setting_rc_follow_enabled) setting_attitude_servo_enabled = false;
            draw_attitude_settings_row();
            return;
        }
        if (touch.wasPressed() && tx >= 8 && tx <= 152 && ty >= 133 && ty <= 157) {
            setting_attitude_yaw_source_roll = !setting_attitude_yaw_source_roll;
            draw_attitude_settings_row();
            return;
        }
        if (touch.wasPressed() && tx >= 160 && tx <= 312 && ty >= 133 && ty <= 157) {
            setting_attitude_scale += 25;
            if (setting_attitude_scale > 100) setting_attitude_scale = 0;
            draw_attitude_settings_row();
            return;
        }
        if (touch.wasPressed() && tx >= 8 && tx <= 312 && ty >= 163 && ty <= 191) {
            mavlink_attitude_yaw_source_roll = setting_attitude_yaw_source_roll;
            mavlink_attitude_scale = setting_attitude_scale;
            servo_attitude_reset_front();
            draw_attitude_settings_row();
            return;
        }
        return;
    }

    if (setting_page == 2) {
        if (touch.wasPressed()) {
            for (size_t i = 0; i < llm_model_count; ++i) {
                int row_y = 58 + (int)i * 34;
                if (tx >= 12 && tx <= 308 && ty >= row_y && ty <= row_y + 30) {
                    setting_model_index = (int)i;
                    draw_llm_model_list();
                    return;
                }
            }
        }
        return;
    }

    if (touch.wasPressed() && tx >= 12 && tx <= 304 && ty >= 41 && ty <= 77 && wifi_credential_count > 0) {
        int next_index = (ty - 41) / 12;
        if (next_index >= 0 && (size_t)next_index < wifi_credential_count) {
            setting_wifi_index = next_index;
            draw_wifi_list();
        }
        return;
    }

    auto apply_slider_value = [&](int touch_x, int min_val, int max_val) -> int {
        const int bar_x = 8;
        const int bar_w = 304;
        int clamped_x = max(bar_x, min(bar_x + bar_w, touch_x));
        int value = min_val + ((clamped_x - bar_x) * (max_val - min_val)) / bar_w;
        return constrain(value, min_val, max_val);
    };

    if (ty >= 100 && ty <= 114 && tx >= 8 && tx <= 312) {
        int next_volume = apply_slider_value(tx, 0, 255);
        if (next_volume != setting_volume) {
            setting_volume = next_volume;
            draw_slider_row(84, "Volume", setting_volume);
        }
        return;
    }

    if (ty >= 142 && ty <= 156 && tx >= 8 && tx <= 312) {
        int next_brightness = apply_slider_value(tx, 20, 255);
        if (next_brightness != setting_brightness) {
            setting_brightness = next_brightness;
            M5.Display.setBrightness(setting_brightness);
            draw_slider_row(126, "Brightness", setting_brightness);
        }
        return;
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
    bpm_play_last_move_ms = 0;
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

static bool bpm_read_audio_frame(float& avg, int16_t& peak, float& raw_level) {
    avg = 0.0f;
    peak = 0;
    raw_level = 0.0f;

    if (!bpm_audio_active) {
        bpm_start_audio();
        return false;
    }
    if (!M5.Mic.isEnabled()) return false;
    if (!M5.Mic.record(bpm_buf, BPM_RECORD_LENGTH, BPM_SAMPLE_RATE, false)) return false;

    int32_t sample_sum = 0;
    for (size_t i = 0; i < BPM_RECORD_LENGTH; ++i) {
        sample_sum += bpm_buf[i];
    }
    int32_t dc_offset = sample_sum / static_cast<int32_t>(BPM_RECORD_LENGTH);

    uint32_t accum = 0;
    peak = 0;
    for (size_t i = 0; i < BPM_RECORD_LENGTH; ++i) {
        int32_t centered = static_cast<int32_t>(bpm_buf[i]) - dc_offset;
        int16_t amp = abs(centered);
        accum += amp;
        if (amp > peak) peak = amp;
    }

    avg = static_cast<float>(accum) / BPM_RECORD_LENGTH;
    raw_level = avg * 0.85f + peak * 0.15f;
    return true;
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
    auto* sy = system_config.getServoInfo(AXIS_Y);
    int play_center_y = srv_cy - 10;
    if (sy) {
        play_center_y = max<int>(sy->lower_limit, min<int>(sy->upper_limit, play_center_y));
    }
    sc_servo.moveY(play_center_y, 180, false);
    avatar.setExpression(Expression::Happy);
    show("Play " + String(play_bpm) + " BPM");
}

static void bpm_detect_tick() {
    uint32_t now = millis();
    if (!servo_ready) return;

    float avg = 0.0f;
    float raw_level = 0.0f;
    int16_t peak = 0;
    if (!bpm_read_audio_frame(avg, peak, raw_level)) {
        bpm_next_log_ms = now;
        return;
    }

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
    if (!servo_ready) return;

    auto* sx = system_config.getServoInfo(AXIS_X);
    int left_target = srv_cx - 12;
    int right_target = srv_cx + 12;
    if (sx) {
        left_target = max<int>(sx->lower_limit, left_target);
        right_target = min<int>(sx->upper_limit, right_target);
    }

    uint16_t bpm = play_bpm;
    if (bpm < 60 || bpm > 180) bpm = 120;
    uint32_t step_interval = 60000UL / bpm;
    if (step_interval < 320) step_interval = 320;
    if (step_interval > 1200) step_interval = 1200;

    float avg = 0.0f;
    float raw_level = 0.0f;
    int16_t peak = 0;
    if (bpm_read_audio_frame(avg, peak, raw_level)) {
        if (bpm_calibration_frames < 24) {
            bpm_noise_floor = (bpm_calibration_frames == 0) ? raw_level : (bpm_noise_floor * 0.82f + raw_level * 0.18f);
            bpm_calibration_frames++;
            bpm_env = 0.0f;
            bpm_avg_env = 0.0f;
        } else {
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
                    if (estimated) {
                        play_bpm = clamp_bpm_value((play_bpm * 3 + estimated) / 4);
                        bpm = play_bpm;
                        step_interval = 60000UL / bpm;
                        if (step_interval < 320) step_interval = 320;
                        if (step_interval > 1200) step_interval = 1200;
                    }
                }
                bpm_last_peak_ms = now;
                uint32_t phase_offset = bpm_play_next_step_ms
                    ? static_cast<uint32_t>(abs(static_cast<int32_t>(bpm_play_next_step_ms - now)))
                    : step_interval;
                uint32_t max_correction = max<uint32_t>(40, step_interval / 5);
                if (phase_offset <= step_interval / 2) {
                    if (bpm_play_next_step_ms > now) {
                        uint32_t correction = min<uint32_t>(phase_offset, max_correction);
                        bpm_play_next_step_ms -= correction;
                    } else {
                        uint32_t correction = min<uint32_t>(now - bpm_play_next_step_ms, max_correction);
                        bpm_play_next_step_ms += correction;
                    }
                }
            }
        }
    }

    if (!bpm_play_next_step_ms) bpm_play_next_step_ms = now;
    if (time_reached(now, bpm_play_next_step_ms)) {
        bpm_play_swing_right = !bpm_play_swing_right;
        int target_x = bpm_play_swing_right ? right_target : left_target;
        uint32_t move_ms = min<uint32_t>(220, step_interval > 120 ? (step_interval - 120) : 140);
        sc_servo.moveX(target_x, move_ms);
        avatar.setMouthOpenRatio(0.10f);
        bpm_play_last_move_ms = now;
        bpm_play_next_step_ms = now + step_interval;
        bpm_play_hold_until_ms = now + (step_interval * 3 / 5);
    } else if (bpm_play_hold_until_ms && time_reached(now, bpm_play_hold_until_ms)) {
        sc_servo.moveX(srv_cx, 120);
        avatar.setMouthOpenRatio(0.0f);
        bpm_play_hold_until_ms = 0;
    }

    if (time_reached(now, bpm_next_log_ms)) {
        Serial.printf("BPMPlay ms=%lu bpm=%u step=%lu right=%u env=%.1f avgEnv=%.1f floor=%.1f calib=%u\n",
                      bpm_mode_started_ms ? (unsigned long)(now - bpm_mode_started_ms) : 0UL,
                      bpm, (unsigned long)step_interval, bpm_play_swing_right ? 1 : 0,
                      bpm_env, bpm_avg_env, bpm_noise_floor, bpm_calibration_frames);
        bpm_next_log_ms = now + 1000;
    }
}

static bool speak_mavlink_notification(const String& text) {
    if (busy) return false;

    busy = true;
    clear_error_state();
    avatar.setExpression(Expression::Happy);
    led_set(LED_SPEAKING);
    show("[MAVLink] " + text);
    chat_add(false, text);
    speech_scroll_start(text);

    const String audio_path = mavlink_notification_audio_path(text);
    bool ok = false;
    if (!audio_path.isEmpty() && SPIFFS.exists(audio_path)) {
        ok = play_wav_file(audio_path.c_str());
    }
    if (!ok) {
        ok = !audio_path.isEmpty()
             ? call_tts_cache_wav(text, audio_path.c_str())
             : call_tts(text);
    }
    speech_scroll_stop();
    avatar.setExpression(Expression::Neutral);
    led_set(LED_OFF);
    show("");
    busy = false;
    return ok;
}

static String mavlink_notification_audio_path(const String& text) {
    if (text == "アームしました") return "/audio/mav_arm.wav";
    if (text == "ディスアームしました") return "/audio/mav_disarm.wav";
    if (text.indexOf("アーム中です") >= 0) return "/audio/mav_armed.wav";
    if (text.indexOf("ディスアーム中です") >= 0) return "/audio/mav_disarmed.wav";

    if (text.indexOf("マニュアルモードです") >= 0) return "/audio/mav_mode_manual.wav";
    if (text.indexOf("アクロモードです") >= 0) return "/audio/mav_mode_acro.wav";
    if (text.indexOf("ステアリングモードです") >= 0) return "/audio/mav_mode_steering.wav";
    if (text.indexOf("ホールドモードです") >= 0) return "/audio/mav_mode_hold.wav";
    if (text.indexOf("ロイターモードです") >= 0) return "/audio/mav_mode_loiter.wav";
    if (text.indexOf("フォローモードです") >= 0) return "/audio/mav_mode_follow.wav";
    if (text.indexOf("シンプルモードです") >= 0) return "/audio/mav_mode_simple.wav";
    if (text.indexOf("ドックモードです") >= 0) return "/audio/mav_mode_dock.wav";
    if (text.indexOf("サークルモードです") >= 0) return "/audio/mav_mode_circle.wav";
    if (text.indexOf("オートモードです") >= 0) return "/audio/mav_mode_auto.wav";
    if (text.indexOf("スマート アールティーエルモードです") >= 0) return "/audio/mav_mode_smart_rtl.wav";
    if (text.indexOf("アールティーエルモードです") >= 0) return "/audio/mav_mode_rtl.wav";
    if (text.indexOf("ガイデッドモードです") >= 0) return "/audio/mav_mode_guided.wav";
    if (text.indexOf("イニシャライズ中モードです") >= 0) return "/audio/mav_mode_initialising.wav";
    if (text.indexOf("オートチューンモードです") >= 0) return "/audio/mav_mode_autotune.wav";
    if (text.indexOf("ランドモードです") >= 0) return "/audio/mav_mode_land.wav";
    if (text == "挨拶モードです") return "/audio/mav_mode_greeting.wav";
    if (text == "高姿勢モードです") return "/audio/mav_posture_high.wav";
    if (text == "低姿勢モードです") return "/audio/mav_posture_low.wav";
    return "";
}

static void mavlink_notification_tick() {
    mavlink_receive_tick(true);
}

static bool rc_ch5_greeting_update(const MavlinkRcChannels& rc_channels, String& notification) {
    notification = "";
    if (!rc_channels.received || rc_channels.ch5_raw == UINT16_MAX) return false;

    const bool next_active = rc_channels.ch5_raw > 2000;
    if (next_active == rc_ch5_greeting_active) return false;

    rc_ch5_greeting_active = next_active;
    if (next_active) {
        notification = "挨拶モードです";
        Serial.printf("[MAVLink] RC Ch5=%u greeting notification: %s\n",
                      rc_channels.ch5_raw,
                      notification.c_str());
    }
    return !notification.isEmpty();
}

static bool rc_ch6_school_greeting_update(const MavlinkRcChannels& rc_channels, String& notification) {
    notification = "";
    if (!rc_channels.received || rc_channels.ch6_raw == UINT16_MAX) return false;

    const bool next_active = rc_channels.ch6_raw > 2000;
    if (next_active == rc_ch6_school_greeting_active) return false;

    rc_ch6_school_greeting_active = next_active;
    if (next_active) {
        notification = "ｽﾀｯｸﾁｬﾝ 5歳のお誕生日おめでとうございます。M5Stackチーム、日本のコミュニティの皆さん、素晴らしいプロダクトをありがとうございます。";
        Serial.printf("[MAVLink] RC Ch6=%u school greeting notification\n", rc_channels.ch6_raw);
    }
    return !notification.isEmpty();
}

static bool rc_ch9_posture_update(const MavlinkRcChannels& rc_channels, String& notification) {
    notification = "";
    if (!rc_channels.received || rc_channels.ch9_raw == UINT16_MAX) return false;

    int8_t next_state = 0;
    if (rc_channels.ch9_raw > 2000) {
        next_state = 1;
    } else if (rc_channels.ch9_raw < 1000) {
        next_state = 2;
    }

    if (next_state == rc_ch9_posture_state) return false;

    rc_ch9_posture_state = next_state;
    if (next_state == 1) {
        notification = "高姿勢モードです";
    } else if (next_state == 2) {
        notification = "低姿勢モードです";
    }

    if (!notification.isEmpty()) {
        Serial.printf("[MAVLink] RC Ch9=%u posture notification: %s\n",
                      rc_channels.ch9_raw,
                      notification.c_str());
    }
    return !notification.isEmpty();
}

static void mavlink_receive_tick(bool allow_speech) {
    MavlinkHeartbeat heartbeat;
    MavlinkAttitude attitude;
    MavlinkRcChannels rc_channels;
    mavlink_receiver.poll(&heartbeat, &attitude, &rc_channels);

    if (heartbeat.received) {
        String notification;
        if (flight_state_monitor.update(heartbeat, notification)) {
            Serial.println("[MAVLink] notification: " + notification);
            mavlink_pending_notification = notification;
        }
    }
    if (rc_channels.received) {
        String notification;
        servo_rc_follow_set_target(rc_channels);
        if (rc_ch5_greeting_update(rc_channels, notification)) {
            mavlink_pending_notification = notification;
        }
        if (rc_ch6_school_greeting_update(rc_channels, notification)) {
            mavlink_pending_notification = notification;
        }
        if (rc_ch9_posture_update(rc_channels, notification)) {
            mavlink_pending_notification = notification;
        }
    }
    if (attitude.received) {
        servo_attitude_set_target(attitude);
    }
    servo_attitude_tick();

    if (allow_speech && !mavlink_pending_notification.isEmpty() && !busy) {
        const String notification = mavlink_pending_notification;
        mavlink_pending_notification = "";
        stackchan_speech.speak(notification);
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
    stackchan_speech.begin(speak_mavlink_notification);
    mavlink_receiver.begin(115200, 1, 2);
    servo_idle_enabled = servo_on_boot;
    M5.Display.setBrightness(brightness_val);  // set before LED task starts
    led_init();  // must be after servo_begin() 窶・needs ioexpander initialized

    avatar.init();
    avatar.setSpeechFont(&fonts::efontJA_12);
    avatar_ready = true;
    show("Connecting WiFi...");
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    if (ensure_wifi_connected(10000)) {
        speak_server.begin();
        udp_beat.begin(UDP_BEAT_PORT);
        M5_LOGI("WiFi: %s", WiFi.localIP().toString().c_str());
        show("IP: " + WiFi.localIP().toString());
        delay(3000);
        show(servo_idle_enabled ? "M:Voice R:Servo ON TR:Settings" : "M:Voice R:Servo OFF TR:Settings");
    } else {
        show("WiFi FAILED");
    }

    // ESP-NOW head tracking receiver (works alongside WiFi on same channel)
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(on_head_track_recv);
        M5_LOGI("ESP-NOW MAC: %s", WiFi.macAddress().c_str());
        Serial.printf("ESP-NOW MAC: %s\n", WiFi.macAddress().c_str());
    } else {
        M5_LOGE("ESP-NOW init failed");
    }
}

bool touchedZone(int zone) {
    const auto& touch = current_touch_detail;
    if (normal_touch_guard_wait_release) {
        if (touch.isPressed() || !time_reached(millis(), normal_touch_guard_until_ms)) return false;
        normal_touch_guard_wait_release = false;
    }
    if (!touch.wasClicked() || touch.y < 192) return false;
    return (touch.x / (320 / 3)) == zone;
}

bool touchHoldZone(int zone) {
    const auto& touch = current_touch_detail;
    if (normal_touch_guard_wait_release) {
        if (touch.isPressed() || !time_reached(millis(), normal_touch_guard_until_ms)) return false;
        normal_touch_guard_wait_release = false;
    }
    if (!touch.wasHold() || touch.y < 192) return false;
    return (touch.x / (320 / 3)) == zone;
}

bool touchClickZone(int zone) {
    const auto& touch = current_touch_detail;
    if (normal_touch_guard_wait_release) {
        if (touch.isPressed() || !time_reached(millis(), normal_touch_guard_until_ms)) return false;
        normal_touch_guard_wait_release = false;
    }
    if (!touch.wasClicked() || touch.y < 192) return false;
    return (touch.x / (320 / 3)) == zone;
}

static bool zoneTapTriggered(int zone) {
    if (touchedZone(zone)) return true;
    if (zone == 0) return M5.BtnA.wasPressed();
    if (zone == 1) return M5.BtnB.wasPressed();
    if (zone == 2) return M5.BtnC.wasPressed();
    return false;
}

static bool zoneHoldTriggered(int zone) {
    if (touchHoldZone(zone)) return true;
    if (zone == 0) return M5.BtnA.wasHold();
    if (zone == 1) return M5.BtnB.wasHold();
    if (zone == 2) return M5.BtnC.wasHold();
    return false;
}

void loop() {
    M5.update();
    current_touch_detail = M5.Touch.getDetail();
    mavlink_notification_tick();

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
        if (time_reached(millis(), bpm_toggle_cooldown_until_ms) && zoneHoldTriggered(2)) {
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
        if (time_reached(millis(), bpm_toggle_cooldown_until_ms) && zoneHoldTriggered(2)) {
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
    handle_udp_beat();
    serial_beat_tick();
    handle_speak_server();
    process_web_tts_pending();

    // Right hold: BPM mode
    if (btn_bpm_hold && !busy && time_reached(millis(), bpm_toggle_cooldown_until_ms) && zoneHoldTriggered(2)) {
        clear_error_state();
        bpm_enter_detect_mode();
        return;
    }

    // 蟾ｦ繝帙・繝ｫ繝・ 螳壽悄逋ｺ隧ｱ繝｢繝ｼ繝・ON/OFF 繝医げ繝ｫ
    if (btn_periodic_hold && !busy && zoneHoldTriggered(0) && time_reached(millis(), periodic_hold_cooldown_ms)) {
        periodic_mode_enabled = !periodic_mode_enabled;
        periodic_hold_cooldown_ms = millis() + 2000;
        show(periodic_mode_enabled ? "Periodic: ON" : "Periodic: OFF");
    }

    if (btn_llm_tap && zoneTapTriggered(0) && !busy) {
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
        chat_add(false, reply);
        speech_scroll_start(reply);
        speech_scroll_run_to_end();
        if (call_tts(reply)) {
            speech_scroll_stop();
            avatar.setExpression(Expression::Neutral);
            led_set(LED_OFF);
            show("");
        } else {
            speech_scroll_stop();
            show_error_state("TTS failed");
        }
        busy = false;
    }

    // 蜿ｳ繧ｿ繝・メ: 繧｢繧､繝峨Ν繧ｵ繝ｼ繝懷●豁｢/蜀埼幕繝医げ繝ｫ
    if (btn_servo_tap && zoneTapTriggered(2)) {
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

    // 荳ｭ螟ｮ繧ｿ繝・メ: Push-to-Talk (STT 竊・Hermes 竊・TTS)
    if (btn_stt_tap && zoneTapTriggered(1) && !busy) {
        clear_error_state();
        busy = true;
        servo_idle_enabled = true;  // resume idle servo when user talks
        servo_idle_next_ms = millis() + 2000;
        avatar.setExpression(Expression::Happy);
        show("Speak now!");
        String text = call_stt();
        if (text.isEmpty()) {
            show_error_state("STT failed");
            busy = false;
            return;
        }
        chat_add(true, text);
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
        chat_add(false, reply);
        speech_scroll_start(reply);
        speech_scroll_run_to_end();
        if (call_tts(reply)) {
            speech_scroll_stop();
            avatar.setExpression(Expression::Neutral);
            led_set(LED_OFF);
            show("");
        } else {
            speech_scroll_stop();
            show_error_state("TTS failed");
        }
        busy = false;
    }

    // 險ｭ螳壹ヨ繝ｪ繧ｬ繝ｼ: 逕ｻ髱｢蜿ｳ荳翫さ繝ｼ繝翫・・・>260, y<50・峨ち繝・・
    if (!busy) {
        const auto& t = current_touch_detail;
        if (t.wasPressed() && t.x > 260 && t.y < 50) {
            settings_enter();
            return;
        }
    }

    http_beat_tick();
    periodic_tick();
    head_track_tick();
    servo_idle_tick();
    uint32_t now = millis();
    if (!led_effect_active && time_reached(now, head_touch_next_poll_ms)) {
        head_touch_next_poll_ms = now + 50;
        { HeadGesture g = head_pat_detected(); if (g != GESTURE_NONE) head_pat_reaction(g); }
    }

    delay(10);
}

