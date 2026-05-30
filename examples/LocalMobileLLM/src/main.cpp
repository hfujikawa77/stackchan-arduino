/**
 * LocalMobileLLM -- StackChan firmware for smartphone local LLM
 *
 * Based on HermesLLM example. New features:
 *   - VAD (Voice Activity Detection) for faster recording
 *   - Whisper STT via OpenAI-compatible local endpoint
 *   - OpenAI-compatible TTS (kokoro / piper via companion server)
 *   - All three services (STT / LLM / TTS) configurable per YAML
 *
 * Companion server: ../../../local-llm-server/server.py
 */

#include <Arduino.h>
#include <M5Unified.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Avatar.h>
#include <math.h>
#include <ServoEasing.hpp>
#include "Si12T.h"

using namespace m5avatar;

// ─── LED ──────────────────────────────────────────────────────────────────
enum LedMode { LED_IDLE, LED_HEARING, LED_THINKING, LED_SPEAKING, LED_ERROR };
static LedMode g_led = LED_IDLE;

static void update_display_color() {
    uint16_t col = TFT_WHITE;
    switch (g_led) {
        case LED_HEARING:  col = 0x001F; break; // blue
        case LED_THINKING: col = 0xFFE0; break; // yellow
        case LED_SPEAKING: col = 0x07E0; break; // green
        case LED_ERROR:    col = 0xF800; break; // red
        default:           col = TFT_BLACK; break;
    }
    // Just set a small status indicator in top-right corner
    M5.Lcd.fillRect(M5.Lcd.width()-16, 0, 16, 16, col);
}

inline void led_set(LedMode m) { g_led = m; update_display_color(); }

// ─── Avatar ───────────────────────────────────────────────────────────────
static Avatar avatar;

// ─── Servo ────────────────────────────────────────────────────────────────
static ServoEasing servo_x, servo_y;
static int srv_cx = 90, srv_cy = 90;
static int srv_xmin = 0, srv_xmax = 180;
static int srv_ymin = 60, srv_ymax = 120;
static bool servo_enabled = true;

static void servo_move(int x, int y, int ms = 300) {
    if (!servo_enabled) return;
    servo_x.startEaseTo(constrain(x, srv_xmin, srv_xmax), 60);
    servo_y.startEaseTo(constrain(y, srv_ymin, srv_ymax), 60);
    delay(ms);
}

// ─── Config ───────────────────────────────────────────────────────────────
struct WiFiCred { String ssid, pass; };
static WiFiCred wifi_list[3];
static int wifi_count = 0;

// LLM
static String hermes_endpoint   = "";
static String hermes_model      = "gemma-3-4b-it";
static String hermes_api_key    = "";
static int    hermes_max_tokens = 80;
static int    hermes_timeout_ms = 30000;

// STT
static String stt_api_key = "";
static String stt_mode    = "google";   // "google" | "whisper"

// TTS
static String tts_api_key      = "";
static String voicevox_host    = "";
static int    voicevox_speaker = 1;
static int    tts_volume       = 100;
static String tts_mode_cfg     = "";    // "" | "voicevox_local" | "openai"
static String tts_voice        = "af_heart";
static String whisper_model_name = "whisper-1";

// Local mode
static String local_host    = "";
static int    vad_threshold = 300;
static int    vad_tail_ms   = 500;
static int    brightness    = 80;

// ─── Mic buffer ───────────────────────────────────────────────────────────
#define MIC_SAMPLE_RATE   16000
#define MIC_MAX_SEC       5
#define MIC_BUF_SAMPLES   (MIC_SAMPLE_RATE * MIC_MAX_SEC)
#define MIC_BUF_BYTES     (MIC_BUF_SAMPLES * 2)
static int16_t* rec_buf = nullptr;

// ─── State ────────────────────────────────────────────────────────────────
static volatile bool busy = false;
static SemaphoreHandle_t busy_mutex = nullptr;

class ScopedLock {
    SemaphoreHandle_t _m;
public:
    explicit ScopedLock(SemaphoreHandle_t m) : _m(m) { xSemaphoreTake(_m, portMAX_DELAY); }
    ~ScopedLock() { xSemaphoreGive(_m); }
};

// ─── Head touch sensor ────────────────────────────────────────────────────
static Si12T head_sensor(SI12T_Type_High, SI12T_Sensitivity_Level_4);

// ─── WiFi ─────────────────────────────────────────────────────────────────
static bool ensure_wifi_connected(uint32_t timeout_ms = 15000) {
    if (WiFi.status() == WL_CONNECTED) return true;
    for (int i = 0; i < wifi_count; i++) {
        WiFi.begin(wifi_list[i].ssid.c_str(), wifi_list[i].pass.c_str());
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) delay(200);
        if (WiFi.status() == WL_CONNECTED) {
            M5_LOGI("WiFi connected: %s", WiFi.localIP().toString().c_str());
            return true;
        }
        WiFi.disconnect(true);
        delay(200);
    }
    return false;
}

// ─── WAV utilities ────────────────────────────────────────────────────────
static void build_wav_header(uint8_t* hdr, uint32_t num_samples,
                              uint32_t sr = 16000) {
    const uint16_t ch = 1, bits = 16;
    uint32_t data_sz  = num_samples * ch * (bits / 8);
    uint32_t riff_sz  = 36 + data_sz;
    uint32_t br       = sr * ch * bits / 8;
    uint16_t ba       = ch * bits / 8;
    uint16_t afmt     = 1, fsz = 16;
    memcpy(hdr+ 0,"RIFF",4); memcpy(hdr+ 4,&riff_sz,4);
    memcpy(hdr+ 8,"WAVE",4); memcpy(hdr+12,"fmt ",4);
    memcpy(hdr+16,&fsz,4);   memcpy(hdr+20,&afmt,2);
    memcpy(hdr+22,&ch,2);    memcpy(hdr+24,&sr,4);
    memcpy(hdr+28,&br,4);    memcpy(hdr+32,&ba,2);
    memcpy(hdr+34,&bits,2);  memcpy(hdr+36,"data",4);
    memcpy(hdr+40,&data_sz,4);
}

static void play_wav(const uint8_t* buf, size_t len) {
    if (len < 44) return;
    // Find data chunk
    size_t data_off = 44;
    uint32_t sample_rate = 16000;
    // Parse RIFF header
    if (memcmp(buf, "RIFF", 4) == 0 && memcmp(buf + 8, "WAVE", 4) == 0) {
        memcpy(&sample_rate, buf + 24, 4);
        // Find "data" chunk
        size_t pos = 12;
        while (pos + 8 < len) {
            uint32_t chunk_sz;
            memcpy(&chunk_sz, buf + pos + 4, 4);
            if (memcmp(buf + pos, "data", 4) == 0) {
                data_off = pos + 8;
                break;
            }
            pos += 8 + chunk_sz;
        }
    }
    if (data_off >= len) return;

    M5.Speaker.setVolume(tts_volume);
    led_set(LED_SPEAKING);
    avatar.setExpression(Expression::Happy);
    M5.Speaker.playRaw((const int16_t*)(buf + data_off),
                       (len - data_off) / 2, sample_rate, false, 1, 0, true);
    while (M5.Speaker.isPlaying()) {
        M5.update();
        delay(20);
    }
    avatar.setExpression(Expression::Neutral);
    led_set(LED_IDLE);
}

// ─── VAD recording ────────────────────────────────────────────────────────
static size_t record_with_vad() {
    const size_t CHUNK     = MIC_SAMPLE_RATE / 4; // 250 ms
    const int    NEED_SIL  = max(1, vad_tail_ms / 250);

    size_t total  = 0;
    bool   active = false;
    int    sil    = 0;

    while (total + CHUNK <= (size_t)MIC_BUF_SAMPLES) {
        M5.Mic.record(rec_buf + total, CHUNK, MIC_SAMPLE_RATE, true);

        int64_t sum = 0;
        for (size_t i = 0; i < CHUNK; i++) {
            int32_t s = rec_buf[total + i];
            sum += s * s;
        }
        int rms = (int)sqrtf((float)sum / (float)CHUNK);
        total += CHUNK;

        if (rms >= vad_threshold) { active = true; sil = 0; }
        else if (active && ++sil >= NEED_SIL) break;

        M5.update();
        if (M5.BtnA.wasClicked()) break;
    }
    return total;
}

// ─── Google Cloud STT ─────────────────────────────────────────────────────
static String call_stt_google() {
    if (stt_api_key.isEmpty()) return "";

    M5.Mic.record(rec_buf, MIC_BUF_SAMPLES, MIC_SAMPLE_RATE, false);
    uint32_t t0 = millis();
    while (M5.Mic.isRecording()) {
        if (millis() - t0 > 6000) { M5.Mic.stop(); break; }
        M5.update(); delay(100);
    }

    // Base64 encode
    const uint8_t* raw = (const uint8_t*)rec_buf;
    size_t raw_len = MIC_BUF_BYTES;
    static const char b64_chars[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t b64_len = ((raw_len + 2) / 3) * 4;
    String b64;
    b64.reserve(b64_len + 1);
    for (size_t i = 0; i < raw_len; i += 3) {
        uint32_t v = ((uint32_t)raw[i] << 16)
                   | (i+1 < raw_len ? (uint32_t)raw[i+1] << 8 : 0)
                   | (i+2 < raw_len ? (uint32_t)raw[i+2]      : 0);
        b64 += b64_chars[(v >> 18) & 0x3F];
        b64 += b64_chars[(v >> 12) & 0x3F];
        b64 += (i+1 < raw_len) ? b64_chars[(v >> 6) & 0x3F] : '=';
        b64 += (i+2 < raw_len) ? b64_chars[ v       & 0x3F] : '=';
    }

    String url = "https://speech.googleapis.com/v1/speech:recognize?key=" + stt_api_key;
    String body = "{\"config\":{\"encoding\":\"LINEAR16\",\"sampleRateHertz\":16000,"
                  "\"languageCode\":\"ja-JP\"},\"audio\":{\"content\":\"" + b64 + "\"}}";

    WiFiClientSecure wsc; wsc.setInsecure();
    HTTPClient http;
    http.begin(wsc, url);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(30000);
    int code = http.POST(body);
    if (code != 200) { http.end(); return ""; }
    String resp = http.getString();
    http.end();

    JsonDocument doc;
    deserializeJson(doc, resp);
    return doc["results"][0]["alternatives"][0]["transcript"].as<String>();
}

// ─── Whisper STT (local) ──────────────────────────────────────────────────
static String call_stt_whisper() {
    if (local_host.isEmpty()) return "";

    String h = local_host;
    int    p = 80;
    int    ci = h.indexOf(':');
    if (ci >= 0) { p = h.substring(ci + 1).toInt(); h = h.substring(0, ci); }

    avatar.setSpeechText("聴いています...");
    size_t samples = record_with_vad();
    if (samples == 0) return "";

    avatar.setSpeechText("認識中...");

    String bnd = "SCB" + String(millis(), HEX);
    String pm  = "--" + bnd + "\r\nContent-Disposition: form-data; name=\"model\"\r\n\r\n"
                 + whisper_model_name + "\r\n";
    String pl  = "--" + bnd + "\r\nContent-Disposition: form-data; name=\"language\"\r\n\r\nja\r\n";
    String pf  = "--" + bnd + "\r\nContent-Disposition: form-data; name=\"file\"; "
                 "filename=\"audio.wav\"\r\nContent-Type: audio/wav\r\n\r\n";
    String pe  = "\r\n--" + bnd + "--\r\n";

    uint8_t wh[44]; build_wav_header(wh, samples);
    size_t wav_bytes = 44 + samples * 2;
    size_t body_len  = pm.length() + pl.length() + pf.length() + wav_bytes + pe.length();

    WiFiClient cli; cli.setTimeout(30);
    if (!cli.connect(h.c_str(), p)) { M5_LOGE("Whisper connect fail"); return ""; }

    cli.printf("POST /v1/audio/transcriptions HTTP/1.1\r\n");
    cli.printf("Host: %s\r\nContent-Type: multipart/form-data; boundary=%s\r\n",
               local_host.c_str(), bnd.c_str());
    cli.printf("Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)body_len);
    cli.print(pm); cli.print(pl); cli.print(pf);
    cli.write(wh, 44);
    const uint8_t* ptr = (const uint8_t*)rec_buf;
    for (size_t i = 0; i < samples * 2; i += 512)
        cli.write(ptr + i, min((size_t)512, samples * 2 - i));
    cli.print(pe);

    uint32_t t0 = millis();
    while (!cli.available() && millis() - t0 < 30000) delay(10);
    cli.readStringUntil('\n'); // status
    while (cli.available()) {
        String ln = cli.readStringUntil('\n'); ln.trim();
        if (ln.isEmpty()) break;
    }
    String rb;
    while (cli.available()) rb += (char)cli.read();
    cli.stop();

    JsonDocument doc;
    if (deserializeJson(doc, rb) != DeserializationError::Ok) return "";
    String text = doc["text"].as<String>(); text.trim();
    return text;
}

static String call_stt() {
    if (!ensure_wifi_connected()) return "";
    if (stt_mode == "whisper") return call_stt_whisper();
    return call_stt_google();
}

// ─── VOICEVOX cloud TTS ───────────────────────────────────────────────────
static bool call_tts_voicevox_cloud(const String& text) {
    if (tts_api_key.isEmpty()) return false;
    String enc_text = "";
    for (char c : text) {
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') enc_text += c;
        else { enc_text += '%'; enc_text += String((uint8_t)c, HEX); }
    }
    String url1 = "https://api.tts.quest/v3/voicevox/synthesis?text=" + enc_text
                  + "&speaker=" + voicevox_speaker + "&key=" + tts_api_key;
    WiFiClientSecure wsc; wsc.setInsecure();
    HTTPClient http; http.begin(wsc, url1); http.setTimeout(15000);
    int code = http.GET();
    if (code != 200) { http.end(); return false; }
    JsonDocument doc; deserializeJson(doc, http.getString()); http.end();
    String status_url = doc["audioStatusUrl"].as<String>();
    String dl_url     = doc["wavDownloadUrl"].as<String>();
    // Poll until ready
    for (int i = 0; i < 40; i++) {
        delay(500);
        http.begin(wsc, status_url); code = http.GET();
        if (code == 200) {
            JsonDocument s; deserializeJson(s, http.getString()); http.end();
            if (s["isAudioReady"].as<bool>()) break;
        } else { http.end(); }
    }
    // Download WAV
    http.begin(wsc, dl_url);
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http.setTimeout(30000);
    code = http.GET();
    if (code != 200) { http.end(); return false; }
    int wav_len = http.getSize();
    uint8_t* wav = (uint8_t*)heap_caps_malloc(wav_len > 0 ? wav_len : 256*1024, MALLOC_CAP_SPIRAM);
    if (!wav) { http.end(); return false; }
    http.getStream().readBytes(wav, wav_len);
    http.end();
    play_wav(wav, wav_len);
    heap_caps_free(wav);
    return true;
}

// ─── VOICEVOX local TTS ───────────────────────────────────────────────────
static bool call_tts_voicevox_local(const String& text) {
    if (voicevox_host.isEmpty()) return false;
    String enc_text = "";
    for (char c : text) {
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') enc_text += c;
        else { enc_text += '%'; enc_text += String((uint8_t)c, HEX); }
    }
    WiFiClient cli;
    String url1 = "/audio_query?text=" + enc_text + "&speaker=" + voicevox_speaker;
    String host = voicevox_host;
    int    port = 50021;
    int    ci   = host.indexOf(':');
    if (ci >= 0) { port = host.substring(ci+1).toInt(); host = host.substring(0,ci); }

    cli.setTimeout(15); cli.connect(host.c_str(), port);
    cli.printf("POST %s HTTP/1.1\r\nHost: %s\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
               url1.c_str(), voicevox_host.c_str());
    uint32_t t0 = millis();
    while (!cli.available() && millis()-t0 < 15000) delay(10);
    cli.readStringUntil('\n');
    while (cli.available()) { String ln=cli.readStringUntil('\n'); ln.trim(); if(ln.isEmpty()) break; }
    String query_json; while (cli.available()) query_json += (char)cli.read();
    cli.stop();

    // Step 2: synthesis
    cli.connect(host.c_str(), port);
    cli.printf("POST /synthesis?speaker=%d HTTP/1.1\r\n", voicevox_speaker);
    cli.printf("Host: %s\r\nContent-Type: application/json\r\n", voicevox_host.c_str());
    cli.printf("Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)query_json.length());
    cli.print(query_json);
    t0 = millis();
    while (!cli.available() && millis()-t0 < 30000) delay(10);
    cli.readStringUntil('\n');
    size_t wav_len = 0;
    while (cli.available()) {
        String ln=cli.readStringUntil('\n'); ln.trim(); if(ln.isEmpty()) break;
        if (ln.startsWith("Content-Length:")) wav_len = ln.substring(15).toInt();
    }
    size_t buf_sz = wav_len > 0 ? wav_len : 256*1024;
    uint8_t* wav = (uint8_t*)heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM);
    if (!wav) { cli.stop(); return false; }
    size_t rcvd = 0;
    while (rcvd < buf_sz && millis()-t0 < 30000) {
        if (cli.available()) wav[rcvd++] = cli.read(); else delay(1);
    }
    cli.stop();
    play_wav(wav, rcvd);
    heap_caps_free(wav);
    return true;
}

// ─── OpenAI TTS (local companion server) ─────────────────────────────────
static bool call_tts_openai_speech(const String& text) {
    if (local_host.isEmpty()) return false;
    String h = local_host;
    int    p = 80;
    int    ci = h.indexOf(':');
    if (ci >= 0) { p = h.substring(ci+1).toInt(); h = h.substring(0,ci); }

    String body = "{\"model\":\"tts-1\",\"input\":\"";
    for (char c : text) {
        if      (c=='"')  body += "\\\"";
        else if (c=='\\') body += "\\\\";
        else              body += c;
    }
    body += "\",\"voice\":\"" + tts_voice + "\",\"response_format\":\"wav\"}";

    WiFiClient cli; cli.setTimeout(30);
    if (!cli.connect(h.c_str(), p)) return false;
    cli.printf("POST /v1/audio/speech HTTP/1.1\r\nHost: %s\r\n", local_host.c_str());
    cli.print("Content-Type: application/json\r\n");
    cli.printf("Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)body.length());
    cli.print(body);

    uint32_t t0 = millis();
    while (!cli.available() && millis()-t0 < 30000) delay(10);
    cli.readStringUntil('\n');
    size_t wav_len = 0;
    while (cli.available()) {
        String ln=cli.readStringUntil('\n'); ln.trim(); if(ln.isEmpty()) break;
        if (ln.startsWith("Content-Length:")) wav_len = ln.substring(15).toInt();
    }
    size_t buf_sz = (wav_len>0 && wav_len<1024*1024) ? wav_len : 512*1024;
    uint8_t* wav = (uint8_t*)heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM);
    if (!wav) { cli.stop(); return false; }
    size_t rcvd = 0;
    while (rcvd < buf_sz && millis()-t0 < 30000) {
        if (cli.available()) wav[rcvd++] = cli.read(); else delay(1);
    }
    cli.stop();
    play_wav(wav, rcvd);
    heap_caps_free(wav);
    return true;
}

static bool call_tts(const String& text) {
    if (!ensure_wifi_connected()) return false;
    if (tts_mode_cfg == "openai")         return call_tts_openai_speech(text);
    if (tts_mode_cfg == "voicevox_local") return call_tts_voicevox_local(text);
    if (!voicevox_host.isEmpty())         return call_tts_voicevox_local(text);
    return call_tts_voicevox_cloud(text);
}

// ─── LLM call ─────────────────────────────────────────────────────────────
static String call_llm(const String& user_msg) {
    if (hermes_endpoint.isEmpty() || !ensure_wifi_connected()) return "";

    JsonDocument req;
    req["model"]      = hermes_model;
    req["max_tokens"] = hermes_max_tokens;
    req["stream"]     = false;
    JsonArray msgs = req["messages"].to<JsonArray>();
    JsonObject m = msgs.add<JsonObject>();
    m["role"]    = "user";
    m["content"] = user_msg;

    String body; serializeJson(req, body);

    bool is_https = hermes_endpoint.startsWith("https");
    HTTPClient http;
    if (is_https) {
        static WiFiClientSecure wsc; wsc.setInsecure();
        http.begin(wsc, hermes_endpoint);
    } else {
        http.begin(hermes_endpoint);
    }
    if (!hermes_api_key.isEmpty())
        http.addHeader("Authorization", "Bearer " + hermes_api_key);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(hermes_timeout_ms);
    int code = http.POST(body);
    if (code != 200) { http.end(); return ""; }
    String resp = http.getString();
    http.end();

    JsonDocument doc;
    deserializeJson(doc, resp);
    return doc["choices"][0]["message"]["content"].as<String>();
}

// ─── Full pipeline ────────────────────────────────────────────────────────
static void run_pipeline() {
    if (busy) return;
    ScopedLock lk(busy_mutex);
    busy = true;

    led_set(LED_HEARING);
    avatar.setSpeechText("聴いています...");
    avatar.setExpression(Expression::Sleepy);
    servo_move(srv_cx - 10, srv_cy - 5);

    String text = call_stt();
    if (text.isEmpty()) {
        led_set(LED_ERROR);
        avatar.setSpeechText("認識できませんでした");
        delay(1500);
        led_set(LED_IDLE); avatar.setSpeechText(""); avatar.setExpression(Expression::Neutral);
        servo_move(srv_cx, srv_cy);
        busy = false; return;
    }
    M5_LOGI("STT: %s", text.c_str());
    avatar.setSpeechText(text.c_str());

    led_set(LED_THINKING);
    avatar.setExpression(Expression::Doubt);
    servo_move(srv_cx, srv_cy);

    String reply = call_llm(text);
    if (reply.isEmpty()) reply = "すみません、うまく答えられませんでした。";
    M5_LOGI("LLM: %s", reply.c_str());
    avatar.setSpeechText(reply.c_str());
    servo_move(srv_cx + 10, srv_cy);

    call_tts(reply);

    led_set(LED_IDLE);
    avatar.setSpeechText("");
    avatar.setExpression(Expression::Neutral);
    servo_move(srv_cx, srv_cy);
    busy = false;
}

// ─── Config loading ───────────────────────────────────────────────────────
static String extract_yaml_str(const String& yaml, const String& key) {
    int pos = yaml.indexOf(key + ":");
    if (pos < 0) return "";
    int q1 = yaml.indexOf('"', pos);
    int q2 = yaml.indexOf('"', q1 + 1);
    if (q1 < 0 || q2 <= q1) return "";
    return yaml.substring(q1 + 1, q2);
}

static int extract_yaml_int(const String& yaml, const String& key, int def = 0) {
    int pos = yaml.indexOf(key + ":");
    if (pos < 0) return def;
    int nl = yaml.indexOf('\n', pos);
    String v = yaml.substring(pos + key.length() + 1, nl);
    v.trim();
    return v.isEmpty() ? def : v.toInt();
}

static void load_config(fs::FS& fs) {
    // Basic config (servo info)
    if (!fs.exists("/yaml/SC_BasicConfig.yaml")) return;
    File f = fs.open("/yaml/SC_BasicConfig.yaml");
    String yaml = f.readString(); f.close();
    srv_cx = extract_yaml_int(yaml, "    x", 90);
    srv_cy = extract_yaml_int(yaml, "    y", 90);

    // Secret config
    if (!fs.exists("/yaml/SC_SecConfig.yaml")) return;
    f = fs.open("/yaml/SC_SecConfig.yaml");
    yaml = f.readString(); f.close();

    hermes_endpoint   = extract_yaml_str(yaml, "  endpoint");
    hermes_model      = extract_yaml_str(yaml, "  model");
    hermes_api_key    = extract_yaml_str(yaml, "  api_key");
    hermes_max_tokens = extract_yaml_int(yaml, "  max_tokens", 80);
    hermes_timeout_ms = extract_yaml_int(yaml, "  timeout_ms", 30000);

    stt_api_key    = extract_yaml_str(yaml, "stt_api_key");
    stt_mode       = extract_yaml_str(yaml, "stt_mode");
    if (stt_mode.isEmpty()) stt_mode = "google";

    tts_api_key    = extract_yaml_str(yaml, "tts_api_key");
    voicevox_host  = extract_yaml_str(yaml, "voicevox_host");
    voicevox_speaker = extract_yaml_int(yaml, "voicevox_speaker", 1);
    tts_volume     = extract_yaml_int(yaml, "tts_volume", 100);
    tts_mode_cfg   = extract_yaml_str(yaml, "tts_mode");
    tts_voice      = extract_yaml_str(yaml, "tts_voice");
    if (tts_voice.isEmpty()) tts_voice = "af_heart";

    local_host     = extract_yaml_str(yaml, "local_host");
    vad_threshold  = extract_yaml_int(yaml, "vad_threshold", 300);
    vad_tail_ms    = extract_yaml_int(yaml, "vad_tail_ms", 500);
    brightness     = extract_yaml_int(yaml, "brightness", 80);

    // WiFi
    wifi_count = 0;
    int wl = yaml.indexOf("wifi_list:");
    if (wl >= 0) {
        String wblock = yaml.substring(wl);
        for (int i = 0; i < 3 && wifi_count < 3; i++) {
            String s = extract_yaml_str(wblock, "  - ssid");
            if (s.isEmpty()) { s = extract_yaml_str(wblock, "    ssid"); }
            String pw = extract_yaml_str(wblock, "    pass");
            if (!s.isEmpty()) { wifi_list[wifi_count].ssid = s; wifi_list[wifi_count].pass = pw; wifi_count++; }
            // advance past this entry
            int next = wblock.indexOf("  - ssid:", 5);
            if (next < 0) break;
            wblock = wblock.substring(next);
        }
    }
    // Fallback: single ssid/pass at top level
    if (wifi_count == 0) {
        String s = extract_yaml_str(yaml, "ssid");
        String pw = extract_yaml_str(yaml, "pass");
        if (!s.isEmpty()) { wifi_list[0].ssid=s; wifi_list[0].pass=pw; wifi_count=1; }
    }

    M5_LOGI("Config loaded. LLM: %s  STT: %s  TTS: %s  LocalHost: %s",
            hermes_endpoint.c_str(), stt_mode.c_str(), tts_mode_cfg.c_str(), local_host.c_str());
}

// ─── Web config server ────────────────────────────────────────────────────
static WiFiServer speak_server(80);

static String html_input(const String& name, const String& label, const String& val,
                          const String& placeholder = "") {
    return "<tr><td>" + label + "</td><td><input name='" + name +
           "' value='" + val + "' placeholder='" + placeholder + "'></td></tr>\n";
}

struct HtmlOpt { const char* v; const char* l; };

static String html_select(const String& name, const String& label,
                           const HtmlOpt* opts, const String& cur) {
    String s = "<tr><td>" + label + "</td><td><select name='" + name + "'>";
    for (const HtmlOpt* o = opts; o->v != nullptr; o++)
        s += String("<option value='") + o->v + "'" + (cur == o->v ? " selected" : "") + ">" + o->l + "</option>";
    s += "</select></td></tr>\n";
    return s;
}

static void handle_config_get(WiFiClient& client) {
    String html = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n";
    html += "<!DOCTYPE html><html><head><meta charset='utf-8'><title>StackChan Local LLM Config</title>"
            "<style>body{font-family:sans-serif;background:#111;color:#eee;padding:16px}"
            "input,select{background:#222;color:#eee;border:1px solid #555;padding:4px;width:280px}"
            "td{padding:4px 8px}button{background:#444;color:#fff;padding:8px 24px;border:none;cursor:pointer}"
            "h2{color:#88f}h3{color:#8df}</style></head><body>"
            "<h2>StackChan LocalMobileLLM Config</h2>"
            "<form method='POST' action='/config'><table>";

    html += "<tr><th colspan=2><h3>LLM (Hermes)</h3></th></tr>";
    html += html_input("hermes_endpoint", "Endpoint", hermes_endpoint, "http://phone-ip:1234/v1/chat/completions");
    html += html_input("hermes_model",    "Model",    hermes_model);
    html += html_input("hermes_api_key",  "API Key",  hermes_api_key, "lm-studio");
    html += html_input("hermes_max_tokens","Max Tokens", String(hermes_max_tokens));
    html += html_input("hermes_timeout_ms","Timeout (ms)", String(hermes_timeout_ms));

    html += "<tr><th colspan=2><h3>Local Mode (Companion Server)</h3></th></tr>";
    html += html_input("local_host", "Companion Host:Port", local_host, "192.168.x.x:8080");
    { static const HtmlOpt o[] = {{"google","Google Cloud STT"},{"whisper","Local Whisper"},{nullptr,nullptr}};
      html += html_select("stt_mode", "STT Mode", o, stt_mode); }
    { static const HtmlOpt o[] = {{"","VOICEVOX Cloud"},{"voicevox_local","VOICEVOX Local"},{"openai","Local OpenAI TTS"},{nullptr,nullptr}};
      html += html_select("tts_mode", "TTS Mode", o, tts_mode_cfg); }
    html += html_input("tts_voice", "TTS Voice (OpenAI)", tts_voice, "af_heart");
    html += html_input("vad_threshold", "VAD Threshold (0-5000)", String(vad_threshold));
    html += html_input("vad_tail_ms",   "VAD Tail (ms)",          String(vad_tail_ms));

    html += "<tr><th colspan=2><h3>Google STT</h3></th></tr>";
    html += html_input("stt_api_key", "Google STT API Key", stt_api_key);

    html += "<tr><th colspan=2><h3>VOICEVOX</h3></th></tr>";
    html += html_input("tts_api_key",      "TTS Quest API Key",    tts_api_key);
    html += html_input("voicevox_host",    "VOICEVOX Local Host",  voicevox_host, "192.168.x.x:50021");
    html += html_input("voicevox_speaker", "VOICEVOX Speaker",     String(voicevox_speaker));
    html += html_input("tts_volume",       "TTS Volume (0-100)",   String(tts_volume));

    html += "<tr><th colspan=2><h3>WiFi</h3></th></tr>";
    for (int i = 0; i < 3; i++) {
        html += html_input("ssid" + String(i), "SSID " + String(i+1),
                i < wifi_count ? wifi_list[i].ssid : "");
        html += html_input("pass" + String(i), "Pass " + String(i+1),
                i < wifi_count ? wifi_list[i].pass : "");
    }

    html += "</table><br><button type='submit'>Save &amp; Restart</button></form></body></html>";
    client.print(html);
}

static void handle_config_post(WiFiClient& client, const String& body) {
    auto get_field = [&](const String& key) -> String {
        int pos = body.indexOf(key + "=");
        if (pos < 0) return "";
        int end = body.indexOf('&', pos);
        String val = (end < 0) ? body.substring(pos + key.length() + 1)
                                : body.substring(pos + key.length() + 1, end);
        // URL decode
        String out; out.reserve(val.length());
        for (int i = 0; i < (int)val.length(); i++) {
            if (val[i] == '+') out += ' ';
            else if (val[i] == '%' && i + 2 < (int)val.length()) {
                char hex[3] = { val[i+1], val[i+2], 0 };
                out += (char)strtol(hex, nullptr, 16);
                i += 2;
            } else out += val[i];
        }
        return out;
    };

    // Build new YAML
    String yaml = "";
    yaml += "hermes:\n";
    yaml += "  endpoint: \"" + get_field("hermes_endpoint") + "\"\n";
    yaml += "  model: \"" + get_field("hermes_model") + "\"\n";
    yaml += "  api_key: \"" + get_field("hermes_api_key") + "\"\n";
    yaml += "  max_tokens: " + get_field("hermes_max_tokens") + "\n";
    yaml += "  timeout_ms: " + get_field("hermes_timeout_ms") + "\n";
    yaml += "local_host: \"" + get_field("local_host") + "\"\n";
    yaml += "stt_mode: \"" + get_field("stt_mode") + "\"\n";
    yaml += "tts_mode: \"" + get_field("tts_mode") + "\"\n";
    yaml += "tts_voice: \"" + get_field("tts_voice") + "\"\n";
    yaml += "vad_threshold: " + get_field("vad_threshold") + "\n";
    yaml += "vad_tail_ms: " + get_field("vad_tail_ms") + "\n";
    yaml += "stt_api_key: \"" + get_field("stt_api_key") + "\"\n";
    yaml += "tts_api_key: \"" + get_field("tts_api_key") + "\"\n";
    yaml += "voicevox_host: \"" + get_field("voicevox_host") + "\"\n";
    yaml += "voicevox_speaker: " + get_field("voicevox_speaker") + "\n";
    yaml += "tts_volume: " + get_field("tts_volume") + "\n";
    yaml += "wifi_list:\n";
    for (int i = 0; i < 3; i++) {
        String s = get_field("ssid" + String(i));
        String pw = get_field("pass" + String(i));
        if (!s.isEmpty()) yaml += "  - ssid: \"" + s + "\"\n    pass: \"" + pw + "\"\n";
    }

    File f = SPIFFS.open("/yaml/SC_SecConfig.yaml", FILE_WRITE);
    if (f) { f.print(yaml); f.close(); }

    client.print("HTTP/1.1 303 See Other\r\nLocation: /\r\n\r\n");
    delay(200);
    ESP.restart();
}

static void handle_speak_post(WiFiClient& client, const String& body) {
    JsonDocument doc; deserializeJson(doc, body);
    String text = doc["text"].as<String>();
    if (!text.isEmpty() && !busy) {
        client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n{\"ok\":true}");
        xTaskCreate([](void* p){
            String* t = (String*)p;
            if (!busy) { busy=true; call_tts(*t); busy=false; }
            delete t;
            vTaskDelete(nullptr);
        }, "speak", 8192, new String(text), 2, nullptr);
    } else {
        client.print("HTTP/1.1 503 Busy\r\n\r\n");
    }
}

static void handle_web_server() {
    WiFiClient client = speak_server.available();
    if (!client) return;
    uint32_t t0 = millis();
    while (!client.available() && millis()-t0 < 2000) delay(10);
    String req = client.readStringUntil('\r');
    client.readStringUntil('\n');

    String method = req.substring(0, req.indexOf(' '));
    String path   = req.substring(req.indexOf(' ')+1, req.lastIndexOf(' '));

    // Read headers
    size_t content_len = 0;
    while (client.available()) {
        String h = client.readStringUntil('\n'); h.trim();
        if (h.isEmpty()) break;
        if (h.startsWith("Content-Length:")) content_len = h.substring(15).toInt();
    }
    String body = "";
    for (size_t i = 0; i < content_len && client.available(); i++)
        body += (char)client.read();

    if (method == "GET" && path == "/") {
        handle_config_get(client);
    } else if (method == "POST" && path == "/config") {
        handle_config_post(client, body);
    } else if (method == "POST" && path == "/speak") {
        handle_speak_post(client, body);
    } else {
        client.print("HTTP/1.1 404 Not Found\r\n\r\n");
    }
    client.stop();
}

// ─── Servo idle ───────────────────────────────────────────────────────────
static uint32_t servo_idle_next = 0;

static void servo_idle_tick() {
    if (!servo_enabled || busy) return;
    if (millis() < servo_idle_next) return;
    int x = srv_cx + random(-20, 21);
    int y = srv_cy + random(-10, 11);
    x = constrain(x, srv_xmin, srv_xmax);
    y = constrain(y, srv_ymin, srv_ymax);
    servo_x.startEaseTo(x, 30);
    servo_y.startEaseTo(y, 30);
    servo_idle_next = millis() + random(3000, 7000);
}

// ─── Setup / Loop ─────────────────────────────────────────────────────────
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Speaker.begin();
    M5.Lcd.setBrightness(80);

    SPIFFS.begin(true);

    busy_mutex = xSemaphoreCreateMutex();

    // Allocate mic buffer in PSRAM
    rec_buf = (int16_t*)heap_caps_malloc(MIC_BUF_BYTES, MALLOC_CAP_SPIRAM);
    if (!rec_buf) rec_buf = new int16_t[MIC_BUF_SAMPLES];

    // Load config
    load_config(SPIFFS);
    M5.Lcd.setBrightness(brightness);

    // Servo init
    servo_x.attach(7);  // defaults; overridden per SC_BasicConfig
    servo_y.attach(6);
    servo_x.setEasingType(EASE_QUADRATIC_IN_OUT);
    servo_y.setEasingType(EASE_QUADRATIC_IN_OUT);
    servo_move(srv_cx, srv_cy, 500);

    // Avatar
    avatar.init();
    avatar.setSpeechFont(&fonts::efontJA_16);

    // Head touch sensor
    head_sensor.begin();

    // WiFi
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    if (ensure_wifi_connected(15000)) {
        speak_server.begin();
        M5_LOGI("Web config: http://%s/", WiFi.localIP().toString().c_str());
    }

    led_set(LED_IDLE);
    avatar.setSpeechText("Ready");
    delay(800);
    avatar.setSpeechText("");
}

void loop() {
    M5.update();

    // Head touch sensor - tap front triggers pipeline
    head_sensor.read_touch_result();
    head_sensor.parse_touch_result();
    bool head_tapped = (head_sensor.point_type[1] != OUTPUT_NONE);

    // BtnA / touch center triggers pipeline
    bool triggered = M5.BtnA.wasClicked() || head_tapped;

    // CoreS3 touchscreen: tap center
    if (M5.Touch.getCount() > 0) {
        auto t = M5.Touch.getDetail(0);
        if (t.wasClicked()) {
            int cx = M5.Lcd.width() / 2;
            int cy = M5.Lcd.height() / 2;
            if (abs(t.x - cx) < cx/2 && abs(t.y - cy) < cy/2)
                triggered = true;
        }
    }

    if (triggered && !busy) {
        xTaskCreate([](void*){ run_pipeline(); vTaskDelete(nullptr); },
                    "pipeline", 16384, nullptr, 2, nullptr);
    }

    handle_web_server();
    servo_idle_tick();
    delay(10);
}
