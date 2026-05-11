#include <Arduino.h>
#include <M5Unified.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Stackchan_system_config.h>
#include <Stackchan_servo.h>
bool touchedZone(int zone);
void show(const String& text);
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

// --- Servo ---
StackchanSERVO sc_servo;
static bool servo_ready = false;
static int16_t srv_cx = 150, srv_cy = 90;
static uint32_t servo_idle_next_ms = 0;

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
    if (!servo_ready) return;
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
    servo_idle_next_ms = millis() + random(si->interval_min, si->interval_max + 1);
}

String hermes_endpoint = "";
String hermes_model    = "";
String hermes_api_key  = "";
int    tts_volume          = 100;
int    hermes_max_tokens   = 80;
int    hermes_timeout_ms   = 60000;

void load_hermes_config(fs::FS& fs) {
    File f = fs.open("/yaml/SC_SecConfig.yaml", "r");
    if (!f) { M5_LOGE("SC_SecConfig.yaml not found"); return; }
    String yaml = f.readString();
    f.close();

    int hermes_pos = yaml.indexOf("hermes:");
    if (hermes_pos >= 0) {
        auto extract = [&](const char* key) -> String {
            String search = String("  ") + key + ": \"";
            int pos = yaml.indexOf(search, hermes_pos);
            if (pos < 0) return "";
            pos += search.length();
            return yaml.substring(pos, yaml.indexOf("\"", pos));
        };
        hermes_endpoint = extract("endpoint");
        hermes_model    = extract("model");
        hermes_api_key  = extract("api_key");
        M5_LOGI("Hermes endpoint: %s", hermes_endpoint.c_str());
    }

    auto extractInt = [&](const char* key, int defaultVal) -> int {
        String search = String(key) + ":";
        int pos = yaml.indexOf(search);
        if (pos < 0) return defaultVal;
        pos += search.length();
        while (pos < (int)yaml.length() && yaml[pos] == ' ') pos++;
        return yaml.substring(pos).toInt();
    };

    tts_volume        = extractInt("tts_volume", tts_volume);
    hermes_max_tokens = extractInt("hermes_max_tokens", hermes_max_tokens);
    hermes_timeout_ms = extractInt("hermes_timeout_ms", hermes_timeout_ms);
    M5_LOGI("tts_volume: %d, hermes_max_tokens: %d, hermes_timeout_ms: %d", tts_volume, hermes_max_tokens, hermes_timeout_ms);
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

bool call_tts(const String& text) {
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
    M5_LOGI("playRaw: offset=%u rate=%u samples=%u", data_offset, sample_rate, (read_len - data_offset) / 2);
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
                sc_servo.moveY(speak_phase ? srv_cy - 8 : srv_cy, 500, false);  // gentle nod, non-blocking
                speak_next_ms = now + 900;
            }
        }
    }
    if (servo_ready) sc_servo.moveXY(srv_cx, srv_cy, 500);  // return to center after speaking

    heap_caps_free(wav_buf);
    return true;
}

String call_stt() {
    String api_key = system_config.getAPISetting()->stt;
    if (api_key.isEmpty()) { M5_LOGE("STT API key not set"); return ""; }

    int16_t* rec_buf = (int16_t*)heap_caps_malloc(MIC_BUF_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rec_buf) { M5_LOGE("rec_buf malloc failed"); return ""; }

    M5.Speaker.end();
    M5.Mic.begin();
    if (servo_ready) sc_servo.moveXY(srv_cx + 15, srv_cy - 5, 500);  // listening tilt
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
    M5_LOGI("Recording done");
    if (cancelled) { heap_caps_free(rec_buf); show("Cancelled"); return ""; }

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
    M5.Display.clear();
    M5.Display.setCursor(0, 0);
    M5.Display.println(text);
    Serial.println(text);
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    auto spk_cfg = M5.Speaker.config();
    spk_cfg.dma_buf_len = 1024;  // spk_task stack = 1280 + dma_buf_len*4 = 5376 bytes
    M5.Speaker.config(spk_cfg);
    M5.Speaker.begin();
    M5.Display.setFont(&fonts::efontJA_16);
    M5.Display.setTextSize(1);
    M5.Display.setTextWrap(true);

    if (!SPIFFS.begin(true)) { show("SPIFFS ERROR"); return; }

    system_config.loadConfig(SPIFFS, "/yaml/SC_BasicConfig.yaml");
    servo_begin();
    load_hermes_config(SPIFFS);

    show("Connecting WiFi...");
    wifi_s* wifi = system_config.getWiFiSetting();
    WiFi.begin(wifi->ssid.c_str(), wifi->password.c_str());
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) { delay(500); retry++; }

    if (WiFi.status() == WL_CONNECTED) {
        M5_LOGI("WiFi: %s", WiFi.localIP().toString().c_str());
        show("WiFi OK!\nL:Test  M:Voice\nR:Stop");
    } else {
        show("WiFi FAILED");
    }
}

bool touchedZone(int zone) {
    auto touch = M5.Touch.getDetail();
    if (!touch.wasPressed() || touch.y < 192) return false;
    return (touch.x / (320 / 3)) == zone;
}

void loop() {
    M5.update();
    servo_idle_tick();

    // 左タッチ: テスト発話
    if (touchedZone(0)) {
        show("Thinking...");
        String reply = call_hermes("こんにちは！一言で自己紹介してください。");
        if (reply.startsWith("Error:") || reply.startsWith("Parse error") || reply.startsWith("No content")) {
            show(reply);
            return;
        }
        show(reply + "\n[Speaking...]");
        if (call_tts(reply)) {
            show(reply + "\n[Done]");
        } else {
            show(reply + "\n[TTS failed]");
        }
    }

    // 中央タッチ: Push-to-Talk (STT → Hermes → TTS)
    if (touchedZone(1)) {
        show("Recording...\n(5 sec)\nSpeak now!");
        String text = call_stt();
        if (text.isEmpty()) { show("STT failed\nor no speech"); return; }
        show("You: " + text + "\nThinking...");
        String reply = call_hermes(text);
        if (reply.startsWith("Error:") || reply.startsWith("Parse error") || reply.startsWith("No content")) {
            show(reply);
            return;
        }
        show(reply + "\n[Speaking...]");
        if (call_tts(reply)) {
            show(reply + "\n[Done]");
        } else {
            show(reply + "\n[TTS failed]");
        }
    }

    delay(10);
}
