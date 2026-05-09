#include <Arduino.h>
#include <M5Unified.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Stackchan_system_config.h>
#include <Stackchan_servo.h>
#include <Avatar.h>

using namespace m5avatar;

Avatar avatar;
StackchanSERVO servo;
StackchanSystemConfig system_config;

// Hermes config (loaded from YAML)
String hermes_endpoint = "";
String hermes_model    = "";
String hermes_api_key  = "";

void load_hermes_config(fs::FS& fs) {
    File f = fs.open("/yaml/SC_SecConfig.yaml", "r");
    if (!f) {
        M5_LOGE("SC_SecConfig.yaml not found");
        return;
    }
    String yaml = f.readString();
    f.close();

    // Simple line-by-line parse for hermes section
    int hermes_pos = yaml.indexOf("hermes:");
    if (hermes_pos < 0) return;

    auto extract = [&](const char* key) -> String {
        String search = String("  ") + key + ": \"";
        int pos = yaml.indexOf(search, hermes_pos);
        if (pos < 0) return "";
        pos += search.length();
        int end = yaml.indexOf("\"", pos);
        return yaml.substring(pos, end);
    };

    hermes_endpoint = extract("endpoint");
    hermes_model    = extract("model");
    hermes_api_key  = extract("api_key");

    M5_LOGI("Hermes endpoint: %s", hermes_endpoint.c_str());
    M5_LOGI("Hermes model: %s", hermes_model.c_str());
}

String call_hermes(const String& user_message) {
    if (hermes_endpoint.isEmpty()) {
        return "Hermes not configured.";
    }

    HTTPClient http;
    WiFiClientSecure secure_client;
    // Let's Encrypt cert — skip CA pinning for prototype
    secure_client.setInsecure();
    http.begin(secure_client, hermes_endpoint);

    http.addHeader("Content-Type", "application/json");
    if (!hermes_api_key.isEmpty()) {
        http.addHeader("Authorization", "Bearer " + hermes_api_key);
    }

    JsonDocument doc;
    doc["model"] = hermes_model;
    JsonArray messages = doc["messages"].to<JsonArray>();
    JsonObject msg = messages.add<JsonObject>();
    msg["role"]    = "user";
    msg["content"] = user_message;
    doc["max_tokens"] = 256;
    doc["stream"]     = false;

    String body;
    serializeJson(doc, body);
    M5_LOGI("POST %s", hermes_endpoint.c_str());

    int status = http.POST(body);
    if (status != 200) {
        M5_LOGE("HTTP error: %d", status);
        http.end();
        return "Error: HTTP " + String(status);
    }

    String response = http.getString();
    http.end();

    JsonDocument resp;
    DeserializationError err = deserializeJson(resp, response);
    if (err) {
        M5_LOGE("JSON parse error");
        return "Parse error";
    }

    const char* content = resp["choices"][0]["message"]["content"];
    return content ? String(content) : "No content";
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5_LOGI("StackChan Hermes LLM Example");

    if (!SPIFFS.begin(true)) {
        M5_LOGE("SPIFFS mount failed");
        avatar.setSpeechText("SPIFFS ERROR");
        return;
    }

    system_config.loadConfig(SPIFFS, "/yaml/SC_BasicConfig.yaml");
    system_config.loadSecretConfig(SPIFFS, "/yaml/SC_SecConfig.yaml", 2048);
    load_hermes_config(SPIFFS);

    // Servo
    servo.begin(system_config.getServoInfo(AXIS_X)->pin, system_config.getServoInfo(AXIS_X)->start_degree,
                system_config.getServoInfo(AXIS_X)->offset,
                system_config.getServoInfo(AXIS_Y)->pin, system_config.getServoInfo(AXIS_Y)->start_degree,
                system_config.getServoInfo(AXIS_Y)->offset,
                (ServoType)system_config.getServoType());

    avatar.init();
    avatar.setSpeechText("Connecting WiFi...");

    // WiFi
    wifi_s* wifi = system_config.getWiFiSetting();
    WiFi.begin(wifi->ssid.c_str(), wifi->password.c_str());
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        retry++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        M5_LOGI("WiFi connected: %s", WiFi.localIP().toString().c_str());
        avatar.setSpeechText("WiFi OK! Press BtnA");
    } else {
        M5_LOGE("WiFi failed");
        avatar.setSpeechText("WiFi FAILED");
    }
}

void loop() {
    M5.update();

    // BtnA: send test message to Hermes
    if (M5.BtnA.wasPressed()) {
        avatar.setSpeechText("Thinking...");
        avatar.setExpression(Expression::Sad);

        String reply = call_hermes("こんにちは！自己紹介してください。");

        M5_LOGI("Hermes reply: %s", reply.c_str());
        // Trim to display length
        if (reply.length() > 60) reply = reply.substring(0, 60) + "...";
        avatar.setSpeechText(reply.c_str());
        avatar.setExpression(Expression::Happy);
    }

    delay(10);
}
