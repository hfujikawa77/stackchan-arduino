#include <Arduino.h>
#include <M5Unified.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Stackchan_system_config.h>

StackchanSystemConfig system_config;

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
    if (hermes_endpoint.isEmpty()) return "Hermes not configured.";

    HTTPClient http;
    WiFiClientSecure secure_client;
    secure_client.setInsecure();
    http.begin(secure_client, hermes_endpoint);
    http.setTimeout(30000);

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
    if (deserializeJson(resp, response)) return "Parse error";

    const char* content = resp["choices"][0]["message"]["content"];
    return content ? String(content) : "No content";
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
    M5.Display.setTextSize(2);
    M5.Display.setTextWrap(true);
    M5_LOGI("StackChan Hermes LLM Example");

    if (!SPIFFS.begin(true)) {
        show("SPIFFS ERROR");
        return;
    }

    system_config.loadConfig(SPIFFS, "/yaml/SC_BasicConfig.yaml");
    load_hermes_config(SPIFFS);

    show("Connecting WiFi...");
    wifi_s* wifi = system_config.getWiFiSetting();
    WiFi.begin(wifi->ssid.c_str(), wifi->password.c_str());

    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        retry++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        M5_LOGI("WiFi connected: %s", WiFi.localIP().toString().c_str());
        show("WiFi OK!\nCalling Hermes...");
        delay(1000);
        String reply = call_hermes("こんにちは！一言で自己紹介してください。");
        M5_LOGI("Reply: %s", reply.c_str());
        show(reply);
    } else {
        show("WiFi FAILED");
    }
}

void loop() {
    M5.update();

    if (M5.BtnA.wasPressed()) {
        show("Thinking...");
        String reply = call_hermes("こんにちは！一言で自己紹介してください。");
        M5_LOGI("Reply: %s", reply.c_str());
        show(reply);
    }

    delay(10);
}
