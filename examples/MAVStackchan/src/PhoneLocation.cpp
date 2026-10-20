#include "PhoneLocation.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace {

constexpr uint32_t kFetchIntervalMs = 60 * 1000;
constexpr uint32_t kHttpTimeoutMs = 5000;
constexpr uint32_t kTaskStackBytes = 12 * 1024;  // TLS の処理に必要（HermesLLM と同程度）

SemaphoreHandle_t location_mutex = nullptr;
PhoneLocation latest;
String base_url;
String token;

// GET {base}/location を呼び、位置を取り出す。
bool fetchOnce(PhoneLocation& out) {
    WiFiClientSecure client;
    client.setInsecure();  // HermesLLM と同じ。証明書の検証は行っていない
    HTTPClient http;
    http.setTimeout(kHttpTimeoutMs);
    if (!http.begin(client, base_url + "/location")) return false;
    http.addHeader("Authorization", "Bearer " + token);

    const int code = http.GET();
    if (code != 200) {
        Serial.printf("[MAV] location fetch failed: http=%d\n", code);
        http.end();
        return false;
    }
    const String body = http.getString();
    http.end();

    JsonDocument doc;
    if (deserializeJson(doc, body)) return false;
    if (!doc["ok"].as<bool>()) return false;

    const double lat = doc["lat"].as<double>();
    const double lon = doc["lon"].as<double>();
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;

    out.valid = true;
    out.lat = lat;
    out.lon = lon;
    return true;
}

void locationTask(void*) {
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            PhoneLocation fetched;
            if (fetchOnce(fetched)) {
                if (xSemaphoreTake(location_mutex, portMAX_DELAY) == pdTRUE) {
                    latest = fetched;
                    xSemaphoreGive(location_mutex);
                }
                Serial.printf("[MAV] phone location lat=%.5f lon=%.5f\n", fetched.lat, fetched.lon);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(kFetchIntervalMs));
    }
}

}  // namespace

void startPhoneLocationTask(const String& url, const String& tok) {
    if (url.isEmpty() || tok.isEmpty()) return;
    base_url = url;
    token = tok;
    location_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(locationTask, "phone_loc", kTaskStackBytes, nullptr, 1, nullptr, 0);
    Serial.printf("[MAV] phone location: %s\n", base_url.c_str());
}

bool latestPhoneLocation(PhoneLocation& out) {
    if (!location_mutex) return false;
    if (xSemaphoreTake(location_mutex, 0) != pdTRUE) return false;  // 取得中ならこの回は見送る
    out = latest;
    xSemaphoreGive(location_mutex);
    return out.valid;
}
