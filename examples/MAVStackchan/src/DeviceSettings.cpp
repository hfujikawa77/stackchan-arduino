#include "DeviceSettings.h"

#include <ArduinoJson.h>
#include <SPIFFS.h>

namespace {
constexpr char kPath[] = "/settings.json";
}

bool loadDeviceSettings(DeviceSettings& out) {
    if (!SPIFFS.exists(kPath)) return false;
    File f = SPIFFS.open(kPath, "r");
    if (!f) return false;
    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) return false;

    out.slide_mode = doc["slide_mode"] | out.slide_mode;
    out.volume = doc["volume"] | out.volume;
    out.brightness = doc["brightness"] | out.brightness;
    out.wifi_index = doc["wifi_index"] | out.wifi_index;
    out.mav_enabled = doc["mav_enabled"] | out.mav_enabled;
    out.cue_beep = doc["cue_beep"] | out.cue_beep;
    out.greeting_on = doc["greeting_on"] | out.greeting_on;
    return true;
}

bool saveDeviceSettings(const DeviceSettings& in) {
    JsonDocument doc;
    doc["slide_mode"] = in.slide_mode;
    doc["volume"] = in.volume;
    doc["brightness"] = in.brightness;
    doc["wifi_index"] = in.wifi_index;
    doc["mav_enabled"] = in.mav_enabled;
    doc["cue_beep"] = in.cue_beep;
    doc["greeting_on"] = in.greeting_on;

    File f = SPIFFS.open(kPath, "w");
    if (!f) return false;
    const size_t n = serializeJson(doc, f);
    f.close();
    return n > 0;
}
