#include "Settings.h"

namespace {

// 行末の # コメントを除去する。" の内側の # は値の一部として残す。
String stripComment(const String& line) {
    bool in_quote = false;
    for (unsigned int i = 0; i < line.length(); ++i) {
        const char c = line[i];
        if (c == '"') {
            in_quote = !in_quote;
        } else if (c == '#' && !in_quote) {
            return line.substring(0, i);
        }
    }
    return line;
}

int indentOf(const String& line) {
    int n = 0;
    while (n < static_cast<int>(line.length()) && line[n] == ' ') ++n;
    return n;
}

// "key: value" を分解する。value の前後の空白と、囲みの " は取り除く。
bool splitKeyValue(const String& text, String& key, String& value) {
    const int colon = text.indexOf(':');
    if (colon < 0) return false;

    key = text.substring(0, colon);
    key.trim();
    value = text.substring(colon + 1);
    value.trim();
    if (value.length() >= 2 && value[0] == '"' && value[value.length() - 1] == '"') {
        value = value.substring(1, value.length() - 1);
    }
    return true;
}

}  // namespace

void parseSettingsYaml(const String& yaml, MavSettings& out) {
    out.wifi_count = 0;

    String section;
    int entry = -1;  // wifi_list 内で現在処理中の項目（-1 は対象外）
    int pos = 0;
    while (pos < static_cast<int>(yaml.length())) {
        int end = yaml.indexOf('\n', pos);
        if (end < 0) end = yaml.length();
        const String line = stripComment(yaml.substring(pos, end));
        pos = end + 1;

        String text = line;
        text.trim();
        if (text.isEmpty()) continue;

        // インデント 0 の行はセクション見出し
        if (indentOf(line) == 0) {
            String key, value;
            splitKeyValue(text, key, value);
            section = key;
            entry = -1;
            // トップレベルの値（HermesLLM と同じ書式）
            if (key == "digest_url" && !value.isEmpty()) {
                while (value.endsWith("/")) value.remove(value.length() - 1);
                out.digest_url = value;
            } else if (key == "digest_token" && !value.isEmpty()) {
                out.digest_token = value;
            } else if (key == "slide_mode") {
                out.slide_mode = (value == "1" || value == "true" || value == "on");
            } else if (key == "volume") {
                const int v = value.toInt();
                if (v >= 0 && v <= 100) out.volume = static_cast<uint8_t>(v);
            }
            continue;
        }

        if (section == "wifi_list") {
            if (text.startsWith("- ")) {
                String key, value;
                splitKeyValue(text.substring(2), key, value);
                if (key == "ssid" && !value.isEmpty() && out.wifi_count < kWifiMaxEntries) {
                    entry = static_cast<int>(out.wifi_count);
                    out.wifi[entry].ssid = value;
                    out.wifi[entry].password = "";
                    ++out.wifi_count;
                } else {
                    entry = -1;
                }
            } else if (entry >= 0) {
                String key, value;
                if (splitKeyValue(text, key, value) && key == "password") {
                    out.wifi[entry].password = value;
                }
            }
        } else if (section == "mav") {
            String key, value;
            if (!splitKeyValue(text, key, value)) continue;
            const int number = value.toInt();
            if (key == "system_id" && number >= 1 && number <= 255) {
                out.system_id = static_cast<uint8_t>(number);
            } else if (key == "udp_port" && number >= 1 && number <= 65535) {
                out.udp_port = static_cast<uint16_t>(number);
            }
        }
    }
}
