#include "FlightStateMonitor.h"

static bool isCopterType(uint8_t type) {
    return type == 2 || type == 3 || type == 4 || type == 13 || type == 14 || type == 15;
}

static bool isRoverType(uint8_t type) {
    return type == 10 || type == 11;
}

String FlightStateMonitor::vehicleTypeName(uint8_t type) {
    switch (type) {
        case 1: return "FIXED_WING";
        case 2: return "QUADROTOR";
        case 3: return "COAXIAL";
        case 4: return "HELICOPTER";
        case 10: return "GROUND_ROVER";
        case 11: return "SURFACE_BOAT";
        case 13: return "HEXAROTOR";
        case 14: return "OCTOROTOR";
        case 15: return "TRICOPTER";
        default: return "MAV_TYPE(" + String(type) + ")";
    }
}

static String copterModeName(uint32_t custom_mode) {
    switch (custom_mode) {
        case 0: return "STABILIZE";
        case 1: return "ACRO";
        case 2: return "ALT_HOLD";
        case 3: return "AUTO";
        case 4: return "GUIDED";
        case 5: return "LOITER";
        case 6: return "RTL";
        case 7: return "CIRCLE";
        case 9: return "LAND";
        case 11: return "DRIFT";
        case 13: return "SPORT";
        case 14: return "FLIP";
        case 15: return "AUTOTUNE";
        case 16: return "POSHOLD";
        case 17: return "BRAKE";
        case 18: return "THROW";
        case 19: return "AVOID_ADSB";
        case 20: return "GUIDED_NOGPS";
        case 21: return "SMART_RTL";
        case 22: return "FLOWHOLD";
        case 23: return "FOLLOW";
        case 24: return "ZIGZAG";
        case 25: return "SYSTEMID";
        case 26: return "AUTOROTATE";
        default: return "UNKNOWN(" + String(custom_mode) + ")";
    }
}

static String copterModeSpeechName(uint32_t custom_mode) {
    switch (custom_mode) {
        case 0: return "スタビライズ";
        case 1: return "アクロ";
        case 2: return "アルトホールド";
        case 3: return "オート";
        case 4: return "ガイデッド";
        case 5: return "ロイター";
        case 6: return "アールティーエル";
        case 7: return "サークル";
        case 9: return "ランド";
        case 11: return "ドリフト";
        case 13: return "スポーツ";
        case 14: return "フリップ";
        case 15: return "オートチューン";
        case 16: return "ポジションホールド";
        case 17: return "ブレーキ";
        case 18: return "スロー";
        case 19: return "エーディーエスビー回避";
        case 20: return "ガイデッド ノー ジーピーエス";
        case 21: return "スマート アールティーエル";
        case 22: return "フローホールド";
        case 23: return "フォロー";
        case 24: return "ジグザグ";
        case 25: return "システム アイディー";
        case 26: return "オートローテート";
        default: return "未対応モード " + String(custom_mode);
    }
}

static String roverModeName(uint32_t custom_mode) {
    switch (custom_mode) {
        case 0: return "MANUAL";
        case 1: return "ACRO";
        case 3: return "STEERING";
        case 4: return "HOLD";
        case 5: return "LOITER";
        case 6: return "FOLLOW";
        case 7: return "SIMPLE";
        case 8: return "DOCK";
        case 9: return "CIRCLE";
        case 10: return "AUTO";
        case 11: return "RTL";
        case 12: return "SMART_RTL";
        case 15: return "GUIDED";
        case 16: return "INITIALISING";
        default: return "UNKNOWN(" + String(custom_mode) + ")";
    }
}

static String roverModeSpeechName(uint32_t custom_mode) {
    switch (custom_mode) {
        case 0: return "マニュアル";
        case 1: return "アクロ";
        case 3: return "ステアリング";
        case 4: return "ホールド";
        case 5: return "ロイター";
        case 6: return "フォロー";
        case 7: return "シンプル";
        case 8: return "ドック";
        case 9: return "サークル";
        case 10: return "オート";
        case 11: return "アールティーエル";
        case 12: return "スマート アールティーエル";
        case 15: return "ガイデッド";
        case 16: return "イニシャライズ中";
        default: return "未対応モード " + String(custom_mode);
    }
}

String FlightStateMonitor::modeName(uint8_t vehicle_type, uint32_t custom_mode) {
    if (isRoverType(vehicle_type)) return roverModeName(custom_mode);
    if (isCopterType(vehicle_type)) return copterModeName(custom_mode);
    return "UNKNOWN(" + String(custom_mode) + ")";
}

String FlightStateMonitor::modeSpeechName(uint8_t vehicle_type, uint32_t custom_mode) {
    if (isRoverType(vehicle_type)) return roverModeSpeechName(custom_mode);
    if (isCopterType(vehicle_type)) return copterModeSpeechName(custom_mode);
    return "未対応モード " + String(custom_mode);
}

bool FlightStateMonitor::update(const MavlinkHeartbeat& heartbeat, String& notification) {
    notification = "";

    if (!has_state_) {
        has_state_ = true;
        armed_ = heartbeat.armed;
        custom_mode_ = heartbeat.custom_mode;
        vehicle_type_ = heartbeat.type;
        Serial.println("[MAVLink] vehicle=" + vehicleTypeName(vehicle_type_)
                       + " mode=" + modeName(vehicle_type_, custom_mode_));
        notification = String(armed_ ? "アーム中です" : "ディスアーム中です")
                     + "。"
                     + modeSpeechName(vehicle_type_, custom_mode_)
                     + "モードです";
        return true;
    }

    if (vehicle_type_ != heartbeat.type) {
        vehicle_type_ = heartbeat.type;
        Serial.println("[MAVLink] vehicle changed: " + vehicleTypeName(vehicle_type_));
    }

    if (armed_ != heartbeat.armed) {
        armed_ = heartbeat.armed;
        notification = armed_ ? "アームしました" : "ディスアームしました";
    }

    if (custom_mode_ != heartbeat.custom_mode) {
        custom_mode_ = heartbeat.custom_mode;
        Serial.println("[MAVLink] mode changed: " + modeName(vehicle_type_, custom_mode_));
        if (!notification.isEmpty()) notification += "。";
        notification += modeSpeechName(vehicle_type_, custom_mode_) + "モードです";
    }

    return !notification.isEmpty();
}
