#pragma once

#include <Arduino.h>
#include "MavlinkReceiver.h"

class FlightStateMonitor {
public:
    bool update(const MavlinkHeartbeat& heartbeat, String& notification);
    static String vehicleTypeName(uint8_t type);
    static String modeName(uint8_t vehicle_type, uint32_t custom_mode);
    static String modeSpeechName(uint8_t vehicle_type, uint32_t custom_mode);

private:
    bool has_state_ = false;
    bool armed_ = false;
    uint32_t custom_mode_ = 0;
    uint8_t vehicle_type_ = 0;
};
