#pragma once

#include <Arduino.h>

struct MavlinkHeartbeat {
    bool received = false;
    bool armed = false;
    uint32_t custom_mode = 0;
    uint8_t base_mode = 0;
    uint8_t type = 0;
    uint8_t autopilot = 0;
    uint8_t system_status = 0;
};

struct MavlinkAttitude {
    bool received = false;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    uint32_t time_boot_ms = 0;
};

struct MavlinkRcChannels {
    bool received = false;
    uint16_t ch1_raw = 0;
    uint16_t ch2_raw = 0;
    uint16_t ch5_raw = 0;
    uint16_t ch6_raw = 0;
    uint16_t ch9_raw = 0;
    uint32_t time_boot_ms = 0;
};

class MavlinkReceiver {
public:
    void begin(uint32_t baud = 115200, int8_t rx_pin = 18, int8_t tx_pin = 17);
    bool pollHeartbeat(MavlinkHeartbeat& heartbeat);
    bool poll(MavlinkHeartbeat* heartbeat, MavlinkAttitude* attitude, MavlinkRcChannels* rc_channels = nullptr);

private:
    bool started_ = false;
};
