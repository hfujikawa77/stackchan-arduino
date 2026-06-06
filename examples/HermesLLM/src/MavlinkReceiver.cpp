#include "MavlinkReceiver.h"

#include <HardwareSerial.h>
#include <common/mavlink.h>

#ifndef MAVLINK_UART_NUM
#define MAVLINK_UART_NUM 1
#endif

static HardwareSerial mavSerial(MAVLINK_UART_NUM);
static mavlink_message_t mav_msg;
static mavlink_status_t mav_status;
static uint32_t mav_rx_bytes = 0;
static uint32_t mav_rx_bytes_last = 0;
static uint32_t mav_parsed_messages = 0;
static uint32_t mav_parsed_messages_last = 0;
static uint32_t mav_heartbeats = 0;
static uint32_t mav_heartbeats_last = 0;
static uint32_t mav_attitudes = 0;
static uint32_t mav_attitudes_last = 0;
static uint32_t mav_rc_channels = 0;
static uint32_t mav_rc_channels_last = 0;
static uint32_t mav_next_stats_ms = 0;

void MavlinkReceiver::begin(uint32_t baud, int8_t rx_pin, int8_t tx_pin) {
    rx_pin_ = rx_pin;
    mavSerial.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
    started_ = true;
    Serial.printf("[MAVLink] UART%d started: baud=%lu rx=%d tx=%d\n",
                  MAVLINK_UART_NUM,
                  static_cast<unsigned long>(baud),
                  rx_pin,
                  tx_pin);
}

bool MavlinkReceiver::pollHeartbeat(MavlinkHeartbeat& heartbeat) {
    return poll(&heartbeat, nullptr);
}

bool MavlinkReceiver::poll(MavlinkHeartbeat* heartbeat, MavlinkAttitude* attitude, MavlinkRcChannels* rc_channels) {
    if (!started_) return false;

    bool found = false;
    while (mavSerial.available()) {
        const uint8_t c = static_cast<uint8_t>(mavSerial.read());
        mav_rx_bytes++;
        if (!mavlink_parse_char(MAVLINK_COMM_0, c, &mav_msg, &mav_status)) {
            continue;
        }

        mav_parsed_messages++;
        if (mav_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && heartbeat) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&mav_msg, &hb);
            mav_heartbeats++;
            heartbeat->received = true;
            heartbeat->armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            heartbeat->custom_mode = hb.custom_mode;
            heartbeat->base_mode = hb.base_mode;
            heartbeat->type = hb.type;
            heartbeat->autopilot = hb.autopilot;
            heartbeat->system_status = hb.system_status;
            Serial.printf("[MAVLink] HEARTBEAT armed=%u custom_mode=%lu base_mode=0x%02x status=%u\n",
                          heartbeat->armed ? 1 : 0,
                          static_cast<unsigned long>(heartbeat->custom_mode),
                          heartbeat->base_mode,
                          heartbeat->system_status);
            found = true;
        } else if (mav_msg.msgid == MAVLINK_MSG_ID_ATTITUDE && attitude) {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&mav_msg, &att);
            attitude->received = true;
            attitude->roll = att.roll;
            attitude->pitch = att.pitch;
            attitude->yaw = att.yaw;
            attitude->time_boot_ms = att.time_boot_ms;
            mav_attitudes++;
            found = true;
        } else if (mav_msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS && rc_channels) {
            mavlink_rc_channels_t rc;
            mavlink_msg_rc_channels_decode(&mav_msg, &rc);
            rc_channels->received = true;
            rc_channels->ch1_raw = rc.chan1_raw;
            rc_channels->ch2_raw = rc.chan2_raw;
            rc_channels->ch5_raw = rc.chan5_raw;
            rc_channels->ch6_raw = rc.chan6_raw;
            rc_channels->ch9_raw = rc.chan9_raw;
            rc_channels->time_boot_ms = rc.time_boot_ms;
            mav_rc_channels++;
            found = true;
        }
    }

    const uint32_t now = millis();
    if (mav_next_stats_ms == 0 || static_cast<int32_t>(now - mav_next_stats_ms) >= 0) {
        const uint32_t rx_delta = mav_rx_bytes - mav_rx_bytes_last;
        const uint32_t parsed_delta = mav_parsed_messages - mav_parsed_messages_last;
        const uint32_t heartbeat_delta = mav_heartbeats - mav_heartbeats_last;
        const uint32_t attitude_delta = mav_attitudes - mav_attitudes_last;
        const uint32_t rc_channels_delta = mav_rc_channels - mav_rc_channels_last;
        Serial.printf("[MAVLink] stats uart=%d rx=%d rx_bytes=%lu(+%lu) parsed=%lu(+%lu) heartbeats=%lu(+%lu) attitudes=%lu(+%lu) rc_channels=%lu(+%lu)\n",
                      MAVLINK_UART_NUM,
                      rx_pin_,
                      static_cast<unsigned long>(mav_rx_bytes),
                      static_cast<unsigned long>(rx_delta),
                      static_cast<unsigned long>(mav_parsed_messages),
                      static_cast<unsigned long>(parsed_delta),
                      static_cast<unsigned long>(mav_heartbeats),
                      static_cast<unsigned long>(heartbeat_delta),
                      static_cast<unsigned long>(mav_attitudes),
                      static_cast<unsigned long>(attitude_delta),
                      static_cast<unsigned long>(mav_rc_channels),
                      static_cast<unsigned long>(rc_channels_delta));
        mav_rx_bytes_last = mav_rx_bytes;
        mav_parsed_messages_last = mav_parsed_messages;
        mav_heartbeats_last = mav_heartbeats;
        mav_attitudes_last = mav_attitudes;
        mav_rc_channels_last = mav_rc_channels;
        mav_next_stats_ms = now + 5000;
    }

    return found;
}
