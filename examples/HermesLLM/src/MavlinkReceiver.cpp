#include "MavlinkReceiver.h"

#include <HardwareSerial.h>
#include <common/mavlink.h>

#ifndef MAVLINK_UART_NUM
#define MAVLINK_UART_NUM 1
#endif

static HardwareSerial mavSerial(MAVLINK_UART_NUM);
static mavlink_message_t mav_msg;
static mavlink_status_t mav_status;

void MavlinkReceiver::begin(uint32_t baud, int8_t rx_pin, int8_t tx_pin) {
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
        if (!mavlink_parse_char(MAVLINK_COMM_0, c, &mav_msg, &mav_status)) {
            continue;
        }

        if (mav_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && heartbeat) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&mav_msg, &hb);
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
            found = true;
        }
    }

    return found;
}
