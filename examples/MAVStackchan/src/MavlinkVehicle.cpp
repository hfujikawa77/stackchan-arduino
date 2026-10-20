#include "MavlinkVehicle.h"

#include <string.h>

namespace {

// AUTOPILOT_VERSION の custom_version に入れる識別子（8 バイト固定）。
constexpr char kCustomVersionTag[8] = {'M', 'A', 'V', 'S', 't', 'k', 0, 0};

// 機体が実際に持っているパラメータ。値は実体から取る（捏造しない）。
struct ParamEntry {
    const char* id;
    uint8_t type;
};
constexpr ParamEntry kParams[] = {
    {"SYSID_THISMAV", MAV_PARAM_TYPE_REAL32},
};
constexpr size_t kParamCount = sizeof(kParams) / sizeof(kParams[0]);

// 受信パーサは 1 チャンネル分のみ使う（MavlinkVehicle は 1 インスタンス想定）。
mavlink_status_t rx_status;

}  // namespace

MavlinkVehicle::MavlinkVehicle(const Config& config, SendFn send, void* ctx)
    : config_(config), send_(send), ctx_(ctx) {}

void MavlinkVehicle::onBytes(const uint8_t* data, size_t len, uint32_t now_ms) {
    mavlink_message_t msg;
    for (size_t i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &rx_status)) {
            ++messages_received_;
            handleMessage(msg, now_ms);
        }
    }
}

void MavlinkVehicle::tick(uint32_t now_ms) {
    if (now_ms >= next_heartbeat_ms_) {
        sendHeartbeat();
        next_heartbeat_ms_ = now_ms + config_.heartbeat_interval_ms;
    }

    if (attitude_valid_ && now_ms >= next_attitude_ms_) {
        sendAttitude(now_ms);
        next_attitude_ms_ = now_ms + config_.attitude_interval_ms;
    }

    if (gcs_linked_ && (now_ms - gcs_last_ms_) > config_.gcs_timeout_ms) {
        gcs_linked_ = false;
        // 安全側: GCS との通信が切れたら DISARM する（再接続しても自動では ARM しない）
        setArmed(false, "GCS link lost: DISARMED");
    }
}

void MavlinkVehicle::setAttitude(const AttitudeReport& attitude) {
    attitude_ = attitude;
    attitude_valid_ = true;
}

void MavlinkVehicle::sendAttitude(uint32_t now_ms) {
    mavlink_attitude_t att;
    memset(&att, 0, sizeof(att));
    att.time_boot_ms = now_ms;
    att.roll = attitude_.roll;
    att.pitch = attitude_.pitch;
    att.yaw = attitude_.yaw;
    att.rollspeed = attitude_.rollspeed;
    att.pitchspeed = attitude_.pitchspeed;
    att.yawspeed = attitude_.yawspeed;

    mavlink_message_t msg;
    mavlink_msg_attitude_encode(config_.system_id, config_.component_id, &msg, &att);
    transmit(msg);
    ++attitudes_sent_;
}

bool MavlinkVehicle::takeGcsHeartbeat() {
    const bool pending = gcs_heartbeat_pending_;
    gcs_heartbeat_pending_ = false;
    return pending;
}

void MavlinkVehicle::handleMessage(const mavlink_message_t& msg, uint32_t now_ms) {
    // 自分の送信分（ループバック等）は無視する
    if (msg.sysid == config_.system_id) return;

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            if (hb.type != MAV_TYPE_GCS) return;

            gcs_last_ms_ = now_ms;
            gcs_heartbeat_pending_ = true;
            if (!gcs_linked_) {
                gcs_linked_ = true;
                sendStatusText("MAVStackchan: link up (Phase 1 HEARTBEAT only)");
            }
            return;
        }

        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&msg, &cmd);
            // 宛先が別機体の場合は応答しない
            if (cmd.target_system != 0 && cmd.target_system != config_.system_id) return;
            handleCommandLong(msg, cmd);
            return;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            mavlink_param_request_list_t req;
            mavlink_msg_param_request_list_decode(&msg, &req);
            if (req.target_system != 0 && req.target_system != config_.system_id) return;
            for (size_t i = 0; i < kParamCount; ++i) {
                sendParamValue(i);
            }
            return;
        }

        case MAVLINK_MSG_ID_MANUAL_CONTROL: {
            mavlink_manual_control_t mc;
            mavlink_msg_manual_control_decode(&msg, &mc);
            if (mc.target != 0 && mc.target != config_.system_id) return;
            manual_x_ = mc.x;
            manual_y_ = mc.y;
            manual_ms_ = now_ms;
            manual_received_ = true;
            return;
        }

        case MAVLINK_MSG_ID_SET_MODE: {
            mavlink_set_mode_t sm;
            mavlink_msg_set_mode_decode(&msg, &sm);
            if (sm.target_system != 0 && sm.target_system != config_.system_id) return;
            // SET_MODE には応答メッセージが無いため、結果は HEARTBEAT と STATUSTEXT で伝える
            if (!requestMode(sm.base_mode, sm.custom_mode)) {
                sendStatusText("MODE REJECTED (unsupported)");
            }
            return;
        }

        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
            // ミッションは未対応。件数 0 を返す（QGC は接続時にこれを要求する）
            mavlink_mission_request_list_t req;
            mavlink_msg_mission_request_list_decode(&msg, &req);
            if (req.target_system != 0 && req.target_system != config_.system_id) return;
            sendMissionCount();
            return;
        }

        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
            // 機体側のミッションは元々空なので、消去要求は受理する
            mavlink_mission_clear_all_t req;
            mavlink_msg_mission_clear_all_decode(&msg, &req);
            if (req.target_system != 0 && req.target_system != config_.system_id) return;
            sendMissionAck(msg.sysid, msg.compid);
            return;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            mavlink_param_request_read_t req;
            mavlink_msg_param_request_read_decode(&msg, &req);
            if (req.target_system != 0 && req.target_system != config_.system_id) return;
            handleParamRequestRead(msg);
            return;
        }

        default:
            return;
    }
}

void MavlinkVehicle::handleParamRequestRead(const mavlink_message_t& msg) {
    mavlink_param_request_read_t req;
    mavlink_msg_param_request_read_decode(&msg, &req);

    // param_index が有効な範囲ならそれを使い、-1 なら名前で探す
    if (req.param_index >= 0 && static_cast<size_t>(req.param_index) < kParamCount) {
        sendParamValue(static_cast<size_t>(req.param_index));
        return;
    }
    for (size_t i = 0; i < kParamCount; ++i) {
        if (strncmp(req.param_id, kParams[i].id, sizeof(req.param_id)) == 0) {
            sendParamValue(i);
            return;
        }
    }
    // 該当しないパラメータには応答しない（存在しない値を返さない）
}

void MavlinkVehicle::sendMissionCount() {
    mavlink_mission_count_t mc;
    memset(&mc, 0, sizeof(mc));
    mc.count = 0;
    mc.target_system = 0;
    mc.target_component = 0;
    mc.mission_type = MAV_MISSION_TYPE_MISSION;

    mavlink_message_t msg;
    mavlink_msg_mission_count_encode(config_.system_id, config_.component_id, &msg, &mc);
    transmit(msg);
}

void MavlinkVehicle::sendMissionAck(uint8_t target_sysid, uint8_t target_compid) {
    mavlink_mission_ack_t ack;
    memset(&ack, 0, sizeof(ack));
    ack.target_system = target_sysid;
    ack.target_component = target_compid;
    ack.type = MAV_MISSION_ACCEPTED;
    ack.mission_type = MAV_MISSION_TYPE_MISSION;

    mavlink_message_t msg;
    mavlink_msg_mission_ack_encode(config_.system_id, config_.component_id, &msg, &ack);
    transmit(msg);
}

void MavlinkVehicle::handleArmDisarm(const mavlink_message_t& msg, const mavlink_command_long_t& cmd) {
    const float p1 = cmd.param1;
    if (p1 != 0.0f && p1 != 1.0f) {  // 0 = DISARM, 1 = ARM のみ受け付ける
        sendCommandAck(cmd.command, MAV_RESULT_DENIED, msg.sysid, msg.compid);
        return;
    }

    if (p1 == 0.0f) {
        setArmed(false, "DISARMED");
        sendCommandAck(cmd.command, MAV_RESULT_ACCEPTED, msg.sysid, msg.compid);
        return;
    }

    // ARM の条件: GCS のリンクが生きていること。
    // サーボの初期化確認とフェイルセーフ判定は Phase 4 で追加する（Phase 3 はアクチュエータ無し）。
    if (!gcs_linked_) {
        sendCommandAck(cmd.command, MAV_RESULT_DENIED, msg.sysid, msg.compid);
        return;
    }
    setArmed(true, "ARMED");
    sendCommandAck(cmd.command, MAV_RESULT_ACCEPTED, msg.sysid, msg.compid);
}

bool MavlinkVehicle::requestMode(uint8_t base_mode, uint32_t custom_mode) {
    if (!(base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) return false;

    if (custom_mode == static_cast<uint32_t>(VehicleMode::Manual)) {
        setMode(VehicleMode::Manual);
        return true;
    }
    if (custom_mode == static_cast<uint32_t>(VehicleMode::Hold)) {
        setMode(VehicleMode::Hold);
        return true;
    }
    return false;  // 対応していないモードは拒否する
}

MavlinkVehicle::ManualInput MavlinkVehicle::manualInput(uint32_t now_ms) const {
    ManualInput in;
    if (!manual_received_ || !armed_ || mode_ != VehicleMode::Manual) return in;
    if ((now_ms - manual_ms_) > config_.manual_timeout_ms) return in;  // 入力が途絶えた
    in.active = true;
    in.x = manual_x_;
    in.y = manual_y_;
    return in;
}

void MavlinkVehicle::setArmed(bool armed, const char* reason) {
    if (armed == armed_) return;
    armed_ = armed;
    sendStatusText(reason);
}

void MavlinkVehicle::setMode(VehicleMode mode) {
    if (mode == mode_) return;
    mode_ = mode;
    sendStatusText(mode_ == VehicleMode::Manual ? "MODE MANUAL" : "MODE HOLD");
}

void MavlinkVehicle::sendParamValue(size_t index) {
    mavlink_param_value_t pv;
    memset(&pv, 0, sizeof(pv));
    pv.param_value = static_cast<float>(config_.system_id);  // SYSID_THISMAV
    pv.param_count = static_cast<uint16_t>(kParamCount);
    pv.param_index = static_cast<uint16_t>(index);
    strncpy(pv.param_id, kParams[index].id, sizeof(pv.param_id));
    pv.param_type = kParams[index].type;

    mavlink_message_t msg;
    mavlink_msg_param_value_encode(config_.system_id, config_.component_id, &msg, &pv);
    transmit(msg);
}

void MavlinkVehicle::handleCommandLong(const mavlink_message_t& msg, const mavlink_command_long_t& cmd) {
    switch (cmd.command) {
        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            if (cmd.param1 == 1.0f) {
                sendAutopilotVersion();
                sendCommandAck(cmd.command, MAV_RESULT_ACCEPTED, msg.sysid, msg.compid);
            } else {
                sendCommandAck(cmd.command, MAV_RESULT_DENIED, msg.sysid, msg.compid);
            }
            return;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            handleArmDisarm(msg, cmd);
            return;

        case MAV_CMD_DO_SET_MODE:
            if (requestMode(static_cast<uint8_t>(cmd.param1), static_cast<uint32_t>(cmd.param2))) {
                sendCommandAck(cmd.command, MAV_RESULT_ACCEPTED, msg.sysid, msg.compid);
            } else {
                sendCommandAck(cmd.command, MAV_RESULT_DENIED, msg.sysid, msg.compid);
            }
            return;

        case MAV_CMD_REQUEST_MESSAGE:
            if (static_cast<uint32_t>(cmd.param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
                sendAutopilotVersion();
                sendCommandAck(cmd.command, MAV_RESULT_ACCEPTED, msg.sysid, msg.compid);
            } else {
                ++commands_unsupported_;
                sendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED, msg.sysid, msg.compid);
            }
            return;

        default:
            // SET_MESSAGE_INTERVAL などは未対応（ストリーム設定は Phase 4 以降で検討）
            ++commands_unsupported_;
            sendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED, msg.sysid, msg.compid);
            return;
    }
}

void MavlinkVehicle::sendHeartbeat() {
    mavlink_heartbeat_t hb;
    memset(&hb, 0, sizeof(hb));
    hb.type = MAV_TYPE_GROUND_ROVER;
    hb.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (armed_) base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    hb.base_mode = base_mode;
    hb.custom_mode = static_cast<uint32_t>(mode_);
    hb.system_status = armed_ ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_encode(config_.system_id, config_.component_id, &msg, &hb);
    transmit(msg);
    ++heartbeats_sent_;
}

void MavlinkVehicle::sendAutopilotVersion() {
    // バージョン番号・UID は未設定（0）のまま。捏造しないこと。
    mavlink_autopilot_version_t av;
    memset(&av, 0, sizeof(av));
    av.capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2;
    memcpy(av.flight_custom_version, kCustomVersionTag, sizeof(kCustomVersionTag));

    mavlink_message_t msg;
    mavlink_msg_autopilot_version_encode(config_.system_id, config_.component_id, &msg, &av);
    transmit(msg);
}

void MavlinkVehicle::sendCommandAck(uint16_t command, uint8_t result, uint8_t target_sysid, uint8_t target_compid) {
    mavlink_command_ack_t ack;
    memset(&ack, 0, sizeof(ack));
    ack.command = command;
    ack.result = result;
    ack.progress = UINT8_MAX;  // 進捗は未使用
    ack.target_system = target_sysid;
    ack.target_component = target_compid;

    mavlink_message_t msg;
    mavlink_msg_command_ack_encode(config_.system_id, config_.component_id, &msg, &ack);
    transmit(msg);
}

void MavlinkVehicle::sendStatusText(const char* text) {
    mavlink_statustext_t st;
    memset(&st, 0, sizeof(st));
    st.severity = MAV_SEVERITY_INFO;
    strncpy(st.text, text, sizeof(st.text) - 1);

    mavlink_message_t msg;
    mavlink_msg_statustext_encode(config_.system_id, config_.component_id, &msg, &st);
    transmit(msg);
}

void MavlinkVehicle::transmit(const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (send_) send_(ctx_, buf, len);
}
