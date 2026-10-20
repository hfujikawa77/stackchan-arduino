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

// RC の PWM（1000〜2000 µs、中立 1500）を MANUAL_CONTROL と同じ -1000〜1000 に変換する。
// 0 や UINT16_MAX は「未使用」なので中立（0）にする。
int16_t rcToStick(uint16_t pwm) {
    if (pwm == 0 || pwm == UINT16_MAX) return 0;
    int value = (static_cast<int>(pwm) - 1500) * 2;
    if (value < -1000) value = -1000;
    if (value > 1000) value = 1000;
    return static_cast<int16_t>(value);
}

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

    if (position_valid_ && now_ms >= next_position_ms_) {
        sendGlobalPositionInt(now_ms);
        next_position_ms_ = now_ms + config_.position_interval_ms;
    }

    if (now_ms >= next_sys_status_ms_) {
        sendSysStatus();
        if (system_.wifi_valid) sendRadioStatus();
        // QGC 4.0 以降は電池を BATTERY_STATUS から読むため、SYS_STATUS と併せて送る
        if (system_.battery_valid) sendBatteryStatus();
        next_sys_status_ms_ = now_ms + config_.sys_status_interval_ms;
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

void MavlinkVehicle::sendSysStatus() {
    mavlink_sys_status_t st;
    memset(&st, 0, sizeof(st));

    // IMU が使える場合だけ、ジャイロと加速度のセンサとして報告する
    if (system_.imu_ok) {
        const uint32_t imu_bits = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        st.onboard_control_sensors_present = imu_bits;
        st.onboard_control_sensors_enabled = imu_bits;
        st.onboard_control_sensors_health = imu_bits;
    }

    // 不明な値は「未設定」にする。電流は測っていないので常に不明。
    st.voltage_battery = system_.battery_valid ? system_.battery_mv : UINT16_MAX;
    st.current_battery = -1;
    st.battery_remaining = system_.battery_valid ? system_.battery_pct : -1;

    mavlink_message_t msg;
    mavlink_msg_sys_status_encode(config_.system_id, config_.component_id, &msg, &st);
    transmit(msg);
}

void MavlinkVehicle::setPosition(double lat_deg, double lon_deg) {
    position_lat_ = lat_deg;
    position_lon_ = lon_deg;
    position_valid_ = true;
}

void MavlinkVehicle::sendGlobalPositionInt(uint32_t now_ms) {
    mavlink_global_position_int_t gp;
    memset(&gp, 0, sizeof(gp));
    gp.time_boot_ms = now_ms;
    gp.lat = static_cast<int32_t>(position_lat_ * 1e7);
    gp.lon = static_cast<int32_t>(position_lon_ * 1e7);
    // 高度: 気圧センサーは未接続のため、未計測として 0 を入れている（後で相対高度に置き換える）
    gp.alt = 0;
    gp.relative_alt = 0;
    // 速度・方位は未計測のため 0 / 不明（UINT16_MAX）
    gp.vx = 0;
    gp.vy = 0;
    gp.vz = 0;
    gp.hdg = UINT16_MAX;

    mavlink_message_t msg;
    mavlink_msg_global_position_int_encode(config_.system_id, config_.component_id, &msg, &gp);
    transmit(msg);
}

void MavlinkVehicle::sendBatteryStatus() {
    mavlink_battery_status_t bs;
    memset(&bs, 0, sizeof(bs));
    bs.id = 0;                                   // 電池 1 本
    bs.battery_function = MAV_BATTERY_FUNCTION_ALL;
    bs.type = MAV_BATTERY_TYPE_UNKNOWN;          // 種類（Li-ion 等）は未確認
    bs.temperature = INT16_MAX;                  // 温度は測っていない（未知）
    for (int i = 0; i < 10; ++i) bs.voltages[i] = UINT16_MAX;  // 未使用セル = 未知
    bs.voltages[0] = system_.battery_mv;         // 電池全体の電圧を 1 つ目に入れる
    bs.current_battery = -1;                     // 電流は測っていない
    bs.current_consumed = -1;
    bs.energy_consumed = -1;
    bs.battery_remaining = system_.battery_pct;  // 不明なら -1
    bs.time_remaining = 0;                       // 残り時間は計算していない（未知）
    bs.charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED;

    mavlink_message_t msg;
    mavlink_msg_battery_status_encode(config_.system_id, config_.component_id, &msg, &bs);
    transmit(msg);
}

void MavlinkVehicle::sendRadioStatus() {
    // RADIO_STATUS の rssi は 0〜100 の目安値。Wi-Fi の dBm から仮に換算する:
    //   -100 dBm → 0, -50 dBm → 100（範囲外は丸める）。換算式は QGC 側の見え方で要確認。
    const int dbm = system_.wifi_rssi_dbm;
    int pct = (dbm + 100) * 2;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    mavlink_radio_status_t rs;
    memset(&rs, 0, sizeof(rs));
    rs.rssi = static_cast<uint8_t>(pct);
    rs.remrssi = UINT8_MAX;   // 相手側の値は取れない（未知）
    rs.noise = UINT8_MAX;     // ノイズは測っていない（未知）
    rs.remnoise = UINT8_MAX;
    rs.txbuf = 0;             // 送信バッファの使用率は測っていない

    mavlink_message_t msg;
    mavlink_msg_radio_status_encode(config_.system_id, config_.component_id, &msg, &rs);
    transmit(msg);
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

        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
            // MissionPlanner のジョイスティックはこのメッセージで送る（QGC は MANUAL_CONTROL）
            mavlink_rc_channels_override_t rc;
            mavlink_msg_rc_channels_override_decode(&msg, &rc);
            if (rc.target_system != 0 && rc.target_system != config_.system_id) return;
            rc_chan1_ = rc.chan1_raw;  // パン（RC1 = ロール）
            rc_chan2_ = rc.chan2_raw;  // チルト（RC2 = ピッチ）。RC3 はスロットルのため使わない
            rc_ms_ = now_ms;
            rc_received_ = true;
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

bool MavlinkVehicle::takeLevelCalibrationRequest() {
    const bool pending = level_request_pending_;
    level_request_pending_ = false;
    return pending;
}

void MavlinkVehicle::finishLevelCalibration(bool ok) {
    sendCommandAck(MAV_CMD_PREFLIGHT_CALIBRATION,
                   ok ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED,
                   level_requester_sysid_, level_requester_compid_);
    sendStatusText(ok ? "Level calibration done" : "Level calibration failed: keep still and retry");
}

MavlinkVehicle::ManualInput MavlinkVehicle::manualInput(uint32_t now_ms) const {
    ManualInput in;
    if (!armed_ || mode_ != VehicleMode::Manual) return in;

    const bool mc_fresh = manual_received_ && (now_ms - manual_ms_) <= config_.manual_timeout_ms;
    const bool rc_fresh = rc_received_ && (now_ms - rc_ms_) <= config_.manual_timeout_ms;
    if (!mc_fresh && !rc_fresh) return in;  // 入力が途絶えた

    // 両方来ているときは新しい方を使う
    if (rc_fresh && (!mc_fresh || rc_ms_ >= manual_ms_)) {
        // RC_CHANNELS_OVERRIDE: PWM 1000〜2000 を -1000〜1000 に変換。未使用（0 / UINT16_MAX）は中立
        in.x = rcToStick(rc_chan2_);  // RC2（ピッチ）→ チルト
        in.y = rcToStick(rc_chan1_);  // RC1（ロール）→ パン
    } else {
        in.x = manual_x_;
        in.y = manual_y_;
    }
    in.active = true;
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

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            // param5 = 2: 水平（レベル）キャリブレーション。機体を水平に静止させて基準を取る。
            // ARM 中は実行しない。
            if (armed_) {
                sendStatusText("Disarm to allow calibration");
                sendCommandAck(cmd.command, MAV_RESULT_DENIED, msg.sysid, msg.compid);
                return;
            }
            if (static_cast<int>(cmd.param5) == 2) {
                level_request_pending_ = true;
                level_requester_sysid_ = msg.sysid;
                level_requester_compid_ = msg.compid;
                sendStatusText("Level calibration: keep the device still and level");
                sendCommandAck(cmd.command, MAV_RESULT_IN_PROGRESS, msg.sysid, msg.compid);
                return;
            }
            ++commands_unsupported_;
            sendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED, msg.sysid, msg.compid);
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
