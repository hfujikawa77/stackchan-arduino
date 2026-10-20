#pragma once

#include <stddef.h>
#include <stdint.h>

#include <common/mavlink.h>

// MAVStackchan の MAVLink 機体側ロジック。
// Arduino / Wi-Fi に依存しない。送信は SendFn に渡し、受信バイト列は onBytes() に渡す。
//
// Phase 1 で実装する範囲:
//   - HEARTBEAT の定期送信（既定 1Hz）。機体は Rover / DISARMED / HOLD 固定
//   - GCS(QGC) の HEARTBEAT 受信による接続検知と、タイムアウトによる切断検知
//   - AUTOPILOT_VERSION の応答（REQUEST_MESSAGE / REQUEST_AUTOPILOT_CAPABILITIES）
//   - それ以外の COMMAND_LONG には COMMAND_ACK(MAV_RESULT_UNSUPPORTED) を返す
//
//   - パラメータ応答（PARAM_REQUEST_LIST / PARAM_REQUEST_READ）。実在する値のみ返す。
//     現状は SYSID_THISMAV（= System ID）の 1 件だけ。ArduPilot の全パラメータは偽装しない。
//   - ミッション: MISSION_REQUEST_LIST には件数 0、MISSION_CLEAR_ALL には ACK を返す（対応ミッションは無し）。
//
//   - ARM/DISARM（COMMAND_LONG 400）と、モード変更（DO_SET_MODE 176 / SET_MODE）。
//     モードは HOLD(4) と MANUAL(0) のみ。仕様は README の「Phase 3 の仕様」を参照。
//   - GCS のリンクが切れたら自動で DISARM する。
//
// 未実装（Phase 4 以降）: MANUAL_CONTROL、サーボ出力、PARAM_SET、強制 ARM。
// 未実装の機能は ACK で拒否し、対応しているように見せないこと。

// ArduRover の custom_mode 値（モードの対応は ArduPilot の Rover 定義で要確認）
enum class VehicleMode : uint32_t {
    Manual = 0,
    Hold = 4,
};

class MavlinkVehicle {
public:
    struct Config {
        uint8_t system_id = 1;       // 既定 1（指示書 4.1）
        uint8_t component_id = 1;    // MAV_COMP_ID_AUTOPILOT1
        uint32_t heartbeat_interval_ms = 1000;
        uint32_t gcs_timeout_ms = 3000;
        uint32_t attitude_interval_ms = 50;  // ATTITUDE 20Hz（指示書 6.2 の 10〜20Hz）
        uint32_t manual_timeout_ms = 500;    // バーチャルスティックの入力がこの間途絶えたら中立（指示書 5.4）
    };

    // 有効なスティック入力。ARM 中かつ MANUAL モードで、タイムアウト内に届いたものだけ active になる。
    struct ManualInput {
        bool active = false;
        int16_t x = 0;
        int16_t y = 0;
    };

    // ATTITUDE で送る姿勢（角度は rad、角速度は rad/s、body 座標系）
    struct AttitudeReport {
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        float rollspeed = 0.0f;
        float pitchspeed = 0.0f;
        float yawspeed = 0.0f;
    };

    using SendFn = void (*)(void* ctx, const uint8_t* data, size_t len);

    MavlinkVehicle(const Config& config, SendFn send, void* ctx);

    // 受信バイト列を解析する。1 パケット分でも分割された断片でもよい。
    void onBytes(const uint8_t* data, size_t len, uint32_t now_ms);

    // 定期処理（HEARTBEAT 送信、GCS タイムアウト判定）。loop() から毎回呼ぶ。
    void tick(uint32_t now_ms);

    bool gcsLinked() const { return gcs_linked_; }
    bool armed() const { return armed_; }
    VehicleMode mode() const { return mode_; }

    // 現在有効なスティック入力を返す。ARM かつ MANUAL かつ新しい入力があるときだけ active。
    ManualInput manualInput(uint32_t now_ms) const;

    // QGC からの水平（レベル）キャリブレーション要求を 1 回だけ取り出す。
    bool takeLevelCalibrationRequest();
    // 水平キャリブレーションの結果を要求元へ返す（成功 / 失敗）。
    void finishLevelCalibration(bool ok);

    // GCS の HEARTBEAT を受けたときに true を返し、フラグを消費する。
    // 呼び出し側は送信先アドレスの学習に使う。
    bool takeGcsHeartbeat();

    // 最新の姿勢を設定する。valid=false（IMU 未初期化など）の間は ATTITUDE を送らない。
    void setAttitude(const AttitudeReport& attitude);
    void clearAttitude() { attitude_valid_ = false; }

    uint32_t heartbeatsSent() const { return heartbeats_sent_; }
    uint32_t attitudesSent() const { return attitudes_sent_; }
    uint32_t messagesReceived() const { return messages_received_; }
    uint32_t commandsUnsupported() const { return commands_unsupported_; }

private:
    void handleMessage(const mavlink_message_t& msg, uint32_t now_ms);
    void handleCommandLong(const mavlink_message_t& msg, const mavlink_command_long_t& cmd);
    void handleParamRequestRead(const mavlink_message_t& msg);
    void handleArmDisarm(const mavlink_message_t& msg, const mavlink_command_long_t& cmd);
    bool requestMode(uint8_t base_mode, uint32_t custom_mode);
    void setArmed(bool armed, const char* reason);
    void setMode(VehicleMode mode);
    void sendParamValue(size_t index);
    void sendMissionCount();
    void sendMissionAck(uint8_t target_sysid, uint8_t target_compid);
    void sendHeartbeat();
    void sendAttitude(uint32_t now_ms);
    void sendAutopilotVersion();
    void sendCommandAck(uint16_t command, uint8_t result, uint8_t target_sysid, uint8_t target_compid);
    void sendStatusText(const char* text);
    void transmit(const mavlink_message_t& msg);

    Config config_;
    SendFn send_;
    void* ctx_;

    bool gcs_linked_ = false;
    bool gcs_heartbeat_pending_ = false;

    bool level_request_pending_ = false;
    uint8_t level_requester_sysid_ = 0;
    uint8_t level_requester_compid_ = 0;

    bool manual_received_ = false;
    int16_t manual_x_ = 0;
    int16_t manual_y_ = 0;
    uint32_t manual_ms_ = 0;

    VehicleMode mode_ = VehicleMode::Hold;  // 起動時は HOLD
    bool armed_ = false;                    // 起動時は DISARMED（再起動で引き継がない）
    uint32_t gcs_last_ms_ = 0;
    uint32_t next_heartbeat_ms_ = 0;

    AttitudeReport attitude_;
    bool attitude_valid_ = false;
    uint32_t next_attitude_ms_ = 0;

    uint32_t heartbeats_sent_ = 0;
    uint32_t attitudes_sent_ = 0;
    uint32_t messages_received_ = 0;
    uint32_t commands_unsupported_ = 0;
};
