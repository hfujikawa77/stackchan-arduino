"""MAVStackchan Phase 1 の実機確認用プローブ（GCS の代わりに振る舞う）。

使い方:
    python tools/mav_probe.py <Stack-chan の IP> [--port 14550] [--timeout 10]

確認項目:
  1. 機体の HEARTBEAT（Rover / ArduPilot / DISARMED）をブロードキャストで受信できる
  2. GCS の HEARTBEAT を送ると、以後の HEARTBEAT が学習した送信元へユニキャストで届く
  3. REQUEST_AUTOPILOT_CAPABILITIES に AUTOPILOT_VERSION と COMMAND_ACK(ACCEPTED) を返す
  4. ARM(400) は Phase 1 では未実装のため COMMAND_ACK(UNSUPPORTED) を返し、機体は DISARMED のまま
  5. 別 System ID 宛ての COMMAND_LONG には応答しない

注意:
  - UDP 14550 を占有するため、同じ PC で QGroundControl を開いている場合は閉じてから実行する。
  - 実行には pymavlink が必要（pip install pymavlink）。
"""

import argparse
import socket
import sys
import time

from pymavlink.dialects.v20 import common as mavlink2

MAV_TYPE_GCS = 6
MAV_TYPE_GROUND_ROVER = 10
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_UNSUPPORTED = 3
MAV_MSG_ID_AUTOPILOT_VERSION = 148

DEVICE_SYSID = 1
GCS_SYSID = 255
GCS_COMPID = 190


class UdpWriter:
    """pymavlink の送信先に渡す最小限の file-like オブジェクト。"""

    def __init__(self, sock, addr):
        self.sock = sock
        self.addr = addr

    def write(self, buf):
        self.sock.sendto(buf, self.addr)


class Probe:
    def __init__(self, device_ip, port):
        self.device_addr = (device_ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)
        self.mav = mavlink2.MAVLink(
            UdpWriter(self.sock, self.device_addr), srcSystem=GCS_SYSID, srcComponent=GCS_COMPID
        )
        # (msg, source_addr) を受信順に溜める
        self.received = []
        self.results = []

    def check(self, name, ok, detail=""):
        self.results.append((name, ok, detail))
        mark = "PASS" if ok else "FAIL"
        print(f"[{mark}] {name}" + (f" -- {detail}" if detail else ""))

    def pump(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            for msg in self.mav.parse_buffer(data) or []:
                self.received.append((msg, addr))

    def wait_for(self, predicate, seconds):
        """受信済みメッセージと新着を走査し、条件に合うものを返す。"""
        deadline = time.time() + seconds
        seen = 0
        while time.time() < deadline:
            for msg, addr in self.received[seen:]:
                if predicate(msg, addr):
                    return msg, addr
            seen = len(self.received)
            self.pump(0.1)
        return None, None

    def send_gcs_heartbeat(self):
        self.mav.heartbeat_send(MAV_TYPE_GCS, 8, 0, 0, 4)  # 8 = MAV_AUTOPILOT_INVALID

    def send_command(self, target_sysid, command, *params):
        p = list(params) + [0] * (7 - len(params))
        self.mav.command_long_send(target_sysid, 1, command, 0, *p[:7])


def _text(msg):
    text = msg.text
    return text.decode(errors="replace") if isinstance(text, bytes) else str(text)


def _text_param(msg):
    name = msg.param_id
    return name.decode(errors="replace").rstrip("\x00") if isinstance(name, bytes) else str(name).rstrip("\x00")


def is_device_heartbeat(msg, addr):
    return msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == DEVICE_SYSID and msg.type == MAV_TYPE_GROUND_ROVER


def is_ack(command):
    def _match(msg, _addr):
        return msg.get_type() == "COMMAND_ACK" and msg.command == command
    return _match


def run(device_ip, port, timeout):
    probe = Probe(device_ip, port)
    print(f"[probe] device={device_ip}:{port} listening on 0.0.0.0:{port}")

    # 1. 機体 HEARTBEAT（まだ GCS を学習していないのでブロードキャストで届く想定）
    hb, _ = probe.wait_for(is_device_heartbeat, timeout)
    if hb is None:
        probe.check("device HEARTBEAT received", False, "no Rover HEARTBEAT within timeout")
        return probe.results
    probe.check("device HEARTBEAT received", True,
                f"sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()}")
    probe.check("autopilot is ArduPilotMega", hb.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA,
                f"autopilot={hb.autopilot}")
    probe.check("initially DISARMED", not (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
                f"base_mode=0x{hb.base_mode:02x}")

    # 2. GCS HEARTBEAT を送ると、機体が「link up」の STATUSTEXT を返す（GCS 検知の確認）
    #    ※ 機体はこの後 GCS の送信元へユニキャストで応答する。ブロードキャストと区別できないため、
    #      宛先の確認は機体のシリアルログ "[MAV] GCS peer" で行う。
    link_up = None
    for _ in range(10):
        probe.send_gcs_heartbeat()
        link_up, _ = probe.wait_for(
            lambda m, _a: m.get_type() == "STATUSTEXT" and "link up" in _text(m), 0.5)
        if link_up is not None:
            break
    probe.check("GCS HEARTBEAT detected by device", link_up is not None,
                "" if link_up is not None else "no 'link up' STATUSTEXT (device already linked? retry after 5s)")

    # 3. AUTOPILOT_VERSION の取得
    probe.send_command(DEVICE_SYSID, MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1)
    ack, _ = probe.wait_for(is_ack(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES), 3)
    probe.check("REQUEST_AUTOPILOT_CAPABILITIES ACK", ack is not None and ack.result == MAV_RESULT_ACCEPTED,
                "" if ack is None else f"result={ack.result}")
    av, _ = probe.wait_for(lambda m, _a: m.get_type() == "AUTOPILOT_VERSION", 3)
    probe.check("AUTOPILOT_VERSION received", av is not None,
                "" if av is None else f"capabilities=0x{av.capabilities:x}")

    # 3b. パラメータ一覧（QGC の初期化で必須。ここで応答が無いと QGC が取得待ちで止まる）
    probe.mav.param_request_list_send(DEVICE_SYSID, 1)
    pv, _ = probe.wait_for(lambda m, _a: m.get_type() == "PARAM_VALUE", 3)
    probe.check("PARAM_REQUEST_LIST answered", pv is not None,
                "" if pv is None else f"{_text_param(pv)} count={pv.param_count}")

    # 3c. ATTITUDE（Phase 2）。1 秒間に 5 件以上（20Hz 設定なら約 20 件）届くこと
    #     IMU 未検出時は送らないため、この項目は FAIL になる（シリアルで IMU を確認）
    probe.pump(0.2)
    start = len([m for m, _ in probe.received if m.get_type() == "ATTITUDE"])
    probe.pump(1.0)
    atts = [m for m, _ in probe.received if m.get_type() == "ATTITUDE"]
    rate = len(atts) - start
    probe.check("ATTITUDE streaming (>=5 Hz)", rate >= 5, f"{rate} msgs/s")
    if atts:
        a = atts[-1]
        finite = all(abs(v) < 4 for v in (a.roll, a.pitch, a.yaw))
        probe.check("ATTITUDE values in range (rad)", finite,
                    f"roll={a.roll:.3f} pitch={a.pitch:.3f} yaw={a.yaw:.3f}")

    # 4. ARM は Phase 1 では未実装 → UNSUPPORTED を返し、DISARMED のまま
    probe.send_command(DEVICE_SYSID, MAV_CMD_COMPONENT_ARM_DISARM, 1)
    ack, _ = probe.wait_for(is_ack(MAV_CMD_COMPONENT_ARM_DISARM), 3)
    probe.check("ARM rejected as UNSUPPORTED (Phase 1)",
                ack is not None and ack.result == MAV_RESULT_UNSUPPORTED,
                "" if ack is None else f"result={ack.result}")
    probe.pump(1.5)
    latest = [m for m, a in probe.received if is_device_heartbeat(m, a)][-1:]
    still_disarmed = bool(latest) and not (latest[0].base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
    probe.check("still DISARMED after ARM request", still_disarmed)

    # 5. 別機体宛ての COMMAND_LONG には応答しない
    before = len([m for m, _ in probe.received if m.get_type() == "COMMAND_ACK"])
    probe.send_command(2, MAV_CMD_COMPONENT_ARM_DISARM, 1)
    probe.pump(1.5)
    after = len([m for m, _ in probe.received if m.get_type() == "COMMAND_ACK"])
    probe.check("no reply to other system ID", after == before, f"acks before={before} after={after}")

    return probe.results


def main():
    parser = argparse.ArgumentParser(description="MAVStackchan Phase 1 probe")
    parser.add_argument("device_ip")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    try:
        results = run(args.device_ip, args.port, args.timeout)
    except OSError as err:
        print(f"[probe] socket error: {err} (QGroundControl など 14550 を使うアプリを閉じてください)")
        return 2

    failed = [name for name, ok, _ in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    return 1 if failed or not results else 0


if __name__ == "__main__":
    sys.exit(main())
