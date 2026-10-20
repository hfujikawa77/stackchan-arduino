# MAVStackchan

Stack-chan（M5Stack CoreS3）を MAVLink 機体として振る舞わせ、Android 版 QGroundControl（QGC）から
認識させる PoC ファームです。開発指示書は `examples/logs/20261020_mav-stackchan/direction.md` を参照してください。

## 現在の状態

| Phase | 内容 | 状態 |
|---|---|---|
| 1 | Wi-Fi UDP、HEARTBEAT 送信、GCS 検知、未対応コマンドへの ACK | **実機で確認済み**（QGC で機体として認識） |
| 2 | IMU 姿勢（ATTITUDE 20Hz）と HUD 表示 | **実機で確認済み**（ロール・ピッチが HUD に追従） |
| 3 | ARM / DISARM、HOLD / MANUAL 切替 | 未着手（現在は ACK で UNSUPPORTED を返す） |
| 4 | Virtual Joystick、サーボ出力、フェイルセーフ | 未着手 |
| 5 | Stack-chan の表情・音声との統合 | 未着手 |

Phase 1 で実装しているもの:

- HEARTBEAT を 1Hz で送信（Rover / ArduPilotMega / DISARMED / HOLD 固定、System ID 1）
- QGC の HEARTBEAT を受けると「link up」の STATUSTEXT を送り、その送信元へ応答を返す
- 3 秒間 QGC の HEARTBEAT が無ければ切断扱いにする
- `REQUEST_AUTOPILOT_CAPABILITIES` / `REQUEST_MESSAGE(AUTOPILOT_VERSION)` に AUTOPILOT_VERSION を返す
  （バージョン番号と UID は未設定の 0。捏造しない）
- それ以外の COMMAND_LONG（ARM など）には `MAV_RESULT_UNSUPPORTED` を返す

Phase 1 でまだ **対応していないもの**（QGC 側で機能が出ても動かない）:

- ARM / DISARM、モード変更、Virtual Joystick、サーボ出力、MAVLink パラメータ一覧
- `PARAM_REQUEST_LIST` には応答しない。QGC がパラメータ取得待ちで止まる可能性があり、要確認

Phase 2 で追加したもの:

- IMU（CoreS3 内蔵 BMI270）を 100Hz で読み、相補フィルタでロール・ピッチを推定
- `ATTITUDE` を 20Hz で送信（ロールとピッチは HUD で確認済み、ヨーは磁気補正なしのためドリフトする）
- 起動直後 2 秒間は機体を水平に置いて静止させる（その姿勢を 0 度とする）
- 軸割り当ては実機の静止データから決定（`AttitudeEstimator.cpp` の `kAxisMap`）
- ロールの出力符号は、HUD の表示と合うように反転している（内部の推定は変更なし）

## ファイル構成

```
examples/MAVStackchan/
├── platformio.ini          ビルド設定（依存は HermesLLM と同じ。SPIFFS 有効）
├── data/yaml/
│   └── SC_SecConfig.example.yaml  設定の雛形（コピーして SC_SecConfig.yaml を作る）
├── src/
│   ├── main.cpp            Wi-Fi / UDP / 表示（Arduino 依存部）
│   ├── Settings.h/.cpp     YAML 設定の解析（HermesLLM と同じ書式）
│   ├── AttitudeEstimator.h/.cpp  IMU からロール・ピッチ・ヨーを推定（軸割り当て kAxisMap）
│   └── MavlinkVehicle.h/.cpp  MAVLink 機体ロジック（Arduino 非依存）
└── tools/
    └── mav_probe.py        GCS の代わりに実機へ問い合わせる確認スクリプト
```

## ビルドと書き込み

1. 設定ファイルを作る（Wi-Fi の認証情報を含むため git 管理外）:

   ```
   copy data\yaml\SC_SecConfig.example.yaml data\yaml\SC_SecConfig.yaml
   ```

   `data/yaml/SC_SecConfig.yaml` の `wifi_list` に SSID とパスワードを書きます。
   HermesLLM の `data/yaml/SC_SecConfig.yaml` と同じ書式なので、そこからコピーしても使えます。
   ただし、HermesLLM 側の API キーなど不要な項目は含めないでください。

2. ビルドと書き込み（PlatformIO、CoreS3 を USB 接続）:

   ```
   pio run -e m5stack-cores3 -t uploadfs   # 設定ファイルを SPIFFS へ書き込む（シリアルモニターは閉じる）
   pio run -e m5stack-cores3 -t upload     # ファームを書き込む
   pio device monitor
   ```

   シリアルに `[MAV] settings: wifi=N system_id=1 udp_port=14550` と
   `[MAV] UDP 14550 open, local=... broadcast=...` が出れば Wi-Fi 接続完了です。
   設定ファイルが無い場合は画面に `No settings` と表示されます。

設定項目:

| キー | 内容 | 省略時 |
|---|---|---|
| `wifi_list` | 接続候補（先頭から順に試し、失敗すると次へ循環。最大 8 件） | 接続しない |
| `mav.system_id` | MAVLink System ID（1〜255） | 1 |
| `mav.udp_port` | 待ち受け・送信の UDP ポート | 14550 |

設定を変えたら `uploadfs` を再実行して再起動します（ファームの書き換えは不要）。

## QGC（Android）での確認手順

1. スマホと CoreS3 を同じ Wi-Fi に接続する。
2. QGC を起動する。UDP 14550 の自動接続で機体が見えるはず。
3. Fly View / 機体一覧で Rover（ArduPilot）として表示されるかを確認する。
4. 表示されない場合は、シリアルログで `GCS link UP` と `GCS peer` が出ているかを確認する。

**Phase 1 の受け入れ条件（指示書 11 章）**: Android 版 QGC に Stack-chan が機体として表示されること。
実機で確認するまでは成功扱いにしません。

## mav_probe.py（PC からの確認）

QGC の代わりに、PC から HEARTBEAT・ACK・AUTOPILOT_VERSION・ARM 拒否を確認します。

```
pip install pymavlink
python tools/mav_probe.py <CoreS3 の IP>
```

確認内容:

1. 機体の HEARTBEAT（Rover / ArduPilotMega / DISARMED）を受信できる
2. GCS の HEARTBEAT で「link up」が返る
3. `REQUEST_AUTOPILOT_CAPABILITIES` に AUTOPILOT_VERSION と ACK(ACCEPTED) が返る
4. ARM は ACK(UNSUPPORTED) で拒否され、機体は DISARMED のまま
5. 別 System ID 宛てのコマンドには応答しない

注意: UDP 14550 を使うため、同じ PC で QGC を開いている場合は閉じてから実行してください。

## 未検証・仮定事項

- **QGC の残課題**: 機体は認識されますが、ArduPilot 固有のパラメータ不足と、コンパス・加速度センサ未設定の警告が出ます（Phase 3 までの既知事項）。
- **ヨーのドリフト**: 磁気補正が無いため、時間とともにずれます。
- **YAML 解析は未テスト**: `Settings.cpp` はビルドのみ確認。ホスト側のテストは環境に無いため、実機のシリアルログ（`settings: wifi=N ...`）で確認します。
- **ArduRover の HOLD = custom_mode 4**: 既知の値として実装していますが、ArduPilot の Rover モード定義で再確認が必要です。
- **MAV_COMP_ID_AUTOPILOT1 = 1**: 指示書は名前で指定しています。値は c_library_v2 の定義から取りました。
- **QGC が要求するメッセージの網羅性**: パラメータ、ホームポジション、STATUSTEXT の扱いは実機ログで確認が必要です。
- **ブロードキャスト受信**: Android 側で UDP ブロードキャストが届くかは未確認です。届かない場合は QGC 側の手動接続（UDP ホスト指定）を試します。
- **既存の HermesLLM とは別機**: 本ファームは HermesLLM の MAVLink 受信機能（UART）とは独立しています。
  HermesLLM の変更は行っていません。

## 変更履歴

- Phase 1: 新規作成。HermesLLM 側の変更なし。
