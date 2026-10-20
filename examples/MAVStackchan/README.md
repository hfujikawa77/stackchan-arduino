# MAVStackchan

Stack-chan（M5Stack CoreS3）を MAVLink 機体として振る舞わせ、Android 版 QGroundControl（QGC）から
認識させる PoC ファームです。開発指示書は `examples/logs/20261020_mav-stackchan/direction.md` を参照してください。

## 現在の状態

| Phase | 内容 | 状態 |
|---|---|---|
| 1 | Wi-Fi UDP、HEARTBEAT 送信、GCS 検知、未対応コマンドへの ACK | **実機で確認済み**（QGC で機体として認識） |
| 2 | IMU 姿勢（ATTITUDE 20Hz）と HUD 表示 | **実機で確認済み**（ロール・ピッチが HUD に追従） |
| 3 | ARM / DISARM、HOLD / MANUAL 切替 | **実機で確認済み**（probe 23 項目 PASS、GCS 切断時の自動 DISARM を確認。仕様は下記「Phase 3 の仕様」） |
| 4 | バーチャルスティックで Stack-chan 内蔵の頭部サーボを動かす | **実装済み・実機未確認**（仕様は下記「Phase 4 の仕様」） |
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

## Phase 3 の仕様（ARM / モード）

状態は「モード（HOLD / MANUAL）」と「ARM 状態」の 2 つです。

- 起動直後は **HOLD / DISARMED**。再起動で ARM 状態は引き継ぎません。
- モード番号は ArduRover の custom_mode に合わせます（MANUAL = 0、HOLD = 4）。対応するのはこの 2 つだけです。
  値は ArduPilot の Rover モード定義で要確認です。

**ARM（MAV_CMD_COMPONENT_ARM_DISARM, param1 = 1）**
- 許可する条件: GCS のリンクが生きていること（モードは HOLD でも MANUAL でも可）。
- 条件を満たさない場合は `MAV_RESULT_DENIED`。強制 ARM（param2 = 21196）は未対応で、無視します。
- サーボの初期化確認とフェイルセーフ判定は **Phase 4 で追加**します。Phase 3 はアクチュエータが無いため、この条件は省略しています。

**DISARM（param1 = 0）**: いつでも許可します。

**モード変更（MAV_CMD_DO_SET_MODE と MAVLink の SET_MODE）**
- base_mode に CUSTOM_MODE フラグが必要です。
- custom_mode が 0（MANUAL）と 4（HOLD）以外は `MAV_RESULT_DENIED`。
- HOLD に切り替えても ARM 状態は変えません。サーボの中立化は Phase 4 で行います。

**安全側の規則**
- GCS のリンクが切れたら **自動で DISARM** します。再接続しても、自動では ARM しません。

**HEARTBEAT への反映**
- ARM 中は base_mode に SAFETY_ARMED を立て、system_status は ACTIVE にします。DISARM 中は STANDBY です。
- custom_mode には現在のモードを入れます。

**通知**: ARM・DISARM・モード変更は STATUSTEXT で QGC に通知します。

## Phase 4 の仕様（バーチャルスティックで頭部を動かす）

新しいサーボは追加しません。Stack-chan 内蔵の頭部サーボ（パン・チルト）を、QGC のバーチャルスティックで動かします。
制御には HermesLLM と同じ `StackchanSERVO` を使い、設定（中立角・可動範囲）は `data/yaml/SC_BasicConfig.yaml` から読みます。

**入力**: `MANUAL_CONTROL` の x / y（-1000〜1000）。

**割り当て（実機で確認済み）**
- x（前後スティック）→ 頭部の上下（チルト）。符号は反転（前に倒すと頭部も同じ向き）
- y（左右スティック）→ 頭部の左右（パン）。符号は反転（右に倒すと頭部も同じ向き）

**動かす条件**: ARM 中かつ MANUAL モードで、タイムアウト内（既定 500ms）に入力が届いているとき。
それ以外（DISARM、HOLD、入力の途絶え、GCS 切断）は、頭部を中立角に戻します。

**可動範囲**: `SC_BasicConfig.yaml` の `lower_limit` / `upper_limit` を超えません。
内蔵サーボの Y 軸は、中立角 90 に対して可動範囲が 0〜90 のため、片側にしか動きません。
両側に動かしたい場合は、設定の上限を変えてください。

**不感帯**: スティックの中央付近（±5%）は中立として扱います。

## 水平（レベル）キャリブレーション

- QGC の Vehicle Setup の Sensors から Level horizon を実行できます。
- 機体を水平に置いて静止させると、そのときの姿勢を水平の基準（0 度）にします。約 2 秒間です。
- ARM 中は実行できません（`Disarm to allow calibration`）。
- 期間中に機体が動いたら失敗にし、前の基準を残します。
- 起動直後も、同じ方法で基準を取ります。起動時に動かしていた場合は、QGC からやり直してください。

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
│   ├── ManualControl.h/.cpp  スティック値 → 頭部の目標角（Arduino 非依存）
│   ├── HeadServo.h/.cpp    内蔵の頭部サーボ（StackchanSERVO）のラッパー
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
