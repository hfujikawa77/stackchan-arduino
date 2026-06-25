# Stack-chan Dance — 楽曲同期ダンス

PC/スマホで楽曲を再生し、そのビートに同期して Stack-chan（HermesLLM / CoreS3）を踊らせるツール一式です。
設計の背景は Issue #24 を参照してください。

## 仕組み

```
[PC/スマホ ブラウザ player.html]              [Stack-chan CoreS3 / HermesLLM]
  音源を Web Audio で再生  ──HTTP POST /beat──▶  既存の /beat エンドポイント
  audioContext.currentTime を基準に             → queue_http_beat() → サーボ1ビート
  ビートマップ(秒数配列)を参照して送信
```

- **音源とビートマップは同一クロック**（`audioContext.currentTime`）から導出するため、原理的にズレません。
- 各ビートは可聴タイミングより `LEAD`（既定120ms）早く送り、サーボの動き出し遅延を補償します。
- 受信側（ファーム）は**改修不要**。`/beat`（HTTP, port 80）をそのまま利用します。

## 1. ビートマップを作る（曲ごとに1回・オフライン）

```bash
cd tools/beatmap
pip install -r requirements.txt

# 基本（ダウンビートにアクセント、左右スイング）
python generate_beatmap.py "BillieJean.mp3" -o billie_jean.json

# 盛り上がり区間で縦ノリ(ud)を自動付与したい場合
python generate_beatmap.py "BillieJean.mp3" --auto-motion -o billie_jean.json
```

出力 JSON:

```json
{
  "title": "Billie Jean",
  "bpm": 117,
  "duration": 294.0,
  "beats_per_bar": 4,
  "beats": [
    { "t": 0.512, "accent": true,  "motion": "lr" },
    { "t": 1.026, "accent": false, "motion": "lr" }
  ]
}
```

- `t`: 曲頭からの秒数 / `accent`: 大きく動く拍（小節頭の近似）/ `motion`: `lr`=左右, `ud`=縦

オプション:
- `--bar N` : 1小節の拍数（既定4）。アクセント間隔
- `--downbeat-offset N` : 最初のダウンビート位置の調整
- `--auto-motion` : フレーズ単位で `ud`/`lr` を割り当て（エネルギーの高いフレーズが `ud`）
- `--phrase-bars N` : フレーズ長（小節）。動きはフレーズ内で一定（既定8）

## 1.5. ビートマップを編集する（editor.html）

再生しながら、イントロ/Aメロ/サビなどの**セクション境界・アクセント・動き(lr/ud)**を当て込むエディタです。
秒数を手入力する必要はなく、**聴きながらキーでマーキング**して JSON に書き出せます。

1. `editor.html` をブラウザで開く（PC推奨。`file://` でOK）
2. 音源ファイルとビートマップ JSON を読み込む
3. <kbd>Space</kbd> で再生。タイムライン上のプレイヘッドで現在位置を確認
4. 聴きながらキー操作で当て込む:
   - <kbd>S</kbd> 直近の拍にセクション境界を追加（動きは自動反転）
   - <kbd>L</kbd>/<kbd>U</kbd> 現在セクションを 横(lr)/縦(ud) に
   - <kbd>A</kbd> 直近の拍のアクセントをトグル（拍をクリックでも可）
   - <kbd>R</kbd> 直近の拍を**休符**にトグル（その拍は実機に送らない＝動かない）。薄い中空マーカーで表示
   - <kbd>X</kbd> 近くの境界を削除、<kbd>Shift</kbd>+<kbd>←/→</kbd> 境界を1拍ずらす
   - <kbd>[</kbd>/<kbd>]</kbd> ループ始点/終点、<kbd>\\</kbd> ループ on/off（区切りの追い込みに便利）
   - <kbd>Ctrl</kbd>+<kbd>Z</kbd> 取り消し / <kbd>Ctrl</kbd>+<kbd>Y</kbd>（または <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>Z</kbd>）やり直し（上部の Undo/Redo ボタンでも可）
5. 「⬇ JSON保存」でダウンロード（Chrome等は「💾 上書き保存」で元ファイルに直接保存可）

- **タップ補正(ms)**: 再生中にキーを押すと人間の反応ぶん遅れるため、その値だけ手前の拍に合わせます（既定120ms）
- 拍グリッド自体（テンポ）は生成スクリプト側の責務。エディタはその上で区切り・アクセント・動きを編集します

## 2. 再生して踊らせる

1. Stack-chan を起動し、設定画面などで **IPアドレス**を確認
2. Stack-chan 側で**アイドル自動モーションを OFF**（Rゾーンタップ）にしておくとビートが綺麗に出ます
3. `player.html` をブラウザで開く（PCはダブルクリックの `file://` でOK）
4. IPアドレスを入力 →「接続テスト」で 1 ビート送り、首が振れることを確認
5. 音源ファイルとビートマップ JSON を選択
6. 「再生」

### 遅延補償（LEAD）の合わせ方
- サーボの山がビートより**遅れる** → LEAD を**増やす**
- **早すぎる** → LEAD を**減らす**
- 既定120msはサーボ移動（約140ms）を見込んだ初期値。曲・環境で微調整

### 送信先: WiFi か USB（Web Serial）か
`player.html` / `editor.html` の「送信先」で選べます。

- **WiFi (HTTP)**: IPを入れて `/beat` を送る通常モード。手軽だが、スマホテザリングや混雑Wi‑Fiでは
  パケットが滞留→バーストして「無反応→急に激しく動く」ことがある
- **USB (Serial)**: スタックチャンを **USB-CでPCに直結**し、ブラウザの Web Serial で送る。
  WiFiを介さないのでジッタ・滞留が原理的にゼロ。**カフェ等の不安定回線で確実**
  1. CoreS3 を USB で接続（書き込みに使うのと同じケーブル）
  2. 送信先で「USB (Serial)」を選ぶ →「USB接続」でポートを選択（Chrome系PCのみ）
  3. あとは WiFi と同じ。「接続テスト」で首が振れることを確認して再生
  - シリアル行プロトコル: `B <step> <accent 0|1> <motion lr|ud> <bpm>` / `C`（センター復帰）
  - Web Serial は Chrome/Edge デスクトップが必要（スマホ・Safari不可）。シリアルモニタは閉じておく

## トラブルシュート

- **首が動かない**: IP違い / Stack-chan が busy（会話中）/ 設定画面中。まず「接続テスト」で確認
- **応答が見えない**: 仕様です。ブラウザの no-cors 送信のためレスポンスは読めませんが、リクエストは届きます
- **アイドル動作と干渉する**: Stack-chan 側のアイドルモーションを OFF に
- **スマホで音源が選べない**: 端末によりローカル音源の選択が制限されます。PC利用を推奨

## 既知の制約 / 将来の最適化

- WiFi のジッタが気になる場合は **USB (Web Serial) 送信**を使ってください（上記）。WiFiを介さず最も安定します。
- さらに高度な同期が必要なら、ビートマップを CoreS3 側に持たせ再生側はトランスポートクロックのみ送る方式にも拡張できます（Issue #24 参照）。
- 低レイテンシのデスクトップ用途には UDP `/beat`（port 8888）も受信側にありますが、ブラウザからは使えません。
