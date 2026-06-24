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
- `--auto-motion` : ビートのエネルギーが中央値以上なら `ud`、それ以外は `lr`

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

## トラブルシュート

- **首が動かない**: IP違い / Stack-chan が busy（会話中）/ 設定画面中。まず「接続テスト」で確認
- **応答が見えない**: 仕様です。ブラウザの no-cors 送信のためレスポンスは読めませんが、リクエストは届きます
- **アイドル動作と干渉する**: Stack-chan 側のアイドルモーションを OFF に
- **スマホで音源が選べない**: 端末によりローカル音源の選択が制限されます。PC利用を推奨

## 既知の制約 / 将来の最適化

- WiFi のジッタが気になる場合、ビートマップを CoreS3 側に持たせ、再生側はトランスポートクロックのみ定期送信（NTP的位置合わせ）してファーム側でスケジュールする方式に拡張できます（Issue #24 参照）。
- 低レイテンシのデスクトップ用途には UDP `/beat`（port 8888）も受信側にありますが、ブラウザからは使えないため本ツールは HTTP を使用します。
