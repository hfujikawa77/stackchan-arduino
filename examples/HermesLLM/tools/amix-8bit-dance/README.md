# Stack-chan 8bit Dance Bridge

[EASY 8BIT EDITOR](https://amix-design.com/tl/tool-s-8bit/)（amix-design 様制作の無料チップチューン作曲ツール）の
再生ビートを **Web Serial** 経由で Stack-chan（HermesLLM ファーム）へ送り、音楽に合わせて首を振らせる
Tampermonkey ユーザースクリプトです。

https://user-images.example/ ← デモ動画があればここに

---

## ⚠️ 免責・お断り

- 本ツールは **非公式** です。**amix-design 様および EASY 8BIT EDITOR とは一切関係ありません**。
- 素晴らしいツールを無償で公開してくださっている作者様に感謝します。🙏
  本ツールはその音楽を楽しむためのファン制作物です。
- **動作は無保証**です。EASY 8BIT EDITOR の内部実装（非公開）に依存しているため、
  サイトの更新でいつでも動かなくなる可能性があります。
- 本リポジトリは EASY 8BIT EDITOR のコード・音源・アセットを **一切含みません／再配布しません**。
- 利用は各自の責任で、サイトの利用規約を尊重してご利用ください。

## 動作環境

- ブラウザ: **Chrome または Edge**（Web Serial 対応が必須。Firefox/Safari 不可）
- 拡張: [Tampermonkey](https://www.tampermonkey.net/)
- ハード: M5Stack CoreS3 + Stack-chan、**HermesLLM** ファーム書き込み済み（無改修で動作）

## 仕組み

EASY 8BIT EDITOR は React 製で、内部に Web Audio シーケンサ（コントローラ）を持ちます。
本スクリプトは React fiber を辿ってそのコントローラへ到達し、以下を読み取ります:

| 参照 | 意味 |
|---|---|
| `state.bpm` | 現在の BPM |
| `state.playing` | 再生中フラグ |
| `state.steps` | ループのステップ数（16分音符単位） |
| `nextStep` | 進行中のステップカウンタ |
| `graph.ctx` | AudioContext（時計） |

1 ステップ = 16分音符なので、**拍 = `nextStep % 4 == 0`**、**小節頭（4/4想定）= `nextStep % 16 == 0`（accent）**。
拍が変わるたびに、HermesLLM の `serial_beat_tick()` が解釈する行を USB シリアルへ送ります:

```
B <拍番号> <accent 0|1> <motion lr|ud> <bpm>\n   拍のパルス
C\n                                              正面に戻す（停止時）
```

## 使い方

1. `amix-8bit-dance.user.js` を Tampermonkey に登録。
2. Stack-chan（CoreS3）を PC に USB 接続（HermesLLM 書き込み済み・通常モード）。
3. [サイト](https://amix-design.com/tl/tool-s-8bit/)を開き、右下パネルの **「🔌 接続」** → シリアルポート（CoreS3）を選択。
4. スペース or PLAY で再生 → 拍に合わせて Stack-chan が踊ります。

## 調整（スクリプト冒頭の定数）

| 定数 | 説明 |
|---|---|
| `MOTION` | `'lr'`（左右スイング）↔ `'ud'`（縦うなずき） |
| `LEAD_STEPS` | 動きが音より遅れる/早い場合に ±整数で前後補正 |
| `STEPS_PER_BAR` | 4/4 以外の曲で accent 位置を変えたい時 |
| `BAUD` | シリアル速度（既定 115200、`platformio.ini` の `monitor_speed` と一致） |

## クレジット

- 音楽ツール: **EASY 8BIT EDITOR** by [amix-design](https://amix-design.com/tl/tool-s-8bit/) 様
- Stack-chan: [ロボットゆうき](https://github.com/meganetaaan/stack-chan) 様ほかコミュニティ
