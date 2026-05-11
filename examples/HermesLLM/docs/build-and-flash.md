# HermesLLM ビルド・書き込み手順

## 前提

- PlatformIO が VS Code 拡張としてインストール済み
- 作業ディレクトリ: `stackchan-arduino/examples/HermesLLM`
- PowerShell を使用（`pio` コマンドは利用不可のため PlatformIO 内蔵 Python を使う）

```powershell
cd C:\Users\hfuji\OneDrive\dev\stackchan-lab\stackchan-arduino\examples\HermesLLM
```

---

## 1. ファームウェア ビルド＆アップロード

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m platformio run -t upload
```

- ビルドとアップロードを一括実行
- `[SUCCESS]` が表示されれば完了

---

## 2. ファイルシステム (SPIFFS) アップロード

YAML 設定ファイル (`data/yaml/`) を書き込む。WiFi・APIキーを変更したときは必ず実行。

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m platformio run -t uploadfs
```

> **注意**: `No serial data received` エラーが出た場合は USB を抜き差しして再実行する。

---

## 3. シリアルモニタ

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m platformio device monitor
```

- ボーレート: 115200
- 終了: `Ctrl+C`

---

## 4. フラッシュ全消去（トラブル時）

タッチが効かないなど挙動がおかしいときに実行する。消去後は 1→2 の順で再書き込みが必要。

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m platformio run -t erase
```

---

## 通常の更新フロー

| 変更内容 | 必要な手順 |
|---------|-----------|
| ファームウェア (`src/`) を変更 | 手順 1 のみ |
| YAML 設定 (`data/yaml/`) のみ変更 | 手順 2 のみ |
| 両方変更 | 手順 1 → 2 |
| 動作確認 | 手順 3 |

---

## YAML 設定パラメータ一覧

`data/yaml/SC_SecConfig.yaml` で変更可能なパラメータ（uploadfs のみで反映）:

| パラメータ | 説明 | デフォルト |
|-----------|------|-----------|
| `wifi.ssid` | WiFi SSID | — |
| `wifi.password` | WiFi パスワード | — |
| `apikey.stt` | Google Cloud STT API キー | — |
| `apikey.tts` | tts.quest API キー | — |
| `hermes.endpoint` | Hermes LLM エンドポイント URL | — |
| `hermes.model` | 使用モデル名 | — |
| `hermes.api_key` | Hermes API キー | — |
| `tts_volume` | スピーカー音量 (0–255) | 80 |
| `hermes_max_tokens` | LLM 最大出力トークン数 | 40 |
| `hermes_timeout_ms` | Hermes HTTP タイムアウト (ms) | 24000 |
