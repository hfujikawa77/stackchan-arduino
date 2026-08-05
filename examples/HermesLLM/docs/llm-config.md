# LLM 設定ガイド（HermesLLM）

HermesLLM ファームの LLM 呼び出しは **OpenAI 互換 Chat Completions** 形式です。
そのため OpenRouter をはじめ OpenAI 互換のエンドポイントをそのまま利用できます。

設定は秘密設定ファイル **`data/yaml/SC_SecConfig.yaml`** に書きます（このファイルは
API キーや WiFi パスワードを含むため git 管理外）。編集後は **`uploadfs`** で
本体の SPIFFS へ書き込みます（※書き込み前にシリアルモニターを閉じること）。

```
pio run -e m5stack-cores3 -t uploadfs
```

---

## 1. `hermes:` ブロック（接続先とモデル）

```yaml
hermes:
  endpoint: "https://openrouter.ai/api/v1/chat/completions"
  model: "openai/gpt-4o-mini"          # 現在使用中のモデル（本体メニューで切替可）
  api_key: "sk-or-v1-xxxxxxxxxxxx"      # OpenRouter の API キー
  # 返答を短くするためのシステム指示。文字数を変えたいときはここを編集。
  system: "あなたはStack-chan。返答は日本語で必ず1文・30文字以内。簡潔に。"
```

| キー | 内容 |
|------|------|
| `endpoint` | Chat Completions のフル URL。OpenRouter は必ず `https://openrouter.ai/api/v1/chat/completions`（`/api/v1` が必要） |
| `model`    | 使用中のモデル ID（slug）。OpenRouter は `ベンダー/モデル` 形式。本体メニューで切り替えると自動で書き換わる |
| `api_key`  | OpenRouter の `sk-or-v1-…`。空にすると `Authorization` ヘッダを送らない |
| `system`   | 毎回の呼び出しに付与するシステム指示（任意）。**省略可**。指定が無ければファーム内蔵の既定文が使われる |

> **`system:` の注意**
> パーサの都合で、値の中に **`"`（ダブルクオート）と `#`（シャープ）は使えません**。
> `#` 以降は行コメントとして除去され、`"` は文字列の終端になります。

---

## 2. `llm_models:` リスト（本体メニューの選択肢）

顔画面の右上タップ → **Settings → LLM タブ** に並ぶモデル候補を定義します。
`wifi_list:` と同じ書式で、**最大 8 件**まで。

```yaml
llm_models:
  - label: "Gemini Flash"
    model: "google/gemini-2.5-flash"
  - label: "GPT-4o mini"
    model: "openai/gpt-4o-mini"
  - label: "Claude Haiku"
    model: "anthropic/claude-3.5-haiku"
```

| キー | 内容 |
|------|------|
| `label` | 画面に表示する名前（任意。省略時は `model` の値を表示） |
| `model` | OpenRouter のモデル slug。選択すると `hermes: model:` に反映・保存される |

- **`llm_models:` を書かなかった場合**は、ファーム内蔵の既定 3 モデル
  （Gemini Flash / GPT-4o mini / Claude Haiku）にフォールバックします。
- 候補を増減したいときはこのリストを編集して `uploadfs` するだけ。
  すべて `hermes:` の `endpoint`/`api_key` を共有し、`model` だけが切り替わります。

### 本体での切り替え手順

1. 顔の**右上をタップ** → Settings
2. 上部タブ **「LLM」** をタップ
3. 一覧から使いたいモデルの行をタップ（現在のモデルが緑でハイライト）
4. 右下 **Save** で確定（次回起動後も維持）

---

## 3. 返答の長さ・タイムアウト

```yaml
hermes_max_tokens: 48      # 応答トークンの上限（ハード上限）
hermes_timeout_ms: 60000   # HTTP タイムアウト（ミリ秒）
```

- **返答を短くしたい**ときは、まず `hermes:` の `system:` で「N 文字以内」などと指示するのが
  きれいです（文を完結させたまま短くなる）。
- `hermes_max_tokens` は上限のみを決めるハード制限で、下げすぎると**文の途中で切れます**。
  日本語は 1 文字 ≒ 1 トークン程度が目安。
- 推論（reasoning）系モデルは推論分もトークンを消費するため、`hermes_max_tokens` が
  小さいと返答が空になることがあります。短い相槌用途では非推論モデル推奨。

---

## 4. トラブルシューティング

| 症状 | 原因・対処 |
|------|-----------|
| `Error: HTTP 402` | OpenRouter のクレジット不足。入金するか無料（`:free`）モデルを使う |
| `Error: HTTP 401` | `api_key` が不正／未設定 |
| `Hermes not configured.` | `hermes: endpoint:` が空。エンドポイントを設定する |
| 返答が空（`No content`） | `hermes_max_tokens` が小さすぎる／推論モデルで枠を使い切っている |
| `Could not open COMx` で書き込み失敗 | シリアルモニターがポートを占有。モニターを閉じてから `uploadfs` |
| `E ... I2S: register I2S object to platform failed` | スピーカー/マイク切替時の無害な警告。録音・再生が動いていれば無視してよい |
