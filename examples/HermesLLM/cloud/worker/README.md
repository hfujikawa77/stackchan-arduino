# stackchan-digest (Cloudflare Worker)

スマホの現在地を預かり、ｽﾀｯｸﾁｬﾝが撫でられた時に話す「根拠のある事実テキスト」を返す小さなAPIです。
言い回しへの整形（LLM）と音声合成（VOICEVOX）は従来どおり端末側で行います。

```
Android(MacroDroid) --POST /location--> Worker(KV) <--GET /digest/<cat>-- ｽﾀｯｸﾁｬﾝ(precache生成時)
```

## エンドポイント

すべて `Authorization: Bearer <SHARED_TOKEN>` が必要です（`GET /` のみ無認証のヘルスチェック）。

| メソッド/パス | 内容 |
|---|---|
| `POST /location` | `{"lat":35.66,"lon":139.70}` を保存（文字列の数値も可） |
| `GET /location` | 保存済みの位置を返す（デバッグ用） |
| `GET /digest/weather` | Open-Meteo の天気。20分キャッシュ、3km以上移動で再取得 |
| `GET /digest/gourmet` | ホットペッパーグルメ（要 `HOTPEPPER_KEY`）。400m以上移動 or 12時間で再取得、リクエスト毎にランダムで1店 |
| `GET /digest/trivia` | 現在地の地名 + 近くのWikipedia記事の冒頭（根拠付き）。リクエスト毎にランダムで1件 |
| `GET /digest/sleep` | `POST /sleep` で預けた直近（今日/昨日）の睡眠 |
| `POST /sleep` | `{"minutes_asleep":432,"efficiency":91,"deep":70,"rem":95,"minutes_awake":28,"date":"2026-10-05"}`（`minutes_asleep` 必須、`date` 省略時はJST今日） |

応答は `{"ok":true,"category":"weather","text":"…","place":"渋谷区渋谷","updated_at":…}`。
失敗時は `{"ok":false,"error":"…"}`（`no_location` 409 / `gourmet_not_configured` 503 / `no_recent_sleep` 404 など）。
端末側は200以外ならその項目を黙ってスキップします。

## デプロイ

```powershell
cd examples/HermesLLM/cloud/worker
npm install
npx wrangler login
npx wrangler kv namespace create STATE        # 出力された id を wrangler.toml に貼る
node -e "console.log(require('crypto').randomBytes(24).toString('hex'))"   # トークン生成
npx wrangler secret put SHARED_TOKEN          # 上で生成した値
npx wrangler secret put HOTPEPPER_KEY         # 任意（グルメを使う場合）
npx wrangler deploy
```

ローカル確認: `.dev.vars` に `SHARED_TOKEN=test-token` を書いて `npm run dev`、テストは `npm test`。

## ｽﾀｯｸﾁｬﾝ側の設定

`data/yaml/SC_SecConfig.yaml` に追記して `uploadfs`:

```yaml
digest_url: "https://stackchan-digest.<your-subdomain>.workers.dev"
digest_token: "<SHARED_TOKEN と同じ値>"
```

設定すると、デフォルトの撫で項目に天気/グルメ/ご当地/睡眠の4つが追加されます。
Web UIの撫で項目のプロンプトに `{digest:weather}` `{digest:gourmet}` `{digest:trivia}` `{digest:sleep}` を書けば自由に組み合わせられます
（`{news:N}` と同じ仕組みで、取得に失敗した項目はスキップされます）。

## MacroDroid（Android）の設定

- トリガー: 位置情報の変化（精度は「バランス」）。必要なら「自宅Wi-Fi未接続」を条件に追加
- アクション: HTTPリクエスト
  - URL: `https://stackchan-digest.<your-subdomain>.workers.dev/location`
  - メソッド: POST
  - ヘッダー: `Authorization: Bearer <SHARED_TOKEN>`、`Content-Type: application/json`
  - ボディ: `{"lat":"<緯度のマジックテキスト>","lon":"<経度のマジックテキスト>"}`
    （マジックテキスト選択画面から緯度・経度を挿入してください。数値を文字列で囲んでも受け付けます）

## 睡眠データについて

レガシーのFitbit Web APIは2026-09-30にサポート終了、2026-10-30に停止します。移行先のGoogle Health APIは
全スコープが制限付きで、新規プロジェクトの受付も限定的です。そのため、取得元を問わず
`POST /sleep` で受け取る作りにしています（手動、将来のGoogle Health API連携、Health Connect対応アプリなど）。

## 注意

- 端末側のTLSは他のAPI呼び出しと同様に証明書検証なし（`setInsecure()`）です。トークンの扱いに注意してください。
- KV無料枠は書き込み1000回/日。キャッシュ更新のみ書き込むので通常は余裕があります。
- ホットペッパーグルメAPIの提供状況は公式（webservice.recruit.co.jp）で確認してください。
