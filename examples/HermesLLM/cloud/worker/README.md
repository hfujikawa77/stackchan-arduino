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
| `GET /digest/place` | 現在地の地名と、その場所の季節（南半球は反転）。動物の鳴き声など、地域・季節に合わせた話題用 |
| `GET /digest/sleep` | `POST /sleep` で預けた直近（今日/昨日）の睡眠 |
| `POST /sleep` | `{"minutes_asleep":432,"efficiency":91,"deep":70,"rem":95,"minutes_awake":28,"date":"2026-10-05"}`（`minutes_asleep` 必須、`date` 省略時はJST今日）。Google Health 連携（cron）でも同じ形で保存される |
| `GET /digest/steps` | 昨日の歩数と、その前の1週間の1日平均（cron で Google Health から取得）。言い回し（「よく歩いたね」など）は端末側のプロンプトで付ける |
| `GET /health/status` | 直近の cron の結果（睡眠・歩数ごとの ok / no_data / error、時刻） |
| `POST /health/refresh` | cron と同じ取得を今すぐ実行（デプロイ後の動作確認用） |

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
npx wrangler secret put HOME_LAT              # 任意（自宅の緯度。設定すると、離れている間 digest が away:true を返し、端末が現地情報を多めに話す）
npx wrangler secret put HOME_LON              # 任意（自宅の経度）
npx wrangler secret put AWAY_KM               # 任意（自宅からこの距離[km]を超えたら away。省略時30）
npx wrangler secret put GOOGLE_CLIENT_ID      # 任意（睡眠・歩数の自動取得を使う場合。下の「Google Health 連携」参照）
npx wrangler secret put GOOGLE_CLIENT_SECRET
npx wrangler secret put GOOGLE_REFRESH_TOKEN
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
Web UIの撫で項目のプロンプトに `{digest:weather}` `{digest:gourmet}` `{digest:trivia}` `{digest:sleep}` `{digest:steps}` を書けば自由に組み合わせられます
（`{news:N}` と同じ仕組みで、取得に失敗した項目はスキップされます）。

## MacroDroid（Android）の設定

- トリガー: 位置情報の変化（精度は「バランス」）。必要なら「自宅Wi-Fi未接続」を条件に追加
- アクション: HTTPリクエスト
  - URL: `https://stackchan-digest.<your-subdomain>.workers.dev/location`
  - メソッド: POST
  - ヘッダー: `Authorization: Bearer <SHARED_TOKEN>`、`Content-Type: application/json`
  - ボディ: `{"lat":"<緯度のマジックテキスト>","lon":"<経度のマジックテキスト>"}`
    （マジックテキスト選択画面から緯度・経度を挿入してください。数値を文字列で囲んでも受け付けます）

## Google Health 連携（睡眠・歩数）

毎朝 cron（`wrangler.toml` の `[triggers]`。JST 7:30 / 8:30 / 12:00）で Google Health API から取得し、KV に保存します。
Fitbit の同期が遅れた日のための再実行で、同じ記録を上書きするだけです。

| 項目 | 取得元 |
|---|---|
| 睡眠 | 今日（JST）に終わった睡眠のうち、メインの睡眠（`mainSleep`、なければ最長）。睡眠効率は `実睡眠 ÷ ベッドにいた時間` で算出（Fitbitアプリの表示と一致） |
| 歩数 | `dailyRollUp` の昨日1日分と、その前の7日分の平均。Fitbitとスマホ（Health Connect）の二重計上は API 側で除かれる |

- 項目ごとに独立して失敗します。データなしや失敗のときは、以前の値を上書きしません。結果は `GET /health/status` で確認できます。
- ログには状態だけを出し、睡眠・歩数の値は出しません。
- 認証情報は Secrets（`GOOGLE_CLIENT_ID` / `GOOGLE_CLIENT_SECRET` / `GOOGLE_REFRESH_TOKEN`）。Refresh Token は Google OAuth 2.0 Playground などで取得します（スコープは `googlehealth.sleep.readonly` と `googlehealth.activity_and_fitness.readonly`）。
- OAuth 同意画面が「テスト」のままだと Refresh Token が 7 日で失効します。「本番」にする場合は、未確認アプリの警告が出る点に注意してください。
- ローカル確認: `.dev.vars` に上の3つを足し、`npx wrangler dev --test-scheduled` で起動して `curl "http://localhost:8787/__scheduled"`。
- 歩数の話し方の例（Web UI の撫で項目）: `次の情報だけを元に、昨日の歩数をねぎらって短く伝えて。医療的な助言はしない。情報にない事実は足さない。情報: {digest:steps}`

取得元を問わず `POST /sleep` で預けることもできます（手動、Health Connect 対応アプリなど）。
レガシーのFitbit Web APIは2026-09-30にサポート終了、2026-10-30に停止します。

## 注意

- 端末側のTLSは他のAPI呼び出しと同様に証明書検証なし（`setInsecure()`）です。トークンの扱いに注意してください。
- KV無料枠は書き込み1000回/日。キャッシュ更新のみ書き込むので通常は余裕があります。
- ホットペッパーグルメAPIの提供状況は公式（webservice.recruit.co.jp）で確認してください。

## Discord 連携

### 発話のログ（一方向）
撫での発話を、再生後に `POST /log` で Discord へ送る。Webhook URL は Worker のシークレット `DISCORD_WEBHOOK_URL`。
`{"text":"…","facts":"…"}`。`facts` は発話の元にした情報で、`msg:<DiscordのメッセージID>` として3日間 KV に保存し、追い質問への回答に使う。

### 返信への回答（双方向）
Discord で返信（または普通に投稿）すると、ｽﾀｯｸﾁｬﾝが答えを作り、次の撫でで話す。

| メソッド/パス | 内容 |
|---|---|
| `GET /inbox` | 前回の確認応答以降の、許可ユーザーの投稿を1件（古い順）。Discord API は取得時にだけ呼ぶ（8秒キャッシュ）。初回は現在の最新を起点にして、過去ログは読まない |
| `POST /inbox/ack` | `{"id":"…"}` 処理済みにする（カーソルは後ろへ戻らない） |
| `POST /reply` | `{"to":"<返信先ID>","text":"…","facts":"…"}` Bot が Discord の「返信」として投稿 |

`GET /inbox` の応答:
```json
{"ok":true,"items":[{"id":"…","text":"どこにあるの？","ts":1791430000000,
  "reply_to":{"id":"…","text":"ｽﾀｯｸﾁｬﾝが話した文","facts":"元の情報","by":"stackchan"}}]}
```
`reply_to` は返信でない投稿では `null`。`by` は `stackchan`（発話を保存済み）/ `bot` / `other`。

### Bot の準備（一度だけ）
1. https://discord.com/developers/applications で New Application → Bot を作成
2. Bot の設定で **MESSAGE CONTENT INTENT** を有効にする
3. OAuth2 → URL Generator で `bot` を選び、権限は「メッセージを見る」「メッセージ履歴を読む」「メッセージを送信」。出てきた URL でサーバーに招待
4. 対象チャンネル（Webhook を作ったのと同じ）で Bot が見えて書けることを確認
5. ユーザーID・チャンネルIDは、Discord の設定 → 詳細設定 → 開発者モード をオンにして、右クリック →「IDをコピー」
6. シークレットを登録（値はチャットに貼らない）:
```powershell
npx wrangler secret put DISCORD_BOT_TOKEN
npx wrangler secret put DISCORD_CHANNEL_ID
npx wrangler secret put DISCORD_USER_ID      # 答える相手。この人以外の投稿は無視する
```
未設定の間、`/inbox` `/inbox/ack` `/reply` は `503 inbox_not_configured`。端末はそれを見て10分間隔まで間引く。

### 端末側の設定（SC_SecConfig.yaml）
```yaml
inbox_poll_sec: 30   # 待機中に inbox を見る間隔(秒)。0 にすると、撫でて補充が走るときだけ見る
```