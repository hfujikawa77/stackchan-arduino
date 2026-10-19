import { test, beforeEach } from "node:test";
import assert from "node:assert/strict";
import worker from "../src/index.js";
import { compareSnowflake, snowflakeTs, toInboxItem } from "../src/lib.js";

const ME = "1000000000000000001";
const OTHER = "1000000000000000002";

class MemKV {
  constructor() { this.m = new Map(); this.ttl = new Map(); }
  async get(k, type) { const v = this.m.get(k); return v === undefined ? null : type === "json" ? JSON.parse(v) : v; }
  async put(k, v, opts) { this.m.set(k, v); if (opts?.expirationTtl) this.ttl.set(k, opts.expirationTtl); }
}

let env, discordMessages, calls;

const msg = (id, content, over = {}) => ({
  id: String(id), type: 0, content, author: { id: ME, bot: false }, ...over,
});

beforeEach(() => {
  env = {
    SHARED_TOKEN: "t", STATE: new MemKV(),
    DISCORD_BOT_TOKEN: "bot", DISCORD_CHANNEL_ID: "777", DISCORD_USER_ID: ME,
    DISCORD_WEBHOOK_URL: "https://discord.com/api/webhooks/1/abc",
  };
  discordMessages = [];
  calls = [];
  globalThis.fetch = async (url, init = {}) => {
    const u = new URL(url);
    calls.push({ url: u.toString(), method: init.method ?? "GET", body: init.body });
    if (u.pathname.startsWith("/api/webhooks/")) return Response.json({ id: "5000" });
    if (u.pathname === "/api/v10/channels/777/messages" && (init.method ?? "GET") === "GET") {
      const after = u.searchParams.get("after");
      const list = discordMessages
        .filter((m) => !after || compareSnowflake(m.id, after) > 0)
        .sort((a, b) => compareSnowflake(b.id, a.id));
      return Response.json(list.slice(0, Number(u.searchParams.get("limit") ?? 50)));
    }
    if (u.pathname === "/api/v10/channels/777/messages") return Response.json({ id: "6000" });
    return new Response("nope", { status: 404 });
  };
});

const call = (path, { method = "GET", body, auth = "Bearer t" } = {}) =>
  worker.fetch(
    new Request(`https://w.example${path}`, {
      method, headers: { authorization: auth }, body: body ? JSON.stringify(body) : undefined,
    }),
    env,
  ).then(async (r) => ({ status: r.status, body: await r.json() }));

test("toInboxItem ignores other users, bots, webhooks, system messages and empty text", () => {
  assert.equal(toInboxItem(msg(1, "hi", { author: { id: OTHER } }), ME), undefined);
  assert.equal(toInboxItem(msg(1, "hi", { author: { id: ME, bot: true } }), ME), undefined);
  assert.equal(toInboxItem(msg(1, "hi", { webhook_id: "9" }), ME), undefined);
  assert.equal(toInboxItem(msg(1, "hi", { type: 7 }), ME), undefined);
  assert.equal(toInboxItem(msg(1, "   "), ME), undefined);
  assert.equal(toInboxItem(msg(1, "hi"), ""), undefined);
  assert.equal(toInboxItem(msg(1, "hi"), ME).text, "hi");
});

test("toInboxItem carries the referenced message and caps the length", () => {
  const it = toInboxItem(
    msg("1291", "あ".repeat(500), { type: 19, referenced_message: { id: "1290", content: "元の発話", author: { bot: true } } }),
    ME,
  );
  assert.equal(it.text.length, 300);
  assert.deepEqual(it.ref, { id: "1290", text: "元の発話", bot: true });
  assert.equal(it.ts, snowflakeTs("1291"));
});

test("snowflake helpers compare beyond Number precision", () => {
  assert.equal(compareSnowflake("1291234567890123457", "1291234567890123456"), 1);
  assert.equal(compareSnowflake("9", "10"), -1);
  assert.equal(compareSnowflake("5", "5"), 0);
});

test("endpoints require the shared token", async () => {
  assert.equal((await call("/inbox", { auth: "Bearer x" })).status, 401);
});

test("inbox is a 503 until the bot is configured", async () => {
  delete env.DISCORD_BOT_TOKEN;
  const r = await call("/inbox");
  assert.equal(r.status, 503);
  assert.equal(r.body.error, "inbox_not_configured");
});

test("first poll starts from the latest message and returns nothing", async () => {
  discordMessages = [msg("100", "old history")];
  const r = await call("/inbox");
  assert.deepEqual(r.body, { ok: true, items: [] });
  assert.equal(await env.STATE.get("inbox_cursor", "json"), "100");
});

test("a reply to a logged utterance comes back with its facts, then ack moves the cursor", async () => {
  await call("/log", { method: "POST", body: { text: "デジちゃんは韓国料理が楽しめるよ！", facts: "場所は江古田駅南口より徒歩1分" } });
  assert.deepEqual(env.STATE.m.get("msg:5000") && JSON.parse(env.STATE.m.get("msg:5000")),
    { text: "デジちゃんは韓国料理が楽しめるよ！", facts: "場所は江古田駅南口より徒歩1分" });
  assert.equal(env.STATE.ttl.get("msg:5000"), 3 * 24 * 3600);

  discordMessages = [msg("100", "old")];
  await call("/inbox"); // initialises the cursor at 100
  discordMessages.push(
    msg("101", "どこにあるの？", { type: 19, referenced_message: { id: "5000", content: "デジちゃんは…", author: { bot: true }, webhook_id: "1" } }),
  );
  const r = await call("/inbox");
  assert.equal(r.body.items.length, 1);
  assert.deepEqual(r.body.items[0].reply_to, {
    id: "5000", text: "デジちゃんは韓国料理が楽しめるよ！", facts: "場所は江古田駅南口より徒歩1分", by: "stackchan",
  });
  assert.equal(r.body.items[0].text, "どこにあるの？");

  assert.equal((await call("/inbox/ack", { method: "POST", body: { id: "101" } })).status, 200);
  assert.equal(await env.STATE.get("inbox_cursor", "json"), "101");
  assert.equal((await call("/inbox")).body.items.length, 0);
});

test("a plain message has no reply_to, and an unknown replied-to message is marked other", async () => {
  discordMessages = [msg("100", "old")];
  await call("/inbox");
  discordMessages.push(msg("101", "今日の天気は？"));
  assert.equal((await call("/inbox")).body.items[0].reply_to, null);
  await call("/inbox/ack", { method: "POST", body: { id: "101" } });
  discordMessages.push(
    msg("102", "これは？", { type: 19, referenced_message: { id: "42", content: "誰かの文", author: { bot: false } } }),
  );
  const r = (await call("/inbox")).body.items[0];
  assert.deepEqual(r.reply_to, { id: "42", text: "誰かの文", facts: "", by: "other" });
});

test("other people's messages are skipped past without being returned", async () => {
  discordMessages = [msg("100", "old")];
  await call("/inbox");
  discordMessages.push(msg("101", "雑談", { author: { id: OTHER } }), msg("102", "bot", { author: { id: ME, bot: true } }));
  assert.equal((await call("/inbox")).body.items.length, 0);
  assert.equal(await env.STATE.get("inbox_cursor", "json"), "102");
});

test("oldest message is returned first, one at a time", async () => {
  discordMessages = [msg("100", "old")];
  await call("/inbox");
  discordMessages.push(msg("103", "三つ目"), msg("101", "一つ目"), msg("102", "二つ目"));
  assert.equal((await call("/inbox")).body.items[0].text, "一つ目");
});

test("ack never moves the cursor backwards, and rejects non-ids", async () => {
  await env.STATE.put("inbox_cursor", JSON.stringify("200"));
  await call("/inbox/ack", { method: "POST", body: { id: "150" } });
  assert.equal(await env.STATE.get("inbox_cursor", "json"), "200");
  assert.equal((await call("/inbox/ack", { method: "POST", body: { id: "abc" } })).status, 400);
});

test("reply is posted as a Discord reply with mentions disabled, and its id is remembered with facts", async () => {
  const r = await call("/reply", { method: "POST", body: { to: "101", text: "えとね、駅から近いよ @everyone", facts: "元の情報" } });
  assert.equal(r.status, 200);
  const post = calls.find((c) => c.method === "POST" && c.url.endsWith("/channels/777/messages"));
  const sent = JSON.parse(post.body);
  assert.deepEqual(sent.message_reference, { message_id: "101", fail_if_not_exists: false });
  assert.deepEqual(sent.allowed_mentions, { parse: [], replied_user: false });
  assert.deepEqual(JSON.parse(env.STATE.m.get("msg:6000")), { text: "えとね、駅から近いよ @everyone", facts: "元の情報" });
  assert.equal((await call("/reply", { method: "POST", body: { to: "x", text: "a" } })).status, 400);
  assert.equal((await call("/reply", { method: "POST", body: { text: " " } })).status, 400);
});

test("a Discord failure is reported as discord_error without leaking details", async () => {
  discordMessages = [msg("100", "old")];
  await call("/inbox");
  globalThis.fetch = async () => new Response("secret-ish body", { status: 403 });
  const r = await call("/inbox");
  assert.equal(r.status, 502);
  assert.deepEqual(r.body, { ok: false, error: "discord_error" });
});
