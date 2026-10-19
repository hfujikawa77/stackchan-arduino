import {
  addDays, compareSnowflake, constantTimeEqual, discordPayload, formatGourmet, formatSleep, formatSteps,
  formatSearch, formatTrivia, formatWeather, haversineM, isSnowflake, jstDate, parseNewsRss, parsePlace, pickRandom, toInboxItem, toNumber,
  trimExtract, validLatLon,
} from "./lib.js";
import { fetchSleepRecord, fetchStepsRecord, getAccessToken } from "./google_health.js";
import { botConfigured, fetchChannelMessages, postBotMessage, postWebhookMessage } from "./discord.js";

const UA = "stackchan-digest/1.0 (personal hobby project)";
const CATEGORIES = ["weather", "gourmet", "trivia", "place", "sleep", "steps"];

// A cached value is reused until it is older than TTL or the phone moved farther than MOVE_M from where it was computed.
const MOVE_M = { place: 300, weather: 3000, gourmet: 400, trivia: 400 };
const TTL_MS = {
  place: 24 * 3600e3,
  weather: 20 * 60e3,
  gourmet: 12 * 3600e3,
  trivia: 12 * 3600e3,
};

class HttpError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
  }
}

const json = (body, status = 200) =>
  new Response(JSON.stringify(body), {
    status,
    headers: { "content-type": "application/json; charset=utf-8", "cache-control": "no-store" },
  });
const fail = (status, error) => json({ ok: false, error }, status);

async function getJson(url) {
  const res = await fetch(url, {
    headers: { "user-agent": UA, accept: "application/json" },
    signal: AbortSignal.timeout(8000),
  });
  if (!res.ok) throw new Error(`${new URL(url).host} responded ${res.status}`);
  return res.json();
}

const readKV = (env, key) => env.STATE.get(key, "json");
const writeKV = (env, key, value) => env.STATE.put(key, JSON.stringify(value));

const isFresh = (cache, loc, kind, now) =>
  !!cache &&
  now - cache.at < TTL_MS[kind] &&
  haversineM(cache.lat, cache.lon, loc.lat, loc.lon) <= MOVE_M[kind];

async function readBody(request) {
  try {
    return JSON.parse(await request.text());
  } catch {
    throw new HttpError(400, "invalid_json");
  }
}

async function getPlace(env, loc, now) {
  const cache = await readKV(env, "place");
  if (isFresh(cache, loc, "place", now)) return cache.text;
  try {
    const url =
      "https://nominatim.openstreetmap.org/reverse?format=jsonv2&zoom=16&accept-language=ja" +
      `&lat=${loc.lat}&lon=${loc.lon}`;
    const text = parsePlace(await getJson(url));
    if (text) {
      await writeKV(env, "place", { lat: loc.lat, lon: loc.lon, at: now, text });
      return text;
    }
  } catch (e) {
    console.log("place lookup failed:", e.message);
  }
  // A stale name is only trustworthy if the phone is still nearby.
  return cache && haversineM(cache.lat, cache.lon, loc.lat, loc.lon) < 2000 ? cache.text : "";
}

async function digestWeather(env, loc, now) {
  const place = await getPlace(env, loc, now);
  let cache = await readKV(env, "weather");
  if (!isFresh(cache, loc, "weather", now)) {
    const url =
      `https://api.open-meteo.com/v1/forecast?latitude=${loc.lat.toFixed(3)}&longitude=${loc.lon.toFixed(3)}` +
      "&current=temperature_2m,weather_code&daily=temperature_2m_max,temperature_2m_min," +
      "precipitation_probability_max&timezone=auto&forecast_days=1";
    cache = { lat: loc.lat, lon: loc.lon, at: now, om: await getJson(url) };
    await writeKV(env, "weather", cache);
  }
  const text = formatWeather(place, cache.om);
  if (!text) throw new HttpError(502, "weather_unavailable");
  return { text, place, updated_at: cache.at };
}

async function digestGourmet(env, loc, now) {
  if (!env.HOTPEPPER_KEY) throw new HttpError(503, "gourmet_not_configured");
  const place = await getPlace(env, loc, now);
  let cache = await readKV(env, "gourmet");
  if (!isFresh(cache, loc, "gourmet", now)) {
    const url =
      "https://webservice.recruit.co.jp/hotpepper/gourmet/v1/" +
      `?key=${encodeURIComponent(env.HOTPEPPER_KEY)}&lat=${loc.lat}&lng=${loc.lon}` +
      "&range=3&count=20&order=4&format=json";
    const j = await getJson(url);
    if (j.results?.error) throw new Error(`hotpepper: ${JSON.stringify(j.results.error)}`);
    const shops = (j.results?.shop ?? []).map((s) => ({
      name: s.name, genre: s.genre, access: s.access, catch: s.catch, budget: s.budget,
    }));
    cache = { lat: loc.lat, lon: loc.lon, at: now, shops };
    await writeKV(env, "gourmet", cache);
  }
  const shop = pickRandom(cache.shops);
  if (!shop) throw new HttpError(404, "no_shops_nearby");
  return { text: `近くのお店。${formatGourmet(shop)}`, place, updated_at: cache.at };
}

async function fetchWikiNearby(loc) {
  const base = "https://ja.wikipedia.org/w/api.php?action=query&format=json&formatversion=2";
  const geo = await getJson(
    `${base}&list=geosearch&gsradius=2000&gslimit=20&gscoord=${encodeURIComponent(`${loc.lat}|${loc.lon}`)}`,
  );
  const hits = geo.query?.geosearch ?? [];
  if (!hits.length) return [];
  const ids = hits.map((h) => h.pageid).join("|");
  const ext = await getJson(
    `${base}&prop=extracts&exintro=1&explaintext=1&exlimit=20&exchars=400&pageids=${encodeURIComponent(ids)}`,
  );
  return (ext.query?.pages ?? [])
    .filter((p) => p.extract && p.extract.length > 20)
    .map((p) => ({ title: p.title, extract: trimExtract(p.extract, 200) }));
}

async function digestTrivia(env, loc, now) {
  const place = await getPlace(env, loc, now);
  let cache = await readKV(env, "trivia");
  if (!isFresh(cache, loc, "trivia", now)) {
    let pages = [];
    try {
      pages = await fetchWikiNearby(loc);
    } catch (e) {
      console.log("wikipedia lookup failed:", e.message);
    }
    cache = { lat: loc.lat, lon: loc.lon, at: now, pages };
    await writeKV(env, "trivia", cache);
  }
  const text = formatTrivia(place, pickRandom(cache.pages));
  if (!text) throw new HttpError(404, "no_local_info");
  return { text, place, updated_at: cache.at };
}

// Place name plus the season there (flipped in the southern hemisphere), for region-flavoured small talk.
async function digestPlace(env, loc, now) {
  const place = await getPlace(env, loc, now);
  if (!place) throw new HttpError(404, "no_place");
  const month = new Date(now).getUTCMonth() + 1;
  const north = ["冬", "冬", "春", "春", "春", "夏", "夏", "夏", "秋", "秋", "秋", "冬"][month - 1];
  const flip = { 冬: "夏", 夏: "冬", 春: "秋", 秋: "春" };
  const season = loc.lat >= 0 ? north : flip[north];
  return { text: `いまいる場所は${place}。いまの季節は${season}。`, place, updated_at: now };
}

// Free keyless "web search": recent Google News headlines plus the best-matching Japanese Wikipedia intro.
// Each source fails on its own; an empty text means nothing was found.
async function getSearch(url, env) {
  const q = (url.searchParams.get("q") ?? "").trim().slice(0, 80);
  if (!q) throw new HttpError(400, "q_required");
  const [news, wiki] = await Promise.all([
    fetch(`https://news.google.com/rss/search?q=${encodeURIComponent(q)}&hl=ja&gl=JP&ceid=JP:ja`, {
      headers: { "user-agent": UA }, signal: AbortSignal.timeout(8000),
    }).then((r) => (r.ok ? r.text() : "")).then((x) => parseNewsRss(x, 5)).catch(() => []),
    (async () => {
      const base = "https://ja.wikipedia.org/w/api.php?action=query&format=json&formatversion=2";
      const s = await getJson(`${base}&list=search&srlimit=1&srsearch=${encodeURIComponent(q)}`);
      const id = s.query?.search?.[0]?.pageid;
      if (!id) return null;
      const e = await getJson(`${base}&prop=extracts&exintro=1&explaintext=1&exchars=400&pageids=${id}`);
      const p = e.query?.pages?.[0];
      return p?.extract ? { title: p.title, extract: p.extract } : null;
    })().catch(() => null),
  ]);
  return json({ ok: true, q, text: formatSearch(news, wiki) });
}

async function digestSleep(env, now) {
  const rec = (await readKV(env, "sleep"))?.latest;
  if (!rec) throw new HttpError(404, "no_sleep_data");
  if (rec.date !== jstDate(now) && rec.date !== jstDate(now - 86400e3)) {
    throw new HttpError(404, "no_recent_sleep");
  }
  const text = formatSleep(rec);
  if (!text) throw new HttpError(404, "no_sleep_data");
  return { text, place: "", updated_at: rec.received_at };
}

// Yesterday's steps (fetched by the morning cron). The day before yesterday is still accepted, e.g. just after midnight.
async function digestSteps(env, now) {
  const rec = (await readKV(env, "steps"))?.latest;
  if (!rec) throw new HttpError(404, "no_steps_data");
  const today = jstDate(now);
  if (rec.date !== addDays(today, -1) && rec.date !== addDays(today, -2)) throw new HttpError(404, "no_recent_steps");
  const text = formatSteps(rec, today);
  if (!text) throw new HttpError(404, "no_steps_data");
  return { text, place: "", updated_at: rec.received_at };
}

async function postLocation(request, env, now) {
  const body = await readBody(request);
  const lat = toNumber(body.lat);
  const lon = toNumber(body.lon);
  // (0, 0) is never a real fix; it is what an unresolved variable tends to produce.
  if (!validLatLon(lat, lon) || (lat === 0 && lon === 0)) throw new HttpError(400, "invalid_lat_lon");
  const prev = await readKV(env, "loc");
  await writeKV(env, "loc", { lat, lon, at: now });
  const moved = prev ? Math.round(haversineM(prev.lat, prev.lon, lat, lon)) : null;
  return json({ ok: true, moved_m: moved });
}

async function postSleep(request, env, now) {
  const b = await readBody(request);
  const num = (v) => (Number.isFinite(Number(v)) && Number(v) >= 0 ? Number(v) : undefined);
  const minutes_asleep = num(b.minutes_asleep);
  if (!(minutes_asleep > 0)) throw new HttpError(400, "minutes_asleep_required");
  const date = /^\d{4}-\d{2}-\d{2}$/.test(b.date ?? "") ? b.date : jstDate(now);
  await saveSleep(env, {
    date, minutes_asleep,
    minutes_awake: num(b.minutes_awake), efficiency: num(b.efficiency),
    deep: num(b.deep), light: num(b.light), rem: num(b.rem),
  }, now);
  return json({ ok: true, date });
}

// Shared by POST /sleep and the Google Health cron.
const saveSleep = (env, rec, now) => writeKV(env, "sleep", { latest: { ...rec, received_at: now } });
const saveSteps = (env, rec, now) => writeKV(env, "steps", { latest: { ...rec, received_at: now } });

// Cron: pull last night's sleep and yesterday's steps from Google Health. Each part fails independently, and a
// part with no data never overwrites an earlier value. Only statuses are logged/stored, never health values.
async function collectHealth(env, now) {
  const status = { at: now, sleep: "", steps: "" };
  let token;
  try {
    token = await getAccessToken(env);
  } catch (e) {
    status.error = e.message;
    console.log("health: token failed:", e.message);
    await writeKV(env, "health_status", status);
    return status;
  }
  const parts = [
    ["sleep", fetchSleepRecord, saveSleep],
    ["steps", fetchStepsRecord, saveSteps],
  ];
  for (const [name, fetchRec, save] of parts) {
    try {
      const rec = await fetchRec(token, now);
      if (rec) {
        await save(env, rec, now);
        status[name] = `ok ${rec.date}`;
      } else {
        status[name] = "no_data";
      }
    } catch (e) {
      status[name] = `error: ${e.message}`;
      console.log(`health: ${name} failed:`, e.message);
    }
  }
  await writeKV(env, "health_status", status);
  console.log(`health: sleep=${status.sleep.split(":")[0]} steps=${status.steps.split(":")[0]}`);
  return status;
}

// What was said and the facts it was built from, so a later "どこにあるの？" can be answered from facts rather
// than guessed from the short spoken line. Keyed by the Discord message id that replies will point at.
const MSG_TTL_S = 3 * 24 * 3600;
const saveMsg = (env, id, text, facts) =>
  id
    ? env.STATE.put(`msg:${id}`, JSON.stringify({ text, facts: (facts ?? "").toString().slice(0, 800) }), {
        expirationTtl: MSG_TTL_S,
      })
    : undefined;

// Forwards a line the device just spoke to Discord. The webhook URL stays a Worker secret, off the device.
async function postLog(request, env) {
  if (!env.DISCORD_WEBHOOK_URL) throw new HttpError(503, "discord_not_configured");
  const body = await readBody(request);
  const payload = discordPayload(body.text);
  if (!payload) throw new HttpError(400, "text_required");
  let id;
  try {
    id = await postWebhookMessage(env, payload);
  } catch (e) {
    console.log(e.message);
    throw new HttpError(502, "discord_error");
  }
  await saveMsg(env, id, payload.content, body.facts);
  return json({ ok: true });
}

// ---- Discord -> device inbox. Reads the channel only when the device asks; nothing runs in the background. ----

const MSG_CACHE_MS = 8000;
const messageCaches = new WeakMap(); // per env, best effort within one isolate

function requireBot(env) {
  if (!botConfigured(env)) throw new HttpError(503, "inbox_not_configured");
}

async function getInbox(env, now) {
  requireBot(env);
  const cursor = await readKV(env, "inbox_cursor");
  if (cursor == null) {
    // First run: start from "now" so the channel's history is not read out loud.
    const latest = await fetchChannelMessages(env, { limit: 1 }).catch(discordFail);
    await writeKV(env, "inbox_cursor", latest[0]?.id ?? "0");
    return json({ ok: true, items: [] });
  }

  let cache = messageCaches.get(env);
  if (!cache || cache.cursor !== cursor || now - cache.at > MSG_CACHE_MS) {
    cache = { cursor, at: now, messages: await fetchChannelMessages(env, { after: cursor }).catch(discordFail) };
    messageCaches.set(env, cache);
  }
  let messages = cache.messages;
  messages = [...messages].sort((a, b) => compareSnowflake(a.id, b.id));

  const items = [];
  let maxSeen = cursor;
  for (const m of messages) {
    if (isSnowflake(m.id) && compareSnowflake(m.id, maxSeen) > 0) maxSeen = m.id;
    const item = toInboxItem(m, env.DISCORD_USER_ID);
    if (item) items.push(item);
  }
  // Only other people's messages arrived: skip past them so they are not re-read every poll.
  if (!items.length && maxSeen !== cursor) await writeKV(env, "inbox_cursor", maxSeen);

  const out = [];
  for (const it of items.slice(0, 1)) {
    let reply_to = null;
    if (it.ref) {
      const known = await readKV(env, `msg:${it.ref.id}`);
      reply_to = known
        ? { id: it.ref.id, text: known.text, facts: known.facts, by: "stackchan" }
        : { id: it.ref.id, text: it.ref.text, facts: "", by: it.ref.bot ? "bot" : "other" };
    }
    out.push({ id: it.id, text: it.text, ts: it.ts, reply_to });
  }
  return json({ ok: true, items: out });
}

const discordFail = (e) => {
  console.log(e.message);
  throw new HttpError(502, "discord_error");
};

// The device acknowledges only after it has handled an item, so a failure mid-way does not lose it.
async function postInboxAck(request, env) {
  requireBot(env);
  const { id } = await readBody(request);
  if (!isSnowflake(id)) throw new HttpError(400, "invalid_id");
  const cursor = await readKV(env, "inbox_cursor");
  if (cursor == null || compareSnowflake(id, cursor) > 0) await writeKV(env, "inbox_cursor", id);
  return json({ ok: true });
}

// Posts the answer as a real Discord reply to the user's message, so the conversation chains.
async function postReply(request, env) {
  requireBot(env);
  const body = await readBody(request);
  const payload = discordPayload(body.text);
  if (!payload) throw new HttpError(400, "text_required");
  if (body.to != null && !isSnowflake(body.to)) throw new HttpError(400, "invalid_to");
  const msg = await postBotMessage(env, {
    ...payload,
    allowed_mentions: { parse: [], replied_user: false },
    ...(body.to ? { message_reference: { message_id: body.to, fail_if_not_exists: false } } : {}),
  }).catch(discordFail);
  await saveMsg(env, msg?.id, payload.content, body.facts);
  return json({ ok: true });
}

async function getDigest(category, env, now) {
  if (category === "sleep") return json({ ok: true, category, ...(await digestSleep(env, now)) });
  if (category === "steps") return json({ ok: true, category, ...(await digestSteps(env, now)) });
  const loc = await readKV(env, "loc");
  if (!loc) throw new HttpError(409, "no_location");
  const handler = { weather: digestWeather, gourmet: digestGourmet, trivia: digestTrivia, place: digestPlace }[category];
  return json({ ok: true, category, away: isAway(env, loc), ...(await handler(env, loc, now)) });
}

// True when the phone is farther than AWAY_KM (default 30) from HOME_LAT/HOME_LON. Without a home it is never away.
function isAway(env, loc) {
  const lat = toNumber(env.HOME_LAT);
  const lon = toNumber(env.HOME_LON);
  if (!validLatLon(lat, lon)) return false;
  const km = toNumber(env.AWAY_KM);
  return haversineM(lat, lon, loc.lat, loc.lon) > (km > 0 ? km : 30) * 1000;
}

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const path = url.pathname.replace(/\/+$/, "") || "/";
    if (path === "/") return json({ ok: true, name: "stackchan-digest" });

    if (!env.SHARED_TOKEN) return fail(500, "server_not_configured");
    const auth = request.headers.get("authorization") ?? "";
    if (!constantTimeEqual(auth, `Bearer ${env.SHARED_TOKEN}`)) return fail(401, "unauthorized");

    const now = Date.now();
    try {
      if (path === "/location" && request.method === "POST") return await postLocation(request, env, now);
      if (path === "/location" && request.method === "GET") {
        const loc = await readKV(env, "loc");
        return loc ? json({ ok: true, ...loc }) : fail(404, "no_location");
      }
      if (path === "/search" && request.method === "GET") return await getSearch(url, env);
      if (path === "/sleep" && request.method === "POST") return await postSleep(request, env, now);
      if (path === "/log" && request.method === "POST") return await postLog(request, env);
      if (path === "/inbox" && request.method === "GET") return await getInbox(env, now);
      if (path === "/inbox/ack" && request.method === "POST") return await postInboxAck(request, env);
      if (path === "/reply" && request.method === "POST") return await postReply(request, env);
      if (path === "/health/status" && request.method === "GET") {
        const st = await readKV(env, "health_status");
        return st ? json({ ok: true, ...st }) : fail(404, "no_status");
      }
      if (path === "/health/refresh" && request.method === "POST") {
        return json({ ok: true, ...(await collectHealth(env, now)) });
      }
      const m = path.match(/^\/digest\/([a-z]+)$/);
      if (m && request.method === "GET") {
        if (!CATEGORIES.includes(m[1])) return fail(404, "unknown_category");
        return await getDigest(m[1], env, now);
      }
      return fail(404, "not_found");
    } catch (e) {
      if (e instanceof HttpError) return fail(e.status, e.message);
      console.log("unhandled error:", e.stack ?? e.message);
      return fail(502, "upstream_error");
    }
  },

  // Cron Triggers (see wrangler.toml)
  async scheduled(_controller, env, ctx) {
    ctx.waitUntil(collectHealth(env, Date.now()));
  },
};
