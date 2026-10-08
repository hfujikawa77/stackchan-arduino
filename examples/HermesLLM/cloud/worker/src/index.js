import {
  addDays, constantTimeEqual, discordPayload, formatGourmet, formatSleep, formatSteps, formatTrivia, formatWeather,
  haversineM, jstDate, parsePlace, pickRandom, toNumber, trimExtract, validLatLon,
} from "./lib.js";
import { fetchSleepRecord, fetchStepsRecord, getAccessToken } from "./google_health.js";

const UA = "stackchan-digest/1.0 (personal hobby project)";
const CATEGORIES = ["weather", "gourmet", "trivia", "sleep", "steps"];

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

// Forwards a line the device just spoke to Discord. The webhook URL stays a Worker secret, off the device.
async function postLog(request, env) {
  if (!env.DISCORD_WEBHOOK_URL) throw new HttpError(503, "discord_not_configured");
  const body = await readBody(request);
  const payload = discordPayload(body.text);
  if (!payload) throw new HttpError(400, "text_required");
  const res = await fetch(env.DISCORD_WEBHOOK_URL, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(payload),
    signal: AbortSignal.timeout(8000),
  });
  // Never echo Discord's response: it can carry the webhook id/token.
  if (!res.ok) {
    console.log("discord webhook responded", res.status);
    throw new HttpError(502, "discord_error");
  }
  return json({ ok: true });
}

async function getDigest(category, env, now) {
  if (category === "sleep") return json({ ok: true, category, ...(await digestSleep(env, now)) });
  if (category === "steps") return json({ ok: true, category, ...(await digestSteps(env, now)) });
  const loc = await readKV(env, "loc");
  if (!loc) throw new HttpError(409, "no_location");
  const handler = { weather: digestWeather, gourmet: digestGourmet, trivia: digestTrivia }[category];
  return json({ ok: true, category, ...(await handler(env, loc, now)) });
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
      if (path === "/sleep" && request.method === "POST") return await postSleep(request, env, now);
      if (path === "/log" && request.method === "POST") return await postLog(request, env);
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
