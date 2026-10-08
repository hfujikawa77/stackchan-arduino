// Pure helpers (no I/O) so they can be unit-tested with `node --test`.

export function haversineM(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const rad = (d) => (d * Math.PI) / 180;
  const dLat = rad(lat2 - lat1);
  const dLon = rad(lon2 - lon1);
  const a =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(rad(lat1)) * Math.cos(rad(lat2)) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.min(1, Math.sqrt(a)));
}

// Number("") is 0, so blank strings must not be accepted as coordinates.
export function toNumber(v) {
  if (typeof v === "number") return v;
  if (typeof v === "string" && v.trim() !== "") return Number(v);
  return NaN;
}

export function validLatLon(lat, lon) {
  return (
    typeof lat === "number" && typeof lon === "number" &&
    Number.isFinite(lat) && Number.isFinite(lon) &&
    lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180
  );
}

export function constantTimeEqual(a, b) {
  if (typeof a !== "string" || typeof b !== "string") return false;
  const len = Math.max(a.length, b.length);
  let diff = a.length ^ b.length;
  for (let i = 0; i < len; i++) {
    diff |= (a.charCodeAt(i) || 0) ^ (b.charCodeAt(i) || 0);
  }
  return diff === 0;
}

export function weatherCodeText(code) {
  if (code === 0) return "快晴";
  if (code === 1) return "晴れ";
  if (code === 2) return "薄曇り";
  if (code === 3) return "曇り";
  if (code === 45 || code === 48) return "霧";
  if (code >= 51 && code <= 57) return "霧雨";
  if (code >= 61 && code <= 67) return "雨";
  if (code >= 71 && code <= 77) return "雪";
  if (code >= 80 && code <= 82) return "にわか雨";
  if (code === 85 || code === 86) return "にわか雪";
  if (code >= 95) return "雷雨";
  return "不明";
}

// Same wording as the firmware's fetch_weather_summary(), prefixed with the place.
export function formatWeather(place, om) {
  const cur = om?.current;
  if (!cur) return "";
  const d = om.daily ?? {};
  const r = (v) => Math.round(v ?? 0);
  const head = place ? `${place}の` : "";
  return (
    `${head}天気は${weatherCodeText(cur.weather_code ?? -1)}、` +
    `現在の気温${r(cur.temperature_2m)}度、` +
    `最高${r(d.temperature_2m_max?.[0])}度、最低${r(d.temperature_2m_min?.[0])}度、` +
    `降水確率${r(d.precipitation_probability_max?.[0])}%`
  );
}

// Nominatim reverse (accept-language=ja) -> "東京都渋谷区神南".
export function parsePlace(nominatim) {
  const a = nominatim?.address;
  if (!a) return "";
  const parts = [
    a.state ?? a.province,
    a.city ?? a.town ?? a.village ?? a.municipality ?? a.county,
    a.city_district ?? a.suburb,
    a.quarter ?? a.neighbourhood,
  ].filter(Boolean);
  const out = [];
  for (const p of parts) if (!out.includes(p)) out.push(p);
  return out.join("");
}

export function pickRandom(arr, rng = Math.random) {
  if (!arr.length) return undefined;
  return arr[Math.floor(rng() * arr.length)];
}

export function formatGourmet(shop) {
  if (!shop?.name) return "";
  const genre = shop.genre?.name ? `（${shop.genre.name}）` : "";
  const bits = [`${shop.name}${genre}`];
  if (shop.access) bits.push(`場所は、${shop.access}`);
  if (shop.catch) bits.push(shop.catch);
  if (shop.budget?.name) bits.push(`予算は${shop.budget.name}`);
  return bits.join("。");
}

// Trim a Wikipedia intro to ~max chars, preferring to cut at a sentence end.
export function trimExtract(text, max = 160) {
  const t = (text ?? "").replace(/\s+/g, " ").trim();
  if (t.length <= max) return t;
  const cut = t.slice(0, max);
  const end = cut.lastIndexOf("。");
  return end > max / 3 ? cut.slice(0, end + 1) : cut;
}

export function formatTrivia(place, page) {
  const head = place ? `現在地は${place}。` : "";
  if (!page) return head;
  const extract = trimExtract(page.extract);
  return `${head}近くにある「${page.title}」について。${extract}`;
}

export function jstDate(ms) {
  const d = new Date(ms + 9 * 3600 * 1000);
  return d.toISOString().slice(0, 10);
}

export function formatSleep(rec) {
  if (!rec || !(rec.minutes_asleep > 0)) return "";
  const h = Math.floor(rec.minutes_asleep / 60);
  const m = Math.round(rec.minutes_asleep % 60);
  const bits = [`昨夜の睡眠時間は${h}時間${m}分`];
  if (rec.efficiency > 0) bits.push(`睡眠効率は${Math.round(rec.efficiency)}%`);
  if (rec.deep > 0) bits.push(`深い睡眠は${Math.round(rec.deep)}分`);
  if (rec.rem > 0) bits.push(`レム睡眠は${Math.round(rec.rem)}分`);
  if (rec.minutes_awake > 0) bits.push(`途中で起きていた時間は${Math.round(rec.minutes_awake)}分`);
  return bits.join("、");
}

// ---- Google Health API (https://developers.google.com/health/) ----
// Shapes below follow real responses: numbers arrive as strings ("362"), times are RFC-3339 UTC.

export function addDays(date, n) {
  return new Date(Date.parse(`${date}T00:00:00Z`) + n * 86400e3).toISOString().slice(0, 10);
}

// 00:00 JST of `date` (YYYY-MM-DD) as RFC-3339 UTC, for filters like sleep.interval.end_time >= "...".
export function jstDayStartUtc(date) {
  return new Date(Date.parse(`${date}T00:00:00Z`) - 9 * 3600e3).toISOString().replace(/\.\d{3}Z$/, "Z");
}

const toInt = (v) => {
  if (v === null || v === undefined || v === "") return undefined;
  const n = Number(v);
  return Number.isFinite(n) ? n : undefined;
};

// Prefer the main sleep (not a nap); among several, the longest sleep period.
export function pickMainSleep(points) {
  const list = (points ?? []).filter((p) => p?.sleep?.interval?.endTime && p.sleep.summary);
  if (!list.length) return undefined;
  const mains = list.filter((p) => p.sleep.metadata?.mainSleep);
  const pool = mains.length ? mains : list;
  const len = (p) => toInt(p.sleep.summary.minutesInSleepPeriod) ?? 0;
  return pool.reduce((a, b) => (len(b) > len(a) ? b : a));
}

// dataPoint -> the record shape POST /sleep stores. efficiency = asleep / time in bed (matches the Fitbit app).
export function sleepRecordFromGoogle(point) {
  const s = point?.sleep;
  const minutes_asleep = toInt(s?.summary?.minutesAsleep);
  if (!(minutes_asleep > 0) || !s?.interval?.endTime) return undefined;
  const inPeriod = toInt(s.summary.minutesInSleepPeriod);
  const stage = (type) => toInt((s.summary.stagesSummary ?? []).find((x) => x.type === type)?.minutes);
  return {
    date: jstDate(Date.parse(s.interval.endTime)),
    minutes_asleep,
    minutes_awake: toInt(s.summary.minutesAwake),
    efficiency: inPeriod > 0 ? Math.round((minutes_asleep / inPeriod) * 100) : undefined,
    deep: stage("DEEP"),
    light: stage("LIGHT"),
    rem: stage("REM"),
  };
}

// dailyRollUp response -> { date, count, avg7 }. avg7 is the mean of the earlier days present in the response.
export function stepsRecordFromRollup(rollup, date) {
  const byDate = new Map();
  for (const p of rollup?.rollupDataPoints ?? []) {
    const d = p?.civilStartTime?.date;
    const n = toInt(p?.steps?.countSum);
    if (d && n !== undefined) {
      byDate.set(`${d.year}-${String(d.month).padStart(2, "0")}-${String(d.day).padStart(2, "0")}`, n);
    }
  }
  const count = byDate.get(date);
  if (count === undefined) return undefined;
  const prev = [...byDate].filter(([k]) => k < date).map(([, v]) => v);
  const avg7 = prev.length ? Math.round(prev.reduce((a, b) => a + b, 0) / prev.length) : undefined;
  return { date, count, avg7 };
}

// Facts only; the tone ("よく歩いたね" etc.) is left to the device-side prompt.
export function formatSteps(rec, today) {
  if (!rec || !(rec.count >= 0)) return "";
  const [, m, d] = rec.date.split("-").map(Number);
  const label = rec.date === addDays(today, -1) ? "昨日" : `${m}月${d}日`;
  const bits = [`${label}の歩数は${rec.count}歩`];
  if (rec.avg7 > 0) bits.push(`その前の1週間の1日平均は${rec.avg7}歩`);
  return bits.join("、");
}

// Discord webhook body for a spoken line. allowed_mentions.parse=[] makes "@everyone" in LLM text inert.
export function discordPayload(text, max = 1900) {
  const t = (text ?? "").toString().trim();
  if (!t) return undefined;
  const content = t.length > max ? `${t.slice(0, max - 1)}…` : t;
  return { content, allowed_mentions: { parse: [] } };
}
