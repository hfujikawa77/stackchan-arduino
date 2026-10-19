import { test } from "node:test";
import assert from "node:assert/strict";
import {
  addDays, constantTimeEqual, discordPayload, formatGourmet, formatSleep, formatSteps, formatTrivia, formatWeather,
  haversineM, jstDate, jstDayStartUtc, parsePlace, pickMainSleep, sleepRecordFromGoogle,
  stepsRecordFromRollup, toNumber, trimExtract, validLatLon, weatherCodeText,
} from "../src/lib.js";

test("haversineM: Tokyo Station to Yokohama Station is about 27 km", () => {
  const d = haversineM(35.681, 139.767, 35.466, 139.622);
  assert.ok(d > 26000 && d < 29000, `got ${d}`);
  assert.equal(Math.round(haversineM(35, 139, 35, 139)), 0);
});

test("validLatLon rejects NaN, strings and out-of-range", () => {
  assert.ok(validLatLon(35.6, 139.7));
  assert.ok(!validLatLon(NaN, 139));
  assert.ok(!validLatLon("35", 139));
  assert.ok(!validLatLon(91, 0));
  assert.ok(!validLatLon(0, 181));
});

test("toNumber treats blank and non-numeric input as NaN, not 0", () => {
  assert.equal(toNumber("35.68"), 35.68);
  assert.equal(toNumber(139.76), 139.76);
  assert.ok(Number.isNaN(toNumber("")));
  assert.ok(Number.isNaN(toNumber("  ")));
  assert.ok(Number.isNaN(toNumber("{lat}")));
  assert.ok(Number.isNaN(toNumber(null)));
  assert.ok(Number.isNaN(toNumber(undefined)));
});

test("constantTimeEqual", () => {
  assert.ok(constantTimeEqual("Bearer abc", "Bearer abc"));
  assert.ok(!constantTimeEqual("Bearer abc", "Bearer abd"));
  assert.ok(!constantTimeEqual("Bearer abc", "Bearer abcd"));
  assert.ok(!constantTimeEqual(undefined, "x"));
});

test("formatWeather matches the firmware wording and prefixes the place", () => {
  const om = {
    current: { weather_code: 61, temperature_2m: 18.4 },
    daily: {
      temperature_2m_max: [22.6], temperature_2m_min: [14.2],
      precipitation_probability_max: [70],
    },
  };
  assert.equal(
    formatWeather("東京都渋谷区", om),
    "東京都渋谷区の天気は雨、現在の気温18度、最高23度、最低14度、降水確率70%",
  );
  assert.equal(formatWeather("", om).startsWith("天気は雨"), true);
  assert.equal(formatWeather("x", {}), "");
  assert.equal(weatherCodeText(99), "雷雨");
});

test("parsePlace joins address parts without duplicates", () => {
  const nominatim = {
    address: { state: "東京都", city: "渋谷区", suburb: "渋谷区", neighbourhood: "神南" },
  };
  assert.equal(parsePlace(nominatim), "東京都渋谷区神南");
  assert.equal(parsePlace({}), "");
});

test("formatGourmet omits missing fields", () => {
  assert.equal(
    formatGourmet({ name: "居酒屋A", genre: { name: "居酒屋" }, access: "渋谷駅徒歩3分", catch: "海鮮が自慢", budget: { name: "3001～4000円" } }),
    "居酒屋A（居酒屋）。場所は、渋谷駅徒歩3分。海鮮が自慢。予算は3001～4000円",
  );
  assert.equal(formatGourmet({ name: "B" }), "B");
  assert.equal(formatGourmet(undefined), "");
});

test("trimExtract cuts at a sentence end when possible", () => {
  const long = "あ".repeat(50) + "。" + "い".repeat(200);
  assert.equal(trimExtract(long, 100), "あ".repeat(50) + "。");
  assert.equal(trimExtract("短い。", 100), "短い。");
});

test("formatTrivia", () => {
  assert.equal(
    formatTrivia("東京都千代田区", { title: "東京駅", extract: "赤レンガの駅舎で知られる。" }),
    "現在地は東京都千代田区。近くにある「東京駅」について。赤レンガの駅舎で知られる。",
  );
  assert.equal(formatTrivia("東京都千代田区", undefined), "現在地は東京都千代田区。");
  assert.equal(formatTrivia("", undefined), "");
});

test("jstDate rolls over at 15:00 UTC", () => {
  assert.equal(jstDate(Date.UTC(2026, 9, 5, 14, 59)), "2026-10-05");
  assert.equal(jstDate(Date.UTC(2026, 9, 5, 15, 0)), "2026-10-06");
});

test("formatSleep", () => {
  assert.equal(
    formatSleep({ minutes_asleep: 432, efficiency: 91.2, deep: 70, rem: 95, minutes_awake: 28 }),
    "昨夜の睡眠時間は7時間12分、睡眠効率は91%、深い睡眠は70分、レム睡眠は95分、途中で起きていた時間は28分",
  );
  assert.equal(formatSleep({ minutes_asleep: 0 }), "");
});

// ---- Google Health API (synthetic values in the real response shape: numbers are strings) ----

const sleepPoint = (endTime, { main = true, inPeriod = "475", asleep = "432" } = {}) => ({
  sleep: {
    interval: { startTime: "2026-10-05T15:00:00Z", endTime },
    type: "STAGES",
    metadata: { mainSleep: main },
    summary: {
      minutesInSleepPeriod: inPeriod,
      minutesAsleep: asleep,
      minutesAwake: "43",
      stagesSummary: [
        { type: "AWAKE", minutes: "43", count: "3" },
        { type: "LIGHT", minutes: "250", count: "9" },
        { type: "DEEP", minutes: "70", count: "4" },
        { type: "REM", minutes: "112", count: "5" },
      ],
    },
  },
});

test("jstDayStartUtc / addDays", () => {
  assert.equal(jstDayStartUtc("2026-10-06"), "2026-10-05T15:00:00Z");
  assert.equal(addDays("2026-10-01", -1), "2026-09-30");
  assert.equal(addDays("2026-12-31", 1), "2027-01-01");
});

test("sleepRecordFromGoogle maps stages, derives efficiency and the JST wake date", () => {
  // 21:51Z on 10/5 is 06:51 JST on 10/6
  const rec = sleepRecordFromGoogle(sleepPoint("2026-10-05T21:51:00Z"));
  assert.deepEqual(rec, {
    date: "2026-10-06", minutes_asleep: 432, minutes_awake: 43, efficiency: 91, deep: 70, light: 250, rem: 112,
  });
  assert.equal(
    formatSleep(rec),
    "昨夜の睡眠時間は7時間12分、睡眠効率は91%、深い睡眠は70分、レム睡眠は112分、途中で起きていた時間は43分",
  );
});

test("sleepRecordFromGoogle returns undefined for missing or empty data", () => {
  assert.equal(sleepRecordFromGoogle(undefined), undefined);
  assert.equal(sleepRecordFromGoogle({ sleep: {} }), undefined);
  assert.equal(sleepRecordFromGoogle(sleepPoint("2026-10-05T21:51:00Z", { asleep: "0" })), undefined);
  // no time-in-bed -> efficiency is simply omitted
  assert.equal(sleepRecordFromGoogle(sleepPoint("2026-10-05T21:51:00Z", { inPeriod: "" })).efficiency, undefined);
});

test("pickMainSleep prefers the main sleep over a longer nap, else the longest", () => {
  const main = sleepPoint("2026-10-05T21:00:00Z", { main: true, inPeriod: "400" });
  const nap = sleepPoint("2026-10-06T05:00:00Z", { main: false, inPeriod: "500" });
  assert.equal(pickMainSleep([nap, main]), main);
  const a = sleepPoint("2026-10-05T21:00:00Z", { main: false, inPeriod: "100" });
  assert.equal(pickMainSleep([a, nap]), nap);
  assert.equal(pickMainSleep([]), undefined);
  assert.equal(pickMainSleep(undefined), undefined);
});

const rollupDay = (day, countSum) => ({
  civilStartTime: { date: { year: 2026, month: 10, day }, time: {} },
  steps: { countSum },
});

test("stepsRecordFromRollup returns the day's count and the average of the earlier days", () => {
  const rollup = { rollupDataPoints: [rollupDay(3, "2000"), rollupDay(4, "4000"), rollupDay(5, "3270")] };
  assert.deepEqual(stepsRecordFromRollup(rollup, "2026-10-05"), { date: "2026-10-05", count: 3270, avg7: 3000 });
  assert.deepEqual(stepsRecordFromRollup({ rollupDataPoints: [rollupDay(5, "10")] }, "2026-10-05"), {
    date: "2026-10-05", count: 10, avg7: undefined,
  });
  assert.equal(stepsRecordFromRollup(rollup, "2026-10-06"), undefined);
  assert.equal(stepsRecordFromRollup({}, "2026-10-05"), undefined);
});

test("discordPayload trims, truncates, and disables mentions", () => {
  assert.deepEqual(discordPayload("  こんにちは  "), { content: "こんにちは", allowed_mentions: { parse: [] } });
  assert.equal(discordPayload("   "), undefined);
  assert.equal(discordPayload(undefined), undefined);
  const long = discordPayload("あ".repeat(3000));
  assert.equal(long.content.length, 1900);
  assert.ok(long.content.endsWith("…"));
  assert.deepEqual(discordPayload("@everyone やあ").allowed_mentions, { parse: [] });
});

test("formatSteps states facts and names the date when it is not yesterday", () => {
  const rec = { date: "2026-10-05", count: 3270, avg7: 3000 };
  assert.equal(formatSteps(rec, "2026-10-06"), "昨日の歩数は3270歩、その前の1週間の1日平均は3000歩");
  assert.equal(formatSteps(rec, "2026-10-07"), "10月5日の歩数は3270歩、その前の1週間の1日平均は3000歩");
  assert.equal(formatSteps({ date: "2026-10-05", count: 3270 }, "2026-10-06"), "昨日の歩数は3270歩");
  assert.equal(formatSteps(undefined, "2026-10-06"), "");
});

test("parseNewsRss keeps titles, strips the source suffix and drops summaries that only repeat the title", async () => {
  const { parseNewsRss, formatSearch } = await import("../src/lib.js");
  const xml = `<rss><channel><item><title><![CDATA[孫が優勝 - スポーツ紙]]></title>` +
    `<description><![CDATA[<a href="x">孫が優勝</a>&nbsp;&nbsp;決勝で逆転勝ちを収めた。]]></description></item>` +
    `<item><title>別の話 - 通信社</title><description>別の話</description></item></channel></rss>`;
  const items = parseNewsRss(xml, 5);
  assert.equal(items.length, 2);
  assert.equal(items[0].title, "孫が優勝");
  assert.match(items[0].summary, /逆転勝ち/);
  assert.equal(items[1].summary, "");
  assert.match(formatSearch(items, { title: "X", extract: "説明" }), /・X: 説明/);
});
