import { test } from "node:test";
import assert from "node:assert/strict";
import {
  constantTimeEqual, formatGourmet, formatSleep, formatTrivia, formatWeather,
  haversineM, jstDate, parsePlace, trimExtract, validLatLon, weatherCodeText,
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
