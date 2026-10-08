// Google Health API access (I/O only; parsing lives in lib.js so it can be unit-tested).
// Request shapes (filter syntax, dailyRollUp's civil-datetime range) were verified against the real API.
import { addDays, jstDate, jstDayStartUtc, pickMainSleep, sleepRecordFromGoogle, stepsRecordFromRollup } from "./lib.js";

const BASE_URL = "https://health.googleapis.com/v4";
const TOKEN_URL = "https://oauth2.googleapis.com/token";
const REQUIRED = ["GOOGLE_CLIENT_ID", "GOOGLE_CLIENT_SECRET", "GOOGLE_REFRESH_TOKEN"];

// Error bodies are truncated; secrets are never part of them (and are never logged by this module).
async function parse(res, label) {
  const text = await res.text();
  if (!res.ok) throw new Error(`${label} responded ${res.status}: ${text.slice(0, 200)}`);
  return JSON.parse(text);
}

export async function getAccessToken(env) {
  if (REQUIRED.some((k) => !env[k])) throw new Error("google_not_configured");
  const res = await fetch(TOKEN_URL, {
    method: "POST",
    body: new URLSearchParams({
      client_id: env.GOOGLE_CLIENT_ID,
      client_secret: env.GOOGLE_CLIENT_SECRET,
      refresh_token: env.GOOGLE_REFRESH_TOKEN,
      grant_type: "refresh_token",
    }),
    signal: AbortSignal.timeout(8000),
  });
  return (await parse(res, "token endpoint")).access_token;
}

const headers = (token) => ({ Authorization: `Bearer ${token}`, Accept: "application/json" });

// The sleep that ended today (JST), i.e. last night's sleep.
export async function fetchSleepRecord(token, now) {
  const today = jstDate(now);
  const filter =
    `sleep.interval.end_time >= "${jstDayStartUtc(today)}" AND ` +
    `sleep.interval.end_time < "${jstDayStartUtc(addDays(today, 1))}"`;
  const url = `${BASE_URL}/users/me/dataTypes/sleep/dataPoints?${new URLSearchParams({ filter, pageSize: "25" })}`;
  const res = await fetch(url, { headers: headers(token), signal: AbortSignal.timeout(8000) });
  const body = await parse(res, "sleep");
  return sleepRecordFromGoogle(pickMainSleep(body.dataPoints));
}

const civil = (date) => {
  const [year, month, day] = date.split("-").map(Number);
  return { date: { year, month, day }, time: {} };
};

// Yesterday's total plus the 7 days before it. dailyRollUp de-duplicates Fitbit and phone (Health Connect) steps.
export async function fetchStepsRecord(token, now) {
  const today = jstDate(now);
  const yesterday = addDays(today, -1);
  const res = await fetch(`${BASE_URL}/users/me/dataTypes/steps/dataPoints:dailyRollUp`, {
    method: "POST",
    headers: { ...headers(token), "Content-Type": "application/json" },
    body: JSON.stringify({ range: { start: civil(addDays(yesterday, -7)), end: civil(today) }, windowSizeDays: 1 }),
    signal: AbortSignal.timeout(8000),
  });
  return stepsRecordFromRollup(await parse(res, "steps"), yesterday);
}
