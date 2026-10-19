// Discord I/O. Errors carry only a status, never a response body (it can echo ids or tokens).

const API = "https://discord.com/api/v10";
const UA = "DiscordBot (https://github.com/hfujikawa77/stackchan-arduino, 1.0)";

export const botConfigured = (env) =>
  Boolean(env.DISCORD_BOT_TOKEN && env.DISCORD_CHANNEL_ID && env.DISCORD_USER_ID);

async function botFetch(env, path, init = {}) {
  const res = await fetch(`${API}${path}`, {
    ...init,
    headers: {
      authorization: `Bot ${env.DISCORD_BOT_TOKEN}`,
      "user-agent": UA,
      ...(init.body ? { "content-type": "application/json" } : {}),
    },
    signal: AbortSignal.timeout(8000),
  });
  if (!res.ok) throw new Error(`discord ${init.method ?? "GET"} ${path.split("?")[0]} -> ${res.status}`);
  return res.json();
}

// Newest-first list of messages after `after` (or the latest `limit` when omitted).
export function fetchChannelMessages(env, { after, limit = 50 } = {}) {
  const q = new URLSearchParams({ limit: String(limit) });
  if (after) q.set("after", after);
  return botFetch(env, `/channels/${env.DISCORD_CHANNEL_ID}/messages?${q}`);
}

export function postBotMessage(env, payload) {
  return botFetch(env, `/channels/${env.DISCORD_CHANNEL_ID}/messages`, {
    method: "POST",
    body: JSON.stringify(payload),
  });
}

// ?wait=true makes Discord return the created message, whose id is what replies will point at.
export async function postWebhookMessage(env, payload) {
  const url = new URL(env.DISCORD_WEBHOOK_URL);
  url.searchParams.set("wait", "true");
  const res = await fetch(url, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(payload),
    signal: AbortSignal.timeout(8000),
  });
  if (!res.ok) throw new Error(`discord webhook -> ${res.status}`);
  try {
    return (await res.json())?.id;
  } catch {
    return undefined;
  }
}
