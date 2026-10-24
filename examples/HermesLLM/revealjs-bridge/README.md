# Stack-chan x reveal.js bridge

Head swipe on Stack-chan moves the slides; slide changes drive Stack-chan's expression.
No PC server: the browser connects straight to Stack-chan's `ws://<ip>/servo-ws`.

## Setup
1. Turn on **Slide mode**: on-device Settings > Main > `Slide mode: ON` > Save, or the Web config page (checkbox) > Save (the device restarts). Default is OFF.
2. Open the deck from `file://` or `http://` (an `https://` page cannot reach `ws://`):
   `index.html?sc=<stackchan-ip>`. The IP is remembered in localStorage.
3. To use your own deck: add `<script src="stackchan-bridge.js"></script>` and put `StackchanBridge` in `plugins`.

## Behaviour
| Event | Result |
|---|---|
| Swipe front -> back | `{"type":"control","action":"next"}` -> `Reveal.next()` |
| Swipe back -> front | `prev` -> `Reveal.prev()` |
| Slide changes (key, touch, swipe) | `{"type":"slide","indexh":N,"indexv":0}` -> serial log + expression |
| Taps (2x / 3x) | unchanged |

Slide mode ON but no browser connected: swipes are normal head pats. Slide mode OFF: `slide` is only logged.
Expressions cycle every 6 slides by `indexh` (table in `slide_apply_tick()`, `main.cpp`): page 1 Neutral, then Happy, Doubt, Sad, Angry, Sleepy, repeating from page 7.
One browser at a time (a new connection replaces the old one). The page reconnects automatically (1-5 s) and sends a heartbeat every 10 s.

## Check
Serial monitor: `Slide changed: h=2 v=0` on slide change, `Slide control: next` on swipe.

## Not done (Phase 3 hooks)
Per-slide servo / cached voice / per-action ON/OFF: extend `slide_apply_tick()`. Vertical slides and left/right input are out of scope (the head sensor only has a front-back axis).

## Existing deck on a hosted (https) site
An `https://` page cannot open `ws://` to a LAN IP (mixed content). Either:
- Chrome: lock icon > Site settings > **Insecure content: Allow** for that site (and allow the local-network prompt if asked), or
- clone the deck and open it from `file://` / `http://localhost`.

To try without editing the deck, open `<deck-url>?sc=<ip>`, then paste `console-snippet.js` into the DevTools console.
For a permanent setup add `stackchan-bridge.js` to the deck repo and put `StackchanBridge` in `plugins`.

Bubble shows "N. <title>" (title = `data-stackchan-title` on the slide, else its first h1-h4). Long titles scroll for 4 s.

## Per-slide motion (once per transition)
`<section data-stackchan-motion="nod">` - `nod` (default), `shake` (head left-right), `spin` (full X-axis spin, same as the triple tap; runs without blocking the loop), `none`.
Bubble title override: `<section data-stackchan-title="Short title">`.

## Per-slide LED (held until the next transition)
`<section data-stackchan-led='{"pattern":"breath","color":"#0078C8","speed":5}'>` - the same JSON the Web UI LED page posts to `/led`
(`pattern`: solid, blink, breath, chase, rainbow, wave, split, split_blink, split_breath, split_wave, split_chase, alternate; `color`, `color2` as `#RRGGBB`; `speed` 1-10).
A preset name also works: `data-stackchan-led="ocean"` (warm_white, cool_blue, alert_red, calm_breath, christmas, ocean, rainbow, party).
The LED stays until the next slide; a slide without the attribute turns a held LED off.

Explicit expression: `data-stackchan-expression="neutral|happy|sad|angry|doubt|sleepy"` (default: a 6-step cycle by slide index).
