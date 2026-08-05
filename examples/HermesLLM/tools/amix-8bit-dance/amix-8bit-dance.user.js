// ==UserScript==
// @name         Stack-chan 8bit Dance Bridge (amix EASY 8BIT EDITOR)
// @namespace    stackchan-lab
// @version      0.1.0
// @description  EASY 8BIT EDITOR の再生ビートを Web Serial 経由で Stack-chan(HermesLLM) に送って踊らせる
// @author       stackchan-lab
// @match        https://amix-design.com/tl/tool-s-8bit/*
// @run-at       document-idle
// @grant        none
// ==/UserScript==
//
// 仕組み:
//   EASY 8BIT EDITOR は React 製で、内部に Web Audio シーケンサ(コントローラ)を持つ。
//   コントローラは React fiber の stateNode.logic として到達でき、
//     .state.bpm      現在の BPM
//     .state.playing  再生中フラグ
//     .state.steps    ループのステップ数(16分音符単位)
//     .nextStep       進行中のステップカウンタ (0..steps-1)
//     .graph.ctx      AudioContext
//   1 ステップ = 16分音符。よって
//     拍(4分音符)      = nextStep % 4 === 0
//     小節頭(4/4想定)  = nextStep % 16 === 0  → accent
//
//   拍が変わるたびに、HermesLLM ファーム(serial_beat_tick)が解釈する行を USB シリアルへ送る:
//     "B <step> <accent 0|1> <motion lr|ud> <bpm>\n"   拍のパルス
//     "C\n"                                            正面に戻す(停止時)
//
// 使い方:
//   1) Tampermonkey にこのスクリプトを入れる。
//   2) Stack-chan(CoreS3) を PC に USB 接続し、HermesLLM を書き込んでおく。
//   3) サイトを開き、右下パネルの「🔌 接続」→ シリアルポート(CoreS3)を選ぶ。
//   4) スペース or PLAY で再生 → 拍に合わせて Stack-chan が首を振る。
//
// 注意:
//   - Web Serial は Chrome/Edge のみ。requestPort() はクリック(ユーザー操作)が必須。
//   - ボーレートは 115200 (platformio.ini の monitor_speed と一致)。
//   - シーケンサは音より僅かに先読み(≈80-130ms)して nextStep を進めるため、
//     サーボの機械遅延をちょうど相殺して聴感と合いやすい。ズレる場合は LEAD_STEPS を調整。

(() => {
  'use strict';

  // ---- 設定 -------------------------------------------------------------
  const BAUD = 115200;
  const MOTION = 'lr';       // 'lr'=左右スイング / 'ud'=縦うなずき
  const STEPS_PER_BEAT = 4;  // 1拍 = 16分音符4つ
  const STEPS_PER_BAR = 16;  // 4/4想定: 1小節 = 16ステップ (accent判定用)
  const LEAD_STEPS = 0;      // 早める(＋)/遅らせる(−) 微調整。整数ステップ
  // ----------------------------------------------------------------------

  const isCtrl = (o) =>
    o && o.state && typeof o.state.bpm === 'number' &&
    ('nextStep' in o || typeof o.tick === 'function');

  // React fiber を辿ってコントローラ(.logic 等)を探す
  function findPlayer() {
    const seen = new Set();
    let found = null;
    function walk(f) {
      let node = f, g = 0;
      while (node && g++ < 200000 && !found) {
        const sn = node.stateNode;
        if (sn && !seen.has(sn)) {
          seen.add(sn);
          if (isCtrl(sn)) found = sn;
          else if (sn.logic && isCtrl(sn.logic)) found = sn.logic;
        }
        let ms = node.memoizedState, h = 0;
        while (ms && typeof ms === 'object' && h++ < 60 && !found) {
          const mv = ms.memoizedState;
          if (isCtrl(mv)) found = mv;
          else if (mv && isCtrl(mv.current)) found = mv.current;
          else if (mv && mv.logic && isCtrl(mv.logic)) found = mv.logic;
          ms = ms.next;
        }
        if (node.child) walk(node.child);
        node = node.sibling;
      }
    }
    for (const el of document.querySelectorAll('*')) {
      const k = Object.keys(el).find(
        (k) => k.startsWith('__reactFiber') || k.startsWith('__reactContainer') || k.startsWith('__reactInternalInstance')
      );
      if (!k) continue;
      try { walk(el[k]); } catch (e) {}
      if (found) break;
    }
    return found;
  }

  // ---- Web Serial -------------------------------------------------------
  let port = null, writer = null, writeChain = Promise.resolve();
  const enc = new TextEncoder();

  async function connect() {
    if (!('serial' in navigator)) { setStatus('Web Serial 非対応ブラウザ', '#ff5d8f'); return; }
    try {
      port = await navigator.serial.requestPort();
      await port.open({ baudRate: BAUD });
      writer = port.writable.getWriter();
      drainReads();               // 受信(ログ)を捨ててバッファ詰まりを防ぐ
      port.addEventListener?.('disconnect', onDisconnect);
      setStatus('接続済み', '#7CFFB2');
      btnConnect.textContent = '🔌 切断';
    } catch (e) {
      setStatus('接続失敗: ' + (e.message || e), '#ff5d8f');
    }
  }

  async function disconnect() {
    try { if (writer) { await writeChain.catch(() => {}); writer.releaseLock(); } } catch (e) {}
    try { if (port) await port.close(); } catch (e) {}
    writer = null; port = null;
    setStatus('未接続', '#ffd23e');
    btnConnect.textContent = '🔌 接続';
  }

  function onDisconnect() { writer = null; port = null; setStatus('切断された', '#ff5d8f'); btnConnect.textContent = '🔌 接続'; }

  async function drainReads() {
    if (!port || !port.readable) return;
    const reader = port.readable.getReader();
    try { while (true) { const { done } = await reader.read(); if (done) break; } }
    catch (e) {} finally { try { reader.releaseLock(); } catch (e) {} }
  }

  function send(line) {
    if (!writer) return;
    const data = enc.encode(line);
    writeChain = writeChain.then(() => writer && writer.write(data)).catch(() => {});
  }

  // ---- ビートループ -----------------------------------------------------
  let player = null;
  let wasPlaying = false;
  let lastQ = -1;
  let nextSearchMs = 0;

  function tick() {
    if ((!player || !player.state) && performance.now() >= nextSearchMs) {
      player = findPlayer();
      nextSearchMs = performance.now() + 500;   // 未検出時の全DOM走査は0.5秒間隔に間引く
    }
    const p = player;
    if (p && p.state) {
      const playing = !!p.state.playing;
      const bpm = Math.round(p.state.bpm) || 120;
      const steps = (p.state.steps | 0) || 256;
      let step = ((p.nextStep | 0) + LEAD_STEPS) % steps;
      if (step < 0) step += steps;

      if (playing && !wasPlaying) lastQ = -1;                 // 再生開始: 最初の拍を即発火
      if (!playing && wasPlaying) { send('C\n'); lastQ = -1; } // 停止: 正面へ
      wasPlaying = playing;

      if (playing && writer) {
        const q = Math.floor(step / STEPS_PER_BEAT);          // 拍インデックス
        if (q !== lastQ) {
          lastQ = q;
          const accent = (step % STEPS_PER_BAR === 0) ? 1 : 0;
          send(`B ${q} ${accent} ${MOTION} ${bpm}\n`);
          flashBeat(accent);
        }
      }
      updateHud(playing, bpm, step, steps);
    } else {
      updateHud(null);
    }
    requestAnimationFrame(tick);
  }

  // ---- 画面パネル(HUD) --------------------------------------------------
  let elStatus, elInfo, elDot, btnConnect;

  function buildHud() {
    const box = document.createElement('div');
    box.style.cssText =
      'position:fixed;right:14px;bottom:14px;z-index:2147483647;' +
      'font:12px/1.5 "DotGothic16",monospace;color:#e8eaf6;background:#12141f;' +
      'border:2px solid #3a3f5a;border-radius:10px;padding:10px 12px;min-width:180px;' +
      'box-shadow:0 6px 20px rgba(0,0,0,.4)';
    box.innerHTML =
      '<div style="font-weight:bold;margin-bottom:6px">🤖 Stack-chan Dance</div>';

    btnConnect = document.createElement('button');
    btnConnect.textContent = '🔌 接続';
    btnConnect.style.cssText =
      'cursor:pointer;border:0;border-radius:6px;padding:6px 10px;margin-bottom:8px;' +
      'font:12px "DotGothic16",monospace;background:#ffd23e;color:#12141f;width:100%';
    btnConnect.onclick = () => (port ? disconnect() : connect());
    box.appendChild(btnConnect);

    const row = document.createElement('div');
    row.style.cssText = 'display:flex;align-items:center;gap:6px;margin-bottom:4px';
    elDot = document.createElement('span');
    elDot.style.cssText = 'width:10px;height:10px;border-radius:50%;background:#555;display:inline-block';
    elStatus = document.createElement('span');
    elStatus.textContent = '未接続';
    row.appendChild(elDot); row.appendChild(elStatus);
    box.appendChild(row);

    elInfo = document.createElement('div');
    elInfo.style.cssText = 'opacity:.8;font-size:11px';
    elInfo.textContent = 'コントローラ検索中…';
    box.appendChild(elInfo);

    document.body.appendChild(box);
  }

  function setStatus(text, color) {
    if (elStatus) elStatus.textContent = text;
    if (elDot && color) elDot.style.background = color;
  }

  function updateHud(playing, bpm, step, steps) {
    if (!elInfo) return;
    if (playing === null) { elInfo.textContent = 'コントローラ未検出(サイト読込待ち)'; return; }
    elInfo.textContent =
      (playing ? '▶ 再生中' : '⏸ 停止') + `  BPM ${bpm}\n` + `step ${step}/${steps}`;
    elInfo.style.whiteSpace = 'pre';
  }

  let beatFlashUntil = 0;
  function flashBeat(accent) {
    beatFlashUntil = performance.now() + 120;
    if (elDot && writer) {
      elDot.style.background = accent ? '#ff5d8f' : '#7CFFB2';
      setTimeout(() => { if (writer && performance.now() >= beatFlashUntil) elDot.style.background = '#7CFFB2'; }, 120);
    }
  }

  // ---- 起動 -------------------------------------------------------------
  function boot() {
    buildHud();
    requestAnimationFrame(tick);
  }
  if (document.body) boot();
  else window.addEventListener('DOMContentLoaded', boot);
})();
