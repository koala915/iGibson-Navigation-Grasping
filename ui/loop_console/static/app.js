// 決賽操作台：一份狀態（SSE 推送），畫地圖、狀態、自檢與紀錄；按鍵只送 POST。
'use strict';

const $ = (id) => document.getElementById(id);
let S = null;            // the newest state from the server
let pending = null;      // what the confirm dialog will start

// ── theme ──────────────────────────────────────────────────────────────
function setTheme(t) {
  document.documentElement.dataset.theme = t;
  $('themeBtn').textContent = t === 'dark' ? '淺色' : '深色';
  try { localStorage.setItem('loopTheme', t); } catch (e) { /* private window */ }
  drawMap();
}
$('themeBtn').onclick = () =>
  setTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark');
try {
  const saved = localStorage.getItem('loopTheme');
  if (saved) setTheme(saved);
  else if (matchMedia('(prefers-color-scheme: dark)').matches) setTheme('dark');
} catch (e) { /* defaults */ }

// ── server ─────────────────────────────────────────────────────────────
async function post(path, body) {
  const r = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'},
                               body: JSON.stringify(body || {})});
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.error || ('HTTP ' + r.status));
  return data;
}

function banner(text) {
  const b = $('banner');
  b.textContent = text || '';
  b.hidden = !text;
}

function connect() {
  const es = new EventSource('/api/events');
  es.onmessage = (ev) => { banner(''); render(JSON.parse(ev.data)); };
  es.onerror = () => {
    banner('和操作台伺服器的連線中斷，重新連線中…');
    es.close();
    setTimeout(connect, 2000);
  };
}

// ── render ─────────────────────────────────────────────────────────────
const STAGE_CHIP = {idle: 'mute', localized: 'ok', done: 'ok', released: 'ok', grasped: 'ok',
                    stopped: 'warn', error: 'crit'};

function render(s) {
  S = s;
  const running = !!s.job;
  const pose = s.pose || {};
  // connection pill
  const live = s.link && pose.age != null;
  $('connDot').classList.toggle('live', !!live);
  $('connDot').style.background = live ? (pose.ready ? 'var(--ok)' : 'var(--warn)') : 'var(--crit)';
  $('connTxt').textContent = s.host + (live ? (pose.ready ? ' · 定位就緒' : ' · 未定位')
                                            : ' · 未連上');
  $('hazBar').hidden = !running;

  // status
  const chip = $('stageChip');
  chip.textContent = s.stage_text + (s.stage === 'patrol' && s.lap ? ' · 第 ' + s.lap + ' 圈' : '');
  chip.className = 'chip ' + (STAGE_CHIP[s.stage] || '');
  $('mDelivered').textContent = s.delivered;
  $('mFailures').textContent = s.failures;
  $('mLap').textContent = s.lap || '—';
  $('stageDetail').textContent = s.detail || '';
  tickClock();

  // actions
  const crit = (s.checks && s.checks.items || []).filter((i) => i.level === 'crit');
  $('relocBtn').disabled = running;
  $('startBtn').disabled = running || crit.length > 0;
  const hint = $('actionHint');
  if (running) {
    hint.textContent = '正在執行：' + ({relocalize: '重新定位', mission: '任務', fix: '修復'}[s.job]
                                   || s.job) + '。要中斷按右上角「停止」。';
    hint.className = 'hint';
  } else if (crit.length) {
    hint.textContent = '自檢有紅燈，先處理：' + crit.map((i) => i.label).join('、');
    hint.className = 'hint why';
  } else if (!pose.ready) {
    hint.textContent = '還沒定位：先按「重新定位」，或開始時勾選「先重新定位」。';
    hint.className = 'hint why';
  } else {
    hint.textContent = '重新定位看不到兩面牆的標記時，車子會原地轉動找。兩個都會先確認人在車旁。';
    hint.className = 'hint';
  }

  renderChecks(s.checks);
  renderLog(s.log || []);
  $('poseTxt').textContent = pose.x != null
    ? 'x ' + pose.x.toFixed(2) + ' · y ' + pose.y.toFixed(2) + ' · '
      + Math.round(pose.yaw * 180 / Math.PI) + '°'
    : '位置 —';
  $('mapTok').textContent = s.map_error ? '地圖讀取失敗' : (s.map ? '2×2 m' : '讀取中');
  drawMap();
}

// A stage with a usual duration (relocalize ~60 s, the robot standing still
// throughout) gets a running clock, so a long quiet step does not read as a hang.
function tickEta() {
  const box = $('stageEta');
  const eta = S && S.job && S.stage_eta_s;
  box.hidden = !eta || !S.stage_since;
  if (box.hidden) return;
  const t = Math.max(0, Math.round(Date.now() / 1000 - S.stage_since));
  const late = t > eta;
  box.classList.toggle('late', late);
  $('stageEtaBar').style.width = Math.min(100, 100 * t / eta) + '%';
  $('stageEtaTxt').textContent = late
    ? '已 ' + t + ' 秒，比平常久，仍在等車上回覆'
    : '已 ' + t + ' 秒 · 通常約 ' + Math.round(eta) + ' 秒';
}

function tickClock() {
  tickEta();
  if (!S || !S.started_at) { $('mTime').textContent = '—'; return; }
  const end = S.job ? Date.now() / 1000 : (S.ended_at || Date.now() / 1000);
  const t = Math.max(0, Math.round(end - S.started_at));
  $('mTime').textContent = Math.floor(t / 60) + ':' + String(t % 60).padStart(2, '0');
}
setInterval(tickClock, 1000);

function renderChecks(checks) {
  const ul = $('checks');
  if (!checks) { ul.innerHTML = '<li class="empty">檢查中…</li>'; return; }
  const ago = Math.round(Date.now() / 1000 - checks.at);
  $('checkTok').textContent = ago < 5 ? '剛剛' : ago + ' 秒前';
  ul.textContent = '';
  for (const it of checks.items) {
    const li = document.createElement('li');
    li.className = it.level;
    li.innerHTML = '<span class="lv"></span><span class="lb"></span><span class="dt"></span>';
    li.querySelector('.lb').textContent = it.label;
    li.querySelector('.dt').textContent = it.detail;
    if (it.fix) {
      const b = document.createElement('button');
      b.className = 'iconbtn small';
      b.textContent = it.fix === 'arm_stow' ? '收回手臂' : '重啟服務';
      b.disabled = !!(S && S.job);
      b.onclick = () => fix(it.fix);
      li.appendChild(b);
    } else {
      li.appendChild(document.createElement('span'));
    }
    ul.appendChild(li);
  }
}

function renderLog(log) {
  const ol = $('log');
  ol.textContent = '';
  for (const e of log.slice().reverse()) {
    const li = document.createElement('li');
    const t = new Date(e.t * 1000);
    li.innerHTML = '<time></time><span></span>';
    li.querySelector('time').textContent = t.toLocaleTimeString('zh-TW', {hour12: false});
    const span = li.querySelector('span');
    span.textContent = e.text;
    span.className = e.kind;
    ol.appendChild(li);
  }
}

// ── map ────────────────────────────────────────────────────────────────
const BOX = {w: 0.18, d: 0.135};          // placement box, ncu's tape measure
const ROBOT = {front: 0.15, back: 0.15, half: 0.12};

function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

function drawMap() {
  const canvas = $('map');
  if (!canvas || !S) return;
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  const g = canvas.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, rect.width, rect.height);
  const m = S.map;
  if (!m) {
    g.fillStyle = css('--ink-3');
    g.font = '13px ' + css('--sans');
    g.fillText(S.map_error ? '地圖讀取失敗：' + S.map_error.slice(0, 60) : '讀取車上的地圖…', 14, 24);
    return;
  }
  // The map frame is turned against the walls (about 27 deg on this field): draw
  // in the walls' own frame, so the room stands square on screen.
  const [w0, w1] = m.walls;
  let theta = Math.atan2(w1[1] - w0[1], w1[0] - w0[0]);
  theta -= Math.round(theta / (Math.PI / 2)) * (Math.PI / 2);
  const cx = m.walls.reduce((a, p) => a + p[0], 0) / m.walls.length;
  const cy = m.walls.reduce((a, p) => a + p[1], 0) / m.walls.length;
  const cos = Math.cos(-theta), sin = Math.sin(-theta);
  const R = (x, y) => [cx + (x - cx) * cos - (y - cy) * sin, cy + (x - cx) * sin + (y - cy) * cos];
  const rw = m.walls.map(([x, y]) => R(x, y));
  const x0 = Math.min(...rw.map((p) => p[0])), x1 = Math.max(...rw.map((p) => p[0]));
  const y0 = Math.min(...rw.map((p) => p[1])), y1 = Math.max(...rw.map((p) => p[1]));
  const pad = 0.12;
  const span = Math.max(x1 - x0, y1 - y0) + 2 * pad;
  const k = Math.min(rect.width, rect.height) / span;
  const ox = (rect.width - (x1 - x0) * k) / 2, oy = (rect.height - (y1 - y0) * k) / 2;
  const V = ([x, y]) => [ox + (x - x0) * k, rect.height - (oy + (y - y0) * k)];
  const P = (x, y) => V(R(x, y));

  // allowed band
  if (m.mask) {
    g.fillStyle = css('--ok-soft');
    const c = m.mask.cell, [mx, my] = m.mask.origin;
    g.beginPath();
    m.mask.rows.forEach((row, r) => {
      for (let i = 0; i < row.length; i++) {
        if (row[i] !== '1') continue;
        const corners = [[i, r], [i + 1, r], [i + 1, r + 1], [i, r + 1]]
          .map(([u, v]) => P(mx + u * c, my + v * c));
        g.moveTo(...corners[0]);
        corners.slice(1).forEach((q) => g.lineTo(...q));
        g.closePath();
      }
    });
    g.fill();
    g.strokeStyle = css('--ok-soft');     // closes the hairline seams between cells
    g.lineWidth = 1;
    g.stroke();
  }
  // half-metre grid, in the walls' frame
  g.strokeStyle = css('--line-soft');
  g.lineWidth = 1;
  for (let x = x0 + 0.5; x < x1 - 0.01; x += 0.5) {
    const [a, b] = V([x, y0]), [, d] = V([x, y1]);
    g.beginPath(); g.moveTo(a, b); g.lineTo(a, d); g.stroke();
  }
  for (let y = y0 + 0.5; y < y1 - 0.01; y += 0.5) {
    const [a, b] = V([x0, y]), [c] = V([x1, y]);
    g.beginPath(); g.moveTo(a, b); g.lineTo(c, b); g.stroke();
  }
  // walls
  g.strokeStyle = css('--ink');
  g.lineWidth = 2.5;
  g.beginPath();
  m.walls.forEach(([x, y], i) => { const [a, b] = P(x, y); i ? g.lineTo(a, b) : g.moveTo(a, b); });
  g.closePath(); g.stroke();
  // markers
  g.font = '600 11px ' + css('--mono');
  for (const mk of m.markers || []) {
    const [a, b] = P(mk.x, mk.y);
    g.fillStyle = css('--ink-2');
    g.fillRect(a - 5, b - 5, 10, 10);
    g.fillStyle = css('--panel');
    g.fillText(mk.id, a - 3, b + 4);
  }
  // placement box
  if (m.box) {
    // Long edge (BOX.w) along the layout's yaw; level when the layout has none.
    const yaw = m.box.yaw != null ? m.box.yaw : theta;
    const ux = Math.cos(yaw), uy = Math.sin(yaw);
    const corners = [[1, 1], [1, -1], [-1, -1], [-1, 1]].map(([i, j]) =>
      P(m.box.x + i * BOX.w / 2 * ux - j * BOX.d / 2 * uy,
        m.box.y + i * BOX.w / 2 * uy + j * BOX.d / 2 * ux));
    const a = Math.min(...corners.map((p) => p[0])), b = Math.min(...corners.map((p) => p[1]));
    g.fillStyle = css('--warn-soft');
    g.strokeStyle = css('--warn');
    g.lineWidth = 1.5;
    g.setLineDash(m.box.verified ? [] : [4, 3]);
    g.beginPath();
    corners.forEach(([px, py], i) => (i ? g.lineTo(px, py) : g.moveTo(px, py)));
    g.closePath();
    g.fill();
    g.stroke();
    g.setLineDash([]);
    g.fillStyle = css('--warn');
    g.font = '12px ' + css('--sans');
    const label = m.box.verified ? '放置盒' : '放置盒（未驗證）';
    // Kept inside the canvas: the box sits near a wall, and a phone is narrow.
    const lx = Math.max(4, Math.min(a, g.canvas.clientWidth - g.measureText(label).width - 4));
    g.fillText(label, lx, b - 5);
  }
  // trail
  const trail = S.trail || [];
  if (trail.length > 1) {
    g.strokeStyle = css('--accent');
    g.globalAlpha = 0.45;
    g.lineWidth = 1.5;
    g.setLineDash([3, 4]);
    g.beginPath();
    trail.forEach(([x, y], i) => { const [a, b] = P(x, y); i ? g.lineTo(a, b) : g.moveTo(a, b); });
    g.stroke();
    g.setLineDash([]);
    g.globalAlpha = 1;
  }
  // sightings
  for (const d of S.sightings || []) {
    const [a, b] = P(d.x, d.y);
    if (d.rough && (d.state === 'candidate' || d.state === 'confirmed')) {
      g.fillStyle = css('--accent-soft');
      g.strokeStyle = css('--accent');
      g.setLineDash([4, 3]);
      g.beginPath(); g.arc(a, b, 0.25 * k, 0, 7); g.fill(); g.stroke();
      g.setLineDash([]);
    }
    g.lineWidth = 2;
    if (d.state === 'candidate') {
      g.strokeStyle = css('--accent');
      g.beginPath(); g.arc(a, b, 7, 0, 7); g.stroke();
    } else if (d.state === 'confirmed') {
      g.fillStyle = css('--accent');
      g.beginPath(); g.arc(a, b, 7, 0, 7); g.fill();
    } else if (d.state === 'failed') {
      g.fillStyle = css('--warn');
      g.beginPath(); g.arc(a, b, 6, 0, 7); g.fill();
    } else if (d.state === 'skipped') {
      g.strokeStyle = css('--crit');
      g.beginPath(); g.moveTo(a - 6, b - 6); g.lineTo(a + 6, b + 6);
      g.moveTo(a + 6, b - 6); g.lineTo(a - 6, b + 6); g.stroke();
    }
  }
  // robot
  const p = S.pose || {};
  if (p.x != null) {
    const [a, b] = P(p.x, p.y);
    g.save();
    g.translate(a, b);
    g.rotate(-(p.yaw - theta));
    g.globalAlpha = p.ready ? 1 : 0.45;
    g.fillStyle = css('--accent-soft');
    g.strokeStyle = css('--accent');
    g.lineWidth = 2;
    g.fillRect(-ROBOT.back * k, -ROBOT.half * k, (ROBOT.front + ROBOT.back) * k, 2 * ROBOT.half * k);
    g.strokeRect(-ROBOT.back * k, -ROBOT.half * k, (ROBOT.front + ROBOT.back) * k, 2 * ROBOT.half * k);
    g.beginPath();
    g.moveTo(ROBOT.front * k + 10, 0);
    g.lineTo(ROBOT.front * k - 2, -7);
    g.lineTo(ROBOT.front * k - 2, 7);
    g.closePath();
    g.fillStyle = css('--accent');
    g.fill();
    g.restore();
  }
}
window.addEventListener('resize', drawMap);

// ── actions ────────────────────────────────────────────────────────────
$('relocBtn').onclick = () => openModal('reloc');
$('estopBtn').onclick = async () => {
  try { await post('/api/stop'); } catch (e) { banner('停止沒送出：' + e.message + '。請用電源開關'); }
};
$('checkBtn').onclick = () => post('/api/check').catch((e) => banner(e.message));
$('mapReload').onclick = () => post('/api/map').catch((e) => banner(e.message));

$('startBtn').onclick = () => openModal('start');

function fix(what) {
  if (what === 'arm_stow') return openModal('arm_stow');
  post('/api/fix', {what}).catch((e) => banner(e.message));
}

function openModal(kind) {
  pending = kind;
  const start = kind === 'start';
  $('modalTitle').textContent = {start: '開始 loop', reloc: '重新定位', arm_stow: '收回手臂'}[kind];
  $('modalText').textContent = {
    start: '車子會開始巡航、夾取與放置，直到連續 2 圈沒看到糖果盒（或達到送達上限）。',
    reloc: '用後相機看牆上的標記定位（約 1 分鐘）。只看到同一面牆的標記時，車子會原地往右轉，'
      + '每次 30°，看到兩面牆的標記就停。四周要留 40 cm；地上的盒子雷達看不到，請留意。',
    arm_stow: '手臂會先張開夾爪移到 E1，再收回行駛姿態，最後重啟夾取服務（約 1 分鐘）。夾爪裡要是空的。',
  }[kind];
  $('startOpts').hidden = !start;
  $('optReloc').checked = start && !(S && S.pose && S.pose.ready);
  $('optBeside').checked = false;
  $('optMax').value = '';
  $('modalErr').hidden = true;
  $('modalOk').textContent = {start: '開始', reloc: '定位', arm_stow: '收回'}[kind];
  $('modal').hidden = false;
  $('optBeside').focus();
}

function closeModal() { $('modal').hidden = true; pending = null; }
$('modalCancel').onclick = closeModal;
$('modal').addEventListener('keydown', (e) => { if (e.key === 'Escape') closeModal(); });
$('optBeside').onchange = () => { $('modalErr').hidden = true; };

$('modalOk').onclick = async () => {
  const err = $('modalErr');
  if (!$('optBeside').checked) {
    err.textContent = '先勾選「我在車旁，手放在電源開關上」';
    err.hidden = false;
    return;
  }
  const max = $('optMax').value.trim();
  if (pending === 'start' && max && !(/^\d+$/.test(max) && Number(max) >= 1)) {
    err.textContent = '送達上限要是 1 以上的整數，或留空';
    err.hidden = false;
    return;
  }
  try {
    if (pending === 'start') {
      await post('/api/start', {beside_robot: true, relocalize: $('optReloc').checked,
                                max_deliveries: max || null});
    } else if (pending === 'reloc') {
      await post('/api/relocalize', {beside_robot: true});
    } else {
      await post('/api/fix', {what: 'arm_stow', beside_robot: true});
    }
    closeModal();
  } catch (e) {
    err.textContent = e.message;
    err.hidden = false;
  }
};

connect();
