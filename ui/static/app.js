"use strict";
/* X3Plus 操作台 — 前端
 *
 * 唯一的狀態來源是伺服器推來的 SSE。這支程式不模擬、不推測、不快取猜測值：
 * 收到什麼畫什麼，沒收到就明說沒收到。機器人的畫面上寫著現況，就必須真的是現況。
 */

const el = id => document.getElementById(id);
/* 任何進到 innerHTML 的字串都要先過這裡。檢查項目的細節是機器輸出的原文，
 * 裡面本來就會有 < 和 &，不跳脫的話畫面會被自己的診斷訊息弄壞。 */
const esc = s => String(s).replace(/[&<>"]/g, c =>
  ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const api = (path, body) => fetch(path, {
  method: body === undefined ? "GET" : "POST",
  headers: body === undefined ? {} : { "Content-Type": "application/json" },
  body: body === undefined ? undefined : JSON.stringify(body),
}).then(r => r.json());

/* ── 主題 ───────────────────────────────────────────── */
const root = document.documentElement, themeBtn = el("themeBtn");
const sysDark = () => matchMedia("(prefers-color-scheme:dark)").matches;
const isDark = () => root.dataset.theme ? root.dataset.theme === "dark" : sysDark();
const paintTheme = () => { themeBtn.textContent = isDark() ? "淺色" : "深色"; };
themeBtn.addEventListener("click", () => {
  root.dataset.theme = isDark() ? "light" : "dark";
  try { localStorage.setItem("x3plus.theme", root.dataset.theme); } catch (e) {}
  paintTheme();
});
try {
  const saved = localStorage.getItem("x3plus.theme");
  if (saved) root.dataset.theme = saved;
} catch (e) {}
paintTheme();

/* ── 裝飾用 QR 圖樣 ─────────────────────────────────── */
{
  const cells = [];
  const finder = (ox, oy) => {
    for (let y = 0; y < 7; y++) for (let x = 0; x < 7; x++) {
      const edge = x === 0 || x === 6 || y === 0 || y === 6;
      const core = x >= 2 && x <= 4 && y >= 2 && y <= 4;
      if (edge || core) cells.push([ox + x, oy + y]);
    }
  };
  finder(0, 0); finder(22, 0); finder(0, 22);
  let seed = 20260806;
  const rnd = () => (seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff;
  for (let y = 0; y < 29; y++) for (let x = 0; x < 29; x++) {
    const inFinder = (x < 9 && y < 9) || (x > 19 && y < 9) || (x < 9 && y > 19);
    if (!inFinder && rnd() > 0.55) cells.push([x, y]);
  }
  el("qrCells").innerHTML = cells
    .map(([x, y]) => `<rect x="${x}" y="${y}" width="1" height="1"/>`).join("");
}

/* ── 連線頁 ─────────────────────────────────────────── */
const IP_RE = /^(\d{1,3}\.){3}\d{1,3}$/;
const validHost = v => v === "localhost" || v.endsWith(".local") ||
  (IP_RE.test(v) && v.split(".").every(n => +n >= 0 && +n <= 255));
const phase = p => {
  el("cForm").hidden = p !== "form";
  el("cBusy").hidden = p !== "busy";
  el("cErr").hidden = p !== "err";
};

el("ipIn").value = location.hostname || "127.0.0.1";
el("portIn").value = location.port || "8080";

el("cBtn").addEventListener("click", async () => {
  const host = el("ipIn").value.trim(), port = el("portIn").value || "8080";
  if (!validHost(host)) {
    el("ipIn").setAttribute("aria-invalid", "true");
    el("ipHint").innerHTML = `<span style="color:var(--crit)">請輸入四段數字的 IP，例如 192.168.1.42。</span>`;
    return;
  }
  el("ipIn").removeAttribute("aria-invalid");

  // 指到別台機器人就整頁跳過去 —— 那台的操作台由那台自己提供。
  if (host !== location.hostname || port !== (location.port || "8080")) {
    location.href = `http://${host}:${port}/`;
    return;
  }

  phase("busy");
  el("busyIp").textContent = `${host}:${port}`;
  el("cTitle").textContent = "連線中";
  el("cDot").style.background = "var(--warn)";
  try {
    const h = await api("/healthz");
    if (!h.ok) throw new Error("伺服器回報異常");
    enterApp(`${host}:${port}`);
  } catch (err) {
    phase("err");
    el("cTitle").textContent = "連不上";
    el("cDot").style.background = "var(--crit)";
    el("errMsg").innerHTML =
      `<span>✕</span><div><b>${host}:${port}</b> 沒有回應。<br>
       請確認機器人上的操作台服務仍在執行，且手機和機器人連到同一個網路。</div>`;
  }
});
el("ipIn").addEventListener("keydown", e => { if (e.key === "Enter") el("cBtn").click(); });
el("retryBtn").addEventListener("click", () => {
  phase("form"); el("cTitle").textContent = "連線到機器人";
  el("cDot").style.background = "var(--ink-3)";
});

// 桌面啟動器產生的 QR code 帶 connect=1。手機掃描後直接進入操作台，
// 不需要再看一次 IP 或按「連線」；手動開啟網址時仍保留連線頁作為備援。
if (new URLSearchParams(location.search).get("connect") === "1") {
  history.replaceState(null, "", location.pathname);
  el("cBtn").click();
}

function enterApp(label) {
  el("v-landing").classList.remove("on");
  el("v-app").classList.add("on");
  el("connPill").hidden = el("dcBtn").hidden = el("estopBtn").hidden = false;
  el("railWrap").hidden = el("hazBar").hidden = false;
  el("connIp").textContent = label;
  window.scrollTo({ top: 0 });
  connect();
}
el("dcBtn").addEventListener("click", () => {
  el("v-app").classList.remove("on"); el("v-landing").classList.add("on");
  el("connPill").hidden = el("dcBtn").hidden = el("estopBtn").hidden = true;
  el("railWrap").hidden = el("hazBar").hidden = true;
  phase("form"); el("cTitle").textContent = "連線到機器人";
  el("cDot").style.background = "var(--ink-3)";
  window.scrollTo({ top: 0 });
});

/* ── 分頁 ───────────────────────────────────────────── */
const showScreen = id => document.querySelectorAll(".screen")
  .forEach(s => { s.style.display = s.id === id ? "block" : "none"; });
document.querySelectorAll("[data-go]").forEach(b => b.addEventListener("click", () => {
  document.querySelectorAll("[data-go]").forEach(x =>
    x.setAttribute("aria-selected", String(x === b)));
  showScreen("s-" + b.dataset.go);
  window.scrollTo({ top: 0, behavior: "smooth" });
}));
showScreen("s-console");

/* ── 狀態機緞帶 ─────────────────────────────────────── */
const CHAIN = ["BOOT", "SELF_CHECK", "IDLE", "PATROL", "INVESTIGATE", "APPROACH",
  "ALIGN", "STATIONARY_GATE", "LATCH", "GRASP", "VERIFY", "CARRY_HOME",
  "DELIVER", "PLACE_ALIGN", "PLACE", "RESUME"];
el("ribbon").innerHTML = CHAIN.map(s => `<i>${s}</i>`).join("");

/* 機器狀態 → 給人看的一句話。緞帶外的狀態（RETRY/PAUSED/FAULT/ESTOP）也要有。 */
const HUMAN = {
  BOOT: "系統啟動中。",
  SELF_CHECK: "正在自我檢查，確認感測器與定位都可信。",
  IDLE: "檢查通過，待命中。",
  PATROL: "沿路線巡航中。",
  INVESTIGATE: "看到疑似目標，轉向確認。",
  APPROACH: "朝目標前進，一路避開障礙物。",
  ALIGN: "改用手臂相機做最後對位。",
  STATIONARY_GATE: "等底盤完全停穩，車還在晃就不能動手臂。",
  LATCH: "鎖定物體座標。手臂一動相機就會跟著動，所以現在記住位置，整輪不再更新。",
  GRASP: "手臂正在夾取。",
  VERIFY: "確認物體是不是真的離地。",
  RETRY: "上一次沒夾到，後退重新對位。",
  CARRY_HOME: "手臂收回原位，準備移動。",
  DELIVER: "帶著物體前往垃圾桶。",
  PLACE_ALIGN: "到垃圾桶前，對位並停穩。",
  PLACE: "伸出手臂、鬆開夾爪、收回。",
  RESUME: "回到路線繼續巡航。",
  COMPLETE: "任務完成。",
  PAUSED: "已暫停。排除原因後按「確認開始」重新檢查並繼續。",
  FAULT: "發生無法自行恢復的錯誤，已停止。需要人工處理後重新啟動。",
  ESTOP: "已停止。這是軟停止 —— 真正的急停請按電源開關。",
};
const GRASP_LABEL = ["對位中", "閉合中", "夾持中"];
const fmt = (v, unit, digits = 2) =>
  (v === null || v === undefined) ? "—" : `${Number(v).toFixed(digits)}${unit || ""}`;
const yn = v => v === undefined ? "—" : (v ? "是" : "否");

/* ── 即時狀態 ───────────────────────────────────────── */
let source = null, lastSeq = null, latched = false, allowReal = false, simulate = false;
/* 任務在不在跑，用伺服器送來的布林值記著。之前是去讀 procTok 這個元素上的
 * 中文字有沒有「執行中」—— 改一句文案，按鈕就會在任務執行中悄悄變成可按。 */
let missionRunning = false;
const changes = [];       // 狀態變化紀錄
const trail = [];         // 地圖軌跡

function banner(text, kind) {
  const b = el("banner");
  if (!text) { b.hidden = true; return; }
  b.hidden = false;
  b.className = "banner " + (kind || "");
  b.textContent = text;
}

function connect() {
  if (source) source.close();
  source = new EventSource("/api/events");
  source.addEventListener("snapshot", e => applySnapshot(JSON.parse(e.data)));
  source.addEventListener("status", e => applyStatus(JSON.parse(e.data)));
  source.addEventListener("process", e => applyProcess(JSON.parse(e.data)));
  source.addEventListener("estop", e => applyEstop(JSON.parse(e.data).latched));
  source.addEventListener("system", e => applyMemory(JSON.parse(e.data).memory));
  source.onopen = () => { banner(null); el("connDot").style.background = "var(--ok)"; };
  source.onerror = () => {
    // EventSource 會自己重連，所以這是「暫時斷線」而不是「壞掉了」。
    el("connDot").style.background = "var(--crit)";
    banner("與機器人的連線中斷，正在自動重試。畫面上的數值可能已經不是現況。", "crit");
  };
}

function applySnapshot(s) {
  allowReal = !!s.allow_real;
  simulate = !!s.simulate;
  el("allowRealNote").innerHTML = allowReal
    ? `<span>🔒</span><div><code class="mono">--allow-real</code> 只開啟第一道閘；目前 A/B/C 都缺少操作台可提交的模式專用校正證據，因此實機請求仍會被拒絕。</div>`
    : `<span>🔒</span><div>這台伺服器<b>不允許驅動硬體</b>。要實際動作，需在機器人上以 <code class="mono">--allow-real</code> 重新啟動操作台。</div>`;
  const foot = simulate
    ? "<b>模擬模式</b> — 狀態由伺服器的模擬器產生，沒有連接實體機器人。"
    : "狀態由任務程序即時推送。這個網頁不直接操作硬體。";
  el("appFoot").innerHTML = foot;
  el("landFoot").innerHTML = foot;
  if (simulate) banner("模擬模式：畫面上的狀態是伺服器產生的腳本，不是實體機器人。", "warn");
  if (s.status && s.status.state) applyStatus(s.status);
  applyProcess(s.process || {});
  applyMemory(s.memory);
  if (!routeData) loadRoute("");
  buildCmd();
}

/* 記憶體餘量。Jetson Nano 跑這個專案本來就吃到九成，所以這件事必須看得見，
 * 而不是等 OOM killer 選一個程序殺掉才發現。非 Linux 上伺服器不會給值，
 * 這時候就不顯示 —— 編一個數字比不顯示更糟。 */
function applyMemory(m) {
  if (!m || m.used_pct == null) { el("memPill").hidden = true; return; }
  el("memPill").hidden = false;
  const own = m.console_mb != null ? `（操作台 ${m.console_mb} MB）` : "";
  el("memTxt").textContent = `記憶體 ${m.used_pct}%`;
  el("memPill").title = `已用 ${m.used_pct}%，可用 ${m.available_mb} / ${m.total_mb} MB${own}`;
  el("memTxt").style.color = m.used_pct >= 95 ? "var(--crit)"
    : (m.used_pct >= 88 ? "var(--warn)" : "");
}

function applyStatus(f) {
  if (!f || !f.state) return;
  const st = f.state;
  el("stName").textContent = st;
  el("stName").style.color = (st === "FAULT" || st === "ESTOP") ? "var(--crit)" : "var(--accent)";
  el("stAction").textContent = f.action || "—";
  el("stAction").className = "chip" + ((st === "FAULT" || st === "ESTOP") ? " crit" : "");
  el("stHuman").textContent = HUMAN[st] || "（這個狀態還沒有對應的說明）";
  el("stReason").textContent = f.reason || "—";
  el("seqTok").textContent = f.seq ? `#${f.seq}` : "";

  if (f.waiting === "start") {
    el("stHuman").textContent = "已通過自我檢查，等待你確認才會開始移動。請先清空周圍。";
  } else if (f.waiting === "pause") {
    el("stHuman").textContent = "已暫停並停住底盤。排除原因後按「確認開始」重新檢查並繼續。";
  }
  el("confirmBtn").hidden = !f.waiting;

  const at = CHAIN.indexOf(st);
  el("ribbon").querySelectorAll("i").forEach((n, k) => {
    n.className = at < 0 ? "" : (k < at ? "done" : (k === at ? "now" : ""));
  });
  const now = el("ribbon").querySelector("i.now");
  if (now) now.scrollIntoView({ block: "nearest", inline: "center", behavior: "smooth" });

  const t = f.target || {}, g = f.grasp || {}, b = f.base || {}, goal = f.goal || {};
  el("tDist").innerHTML = t.dist == null ? "—" : `${t.dist.toFixed(2)}<small>m</small>`;
  el("tWp").textContent = goal.id || "—";
  el("tPick").textContent = f.lifted || 0;
  el("tLap").textContent = f.laps || 0;

  el("detChip").textContent = t.visible ? `已偵測 ${t.streak || 0}/3` : "未偵測";
  el("detChip").className = t.visible ? "chip ok" : "chip mute";
  el("vBase").textContent = b.stationary ? "靜止" : (f.chassis_allowed ? "可移動" : "停止中");
  el("vArm").textContent = b.arm_at_home ? "在原位" : (f.arm_allowed ? "作動中" : "—");
  el("vLatch").textContent = yn(g.latched);
  el("vVerify").textContent = yn(g.verified);
  el("vGoal").textContent = fmt(goal.dist, " m");
  el("vBlock").textContent = f.blocking || "無";

  // 夾取階段由回報的旗標推斷，不是靠猜時間。
  let stage = -1;
  if (st === "GRASP") stage = g.verified ? 2 : (g.finished ? 2 : (g.handoff_ready ? 1 : 0));
  else if (["VERIFY", "CARRY_HOME", "DELIVER", "PLACE_ALIGN", "PLACE"].includes(st)) stage = 2;
  el("stages").querySelectorAll(".stage").forEach((n, k) => {
    n.dataset.s = stage < 0 ? "idle" : (k < stage ? "done" : (k === stage ? "now" : "idle"));
  });
  el("graspTok").textContent = stage < 0 ? "未開始" : GRASP_LABEL[stage];

  el("mapTok").textContent = st;
  if (f.pose && f.pose.x != null && f.pose.y != null) drawPose(f.pose);

  if (f.changed || lastSeq === null) {
    changes.push({ t: f.t, st, reason: f.reason || "", human: HUMAN[st] || "" });
    if (changes.length > 300) changes.shift();
    renderChanges();
  }
  lastSeq = f.seq;
  markFresh(0);
}

/* ── 地圖 ─────────────────────────────────────────────
 * 座標是地圖框的公尺，不是畫素。視野由路線的範圍決定，路線沒載入時就用機器人
 * 自己走出來的範圍 —— 但兩者一定用同一個轉換，否則路線和位置會對不起來，
 * 而一張對不起來的地圖比沒有地圖更容易讓人做錯決定。 */
const VIEW = 100, PAD = 8;
let mapFit = null;              // {minx, miny, scale} 或 null
function fitFrom(points) {
  if (!points.length) return null;
  const xs = points.map(p => p[0]), ys = points.map(p => p[1]);
  const minx = Math.min(...xs), maxx = Math.max(...xs);
  const miny = Math.min(...ys), maxy = Math.max(...ys);
  const span = Math.max(maxx - minx, maxy - miny, 1);
  const scale = (VIEW - PAD * 2) / span;
  return { minx, miny, maxx, maxy, span, scale,
           cx: (minx + maxx) / 2, cy: (miny + maxy) / 2 };
}
/* 地圖框的 y 向上，SVG 的 y 向下，所以 y 要翻。 */
const toView = (x, y) => {
  if (!mapFit) return [VIEW / 2, VIEW / 2];
  return [VIEW / 2 + (x - mapFit.cx) * mapFit.scale,
          VIEW / 2 - (y - mapFit.cy) * mapFit.scale];
};

let routeData = null;
async function loadRoute(path) {
  const r = await api("/api/route", { route: path || "" }).catch(() => null);
  const tok = el("routeTok");
  if (!r || !r.ok) {
    routeData = null;
    tok.textContent = "路線未載入";
    el("mapNote").textContent = (r && r.error) || "路線讀取失敗。";
    return;
  }
  routeData = r;
  const pts = r.waypoints.map(w => [w.x, w.y]);
  if (r.bin) pts.push(r.bin);
  mapFit = fitFrom(pts);
  trail.length = 0;
  el("trail").setAttribute("points", "");

  el("routeLine").setAttribute("points",
    r.waypoints.map(w => toView(w.x, w.y).map(v => v.toFixed(2)).join(",")).join(" "));
  el("routeDots").innerHTML = r.waypoints.map(w => {
    const [vx, vy] = toView(w.x, w.y);
    return `<circle cx="${vx.toFixed(2)}" cy="${vy.toFixed(2)}" r=".55" fill="var(--accent)" opacity=".5"><title>${esc(w.id)}</title></circle>`;
  }).join("");
  if (r.bin) {
    const [bx, by] = toView(r.bin[0], r.bin[1]);
    el("binMark").setAttribute("transform", `translate(${bx.toFixed(2)},${by.toFixed(2)})`);
    el("binMark").setAttribute("opacity", "1");
  } else {
    el("binMark").setAttribute("opacity", "0");
  }
  // 一公尺在畫面上多長，決定格線間距，這樣比例尺是看得出來的
  const step = mapFit.span > 40 ? 10 : (mapFit.span > 12 ? 5 : 1);
  let g = "";
  for (let v = Math.ceil(mapFit.minx / step) * step; v <= mapFit.maxx; v += step) {
    const [gx] = toView(v, 0); g += `<path d="M${gx.toFixed(2)} 0V${VIEW}"/>`;
  }
  for (let v = Math.ceil(mapFit.miny / step) * step; v <= mapFit.maxy; v += step) {
    const [, gy] = toView(0, v); g += `<path d="M0 ${gy.toFixed(2)}H${VIEW}"/>`;
  }
  el("mapGrid").innerHTML = g;
  // 主控台那張小圖共用同一份轉換，兩張圖不會各畫各的
  el("miniRoute").setAttribute("points", el("routeLine").getAttribute("points"));
  el("miniMapTok").textContent = `${r.waypoints.length} 點`;
  tok.textContent = `${r.waypoints.length} 個航點`;
  // 首頁那格也用同一個數字。route.yaml 原始有 117 點，但那個間距根本載不起來
  // （會撞上到達半徑的檢查），實際跑的是重取樣後的數量 —— 兩個畫面必須一致。
  el("wpCount").textContent = r.waypoints.length;
  el("scaleTok").textContent = `格線 ${step} m`;
  el("mapNote").textContent = `路線來源：${r.source}`;
}
el("reloadRoute").addEventListener("click", () => loadRoute(el("fRoute").value.trim()));

function drawPose(pose) {
  // 路線沒載入時，用機器人自己走過的範圍撐出視野，仍然是真實座標。
  if (!mapFit) {
    mapFit = fitFrom([[pose.x - 2, pose.y - 2], [pose.x + 2, pose.y + 2]]);
    el("scaleTok").textContent = "尚無路線，比例尺依實際位置推算";
  }
  const [x, y] = toView(pose.x, pose.y);
  // 地圖框的 yaw 逆時針為正，SVG 的旋轉順時針為正，而箭頭朝上＝0°。
  const deg = pose.yaw == null ? 0 : (90 - pose.yaw * 180 / Math.PI);
  const at = `translate(${x.toFixed(2)},${y.toFixed(2)}) rotate(${deg.toFixed(1)})`;
  el("robot").setAttribute("transform", at);
  el("robot").setAttribute("opacity", "1");
  el("miniRobot").setAttribute("transform", at);
  el("miniRobot").setAttribute("opacity", "1");
  el("miniPose").textContent = `x ${pose.x.toFixed(1)}  y ${pose.y.toFixed(1)} m`;
  const last = trail[trail.length - 1];
  const here = `${x.toFixed(2)},${y.toFixed(2)}`;
  if (here !== last) {
    trail.push(here);
    if (trail.length > 600) trail.shift();
    el("trail").setAttribute("points", trail.join(" "));
  }
  el("poseTok").textContent = `x ${pose.x.toFixed(2)}  y ${pose.y.toFixed(2)} m`;
}

/* 資料新鮮度：夾取是阻塞呼叫，那段期間任務不會回報。畫面必須說出這件事，
 * 而不是讓一個凍住的數字看起來像現況。 */
let lastFrameAt = 0;
function markFresh() { lastFrameAt = Date.now(); }
setInterval(() => {
  const chip = el("ageChip");
  if (!lastFrameAt) { chip.hidden = true; return; }
  const age = (Date.now() - lastFrameAt) / 1000;
  if (age < 2) { chip.hidden = true; return; }
  chip.hidden = false;
  chip.textContent = `已 ${age.toFixed(0)} 秒沒有更新`;
  chip.className = age > 20 ? "chip crit" : "chip warn";
}, 500);

function renderChanges() {
  const row = e => {
    const ts = e.t ? new Date(e.t * 1000).toLocaleTimeString("zh-TW", { hour12: false }) : "—";
    return `<div class="row"><span class="ts">${ts}</span><span class="st">${e.st}</span>` +
           `<span class="rs">${e.human || ""}<span style="color:var(--ink-3)"> ${e.reason}</span></span></div>`;
  };
  const rows = changes.map(row);
  const empty = `<div class="row"><span class="ts">—</span><span class="st">—</span><span class="rs" style="color:var(--ink-3)">還沒有狀態變化。</span></div>`;
  el("miniLog").innerHTML = rows.slice(-6).reverse().join("") || empty;
}
renderChanges();

function applyProcess(p) {
  const running = missionRunning = !!p.running;
  el("stopBtn").hidden = !running;
  el("procTok").textContent = running
    ? `任務執行中 · pid ${p.pid} · ${Math.round(p.uptime || 0)} 秒`
    : (p.exit_code == null ? "任務未啟動" : `任務已結束（代碼 ${p.exit_code}）`);
  applyEstop(!!p.estop_latched);
  el("startBtn").disabled = running || latched;
  if (running) el("startBtn").textContent = "任務執行中";
  else buildCmd();
}

function applyEstop(on) {
  latched = on;
  const b = el("estopBtn");
  b.dataset.latched = on ? "1" : "0";
  b.textContent = on ? "已停止 · 點此解除" : "■ 停止";
  el("startBtn").disabled = on || missionRunning;
}

/* ── 控制 ───────────────────────────────────────────── */
el("estopBtn").addEventListener("click", async () => {
  if (latched) {
    if (!confirm("解除停止鎖定？解除後才能開始新的任務。")) return;
    await api("/api/estop/clear", {});
    return;
  }
  const r = await api("/api/estop", {});
  if (r.note) banner(r.note, "warn");
});
el("stopBtn").addEventListener("click", async () => {
  const r = await api("/api/mission/stop", {});
  if (r.note) banner(r.note, "warn");
  else if (r.error) banner(r.error, "crit");
});
el("confirmBtn").addEventListener("click", async () => {
  const r = await api("/api/mission/confirm", {});
  if (r.error) banner(r.error, "crit"); else banner(null);
});

/* ── 模式 A 設定與伺服器端驗證 ─────────────────────── */
["fRoute", "fClass", "fH", "fLaps", "fDeliver", "fReal", "unlockChk"]
  .forEach(id => el(id).addEventListener("input", buildCmd));

const config = () => ({
  mode: "A",
  route: el("fRoute").value.trim(),
  cls: el("fClass").value.trim() || "sugarbox",
  height_cm: parseFloat(el("fH").value),
  laps: parseInt(el("fLaps").value, 10) || 0,
  deliver: el("fDeliver").checked,
  real: el("fReal").checked,
  unlock: el("unlockChk").checked,
});

/* 預覽指令是非同步的，而使用者會連續改好幾個欄位。去抖只擋掉多餘的「發送」，
 * 擋不掉已經在路上的回應：舊的請求晚回來就會蓋掉新的答案，畫面可能寫著
 * 「不驅動硬體，可以直接執行」，而開關其實是開的。所以每個請求帶一個序號，
 * 不是最新的一律丟掉。 */
let cmdTimer = null, cmdSeq = 0;
function buildCmd() {
  clearTimeout(cmdTimer);
  cmdTimer = setTimeout(async () => {
    const mine = ++cmdSeq;
    const r = await api("/api/mission/preview", config())
      .catch(() => ({ ok: false, error: "伺服器沒有回應。" }));
    if (mine !== cmdSeq) return;              // 已經有更新的答案了
    const note = el("gateNote"), start = el("startBtn");
    if (!r.ok) {
      note.className = "note warn";
      note.innerHTML = `<span>🔒</span><div>${r.error}</div>`;
      start.disabled = true;
      return;
    }
    const real = el("fReal").checked;
    note.className = real ? "note warn" : "note";
    note.innerHTML = real
      ? `<span>⚠</span><div>將驅動實體手臂。請確認<b>有人在場、手放在電源開關上</b>。</div>`
      : `<span>✓</span><div>不驅動硬體，可以直接執行。</div>`;
    start.disabled = latched || missionRunning;
    start.textContent = real ? "開始任務（驅動實體硬體）" : "開始任務（不驅動硬體）";
  }, 150);
}

el("startBtn").addEventListener("click", async () => {
  if (el("fReal").checked &&
      !confirm("即將驅動實體機器人。\n\n確認周圍淨空、有人在場、手放在電源開關上？")) return;
  const r = await api("/api/mission/start", config());
  if (!r.ok) { banner(r.error, "crit"); return; }
  banner(null);
  document.querySelector('[data-go="console"]').click();
});

