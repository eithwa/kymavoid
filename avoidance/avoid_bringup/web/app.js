// 避障控制台: one page for the real robot and the simulation (docs/avoidance-redesign-plan.md, 7).
// Principles: show the real state (everything comes from /avoid/status and live topics, nothing is
// assumed), every command gets an answer (service result or timeout, as a toast).
'use strict';

const qs = new URLSearchParams(location.search);
const HOST = location.hostname || 'localhost';
const BRIDGE_URL = qs.get('bridge') || `ws://${HOST}:9090`;
const MJPEG_URL = qs.get('mjpeg') || `http://${HOST}:8080`;

const STATE_ZH = { IDLE: '待命', MAPPING: '建圖', RUNNING: '行駛中', FINISHED: '完成', SAFE_STOP: '異常停車' };
const AGE_OK = 0.3;      // nav_node's own input timeouts
const SCAN_STALE = 1.0;  // live scan dots disappear after this
const NAV_SILENT = 0.6;   // /avoid/status comes at 10 Hz

// nav_node tuning (avoid_nav/config/tuning_<sensor>.yaml); re-read by nav_node at every start
const TUNABLES = [
  ['速度規劃', [
    ['v_max', '最高速度', 'm/s', 0.05],
    ['a_max', '加速度', 'm/s²', 0.1],
    ['a_brake', '煞車減速度', 'm/s²', 0.1],
    ['react_time', '反應時間', 's', 0.02],
    ['stop_margin', '停車餘量', 'm', 0.01],
    ['v_tight', '零間隙速度', 'm/s', 0.05],
    ['v_per_clearance', '每公尺間隙加速', 'm/s/m', 0.5],
  ]],
  ['路徑追蹤', [
    ['lookahead', '前視距離', 'm', 0.05],
    ['k_heading', '航向增益', '', 0.1],
    ['w_max', '最大角速度', 'rad/s', 0.1],
  ]],
  ['安全', [
    ['safety_margin', '安全層掃掠餘量', 'm', 0.01],
    ['line_margin_base', '紅線內縮', 'm', 0.01],
    ['line_margin_sigma', '紅線內縮 × σy', '', 0.5],
    ['recovery_after', '無路徑→清地圖', 's', 0.5],
    ['no_path_timeout', '無路徑→停車', 's', 0.5],
  ]],
  ['感測', [
    ['max_obstacle_range', '障礙物最遠距離', 'm', 0.1],
    ['red_line_max_range', '紅線修正最遠', 'm', 0.1],
    ['wall_max_range', '矮牆修正最遠', 'm', 0.1],
    ['clear_margin', '清除餘量（最小）', 'm', 0.01],
    ['clear_sigmas', '清除餘量 × 讀值誤差', 'σ', 0.5],
  ]],
];

// nav_node /avoid/debug speed_limit: which limit decided the commanded speed
const LIMITS = {
  v_max: ['最高速度', '#3fb950'],
  accel: ['加速中', '#58a6ff'],
  turn: ['轉向', '#bc8cff'],
  clearance: ['間隙', '#d29922'],
  dfree: ['前方煞車距離', '#ff7b72'],
  safety: ['安全層', '#f85149'],
  corner: ['前方彎道', '#f778ba'],
  horizon: ['感測距離', '#39c5cf'],
  line: ['紅線/牆', '#ffa657'],
  no_path: ['沒有路徑', '#8b949e'],
};

const $ = (id) => document.getElementById(id);
const now = () => performance.now() / 1000;

// ------------------------------------------------------------------ state
const S = {
  ros: null, connected: false, everConnected: false,
  status: null, statusAt: 0, prevState: null,
  field: null, obsCfg: null, footprint: null,
  obstacles: [], obstaclesAt: 0, path: [], scan: null, boundary: null,
  cspace: null, cspaceInfo: null,
  truth: null, layout: [], trail: [], truthTrail: [],
  isSim: false, sensor: null, nodes: null, nodesErr: null, watch: null, debug: null,
  rates: {}, runs: [], lastSeed: null,
  speedHist: [], actualV: null, vmax: 0.8,
};
const topics = {};

// ------------------------------------------------------------------ toasts
function toast(title, msg = '', kind = 'ok') {
  const el = document.createElement('div');
  el.className = `toast ${kind}`;
  el.innerHTML = `<div class="t"></div><div class="m"></div>`;
  el.querySelector('.t').textContent = title;
  el.querySelector('.m').textContent = msg;
  $('toasts').appendChild(el);
  while ($('toasts').children.length > 4) $('toasts').firstChild.remove();
  setTimeout(() => { el.style.opacity = '0'; setTimeout(() => el.remove(), 400); }, kind === 'err' ? 8000 : 2500);
}

// ------------------------------------------------------------------ ROS helpers
function call(name, type, req, timeout = 4.0) {
  return new Promise((resolve, reject) => {
    if (!S.connected) return reject(new Error('rosbridge 未連線'));
    let done = false;
    const t = setTimeout(() => { if (!done) { done = true; reject(new Error(`${name} 沒有回應（${timeout} s）`)); } }, timeout * 1000);
    new ROSLIB.Service({ ros: S.ros, name, serviceType: type }).callService(new ROSLIB.ServiceRequest(req),
      (res) => { if (!done) { done = true; clearTimeout(t); resolve(res); } },
      (err) => { if (!done) { done = true; clearTimeout(t); reject(new Error(String(err))); } });
  });
}
async function getParam(name) {
  const r = await call('/rosapi/get_param', 'rosapi/GetParam', { name, default: '' });
  return r.value === '' ? undefined : JSON.parse(r.value);
}
async function setParam(name, value) {
  await call('/rosapi/set_param', 'rosapi/SetParam', { name, value: JSON.stringify(value) });
}

function subscribe(name, type, cb, opts = {}) {
  unsubscribe(name);
  const t = new ROSLIB.Topic({ ros: S.ros, name, messageType: type, queue_length: 1, ...opts });
  t.subscribe((m) => { countRate(name); cb(m); });
  topics[name] = t;
}
function unsubscribe(name) {
  if (topics[name]) { topics[name].unsubscribe(); delete topics[name]; }
}
function countRate(name) {
  const r = S.rates[name] || (S.rates[name] = []);
  const t = now();
  r.push(t);
  while (r.length && t - r[0] > 2.0) r.shift();
}

// ------------------------------------------------------------------ connection
function connect() {
  $('bridge-url').textContent = BRIDGE_URL;
  const ros = new ROSLIB.Ros({ url: BRIDGE_URL });
  S.ros = ros;
  ros.on('connection', () => {
    S.connected = true;
    if (S.everConnected) toast('rosbridge 已重新連線', BRIDGE_URL, 'info');
    S.everConnected = true;
    onConnected();
  });
  ros.on('error', () => {});
  ros.on('close', () => {
    const was = S.connected;
    S.connected = false;
    for (const k of Object.keys(topics)) delete topics[k];
    if (was) toast('rosbridge 斷線', '每 2 秒重試', 'err');
    setTimeout(connect, 2000);
  });
}

async function onConnected() {
  subscribe('/avoid/status', 'avoid_msgs/Status', onStatus);
  subscribe('/avoid/nodes', 'std_msgs/String', onNodes);
  subscribe('/avoid/path', 'nav_msgs/Path', (m) => { S.path = m.poses.map((p) => [p.pose.position.x, p.pose.position.y]); }, { throttle_rate: 200 });
  subscribe('/avoid/obstacles', 'sensor_msgs/PointCloud', (m) => { S.obstacles = m.points; S.obstaclesAt = now(); });
  subscribe('/avoid/debug', 'std_msgs/String', onDebug, { throttle_rate: 90 });
  subscribe('/odom', 'nav_msgs/Odometry', (m) => { if (!S.isSim) S.actualV = { v: Math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), at: now() }; }, { throttle_rate: 100 });
  syncOptionalTopics();
  await loadStatic();
  detectMode();
  pollNodes();
}

// field geometry and the footprint: parameters, re-read on every (re)connection
let loadingStatic = false, staticTried = -1e9;
async function loadStatic() {
  if (loadingStatic || now() - staticTried < 3) return;
  loadingStatic = true;
  staticTried = now();
  try {
    const [field, obs, fp] = await Promise.all([getParam('/field'), getParam('/obstacles'), getParam('/robot/footprint')]);
    S.field = field || null;
    S.obsCfg = obs || null;
    S.footprint = Array.isArray(fp) && fp.length >= 3 ? fp : null;
  } catch (e) {
    toast('讀取場地參數失敗', e.message, 'err');
  }
  loadingStatic = false;
}

async function detectMode() {
  if (!S.connected) return;
  try {
    const services = (await call('/rosapi/services', 'rosapi/Services', {})).services;
    const sim = services.includes('/avoid_sim/reset');
    if (sim !== S.isSim) {
      S.isSim = sim;
      document.body.classList.toggle('is-sim', sim);
      syncOptionalTopics();
    }
  } catch (e) { /* rosapi not up yet: retry below */ }
  updateModeTag();
}
setInterval(detectMode, 5000);

function updateModeTag() {
  const tag = $('mode-tag');
  const sensor = S.sensor ? (S.sensor === 'camera' ? '攝影機' : '雷射') : '?';
  tag.textContent = S.connected ? `${S.isSim ? '模擬' : '實車'} · ${sensor}` : '—';
  tag.className = 'tag ' + (S.connected ? (S.isSim ? 'sim' : 'real') : '');
  // camera images: black_item on the robot, virtual_camera in the simulation (both via mjpeg_server)
  document.body.classList.toggle('is-camera', S.sensor === 'camera');
  if (S.sensor !== 'camera' && activeTab() === 'camera') showTab('run');
  else syncCamera();
}

// subscriptions that depend on toggles / the mode
function syncOptionalTopics() {
  if (!S.connected) return;
  const scanOn = $('show-scan').checked;
  if (scanOn) {
    subscribe('/scan', 'sensor_msgs/LaserScan', (m) => { S.scan = { m, pose: S.status && S.status.pose, at: now() }; }, { throttle_rate: 150 });
    subscribe('/boundary', 'sensor_msgs/LaserScan', (m) => { S.boundary = { m, pose: S.status && S.status.pose, at: now() }; }, { throttle_rate: 150 });
  } else { unsubscribe('/scan'); unsubscribe('/boundary'); S.scan = S.boundary = null; }
  if ($('show-cspace').checked) {
    subscribe('/avoid/cspace', 'nav_msgs/OccupancyGrid', onCspace, { throttle_rate: 1000, compression: 'png' });
  } else { unsubscribe('/avoid/cspace'); S.cspace = null; }
  if (S.isSim) {
    subscribe('/sim/odom_true', 'nav_msgs/Odometry', onTruth, { throttle_rate: 100 });
    subscribe('/avoid_sim/layout', 'geometry_msgs/PoseArray', (m) => { S.layout = m.poses.map((p) => [p.position.x, p.position.y]); });
    // latched, seq = seed: also catches resets made by others (start-up, evaluate.py)
    subscribe('/avoid_sim/reset_event', 'std_msgs/Header', (m) => {
      if (S.lastSeed !== m.seq) { S.trail = []; S.truthTrail = []; }
      S.lastSeed = m.seq;
      $('sim-seed').value = String(m.seq);
      if (!$('sim-info').textContent.startsWith(`seed ${m.seq}：`)) $('sim-info').textContent = `目前場地 seed ${m.seq}`;
    });
  } else { ['/sim/odom_true', '/avoid_sim/layout', '/avoid_sim/reset_event'].forEach(unsubscribe); S.truth = null; S.layout = []; }
}
for (const id of ['show-scan', 'show-cspace']) $(id).addEventListener('change', syncOptionalTopics);

// ------------------------------------------------------------------ messages
function yawOf(q) { return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)); }

function onStatus(m) {
  S.status = m;
  S.statusAt = now();
  if (!S.field || !S.footprint) loadStatic();     // nav_node (which loads them) started after the page
  if (m.sensor_config && m.sensor_config !== S.sensor) { S.sensor = m.sensor_config; updateModeTag(); loadParams(); }
  const prev = S.prevState;
  S.prevState = m.state_name;
  if (prev && prev !== m.state_name) onTransition(prev, m);
  if (m.state_name === 'MAPPING' || m.state_name === 'RUNNING') {
    const last = S.trail[S.trail.length - 1];
    if (!last || Math.hypot(last[0] - m.pose.x, last[1] - m.pose.y) > 0.01)
      S.trail.push([m.pose.x, m.pose.y, Math.hypot(m.cmd.linear.x, m.cmd.linear.y)]);
    if (S.trail.length > 3000) S.trail.shift();
  }
}

function onDebug(m) {
  try { S.debug = JSON.parse(m.data); } catch (e) { S.debug = { raw: m.data }; return; }
  const st = S.status;
  if (!st || !(st.state_name === 'RUNNING' || st.state_name === 'MAPPING')) return;
  if (S.debug.v_max > 0) S.vmax = S.debug.v_max;
  S.speedHist.push({
    t: st.elapsed, cmd: S.debug.v_cmd, target: S.debug.v_target, limit: S.debug.speed_limit,
    actual: S.actualV && now() - S.actualV.at < 0.5 ? S.actualV.v : null,
  });
  if (S.speedHist.length > 1200) S.speedHist.shift();
}

function onTransition(prev, m) {
  const active = (s) => s === 'MAPPING' || s === 'RUNNING';
  if (active(m.state_name) && !active(prev)) { S.trail = []; S.truthTrail = []; S.speedHist = []; }
  if (m.state_name === 'FINISHED') {
    const sp = speedStats();
    addRun(`抵達終點 ${m.elapsed.toFixed(2)} s${sp ? `（平均 ${sp.mean.toFixed(2)} m/s）` : ''}`, 'ok');
    toast(`抵達終點：${m.elapsed.toFixed(2)} s`, m.reason, 'ok');
  } else if (m.state_name === 'SAFE_STOP') {
    addRun(`異常停車：${m.reason}`, 'bad');
    toast('異常停車（SAFE_STOP）', m.reason, 'err');
  }
}

function addRun(text, kind) {
  const t = new Date().toLocaleTimeString();
  S.runs.unshift({ t, text, kind, seed: S.isSim ? S.lastSeed : null });
  S.runs = S.runs.slice(0, 8);
  $('last-run').innerHTML = S.runs.map((r) =>
    `<div class="light ${r.kind}" style="margin:2px 0"><span class="dot"></span>${r.t}${r.seed != null ? ` · seed ${r.seed}` : ''}　${escapeHtml(r.text)}</div>`).join('');
}

function onTruth(m) {
  const p = m.pose.pose;
  S.truth = { x: p.position.x, y: p.position.y, th: yawOf(p.orientation), at: now() };
  S.actualV = { v: Math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), at: now() };
  const st = S.status && S.status.state_name;
  if (st === 'MAPPING' || st === 'RUNNING') {
    const last = S.truthTrail[S.truthTrail.length - 1];
    if (!last || Math.hypot(last[0] - S.truth.x, last[1] - S.truth.y) > 0.01) S.truthTrail.push([S.truth.x, S.truth.y]);
    if (S.truthTrail.length > 3000) S.truthTrail.shift();
  }
}

function onCspace(m) {
  const w = m.info.width, h = m.info.height, d = m.data;
  if (!w || !h) { S.cspace = null; return; }         // nav_node was reset
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  const ctx = c.getContext('2d');
  const img = ctx.createImageData(w, h);
  for (let i = 0; i < w * h; ++i) {
    if (d[i] > 50) { const k = i * 4; img.data[k] = 160; img.data[k + 1] = 110; img.data[k + 2] = 255; img.data[k + 3] = 70; }
  }
  ctx.putImageData(img, 0, 0);
  S.cspace = c;
  S.cspaceInfo = { x0: m.info.origin.position.x, y0: m.info.origin.position.y, res: m.info.resolution, w, h };
}

// ------------------------------------------------------------------ nodes
function expectedNodes() {
  const l = ['/nav_node', '/rosbridge_websocket', '/rosapi', '/node_watch'];
  if (S.sensor === 'camera') l.push('/camera_adapter');
  if (S.sensor === 'lidar') l.push('/lidar_adapter');
  if (S.isSim) {
    l.push('/gazebo', '/field_manager', '/odom_noise', '/robot_state_publisher');
    if (S.sensor === 'camera') l.push('/virtual_camera', '/mjpeg_server');
  } else {
    l.push('/motion', '/imu_3d', '/motion_adapter', '/robot_state_publisher');
    if (S.sensor === 'camera') l.push('/prosilica_driver', '/black_item', '/mjpeg_server');
    if (S.sensor === 'lidar') l.push('/laser_0', '/laser_1', '/laser_2');
  }
  return l;
}

// Liveness comes from node_watch (/avoid/nodes): it pings every node, so a crashed node that is
// still registered shows red. Without node_watch, fall back to the master's list via rosapi, which
// cannot tell a crashed node from a live one (said so on the page).
function onNodes(m) {
  try { S.watch = { nodes: JSON.parse(m.data).nodes, at: now() }; } catch (e) { /* ignore */ }
}
const watchFresh = () => S.watch && now() - S.watch.at < 2.5;

async function pollNodes() {
  if (S.connected && !watchFresh()) {
    try {
      S.nodes = (await call('/rosapi/nodes', 'rosapi/Nodes', {})).nodes;
      S.nodesErr = null;
    } catch (e) { S.nodesErr = String(e); }
  }
}
setInterval(pollNodes, 1000);

function renderNodes() {
  const el = $('nodes');
  if (!S.connected) { el.innerHTML = '<span class="muted">rosbridge 未連線</span>'; return; }
  let alive, note = '';
  if (watchFresh()) alive = S.watch.nodes;
  else if (S.nodes && !S.nodesErr) {
    alive = Object.fromEntries(S.nodes.map((n) => [n, true]));
    note = '<div class="muted">node_watch 沒有回報：以下只代表「有註冊」，看不出 crash 的節點</div>';
  } else { el.innerHTML = `<span class="muted">node_watch 和 rosapi 都沒有回應 ${escapeHtml(S.nodesErr || '')}</span>`; return; }
  const exp = expectedNodes();
  const light = (n) => {
    const a = alive[n];
    const cls = a === true ? 'ok' : 'bad';
    const tip = a === false ? '已註冊，但 ping 不到（crash？）' : a === undefined ? '沒有在執行' : '';
    return `<span class="light ${cls}" title="${tip}"><span class="dot"></span>${n.slice(1)}${a === false ? '<span class="val">無回應</span>' : ''}</span>`;
  };
  const hidden = (n) => n.startsWith('/avoid_monitor') || n.startsWith('/rostopic') || n.startsWith('/rosnode') || n === '/rosout';
  const others = Object.keys(alive).filter((n) => !exp.includes(n) && !hidden(n));
  const deadOthers = others.filter((n) => alive[n] === false);
  el.innerHTML = exp.map(light).join('') + deadOthers.map(light).join('') +
    (others.length > deadOthers.length ? `<span class="light" title="${escapeHtml(others.filter((n) => alive[n]).join(' '))}"><span class="dot"></span>其他 ${others.length - deadOthers.length}</span>` : '') + note;
}

// ------------------------------------------------------------------ status bar + state card
// Required inputs turn red at nav_node's own timeout (it refuses to start / stops on them);
// the optional red-line input only turns yellow.
function ageClass(a, required = true) {
  if (a === undefined || a === null || a < 0) return required ? 'bad' : 'warn';
  if (a <= AGE_OK) return 'ok';
  return required ? 'bad' : 'warn';
}
const fmtAge = (a) => (a === undefined || a < 0 ? '無資料' : a < 10 ? `${a.toFixed(2)}s` : `${a.toFixed(0)}s`);

function navAlive() { return S.connected && S.status && now() - S.statusAt < NAV_SILENT; }

function renderStatus() {
  const lights = [];
  lights.push(['rosbridge', S.connected ? 'ok' : 'bad', S.connected ? '已連線' : '重新連線中']);
  const alive = navAlive();
  const navAge = S.status ? now() - S.statusAt : -1;
  lights.push(['nav_node', alive ? 'ok' : 'bad', alive ? '' : (S.status ? `停了 ${navAge.toFixed(0)}s` : '沒有回報')]);
  const st = alive ? S.status : null;
  const L = (name, age, req = true) => lights.push([name, st ? ageClass(age, req) : '', st ? fmtAge(age) : '—']);
  const sensorName = S.sensor === 'lidar' ? '雷射' : S.sensor === 'camera' ? '攝影機' : '障礙物';
  L(sensorName, st && st.age_obstacles);
  if (S.sensor !== 'lidar') L('紅線', st && st.age_boundary, false);
  L('里程計', st && st.age_odom);
  L('IMU', st && st.age_imu);
  if (st) {
    const sxy = Math.max(st.pose_std_x, st.pose_std_y);
    lights.push(['定位', sxy < 0.03 ? 'ok' : sxy < 0.08 ? 'warn' : 'bad', `σ ${(sxy * 100).toFixed(1)}cm`]);
  } else lights.push(['定位', '', '—']);
  if (S.isSim) lights.push(['真值', S.truth && now() - S.truth.at < 1 ? 'ok' : 'warn', S.truth ? '' : '無']);
  $('lights').innerHTML = lights.map(([n, c, v]) =>
    `<span class="light ${c}"><span class="dot"></span>${n}<span class="val">${v}</span></span>`).join('');

  // state card
  const nameEl = $('state-name');
  if (!S.connected) { nameEl.textContent = '未連線'; nameEl.className = 'state'; $('state-reason').textContent = `連不到 ${BRIDGE_URL}`; }
  else if (!alive) { nameEl.textContent = 'nav_node 沒有回應'; nameEl.className = 'state SAFE_STOP'; $('state-reason').textContent = '看 tmux 的 nav 視窗'; }
  else {
    nameEl.textContent = `${STATE_ZH[st.state_name] || st.state_name}`;
    nameEl.title = st.state_name;
    nameEl.className = `state ${st.state_name}`;
    $('state-reason').textContent = `${st.state_name} · ${st.reason}`;
  }
  $('elapsed').textContent = st && st.elapsed > 0 ? `${st.elapsed.toFixed(1)} s` : '';
  $('pose-line').textContent = st ? `x ${st.pose.x.toFixed(3)}  y ${st.pose.y.toFixed(3)}  θ ${(st.pose.theta * 180 / Math.PI).toFixed(1)}°  ` +
    `v ${Math.hypot(st.cmd.linear.x, st.cmd.linear.y).toFixed(2)} m/s` : '';

  renderSpeed(st);

  // start button: only when everything it needs is alive
  let why = '';
  if (!S.connected) why = 'rosbridge 未連線';
  else if (!alive) why = 'nav_node 沒有回應';
  else if (st.state_name === 'RUNNING' || st.state_name === 'MAPPING') why = '';
  else {
    const stale = [];
    if (ageClass(st.age_obstacles) !== 'ok') stale.push(sensorName);
    if (ageClass(st.age_odom) !== 'ok') stale.push('里程計');
    if (ageClass(st.age_imu) !== 'ok') stale.push('IMU');
    if (stale.length) why = `等待資料：${stale.join('、')}`;
  }
  const running = st && (st.state_name === 'RUNNING' || st.state_name === 'MAPPING');
  $('btn-start').disabled = !!why || running || busy.start;
  $('start-hint').textContent = busy.start ? startPhase : why;
}

// ------------------------------------------------------------------ speed
// nav_node was reset: drop everything that belonged to the previous field / run
function clearRunDisplay() {
  S.trail = []; S.truthTrail = []; S.speedHist = [];
  S.obstacles = []; S.path = []; S.cspace = null; S.scan = null; S.boundary = null;
}

function speedStats() {
  const h = S.speedHist.filter((p) => p.cmd !== undefined);
  if (h.length < 2) return null;
  const mean = h.reduce((a, p) => a + p.cmd, 0) / h.length;
  const share = {};
  for (const p of h) share[p.limit] = (share[p.limit] || 0) + 1;
  return { mean, share, n: h.length };
}

function renderSpeed(st) {
  const d = S.debug || {};
  const running = st && (st.state_name === 'RUNNING' || st.state_name === 'MAPPING');
  const v = st ? Math.hypot(st.cmd.linear.x, st.cmd.linear.y) : null;
  $('speed-val').textContent = v === null ? '—' : v.toFixed(2);
  const vmax = S.vmax || 0.8;
  $('speed-bar-fill').style.width = `${Math.min(100, (v || 0) / vmax * 100)}%`;
  $('speed-bar-fill').style.background = speedColor((v || 0) / vmax);
  const lim = running ? d.speed_limit : '';
  const L = LIMITS[lim];
  const tag = $('speed-limit');
  tag.textContent = L ? `限速：${L[0]}` : '';
  tag.style.display = L ? '' : 'none';
  tag.style.borderColor = L ? L[1] : '';
  tag.style.color = L ? L[1] : '';
  const tgt = running && d.v_target !== undefined ? Math.min(1, d.v_target / vmax) : null;
  $('speed-bar-target').style.display = tgt === null ? 'none' : '';
  if (tgt !== null) $('speed-bar-target').style.left = `calc(${tgt * 100}% - 1px)`;
  const a = S.actualV && now() - S.actualV.at < 1 ? S.actualV.v : null;
  $('speed-actual').textContent = a === null ? '' : `${S.isSim ? '真實' : '里程計'} ${a.toFixed(2)} m/s`;
  const sp = speedStats();
  $('speed-stats').textContent = sp ? `平均 ${sp.mean.toFixed(2)} m/s · 最常限速：${(LIMITS[Object.entries(sp.share).sort((x, y) => y[1] - x[1])[0][0]] || ['—'])[0]}` : '';
}

// 0 = red, 0.5 = yellow, 1 = green
function speedColor(f) {
  f = Math.max(0, Math.min(1, f));
  const c = f < 0.5 ? [[248, 81, 73], [242, 201, 76], f / 0.5] : [[242, 201, 76], [63, 185, 80], (f - 0.5) / 0.5];
  const [a, b, t] = c;
  return `rgb(${a.map((x, i) => Math.round(x + (b[i] - x) * t)).join(',')})`;
}

const chart = $('speed-chart');
function drawSpeedChart() {
  const r = chart.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  if (chart.width !== Math.round(r.width * dpr)) { chart.width = Math.round(r.width * dpr); chart.height = Math.round(r.height * dpr); }
  const c = chart.getContext('2d'), W = chart.width, H = chart.height;
  const strip = 8 * dpr, padL = 30 * dpr, padB = strip + 14 * dpr, padT = 6 * dpr;
  c.clearRect(0, 0, W, H);
  const h = S.speedHist;
  const vmax = Math.max(S.vmax || 0.8, 0.1) * 1.1;
  const tmax = Math.max(10, h.length ? h[h.length - 1].t : 0);
  const X = (t) => padL + (W - padL - 4 * dpr) * t / tmax, Y = (v) => padT + (H - padB - padT) * (1 - v / vmax);
  c.font = `${10 * dpr}px sans-serif`;
  c.fillStyle = '#8b93a1';
  c.strokeStyle = 'rgba(255,255,255,0.08)';
  c.lineWidth = dpr;
  for (let v = 0; v <= vmax; v += 0.2) {
    c.beginPath(); c.moveTo(padL, Y(v)); c.lineTo(W, Y(v)); c.stroke();
    c.fillText(v.toFixed(1), 2 * dpr, Y(v) + 3 * dpr);
  }
  for (let t = 0; t <= tmax; t += tmax > 30 ? 10 : 2) c.fillText(`${t}s`, X(t) - 6 * dpr, H - 2 * dpr);
  // v_max
  c.setLineDash([4 * dpr, 4 * dpr]);
  c.strokeStyle = 'rgba(255,255,255,0.35)';
  c.beginPath(); c.moveTo(padL, Y(S.vmax)); c.lineTo(W, Y(S.vmax)); c.stroke();
  c.setLineDash([]);
  if (!h.length) {
    c.fillStyle = '#8b93a1';
    c.fillText('開始之後會畫出這一輪的速度', padL + 10 * dpr, H / 2);
    return;
  }
  // limiter strip
  for (let i = 0; i < h.length; ++i) {
    const L = LIMITS[h[i].limit];
    if (!L) continue;
    const x0 = X(h[i].t), x1 = i + 1 < h.length ? X(h[i + 1].t) : x0 + 2 * dpr;
    c.fillStyle = L[1];
    c.fillRect(x0, H - padB + 3 * dpr, Math.max(1, x1 - x0), strip);
  }
  const line = (key, color, width) => {
    c.strokeStyle = color; c.lineWidth = width * dpr; c.beginPath();
    let started = false;
    for (const p of h) {
      if (p[key] === null || p[key] === undefined) { started = false; continue; }
      started ? c.lineTo(X(p.t), Y(p[key])) : c.moveTo(X(p.t), Y(p[key]));
      started = true;
    }
    c.stroke();
  };
  line('target', 'rgba(255,255,255,0.25)', 1);
  line('actual', 'rgba(255,255,255,0.85)', 1.5);
  line('cmd', '#4c8dff', 2);
}
$('limit-legend').innerHTML = '<span><i style="background:#4c8dff"></i>指令</span><span><i style="background:rgba(255,255,255,.85)"></i>實際</span>' +
  '<span><i style="background:rgba(255,255,255,.25)"></i>上限</span>' +
  Object.values(LIMITS).map(([n, col]) => `<span><i style="background:${col}"></i>${n}</span>`).join('');

// ------------------------------------------------------------------ commands
const busy = {};
async function command(cmd, quiet = false) {
  try {
    const r = await call('/avoid/command', 'avoid_msgs/Command', { command: cmd });
    if (!r.success) toast(`${cmd} 失敗`, r.message, 'err');
    else if (!quiet) toast(`${cmd} 成功`, r.message, 'ok');
    return r.success;
  } catch (e) {
    toast(`${cmd} 失敗`, e.message, 'err');
    return false;
  }
}

let cmdVel = null;
function zeroBurst() {
  // backup for a hung nav_node: a few zero velocity commands straight to the base
  if (!S.connected) return;
  if (!cmdVel) cmdVel = new ROSLIB.Topic({ ros: S.ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' });
  const zero = new ROSLIB.Message({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
  for (let i = 0; i < 10; ++i) setTimeout(() => S.connected && cmdVel.publish(zero), i * 50);
}

// Simulation: every start re-draws the field with the current seed first, so the robot leaves from
// the start pose (nav_node's start anchors the localisation there) on an untouched field. Same
// order as evaluate.py: reset, let the sensors refresh after the teleport, start. Stop (or a
// second start sequence) during the wait cancels it.
let startSeq = 0, startPhase = '';
const sleep = (s) => new Promise((r) => setTimeout(r, s * 1000));

function inputsFresh() {
  const st = S.status;
  return navAlive() && [st.age_obstacles, st.age_odom, st.age_imu].every((a) => ageClass(a) === 'ok');
}

async function doStart() {
  if ($('btn-start').disabled) return;
  const seq = ++startSeq;
  busy.start = true;
  $('btn-start').classList.add('busy');
  try {
    if (S.isSim) {
      startPhase = '重擺場地中…';
      if (!(await simReset(false, true)) || seq !== startSeq) return;
      startPhase = '等感測器更新…';
      await sleep(1.5);
      for (let i = 0; i < 50 && seq === startSeq && !inputsFresh(); ++i) await sleep(0.1);
      if (seq !== startSeq) return;          // stopped meanwhile
    }
    await command('start', true);            // a refusal still shows its reason
  } finally {
    busy.start = false;
    startPhase = '';
    $('btn-start').classList.remove('busy');
  }
}
async function doStop() {
  ++startSeq;
  zeroBurst();
  await command('stop', true);
}
$('btn-start').addEventListener('click', doStart);
$('btn-reset').addEventListener('click', async () => {
  if (await command('reset', true)) {
    clearRunDisplay();
    toast('定位已重設到起點', '障礙物記憶已清除', 'ok');
  }
});
$('btn-stop').addEventListener('click', doStop);
document.addEventListener('keydown', (e) => {
  if (e.code === 'Space' && !['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) {
    e.preventDefault();
    doStop();
  }
});

// simulation: re-draw the field (the result goes in the sim card, only failures as a toast)
const SIM_BUTTONS = ['sim-reset', 'sim-next', 'sim-random'];
async function simReset(nextSeed) {
  const seedEl = $('sim-seed');
  if (nextSeed) seedEl.value = String((parseInt(seedEl.value, 10) || 0) + 1);
  const seed = parseInt(seedEl.value, 10);
  if (!Number.isFinite(seed) || seed < 0) { toast('seed 要是 0 以上的整數', '', 'err'); return false; }
  for (const id of SIM_BUTTONS) $(id).disabled = true;
  let ok = false;
  try {
    if (S.status && S.status.state_name !== 'IDLE') await command('stop', true);
    $('sim-info').textContent = `seed ${seed}：重擺中…`;
    const r = await call('/avoid_sim/reset', 'avoid_msgs/ResetField',
      { seed, layout: $('sim-layout').value, start_error_xy: 0, start_error_yaw: 0, use_obstacles: [], use_start: [] }, 20);
    if (r.success) {
      await command('reset', true);       // robot back on the start pose: re-anchor, forget the old obstacle map
      clearRunDisplay();
      $('sim-info').textContent = `seed ${r.seed}：${r.obstacles.length} 個障礙物，取樣 ${r.attempts} 次`;
      ok = true;
    } else { toast('重擺失敗', r.message, 'err'); $('sim-info').textContent = `seed ${seed}：重擺失敗`; }
  } catch (e) {
    toast('重擺失敗', e.message, 'err');
    $('sim-info').textContent = `seed ${seed}：重擺失敗`;
  }
  for (const id of SIM_BUTTONS) $(id).disabled = false;
  return ok;
}
async function saveCase() {
  const expect = {};
  if ($('case-result').value) expect.result = $('case-result').value;
  const mt = parseFloat($('case-maxtime').value);
  if (Number.isFinite(mt) && mt > 0) expect.max_time = mt;
  const id = $('case-id').value.trim();
  if (id && !/^[A-Za-z0-9_.-]+$/.test(id)) { toast('名稱只能用英數、- _ .', id, 'err'); return; }
  const last = S.runs[0];
  const sp = speedStats();
  const observed = $('case-observed').value.trim() ||
    (last ? `${last.text}${sp ? `；最常限速 ${(LIMITS[Object.entries(sp.share).sort((x, y) => y[1] - x[1])[0][0]] || ['—'])[0]}` : ''}` : '');
  try {
    const r = await call('/avoid_sim/save_case', 'avoid_msgs/SaveCase', {
      file: $('case-file').value.trim(), id, sensor: S.sensor || 'camera', note: $('case-note').value,
      observed, expect: Object.keys(expect).length ? JSON.stringify(expect) : '{}',
      status: document.querySelector('input[name=case-status]:checked').value,
    }, 6);
    if (r.success) {
      toast(`已記錄 case：${r.id}`, r.path, 'ok');
      $('case-id').value = ''; $('case-note').value = ''; $('case-observed').value = '';
      $('case-form').open = false;
    } else toast('記錄失敗', r.message, 'err');
  } catch (e) { toast('記錄失敗', e.message, 'err'); }
}
$('case-save').addEventListener('click', saveCase);
$('sim-reset').addEventListener('click', () => simReset(false));
$('sim-next').addEventListener('click', () => simReset(true));
$('sim-random').addEventListener('click', () => {
  $('sim-seed').value = String(Math.floor(Math.random() * 100000));
  simReset(false);
});

// ------------------------------------------------------------------ tabs
const activeTab = () => document.querySelector('#tabs button.active').dataset.tab;
function showTab(name) {
  document.querySelectorAll('#tabs button').forEach((x) => x.classList.toggle('active', x.dataset.tab === name));
  document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.id === `tab-${name}`));
  syncCamera();
}
document.querySelectorAll('#tabs button').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));

// ------------------------------------------------------------------ parameters
function buildParamForm() {
  $('params').innerHTML = TUNABLES.map(([group, items]) => `<div class="pgroup"><h4>${group}</h4>` +
    items.map(([n, label, unit, step]) => `<div class="prow"><span class="pname">${label}<small>${n}</small></span>` +
      `<input type="number" step="${step}" id="p-${n}" data-name="${n}"><span class="unit">${unit}</span></div>`).join('') +
    '</div>').join('');
  $('params').querySelectorAll('input').forEach((i) => i.addEventListener('input', () =>
    i.classList.toggle('changed', i.value !== i.dataset.loaded)));
}
buildParamForm();

async function loadParams(showToast = false) {
  if (!S.connected) return;
  try {
    const all = await getParam('/nav_node');
    if (!all) throw new Error('沒有 /nav_node 參數（nav_node 沒在跑？）');
    for (const i of $('params').querySelectorAll('input')) {
      const v = all[i.dataset.name];
      i.value = v === undefined ? '' : String(v);
      i.dataset.loaded = i.value;
      i.classList.remove('changed');
    }
    if (showToast) toast('參數已讀取', `/nav_node（${S.sensor || '?'} 設定）`, 'info');
  } catch (e) { toast('讀取參數失敗', e.message, 'err'); }
}
async function applyParams() {
  const changed = [...$('params').querySelectorAll('input.changed')];
  if (!changed.length) { toast('沒有修改', '', 'info'); return 0; }
  try {
    for (const i of changed) {
      const v = parseFloat(i.value);
      if (!Number.isFinite(v)) throw new Error(`${i.dataset.name} 不是數字`);
      await setParam(`/nav_node/${i.dataset.name}`, v);
      i.dataset.loaded = i.value;
      i.classList.remove('changed');
    }
    const running = S.status && (S.status.state_name === 'RUNNING' || S.status.state_name === 'MAPPING');
    toast(`已套用 ${changed.length} 個參數`, running ? '這一輪不變，下一次開始生效' : '下一次開始生效', 'ok');
    return changed.length;
  } catch (e) { toast('套用失敗', e.message, 'err'); return -1; }
}
$('params-reload').addEventListener('click', () => loadParams(true));
$('params-apply').addEventListener('click', applyParams);
$('params-save').addEventListener('click', async () => {
  if ($('params').querySelector('input.changed') && (await applyParams()) < 0) return;
  await command('save');
});

// ------------------------------------------------------------------ debug tab
function renderDebug() {
  const d = S.debug || {};
  $('debug-table').innerHTML = Object.keys(d).map((k) =>
    `<tr><td>${escapeHtml(k)}</td><td>${typeof d[k] === 'number' ? +d[k].toFixed(4) : escapeHtml(String(d[k]))}</td></tr>`).join('') ||
    '<tr><td class="muted">沒有資料</td><td></td></tr>';
  const t = now();
  $('rates-table').innerHTML = Object.keys(S.rates).sort().map((k) => {
    const r = S.rates[k].filter((x) => t - x <= 2.0);
    const hz = r.length > 1 ? (r.length - 1) / (r[r.length - 1] - r[0]) : 0;
    return `<tr><td>${k}</td><td>${hz ? hz.toFixed(1) + ' Hz' : '—'}</td></tr>`;
  }).join('');
}

// ------------------------------------------------------------------ camera tab
// One mjpeg stream per ticked topic, open only while the tab is shown; a broken stream (mjpeg_server
// not up yet, restarting) retries every 3 s instead of raising toasts.
const CAM_NAMES = { '/camera/black': '黑色遮罩', '/camera/red': '紅色遮罩', '/camera/image_raw': '原始影像' };
const camViews = {};
$('cam-note').textContent = `影像來自 mjpeg_server（${MJPEG_URL}）。`;

function camWanted() {
  if (activeTab() !== 'camera' || !document.body.classList.contains('is-camera')) return [];
  return [...$('cam-pick').querySelectorAll('input:checked')]
    .filter((i) => !(S.isSim && i.closest('.real-only')))        // no raw image in the simulation
    .map((i) => i.value);
}
function camOpen(v) {
  clearTimeout(v.retry);
  v.fig.classList.remove('err');
  v.st.textContent = '連線中…';
  // mjpeg_server does not URL-decode: an encoded %2Fcamera%2Fblack subscribes to nothing
  v.img.src = `${MJPEG_URL}/stream?topic=${v.topic}&t=${Date.now()}`;
}
function syncCamera() {
  const want = camWanted();
  for (const topic of Object.keys(camViews)) {
    if (want.includes(topic)) continue;
    const v = camViews[topic];
    clearTimeout(v.retry);
    v.img.removeAttribute('src');          // closes the stream
    v.fig.remove();
    delete camViews[topic];
  }
  for (const topic of want) {
    if (camViews[topic]) continue;
    const fig = document.createElement('figure');
    fig.className = 'cam-view';
    fig.innerHTML = '<img alt=""><figcaption><span class="name"></span><span class="st"></span></figcaption>';
    const v = { topic, fig, img: fig.querySelector('img'), st: fig.querySelector('.st'), retry: null };
    fig.querySelector('.name').textContent = `${CAM_NAMES[topic] || topic}  ${topic}`;
    v.img.addEventListener('load', () => { v.st.textContent = ''; });
    v.img.addEventListener('error', () => {
      if (camViews[topic] !== v) return;                          // closed on purpose
      fig.classList.add('err');
      v.st.textContent = '沒有影像，3 秒後重試';
      v.retry = setTimeout(() => camViews[topic] === v && camOpen(v), 3000);
    });
    camViews[topic] = v;
    camOpen(v);
  }
  // keep the order of the check boxes
  for (const i of $('cam-pick').querySelectorAll('input')) if (camViews[i.value]) $('cam-views').appendChild(camViews[i.value].fig);
}
$('cam-pick').addEventListener('change', syncCamera);

// ------------------------------------------------------------------ map
const canvas = $('map');
const ctx = canvas.getContext('2d');
let view = { s: 100, cx: 0, cy: 0 };
const VIEW = { x0: -3.75, x1: 3.75, y0: -2.75, y1: 2.75 };

function resize() {
  const r = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(r.width * dpr);
  canvas.height = Math.round(r.height * dpr);
  const s = Math.min(canvas.width / (VIEW.x1 - VIEW.x0), canvas.height / (VIEW.y1 - VIEW.y0));
  view = { s, cx: canvas.width / 2 - s * (VIEW.x0 + VIEW.x1) / 2, cy: canvas.height / 2 + s * (VIEW.y0 + VIEW.y1) / 2, dpr };
}
window.addEventListener('resize', resize);
canvas.addEventListener('mousemove', (e) => {
  const r = canvas.getBoundingClientRect(), dpr = view.dpr || 1;
  const x = ((e.clientX - r.left) * dpr - view.cx) / view.s, y = (view.cy - (e.clientY - r.top) * dpr) / view.s;
  $('cursor-pos').textContent = `x ${x.toFixed(2)}  y ${y.toFixed(2)} m`;
});
canvas.addEventListener('mouseleave', () => { $('cursor-pos').textContent = ''; });

const F_DEFAULT = {
  surface: { x_min: -3, x_max: 3, y_min: -2, y_max: 2 }, red_line_y: 1.5, red_line_width: 0.05,
  red_line_x_min: -3, red_line_x_max: 3, start_line_x: -2, goal_line_x: 3, wall_x: 3.55, wall_y: 2.55, wall_thickness: 0.15,
};

function world() { ctx.setTransform(view.s, 0, 0, -view.s, view.cx, view.cy); }   // metres, y up
function px(m) { return m / view.s * (view.dpr || 1); }                              // screen px -> metres

function drawField(f) {
  const sf = f.surface || F_DEFAULT.surface;
  ctx.fillStyle = '#20352a';
  ctx.fillRect(sf.x_min, sf.y_min, sf.x_max - sf.x_min, sf.y_max - sf.y_min);
  // walls
  const wx = f.wall_x, wy = f.wall_y, wt = f.wall_thickness;
  ctx.fillStyle = '#4a4f57';     // black low wall, drawn grey to stand out from the background
  ctx.fillRect(-wx - wt / 2, -wy - wt / 2, 2 * wx + wt, wt);
  ctx.fillRect(-wx - wt / 2, wy - wt / 2, 2 * wx + wt, wt);
  ctx.fillRect(-wx - wt / 2, -wy - wt / 2, wt, 2 * wy + wt);
  ctx.fillRect(wx - wt / 2, -wy - wt / 2, wt, 2 * wy + wt);
  // grid
  ctx.strokeStyle = 'rgba(255,255,255,0.06)';
  ctx.lineWidth = px(1);
  ctx.beginPath();
  for (let x = Math.ceil(-wx * 2) / 2; x <= wx; x += 0.5) { ctx.moveTo(x, -wy); ctx.lineTo(x, wy); }
  for (let y = Math.ceil(-wy * 2) / 2; y <= wy; y += 0.5) { ctx.moveTo(-wx, y); ctx.lineTo(wx, y); }
  ctx.stroke();
  // red lines
  ctx.fillStyle = '#e53935';
  for (const sgn of [1, -1]) ctx.fillRect(f.red_line_x_min, sgn * f.red_line_y - f.red_line_width / 2, f.red_line_x_max - f.red_line_x_min, f.red_line_width);
  // start / goal lines
  ctx.lineWidth = px(2);
  ctx.strokeStyle = 'rgba(255,255,255,0.6)';
  ctx.setLineDash([px(6), px(5)]);
  ctx.beginPath(); ctx.moveTo(f.start_line_x, -f.red_line_y); ctx.lineTo(f.start_line_x, f.red_line_y); ctx.stroke();
  ctx.setLineDash([]);
  ctx.strokeStyle = '#f2c94c';
  ctx.lineWidth = px(3);
  ctx.beginPath(); ctx.moveTo(f.goal_line_x, -f.red_line_y); ctx.lineTo(f.goal_line_x, f.red_line_y); ctx.stroke();
}

function poly(points, x, y, th) {
  const c = Math.cos(th), s = Math.sin(th);
  ctx.beginPath();
  points.forEach(([px_, py_], i) => {
    const X = x + c * px_ - s * py_, Y = y + s * px_ + c * py_;
    i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y);
  });
  ctx.closePath();
}
const FP_FALLBACK = (() => { const p = []; for (let i = 0; i < 24; ++i) p.push([0.2 * Math.cos(i * Math.PI / 12), 0.2 * Math.sin(i * Math.PI / 12)]); return p; })();

function drawRobot(x, y, th, fill, stroke, dashed) {
  const fp = S.footprint || FP_FALLBACK;
  poly(fp, x, y, th);
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  ctx.strokeStyle = stroke;
  ctx.lineWidth = px(2);
  ctx.setLineDash(dashed ? [px(5), px(4)] : []);
  ctx.stroke();
  ctx.setLineDash([]);
  const front = Math.max(...fp.map((p) => p[0]));
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + Math.cos(th) * front, y + Math.sin(th) * front);
  ctx.stroke();
}

function line(points, color, width) {
  if (points.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = px(width);
  ctx.beginPath();
  points.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.stroke();
}

function drawScan(sc, color, innerColor) {
  if (!sc || !sc.pose || now() - sc.at > SCAN_STALE) return;
  const { m, pose } = sc;
  const c = Math.cos(pose.theta), s = Math.sin(pose.theta), d = px(4);
  for (let k = 0; k < m.ranges.length; ++k) {
    const r = m.ranges[k];
    if (r === null || !Number.isFinite(r) || r > 6) continue;      // NaN / inf arrive as null
    const a = m.angle_min + k * m.angle_increment;
    const bx = r * Math.cos(a), by = r * Math.sin(a);
    ctx.fillStyle = m.intensities && m.intensities[k] > 0.5 ? innerColor : color;
    ctx.fillRect(pose.x + c * bx - s * by - d / 2, pose.y + s * bx + c * by - d / 2, d, d);
  }
}

function draw() {
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = '#111316';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  world();
  const f = { ...F_DEFAULT, ...(S.field || {}) };
  drawField(f);

  if (S.cspace && $('show-cspace').checked) {
    const i = S.cspaceInfo;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(S.cspace, i.x0, i.y0, i.w * i.res, i.h * i.res);   // y-up transform: row 0 at y0
  }
  const showTruth = S.isSim && $('show-truth').checked;
  if (S.isSim && S.layout.length) {
    const h = ((S.obsCfg && S.obsCfg.size) || 0.45) / 2;
    ctx.fillStyle = showTruth ? '#3b414b' : 'rgba(59,65,75,0.35)';
    for (const [x, y] of S.layout) ctx.fillRect(x - h, y - h, 2 * h, 2 * h);
  }
  // remembered obstacles (nav_node grid, field frame)
  const stale = now() - S.obstaclesAt > 2;
  ctx.fillStyle = stale ? 'rgba(255,152,0,0.3)' : '#ff9800';
  const d = 0.02;
  for (const p of S.obstacles) ctx.fillRect(p.x - d / 2, p.y - d / 2, d, d);

  if ($('show-scan').checked) {
    drawScan(S.scan, 'rgba(255,255,255,0.85)', '#ff4dd2');
    drawScan(S.boundary, '#ff5252', '#ff5252');
  }
  if ($('show-trail').checked) {
    if (showTruth) line(S.truthTrail, 'rgba(255,255,255,0.45)', 1.5);
    ctx.lineWidth = px(3);
    ctx.lineCap = 'round';
    for (let i = 1; i < S.trail.length; ++i) {
      const [x0, y0] = S.trail[i - 1], [x1, y1, v] = S.trail[i];
      ctx.strokeStyle = speedColor((v || 0) / (S.vmax || 0.8));
      ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
    }
    ctx.lineCap = 'butt';
  }
  const st0 = S.status && S.status.state_name;
  if (st0 === 'RUNNING' || st0 === 'MAPPING') {        // nav_node only plans while running
    line(S.path, '#7ee787', 2.5);
    for (const [x, y] of S.path) { ctx.fillStyle = '#7ee787'; ctx.fillRect(x - px(3), y - px(3), px(6), px(6)); }
  }

  if (showTruth && S.truth) drawRobot(S.truth.x, S.truth.y, S.truth.th, null, 'rgba(255,255,255,0.8)', true);
  const st = S.status;
  if (st) {
    const alive = navAlive();
    drawRobot(st.pose.x, st.pose.y, st.pose.theta, alive ? 'rgba(76,141,255,0.30)' : 'rgba(120,120,120,0.3)', alive ? '#4c8dff' : '#777', false);
    // 2 sigma position uncertainty
    ctx.strokeStyle = 'rgba(76,141,255,0.7)';
    ctx.lineWidth = px(1);
    ctx.beginPath();
    ctx.ellipse(st.pose.x, st.pose.y, Math.max(2 * st.pose_std_x, px(2)), Math.max(2 * st.pose_std_y, px(2)), 0, 0, 2 * Math.PI);
    ctx.stroke();
  }
  // labels (screen space)
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.font = `${12 * (view.dpr || 1)}px sans-serif`;
  ctx.fillStyle = 'rgba(255,255,255,0.5)';
  const lx = (x) => view.cx + x * view.s, ly = (y) => view.cy - y * view.s;
  ctx.fillText('起點線', lx(f.start_line_x) + 4, ly(f.red_line_y) + 16 * (view.dpr || 1));
  ctx.fillText('終點線', lx(f.goal_line_x) - 46 * (view.dpr || 1), ly(f.red_line_y) + 16 * (view.dpr || 1));
  if (!S.field && S.connected) ctx.fillText('（場地參數未讀到，使用預設值）', 10, 20 * (view.dpr || 1));
}

function escapeHtml(s) { return String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c])); }

// ------------------------------------------------------------------ main loop
let lastUi = 0;
function frame() {
  draw();
  const t = now();
  if (t - lastUi > 0.1) { renderStatus(); renderNodes(); renderDebug(); drawSpeedChart(); lastUi = t; }
  requestAnimationFrame(frame);
}
resize();
connect();
renderNodes();
requestAnimationFrame(frame);
