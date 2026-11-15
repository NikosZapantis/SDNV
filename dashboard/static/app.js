'use strict';
/* SDNV dashboard client: Leaflet map + KPIs + controls over Socket.IO.
   Message shapes are produced by sdnv/world.py (describe/frame/stats) and sdnv/netmodel.py (kpi). */

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const num = (x, d = 1) => (x === null || x === undefined ? '—' : Number(x).toFixed(d));
const fmtT = (t) => {
  t = Math.max(0, Math.floor(t));
  const h = Math.floor(t / 3600), m = Math.floor((t % 3600) / 60), s = t % 60;
  return (h ? h + ':' + String(m).padStart(2, '0') : String(m).padStart(2, '0')) + ':' + String(s).padStart(2, '0');
};

const KIND = ['car', 'ambulance', 'police'];
const SIG = { G: '#1fbf4a', g: '#1fbf4a', Y: '#f5b800', R: '#e34948' };
const KPIS = [
  // key, label, unit, decimals
  ['em_latency_ms', 'EV latency', 'ms', 1],
  ['em_jitter_ms', 'EV jitter', 'ms', 1],
  ['em_tput_mbps', 'EV throughput', 'Mbit/s', 1],
  ['em_loss_pct', 'EV loss', '%', 1],
  ['be_latency_ms', 'Civilian latency', 'ms', 1],
  ['be_tput_mbps', 'Civilian tput Σ', 'Mbit/s', 1],
  ['suppression_pct', 'Helper suppression', '%', 0],
  ['policy_reaction_ms', 'Policy reaction', 'ms', 0],
  ['emapt_p90', 'EMAPT p90', 'ms', 1],
  ['mqtt_latency_ms', 'MQTT latency', 'ms', 1],
  ['coverage_pct', 'RSU coverage', '%', 0],
  ['ecm_cells', 'ECM cells', '', 0],
];
const CHART_WINDOW = 300;               // stats samples (≈ 5 min at 1 Hz)

const sock = io();
const S = { init: null, stats: null, sel: null, follow: false, lat: [], tab: 'vehicles', fitted: false, hoverX: null };
const vm = new Map();                   // vehicle id → {marker, el, cls}
const routes = new Map();               // EV id → polyline
let map, base, routeLayer, vehLayer;
let lights = [], lightState = '', rsuCov = [], rsuLbl = [];

const cmd = (op, extra = {}) => sock.emit('cmd', { op, ...extra });

// ── map ──────────────────────────────────────────────────────────────────
function setupMap() {
  map = L.map('map', { zoomSnap: 0.25 });
  map.createPane('lights').style.zIndex = 450;
  // standard OSM tiles (no key); style.css darkens the tile pane in dark mode
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  }).addTo(map);
  map.on('zoomstart', () => map.getContainer().classList.add('zooming'));
  map.on('zoomend', () => map.getContainer().classList.remove('zooming'));
  map.on('dragstart', () => { S.follow = false; renderSel(); });
  base = L.layerGroup().addTo(map);
  routeLayer = L.layerGroup().addTo(map);
  vehLayer = L.layerGroup().addTo(map);
}

function buildStatic(d) {
  base.clearLayers(); routeLayer.clearLayers(); vehLayer.clearLayers();
  routes.clear(); vm.clear();
  for (const [rank, pts] of d.roads) {
    L.polyline(pts, { className: `road r${rank}`, weight: rank >= 3 ? 4 : rank === 2 ? 3 : 2, interactive: false }).addTo(base);
  }
  rsuCov = d.rsus.map((r) => L.circle(r.ll, { radius: r.range, className: 'rsu-cov', interactive: false }).addTo(base));
  rsuLbl = d.rsus.map((r) => L.marker(r.ll, {
    icon: L.divIcon({ className: 'rsu-ic', html: `<b>${esc(r.id)}</b>`, iconSize: [56, 18] }),
    keyboard: false, title: `${r.id} · ${r.range} m${r.signal ? ' · signalised' : ''}`,
  }).addTo(base));
  lights = d.approaches.map(([, lat, lon]) => L.circleMarker([lat, lon], {
    pane: 'lights', radius: 3.5, weight: 1, color: 'rgba(0,0,0,.4)', fillColor: '#888', fillOpacity: 1, interactive: false,
  }).addTo(base));
  lightState = '';
  for (const p of d.pois) {
    L.marker(p.ll, { icon: L.divIcon({ className: 'poi-ic', html: '<b>H</b>', iconSize: [18, 18] }), keyboard: false })
      .bindTooltip(esc(p.name)).addTo(base);
  }
  if (!S.fitted) { map.fitBounds(d.map.bounds); S.fitted = true; }
}

function vehIcon(kind) {
  return L.divIcon({
    className: 'vm', iconSize: [22, 22], iconAnchor: [11, 11],
    html: `<div class="veh k${kind}"><svg viewBox="-8 -8 16 16"><path d="M0 -7 L5.5 6 L0 3 L-5.5 6 Z"/></svg></div>`,
  });
}

function onFrame(f) {
  $('#clock').textContent = fmtT(f.t);
  setScenario(f.scenario);
  const seen = new Set();
  for (const [id, lat, lon, hdg, , flags, , kind] of f.v) {
    seen.add(id);
    let m = vm.get(id);
    if (!m) {
      const marker = L.marker([lat, lon], { icon: vehIcon(kind), zIndexOffset: kind ? 1000 : 500, keyboard: false, title: id })
        .addTo(vehLayer).on('click', () => select(id));
      m = { marker, el: marker.getElement().firstElementChild, cls: '' };
      vm.set(id, m);
    } else {
      m.marker.setLatLng([lat, lon]);
    }
    m.el.style.transform = `rotate(${hdg}deg)`;
    const cls = `veh k${kind}${flags & 1 ? ' siren' : ''}${flags & 2 ? ' yield' : ''}${flags & 4 ? ' parked' : ''}${id === S.sel ? ' sel' : ''}`;
    if (cls !== m.cls) { m.el.className = cls; m.cls = cls; }
  }
  for (const [id, m] of vm) {
    if (!seen.has(id)) { vehLayer.removeLayer(m.marker); vm.delete(id); }
  }
  if (f.L !== lightState) {
    for (let i = 0; i < lights.length; i++) {
      const c = f.L[i];
      if (c === lightState[i]) continue;
      const held = c === 'g';
      lights[i].setStyle({ fillColor: SIG[c] || '#888', radius: held ? 5.5 : 3.5, weight: held ? 2 : 1, color: held ? '#fff' : 'rgba(0,0,0,.4)' });
    }
    lightState = f.L;
  }
  const ecm = new Set(f.ecm);
  rsuCov.forEach((c, i) => {
    c.getElement()?.classList.toggle('ecm', ecm.has(i));
    rsuLbl[i].getElement()?.classList.toggle('ecm', ecm.has(i));
  });
  if (S.follow && vm.has(S.sel)) map.panTo(vm.get(S.sel).marker.getLatLng(), { animate: true, duration: 0.2 });
}

function updateRoutes(rs) {
  for (const [id, pts] of Object.entries(rs || {})) {
    if (routes.has(id)) routes.get(id).setLatLngs(pts);
    else routes.set(id, L.polyline(pts, { className: 'route', interactive: false }).addTo(routeLayer));
  }
  for (const [id, line] of routes) {
    if (!rs || !(id in rs)) { routeLayer.removeLayer(line); routes.delete(id); }
  }
}

// ── side panel ───────────────────────────────────────────────────────────
function renderKpis(k) {
  const em = k.emapt || {};
  const val = (key) => (key === 'emapt_p90' ? em.p90 : k[key]);
  $('#kpis').innerHTML = KPIS.map(([key, label, unit, d]) => {
    const v = val(key);
    const title = key === 'emapt_p90' && em.p90 != null
      ? `EMAPT for ${em.ev} in ${em.rsu}: p50 ${em.p50} · p90 ${em.p90} · p100 ${em.p100} ms (${em.n} helpers)` : label;
    return `<div class="kpi${v == null ? ' idle' : ''}" title="${esc(title)}"><div class="l">${label}</div>` +
      `<div class="v">${num(v, d)}${v != null && unit ? `<small>${unit}</small>` : ''}</div></div>`;
  }).join('');
}

function renderCounters(s) {
  const c = s.counters || {};
  const k = s.kpi || {};
  const items = [['Dispatches', c.dispatches], ['ECM activations', c.ecm], ['Pre-emptions', c.preemptions],
    ['Yields', c.yields], ['Handovers', c.handovers], ['Telemetry msgs', k.msgs]];
  $('#counters').innerHTML = items.map(([l, v]) => `<span>${l} <b>${v ?? 0}</b></span>`).join('');
}

function pill(text, cls = text) { return `<span class="pill ${esc(cls)}">${esc(text)}</span>`; }

function renderVehicles(s) {
  const rows = [...s.vehicles].sort((a, b) => (b.siren - a.siren) || (KIND.indexOf(b.kind) > 0) - (KIND.indexOf(a.kind) > 0) || a.id.localeCompare(b.id));
  $('#tab-vehicles').innerHTML = `<table><thead><tr><th>Vehicle</th><th>Status</th><th>km/h</th><th>RSU</th><th>Policy</th>` +
    `<th title="Round-trip time (ms)">RTT</th><th title="Battery (%)">Batt</th></tr></thead><tbody>` +
    rows.map((v) => `<tr class="click${v.id === S.sel ? ' sel' : ''}" data-id="${esc(v.id)}"><td>${esc(v.id)}</td>` +
      `<td>${pill(v.status)}</td><td>${num(v.speed, 0)}</td><td>${esc(v.rsu || '—')}</td><td>${pill(v.policy)}</td>` +
      `<td>${num(v.net && v.net.rtt)}</td><td>${num(v.battery, 0)}</td></tr>`).join('') + '</tbody></table>';
}

function renderRsus(s) {
  $('#tab-rsus').innerHTML = `<table><thead><tr><th>RSU</th><th>Veh</th><th>EVs</th><th>ECM</th><th>Util</th>` +
    `<th>Load Mbit/s</th><th>Signal</th><th>Pre-empt</th></tr></thead><tbody>` +
    s.rsus.map((r) => `<tr><td>${esc(r.id)}</td><td>${r.n}</td><td>${esc(r.evs.join(', ') || '—')}</td>` +
      `<td>${r.ecm ? pill(`ON ${num(r.ecm_s, 0)}s`, 'on') : '—'}</td><td>${r.util != null ? num(r.util * 100, 0) + '%' : '—'}</td>` +
      `<td>${num(r.load)}</td><td>${r.signal ? esc(r.signal[0] + ' · ph ' + r.signal[1]) : '—'}</td>` +
      `<td>${esc(r.preempt || '—')}</td></tr>`).join('') + '</tbody></table>';
}

function renderMissions(s) {
  if (!s.missions.length) { $('#tab-missions').innerHTML = '<p class="muted">No missions yet. Press “Dispatch EV”.</p>'; return; }
  $('#tab-missions').innerHTML = `<table><thead><tr><th>#</th><th>Vehicle</th><th>Destination</th><th>Scenario</th>` +
    `<th>Time</th><th>km</th><th>km/h</th><th>Red</th><th>Stops</th></tr></thead><tbody>` +
    s.missions.map((m) => `<tr class="${m.active ? 'active' : ''}"><td>${m.n}</td><td>${esc(m.id)}</td>` +
      `<td title="${esc(m.dest)}">${esc(m.dest.length > 26 ? m.dest.slice(0, 25) + '…' : m.dest)}</td><td>${esc(m.scenario)}</td>` +
      `<td>${fmtT(m.time_s)}</td><td>${num(m.dist_m / 1000, 2)}</td><td>${m.active ? 'en route' : num(m.kmh)}</td>` +
      `<td>${m.red}</td><td>${m.stops}</td></tr>`).join('') + '</tbody></table>';
}

function renderController(s) {
  const c = s.controller || {};
  const flows = c.flows || [];
  $('#tab-controller').innerHTML =
    `<p class="muted">${c.modeled ? 'Modelled controller (sim mode)' : 'Ryu controller'} · ${c.datapaths ?? 0} datapaths · ${flows.length} priority flows</p>` +
    (flows.length ? `<table><thead><tr><th>Match</th><th>Prio</th><th>DP</th><th>Note</th></tr></thead><tbody>` +
      flows.map((f) => `<tr><td>${esc(f.match)}</td><td>${f.prio}</td><td>${esc(f.dp)}</td><td>${esc(f.note)}</td></tr>`).join('') +
      '</tbody></table>' : '<p class="muted">No priority flows (baseline scenario).</p>') +
    `<ul class="log">${(c.events || []).map((e) => `<li class="ecm"><time>${fmtT(e.t)}</time>${esc(e.msg)}</li>`).join('')}</ul>`;
}

const TABS = { vehicles: renderVehicles, rsus: renderRsus, missions: renderMissions, controller: renderController };
function renderTab() { if (S.stats && TABS[S.tab]) TABS[S.tab](S.stats); }

function onEvent(e) {
  const li = document.createElement('li');
  li.className = e.lvl;
  li.innerHTML = `<time>${fmtT(e.t)}</time><span>${esc(e.msg)}</span>`;
  const log = $('#log');
  log.prepend(li);
  while (log.children.length > 300) log.lastChild.remove();
}

// ── selected vehicle ─────────────────────────────────────────────────────
function select(id) {
  S.sel = S.sel === id ? null : id;
  S.follow = false;
  for (const [vid, m] of vm) {
    m.el.classList.toggle('sel', vid === S.sel);
    m.cls = m.el.className;
  }
  renderSel();
  renderTab();
}

function renderSel() {
  const card = $('#selCard');
  const v = S.sel && S.stats && S.stats.vehicles.find((x) => x.id === S.sel);
  if (!v) { card.hidden = true; return; }
  const n = v.net || {};
  const ev = v.kind !== 'car';
  card.hidden = false;
  card.innerHTML = `<div class="sel-h"><h2>${esc(v.id)} <span class="muted">· ${esc(v.kind)} · ${esc(v.node)}</span></h2>` +
    `<button data-op="close" title="Close">✕</button></div><div class="sel-grid">` +
    [['Status', pill(v.status)], ['Speed', `${num(v.speed, 0)} km/h`], ['Road', esc(v.road || '—')],
      ['RSU', esc(v.rsu || 'no coverage')], ['Policy', pill(v.policy)], ['Battery', `${num(v.battery, 0)} %`],
      ['RTT', `${num(n.rtt)} ms`], ['Throughput', `${num(n.tput, 2)} Mbit/s`], ['Distance', `${num(v.dist_km, 2)} km`]]
      .map(([l, x]) => `<div><span>${l}</span>${x}</div>`).join('') + '</div><div class="sel-actions">' +
    `<button data-op="follow">${S.follow ? 'Stop following' : 'Follow'}</button>` +
    (ev && !v.siren && v.status !== 'parked' ? `<button data-op="dispatch" class="primary">Dispatch</button>` : '') +
    (v.siren ? `<button data-op="recall">Recall</button>` : '') +
    `<button data-op="engine">${v.status === 'parked' ? 'Engine on' : 'Park'}</button></div>`;
}

$('#selCard').addEventListener('click', (e) => {
  const op = e.target.closest('button')?.dataset.op;
  if (!op || !S.sel) return;
  if (op === 'close') select(S.sel);
  else if (op === 'follow') { S.follow = !S.follow; renderSel(); }
  else cmd(op, { id: S.sel });
});

// ── latency chart (EV vs civilian, one shared ms axis) ───────────────────
function niceTop(max) {
  max = Math.max(max, 10);
  const raw = max / 4, mag = 10 ** Math.floor(Math.log10(raw)), r = raw / mag;
  const step = (r <= 1 ? 1 : r <= 2 ? 2 : r <= 5 ? 5 : 10) * mag;
  return [Math.ceil(max / step) * step, step];
}

function drawChart() {
  const box = $('#latChart');
  const W = box.clientWidth, H = box.clientHeight, P = { l: 34, r: 58, t: 8, b: 18 };
  const data = S.lat;
  if (data.length < 2) {
    box.innerHTML = `<svg><text class="empty" x="${W / 2}" y="${H / 2}" text-anchor="middle">Waiting for data…</text></svg>`;
    return;
  }
  const t0 = data[0].t, t1 = data[data.length - 1].t || 1;
  const [top, step] = niceTop(Math.max(...data.map((d) => Math.max(d.em ?? 0, d.be ?? 0))));
  const x = (t) => P.l + ((t - t0) / Math.max(t1 - t0, 1e-6)) * (W - P.l - P.r);
  const y = (v) => P.t + (1 - v / top) * (H - P.t - P.b);
  let svg = '';
  for (let v = 0; v <= top + 1e-9; v += step) {
    svg += `<line class="${v ? 'gridline' : 'baseline'}" x1="${P.l}" x2="${W - P.r}" y1="${y(v)}" y2="${y(v)}"/>` +
      `<text class="tick" x="${P.l - 5}" y="${y(v) + 3}" text-anchor="end">${v}</text>`;
  }
  [t0, (t0 + t1) / 2, t1].forEach((t, i) => {
    svg += `<text class="tick" x="${x(t)}" y="${H - 4}" text-anchor="${['start', 'middle', 'end'][i]}">${fmtT(t)}</text>`;
  });
  const labels = [];
  for (const [key, cls, name] of [['em', 's1', 'EV'], ['be', 's2', 'BE']]) {
    let d = '', pen = false, lastPt = null;
    for (const p of data) {
      if (p[key] == null) { pen = false; continue; }
      d += `${pen ? 'L' : 'M'}${x(p.t).toFixed(1)},${y(p[key]).toFixed(1)}`;
      pen = true;
      lastPt = p;
    }
    if (!d) continue;
    svg += `<path class="ln ${cls}" d="${d}"/>`;
    if (lastPt === data[data.length - 1]) {
      svg += `<circle class="end ${cls}" cx="${x(lastPt.t)}" cy="${y(lastPt[key])}" r="4"/>`;
      labels.push({ y: y(lastPt[key]), text: `${name} ${num(lastPt[key], 0)}` });
    }
  }
  if (labels.length === 2 && Math.abs(labels[0].y - labels[1].y) < 12) {
    const [a, b] = labels[0].y < labels[1].y ? labels : [labels[1], labels[0]];
    const mid = (a.y + b.y) / 2;
    a.y = mid - 6; b.y = mid + 6;
  }
  for (const l of labels) svg += `<text class="dlabel" x="${W - P.r + 8}" y="${l.y + 3}">${esc(l.text)}</text>`;
  if (S.hoverX != null) {
    const i = Math.round(((S.hoverX - P.l) / (W - P.l - P.r)) * (data.length - 1));
    const p = data[Math.max(0, Math.min(data.length - 1, i))];
    svg += `<line class="cross" x1="${x(p.t)}" x2="${x(p.t)}" y1="${P.t}" y2="${H - P.b}"/>`;
    box._tip = { p, left: x(p.t) };
  } else box._tip = null;
  box.innerHTML = `<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="Emergency vs civilian latency over time">${svg}</svg>`;
  if (box._tip) {
    const { p, left } = box._tip;
    const tip = document.createElement('div');
    tip.className = 'tip';
    tip.innerHTML = `<b>${fmtT(p.t)}</b><br><i style="background:var(--series-1)"></i>EV ${num(p.em)} ms<br>` +
      `<i style="background:var(--series-2)"></i>BE ${num(p.be)} ms`;
    box.appendChild(tip);
    tip.style.top = '4px';
    tip.style.left = `${left + tip.offsetWidth + 12 > W ? left - tip.offsetWidth - 8 : left + 8}px`;
  }
}
$('#latChart').addEventListener('mousemove', (e) => { S.hoverX = e.offsetX; drawChart(); });
$('#latChart').addEventListener('mouseleave', () => { S.hoverX = null; drawChart(); });
window.addEventListener('resize', drawChart);

// ── controls ─────────────────────────────────────────────────────────────
function setScenario(sc) {
  document.querySelectorAll('#scenarioSeg button').forEach((b) => b.classList.toggle('on', b.dataset.scenario === sc));
}

function setMode(d) {
  const sim = d.mode === 'sim';
  document.querySelectorAll('.sim-only').forEach((el) => { el.hidden = !sim; });
  document.querySelectorAll('.dyn-only').forEach((el) => { el.hidden = !d.config.dynamic; });
  $('#modeBadge').textContent = sim ? 'SIM · modelled network' : 'EMU · Mininet-WiFi';
  $('#kpiNote').textContent = (d.modeled ? 'Modelled KPIs (analytical cell model).' : 'Measured on the emulated testbed.') +
    ' EV metrics appear while an emergency vehicle is on a mission.';
}

document.querySelectorAll('#scenarioSeg button').forEach((b) => b.addEventListener('click', () => cmd('scenario', { value: b.dataset.scenario })));
document.querySelectorAll('[data-add]').forEach((b) => b.addEventListener('click', () => cmd('add', { kind: b.dataset.add, n: 1 })));
$('#dispatchBtn').addEventListener('click', () => cmd('dispatch'));
$('#pauseBtn').addEventListener('click', () => cmd('pause'));
$('#speedSel').addEventListener('change', (e) => cmd('speed', { value: Number(e.target.value) }));
document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
  S.tab = b.dataset.tab;
  document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('on', x === b));
  document.querySelectorAll('.tabpane').forEach((p) => { p.hidden = p.id !== `tab-${S.tab}`; });
  renderTab();
}));
$('#tab-vehicles').addEventListener('click', (e) => {
  const id = e.target.closest('tr[data-id]')?.dataset.id;
  if (id) {
    select(id);
    if (S.sel && vm.has(id)) map.panTo(vm.get(id).marker.getLatLng());
  }
});

const dlg = $('#cfgDlg');
$('#cfgBtn').addEventListener('click', () => {
  const c = (S.init && S.init.config) || {};
  for (const el of $('#cfgForm').elements) if (el.name && c[el.name] !== undefined) el.value = c[el.name];
  dlg.showModal();
});
dlg.addEventListener('close', () => {
  if (dlg.returnValue !== 'ok') return;
  const config = Object.fromEntries(new FormData($('#cfgForm')));
  cmd('restart', { config });
});

// ── socket ───────────────────────────────────────────────────────────────
function banner(text) {
  const b = $('#linkBanner');
  b.hidden = !text;
  b.textContent = text || '';
}

sock.on('init', (d) => {
  S.init = d; S.stats = null; S.lat = []; S.sel = null; S.follow = false;
  $('#mapName').textContent = d.map.name;
  $('#log').innerHTML = '';
  setMode(d);
  setScenario(d.config.scenario);
  buildStatic(d);
  renderSel();
  drawChart();
});
sock.on('frame', onFrame);
sock.on('stats', (s) => {
  if (!S.init) return;
  S.stats = s;
  const k = s.kpi || {};
  if (S.lat.length && s.t < S.lat[S.lat.length - 1].t) S.lat = [];
  S.lat.push({ t: s.t, em: k.em_latency_ms ?? null, be: k.be_latency_ms ?? null });
  if (S.lat.length > CHART_WINDOW) S.lat.shift();
  $('#pauseBtn').textContent = s.paused ? 'Resume' : 'Pause';
  $('#speedSel').value = String(s.speed);
  renderKpis(k);
  renderCounters(s);
  updateRoutes(s.routes);
  renderTab();
  renderSel();
  drawChart();
});
sock.on('event', onEvent);
sock.on('link', (l) => banner(l.state === 'connected' ? '' : `Waiting for the emulation hub at ${l.hub}…`));
sock.on('connect', () => banner(''));
sock.on('disconnect', () => banner('Dashboard server disconnected, reconnecting…'));

setupMap();
drawChart();
