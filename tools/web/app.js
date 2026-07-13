/*
 * Copyright 2025 Pennsylvania State University
 * Applied Research Laboratory, Pennsylvania State University
 *
 * DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
 * This software was developed by the Department of the Navy, NAVSEA Unmanned and
 * Small Combatants. It is provided under the terms of use found in the LICENSE
 * file at the source code root directory.
 *
 * UMAA mission console front end. Consumes the mission_console REST + SSE API and
 * renders a live local-tangent-plane chart (SVG, immediate mode), vehicle readouts,
 * mission builder, and command status.
 */
'use strict';

/* ------------------------------------------------- state */

const state = {
  telemetry: null,        // latest /api/stream telemetry object
  mission: { active: false, waypoints: [], planned_path: [], status_history: [] },
  execStatus: null,
  origin: null,           // {lat0, lon0, mPerDegLat, mPerDegLon}
  trail: [],              // [[east, north], ...]
  mode: 'monitor',        // 'monitor' | 'edit'
  draft: [],              // MissionWaypoint drafts while editing
  draftPath: [],          // previewed Dubins route for the draft ([[lat, lon], ...])
  selected: -1,           // selected waypoint index (draft or mission)
  view: { cE: 0, cN: 0, pxPerM: 1.4, userPanned: false },
  lastFinal: null,        // last terminal command status
};

const $ = (id) => document.getElementById(id);
const chart = $('chart');
let lastEditorIdx = -1;  // waypoint whose values currently populate the editor inputs

/* ------------------------------------------------- projection helpers */

function setOrigin(latDeg, lonDeg) {
  state.origin = {
    lat0: latDeg,
    lon0: lonDeg,
    mPerDegLat: 111132.0,
    mPerDegLon: 111320.0 * Math.cos(latDeg * Math.PI / 180.0),
  };
}

function toLocal(latDeg, lonDeg) {
  const o = state.origin;
  return [(lonDeg - o.lon0) * o.mPerDegLon, (latDeg - o.lat0) * o.mPerDegLat];
}

function toGeo(east, north) {
  const o = state.origin;
  return [o.lat0 + north / o.mPerDegLat, o.lon0 + east / o.mPerDegLon];
}

function toScreen(east, north) {
  const { cE, cN, pxPerM } = state.view;
  return [chart.clientWidth / 2 + (east - cE) * pxPerM,
          chart.clientHeight / 2 - (north - cN) * pxPerM];
}

function toWorld(px, py) {
  const { cE, cN, pxPerM } = state.view;
  return [cE + (px - chart.clientWidth / 2) / pxPerM,
          cN - (py - chart.clientHeight / 2) / pxPerM];
}

/* ------------------------------------------------- formatting */

const fmt = {
  deg: (v, dp = 6) => v.toFixed(dp) + '°',
  m: (v) => (Math.abs(v) >= 1000 ? (v / 1000).toFixed(2) + ' km' : v.toFixed(1) + ' m'),
  mps: (v) => v.toFixed(2) + ' m/s',
  hdg: (rad) => {
    let d = rad * 180 / Math.PI;
    d = ((d % 360) + 360) % 360;
    return d.toFixed(1) + '°T';
  },
};

/* ------------------------------------------------- svg helpers */

const SVG_NS = 'http://www.w3.org/2000/svg';

function el(tag, attrs, parent) {
  const node = document.createElementNS(SVG_NS, tag);
  for (const [k, v] of Object.entries(attrs)) {
    node.setAttribute(k, v);
  }
  if (parent) parent.appendChild(node);
  return node;
}

function polyline(points, cls, parent) {
  if (points.length < 2) return null;
  return el('polyline', {
    points: points.map(([e, n]) => toScreen(e, n).map((v) => v.toFixed(1)).join(',')).join(' '),
    class: cls,
  }, parent);
}

/* ------------------------------------------------- grid */

function gridStep() {
  // Aim for ~90 px between lines using a 1/2/5 ladder.
  const targetM = 90 / state.view.pxPerM;
  const pow = Math.pow(10, Math.floor(Math.log10(targetM)));
  for (const mult of [1, 2, 5, 10]) {
    if (pow * mult >= targetM) return pow * mult;
  }
  return pow * 10;
}

function drawGrid(root) {
  const step = gridStep();
  const w = chart.clientWidth;
  const h = chart.clientHeight;
  const [minE, maxN] = toWorld(0, 0);
  const [maxE, minN] = toWorld(w, h);
  const g = el('g', {}, root);
  for (let e = Math.ceil(minE / step) * step; e <= maxE; e += step) {
    const [x] = toScreen(e, 0);
    el('line', { x1: x, y1: 0, x2: x, y2: h, class: 'grid-line' }, g);
    el('text', { x: x + 3, y: h - 6, class: 'grid-label' }, g)
      .textContent = fmtGridLabel(e, 'E');
  }
  for (let n = Math.ceil(minN / step) * step; n <= maxN; n += step) {
    const [, y] = toScreen(0, n);
    el('line', { x1: 0, y1: y, x2: w, y2: y, class: 'grid-line' }, g);
    el('text', { x: 4, y: y - 4, class: 'grid-label' }, g)
      .textContent = fmtGridLabel(n, 'N');
  }
  $('scale-readout').textContent = 'grid ' + fmt.m(step);
}

function fmtGridLabel(v, axis) {
  const m = Math.abs(v) >= 1000 ? (v / 1000).toFixed(1) + 'k' : v.toFixed(0);
  return m + ' ' + axis;
}

/* ------------------------------------------------- waypoint marks */

function waypointStates() {
  // done / current / pending per mission waypoint, from the execution status.
  const total = state.mission.waypoints.length;
  let done = 0;
  if (state.execStatus && state.mission.active) {
    done = Math.max(0, total - state.execStatus.waypoints_remaining);
  } else if (!state.mission.active && state.lastFinal === 'COMPLETED') {
    done = total;
  }
  return state.mission.waypoints.map((_, i) => {
    if (i < done) return 'done';
    if (i === done && state.mission.active) return 'current';
    return 'pending';
  });
}

function drawWaypoint(root, wp, i, cls, extraCls) {
  const [e, n] = toLocal(wp.lat_deg, wp.lon_deg);
  const [x, y] = toScreen(e, n);
  const g = el('g', { class: `wp ${cls} ${extraCls}`.trim(), 'data-idx': i }, root);
  const rPx = Math.max(4, (wp.capture_radius_m || 2.5) * state.view.pxPerM);
  el('circle', { cx: x, cy: y, r: rPx, class: 'wp-capture' }, g);
  if (wp.arrival_yaw_rad !== undefined && wp.arrival_yaw_rad !== null) {
    const az = wp.arrival_yaw_rad;
    const len = rPx + 14;
    const dx = Math.sin(az);
    const dy = -Math.cos(az);
    const tipX = x + dx * len;
    const tipY = y + dy * len;
    el('path', {
      d: `M ${x + dx * rPx} ${y + dy * rPx} L ${tipX} ${tipY}` +
         ` M ${tipX - 4 * dy - 4 * dx} ${tipY + 4 * dx - 4 * dy} L ${tipX} ${tipY}` +
         ` L ${tipX + 4 * dy - 4 * dx} ${tipY - 4 * dx - 4 * dy}`,
      class: 'wp-arrow',
    }, g);
  }
  el('circle', { cx: x, cy: y, r: 6, class: 'wp-dot', 'data-idx': i }, g);
  el('text', { x: x, y: y - 10, class: 'wp-index' }, g).textContent = String(i + 1);
  return g;
}

/* ------------------------------------------------- main render */

function render() {
  chart.replaceChildren();
  const root = el('g', {}, chart);

  if (!state.origin) {
    el('text', {
      x: chart.clientWidth / 2, y: chart.clientHeight / 2, class: 'awaiting',
    }, root).textContent = 'Awaiting vehicle telemetry…';
    renderSidebar();
    return;
  }

  drawGrid(root);

  // Planned route for the active/last mission, then the draft preview when editing.
  polyline(state.mission.planned_path.map(([la, lo]) => toLocal(la, lo)), 'planned-path', root);
  if (state.mode === 'edit') {
    polyline(state.draftPath.map(([la, lo]) => toLocal(la, lo)), 'draft-path', root);
  }

  // Vehicle trail.
  polyline(state.trail, 'trail', root);

  // Mission waypoints (fade the finished ones, pulse the current one).
  const wpStates = waypointStates();
  state.mission.waypoints.forEach((wp, i) => {
    const cls = wpStates[i] === 'current' ? 'wp-current' : 'wp-mission';
    const extra = (wpStates[i] === 'done' ? 'wp-done ' : '') +
                  (state.mode === 'monitor' && state.selected === i ? 'wp-selected' : '');
    drawWaypoint(root, wp, i, cls, extra);
  });

  // Draft waypoints on top while editing.
  if (state.mode === 'edit') {
    state.draft.forEach((wp, i) => {
      drawWaypoint(root, wp, i, 'wp-draft', state.selected === i ? 'wp-selected' : '');
    });
  }

  // Animated active leg: vehicle -> current waypoint.
  const t = state.telemetry;
  if (t && state.mission.active) {
    const cur = wpStates.indexOf('current');
    if (cur >= 0) {
      const [ve, vn] = toLocal(t.lat_deg, t.lon_deg);
      const [we, wn] = toLocal(state.mission.waypoints[cur].lat_deg,
                               state.mission.waypoints[cur].lon_deg);
      const [x1, y1] = toScreen(ve, vn);
      const [x2, y2] = toScreen(we, wn);
      el('line', { x1, y1, x2, y2, class: 'active-leg' }, root);
    }
  }

  // Vehicle icon (triangle pointing along heading).
  if (t) {
    const [e, n] = toLocal(t.lat_deg, t.lon_deg);
    const [x, y] = toScreen(e, n);
    const hdgDeg = (t.yaw_rad || 0) * 180 / Math.PI;
    el('path', {
      d: 'M 0 -11 L 7 9 L 0 5 L -7 9 Z',
      class: 'vehicle-icon',
      transform: `translate(${x.toFixed(1)} ${y.toFixed(1)}) rotate(${hdgDeg.toFixed(1)})`,
    }, root);
  }

  renderSidebar();
}

/* ------------------------------------------------- sidebar */

function setText(id, text) { $(id).textContent = text; }

function renderSidebar() {
  const t = state.telemetry;
  setText('ro-lat', t ? fmt.deg(t.lat_deg) : '—');
  setText('ro-lon', t ? fmt.deg(t.lon_deg) : '—');
  setText('ro-hdg', t ? fmt.hdg(t.yaw_rad) : '—');
  setText('ro-sog', t && t.sog_mps !== undefined ? fmt.mps(t.sog_mps) : '—');
  setText('ro-depth', t && t.depth_m !== undefined ? fmt.m(t.depth_m) : '—');
  setText('ro-asf', t && t.alt_asf_m !== undefined ? fmt.m(t.alt_asf_m) : '—');
  setText('ro-age', t ? t.age_s.toFixed(1) + ' s' : '—');

  const ex = state.mission.active ? state.execStatus : null;
  setText('ro-dist-wp', ex ? fmt.m(ex.distance_to_waypoint_m) : '—');
  setText('ro-dist-rem', ex ? fmt.m(ex.distance_remaining_m) : '—');
  setText('ro-xte', ex && ex.cross_track_error_m !== undefined
      ? fmt.m(ex.cross_track_error_m) : '—');
  setText('ro-wp-rem', ex ? String(ex.waypoints_remaining) : '—');

  renderWaypointList();
  renderStatusLog();
  renderExecuteDock();
}

function describeWaypoint(wp) {
  const bits = [`${(wp.speed_mps ?? 3).toFixed(1)} m/s`, `±${(wp.capture_radius_m ?? 2.5).toFixed(1)} m`];
  if (wp.arrival_yaw_rad !== undefined && wp.arrival_yaw_rad !== null) {
    bits.push(fmt.hdg(wp.arrival_yaw_rad));
  }
  if (wp.elev_value_m !== undefined && wp.elev_value_m !== null) {
    bits.push(`${wp.elev_value_m.toFixed(0)} m ${wp.elev_frame === 'asf' ? 'ASF' : 'depth'}`);
  }
  return bits.join(' · ');
}

let lastListSignature = '';

function renderWaypointList() {
  const list = $('wp-list');
  const editing = state.mode === 'edit';
  const wps = editing ? state.draft : state.mission.waypoints;
  const states = editing ? wps.map(() => 'draft') : waypointStates();
  // Rebuild the list only when its content actually changes: constant 5 Hz DOM churn
  // makes in-flight clicks land on detached nodes.
  const signature = JSON.stringify([editing, state.selected, states, wps.map(describeWaypoint)]);
  if (signature !== lastListSignature) {
    lastListSignature = signature;
    rebuildWaypointList(list, editing, wps, states);
  }
  renderWaypointEditor(editing);
}

function rebuildWaypointList(list, editing, wps, states) {
  list.replaceChildren();
  if (!wps.length) {
    const li = document.createElement('li');
    li.className = 'empty';
    li.textContent = editing ? 'Click the chart to add waypoints.' : 'No mission loaded.';
    list.appendChild(li);
  }
  wps.forEach((wp, i) => {
    const li = document.createElement('li');
    li.className = `state-${states[i]}` + (state.selected === i ? ' selected' : '');
    li.dataset.idx = i;
    li.innerHTML = `<span class="wp-state"></span><span class="wp-desc">WP${i + 1} — ` +
                   `${describeWaypoint(wp)}</span>`;
    list.appendChild(li);
  });
}

function renderWaypointEditor(editing) {
  const ed = $('wp-editor');
  if (editing && state.selected >= 0 && state.selected < state.draft.length) {
    ed.hidden = false;
    if (state.selected !== lastEditorIdx && ed.contains(document.activeElement)) {
      document.activeElement.blur();  // selection moved: stop editing the old waypoint
    }
    lastEditorIdx = state.selected;
    const wp = state.draft[state.selected];
    // Never stomp the value of the input the user is typing in; checkbox/disabled state
    // always mirrors the model (a checkbox toggle IS a model change).
    const setVal = (inp, v) => { if (document.activeElement !== inp) inp.value = v; };
    $('wp-editor-title').textContent = `Waypoint ${state.selected + 1}`;
    setVal($('ed-speed'), wp.speed_mps ?? 3.0);
    setVal($('ed-capture'), wp.capture_radius_m ?? 2.5);
    const hasYaw = wp.arrival_yaw_rad !== undefined && wp.arrival_yaw_rad !== null;
    $('ed-yaw-on').checked = hasYaw;
    $('ed-yaw').disabled = !hasYaw;
    setVal($('ed-yaw'), hasYaw ? (wp.arrival_yaw_rad * 180 / Math.PI).toFixed(0) : '');
    const hasElev = wp.elev_value_m !== undefined && wp.elev_value_m !== null;
    $('ed-elev-on').checked = hasElev;
    $('ed-elev').disabled = !hasElev;
    $('ed-frame').disabled = !hasElev;
    setVal($('ed-elev'), hasElev ? wp.elev_value_m : '');
    if (document.activeElement !== $('ed-frame')) $('ed-frame').value = wp.elev_frame || 'depth';
  } else {
    ed.hidden = true;
    lastEditorIdx = -1;
  }
}

function renderStatusLog() {
  const log = $('status-log');
  log.replaceChildren();
  const hist = state.mission.status_history || [];
  if (!hist.length) {
    const li = document.createElement('li');
    li.className = 'empty';
    li.textContent = 'No command activity.';
    log.appendChild(li);
    return;
  }
  for (const s of [...hist].reverse()) {
    const li = document.createElement('li');
    const reason = s.reason && s.reason !== 'SUCCEEDED' && s.reason !== s.status
        ? ` (${s.reason})` : '';
    li.innerHTML = `<span class="st ${s.status}">${s.status}${reason}</span>` +
                   `<span class="msg">${s.message || ''}</span>`;
    log.appendChild(li);
  }
}

function renderExecuteDock() {
  const btn = $('btn-execute');
  const chip = $('cmd-status-chip');
  const ack = $('ack-chip');
  const hist = state.mission.status_history || [];
  const last = hist.length ? hist[hist.length - 1] : null;

  if (state.mission.active) {
    btn.textContent = 'CANCEL';
    btn.classList.add('cancel');
    btn.disabled = false;
  } else {
    btn.textContent = 'EXECUTE';
    btn.classList.remove('cancel');
    btn.disabled = !(state.mode === 'edit' && state.draft.length > 0);
  }

  if (last) {
    chip.textContent = last.status + (last.reason && last.reason !== 'SUCCEEDED' &&
        last.reason !== last.status ? ' · ' + last.reason : '');
    chip.className = 'chip ' + ({
      ISSUED: 'chip-pending', COMMANDED: 'chip-pending', EXECUTING: 'chip-good',
      COMPLETED: 'chip-good', CANCELED: 'chip-serious', FAILED: 'chip-critical',
    }[last.status] || 'chip-idle');
  } else {
    chip.textContent = state.mode === 'edit'
        ? `DRAFT · ${state.draft.length} WP` : 'NO MISSION';
    chip.className = 'chip chip-idle';
  }

  if (state.mission.active || (state.mission.session && !last)) {
    ack.hidden = false;
    if (state.mission.ack_received) {
      ack.textContent = 'ACK ✓';
      ack.className = 'chip chip-good';
    } else {
      ack.textContent = 'AWAITING ACK';
      ack.className = 'chip chip-pending';
    }
  } else {
    ack.hidden = !state.mission.session;
    if (state.mission.session) {
      ack.textContent = state.mission.ack_received ? 'ACK ✓' : 'NO ACK';
      ack.className = 'chip ' + (state.mission.ack_received ? 'chip-good' : 'chip-idle');
    }
  }
}

/* ------------------------------------------------- waypoint selection & popup */

function selectWaypoint(i) {
  // Monitor mode toggles the info popup; edit mode always selects (re-click keeps it).
  if (state.mode === 'monitor') {
    state.selected = state.selected === i ? -1 : i;
    showWaypointPopup(state.selected);
  } else {
    state.selected = i;
  }
  render();
}

function showWaypointPopup(i) {
  const popup = $('wp-popup');
  if (i < 0 || i >= state.mission.waypoints.length) {
    popup.hidden = true;
    return;
  }
  const wp = state.mission.waypoints[i];
  const wpState = waypointStates()[i];
  const hist = state.mission.status_history || [];
  const last = hist.length ? hist[hist.length - 1] : null;
  const rows = [
    ['State', wpState.toUpperCase()],
    ['Position', `${wp.lat_deg.toFixed(6)}, ${wp.lon_deg.toFixed(6)}`],
    ['Speed', fmt.mps(wp.speed_mps ?? 3)],
    ['Capture', fmt.m(wp.capture_radius_m ?? 2.5)],
  ];
  if (wp.arrival_yaw_rad != null) rows.push(['Arrival hdg', fmt.hdg(wp.arrival_yaw_rad)]);
  if (wp.elev_value_m != null) {
    rows.push(['Elevation', `${wp.elev_value_m} m ${wp.elev_frame === 'asf' ? 'ASF' : 'depth'}`]);
  }
  if (last) rows.push(['Command', last.status]);
  if (state.mission.ack_received) rows.push(['Ack', 'received']);
  if (wpState === 'current' && state.execStatus) {
    const ex = state.execStatus;
    rows.push(['Distance', fmt.m(ex.distance_to_waypoint_m)]);
    rows.push(['Position ok', ex.position_achieved ? 'yes' : 'no']);
    if (ex.attitude_achieved !== undefined) {
      rows.push(['Attitude ok', ex.attitude_achieved ? 'yes' : 'no']);
    }
    rows.push(['Elevation ok', ex.elevation_achieved ? 'yes' : 'no']);
  }
  popup.innerHTML = `<h3>Waypoint ${i + 1}</h3>` +
      rows.map(([k, v]) => `<div class="kv"><span>${k}</span><b>${v}</b></div>`).join('');
  const [e, n] = toLocal(wp.lat_deg, wp.lon_deg);
  const [x, y] = toScreen(e, n);
  popup.style.left = Math.min(x + 16, chart.clientWidth - 240) + 'px';
  popup.style.top = Math.max(10, y - 30) + 'px';
  popup.hidden = false;
}

/* ------------------------------------------------- edit mode & API */

async function api(path, body) {
  const res = await fetch(path, body === undefined ? {} : {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || res.statusText);
  return data;
}

let previewTimer = null;
function schedulePreview() {
  clearTimeout(previewTimer);
  if (state.draft.length < 1) {
    state.draftPath = [];
    return;
  }
  previewTimer = setTimeout(async () => {
    try {
      const data = await api('/api/preview', { waypoints: state.draft });
      state.draftPath = data.path || [];
      render();
    } catch (e) { /* preview is best-effort */ }
  }, 250);
}

function enterEditMode() {
  state.mode = 'edit';
  state.draft = [];
  state.draftPath = [];
  state.selected = -1;
  $('wp-popup').hidden = true;
  $('btn-new-mission').hidden = true;
  $('btn-clear').hidden = false;
  $('btn-done-edit').hidden = false;
  $('edit-hint').hidden = false;
  render();
}

function exitEditMode() {
  state.mode = 'monitor';
  state.draft = [];
  state.draftPath = [];
  state.selected = -1;
  $('btn-new-mission').hidden = false;
  $('btn-clear').hidden = true;
  $('btn-done-edit').hidden = true;
  $('edit-hint').hidden = true;
  render();
}

function addDraftWaypoint(east, north) {
  const [lat, lon] = toGeo(east, north);
  state.draft.push({ lat_deg: lat, lon_deg: lon, speed_mps: 3.0, capture_radius_m: 2.5 });
  state.selected = state.draft.length - 1;
  schedulePreview();
  render();
}

async function onExecute() {
  const btn = $('btn-execute');
  if (state.mission.active) {
    btn.disabled = true;
    try { await api('/api/mission/cancel', {}); } catch (e) { console.error(e); }
    btn.disabled = false;
    return;
  }
  if (state.mode !== 'edit' || !state.draft.length) return;
  btn.disabled = true;
  try {
    await api('/api/mission', { waypoints: state.draft });
    exitEditMode();
  } catch (e) {
    alert('Failed to start mission: ' + e.message);
    btn.disabled = false;
  }
}

/* ------------------------------------------------- editor bindings */

function bindEditor() {
  const upd = (fn) => {
    if (state.selected < 0 || state.selected >= state.draft.length) return;
    fn(state.draft[state.selected]);
    schedulePreview();
    render();
  };
  $('ed-speed').addEventListener('change', (ev) => upd((wp) => {
    wp.speed_mps = Math.max(0.1, parseFloat(ev.target.value) || 3.0);
  }));
  $('ed-capture').addEventListener('change', (ev) => upd((wp) => {
    wp.capture_radius_m = Math.max(0.5, parseFloat(ev.target.value) || 2.5);
  }));
  $('ed-yaw-on').addEventListener('change', (ev) => upd((wp) => {
    if (ev.target.checked) {
      wp.arrival_yaw_rad = (parseFloat($('ed-yaw').value) || 0) * Math.PI / 180;
    } else {
      delete wp.arrival_yaw_rad;
    }
  }));
  $('ed-yaw').addEventListener('change', (ev) => upd((wp) => {
    wp.arrival_yaw_rad = (parseFloat(ev.target.value) || 0) * Math.PI / 180;
  }));
  $('ed-elev-on').addEventListener('change', (ev) => upd((wp) => {
    if (ev.target.checked) {
      wp.elev_value_m = parseFloat($('ed-elev').value) || 5.0;
      wp.elev_frame = $('ed-frame').value;
    } else {
      delete wp.elev_value_m;
      delete wp.elev_frame;
    }
  }));
  $('ed-elev').addEventListener('change', (ev) => upd((wp) => {
    wp.elev_value_m = parseFloat(ev.target.value) || 0;
  }));
  $('ed-frame').addEventListener('change', (ev) => upd((wp) => {
    wp.elev_frame = ev.target.value;
  }));
  $('ed-delete').addEventListener('click', () => {
    if (state.selected < 0) return;
    state.draft.splice(state.selected, 1);
    state.selected = -1;
    schedulePreview();
    render();
  });
}

/* ------------------------------------------------- chart interactions */

function bindChart() {
  let dragging = false;
  let moved = false;
  let lastX = 0;
  let lastY = 0;

  chart.addEventListener('mousedown', (ev) => {
    dragging = true;
    moved = false;
    lastX = ev.clientX;
    lastY = ev.clientY;
  });
  window.addEventListener('mousemove', (ev) => {
    const rect = chart.getBoundingClientRect();
    if (state.origin) {
      const [e, n] = toWorld(ev.clientX - rect.left, ev.clientY - rect.top);
      const [lat, lon] = toGeo(e, n);
      $('cursor-pos').textContent =
          `${lat.toFixed(6)}, ${lon.toFixed(6)}  ·  ${e.toFixed(0)} E ${n.toFixed(0)} N`;
    }
    if (!dragging) return;
    const dx = ev.clientX - lastX;
    const dy = ev.clientY - lastY;
    if (Math.abs(dx) + Math.abs(dy) > 3) {
      moved = true;
      chart.classList.add('panning');
      state.view.cE -= dx / state.view.pxPerM;
      state.view.cN += dy / state.view.pxPerM;
      state.view.userPanned = true;
      $('follow-vehicle').checked = false;
      lastX = ev.clientX;
      lastY = ev.clientY;
      render();
    }
  });
  window.addEventListener('mouseup', (ev) => {
    if (!dragging) return;
    dragging = false;
    chart.classList.remove('panning');
    if (moved || !state.origin) return;
    const rect = chart.getBoundingClientRect();
    const target = ev.target.closest ? ev.target.closest('.wp') : null;
    if (target && target.dataset.idx !== undefined) {
      selectWaypoint(parseInt(target.dataset.idx, 10));
      return;
    }
    if (state.mode === 'edit' && ev.target.closest('#chart')) {
      const [e, n] = toWorld(ev.clientX - rect.left, ev.clientY - rect.top);
      addDraftWaypoint(e, n);
    } else if (state.mode === 'monitor') {
      state.selected = -1;
      $('wp-popup').hidden = true;
      render();
    }
  });
  chart.addEventListener('wheel', (ev) => {
    ev.preventDefault();
    if (!state.origin) return;
    const rect = chart.getBoundingClientRect();
    const [e0, n0] = toWorld(ev.clientX - rect.left, ev.clientY - rect.top);
    const factor = ev.deltaY < 0 ? 1.18 : 1 / 1.18;
    state.view.pxPerM = Math.min(40, Math.max(0.02, state.view.pxPerM * factor));
    // Keep the point under the cursor fixed.
    const [e1, n1] = toWorld(ev.clientX - rect.left, ev.clientY - rect.top);
    state.view.cE += e0 - e1;
    state.view.cN += n0 - n1;
    render();
  }, { passive: false });
}

/* ------------------------------------------------- SSE feed */

function applySnapshot(snap) {
  const t = snap.telemetry && snap.telemetry.lat_deg !== undefined ? snap.telemetry : null;
  state.mission = snap.mission || state.mission;
  state.execStatus = snap.exec_status || null;

  const hist = state.mission.status_history || [];
  for (const s of hist) {
    if (['COMPLETED', 'CANCELED', 'FAILED'].includes(s.status)) state.lastFinal = s.status;
  }

  if (t) {
    if (!state.origin) {
      setOrigin(t.lat_deg, t.lon_deg);
      state.view.cE = 0;
      state.view.cN = 0;
    }
    state.telemetry = t;
    const [e, n] = toLocal(t.lat_deg, t.lon_deg);
    const last = state.trail[state.trail.length - 1];
    if (!last || Math.hypot(e - last[0], n - last[1]) > 0.5) {
      state.trail.push([e, n]);
      if (state.trail.length > 5000) state.trail.splice(0, 1000);
    }
    if ($('follow-vehicle').checked && !dragActive()) {
      state.view.cE = e;
      state.view.cN = n;
    }
  }
  if (state.mode === 'monitor' && state.selected >= 0) {
    showWaypointPopup(state.selected);
  }
  render();
}

function dragActive() { return chart.classList.contains('panning'); }

function connectStream() {
  const es = new EventSource('/api/stream');
  es.onmessage = (ev) => {
    try { applySnapshot(JSON.parse(ev.data)); } catch (e) { console.error(e); }
  };
  es.onerror = () => { /* EventSource auto-reconnects */ };
}

/* ------------------------------------------------- boot */

$('wp-list').addEventListener('click', (ev) => {
  const li = ev.target.closest('li[data-idx]');
  if (li) selectWaypoint(parseInt(li.dataset.idx, 10));
});
$('btn-new-mission').addEventListener('click', enterEditMode);
$('btn-clear').addEventListener('click', () => {
  state.draft = [];
  state.draftPath = [];
  state.selected = -1;
  render();
});
$('btn-done-edit').addEventListener('click', exitEditMode);
$('btn-execute').addEventListener('click', onExecute);
$('follow-vehicle').addEventListener('change', () => render());
window.addEventListener('resize', render);

bindEditor();
bindChart();
connectStream();
render();
