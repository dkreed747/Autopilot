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
  view: { cE: 0, cN: 0, pxPerM: 1.4, userPanned: false, rotRad: 0 },
  lastFinal: null,        // last terminal command status
  constraints: { enabled: false, items: [], active_ids: [], active_known: false },
  conSelected: null,      // selected constraint id (opens the editor)
  conDraft: null,         // local edit buffer for the selected constraint (see bindConstraintEditor)
  zoneDraft: null,        // { kind: 'keep_in'|'keep_out', points: [[e,n],...] } in zone-draw mode
  pendingActive: null,    // optimistic active-id set awaiting the autopilot's ack echo
  opMode: null,           // snapshot.operational_mode
  vector: null,           // snapshot.vector (the console's own vector session)
  traffic: { missions: [], vectors: [] },  // everything observed on the bus, classified
  platform: null,         // snapshot.platform (speed limits, sim floor depth)
  // Local draft for the vector editor (never re-rendered from server values).
  vecDraft: { heading_deg: 0, speed_mps: 1.5, elevOn: false, elev_value_m: 5,
              elev_frame: 'depth', timeoutOn: false, timeout_s: 30 },
  // WASD remote control: held keys, adjustable speed/elevation, link health.
  rc: { engaged: false, engagedAt: 0, keys: new Set(), speedMps: 1.0, elevOn: false,
        elevValueM: 5, elevFrame: 'depth', lastHeadingRad: null, timer: null, linkOk: true },
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

// The screen transform optionally rotates the world so the vehicle heading points up
// (rotRad = vehicle yaw in heading-up mode, 0 = classic north-up). u is the world offset
// along the heading (screen up), v the offset to its right (screen right); at rotRad = 0
// this degenerates to the plain translate+scale.
function toScreen(east, north) {
  const { cE, cN, pxPerM, rotRad } = state.view;
  const de = east - cE;
  const dn = north - cN;
  const u = de * Math.sin(rotRad) + dn * Math.cos(rotRad);
  const v = de * Math.cos(rotRad) - dn * Math.sin(rotRad);
  return [chart.clientWidth / 2 + v * pxPerM, chart.clientHeight / 2 - u * pxPerM];
}

function toWorld(px, py) {
  const { cE, cN, pxPerM, rotRad } = state.view;
  const v = (px - chart.clientWidth / 2) / pxPerM;
  const u = (chart.clientHeight / 2 - py) / pxPerM;
  return [cE + u * Math.sin(rotRad) + v * Math.cos(rotRad),
          cN + u * Math.cos(rotRad) - v * Math.sin(rotRad)];
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
  // World-space AABB of the four screen corners (they differ once the chart is rotated);
  // each grid line is a two-point segment so it survives rotation.
  const corners = [toWorld(0, 0), toWorld(w, 0), toWorld(0, h), toWorld(w, h)];
  const minE = Math.min(...corners.map((c) => c[0]));
  const maxE = Math.max(...corners.map((c) => c[0]));
  const minN = Math.min(...corners.map((c) => c[1]));
  const maxN = Math.max(...corners.map((c) => c[1]));
  const headingUp = Math.abs(state.view.rotRad) > 1e-6;
  const g = el('g', {}, root);
  for (let e = Math.ceil(minE / step) * step; e <= maxE; e += step) {
    const [x1, y1] = toScreen(e, minN);
    const [x2, y2] = toScreen(e, maxN);
    el('line', { x1, y1, x2, y2, class: 'grid-line' }, g);
    if (!headingUp) {
      el('text', { x: x1 + 3, y: h - 6, class: 'grid-label' }, g)
        .textContent = fmtGridLabel(e, 'E');
    }
  }
  for (let n = Math.ceil(minN / step) * step; n <= maxN; n += step) {
    const [x1, y1] = toScreen(minE, n);
    const [x2, y2] = toScreen(maxE, n);
    el('line', { x1, y1, x2, y2, class: 'grid-line' }, g);
    if (!headingUp) {
      el('text', { x: 4, y: y1 - 4, class: 'grid-label' }, g)
        .textContent = fmtGridLabel(n, 'N');
    }
  }
  if (headingUp) {
    // Per-line labels land mid-chart at arbitrary angles in heading-up mode; a compass
    // showing true north replaces them.
    const rotDeg = state.view.rotRad * 180 / Math.PI;
    const cg = el('g', { class: 'compass',
                         transform: `translate(${w - 42} 50) rotate(${(-rotDeg).toFixed(1)})` }, root);
    el('circle', { cx: 0, cy: 0, r: 18, class: 'compass-ring' }, cg);
    el('path', { d: 'M 0 -14 L 5 6 L 0 2 L -5 6 Z', class: 'compass-needle' }, cg);
    el('text', { x: 0, y: -22, class: 'compass-label' }, cg).textContent = 'N';
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
    const az = wp.arrival_yaw_rad - state.view.rotRad;  // compensate chart rotation
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

/* ------------------------------------------------- constraint zones */

function zonePointsAttr(latlonPoly) {
  return latlonPoly
      .map(([la, lo]) => toScreen(...toLocal(la, lo)).map((v) => v.toFixed(1)).join(','))
      .join(' ');
}

function drawZones(root) {
  const items = state.constraints.items || [];
  for (const c of items) {
    if (!c.polygon || c.polygon.length < 3) continue;
    const cls = ['zone', c.type === 'keep_in' ? 'zone-kia' : 'zone-koa'];
    if (!isActive(c.id)) cls.push('zone-inactive');
    if (c.state === false && isActive(c.id)) cls.push('zone-violated');
    if (state.conSelected === c.id) cls.push('zone-selected');
    el('polygon', {
      points: zonePointsAttr(c.polygon),
      class: cls.join(' '),
      'fill-rule': 'evenodd',
      'data-cid': c.id,
    }, root);
  }
  // In-progress zone draft (local-frame vertices).
  if (state.mode === 'zone-draw' && state.zoneDraft) {
    const pts = state.zoneDraft.points;
    const cls = 'zone-draft ' + (state.zoneDraft.kind === 'keep_in' ? 'zone-kia' : 'zone-koa');
    if (pts.length >= 2) {
      el('polyline', {
        points: pts.map(([e, n]) => toScreen(e, n).map((v) => v.toFixed(1)).join(',')).join(' '),
        class: cls,
      }, root);
    }
    pts.forEach(([e, n], i) => {
      const [x, y] = toScreen(e, n);
      el('circle', { cx: x, cy: y, r: i === 0 ? 6 : 4, class: 'zone-vertex', 'data-vidx': i }, root);
    });
  }
}

function isActive(id) {
  const ids = state.pendingActive !== null ? state.pendingActive : (state.constraints.active_ids || []);
  return ids.includes(id);
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

  drawZones(root);

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

  // External missions (onboard autonomy / remote operators), dashed and colored per source.
  for (const m of state.traffic.missions || []) {
    if (m.classification === 'own' || !m.active) continue;
    const pts = (m.waypoints || []).filter((w) => w.lat_deg !== undefined);
    if (!pts.length) continue;
    const cls = m.classification === 'local' ? 'wp-local' : 'wp-remote';
    polyline(pts.map((w) => toLocal(w.lat_deg, w.lon_deg)),
             `traffic-path traffic-${m.classification}`, root);
    pts.forEach((w, i) => drawWaypoint(root, w, i, cls, 'wp-traffic'));
  }

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

  // Active vector commands as heading arrows anchored at the vehicle.
  if (t) {
    const [ve, vn] = toLocal(t.lat_deg, t.lon_deg);
    const [vx, vy] = toScreen(ve, vn);
    for (const v of state.traffic.vectors || []) {
      if (!v.active || v.heading_deg === undefined) continue;
      const ang = v.heading_deg * Math.PI / 180 - state.view.rotRad;
      const len = 24 + 6 * (v.speed_mps || 0);
      el('line', {
        x1: vx, y1: vy,
        x2: vx + Math.sin(ang) * len, y2: vy - Math.cos(ang) * len,
        class: `vec-arrow vec-${v.classification || 'remote'}`,
      }, root);
    }
  }

  // Vehicle icon (triangle pointing along heading, compensated for chart rotation).
  if (t) {
    const [e, n] = toLocal(t.lat_deg, t.lon_deg);
    const [x, y] = toScreen(e, n);
    const hdgDeg = ((t.yaw_rad || 0) - state.view.rotRad) * 180 / Math.PI;
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

  renderModePanel();
  renderWaypointList();
  renderVectorPanel();
  renderConstraintsPanel();
  renderStatusLog();
  renderExecuteDock();
}

/* ------------------------------------------------- operational mode panel */

const MODE_CHIP_CLASS = {
  MANUAL: 'chip-serious', STANDBY: 'chip-idle', REMOTE: 'chip-warning', AUTONOMOUS: 'chip-good',
};

let lastModeSignature = '';

function renderModePanel() {
  const op = state.opMode;
  const signature = JSON.stringify(op);
  if (signature === lastModeSignature) return;
  lastModeSignature = signature;
  const chip = $('mode-chip');
  const pending = $('mode-pending-chip');
  const buttons = document.querySelectorAll('.mode-btn');
  if (!op) {
    chip.textContent = 'MODE N/A';
    chip.className = 'chip chip-idle';
    pending.hidden = true;
    buttons.forEach((b) => { b.disabled = true; b.classList.remove('mode-active'); });
    return;
  }
  const mode = op.mode;
  if (!mode) {
    chip.textContent = 'MODE UNKNOWN';
    chip.className = 'chip chip-pending';
  } else if (op.report_age_s > 5) {
    chip.textContent = mode + ' · STALE';
    chip.className = 'chip chip-pending';
  } else {
    chip.textContent = mode;
    chip.className = 'chip ' + (MODE_CHIP_CLASS[mode] || 'chip-idle');
  }
  pending.hidden = !op.pending_mode;
  if (op.pending_mode) pending.textContent = '→ ' + op.pending_mode + '…';
  buttons.forEach((b) => {
    b.disabled = mode === 'MANUAL' || b.dataset.mode === mode;
    b.classList.toggle('mode-active', b.dataset.mode === mode);
  });
}

/* ------------------------------------------------- vector panel */

const STATUS_CHIP_CLASS = {
  ISSUED: 'chip-pending', COMMANDED: 'chip-pending', EXECUTING: 'chip-good',
  COMPLETED: 'chip-good', CANCELED: 'chip-serious', FAILED: 'chip-critical',
};

function renderVectorPanel() {
  const v = state.vector;
  $('vec-execute').disabled = state.rc.engaged;
  $('vec-cancel').disabled = !(v && v.active) || state.rc.engaged;
  $('btn-rc').textContent = state.rc.engaged ? 'Exit RC' : 'RC mode';
  $('vec-elev').disabled = !state.vecDraft.elevOn;
  $('vec-frame').disabled = !state.vecDraft.elevOn;
  $('vec-timeout').disabled = !state.vecDraft.timeoutOn;

  const chip = $('vec-status-chip');
  const hist = (v && v.status_history) || [];
  const last = hist.length ? hist[hist.length - 1] : null;
  if (last) {
    chip.textContent = last.status + (last.reason && last.reason !== 'SUCCEEDED' &&
        last.reason !== last.status ? ' · ' + last.reason : '');
    chip.className = 'chip ' + (STATUS_CHIP_CLASS[last.status] || 'chip-idle');
  } else if (v && v.active) {
    chip.textContent = 'SENT';
    chip.className = 'chip chip-pending';
  } else {
    chip.textContent = 'NO VECTOR';
    chip.className = 'chip chip-idle';
  }
  const ack = $('vec-ack-chip');
  const awaiting = v && v.session && v.active;
  ack.hidden = !awaiting;
  if (awaiting) {
    ack.textContent = v.ack_received ? 'ACK ✓' : 'AWAITING ACK';
    ack.className = 'chip ' + (v.ack_received ? 'chip-good' : 'chip-pending');
  }
  renderTrafficList();
}

let lastTrafficSignature = '';

function renderTrafficList() {
  const rows = [];
  for (const m of state.traffic.missions || []) {
    rows.push({
      cls: m.classification,
      desc: `WP×${(m.waypoints || []).length}` +
            (m.waypoints_remaining !== undefined ? ` (${m.waypoints_remaining} left)` : ''),
      status: m.status || (m.list_complete === false ? 'assembling' : ''),
      active: m.active,
    });
  }
  for (const v of state.traffic.vectors || []) {
    const bits = [];
    if (v.heading_deg !== undefined) bits.push(v.heading_deg.toFixed(0) + '°');
    if (v.speed_mps !== undefined) bits.push(v.speed_mps.toFixed(1) + ' m/s');
    if (v.elev_value_m !== undefined) {
      bits.push(v.elev_value_m.toFixed(0) + ' m ' + (v.elev_frame === 'asf' ? 'ASF' : 'depth'));
    }
    if (v.ends_in_s !== undefined && v.active) bits.push('ends ' + Math.max(0, v.ends_in_s).toFixed(0) + ' s');
    rows.push({ cls: v.classification, desc: 'VECTOR ' + bits.join(' · '), status: v.status || '',
                active: v.active });
  }
  const signature = JSON.stringify(rows);
  if (signature === lastTrafficSignature) return;
  lastTrafficSignature = signature;
  const list = $('traffic-list');
  list.replaceChildren();
  if (!rows.length) {
    const li = document.createElement('li');
    li.className = 'empty';
    li.textContent = 'No commands observed on the bus.';
    list.appendChild(li);
    return;
  }
  for (const r of rows) {
    const li = document.createElement('li');
    li.className = r.active ? '' : 'inactive';
    li.innerHTML = `<span class="badge badge-${r.cls}">${r.cls}</span>` +
                   `<span class="traffic-desc">${r.desc}</span>` +
                   `<span class="traffic-status">${r.status}</span>`;
    list.appendChild(li);
  }
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
  const violated = (state.constraints.items || [])
      .some((c) => c.state === false && isActive(c.id));
  $('violation-chip').hidden = !violated;
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
  if (state.rc.engaged) exitRcMode(true);  // never leave the heartbeat driving blind
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

/* ------------------------------------------------- constraints panel */

function describeConstraint(c) {
  if (c.type === 'keep_in' || c.type === 'keep_out') {
    const kind = c.type === 'keep_in' ? 'keep-in' : 'keep-out';
    const bound = (v, f) => f === 'asf' ? `${v} m ASF` : `${v} m`;
    const band = c.ceiling_m !== undefined && c.floor_m !== undefined
        ? ` · ${bound(c.ceiling_m, c.ceiling_frame)} → ${bound(c.floor_m, c.floor_frame)}` : '';
    return `${kind} · ${(c.polygon || []).length} pts${band}`;
  }
  const op = c.op === 'gte' ? '≥' : '≤';
  if (c.type === 'speed') return `speed ${op} ${(c.value ?? 0).toFixed(1)} m/s`;
  if (c.type === 'depth') return `depth ${op} ${(c.value ?? 0).toFixed(1)} m`;
  return c.type;
}

let lastConSignature = '';

function renderConstraintsPanel() {
  const con = state.constraints;
  const items = con.items || [];
  // The applied-set acknowledgement only exists after the FIRST activation command, so the
  // unknown state must never block toggling (chicken-and-egg); it only warns.
  $('con-banner').hidden = !con.enabled || con.active_known || !items.length;
  const list = $('con-list');
  const signature = JSON.stringify([items, con.active_ids, state.pendingActive, state.conSelected,
                                    con.enabled]);
  if (signature !== lastConSignature) {
    lastConSignature = signature;
    list.replaceChildren();
    if (!con.enabled) {
      const li = document.createElement('li');
      li.className = 'empty';
      li.textContent = 'Constraints are not configured.';
      list.appendChild(li);
    } else if (!items.length) {
      const li = document.createElement('li');
      li.className = 'empty';
      li.textContent = 'No constraints defined.';
      list.appendChild(li);
    }
    for (const c of items) {
      const li = document.createElement('li');
      li.dataset.cid = c.id;
      li.className = (state.conSelected === c.id ? 'selected ' : '') +
                     (c.state === false && isActive(c.id) ? 'violated' : '');
      const pending = state.pendingActive !== null &&
          state.pendingActive.includes(c.id) !== (con.active_ids || []).includes(c.id);
      li.innerHTML =
          `<input type="checkbox" class="con-active" ${isActive(c.id) ? 'checked' : ''}>` +
          `<span class="con-desc"><b>${c.name || c.type}</b> — ${describeConstraint(c)}` +
          `${pending ? ' <em class="pending">pending…</em>' : ''}` +
          `${c.state === false && isActive(c.id) ? ' <em class="viol">VIOLATED</em>' : ''}</span>`;
      list.appendChild(li);
    }
  }
  renderConstraintEditor();
}

function renderConstraintEditor() {
  const ed = $('con-editor');
  const c = (state.constraints.items || []).find((x) => x.id === state.conSelected);
  if (!c) {
    ed.hidden = true;
    state.conDraft = null;
    return;
  }
  // The editor renders from a LOCAL draft, never from the live item: the SSE feed re-renders
  // at 5 Hz, and rendering server values directly would stomp anything the user typed the
  // moment the input blurs. Change events commit into the draft (bindConstraintEditor);
  // Apply sends the draft. The draft is (re)seeded from the item when the selection changes.
  if (!state.conDraft || state.conDraft.id !== c.id) {
    state.conDraft = {
      id: c.id,
      name: c.name || '',
      ceiling_m: c.ceiling_m ?? 0,
      ceiling_frame: c.ceiling_frame || 'depth',
      floor_m: c.floor_m ?? 100,
      floor_frame: c.floor_frame || 'depth',
      value: c.value ?? 0,
      op: c.op || 'lte',
    };
  }
  const d = state.conDraft;
  ed.hidden = false;
  const isZone = c.type === 'keep_in' || c.type === 'keep_out';
  $('con-editor-title').textContent = `${c.name || c.type} (${c.type.replace('_', '-')})`;
  $('con-zone-fields').hidden = !isZone;
  $('con-value-fields').hidden = isZone;
  // Still never touch the input the user is typing in right now (its value is ahead of the
  // draft until its change event fires on blur/Enter).
  const setVal = (inp, v) => { if (document.activeElement !== inp) inp.value = v; };
  setVal($('con-name'), d.name);
  if (isZone) {
    setVal($('con-ceiling'), d.ceiling_m);
    setVal($('con-floor'), d.floor_m);
    setVal($('con-ceiling-frame'), d.ceiling_frame);
    setVal($('con-floor-frame'), d.floor_frame);
  } else {
    setVal($('con-value'), d.value);
    setVal($('con-op'), d.op);
    $('con-unit').textContent = c.type === 'speed' ? 'm/s' : 'm';
  }
}

//! Commit editor field changes into the local draft (bound once at boot).
function bindConstraintEditor() {
  const commit = (id, fn) => $(id).addEventListener('change', (ev) => {
    if (state.conDraft) fn(ev.target);
  });
  commit('con-name', (t) => { state.conDraft.name = t.value; });
  commit('con-ceiling', (t) => { state.conDraft.ceiling_m = parseFloat(t.value) || 0; });
  commit('con-ceiling-frame', (t) => { state.conDraft.ceiling_frame = t.value; });
  commit('con-floor', (t) => { state.conDraft.floor_m = parseFloat(t.value) || 0; });
  commit('con-floor-frame', (t) => { state.conDraft.floor_frame = t.value; });
  commit('con-value', (t) => { state.conDraft.value = parseFloat(t.value) || 0; });
  commit('con-op', (t) => { state.conDraft.op = t.value; });
}

function selectConstraint(id) {
  state.conSelected = state.conSelected === id ? null : id;
  state.conDraft = null;  // reseed the editor from the (possibly different) item
  lastConSignature = '';
  render();
}

async function applyConstraintEdit() {
  const c = (state.constraints.items || []).find((x) => x.id === state.conSelected);
  const d = state.conDraft;
  if (!c || !d) return;
  const body = { id: c.id, type: c.type, name: d.name || c.type };
  if (c.type === 'keep_in' || c.type === 'keep_out') {
    body.polygon = c.polygon;
    body.ceiling_m = d.ceiling_m;
    body.ceiling_frame = d.ceiling_frame;
    body.floor_m = d.floor_m;
    body.floor_frame = d.floor_frame;
  } else {
    body.value = d.value;
    body.op = d.op;
  }
  try {
    await api('/api/constraints', body);
  } catch (e) {
    alert('Failed to apply constraint: ' + e.message);
  }
}

async function deleteConstraint() {
  const id = state.conSelected;
  if (!id) return;
  try {
    await fetch('/api/constraints/' + id, { method: 'DELETE' });
    state.conSelected = null;
  } catch (e) {
    alert('Failed to delete constraint: ' + e.message);
  }
}

/* --- instant apply-on-toggle: optimistic set, debounced command, ack echo clears pending --- */

let activeTimer = null;

function toggleActive(id, on) {
  const base = state.pendingActive !== null ? state.pendingActive
                                            : [...(state.constraints.active_ids || [])];
  const next = base.filter((x) => x !== id);
  if (on) next.push(id);
  state.pendingActive = next;
  lastConSignature = '';
  clearTimeout(activeTimer);
  // Capture the intended set NOW: an ack echo can null pendingActive before the debounce
  // fires, and posting ids:null would deactivate everything.
  const ids = next;
  activeTimer = setTimeout(async () => {
    try {
      await api('/api/constraints/active', { ids });
    } catch (e) {
      alert('Failed to command the active set: ' + e.message);
      state.pendingActive = null;
      lastConSignature = '';
      render();
    }
  }, 300);
}

/* ------------------------------------------------- zone drawing */

function segsIntersect(a, b, c, d) {
  const cross = (p, q, r) => (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
  const s1 = cross(a, b, c) * cross(a, b, d);
  const s2 = cross(c, d, a) * cross(c, d, b);
  return s1 < 0 && s2 < 0;
}

function zoneSelfIntersects(points) {
  const n = points.length;
  for (let i = 0; i < n; i++) {
    for (let j = i + 2; j < n; j++) {
      if (i === 0 && j === n - 1) continue;  // closing edge shares the first vertex
      if (segsIntersect(points[i], points[(i + 1) % n], points[j], points[(j + 1) % n])) {
        return true;
      }
    }
  }
  return false;
}

function enterZoneDraw(kind) {
  if (state.rc.engaged) exitRcMode(true);  // never leave the heartbeat driving blind
  if (state.mode === 'edit') exitEditMode();
  state.mode = 'zone-draw';
  state.zoneDraft = { kind, points: [] };
  $('btn-new-mission').hidden = true;
  $('btn-cancel-zone').hidden = false;
  $('zone-hint').hidden = false;
  render();
}

function exitZoneDraw() {
  state.mode = 'monitor';
  state.zoneDraft = null;
  $('btn-new-mission').hidden = false;
  $('btn-cancel-zone').hidden = true;
  $('zone-hint').hidden = true;
  render();
}

async function finishZoneDraw() {
  const draft = state.zoneDraft;
  if (!draft || draft.points.length < 3) return;
  if (zoneSelfIntersects(draft.points)) {
    alert('Zone polygon must not self-intersect.');
    return;
  }
  const polygon = draft.points.map(([e, n]) => toGeo(e, n));
  const name = (draft.kind === 'keep_in' ? 'keep-in ' : 'keep-out ') +
               ((state.constraints.items || []).length + 1);
  try {
    await api('/api/constraints', {
      type: draft.kind, name, polygon,
      ceiling_m: 0.0, ceiling_frame: 'depth', floor_m: 100.0, floor_frame: 'depth',
    });
    exitZoneDraw();
  } catch (e) {
    alert('Failed to create zone: ' + e.message);
  }
}

function addZoneVertex(east, north) {
  const pts = state.zoneDraft.points;
  // Clicking the first vertex closes the polygon.
  if (pts.length >= 3) {
    const [x0, y0] = toScreen(...pts[0]);
    const [x1, y1] = toScreen(east, north);
    if (Math.hypot(x1 - x0, y1 - y0) < 12) {
      finishZoneDraw();
      return;
    }
  }
  pts.push([east, north]);
  render();
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
      // World-anchor pan (rotation-correct): keep the grabbed point under the cursor.
      const [e0, n0] = toWorld(lastX - rect.left, lastY - rect.top);
      const [e1, n1] = toWorld(ev.clientX - rect.left, ev.clientY - rect.top);
      state.view.cE += e0 - e1;
      state.view.cN += n0 - n1;
      state.view.userPanned = true;
      $('follow-vehicle').checked = false;
      setFollowHeading(false);
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
    if (target && target.dataset.idx !== undefined &&
        !target.classList.contains('wp-traffic')) {
      selectWaypoint(parseInt(target.dataset.idx, 10));
      return;
    }
    if (state.mode === 'zone-draw' && ev.target.closest('#chart')) {
      const [e, n] = toWorld(ev.clientX - rect.left, ev.clientY - rect.top);
      addZoneVertex(e, n);
      return;
    }
    const zone = ev.target.closest ? ev.target.closest('.zone') : null;
    if (zone && zone.dataset.cid && state.mode === 'monitor') {
      selectConstraint(zone.dataset.cid);
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
  chart.addEventListener('dblclick', (ev) => {
    if (state.mode === 'zone-draw') {
      ev.preventDefault();
      finishZoneDraw();
    }
  });
  window.addEventListener('keydown', (ev) => {
    if (state.mode === 'zone-draw') {
      if (ev.key === 'Enter') finishZoneDraw();
      if (ev.key === 'Escape') exitZoneDraw();
      return;
    }
    if (state.mode !== 'rc') return;
    if (ev.code === 'Escape') {  // the escape hatch must work even from a form field
      exitRcMode(true);
      return;
    }
    if (typingInField()) return;
    handleRcKeyDown(ev);
  });
  window.addEventListener('keyup', (ev) => {
    if (state.mode !== 'rc') return;
    handleRcKeyUp(ev);
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

/* ------------------------------------------------- vector panel bindings */

function typingInField() {
  const a = document.activeElement;
  return a && (a.tagName === 'INPUT' || a.tagName === 'SELECT' || a.tagName === 'TEXTAREA');
}

function bindVectorPanel() {
  const d = state.vecDraft;
  $('vec-hdg').value = d.heading_deg;
  $('vec-speed').value = d.speed_mps;
  $('vec-elev').value = d.elev_value_m;
  $('vec-frame').value = d.elev_frame;
  $('vec-timeout').value = d.timeout_s;
  $('vec-hdg').addEventListener('change', (ev) => { d.heading_deg = parseFloat(ev.target.value) || 0; });
  $('vec-speed').addEventListener('change', (ev) => {
    d.speed_mps = Math.max(0, parseFloat(ev.target.value) || 0);
  });
  $('vec-elev-on').addEventListener('change', (ev) => { d.elevOn = ev.target.checked; render(); });
  $('vec-elev').addEventListener('change', (ev) => {
    d.elev_value_m = Math.max(0, parseFloat(ev.target.value) || 0);
  });
  $('vec-frame').addEventListener('change', (ev) => { d.elev_frame = ev.target.value; });
  $('vec-timeout-on').addEventListener('change', (ev) => { d.timeoutOn = ev.target.checked; render(); });
  $('vec-timeout').addEventListener('change', (ev) => {
    d.timeout_s = Math.max(1, parseFloat(ev.target.value) || 30);
  });
  $('vec-execute').addEventListener('click', async () => {
    const body = { heading_deg: ((d.heading_deg % 360) + 360) % 360, speed_mps: d.speed_mps };
    if (d.elevOn) { body.elev_value_m = d.elev_value_m; body.elev_frame = d.elev_frame; }
    if (d.timeoutOn) body.timeout_s = d.timeout_s;
    try { await api('/api/vector', body); } catch (e) { alert('Failed to send vector: ' + e.message); }
  });
  $('vec-cancel').addEventListener('click', async () => {
    try { await api('/api/vector/cancel', {}); } catch (e) { alert('Failed to cancel vector: ' + e.message); }
  });
  $('btn-rc').addEventListener('click', () => {
    if (state.rc.engaged) exitRcMode(true); else enterRcMode();
  });
  document.querySelectorAll('.mode-btn').forEach((b) => {
    b.addEventListener('click', async () => {
      try { await api('/api/mode', { mode: b.dataset.mode }); }
      catch (e) { alert('Failed to command mode: ' + e.message); }
    });
  });
}

/* ------------------------------------------------- WASD remote control */

function rcSpeedLimit() {
  const p = state.platform || {};
  let limit = p.max_speed_mps || 6.0;
  if (state.rc.elevOn && p.max_speed_underwater_mps !== undefined) {
    limit = Math.min(limit, p.max_speed_underwater_mps);
  }
  return limit;
}

function enterRcMode() {
  if (state.mode === 'edit') exitEditMode();
  if (state.mode === 'zone-draw') exitZoneDraw();
  state.mode = 'rc';
  state.rc.engaged = true;
  state.rc.engagedAt = Date.now();
  state.rc.keys.clear();
  state.rc.lastHeadingRad = (state.telemetry && state.telemetry.yaw_rad) || 0;
  state.rc.linkOk = true;
  $('rc-overlay').hidden = false;
  state.rc.timer = setInterval(rcTick, 200);  // ~5 Hz heartbeat; the server owns the deadman
  rcTick();
  render();
}

function exitRcMode(notifyServer) {
  if (state.rc.timer) clearInterval(state.rc.timer);
  state.rc.timer = null;
  state.rc.engaged = false;
  if (state.mode === 'rc') state.mode = 'monitor';
  $('rc-overlay').hidden = true;
  if (notifyServer) api('/api/rc', { active: false }).catch(() => {});
  render();
}

// W = along the vehicle heading; W+A / W+D = ∓/± 45°; A / D alone = ∓/± 90°;
// S = stop (zero speed — UMAA ground speed has no reverse here). Undefined = no key held.
function rcHeadingOffsetDeg() {
  const k = state.rc.keys;
  if (k.has('KeyS')) return null;
  const w = k.has('KeyW');
  const a = k.has('KeyA');
  const d = k.has('KeyD');
  if (w && a && !d) return -45;
  if (w && d && !a) return 45;
  if (w) return 0;
  if (a && !d) return -90;
  if (d && !a) return 90;
  return undefined;
}

async function rcTick() {
  if (!state.rc.engaged) return;
  const t = state.telemetry;
  const vehicleHdg = t ? (t.yaw_rad || 0) : (state.rc.lastHeadingRad || 0);
  const off = rcHeadingOffsetDeg();
  let headingRad;
  let speed;
  if (off === undefined || off === null) {
    // No direction key (or S): zero speed, holding the last commanded heading.
    headingRad = state.rc.lastHeadingRad !== null ? state.rc.lastHeadingRad : vehicleHdg;
    speed = 0;
  } else {
    headingRad = vehicleHdg + off * Math.PI / 180;
    speed = Math.min(state.rc.speedMps, rcSpeedLimit());
    state.rc.lastHeadingRad = headingRad;
  }
  const body = {
    active: true,
    heading_deg: ((headingRad * 180 / Math.PI) % 360 + 360) % 360,
    speed_mps: speed,
  };
  if (state.rc.elevOn) {
    body.elev_value_m = state.rc.elevValueM;
    body.elev_frame = state.rc.elevFrame;
  }
  try {
    await api('/api/rc', body);
    state.rc.linkOk = true;
  } catch (e) {
    state.rc.linkOk = false;
  }
  renderRcOverlay();
}

//! Shift = up, Ctrl = down, frame-aware: up is a smaller depth but a larger above-floor altitude.
function rcStepElevation(upSteps) {
  const rc = state.rc;
  rc.elevOn = true;
  const delta = rc.elevFrame === 'depth' ? -upSteps : upSteps;
  rc.elevValueM = Math.max(0, rc.elevValueM + delta);
  const floor = state.platform && state.platform.floor_depth_m;
  if (floor !== undefined) rc.elevValueM = Math.min(rc.elevValueM, floor);
}

function rcToggleFrame() {
  const rc = state.rc;
  const floor = state.platform && state.platform.floor_depth_m;
  // Convert the setpoint so the toggle keeps roughly the same physical elevation.
  if (rc.elevOn && floor !== undefined) {
    rc.elevValueM = Math.max(0, floor - rc.elevValueM);
  }
  rc.elevFrame = rc.elevFrame === 'depth' ? 'asf' : 'depth';
}

const RC_DIRECTION_KEYS = ['KeyW', 'KeyA', 'KeyS', 'KeyD'];
const RC_HANDLED_KEYS = [...RC_DIRECTION_KEYS, 'KeyR', 'KeyF', 'KeyZ',
                         'ShiftLeft', 'ShiftRight', 'ControlLeft', 'ControlRight'];

function handleRcKeyDown(ev) {
  if (ev.code === 'Escape') { exitRcMode(true); return; }
  if (!RC_HANDLED_KEYS.includes(ev.code)) return;
  ev.preventDefault();
  if (RC_DIRECTION_KEYS.includes(ev.code)) state.rc.keys.add(ev.code);
  if (ev.repeat) { renderRcOverlay(); return; }  // held keys must not autorepeat the steps
  if (ev.code === 'KeyR') state.rc.speedMps = Math.min(rcSpeedLimit(), state.rc.speedMps + 0.25);
  if (ev.code === 'KeyF') state.rc.speedMps = Math.max(0.25, state.rc.speedMps - 0.25);
  if (ev.code === 'ShiftLeft' || ev.code === 'ShiftRight') rcStepElevation(1);
  if (ev.code === 'ControlLeft' || ev.code === 'ControlRight') rcStepElevation(-1);
  if (ev.code === 'KeyZ') rcToggleFrame();
  renderRcOverlay();
}

function handleRcKeyUp(ev) {
  if (RC_DIRECTION_KEYS.includes(ev.code)) {
    state.rc.keys.delete(ev.code);
    renderRcOverlay();
  }
}

function renderRcOverlay() {
  if (!state.rc.engaged) return;
  const keyIds = [['rc-key-w', 'KeyW'], ['rc-key-a', 'KeyA'], ['rc-key-s', 'KeyS'],
                  ['rc-key-d', 'KeyD']];
  for (const [id, code] of keyIds) {
    $(id).classList.toggle('held', state.rc.keys.has(code));
  }
  $('rc-speed').textContent = state.rc.speedMps.toFixed(2) + ' m/s';
  $('rc-elev').textContent = state.rc.elevOn
      ? state.rc.elevValueM.toFixed(0) + ' m ' + (state.rc.elevFrame === 'asf' ? 'ASF' : 'depth')
      : 'unset';
  const banner = $('rc-banner');
  if (!state.rc.linkOk) {
    banner.textContent = 'LINK LOST — vehicle stops in ≤2 s';
    banner.classList.add('lost');
  } else {
    const mode = state.opMode && state.opMode.mode;
    banner.textContent = 'REMOTE CONTROL ACTIVE — keys drive the vehicle' +
        (mode && mode !== 'REMOTE' ? ` (mode ${mode}: commands may be rejected)` : '');
    banner.classList.remove('lost');
  }
}

/* ------------------------------------------------- follow heading */

function setFollowHeading(on) {
  $('follow-heading').checked = on;
  if (!on) state.view.rotRad = 0;
}

/* ------------------------------------------------- SSE feed */

function applySnapshot(snap) {
  const t = snap.telemetry && snap.telemetry.lat_deg !== undefined ? snap.telemetry : null;
  state.mission = snap.mission || state.mission;
  state.execStatus = snap.exec_status || null;
  state.opMode = snap.operational_mode || null;
  state.vector = snap.vector || null;
  state.traffic = snap.traffic || { missions: [], vectors: [] };
  if (snap.platform) state.platform = snap.platform;
  // The server-side deadman (or an external cancel) ended the RC session: leave RC mode
  // locally too. The engage-grace covers the race before the poller reflects rc_active.
  if (state.rc.engaged && state.vector && !state.vector.rc_active &&
      Date.now() - state.rc.engagedAt > 3000) {
    exitRcMode(false);
  }
  if (snap.constraints) {
    state.constraints = snap.constraints;
    // The autopilot's ack echoing the commanded set clears the optimistic pending state.
    if (state.pendingActive !== null) {
      const echoed = [...(state.constraints.active_ids || [])].sort().join(',');
      const wanted = [...state.pendingActive].sort().join(',');
      if (echoed === wanted) state.pendingActive = null;
    }
  }

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
    state.view.rotRad = $('follow-heading').checked ? (t.yaw_rad || 0) : 0;
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
$('btn-cancel-zone').addEventListener('click', exitZoneDraw);
$('btn-add-kia').addEventListener('click', () => enterZoneDraw('keep_in'));
$('btn-add-koa').addEventListener('click', () => enterZoneDraw('keep_out'));
$('btn-add-speed').addEventListener('click', async () => {
  try { await api('/api/constraints', { type: 'speed', name: 'max speed', op: 'lte', value: 3.0 }); }
  catch (e) { alert('Failed to create speed constraint: ' + e.message); }
});
$('btn-add-depth').addEventListener('click', async () => {
  try { await api('/api/constraints', { type: 'depth', name: 'max depth', op: 'lte', value: 20.0 }); }
  catch (e) { alert('Failed to create depth constraint: ' + e.message); }
});
$('con-list').addEventListener('click', (ev) => {
  const li = ev.target.closest('li[data-cid]');
  if (!li) return;
  if (ev.target.classList.contains('con-active')) {
    toggleActive(li.dataset.cid, ev.target.checked);
    return;
  }
  selectConstraint(li.dataset.cid);
});
$('con-apply').addEventListener('click', applyConstraintEdit);
$('con-delete').addEventListener('click', deleteConstraint);
bindConstraintEditor();
$('btn-clear').addEventListener('click', () => {
  state.draft = [];
  state.draftPath = [];
  state.selected = -1;
  render();
});
$('btn-done-edit').addEventListener('click', exitEditMode);
$('btn-execute').addEventListener('click', onExecute);
$('follow-vehicle').addEventListener('change', (ev) => {
  if (!ev.target.checked) setFollowHeading(false);  // heading-up requires following
  render();
});
$('follow-heading').addEventListener('change', (ev) => {
  if (ev.target.checked) {
    $('follow-vehicle').checked = true;
    if (state.telemetry) state.view.rotRad = state.telemetry.yaw_rad || 0;
  } else {
    state.view.rotRad = 0;
  }
  render();
});
window.addEventListener('resize', render);
window.addEventListener('beforeunload', () => {
  // Best-effort RC stop; the server watchdog and DDS endTime cover the rest.
  if (state.rc.engaged) navigator.sendBeacon('/api/rc', JSON.stringify({ active: false }));
});
// Losing focus swallows keyup events: without this, a key held while alt-tabbing away
// would keep the vehicle driving with no operator watching. The next rcTick sends speed 0.
window.addEventListener('blur', () => {
  if (state.rc.engaged) {
    state.rc.keys.clear();
    renderRcOverlay();
  }
});
document.addEventListener('visibilitychange', () => {
  if (document.hidden && state.rc.engaged) {
    state.rc.keys.clear();
    renderRcOverlay();
  }
});

bindEditor();
bindVectorPanel();
bindChart();
connectStream();
render();
