/* Magikid R1-A console — client logic ---------------------------------- */
"use strict";

const DEG = 180 / Math.PI;
const SVGNS = "http://www.w3.org/2000/svg";
// Concrete colors for SVG presentation attributes (var() is unreliable there).
const COL = { armL: "#4aa8ff", armR: "#ffa14a", acc: "#24e0b5", head: "#3a4356" };

/* Joint table in the exact order the bridge streams `q`/`dq`/... (0..25).
   `min`/`max` are display/slider ranges in radians (approximate). */
const JOINTS = [
  { name: "L Hip Pitch",     grp: "Left Leg",   min: -3.14, max: 3.14 },
  { name: "L Hip Roll",      grp: "Left Leg",   min: -1.57, max: 1.57 },
  { name: "L Hip Yaw",       grp: "Left Leg",   min: -1.57, max: 1.57 },
  { name: "L Knee",          grp: "Left Leg",   min:  0.00, max: 2.60 },
  { name: "L Ankle Pitch",   grp: "Left Leg",   min: -1.00, max: 1.00 },
  { name: "L Ankle Roll",    grp: "Left Leg",   min: -0.60, max: 0.60 },
  { name: "R Hip Pitch",     grp: "Right Leg",  min: -3.14, max: 3.14 },
  { name: "R Hip Roll",      grp: "Right Leg",  min: -1.57, max: 1.57 },
  { name: "R Hip Yaw",       grp: "Right Leg",  min: -1.57, max: 1.57 },
  { name: "R Knee",          grp: "Right Leg",  min:  0.00, max: 2.60 },
  { name: "R Ankle Pitch",   grp: "Right Leg",  min: -1.00, max: 1.00 },
  { name: "R Ankle Roll",    grp: "Right Leg",  min: -0.60, max: 0.60 },
  { name: "Waist Roll",      grp: "Waist",      min: -0.60,    max: 0.60 },
  // arm/head limits below are the exact URDF (r1_a5_description) values
  { name: "Waist Yaw",       grp: "Waist",      min: -2.618,   max: 2.618 },
  { name: "L Shoulder Pitch",grp: "Left Arm",   min: -3.1416,  max: 2.0944 },
  { name: "L Shoulder Roll", grp: "Left Arm",   min: -0.22689, max: 2.4784 },
  { name: "L Shoulder Yaw",  grp: "Left Arm",   min: -1.9199,  max: 1.9199 },
  { name: "L Elbow",         grp: "Left Arm",   min: -0.97564, max: 2.1852 },
  { name: "L Wrist Roll",    grp: "Left Arm",   min: -1.9199,  max: 1.9199 },
  { name: "R Shoulder Pitch",grp: "Right Arm",  min: -3.1416,  max: 2.0944 },
  { name: "R Shoulder Roll", grp: "Right Arm",  min: -2.47849, max: 0.2268 },
  { name: "R Shoulder Yaw",  grp: "Right Arm",  min: -1.9199,  max: 1.9199 },
  { name: "R Elbow",         grp: "Right Arm",  min: -0.97564, max: 2.1852 },
  { name: "R Wrist Roll",    grp: "Right Arm",  min: -1.9199,  max: 1.9199 },
  { name: "Head Pitch",      grp: "Head",       min: -0.62832, max: 0.62832 },
  { name: "Head Yaw",        grp: "Head",       min: -2.0071,  max: 2.0071 },
];

/* IDX, ARM_CMD_IDX, KIN and computeFK live in kin.js (shared with the Quest page). */

let latest = null;      // most recent state object
let armEngaged = false; // reported by bridge
let masterOn = false;   // UI safety switch
const targets = new Array(12).fill(0); // command targets (rad), 12 joints

/* ---------------------------------------------------------------------- */
/* DOM builders                                                            */
/* ---------------------------------------------------------------------- */
const $ = (s) => document.querySelector(s);
const el = (t, cls) => { const e = document.createElement(t); if (cls) e.className = cls; return e; };
function svg(tag, attrs) {
  const e = document.createElementNS(SVGNS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  return e;
}

const jointRowEls = [];   // for the "arms" view
const jointRowElsAll = []; // for the "all" view

function buildJointViews() {
  const groupsArms = ["Left Arm", "Right Arm", "Waist", "Head"];
  const groupsAll = ["Left Arm", "Right Arm", "Waist", "Head", "Left Leg", "Right Leg"];
  buildJointGroup($("#joint-arms"), groupsArms, jointRowEls);
  buildJointGroup($("#joint-all"), groupsAll, jointRowElsAll);
}

function buildJointGroup(container, groups, store) {
  container.innerHTML = "";
  groups.forEach((g) => {
    const box = el("div", "jgroup");
    const title = el("div", "jgroup-title"); title.textContent = g; box.appendChild(title);
    JOINTS.forEach((j, i) => {
      if (j.grp !== g) return;
      const row = el("div", "jrow");
      const name = el("div", "jname"); name.textContent = j.name;
      const bar = el("div", "jbar");
      const center = el("div", "jbar-center");
      const fill = el("div", "jbar-fill");
      bar.appendChild(center); bar.appendChild(fill);
      const val = el("div", "jval"); val.textContent = "0.00";
      const vel = el("div", "jvel"); vel.textContent = "0.0";
      const tau = el("div", "jtau"); tau.textContent = "0.0";
      row.appendChild(name); row.appendChild(bar); row.appendChild(val);
      row.appendChild(vel); row.appendChild(tau);
      box.appendChild(row);
      store.push({ i, row, fill, val, vel, tau, name });
    });
    container.appendChild(box);
  });
}

function updateJointStore(store, s) {
  for (const r of store) {
    const j = JOINTS[r.i];
    const q = s.q[r.i];
    const frac = (q - j.min) / (j.max - j.min);       // 0..1
    const c = Math.max(0, Math.min(1, frac));
    // fill grows from center (0 rad) outward for a signed feel
    const zeroFrac = Math.max(0, Math.min(1, (0 - j.min) / (j.max - j.min)));
    const left = Math.min(c, zeroFrac), right = Math.max(c, zeroFrac);
    r.fill.style.left = (left * 100) + "%";
    r.fill.style.width = ((right - left) * 100) + "%";
    r.val.textContent = (q * DEG).toFixed(1) + "°";
    r.vel.textContent = s.dq[r.i].toFixed(1);
    r.tau.textContent = s.tau[r.i].toFixed(1);
    if (s.mstate && s.mstate[r.i]) r.row.classList.add("err");
    else r.row.classList.remove("err");
  }
}

/* ---------------------------------------------------------------------- */
/* Command sliders                                                         */
/* ---------------------------------------------------------------------- */
const cmdSliderEls = [];
function buildCmdSliders() {
  const wrap = $("#cmd-sliders");
  wrap.innerHTML = "";
  ARM_CMD_IDX.forEach((sIdx, k) => {
    const j = JOINTS[sIdx];
    const row = el("div", "cslider");
    const label = el("label"); label.textContent = j.name;
    const input = el("input");
    input.type = "range"; input.min = j.min; input.max = j.max;
    input.step = 0.01; input.value = 0; input.disabled = true;
    const b = el("b"); b.textContent = "0°";
    input.addEventListener("input", () => {
      targets[k] = parseFloat(input.value);
      b.textContent = (targets[k] * DEG).toFixed(0) + "°";
    });
    row.appendChild(label); row.appendChild(input); row.appendChild(b);
    wrap.appendChild(row);
    if (sIdx === IDX.HEAD_YAW) { row.classList.add("off"); label.textContent += " (disabled)"; }
    cmdSliderEls.push({ k, sIdx, input, b });
  });
}
function syncSlidersToState() {
  if (!latest) return;
  cmdSliderEls.forEach((c) => {
    const q = latest.q[c.sIdx];
    c.input.value = q;
    c.b.textContent = (q * DEG).toFixed(0) + "°";
    targets[c.k] = q;
  });
}
function setSlidersEnabled(on) {
  cmdSliderEls.forEach((c) => (c.input.disabled = !on || c.sIdx === IDX.HEAD_YAW));
}

/* ---------------------------------------------------------------------- */
/* Cameras                                                                 */
/* ---------------------------------------------------------------------- */
function buildCameras() {
  fetch("/api/meta").then((r) => r.json()).then((meta) => {
    const grid = $("#cam-grid");
    grid.innerHTML = "";
    const cams = meta.cameras || [];
    const hint = $("#cam-hint");
    if (!cams.length) { grid.innerHTML = '<div class="cam-empty">no cameras detected</div>'; if (hint) hint.textContent = "none"; return; }
    if (hint) hint.textContent = cams.some((c) => c.kind === "head") ? "head + grippers" : "onboard";
    cams.forEach((c) => {
      const box = el("div", "cam" + (c.kind === "head" ? " head" : ""));
      const tag = el("div", "cam-tag");
      const label = c.label || ("CAM " + c.index);
      tag.innerHTML = '<span class="rec">●</span> ' + label;
      const img = el("img");
      img.src = "/cam/" + c.index + "?t=" + Date.now();
      img.onerror = () => { box.classList.add("cam-off"); tag.innerHTML = label + " · offline"; };
      box.appendChild(img); box.appendChild(tag);
      grid.appendChild(box);
    });
  }).catch(() => {});
}


// --- camera / projection ---
let viewAz = 1.05, viewEl = 0.32;   // default 3/4 view
const VIEWS = { front: [Math.PI/2, 0.12], side: [0.0, 0.12], iso: [1.05, 0.32], top: [Math.PI/2, 1.45] };
const V = { CX: 170, CY: 292, SCALE: 360 };
function project(p) {
  const [x, y, z] = p;
  const ca=Math.cos(viewAz), sa=Math.sin(viewAz);
  const x1 = x*ca - y*sa, y1 = x*sa + y*ca;
  const ce=Math.cos(viewEl), se=Math.sin(viewEl);
  const z2 = y1*se + z*ce;
  return { sx: V.CX + V.SCALE*x1, sy: V.CY - V.SCALE*z2 };
}

function initViz3d() {
  const s = $("#svg3d");
  let drag = null;                    // drag to orbit
  s.addEventListener("pointerdown", (e) => { drag = { x: e.clientX, y: e.clientY }; try { s.setPointerCapture(e.pointerId); } catch (_) {} });
  s.addEventListener("pointermove", (e) => {
    if (!drag) return;
    viewAz += (e.clientX - drag.x) * 0.01;
    viewEl = Math.max(-1.4, Math.min(1.5, viewEl + (e.clientY - drag.y) * 0.01));
    drag = { x: e.clientX, y: e.clientY };
    setActiveViewBtn(null);
  });
  const end = () => { drag = null; };
  s.addEventListener("pointerup", end); s.addEventListener("pointercancel", end);
  document.querySelectorAll(".vbtn").forEach((b) => b.addEventListener("click", () => {
    const v = VIEWS[b.dataset.view]; if (v) { viewAz = v[0]; viewEl = v[1]; }
    if (window.Arm3D && window.Arm3D.setView) window.Arm3D.setView(b.dataset.view);
    setActiveViewBtn(b);
  }));
  setActiveViewBtn(document.querySelector('.vbtn[data-view="front"]'));
  // Try the WebGL mesh viewer; if three.js/meshes are missing it stays inert
  // and we keep drawing the SVG skeleton below.
  if (window.Arm3D && window.Arm3D.init) {
    try { window.Arm3D.init(document.getElementById("view3d")); } catch (e) { console.warn("Arm3D init failed", e); }
  }
}
let viz3dActive = false;
function useArm3DIfReady() {
  if (viz3dActive || !(window.Arm3D && window.Arm3D.ready)) return viz3dActive;
  viz3dActive = true;                       // meshes finished loading: switch over
  const sv = $("#svg3d"); if (sv) sv.style.display = "none";
  const v3 = $("#view3d"); if (v3) v3.style.display = "block";
  return true;
}
function setActiveViewBtn(active) {
  document.querySelectorAll(".vbtn").forEach((b) => b.classList.toggle("active", b === active));
}
function setActiveViewBtn(active) {
  document.querySelectorAll(".vbtn").forEach((b) => b.classList.toggle("active", b === active));
}

function line3(g, a, b, color, w) {
  const pa = project(a), pb = project(b);
  const l = svg("line", { x1: pa.sx, y1: pa.sy, x2: pb.sx, y2: pb.sy });
  l.setAttribute("stroke", color); l.setAttribute("stroke-width", w);
  l.setAttribute("stroke-linecap", "round"); g.appendChild(l);
}
function dot3(g, p, r, fill, stroke) {
  const pp = project(p);
  const c = svg("circle", { cx: pp.sx, cy: pp.sy, r });
  c.setAttribute("fill", fill || "none");
  if (stroke) { c.setAttribute("stroke", stroke); c.setAttribute("stroke-width", 2.5); }
  g.appendChild(c);
}

function renderArm3D(s) {
  const g = $("#svg3d"); g.innerHTML = "";
  const F = computeFK(s.q);

  // ground reference cross at the pelvis origin
  line3(g, [-0.13, 0, 0], [0.13, 0, 0], "#202635", 1);
  line3(g, [0, -0.13, 0], [0, 0.13, 0], "#202635", 1);

  // torso: pelvis origin fans up to the two shoulders and the neck
  line3(g, [0,0,0], F.l_sp.p, "#39415a", 7);
  line3(g, [0,0,0], F.r_sp.p, "#39415a", 7);
  line3(g, [0,0,0], F.h_p.p,  "#2f3650", 6);
  dot3(g, [0,0,0], 5, "#39415a");

  // arms
  drawChain3(g, F, ["l_sp","l_sr","l_sy","l_el","l_wr","l_tip"], COL.armL);
  drawChain3(g, F, ["r_sp","r_sr","r_sy","r_el","r_wr","r_tip"], COL.armR);

  // head: neck -> head_yaw, head sphere, gaze marker
  line3(g, F.h_p.p, F.h_y.p, "#2f3650", 6);
  dot3(g, F.h_y.p, 13, COL.head);
  line3(g, F.h_y.p, F.h_top.p, COL.acc, 3);
  dot3(g, F.h_top.p, 3.5, COL.acc);
}

function drawChain3(g, F, names, color) {
  for (let i = 0; i < names.length - 1; i++)
    line3(g, F[names[i]].p, F[names[i+1]].p, color, Math.max(3, 7 - i * 0.6));
  for (let i = 1; i < names.length - 1; i++) dot3(g, F[names[i]].p, 3.4, "#0c0e14", color);
  dot3(g, F[names[0]].p, 5, color);                                   // shoulder
  dot3(g, F[names[names.length - 1]].p, 6, "none", color);            // hand
}

/* ---------------------------------------------------------------------- */
/* Dials                                                                   */
/* ---------------------------------------------------------------------- */
function initDial(id) {
  const s = $("#" + id);
  s.innerHTML = "";
  const ring = svg("circle", { cx: 45, cy: 45, r: 34, fill: "#0c0e14", stroke: "#262c3b", "stroke-width": 2 });
  s.appendChild(ring);
  for (let a = -140; a <= 140; a += 35) {
    const rad = (a - 90) * Math.PI / 180;
    const x1 = 45 + 30 * Math.cos(rad), y1 = 45 + 30 * Math.sin(rad);
    const x2 = 45 + 34 * Math.cos(rad), y2 = 45 + 34 * Math.sin(rad);
    s.appendChild(svg("line", { x1, y1, x2, y2, stroke: "#3a4356", "stroke-width": 2 }));
  }
  const needle = svg("line", { x1: 45, y1: 45, x2: 45, y2: 16, stroke: COL.acc, "stroke-width": 3, "stroke-linecap": "round" });
  needle.setAttribute("id", id + "-needle");
  s.appendChild(needle);
  s.appendChild(svg("circle", { cx: 45, cy: 45, r: 4, fill: COL.acc }));
  const txt = svg("text", { x: 45, y: 66, "text-anchor": "middle", fill: "#8b94a7", "font-size": 11, "font-family": "monospace" });
  txt.setAttribute("id", id + "-txt"); txt.textContent = "0°";
  s.appendChild(txt);
}
function setDial(id, val, range) {
  const frac = Math.max(-1, Math.min(1, val / range));
  const ang = frac * 140 - 90;            // -140..140 deg maps around top
  const needle = $("#" + id + "-needle");
  needle.setAttribute("transform", `rotate(${frac * 140} 45 45)`);
  $("#" + id + "-txt").textContent = (val * DEG).toFixed(0) + "°";
}

/* ---------------------------------------------------------------------- */
/* IMU artificial horizon                                                  */
/* ---------------------------------------------------------------------- */
function initHorizon() {
  const s = $("#svg-horizon");
  s.innerHTML = "";
  const defs = svg("defs", {});
  const clip = svg("clipPath", { id: "hz-clip" });
  clip.appendChild(svg("circle", { cx: 80, cy: 80, r: 66 }));
  defs.appendChild(clip); s.appendChild(defs);
  const grp = svg("g", { "clip-path": "url(#hz-clip)" }); grp.setAttribute("id", "hz-grp");
  const inner = svg("g", {}); inner.setAttribute("id", "hz-inner");
  inner.appendChild(svg("rect", { x: -80, y: -80, width: 320, height: 160, fill: "#2b6cb0" })); // sky
  inner.appendChild(svg("rect", { x: -80, y: 80, width: 320, height: 160, fill: "#6b4a2b" }));   // ground
  inner.appendChild(svg("line", { x1: -80, y1: 80, x2: 240, y2: 80, stroke: "#fff", "stroke-width": 2 }));
  for (let p = -60; p <= 60; p += 20) { if (!p) continue;
    inner.appendChild(svg("line", { x1: 55, y1: 80 + p, x2: 105, y2: 80 + p, stroke: "#ffffffaa", "stroke-width": 1 }));
  }
  grp.appendChild(inner); s.appendChild(grp);
  // fixed aircraft marker
  s.appendChild(svg("line", { x1: 55, y1: 80, x2: 72, y2: 80, stroke: COL.acc, "stroke-width": 3 }));
  s.appendChild(svg("line", { x1: 88, y1: 80, x2: 105, y2: 80, stroke: COL.acc, "stroke-width": 3 }));
  s.appendChild(svg("circle", { cx: 80, cy: 80, r: 2.5, fill: COL.acc }));
  s.appendChild(svg("circle", { cx: 80, cy: 80, r: 66, fill: "none", stroke: "#262c3b", "stroke-width": 3 }));
}
function setHorizon(roll, pitch) {
  const inner = $("#hz-inner");
  const px = Math.max(-60, Math.min(60, pitch * DEG)) * 1.2;
  inner.setAttribute("transform", `rotate(${-roll * DEG} 80 80) translate(0 ${px})`);
}

/* ---------------------------------------------------------------------- */
/* Rendering loop                                                          */
/* ---------------------------------------------------------------------- */
function render() {
  const s = latest;
  if (!(s && s.q) && preview) {                     // no telemetry: still show the sim
    const vq = headForward(previewQ());
    if (useArm3DIfReady()) window.Arm3D.setJoints(vq); else renderArm3D({ q: vq });
  }
  if (s && s.q) {
    updateJointStore(jointRowEls, s);
    if (!$("#joint-all").classList.contains("hidden")) updateJointStore(jointRowElsAll, s);
    const vq = headForward(preview ? previewQ() : s.q);  // sim preview or live
    if (useArm3DIfReady()) window.Arm3D.setJoints(vq);   // WebGL meshes
    else renderArm3D({ q: vq });                         // SVG fallback

    setDial("dial-waist", s.q[IDX.WAIST_YAW], 2.6);
    setDial("dial-headyaw", s.q[IDX.HEAD_YAW], 1.5);
    setDial("dial-headpitch", s.q[IDX.HEAD_PITCH], 0.6);
    if (s.imu_rpy) {
      setHorizon(s.imu_rpy[0], s.imu_rpy[1]);
      $("#imu-roll").textContent = (s.imu_rpy[0] * DEG).toFixed(1) + "°";
      $("#imu-pitch").textContent = (s.imu_rpy[1] * DEG).toFixed(1) + "°";
      $("#imu-yaw").textContent = (s.imu_rpy[2] * DEG).toFixed(1) + "°";
    }
    if (s.imu_gyro) $("#imu-gyro").textContent = s.imu_gyro.map((x) => x.toFixed(2)).join(", ");
    if (s.imu_acc) $("#imu-acc").textContent = s.imu_acc.map((x) => x.toFixed(2)).join(", ");
    if (s.have_torso && s.torso_rpy)
      $("#imu-torso").textContent = s.torso_rpy.map((x) => (x * DEG).toFixed(0) + "°").join(", ");
  }
  requestAnimationFrame(render);
}

/* ---------------------------------------------------------------------- */
/* WebSocket                                                               */
/* ---------------------------------------------------------------------- */
let ws = null, lastMsgTs = 0, hzEMA = 0;
function connect() {
  const url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws";
  ws = new WebSocket(url);
  ws.onopen = () => setLink(true);
  ws.onclose = () => { setLink(false); setTimeout(connect, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    let obj; try { obj = JSON.parse(ev.data); } catch (e) { return; }
    if (obj.type === "state") {
      latest = obj;
      const now = performance.now();
      if (lastMsgTs) { const hz = 1000 / (now - lastMsgTs); hzEMA = hzEMA ? hzEMA * 0.9 + hz * 0.1 : hz; }
      lastMsgTs = now;
      updateHeader(obj);
    } else if (obj.type === "log") {
      $("#log-line").textContent = obj.msg;
    }
  };
}
function setLink(on) {
  const pill = $("#pill-link");
  pill.classList.toggle("on", on);
  pill.classList.toggle("danger", !on);
  $("#link-state").textContent = on ? "connected" : "reconnecting";
}
function updateHeader(s) {
  $("#rate-hz").textContent = hzEMA ? hzEMA.toFixed(0) + " Hz" : "– Hz";
  $("#crc-state").textContent = s.crc_ok ? "ok" : "bad";
  $("#pill-crc").classList.toggle("on", !!s.crc_ok);
  $("#pill-crc").classList.toggle("warn", !s.crc_ok);
  $("#mode-machine").textContent = s.mode_machine;
  armEngaged = !!s.arm_engaged;
  const ap = $("#pill-arm");
  ap.classList.toggle("on", armEngaged);
  ap.classList.remove("warn");
  $("#arm-state").textContent = armEngaged ? "controlling" : "released";
  $("#pill-quest").classList.toggle("on", !!s.teleop_live);
  $("#quest-state").textContent = s.teleop_live ? "driving arms" : "idle";
  onMotionState(s);
  updateMotionButtons();
  updateHeadWarn(s);
}
// Head yaw is disabled in the bridge (its reading is offset ~100+ deg and
// varies between power-ons). Tell the user if the bridge ever reports otherwise.
function updateHeadWarn(s) {
  const w = $("#head-warn");
  const msg = s.head_yaw_enabled === false
    ? "Head yaw is disabled: never powered or commanded (its sensor reads ~" +
      (s.q[IDX.HEAD_YAW] * DEG).toFixed(0) + "° while facing forward). The 3D view always shows the head facing forward."
    : "";
  w.textContent = msg;
  w.classList.toggle("hidden", !msg);
}
function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

/* ---------------------------------------------------------------------- */
/* Command panel wiring                                                    */
/* ---------------------------------------------------------------------- */
function wireCommands() {
  const master = $("#master-toggle");
  const body = $("#cmd-body");
  const controls = ["#btn-engage", "#btn-sync", "#btn-send", "#btn-estop", "#weight", "#btn-home-save"]
    .map((s) => $(s));
  const presets = document.querySelectorAll(".preset");

  master.addEventListener("change", () => {
    masterOn = master.checked;
    if (masterOn) {
      const ok = confirm(
        "Enable ARM CONTROL?\n\nThis page will be able to command the physical " +
        "R1-A arms + head over rt/lowcmd. Make sure the workspace is clear and someone " +
        "is ready at the E-stop.");
      if (!ok) { master.checked = false; masterOn = false; return; }
    }
    body.classList.toggle("locked", !masterOn);
    controls.forEach((c) => (c.disabled = !masterOn));
    presets.forEach((p) => (p.disabled = !masterOn));
    setSlidersEnabled(masterOn && armEngaged);
    if (!masterOn && armEngaged) send({ type: "arm_release" });
    updateMotionButtons();
  });

  $("#btn-engage").addEventListener("click", () => {
    send({ type: "arm_engage" });   // takes low-level control, holds current pose
    setTimeout(() => { syncSlidersToState(); setSlidersEnabled(true); }, 400);
  });
  $("#btn-sync").addEventListener("click", syncSlidersToState);
  $("#btn-send").addEventListener("click", () => {
    if (!armEngaged) { alert("Engage the arms first (takes low-level control)."); return; }
    const spd = parseFloat($("#weight").value) || 0.2;
    send({ type: "arm_speed", value: spd });
    send({ type: "arm_set", q: targets.slice() });
    $("#log-line").textContent = "targets sent (speed " + spd.toFixed(2) + " rad/s)";
  });
  $("#btn-estop").addEventListener("click", () => {
    send({ type: "estop" });
    $("#weight").value = 0; $("#weight-val").textContent = "0.00";
    master.checked = false; masterOn = false;
    body.classList.add("locked");
    controls.forEach((c) => (c.disabled = true));
    presets.forEach((p) => (p.disabled = true));
  });

  const weight = $("#weight");   // repurposed as move-speed (rad/s)
  weight.addEventListener("input", () => {
    $("#weight-val").textContent = parseFloat(weight.value).toFixed(2);
    send({ type: "arm_speed", value: parseFloat(weight.value) });
  });

  presets.forEach((p) => p.addEventListener("click", () => applyPreset(p.dataset.preset)));
}

function applyPreset(name) {
  if (name === "sync") { syncSlidersToState(); return; }
  let pose;
  if (name === "zero") pose = new Array(12).fill(0);
  else if (name === "tpose")
    // arms abducted outward, elbows slightly bent; head neutral (12 joints)
    pose = [0, 1.3, 0, 0.3, 0,  0, -1.3, 0, 0.3, 0,  0, 0];
  else return;
  cmdSliderEls.forEach((c, k) => {
    c.input.value = pose[k]; targets[k] = pose[k];
    c.b.textContent = (pose[k] * DEG).toFixed(0) + "°";
  });
}

/* ---------------------------------------------------------------------- */
/* Scripted motions (home / wave) with a simulation preview                */
/* ---------------------------------------------------------------------- */
// Poses are the 12 commanded joints in ARM_CMD_IDX order. Signs come from the
// URDF FK: shoulder pitch < 0 swings the arm forward/up, elbow > 0 drops the
// forearm (at 0 the forearm already points straight forward).
// Home: upper arm a little forward, forearm parallel to the base plate, hands
// ~7 cm higher than the zero pose: lowest arm point 4 cm above the bottom of
// the base plate (checked against the STL meshes). Head untouched.
const HOME_DEFAULT = [-0.8, 0, 0, 0.8, 0,  -0.8, 0, 0, 0.8, 0];
const HOME_KEY = "magikid.home.v1";
// Wave: arm raised forward/out with the forearm vertical, then shoulder yaw
// swings the hand ~14 cm side to side.
const WAVE = { sp: -1.0, sr: 0.6, el: -0.5, yaw: 0.35, cycles: 3 };
const TRANSIT_VEL = 0.4, SWING_VEL = 0.8;    // planned peak rad/s at "Normal"
const MOTION_LABEL = { home: "Go home", wave_l: "Wave left hand", wave_r: "Wave right hand" };

function loadHome() {
  try {
    const v = JSON.parse(localStorage.getItem(HOME_KEY));
    if (Array.isArray(v) && v.length === 10 && v.every(Number.isFinite)) return v;
  } catch (_) {}
  return HOME_DEFAULT.slice();
}
let homePose = loadHome();

function clampPose(p) {
  return p.map((v, k) => Math.max(JOINTS[ARM_CMD_IDX[k]].min, Math.min(JOINTS[ARM_CMD_IDX[k]].max, v)));
}
function currentArmQ() {
  return latest && latest.q ? ARM_CMD_IDX.map((i) => latest.q[i]) : new Array(12).fill(0);
}
// The head yaw reading is untrusted, so the 3D view always draws the head forward.
function headForward(q) { const c = q.slice(); c[IDX.HEAD_YAW] = 0; return c; }
function to26(q12) {
  const q = latest && latest.q ? latest.q.slice() : new Array(26).fill(0);
  ARM_CMD_IDX.forEach((sIdx, k) => (q[sIdx] = q12[k]));
  return headForward(q);
}

// Build keyframes + segment durations. Same segments are sent to the bridge,
// which plays them with the same cosine easing, so the preview is what runs.
function buildMotion(name, start, speed) {
  const home = clampPose(homePose.concat(start.slice(10, 12)));   // keep head as is
  const keys = [], vel = [];
  const push = (p, v) => { keys.push(clampPose(p)); vel.push(v * speed); };
  if (name === "home") {
    push(home, TRANSIT_VEL);
  } else {
    const o = name === "wave_r" ? 5 : 0, side = name === "wave_r" ? -1 : 1;
    const up = home.slice();
    up[o] = WAVE.sp; up[o + 1] = side * WAVE.sr; up[o + 2] = 0; up[o + 3] = WAVE.el; up[o + 4] = 0;
    push(up, TRANSIT_VEL);
    for (let c = 0; c < WAVE.cycles; c++)
      for (const y of [WAVE.yaw, -WAVE.yaw]) { const p = up.slice(); p[o + 2] = y; push(p, SWING_VEL); }
    push(up, SWING_VEL);
    push(home, TRANSIT_VEL);
  }
  // cosine ease peaks at pi/2 * delta / T, so pick T to hit the planned speed
  const dur = []; let prev = start;
  keys.forEach((k, i) => {
    const d = Math.max(...k.map((v, j) => Math.abs(v - prev[j])));
    dur.push(Math.max(0.5, (Math.PI / 2) * d / vel[i]));
    prev = k;
  });
  return { name, start, keys, dur, total: dur.reduce((a, b) => a + b, 0) };
}

function motionQ(m, t) {
  let from = m.start;
  for (let i = 0; i < m.keys.length; i++) {
    if (t < m.dur[i]) {
      const e = 0.5 - 0.5 * Math.cos(Math.PI * t / m.dur[i]);
      return from.map((v, j) => v + (m.keys[i][j] - v) * e);
    }
    t -= m.dur[i]; from = m.keys[i];
  }
  return m.keys[m.keys.length - 1].slice();
}

// Sweep the planned motion through FK and report clearances.
function checkMotion(m) {
  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  const handsZ = (F) => Math.min(F.l_tip.p[2], F.r_tip.p[2], F.l_wr.p[2], F.r_wr.p[2]);
  const floorZ = Math.min(handsZ(computeFK(to26(m.start))), handsZ(computeFK(to26(m.keys[m.keys.length - 1])))) - 0.01;
  let minHands = Infinity, minHead = Infinity, minZ = Infinity, peak = 0;
  for (let t = 0; t <= m.total; t += 0.04) {
    const F = computeFK(to26(motionQ(m, t)));
    const headC = F.h_y.p.map((v, k) => (v + F.h_top.p[k]) / 2);
    minHands = Math.min(minHands, dist(F.l_tip.p, F.r_tip.p), dist(F.l_wr.p, F.r_wr.p));
    for (const h of [F.l_tip.p, F.r_tip.p, F.l_wr.p, F.r_wr.p]) minHead = Math.min(minHead, dist(h, headC));
    minZ = Math.min(minZ, handsZ(F));
  }
  let prev = m.start;
  m.keys.forEach((k, i) => { peak = Math.max(peak, (Math.PI / 2) * Math.max(...k.map((v, j) => Math.abs(v - prev[j]))) / m.dur[i]); prev = k; });
  const warn = [];
  if (minHands < 0.10) warn.push("hands come within " + (minHands * 100).toFixed(0) + " cm of each other");
  if (minHead < 0.15) warn.push("a hand comes within " + (minHead * 100).toFixed(0) + " cm of the head");
  if (minZ < floorZ) warn.push("hands dip " + ((floorZ + 0.01 - minZ) * 100).toFixed(0) + " cm below the start/end height");
  return { ok: !warn.length, warn, minHands, minHead, peak };
}

let preview = null;        // { m, t0 } while the sim is showing
let motionRunning = false; // bridge reports traj_active

function previewQ() {
  const T = preview.m.total;
  const el = (performance.now() - preview.t0) / 1000 % (T + 1.2);   // pause between loops
  const t = Math.min(el, T);
  $("#mp-prog-fill").style.width = (t / T * 100) + "%";
  return to26(motionQ(preview.m, t));
}

function openPreview(name) {
  const speed = parseFloat($("#motion-speed").value) || 1;
  const m = buildMotion(name, currentArmQ(), speed);
  const chk = checkMotion(m);
  preview = { m, t0: performance.now() };
  document.querySelectorAll(".motion").forEach((b) => b.classList.toggle("active", b.dataset.motion === name));
  $("#motion-preview").classList.remove("hidden");
  $("#sim-banner").classList.remove("hidden");
  document.querySelector(".viz-3d").classList.add("sim");
  $("#mp-name").textContent = MOTION_LABEL[name];
  $("#mp-dur").textContent = "· " + m.total.toFixed(1) + " s · peak " + chk.peak.toFixed(2) + " rad/s";
  const c = $("#mp-check");
  c.classList.toggle("bad", !chk.ok);
  c.textContent = chk.ok
    ? "✓ Sim check: hands ≥ " + (chk.minHands * 100).toFixed(0) + " cm apart, ≥ " + (chk.minHead * 100).toFixed(0) + " cm from head"
    : "⚠ " + chk.warn.join("; ");
  updateMotionButtons();
}

function closePreview() {
  preview = null;
  document.querySelectorAll(".motion").forEach((b) => b.classList.remove("active"));
  $("#sim-banner").classList.add("hidden");
  document.querySelector(".viz-3d").classList.remove("sim");
  if (!motionRunning) $("#motion-preview").classList.add("hidden");
}

function runMotion() {
  if (!preview) return;
  if (!(masterOn && armEngaged)) { alert("Turn on ARM CONTROL and press Engage first."); return; }
  const name = preview.m.name;
  // re-plan from the pose the robot is in right now
  const m = buildMotion(name, currentArmQ(), parseFloat($("#motion-speed").value) || 1);
  const ok = confirm("Run \"" + MOTION_LABEL[name] + "\" on the ROBOT?\n\n" +
    m.total.toFixed(1) + " s, as shown in the preview. Keep the area clear; " +
    "Stop motion / E-STOP are on this panel.");
  if (!ok) return;
  send({ type: "traj", name, dur: m.dur.map((d) => +d.toFixed(3)), q: [].concat(...m.keys).map((v) => +v.toFixed(4)) });
  preview = null;
  $("#sim-banner").classList.add("hidden");
  document.querySelector(".viz-3d").classList.remove("sim");
  $("#mp-name").textContent = MOTION_LABEL[name] + " — running on robot";
}

function updateMotionButtons() {
  const canRun = masterOn && armEngaged && !motionRunning;
  $("#mp-run").disabled = !canRun || !preview;
  $("#mp-run").title = canRun ? "" : "Turn on ARM CONTROL and Engage first";
  $("#mp-run").classList.toggle("hidden", motionRunning);
  $("#mp-stop").classList.toggle("hidden", !motionRunning);
}

function onMotionState(s) {
  const was = motionRunning;
  motionRunning = !!s.traj_active;
  if (motionRunning) {
    $("#motion-preview").classList.remove("hidden");
    $("#mp-prog-fill").style.width = ((s.traj_prog || 0) * 100) + "%";
  }
  if (was && !motionRunning) {                 // finished or stopped
    syncSlidersToState();
    if (!preview) $("#motion-preview").classList.add("hidden");
  }
  if (was !== motionRunning) updateMotionButtons();
}

function wireMotions() {
  document.querySelectorAll(".motion").forEach((b) =>
    b.addEventListener("click", () => openPreview(b.dataset.motion)));
  $("#motion-speed").addEventListener("change", () => { if (preview) openPreview(preview.m.name); });
  $("#mp-close").addEventListener("click", closePreview);
  $("#mp-run").addEventListener("click", runMotion);
  $("#mp-stop").addEventListener("click", () => send({ type: "traj_stop" }));
  $("#btn-home-save").addEventListener("click", () => {
    const p = clampPose(targets.slice()).slice(0, 10);
    if (!confirm("Save the current slider values (arms only) as the Home pose?")) return;
    homePose = p;
    try { localStorage.setItem(HOME_KEY, JSON.stringify(p)); } catch (_) {}
    $("#log-line").textContent = "home pose saved";
  });
  $("#btn-home-reset").addEventListener("click", () => {
    homePose = HOME_DEFAULT.slice();
    try { localStorage.removeItem(HOME_KEY); } catch (_) {}
    $("#log-line").textContent = "home pose reset to default";
  });
}

/* robot-mode (FSM) buttons ---------------------------------------------- */
function wireModeButtons() {
  const CONFIRM = {
    zerotorque: "Zero-Torque will make the robot go LIMP — it can collapse if standing.\n\nContinue?",
    damp: "Enter Damping mode? The robot becomes compliant (may sag if standing).",
    standup: "STAND UP — the robot will move to a standing posture.\n\nIs the area clear and the robot ready?",
    start: "Enter START / 运控 control mode?\n\nThe controller will actively drive the joints. Area clear?",
  };
  document.querySelectorAll(".mode-btn").forEach((b) => b.addEventListener("click", () => {
    const cmd = b.dataset.loco;
    if (CONFIRM[cmd] && !confirm(CONFIRM[cmd])) return;
    send({ type: "loco", cmd });
    $("#log-line").textContent = "loco: " + cmd + " sent";
  }));
}

/* tabs ------------------------------------------------------------------ */
function wireTabs() {
  document.querySelectorAll(".tab").forEach((t) => {
    t.addEventListener("click", () => {
      document.querySelectorAll(".tab").forEach((x) => x.classList.remove("active"));
      t.classList.add("active");
      const which = t.dataset.tab;
      $("#joint-arms").classList.toggle("hidden", which !== "arms");
      $("#joint-all").classList.toggle("hidden", which !== "all");
    });
  });
}

/* boot ------------------------------------------------------------------ */
window.addEventListener("DOMContentLoaded", () => {
  buildJointViews();
  buildCmdSliders();
  buildCameras();
  initViz3d();
  ["dial-waist", "dial-headyaw", "dial-headpitch"].forEach(initDial);
  initHorizon();
  wireCommands();
  wireMotions();
  wireTabs();
  connect();
  requestAnimationFrame(render);
});
