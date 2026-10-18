/* Magikid R1-A — Meta Quest teleop page (WebXR + three.js r128) ----------
   Reads both controllers every XR frame, turns them into 12 joint targets with
   TeleopMath.TeleopCore (teleop.js), and streams {"type":"teleop"} to the
   bridge at ~30 Hz. Safety lives in three places: this page refuses unsafe
   poses (kin.js checkPose), the server only lets this listener send teleop /
   teleop_stop (not engage), and the bridge clamps, rate-limits and holds the
   pose if frames stop for 0.3 s. */
"use strict";

const SEND_MS = 33;                       // ~30 Hz to the bridge
const $ = (s) => document.querySelector(s);
const KEY = new URLSearchParams(location.search).get("key") || "";

let latest = null, engaged = false, everOpen = false;
let ws = null;

/* ---------------------------------------------------------------------- */
/* WebSocket                                                               */
/* ---------------------------------------------------------------------- */
function connect() {
  const url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host +
              "/ws?key=" + encodeURIComponent(KEY);
  ws = new WebSocket(url);
  ws.onopen = () => { everOpen = true; setLink(true); $("#key-err").classList.add("hidden"); };
  ws.onclose = () => {
    setLink(false);
    if (!everOpen) $("#key-err").classList.remove("hidden");   // 403 = missing/wrong key
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    let o; try { o = JSON.parse(ev.data); } catch (_) { return; }
    if (o.type === "state") onState(o);
    else if (o.type === "log") { $("#log-line").textContent = o.msg; hud.log = o.msg; hud.dirty = true; }
  };
}
function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

function setLink(on) {
  $("#pill-link").classList.toggle("on", on);
  $("#pill-link").classList.toggle("danger", !on);
  $("#link-state").textContent = on ? "connected" : "no link";
  hud.link = on; hud.dirty = true;
}
function onState(s) {
  latest = s;
  const was = engaged;
  engaged = !!s.arm_engaged;
  if (was !== engaged) { core.release(); hud.dirty = true; }
  $("#pill-arm").classList.toggle("on", engaged);
  $("#pill-arm").classList.toggle("warn", !engaged);
  $("#arm-state").textContent = engaged ? "engaged" : "not engaged";
  $("#engage-note").classList.toggle("hidden", engaged);
  $("#pill-tele").classList.toggle("on", !!s.teleop_live);
  $("#tele-state").textContent = s.teleop_live ? "live" : "idle";
}

/* ---------------------------------------------------------------------- */
/* Teleop state                                                            */
/* ---------------------------------------------------------------------- */
const core = new TeleopMath.TeleopCore();
let frozen = false, unfreezeArmed = false, lastSend = 0, xrMode = null;
// B/Y: tap = FREEZE (hold pose); keep holding ESTOP_HOLD_MS = E-STOP (release the arms).
const ESTOP_HOLD_MS = 1000;
const stopHold = { l: null, r: null };     // time B/Y went down, per hand
let estopSentAt = -1e9;
const prevBtn = { l: [], r: [] };          // previous pressed state per button
const lastBuzz = { l: 0, r: 0 };
const hud = { dirty: true, link: false, log: "", warn: [], grip: { l: false, r: false }, hold: 0 };

const ARMS_LABEL = { both: "both arms", l: "left arm only", r: "right arm only" };
const SETTINGS_KEY = "magikid.quest.v1";
function readSettings() {
  core.scale = parseFloat($("#opt-scale").value);
  core.headPitch = $("#opt-head").checked;
  core.arms = $("#opt-arms").value;
  try {
    localStorage.setItem(SETTINGS_KEY, JSON.stringify({ scale: $("#opt-scale").value,
      speed: $("#opt-speed").value, head: $("#opt-head").checked, arms: core.arms }));
  } catch (_) {}
}
function loadSettings() {             // remembered per headset; defaults if storage is unavailable
  try {
    const v = JSON.parse(localStorage.getItem(SETTINGS_KEY));
    if (!v) return;
    for (const [id, k] of [["#opt-scale", "scale"], ["#opt-speed", "speed"], ["#opt-arms", "arms"]])
      if ([...$(id).options].some((o) => o.value === v[k])) $(id).value = v[k];
    $("#opt-head").checked = !!v.head;
  } catch (_) {}
}
// A / X: both -> left only -> right only
function cycleArms() {
  const order = ["both", "l", "r"];
  $("#opt-arms").value = order[(order.indexOf(core.arms) + 1) % order.length];
  readSettings();
  hud.dirty = true;
}
const speed = () => parseFloat($("#opt-speed").value);

function freeze() {
  frozen = true; unfreezeArmed = false;
  core.release();
  send({ type: "teleop_stop" });
  hud.dirty = true;
}

/* ---------------------------------------------------------------------- */
/* three.js scene: HUD panel, the "YOU | ROBOT" stage, controller markers  */
/* ---------------------------------------------------------------------- */
let renderer, scene, camera, hudMesh, hudCanvas, hudTex, raycaster;
let stage, backdrop, robotLive, robotTarget, reachGap, human;
const handTgt = {}, handColor = { l: 0x4aa8ff, r: 0xffa14a };
const ctrlMesh = {}, rayLine = {};
const drag = { l: null, r: null };        // { what: "hud"|"stage", dist, off } while the trigger drags
const RAY_LEN = 1.5;
// The stage's own frame is the robot frame (+X away from you, +Y your left, +Z up),
// drawn at STAGE_SCALE. The robot stands at the origin, your figure HUMAN_Y to its left.
const STAGE_SCALE = 0.5, HUMAN_Y = 0.62, HUMAN_HEAD_Z = 0.42, HUMAN_SCALE = 0.6;
const BACK = { x: 0.32, y0: -0.42, y1: 1.02, z0: -0.22, z1: 0.62 };   // backdrop rectangle
const COLOR = { live: 0x9aa3b5, target: 0x24e0b5, l: 0x4aa8ff, r: 0xffa14a, body: 0xd5dbe6,
                blocked: 0xff4d5e, preview: 0xffb020 };
// Flat things on the stage face you: plane +X -> your right (stage -Y), +Y -> up, normal -> you.
const FACE_YOU = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(
  new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(-1, 0, 0)));
// Robot bones over computeFK frame names.
const ROBOT_BONES = [["root", "waist"], ["waist", "h_p"], ["h_p", "h_y"], ["h_y", "h_top"],
  ["waist", "l_sp"], ["l_sp", "l_sr"], ["l_sr", "l_sy"], ["l_sy", "l_el"], ["l_el", "l_wr"], ["l_wr", "l_tip"],
  ["waist", "r_sp"], ["r_sp", "r_sr"], ["r_sr", "r_sy"], ["r_sy", "r_el"], ["r_el", "r_wr"], ["r_wr", "r_tip"]];

function initThree() {
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.xr.enabled = true;
  renderer.xr.setReferenceSpaceType("local-floor");
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(70, 1, 0.02, 20);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 1.0));

  hudCanvas = document.createElement("canvas");
  hudCanvas.width = 640; hudCanvas.height = 360;
  hudTex = new THREE.CanvasTexture(hudCanvas);
  hudMesh = new THREE.Mesh(new THREE.PlaneGeometry(0.40, 0.225),
    new THREE.MeshBasicMaterial({ map: hudTex, transparent: true }));
  scene.add(hudMesh);

  initStage();
  raycaster = new THREE.Raycaster();

  for (const side of ["l", "r"]) {
    const m = new THREE.Mesh(new THREE.ConeGeometry(0.015, 0.06, 16),
      new THREE.MeshBasicMaterial({ color: side === "l" ? 0x4aa8ff : 0xffa14a }));
    m.geometry.rotateX(-Math.PI / 2);          // point along -Z like the target ray
    m.visible = false;
    // pointing ray (local -Z), used to grab the panel / stage with the trigger
    const ray = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -1)]),
      new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.45 }));
    ray.scale.z = RAY_LEN;
    m.add(ray);
    scene.add(m);
    ctrlMesh[side] = m; rayLine[side] = ray;
  }
}

// The stage: a dark backdrop (so the figures don't blend into passthrough) with
// your figure (YOU) and the robot (ROBOT: gray = now, green = target) side by side.
function initStage() {
  stage = new THREE.Group();
  stage.matrixAutoUpdate = false;
  scene.add(stage);
  const bw = BACK.y1 - BACK.y0, bh = BACK.z1 - BACK.z0, cy = (BACK.y0 + BACK.y1) / 2;
  backdrop = new THREE.Mesh(new THREE.PlaneGeometry(bw, bh),
    new THREE.MeshBasicMaterial({ color: 0x0c0e14, transparent: true, opacity: 0.88, side: THREE.DoubleSide }));
  backdrop.quaternion.copy(FACE_YOU);
  backdrop.position.set(BACK.x, cy, (BACK.z0 + BACK.z1) / 2);
  stage.add(backdrop);
  stage.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(
    [[BACK.y0, BACK.z0], [BACK.y1, BACK.z0], [BACK.y1, BACK.z1], [BACK.y0, BACK.z1]]
      .map(([y, z]) => new THREE.Vector3(BACK.x - 0.002, y, z))),
    new THREE.LineBasicMaterial({ color: COLOR.target })));
  // labels along the top edge
  const lc = document.createElement("canvas"); lc.width = 1024; lc.height = 96;
  const g = lc.getContext("2d");
  const u = (y) => ((BACK.y1 - y) / bw) * lc.width;          // stage y -> canvas x
  g.textAlign = "center";
  g.font = "bold 44px system-ui, sans-serif"; g.fillStyle = "#e7ebf3";
  g.fillText("YOU", u(HUMAN_Y), 48); g.fillText("ROBOT", u(0), 48);
  g.font = "26px system-ui, sans-serif"; g.fillStyle = "#8b94a7";
  g.fillText("gray = now · green = target · ball = your hand", u(0), 86);
  const lh = bw * lc.height / lc.width;
  const label = new THREE.Mesh(new THREE.PlaneGeometry(bw, lh),
    new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(lc), transparent: true }));
  label.quaternion.copy(FACE_YOU);
  label.position.set(BACK.x - 0.004, cy, BACK.z1 - lh / 2 - 0.01);
  stage.add(label);

  robotLive = makeBones(ROBOT_BONES.length, 0.008, COLOR.live, 1);
  robotTarget = makeBones(ROBOT_BONES.length, 0.012, COLOR.target, 0.7);
  reachGap = makeBones(2, 0.004, 0xffffff, 0.6);
  for (const s of ["l", "r"]) handTgt[s] = makeBall(0.018, COLOR[s]);
  human = {
    torso: makeBones(3, 0.014, COLOR.body, 1), head: makeBall(0.05, COLOR.body),
    arm: { l: makeBones(2, 0.012, COLOR.l, 1), r: makeBones(2, 0.012, COLOR.r, 1) },
    hand: { l: makeBall(0.024, COLOR.l), r: makeBall(0.024, COLOR.r) },
  };
}
// Cylinders between point pairs (WebGL lines are 1 px wide, unreadable in passthrough).
function makeBones(n, radius, color, opacity) {
  const mat = new THREE.MeshBasicMaterial({ color, transparent: opacity < 1, opacity });
  const geo = new THREE.CylinderGeometry(radius, radius, 1, 8);
  const meshes = [];
  for (let i = 0; i < n; i++) { const m = new THREE.Mesh(geo, mat); m.visible = false; stage.add(m); meshes.push(m); }
  return { meshes, mat };
}
function makeBall(r, color) {
  const m = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), new THREE.MeshBasicMaterial({ color }));
  m.visible = false;
  stage.add(m);
  return m;
}
const Y_AXIS = new THREE.Vector3(0, 1, 0);
function setBone(m, a, b) {
  const A = new THREE.Vector3(a[0], a[1], a[2]), d = new THREE.Vector3(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
  const len = d.length();
  m.visible = len > 1e-4;
  if (!m.visible) return;
  m.position.copy(A).addScaledVector(d, 0.5);
  m.quaternion.setFromUnitVectors(Y_AXIS, d.divideScalar(len));
  m.scale.set(1, len, 1);
}
const with12 = (q26, q12) => { const q = q26.slice(); ARM_CMD_IDX.forEach((i, k) => (q[i] = q12[k])); q[IDX.HEAD_YAW] = 0; return q; };

function drawRobot(set, q26) {
  const F = computeFK(q26);
  F.root = { p: [0, 0, 0] };
  F.waist = { p: [0, 0, 0.19] };            // draw the spine up to shoulder height
  ROBOT_BONES.forEach(([a, b], i) => setBone(set.meshes[i], F[a].p, F[b].p));
  return F;
}
// Two-bone arm with the elbow bent down and outward (only for drawing your figure).
function elbowOf(S, Hd, a, side) {
  const d = [Hd[0] - S[0], Hd[1] - S[1], Hd[2] - S[2]], L = Math.hypot(...d) || 1e-6;
  const n = d.map((v) => v / L), Lc = Math.min(L, 2 * a - 1e-4);
  const h = Math.sqrt(Math.max(0, a * a - (Lc / 2) * (Lc / 2)));
  let pole = [0, side === "l" ? 0.4 : -0.4, -1];
  const k = pole[0] * n[0] + pole[1] * n[1] + pole[2] * n[2];
  pole = pole.map((v, i) => v - k * n[i]);
  const pl = Math.hypot(...pole) || 1;
  return [0, 1, 2].map((i) => S[i] + n[i] * (L / 2) + (pole[i] / pl) * h);
}
// Your figure from the headset + controllers, in your own frame (same axes as the
// robot), scaled by HUMAN_SCALE and drawn next to the robot. Shoulders are estimated.
function drawHuman(inp, headP) {
  if (!headP) return;
  const s = HUMAN_SCALE, H = [0, HUMAN_Y, HUMAN_HEAD_Z];
  const rel = (p) => {
    const r = TeleopMath.toRobot(core.M, [p[0] - headP[0], p[1] - headP[1], p[2] - headP[2]]);
    return [H[0] + s * r[0], H[1] + s * r[1], H[2] + s * r[2]];
  };
  const chest = [H[0], H[1], H[2] - s * 0.25], hip = [H[0], H[1], H[2] - s * 0.75];
  const sh = { l: [chest[0], chest[1] + s * 0.19, chest[2]], r: [chest[0], chest[1] - s * 0.19, chest[2]] };
  human.head.visible = true;
  human.head.position.set(H[0], H[1], H[2]);
  setBone(human.torso.meshes[0], [H[0], H[1], H[2] - 0.05], chest);
  setBone(human.torso.meshes[1], sh.l, sh.r);
  setBone(human.torso.meshes[2], chest, hip);
  for (const side of ["l", "r"]) {
    const c = inp[side], arm = human.arm[side], hand = human.hand[side];
    hand.visible = !!c;
    if (!c) { arm.meshes.forEach((m) => (m.visible = false)); continue; }
    const hp = rel(c.pos), el = elbowOf(sh[side], hp, s * 0.30, side);
    setBone(arm.meshes[0], sh[side], el);
    setBone(arm.meshes[1], el, hp);
    hand.position.set(hp[0], hp[1], hp[2]);
    hand.material.color.set(handColor[side]);
  }
}
function updateStage(inp, out, headP) {
  drawRobot(robotLive, with12(inp.q26, ARM_CMD_IDX.map((i) => inp.q26[i])));
  const F = drawRobot(robotTarget, with12(inp.q26, out.q));
  // where your hand is asking the robot hand to go, and the gap if it can't get there
  ["l", "r"].forEach((s, i) => {
    const t = core.tgt[s], ball = handTgt[s];
    ball.visible = !!t;
    reachGap.meshes[i].visible = false;
    if (!t) return;
    ball.position.set(t[0], t[1], t[2]);
    ball.material.color.set(core.blocked[s] ? COLOR.blocked : COLOR[s]);
    const tip = F[s + "_tip"].p;
    if (Math.hypot(t[0] - tip[0], t[1] - tip[1], t[2] - tip[2]) > 0.01) setBone(reachGap.meshes[i], tip, t);
  });
  drawHuman(inp, headP);
}

// Place the HUD and the stage in front of the operator (world-locked), using M = [f, l, u]:
// stage front-left, status panel front-right, the middle left clear to see the real robot.
function placeWorld(headPos) {
  const [f, l, u] = core.M;
  const at = (fw, lf, up) => new THREE.Vector3(
    headPos[0] + f[0]*fw + l[0]*lf + u[0]*up,
    headPos[1] + f[1]*fw + l[1]*lf + u[1]*up,
    headPos[2] + f[2]*fw + l[2]*lf + u[2]*up);
  hudMesh.position.copy(at(0.62, -0.34, -0.10));                 // front-right
  hudMesh.lookAt(new THREE.Vector3(headPos[0], headPos[1], headPos[2]));
  const m = new THREE.Matrix4().makeBasis(
    new THREE.Vector3(f[0], f[1], f[2]), new THREE.Vector3(l[0], l[1], l[2]), new THREE.Vector3(u[0], u[1], u[2]));
  m.scale(new THREE.Vector3(STAGE_SCALE, STAGE_SCALE, STAGE_SCALE));
  // backdrop centre ~0.5 m to your left, a bit below eye level
  m.setPosition(at(0.9, 0.5 - STAGE_SCALE * (BACK.y0 + BACK.y1) / 2, -0.36));
  stage.matrix.copy(m);
  stage.matrixWorldNeedsUpdate = true;
}

function drawHud() {
  const c = hudCanvas.getContext("2d"), W = hudCanvas.width, H = hudCanvas.height;
  c.clearRect(0, 0, W, H);
  c.fillStyle = "rgba(12,14,20,0.82)"; c.fillRect(0, 0, W, H);
  c.strokeStyle = frozen ? "#ff4d5e" : "#262c3b"; c.lineWidth = 6; c.strokeRect(3, 3, W - 6, H - 6);
  c.font = "bold 34px system-ui, sans-serif";
  const line = (y, txt, col) => { c.fillStyle = col; c.fillText(txt, 24, y); };
  if (hud.hold > 0) line(52, "Keep holding for E-STOP… " + (hud.hold / 1000).toFixed(1) + " s", "#ff4d5e");
  else if (performance.now() - estopSentAt < 4000) line(52, "E-STOP sent — arms released", "#ff4d5e");
  else if (frozen) line(52, "FROZEN — release grips, grab to resume", "#ff4d5e");
  else if (!hud.link) line(52, "NO LINK to robot", "#ff4d5e");
  else if (!engaged) line(52, "PREVIEW: not engaged, robot won't move", "#ffb020");
  else line(52, "Teleop ready", "#24e0b5");
  c.font = "30px system-ui, sans-serif";
  const st = (s) => (core.arms !== "both" && core.arms !== s ? "off"
    : hud.grip[s] ? (core.blocked[s] ? "BLOCKED" : engaged ? "moving" : "preview") : "free");
  line(110, "Left: " + st("l") + "    Right: " + st("r") + "    (" + ARMS_LABEL[core.arms] + ")", "#e7ebf3");
  line(160, "scale " + core.scale + "×   speed " + speed() + " rad/s   head " + (core.headPitch ? "on" : "off"), "#8b94a7");
  c.font = "26px system-ui, sans-serif";
  if (hud.warn.length) line(215, "⚠ " + hud.warn.slice(0, 2).join(", "), "#ffb020");
  line(268, (hud.log || "").slice(0, 44), "#5c6577");
  c.font = "23px system-ui, sans-serif";
  line(330, "grip: move · trigger: drag · A/X: arms · B/Y: freeze (hold: E-STOP)", "#5c6577");
  hudTex.needsUpdate = true;
  hud.dirty = false;
}

/* ---------------------------------------------------------------------- */
/* XR frame loop                                                           */
/* ---------------------------------------------------------------------- */
const v3 = (p) => [p.x, p.y, p.z];
const q4 = (o) => [o.x, o.y, o.z, o.w];
let placed = false, lastHudDraw = 0;

// Anchor point of a draggable (HUD centre, or the stage origin = the robot's root).
function anchorOf(what) {
  return what === "hud" ? hudMesh.position.clone() : new THREE.Vector3().setFromMatrixPosition(stage.matrix);
}
function moveTo(what, p, headP) {
  if (what === "hud") {
    hudMesh.position.copy(p);
    hudMesh.lookAt(new THREE.Vector3(headP[0], headP[1], headP[2]));
  } else {
    stage.matrix.setPosition(p);
    stage.matrixWorldNeedsUpdate = true;
  }
}
// Trigger drags whatever the ray points at; thumbstick up/down pushes it away / pulls it in.
function updateDrag(side, src, pose, trig, trigEdge, headP) {
  const o = pose.transform.position, q = pose.transform.orientation;
  const origin = new THREE.Vector3(o.x, o.y, o.z);
  const dir = new THREE.Vector3(0, 0, -1).applyQuaternion(new THREE.Quaternion(q.x, q.y, q.z, q.w));
  const d = drag[side];
  if (d && !trig) drag[side] = null;
  if (drag[side]) {
    const ay = (src.gamepad.axes && src.gamepad.axes[3]) || 0;
    if (Math.abs(ay) > 0.2) d.dist = Math.max(0.2, Math.min(3, d.dist - ay * 0.02));
    moveTo(d.what, origin.clone().addScaledVector(dir, d.dist).add(d.off), headP);
    rayLine[side].scale.z = d.dist;
    return;
  }
  raycaster.set(origin, dir);
  const hit = raycaster.intersectObjects([hudMesh, backdrop])[0];
  rayLine[side].scale.z = hit ? hit.distance : RAY_LEN;
  rayLine[side].material.opacity = hit ? 0.9 : 0.45;
  if (hit && trigEdge && headP) {
    const what = hit.object === hudMesh ? "hud" : "stage";
    drag[side] = { what, dist: hit.distance, off: anchorOf(what).sub(hit.point) };
    pulse(src, 0.3, 20);
  }
}

function onXRFrame(t, frame) {
  if (!frame) return;
  const ref = renderer.xr.getReferenceSpace();
  const vp = frame.getViewerPose(ref);
  let headQ = null, headP = null;
  if (vp) { headQ = q4(vp.transform.orientation); headP = v3(vp.transform.position); }
  if (headQ && !placed) { core.recenter(headQ); placeWorld(headP); placed = true; }

  const inp = { q26: latest && latest.q ? latest.q : null,
                qcmd: engaged && latest ? latest.q_cmd : null, head: headQ, l: null, r: null };
  let anyGrip = false;
  for (const src of frame.session.inputSources) {
    const side = src.handedness === "left" ? "l" : src.handedness === "right" ? "r" : null;
    if (!side || !src.gamepad) continue;           // hand tracking has no gamepad: ignored
    const pose = frame.getPose(src.targetRaySpace, ref);
    const b = src.gamepad.buttons.map((x) => !!(x && x.pressed));
    const edge = (i) => b[i] && !prevBtn[side][i];
    if (edge(5)) { freeze(); stopHold[side] = t; }                  // B / Y: freeze now...
    if (!b[5]) stopHold[side] = null;
    else if (stopHold[side] !== null && t - stopHold[side] >= ESTOP_HOLD_MS) {
      send({ type: "estop" });                                        // ...held 1 s: E-STOP
      estopSentAt = performance.now();
      stopHold[side] = null;
      pulse(src, 1.0, 300);
      hud.dirty = true;
    }
    if (edge(4)) { cycleArms(); pulse(src, 0.3, 30); }              // A / X
    if (edge(3) && headQ) { core.recenter(headQ); placeWorld(headP); pulse(src, 0.4, 40); }   // stick click
    const trigEdge = edge(0);
    prevBtn[side] = b;
    if (pose) updateDrag(side, src, pose, b[0], trigEdge, headP);
    const grip = !!b[1];
    anyGrip = anyGrip || grip;
    if (pose) {
      const m = ctrlMesh[side];
      m.visible = true;
      m.position.set(pose.transform.position.x, pose.transform.position.y, pose.transform.position.z);
      m.quaternion.set(pose.transform.orientation.x, pose.transform.orientation.y, pose.transform.orientation.z, pose.transform.orientation.w);
      // Not engaged = PREVIEW: the grip still drives the green ghost, but nothing is sent.
      const active = grip && !frozen && !drag[side] &&             // dragging the panel: arm holds
                     (core.arms === "both" || core.arms === side);
      handColor[side] = !active ? COLOR[side]
        : core.blocked[side] ? COLOR.blocked : engaged ? COLOR.target : COLOR.preview;
      m.material.color.set(handColor[side]);
      if (active && !hud.grip[side]) pulse(src, 0.3, 25);           // grabbed
      if (hud.grip[side] !== active) { hud.grip[side] = active; hud.dirty = true; }
      inp[side] = { pos: v3(pose.transform.position), quat: q4(pose.transform.orientation), grip: active, src };
    }
  }
  const held = ["l", "r"].filter((k) => stopHold[k] !== null).map((k) => ESTOP_HOLD_MS - (t - stopHold[k]));
  const hold = held.length ? Math.max(0, Math.min(...held)) : 0;
  if (hold !== hud.hold) { hud.hold = hold; hud.dirty = true; }
  if (frozen && !anyGrip) unfreezeArmed = true;
  if (frozen && unfreezeArmed && anyGrip) { frozen = false; hud.dirty = true; }

  if (inp.q26) {
    const out = core.update(inp);
    for (const side of ["l", "r"])
      if (inp[side] && inp[side].grip && core.blocked[side] && t - lastBuzz[side] > 200) {
        pulse(inp[side].src, 0.6, 30); lastBuzz[side] = t;
      }
    if (out.warn.join() !== hud.warn.join()) { hud.warn = out.warn; hud.dirty = true; }
    // engaged + not frozen: stream (all-zero mask = heartbeat that feeds the watchdog)
    if (engaged && !frozen && t - lastSend >= SEND_MS) {
      send({ type: "teleop", q: out.q.map((v) => +v.toFixed(4)), mask: out.mask, vel: speed() });
      lastSend = t;
    }
    updateStage(inp, out, headP);
  }
  if (hud.dirty && t - lastHudDraw > 100) { drawHud(); lastHudDraw = t; }
  renderer.render(scene, camera);
}

function pulse(src, intensity, ms) {
  const h = src && src.gamepad && src.gamepad.hapticActuators && src.gamepad.hapticActuators[0];
  if (h && h.pulse) h.pulse(intensity, ms).catch(() => {});
}

async function enterXR(mode) {
  readSettings();
  $("#xr-err").classList.add("hidden");
  try {
    const session = await navigator.xr.requestSession(mode, { optionalFeatures: ["local-floor"] });
    xrMode = mode;
    renderer.setClearColor(0x0c0e14, mode === "immersive-ar" ? 0 : 1);
    session.addEventListener("end", () => {
      send({ type: "teleop_stop" });
      core.release(); placed = false; xrMode = null;
      renderer.setAnimationLoop(null);
    });
    await renderer.xr.setSession(session);
    frozen = false; hud.dirty = true;
    renderer.setAnimationLoop(onXRFrame);
  } catch (e) {
    $("#xr-err").textContent = "Could not start " + mode + ": " + e.message;
    $("#xr-err").classList.remove("hidden");
  }
}

/* ---------------------------------------------------------------------- */
/* boot                                                                    */
/* ---------------------------------------------------------------------- */
window.addEventListener("DOMContentLoaded", async () => {
  connect();
  try { initThree(); } catch (e) {
    $("#xr-err").textContent = "3D rendering unavailable (" + e.message + ").";
    $("#xr-err").classList.remove("hidden");
    return;
  }
  loadSettings();
  readSettings();
  ["#opt-scale", "#opt-speed", "#opt-head", "#opt-arms"].forEach((s) => $(s).addEventListener("change", () => { readSettings(); hud.dirty = true; }));
  $("#btn-ar").addEventListener("click", () => enterXR("immersive-ar"));
  $("#btn-vr").addEventListener("click", () => enterXR("immersive-vr"));
  if (!navigator.xr) {
    $("#xr-err").textContent = window.isSecureContext
      ? "WebXR is not available in this browser. Open this page in the Meta Quest Browser."
      : "WebXR needs HTTPS. Open the https:// link from scripts/quest_tunnel.sh.";
    $("#xr-err").classList.remove("hidden");
    return;
  }
  const [ar, vr] = await Promise.all(["immersive-ar", "immersive-vr"].map((m) =>
    navigator.xr.isSessionSupported(m).catch(() => false)));
  $("#btn-ar").disabled = !ar;
  $("#btn-vr").disabled = !vr;
  if (!ar && !vr) { $("#xr-err").textContent = "This device supports no immersive WebXR mode."; $("#xr-err").classList.remove("hidden"); }
});
