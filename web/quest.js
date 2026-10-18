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
const prevBtn = { l: [], r: [] };          // previous pressed state per button
const lastBuzz = { l: 0, r: 0 };
const hud = { dirty: true, link: false, log: "", warn: [], grip: { l: false, r: false } };

function readSettings() {
  core.scale = parseFloat($("#opt-scale").value);
  core.headPitch = $("#opt-head").checked;
}
const speed = () => parseFloat($("#opt-speed").value);

function freeze() {
  frozen = true; unfreezeArmed = false;
  core.release();
  send({ type: "teleop_stop" });
  hud.dirty = true;
}

/* ---------------------------------------------------------------------- */
/* three.js scene: HUD panel + ghost skeleton + controller markers         */
/* ---------------------------------------------------------------------- */
let renderer, scene, camera, hudMesh, hudCanvas, hudTex, ghost, ghostTarget, ghostLive;
const ctrlMesh = {};

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

  // Ghost robot: a group whose local frame IS the robot frame (+X fwd, +Y left, +Z up).
  ghost = new THREE.Group();
  ghost.matrixAutoUpdate = false;
  ghostLive = makeSkeleton(0x8b94a7);
  ghostTarget = makeSkeleton(0x24e0b5);
  ghost.add(ghostLive.obj, ghostTarget.obj);
  scene.add(ghost);

  for (const side of ["l", "r"]) {
    const m = new THREE.Mesh(new THREE.ConeGeometry(0.015, 0.06, 16),
      new THREE.MeshBasicMaterial({ color: side === "l" ? 0x4aa8ff : 0xffa14a }));
    m.geometry.rotateX(-Math.PI / 2);          // point along -Z like the target ray
    m.visible = false;
    scene.add(m);
    ctrlMesh[side] = m;
  }
}

// Bone list over computeFK frame names.
const BONES = [["root", "waist"], ["waist", "h_p"], ["h_p", "h_y"], ["h_y", "h_top"],
  ["waist", "l_sp"], ["l_sp", "l_sr"], ["l_sr", "l_sy"], ["l_sy", "l_el"], ["l_el", "l_wr"], ["l_wr", "l_tip"],
  ["waist", "r_sp"], ["r_sp", "r_sr"], ["r_sr", "r_sy"], ["r_sy", "r_el"], ["r_el", "r_wr"], ["r_wr", "r_tip"]];
function makeSkeleton(color) {
  const pos = new Float32Array(BONES.length * 6);
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  const obj = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color }));
  obj.frustumCulled = false;
  return { obj, pos, g };
}
function setSkeleton(sk, q26) {
  const F = computeFK(q26);
  F.root = { p: [0, 0, 0] };
  F.waist = { p: [0, 0, 0.19] };            // draw the spine up to shoulder height
  BONES.forEach(([a, b], i) => { sk.pos.set(F[a].p, i * 6); sk.pos.set(F[b].p, i * 6 + 3); });
  sk.g.attributes.position.needsUpdate = true;
}
const with12 = (q26, q12) => { const q = q26.slice(); ARM_CMD_IDX.forEach((i, k) => (q[i] = q12[k])); q[IDX.HEAD_YAW] = 0; return q; };

// Place HUD and ghost in front of the operator (world-locked), using M = [f, l, u].
function placeWorld(headPos) {
  const [f, l, u] = core.M;
  const at = (fw, lf, up) => new THREE.Vector3(
    headPos[0] + f[0]*fw + l[0]*lf + u[0]*up,
    headPos[1] + f[1]*fw + l[1]*lf + u[1]*up,
    headPos[2] + f[2]*fw + l[2]*lf + u[2]*up);
  hudMesh.position.copy(at(0.6, -0.32, -0.15));                  // front-right, a bit low
  hudMesh.lookAt(new THREE.Vector3(headPos[0], headPos[1], headPos[2]));
  const m = new THREE.Matrix4().makeBasis(
    new THREE.Vector3(f[0], f[1], f[2]), new THREE.Vector3(l[0], l[1], l[2]), new THREE.Vector3(u[0], u[1], u[2]));
  m.setPosition(at(0.75, 0.0, -0.75));                           // ghost waist below eye level
  ghost.matrix.copy(m);
  ghost.matrixWorldNeedsUpdate = true;
}

function drawHud() {
  const c = hudCanvas.getContext("2d"), W = hudCanvas.width, H = hudCanvas.height;
  c.clearRect(0, 0, W, H);
  c.fillStyle = "rgba(12,14,20,0.82)"; c.fillRect(0, 0, W, H);
  c.strokeStyle = frozen ? "#ff4d5e" : "#262c3b"; c.lineWidth = 6; c.strokeRect(3, 3, W - 6, H - 6);
  c.font = "bold 34px system-ui, sans-serif";
  const line = (y, txt, col) => { c.fillStyle = col; c.fillText(txt, 24, y); };
  if (frozen) line(52, "FROZEN — release grips, grab to resume", "#ff4d5e");
  else if (!hud.link) line(52, "NO LINK to robot", "#ff4d5e");
  else if (!engaged) line(52, "Arms NOT engaged (use dashboard)", "#ffb020");
  else line(52, "Teleop ready", "#24e0b5");
  c.font = "30px system-ui, sans-serif";
  const st = (s) => (hud.grip[s] ? (core.blocked[s] ? "BLOCKED" : "moving") : "free");
  line(110, "Left: " + st("l") + "    Right: " + st("r"), "#e7ebf3");
  line(160, "scale " + core.scale + "×   speed " + speed() + " rad/s   head " + (core.headPitch ? "on" : "off"), "#8b94a7");
  c.font = "26px system-ui, sans-serif";
  if (hud.warn.length) line(215, "⚠ " + hud.warn.slice(0, 2).join(", "), "#ffb020");
  line(268, (hud.log || "").slice(0, 44), "#5c6577");
  line(330, "grip = move · B/Y = freeze · stick click = recentre", "#5c6577");
  hudTex.needsUpdate = true;
  hud.dirty = false;
}

/* ---------------------------------------------------------------------- */
/* XR frame loop                                                           */
/* ---------------------------------------------------------------------- */
const v3 = (p) => [p.x, p.y, p.z];
const q4 = (o) => [o.x, o.y, o.z, o.w];
let placed = false, lastHudDraw = 0;

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
    if (edge(5)) freeze();                                          // B / Y
    if (edge(3) && headQ) { core.recenter(headQ); placeWorld(headP); pulse(src, 0.4, 40); }   // stick click
    prevBtn[side] = b;
    const grip = !!b[1];
    anyGrip = anyGrip || grip;
    if (pose) {
      const m = ctrlMesh[side];
      m.visible = true;
      m.position.set(pose.transform.position.x, pose.transform.position.y, pose.transform.position.z);
      m.quaternion.set(pose.transform.orientation.x, pose.transform.orientation.y, pose.transform.orientation.z, pose.transform.orientation.w);
      m.material.color.set(grip && engaged && !frozen ? (core.blocked[side] ? 0xff4d5e : 0x24e0b5) : (side === "l" ? 0x4aa8ff : 0xffa14a));
      const active = grip && engaged && !frozen;
      if (active && !hud.grip[side]) pulse(src, 0.3, 25);           // grabbed
      if (hud.grip[side] !== active) { hud.grip[side] = active; hud.dirty = true; }
      inp[side] = { pos: v3(pose.transform.position), quat: q4(pose.transform.orientation), grip: active, src };
    }
  }
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
    setSkeleton(ghostLive, with12(inp.q26, ARM_CMD_IDX.map((i) => inp.q26[i])));
    setSkeleton(ghostTarget, with12(inp.q26, out.q));
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
  ["#opt-scale", "#opt-speed", "#opt-head"].forEach((s) => $(s).addEventListener("change", () => { readSettings(); hud.dirty = true; }));
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
