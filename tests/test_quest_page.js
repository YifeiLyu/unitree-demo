// Drives web/quest.js against a mock WebXR session + stub three.js / DOM, to
// check the glue the headset would exercise: buttons, freeze, what gets sent.
// Run: node tests/test_quest_page.js
"use strict";
const assert = require("assert");
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const WEB = path.join(__dirname, "..", "web");
const HOME = [-0.8, 0, 0, 0.8, 0, -0.8, 0, 0, 0.8, 0, 0, 0];
const READY = [-0.6, 0.3, 0, -0.2, 0, -0.6, -0.3, 0, -0.2, 0, 0, 0];

// --- DOM stubs ---
const els = {};
function el(id) {
  return els[id] || (els[id] = {
    id, textContent: "", value: "", checked: false, disabled: false, handlers: {},
    classList: { s: new Set(["hidden"]), add(c) { this.s.add(c); }, remove(c) { this.s.delete(c); },
                 toggle(c, on) { if (on === undefined ? !this.s.has(c) : on) this.s.add(c); else this.s.delete(c); },
                 contains(c) { return this.s.has(c); } },
    addEventListener(ev, fn) { this.handlers[ev] = fn; },
  });
}
el("opt-scale").value = "0.7"; el("opt-speed").value = "0.7"; el("opt-arms").value = "both"; el("opt-track").value = "hand";
const ctx2d = new Proxy({}, { get: () => () => {} });
const docHandlers = {};
const document = {
  querySelector: (s) => el(s.replace(/^#/, "")),
  createElement: () => ({ width: 0, height: 0, getContext: () => ctx2d }),
};
// --- WebSocket mock ---
const sent = [];
let sock = null;
class FakeWS {
  constructor(url) { this.url = url; this.readyState = 1; sock = this; setImmediate(() => this.onopen && this.onopen()); }
  send(m) { sent.push(JSON.parse(m)); }
  close() {}
}
// --- three.js: the real r128 build, only the WebGL renderer is stubbed ---
let animLoop = null, session = null;
class FakeRenderer {
  constructor() { this.xr = { enabled: false, setReferenceSpaceType() {}, getReferenceSpace: () => "ref", setSession: async (s) => { session = s; } }; }
  setPixelRatio() {} setClearColor() {} setAnimationLoop(f) { animLoop = f; }
  render(scene) { scene.updateMatrixWorld(); }     // what a real render does to matrices
}
const navigator = { xr: {
  isSessionSupported: async () => true,
  requestSession: async (mode) => ({ mode, inputSources: [], ev: {}, addEventListener(e, f) { this.ev[e] = f; } }),
} };

const sandbox = { console, document, navigator, WebSocket: FakeWS, setTimeout, setImmediate,
  location: { protocol: "https:", host: "mac:8443", search: "?key=abc123" },
  URLSearchParams, Promise, Math, JSON, Set, Number, Object, Array, Float32Array, performance,
  window: { devicePixelRatio: 1, isSecureContext: true, addEventListener: (e, f) => (docHandlers[e] = f) } };
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(WEB, "vendor", "three.min.js"), "utf8"), sandbox, { filename: "three.min.js" });
const THREE = sandbox.THREE;
THREE.WebGLRenderer = FakeRenderer;
for (const f of ["kin.js", "teleop.js", "quest.js"])
  vm.runInContext(fs.readFileSync(path.join(WEB, f), "utf8"), sandbox, { filename: f });

// --- XR frame helpers ---
function pad(pressed) { return { buttons: [0, 1, 2, 3, 4, 5].map((i) => ({ pressed: pressed.includes(i) })), axes: [0, 0, 0, 0], hapticActuators: [{ pulse: async () => {} }] }; }
function src(hand, pos, pressed, quat) {
  return { handedness: hand, targetRaySpace: { pos, quat: quat || [0, 0, 0, 1] }, gamepad: pad(pressed || []) };
}
let bodyJoints = null;            // name -> [x, y, z] (XR) when the mock has body tracking
function frame(sources) {
  session.inputSources = sources;
  return {
    session,
    body: bodyJoints && { get: (n) => (bodyJoints[n] ? { pos: bodyJoints[n], quat: [0, 0, 0, 1] } : undefined) },
    getViewerPose: () => ({ transform: { position: { x: 0, y: 1.6, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }),
    getPose: (space) => ({ transform: { position: { x: space.pos[0], y: space.pos[1], z: space.pos[2] },
                                        orientation: { x: space.quat[0], y: space.quat[1], z: space.quat[2], w: space.quat[3] } } }),
  };
}
function state(engaged, q12, teleop) {
  const q = new Array(26).fill(0);
  q12.forEach((v, k) => (q[14 + k] = v));
  sock.onmessage({ data: JSON.stringify({ type: "state", arm_engaged: engaged, teleop_live: !!teleop, q, q_cmd: q12 }) });
}
let t = 0;
const step = (sources, n = 1) => { for (let i = 0; i < n; i++) { t += 14; animLoop(t, frame(sources)); } };
const teleops = () => sent.filter((m) => m.type === "teleop");

(async () => {
  await docHandlers.DOMContentLoaded();
  await new Promise((r) => setImmediate(r));
  assert.ok(sock.url.endsWith("/ws?key=abc123"), "key passed on the WebSocket URL: " + sock.url);
  assert.strictEqual(el("btn-ar").disabled, false, "AR button enabled when supported");
  console.log("ok  page boots, connects with the key, enables Enter buttons");

  await el("btn-ar").handlers.click();
  assert.ok(session && session.mode === "immersive-ar" && animLoop, "AR session started");

  state(false, READY);
  for (let i = 0; i <= 10; i++) step([src("left", [0, 1, -0.005 * i], [1])]);
  assert.strictEqual(teleops().length, 0, "nothing sent while not engaged");
  const prev = vm.runInContext("core.out.slice(0, 5)", sandbox);
  assert.ok(Math.abs(prev[0] - READY[0]) > 0.02 || Math.abs(prev[3] - READY[3]) > 0.02, "preview: grip moves the green ghost target");
  step([src("left", [0, 1, 0])], 2);                    // let go
  console.log("ok  not engaged -> PREVIEW: grip moves the ghost, no teleop frames sent");

  state(true, READY);
  const afterEngage = vm.runInContext("core.out ? core.out.slice(0, 5) : null", sandbox);
  assert.ok(afterEngage === null, "engaging discards the preview (no jump to the preview pose)");
  step([src("left", [0, 1, 0]), src("right", [0.3, 1, 0])], 6);
  const hb = teleops();
  assert.ok(hb.length >= 2 && hb.every((m) => m.mask.join() === "0,0,0"), "heartbeats with empty mask");
  assert.ok(hb.every((m) => m.vel === 0.7 && m.q.length === 12));
  console.log("ok  engaged, no grip -> ~30 Hz heartbeats, mask 0,0,0");

  sent.length = 0;
  for (let i = 0; i <= 10; i++) step([src("left", [0, 1, -0.005 * i], [1]), src("right", [0.3, 1, 0])], 2);
  const mv = teleops();
  assert.ok(mv.length > 0 && mv.every((m) => m.mask.join() === "1,0,0"), "left grip -> mask 1,0,0");
  const last = mv[mv.length - 1].q;
  assert.ok(Math.abs(last[0] - READY[0]) > 0.02 || Math.abs(last[3] - READY[3]) > 0.02, "left arm target moved");
  assert.deepStrictEqual(last.slice(5, 10), READY.slice(5, 10), "right arm untouched");
  console.log("ok  left grip + pull forward -> left arm targets change, right untouched");

  sent.length = 0;
  step([src("left", [0, 1, -0.05], [1, 5]), src("right", [0.3, 1, 0])]);   // press Y while gripping
  assert.ok(sent.some((m) => m.type === "teleop_stop"), "freeze sends teleop_stop");
  sent.length = 0;
  step([src("left", [0, 1, -0.1], [1]), src("right", [0.3, 1, 0])], 6);   // still gripping
  assert.strictEqual(teleops().length, 0, "frozen: nothing sent");
  step([src("left", [0, 1, -0.1]), src("right", [0.3, 1, 0])], 2);        // release all
  step([src("left", [0, 1, -0.1], [1]), src("right", [0.3, 1, 0])], 6);   // grab again
  assert.ok(teleops().length > 0, "resumes after release + re-grab");
  console.log("ok  B/Y freezes (teleop_stop, sending stops); release + re-grab resumes");

  // the stage: backdrop behind the figures (farther from you), YOU on your left of ROBOT,
  // your hand ball follows the controller, the target ball shows while gripping
  {
    const ctx0 = (e) => vm.runInContext(e, sandbox);
    const head = new THREE.Vector3(0, 1.6, 0);
    const wp = (e) => ctx0(e + ".getWorldPosition(new THREE.Vector3())");
    const back = wp("backdrop"), robotHead = wp("robotLive.meshes[3]"), youHead = wp("human.head");
    assert.ok(back.distanceTo(head) > robotHead.distanceTo(head), "backdrop is behind the robot figure");
    assert.ok(youHead.x < robotHead.x, "YOU is drawn to the left of ROBOT (XR -X = your left)");
    step([src("left", [0, 1.2, -0.3]), src("right", [0.3, 1, 0])], 2);
    const h1 = wp("human.hand.l");
    step([src("left", [0, 1.4, -0.3]), src("right", [0.3, 1, 0])], 2);
    const h2 = wp("human.hand.l");
    const k = ctx0("HUMAN_SCALE * STAGE_SCALE");
    assert.ok(Math.abs(h2.y - h1.y - 0.2 * k) < 1e-3, "your left hand ball rose with the controller: " + (h2.y - h1.y));
    assert.ok(!ctx0("handTgt.l.visible"), "no target ball while not gripping");
    step([src("left", [0, 1.4, -0.3], [1]), src("right", [0.3, 1, 0])], 3);
    assert.ok(ctx0("handTgt.l.visible"), "target ball shows while gripping");
    step([src("left", [0, 1.4, -0.3]), src("right", [0.3, 1, 0])], 2);
    console.log("ok  stage: dark backdrop behind the figures, YOU left of ROBOT, hand + target balls follow");
  }

  // drag the status panel: point at it from the head, hold the trigger, move 20 cm right
  const ctx = (expr) => vm.runInContext(expr, sandbox);
  const hud0 = ctx("hudMesh.position.clone()");
  const eye = new THREE.Vector3(0, 1.6, 0);
  const aim = (from, to) => { const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, -1), to.clone().sub(from).normalize()); return [q.x, q.y, q.z, q.w]; };
  const qa = aim(eye, hud0);
  const at = (dx, pressed) => [src("right", [eye.x + dx, eye.y, eye.z], pressed, qa), src("left", [0, 1, 0])];
  step(at(0, []), 2);                                   // hover
  assert.ok(ctx("rayLine.r.scale.z") < 1.0, "ray stops at the panel when pointing at it");
  sent.length = 0;
  step(at(0, [0, 1]));                                   // trigger (+ grip) on the panel: grab
  assert.ok(ctx("drag.r && drag.r.what") === "hud", "trigger on the panel grabs it");
  for (let i = 1; i <= 10; i++) step(at(0.02 * i, [0, 1]));
  const hud1 = ctx("hudMesh.position.clone()");
  assert.ok(Math.abs(hud1.x - hud0.x - 0.2) < 0.01 && Math.abs(hud1.y - hud0.y) < 0.01, "panel moved 20 cm right: " + (hud1.x - hud0.x));
  assert.ok(teleops().every((m) => m.mask[1] === 0), "grip while dragging does not move the right arm");
  step(at(0.2, []));                                      // release trigger
  step(at(0.5, []), 3);
  assert.ok(ctx("hudMesh.position").distanceTo(hud1) < 1e-6, "panel stays after release");
  console.log("ok  trigger drags the status panel (moved 20 cm), arm ignores grip meanwhile, stays on release");

  // drag the whole YOU|ROBOT stage by its backdrop
  const g0 = ctx("new THREE.Vector3().setFromMatrixPosition(stage.matrix)");
  const gc = ctx("backdrop.getWorldPosition(new THREE.Vector3())");
  const qg = aim(eye, gc);
  const gat = (dy, pressed) => [src("left", [eye.x, eye.y + dy, eye.z], pressed, qg), src("right", [0.3, 1, 0])];
  step(gat(0, []), 2);
  step(gat(0, [0]));
  assert.ok(ctx("drag.l && drag.l.what") === "stage", "trigger on the backdrop grabs the stage");
  for (let i = 1; i <= 10; i++) step(gat(0.015 * i, [0]));
  const g1 = ctx("new THREE.Vector3().setFromMatrixPosition(stage.matrix)");
  assert.ok(Math.abs(g1.y - g0.y - 0.15) < 0.01, "stage moved 15 cm up: " + (g1.y - g0.y));
  step(gat(0.15, []));
  console.log("ok  trigger on the backdrop drags the whole YOU|ROBOT stage");

  // A/X cycles both -> left only -> right only; the unselected arm ignores its grip
  const pressX = () => { step([src("left", [0, 1, 0], [4]), src("right", [0.3, 1, 0])]); step([src("left", [0, 1, 0]), src("right", [0.3, 1, 0])]); };
  pressX();
  assert.strictEqual(ctx("core.arms"), "l");
  sent.length = 0;
  for (let i = 0; i <= 6; i++) step([src("left", [0, 1, 0]), src("right", [0.3, 1, -0.01 * i], [1])], 2);
  assert.ok(teleops().length > 0 && teleops().every((m) => m.mask.join() === "0,0,0"), "left-only: right grip ignored");
  pressX();
  assert.strictEqual(ctx("core.arms"), "r");
  sent.length = 0;
  for (let i = 0; i <= 6; i++) step([src("left", [0, 1, 0], [1]), src("right", [0.3, 1, 0])], 2);
  assert.ok(teleops().every((m) => m.mask[0] === 0), "right-only: left grip ignored");
  sent.length = 0;
  for (let i = 0; i <= 6; i++) step([src("left", [0, 1, 0]), src("right", [0.3, 1, -0.01 * i], [1])], 2);
  assert.ok(teleops().some((m) => m.mask.join() === "0,1,0"), "right-only: right grip drives the right arm");
  pressX();
  assert.strictEqual(ctx("core.arms"), "both");
  assert.strictEqual(el("opt-arms").value, "both", "settings dropdown follows the headset toggle");
  console.log("ok  A/X cycles both -> left only -> right only; unselected arm ignores its grip");

  // B/Y: a tap only freezes; holding it 1 s sends one E-STOP
  sent.length = 0;
  step([src("left", [0, 1, 0]), src("right", [0.3, 1, 0], [5])], 3);       // ~40 ms tap
  step([src("left", [0, 1, 0]), src("right", [0.3, 1, 0])], 3);
  assert.ok(sent.some((m) => m.type === "teleop_stop") && !sent.some((m) => m.type === "estop"), "tap: freeze only");
  sent.length = 0;
  step([src("left", [0, 1, 0]), src("right", [0.3, 1, 0], [5])], 60);      // ~0.84 s: not yet
  assert.ok(!sent.some((m) => m.type === "estop"), "no E-STOP before 1 s");
  step([src("left", [0, 1, 0]), src("right", [0.3, 1, 0], [5])], 40);      // past 1 s, still held
  assert.strictEqual(sent.filter((m) => m.type === "estop").length, 1, "exactly one E-STOP after holding 1 s");
  step([src("left", [0, 1, 0]), src("right", [0.3, 1, 0])], 2);
  console.log("ok  B/Y tap = freeze only; holding 1 s sends a single E-STOP");

  // whole-arm mode with (mock) WebXR body tracking
  {
    const ctx1 = (e) => vm.runInContext(e, sandbox);
    el("opt-track").value = "arm"; el("opt-track").handlers.change();
    assert.strictEqual(ctx1("core.mode"), "arm");
    // a person facing -Z; right arm raised forward and out: elbow ahead-right, wrist further ahead and up
    bodyJoints = { hips: [0, 0.9, 0], chest: [0, 1.3, 0], neck: [0, 1.45, 0],
      "left-arm-upper": [-0.2, 1.4, 0], "left-arm-lower": [-0.22, 1.1, 0], "left-hand-wrist": [-0.22, 1.05, -0.27],
      "right-arm-upper": [0.2, 1.4, 0], "right-arm-lower": [0.3, 1.3, -0.25], "right-hand-wrist": [0.33, 1.45, -0.48] };
    const R = () => [src("left", [-0.22, 1.05, -0.27]), src("right", [0.33, 1.45, -0.48], [1])];
    sent.length = 0;
    step(R(), 30);                                          // ~0.4 s of gripping
    const early = teleops().filter((m) => m.mask[1] === 1);
    assert.ok(early.length > 0 && early.every((m) => m.vel === 0.25), "first 1.5 s of a grab: soft start at 0.25 rad/s");
    step(R(), 110);                                         // past 1.5 s
    const late = teleops().slice(-1)[0];
    assert.strictEqual(late.vel, 0.7, "then the selected speed");
    assert.deepStrictEqual(late.mask, [0, 1, 0]);
    // robot right arm shape == the person's right arm shape (torso frame = robot frame here)
    const K = require("../web/kin.js");
    const q = new Array(26).fill(0); late.q.forEach((v, k) => (q[14 + k] = v));
    const F = K.computeFK(q), nrm = (v) => { const n = Math.hypot(...v); return v.map((x) => x / n); };
    const B = bodyJoints, toR = (v) => [-v[2], -v[0], v[1]];     // XR -> robot for a person facing -Z
    const uH = nrm(toR(B["right-arm-lower"].map((v, i) => v - B["right-arm-upper"][i])));
    const fH = nrm(toR(B["right-hand-wrist"].map((v, i) => v - B["right-arm-lower"][i])));
    const d = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    const e = d(K.upperArmDir(F, "r"), uH) + d(K.forearmDir(F, "r"), fH);
    assert.ok(e < 0.05, "robot right arm copies your arm shape, error " + e.toFixed(3));
    // YOU figure uses the tracked wrist, not the controller
    const before = ctx1("human.hand.r.getWorldPosition(new THREE.Vector3())");
    bodyJoints["right-hand-wrist"] = [0.33, 1.55, -0.48];
    step(R(), 2);
    const after = ctx1("human.hand.r.getWorldPosition(new THREE.Vector3())");
    assert.ok(after.y - before.y > 0.02, "YOU figure follows the tracked wrist");
    // body tracking lost -> nothing new is commanded for that arm
    bodyJoints = null; sent.length = 0;
    step(R(), 10);
    assert.ok(teleops().every((m) => m.mask[1] === 0), "no body data -> right arm not driven");
    step([src("left", [0, 1, 0]), src("right", [0.3, 1, 0])], 2);
    el("opt-track").value = "hand"; el("opt-track").handlers.change();
    console.log("ok  whole-arm mode: soft start, then copies the tracked arm shape; YOU = tracked skeleton; no body = no drive");
  }

  sent.length = 0;
  session.ev.end();
  assert.ok(sent.some((m) => m.type === "teleop_stop"), "leaving XR sends teleop_stop");
  console.log("ok  exiting the session sends teleop_stop");
  console.log("all quest page tests passed");
})().catch((e) => { console.error(e); process.exit(1); });
