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
el("opt-scale").value = "0.7"; el("opt-speed").value = "0.7";
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
// --- three.js stubs (only what quest.js touches) ---
class V3 { constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; } copy(v) { Object.assign(this, v); return this; } set(x, y, z) { this.x = x; this.y = y; this.z = z; } }
class Obj { constructor() { this.position = new V3(); this.quaternion = { set() {} }; this.matrix = { copy() {} }; this.children = []; this.visible = true; } add(...c) { this.children.push(...c); } lookAt() {} }
let animLoop = null, session = null;
const THREE = {
  WebGLRenderer: class { constructor() { this.xr = { enabled: false, setReferenceSpaceType() {}, getReferenceSpace: () => "ref", setSession: async (s) => { session = s; } }; }
    setPixelRatio() {} setClearColor() {} setAnimationLoop(f) { animLoop = f; } render() {} },
  Scene: Obj, Group: Obj, PerspectiveCamera: Obj, HemisphereLight: Obj,
  Mesh: class extends Obj { constructor(g, m) { super(); this.geometry = g; this.material = m; } },
  LineSegments: class extends Obj { constructor(g) { super(); this.geometry = g; } },
  PlaneGeometry: class {}, ConeGeometry: class { rotateX() {} }, CanvasTexture: class {},
  MeshBasicMaterial: class { constructor() { this.color = { set() {} }; } }, LineBasicMaterial: class {},
  BufferGeometry: class { setAttribute(n, a) { this.attributes = { [n]: a }; } },
  BufferAttribute: class { constructor(a) { this.array = a; } },
  Vector3: V3,
  Matrix4: class { makeBasis() { return this; } setPosition() { return this; } },
};
const navigator = { xr: {
  isSessionSupported: async () => true,
  requestSession: async (mode) => ({ mode, inputSources: [], ev: {}, addEventListener(e, f) { this.ev[e] = f; } }),
} };

const sandbox = { console, document, navigator, THREE, WebSocket: FakeWS, setTimeout, setImmediate,
  location: { protocol: "https:", host: "mac:8443", search: "?key=abc123" },
  URLSearchParams, Promise, Math, JSON, Set, Number, Object, Array, Float32Array,
  window: { devicePixelRatio: 1, isSecureContext: true, addEventListener: (e, f) => (docHandlers[e] = f) } };
vm.createContext(sandbox);
for (const f of ["kin.js", "teleop.js", "quest.js"])
  vm.runInContext(fs.readFileSync(path.join(WEB, f), "utf8"), sandbox, { filename: f });

// --- XR frame helpers ---
function pad(pressed) { return { buttons: [0, 1, 2, 3, 4, 5].map((i) => ({ pressed: pressed.includes(i) })), hapticActuators: [{ pulse: async () => {} }] }; }
function src(hand, pos, pressed) {
  return { handedness: hand, targetRaySpace: { pos }, gamepad: pad(pressed || []) };
}
function frame(sources) {
  session.inputSources = sources;
  return {
    session,
    getViewerPose: () => ({ transform: { position: { x: 0, y: 1.6, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }),
    getPose: (space) => ({ transform: { position: { x: space.pos[0], y: space.pos[1], z: space.pos[2] }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }),
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
  step([src("left", [0, 1, 0], [1])], 10);
  assert.strictEqual(teleops().length, 0, "nothing sent while not engaged");
  console.log("ok  not engaged -> no teleop frames");

  state(true, READY);
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

  sent.length = 0;
  session.ev.end();
  assert.ok(sent.some((m) => m.type === "teleop_stop"), "leaving XR sends teleop_stop");
  console.log("ok  exiting the session sends teleop_stop");
  console.log("all quest page tests passed");
})().catch((e) => { console.error(e); process.exit(1); });
