// Node tests for web/kin.js (IK, collision check) and web/teleop.js (Quest mapping).
// Run: node tests/test_teleop.js
"use strict";
const assert = require("assert");
const K = require("../web/kin.js");
const T = require("../web/teleop.js");

const HOME = [-0.8, 0, 0, 0.8, 0, -0.8, 0, 0, 0.8, 0, 0, 0];
// Mid-workspace pose (hands in front of the chest) with room to move 5 cm any way.
const READY = [-0.6, 0.3, 0, -0.2, 0, -0.6, -0.3, 0, -0.2, 0, 0, 0];
const to26 = (q12) => { const q = new Array(26).fill(0); K.ARM_CMD_IDX.forEach((i, k) => (q[i] = q12[k])); return q; };
const cm = (m) => (m * 100).toFixed(1) + " cm";
let n = 0;
const test = (name, fn) => { fn(); n++; console.log("ok  " + name); };
let seed = 12345;                                  // deterministic PRNG
const rand = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);

test("home pose passes the collision check; zero pose is too low", () => {
  assert.ok(K.checkPose(to26(HOME)).ok);
  assert.ok(!K.checkPose(to26(new Array(12).fill(0))).ok);
});

test("IK reaches FK-generated targets 0.2 rad/joint away (position only)", () => {
  let worst = 0;
  for (let t = 0; t < 300; t++) {
    const start = rand() < 0.5 ? HOME : READY, q = start.slice();
    for (const k of [0, 1, 2, 3]) q[k] = start[k] + (rand() - 0.5) * 0.4;
    const goal = K.computeFK(to26(q)).l_tip.p;
    // teleop moves the goal a little each frame; give it 3 frames (12 iterations each)
    let s = to26(start);
    for (let f = 0; f < 3; f++) s = K.solveArmIK(s, "l", goal, null).q;
    const err = Math.hypot(...goal.map((v, i) => v - K.computeFK(s).l_tip.p[i]));
    worst = Math.max(worst, err);
  }
  assert.ok(worst < 0.005, "worst IK error " + cm(worst));
  console.log("    worst tip error " + cm(worst));
});

test("IK never leaves URDF limits", () => {
  const sol = K.solveArmIK(to26(HOME), "r", [0.2, -0.9, 0.2], null);   // unreachable
  K.ARM_CMD_IDX.forEach((i, k) => assert.ok(sol.q[i] >= K.ARM12_MIN[k] - 1e-9 && sol.q[i] <= K.ARM12_MAX[k] + 1e-9));
});

// Simulated operator: facing XR -Z (so robot forward == XR -Z, robot left == XR -X).
const ID = [0, 0, 0, 1];
const qAxis = (ax, a) => [ax[0] * Math.sin(a / 2), ax[1] * Math.sin(a / 2), ax[2] * Math.sin(a / 2), Math.cos(a / 2)];

test("operator frame: XR -Z is robot forward, -X is robot left, +Y is up", () => {
  const M = T.operatorFrame(ID);
  assert.deepStrictEqual(T.toRobot(M, [0, 0, -1]).map(Math.round), [1, 0, 0]);
  assert.deepStrictEqual(T.toRobot(M, [-1, 0, 0]).map(Math.round), [0, 1, 0]);
  assert.deepStrictEqual(T.toRobot(M, [0, 1, 0]).map(Math.round), [0, 0, 1]);
  // operator turned 90 deg left: their forward is XR -X
  const M2 = T.operatorFrame(qAxis([0, 1, 0], Math.PI / 2));
  assert.deepStrictEqual(T.toRobot(M2, [-1, 0, 0]).map((v) => Math.round(v) + 0), [1, 0, 0]);
});

function run(core, frames, start) {   // frames: [{l, r, head}] -> last output
  let out, q26 = to26(start || READY);
  for (const f of frames) {
    out = core.update(Object.assign({ q26, qcmd: null }, f));
  }
  return out;
}

test("no grip -> nothing masked, targets equal the reference", () => {
  const core = new T.TeleopCore();
  const out = run(core, [{ l: { pos: [0, 1, 0], quat: ID, grip: false }, r: null, head: ID }]);
  assert.deepStrictEqual(out.mask, [0, 0, 0]);
  assert.deepStrictEqual(out.q, READY);
});

test("grip + move controller 7 cm forward -> left hand ~4.9 cm forward (scale 0.7)", () => {
  const core = new T.TeleopCore({ scale: 0.7 });
  const p0 = [-0.2, 1.0, -0.3];
  const frames = [];
  for (let i = 0; i <= 14; i++) frames.push({ l: { pos: [p0[0], p0[1], p0[2] - 0.005 * i], quat: ID, grip: true }, head: ID });
  const out = run(core, frames);
  assert.deepStrictEqual(out.mask, [1, 0, 0]);
  const a = K.computeFK(to26(READY)).l_tip.p, b = K.computeFK(to26(out.q)).l_tip.p;
  const d = b.map((v, i) => v - a[i]);
  console.log("    hand moved " + d.map(cm).join(", ") + " (x fwd, y left, z up)");
  assert.ok(Math.abs(d[0] - 0.049) < 0.008 && Math.abs(d[1]) < 0.008 && Math.abs(d[2]) < 0.008);
  assert.deepStrictEqual(out.q.slice(5, 10), READY.slice(5, 10), "right arm untouched");
});

test("controller up / left / back 7 cm -> right hand moves the same way ~4.9 cm", () => {
  for (const [dv, axis, sign] of [[[0, 0.07, 0], 2, 1], [[-0.07, 0, 0], 1, 1], [[0, 0, 0.07], 0, -1]]) {
    const core = new T.TeleopCore({ scale: 0.7 });
    const frames = [];
    for (let i = 0; i <= 14; i++) frames.push({ r: { pos: dv.map((v) => v * i / 14), quat: ID, grip: true }, head: ID });
    const out = run(core, frames);
    const a = K.computeFK(to26(READY)).r_tip.p, b = K.computeFK(to26(out.q)).r_tip.p;
    const moved = sign * (b[axis] - a[axis]);
    assert.ok(Math.abs(moved - 0.049) < 0.008, "axis " + axis + " moved " + cm(moved));
  }
});

test("re-grabbing (ratchet) continues from where the arm is, no jump back", () => {
  const core = new T.TeleopCore({ scale: 0.7 });
  const mv = (z0, z1, n) => Array.from({ length: n + 1 }, (_, i) => ({ l: { pos: [0, 1, z0 + (z1 - z0) * i / n], quat: ID, grip: true }, head: ID }));
  let q26 = to26(READY), out;
  for (const f of mv(0, 0.04, 8)) out = core.update(Object.assign({ q26, qcmd: null }, f));   // back 4 cm
  const qcmd = out.q.slice();                  // bridge has caught up with our target
  core.update({ q26, qcmd, l: { pos: [0, 1, 0.04], quat: ID, grip: false }, head: ID });   // release
  for (const f of mv(0.5, 0.5, 2)) out = core.update(Object.assign({ q26, qcmd }, f));      // re-grab elsewhere, hold still
  out.q.slice(0, 5).forEach((v, k) => assert.ok(Math.abs(v - qcmd[k]) < 1e-3, "joint " + k + " jumped"));
});

test("twisting the controller 30 deg about its pointing axis rolls the wrist 30 deg", () => {
  const core = new T.TeleopCore();
  const out = run(core, [
    { l: { pos: [0, 1, 0], quat: ID, grip: true }, head: ID },
    { l: { pos: [0, 1, 0], quat: qAxis([0, 0, -1], 0.5236), grip: true }, head: ID },
  ]);
  assert.ok(Math.abs(out.q[4] - (READY[4] + 0.5236)) < 1e-3, "wrist " + out.q[4]);
});

test("unsafe targets (hands meeting, too low, into torso) are refused; output stays safe", () => {
  const cases = [
    ["hands meet", (i) => ({ l: { pos: [0.3 * i / 40, 0, -0.2 * i / 40], quat: ID, grip: true },
                              r: { pos: [-0.3 * i / 40, 0, -0.2 * i / 40], quat: ID, grip: true } }), /apart/],
    ["too low", (i) => ({ l: { pos: [0, -0.4 * i / 40, 0], quat: ID, grip: true } }), /too low/],
    ["into torso", (i) => ({ l: { pos: [0, 0, 0.4 * i / 40], quat: ID, grip: true } }), /torso|too low/],
  ];
  for (const [name, frame, re] of cases) {
    const core = new T.TeleopCore({ scale: 1.0 });
    const warned = new Set();
    let out;
    for (let i = 0; i <= 40; i++) {
      out = core.update(Object.assign({ q26: to26(READY), qcmd: null, head: ID }, frame(i)));
      out.warn.forEach((w) => warned.add(w));
      assert.ok(K.checkPose(to26(out.q)).ok, name + ": output pose must always be safe");
    }
    assert.ok([...warned].some((w) => re.test(w)), name + ": expected a warning, got " + [...warned]);
    console.log("    " + name + ": " + [...warned].join("; "));
  }
});

test("head pitch follows the headset only when enabled, head yaw never", () => {
  const down = qAxis([1, 0, 0], -0.3);       // looking 0.3 rad down
  const off = run(new T.TeleopCore(), [{ head: down }]);
  assert.strictEqual(off.mask[2], 0);
  const on = run(new T.TeleopCore({ headPitch: true }), [{ head: down }]);
  assert.strictEqual(on.mask[2], 1);
  assert.ok(Math.abs(on.q[10] - 0.3) < 1e-6, "pitch " + on.q[10]);
  assert.strictEqual(on.q[11], 0);
});

console.log(n + " tests passed");
