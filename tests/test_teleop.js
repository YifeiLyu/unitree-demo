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

test("arms option: 'l' ignores the right grip, 'r' ignores the left grip", () => {
  const mv = (side, n) => Array.from({ length: n + 1 }, (_, i) => ({ [side]: { pos: [0, 1, -0.005 * i], quat: ID, grip: true }, head: ID }));
  const onlyL = run(new T.TeleopCore({ arms: "l" }), mv("r", 10));
  assert.deepStrictEqual(onlyL.mask, [0, 0, 0]);
  assert.deepStrictEqual(onlyL.q, READY);
  const onlyR = run(new T.TeleopCore({ arms: "r" }), mv("r", 10));
  assert.deepStrictEqual(onlyR.mask, [0, 1, 0]);
  const onlyR2 = run(new T.TeleopCore({ arms: "r" }), mv("l", 10));
  assert.deepStrictEqual(onlyR2.mask, [0, 0, 0]);
});

test("robot's real resting pose (both arms below floorZ): right-only arm can move up, not lower; left never blocks it", () => {
  // measured on the robot 2026-10-18 with the arms hanging (lowest point z = 0.006 < floorZ)
  const REST = [-0.62, -0.03, 0, 0.69, 0.01, -0.61, 0.06, 0.04, 0.66, 0.01, 0, 0];
  assert.ok(!K.checkPose(to26(REST)).ok, "precondition: rest pose breaks the floor limit");
  const minZ = (q12, s) => K.poseMetrics(to26(q12))[s].minZ;
  const drive = (dy) => {
    const core = new T.TeleopCore({ arms: "r" }), warned = new Set();
    let out;
    for (let i = 0; i <= 14; i++) {
      out = core.update({ q26: to26(REST), qcmd: null, head: ID, r: { pos: [0, 1 + dy * i / 14, 0], quat: ID, grip: true } });
      out.warn.forEach((w) => warned.add(w));
    }
    return { out, warned: [...warned] };
  };
  const up = drive(0.07);
  assert.deepStrictEqual(up.out.mask, [0, 1, 0]);
  assert.ok(minZ(up.out.q, "r") > minZ(REST, "r") + 0.03, "right hand went up");
  assert.ok(!up.warned.some((w) => /left/.test(w)), "left arm never mentioned: " + up.warned);
  assert.deepStrictEqual(up.out.q.slice(0, 5), REST.slice(0, 5), "left arm untouched");
  const down = drive(-0.07);
  assert.ok(down.warned.includes("right arm too low"), "going lower is refused: " + down.warned);
  assert.ok(minZ(down.out.q, "r") >= minZ(REST, "r") - 1e-3, "never lower than where it started");
  console.log("    up: right hand +" + cm(minZ(up.out.q, "r") - minZ(REST, "r")) + "; down: refused (" + down.warned.join("; ") + ")");
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

// --- whole-arm mode (WebXR body tracking) ---
// A person facing XR -Z (same way as the robot), shoulders 40 cm apart.
const BODY0 = { lsh: [-0.2, 1.4, 0], rsh: [0.2, 1.4, 0], hips: [0, 0.9, 0], chest: [0, 1.3, 0] };
// Build tracked joints whose arm SHAPE equals robot pose q12 (upper arm 30 cm, forearm 27 cm).
function bodyFor(q12, frame) {
  const F = K.computeFK(to26(q12)), Bf = frame || T.bodyFrame(BODY0), b = Object.assign({}, BODY0);
  for (const s of ["l", "r"]) {
    const u = T.fromRobot(Bf, K.upperArmDir(F, s)), f = T.fromRobot(Bf, K.forearmDir(F, s));
    const sh = s === "l" ? b.lsh : b.rsh, el = sh.map((v, i) => v + 0.30 * u[i]), wr = el.map((v, i) => v + 0.27 * f[i]);
    b[s] = { sh, el, wr };
  }
  return b;
}
const dirErr = (qa, qb, s) => {
  const A = K.computeFK(to26(qa)), B = K.computeFK(to26(qb));
  const d = (x, y) => Math.hypot(x[0] - y[0], x[1] - y[1], x[2] - y[2]);
  return d(K.upperArmDir(A, s), K.upperArmDir(B, s)) + d(K.forearmDir(A, s), K.forearmDir(B, s));
};

test("body frame: a person facing XR -Z gives forward -Z, left -X, up +Y", () => {
  const B = T.bodyFrame(BODY0);
  assert.deepStrictEqual(B.map((r) => r.map((v) => Math.round(v) + 0)), [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]);
});

test("arm mode: gripping makes the robot arm take the shape of your arm", () => {
  const goal = [-1.0, 0.4, 0.2, 0.6, 0, -0.6, -0.3, 0, -0.2, 0, 0, 0];     // left arm raised forward/out
  assert.ok(K.checkPose(to26(goal)).ok);
  const core = new T.TeleopCore({ mode: "arm", arms: "l" });
  let out;
  for (let i = 0; i < 6; i++)
    out = core.update({ q26: to26(READY), qcmd: READY, head: ID, l: { pos: [0, 1, 0], quat: ID, grip: true }, body: bodyFor(goal) });
  assert.deepStrictEqual(out.mask, [1, 0, 0]);
  const e = dirErr(out.q, goal, "l");
  console.log("    arm-shape error after 6 frames: " + e.toFixed(4) + " (upper arm + forearm direction)");
  assert.ok(e < 0.02, "arm shape matched");
  assert.deepStrictEqual(out.q.slice(5, 10), READY.slice(5, 10), "right arm untouched");
});

test("arm mode: not gripping -> nothing sent, but the preview shows where your arm maps to", () => {
  const goal = [-1.0, 0.4, 0.2, 0.6, 0, -0.6, -0.3, 0, -0.2, 0, 0, 0];
  const core = new T.TeleopCore({ mode: "arm" });
  let out;
  for (let i = 0; i < 6; i++) out = core.update({ q26: to26(READY), qcmd: READY, head: ID, body: bodyFor(goal) });
  assert.deepStrictEqual(out.mask, [0, 0, 0]);
  assert.deepStrictEqual(out.q, READY);
  assert.ok(dirErr(core.displayQ(out.q), goal, "l") < 0.02, "preview pose matches your arm");
});

test("arm mode: turning your whole body doesn't move the robot (torso frame, not world)", () => {
  const goal = [-1.0, 0.4, 0.2, 0.6, 0, -0.6, -0.3, 0, -0.2, 0, 0, 0];
  const turn = (p) => [p[0] * Math.cos(0.8) + p[2] * Math.sin(0.8), p[1], -p[0] * Math.sin(0.8) + p[2] * Math.cos(0.8)];
  const b = bodyFor(goal), tb = {};
  for (const k of ["lsh", "rsh", "hips", "chest"]) tb[k] = turn(b[k]);
  for (const s of ["l", "r"]) tb[s] = { sh: turn(b[s].sh), el: turn(b[s].el), wr: turn(b[s].wr) };
  const core = new T.TeleopCore({ mode: "arm", arms: "l" });
  let out;
  for (let i = 0; i < 6; i++) out = core.update({ q26: to26(READY), qcmd: READY, head: ID, l: { pos: [0, 1, 0], quat: ID, grip: true }, body: tb });
  assert.ok(dirErr(out.q, goal, "l") < 0.02, "same arm shape after turning 46 deg");
});

test("arm mode: no body data while gripping -> warns and holds; unsafe shapes are refused", () => {
  const core = new T.TeleopCore({ mode: "arm" });
  const o1 = core.update({ q26: to26(READY), qcmd: READY, head: ID, l: { pos: [0, 1, 0], quat: ID, grip: true }, body: null });
  assert.deepStrictEqual(o1.mask, [0, 0, 0]);
  assert.ok(o1.warn.includes("no body tracking"));
  // both forearms pointing across the chest at each other -> hands meet -> refused, output stays safe
  const cross = [-1.2, -0.2, -1.2, 0.9, 0, -1.2, 0.2, 1.2, 0.9, 0, 0, 0];
  const c2 = new T.TeleopCore({ mode: "arm" });
  const warned = new Set();
  let out;
  for (let i = 0; i < 10; i++) {
    out = c2.update({ q26: to26(READY), qcmd: READY, head: ID, l: { pos: [0, 1, 0], quat: ID, grip: true },
                      r: { pos: [0.3, 1, 0], quat: ID, grip: true }, body: bodyFor(cross) });
    out.warn.forEach((w) => warned.add(w));
  }
  assert.ok(K.checkPose(to26(out.q)).ok, "output stays safe");
  console.log("    crossing arms: " + ([...warned].join("; ") || "no warning"));
});

console.log(n + " tests passed");
