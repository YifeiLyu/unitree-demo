/* Magikid R1-A — Quest teleop mapping (pure math, no DOM / WebXR) ---------
   Turns controller poses into 12 joint targets for the bridge's "teleop"
   command. Needs kin.js. Loadable from Node for tests.

   Control model ("clutch"): while a grip is held, that robot hand follows the
   controller RELATIVE to where both were when the grip was pressed. Position
   moves the hand tip (scaled), the controller's rotation turns the forearm
   direction, and its twist about the pointing axis drives the wrist roll.
   Releasing the grip leaves the arm where it is, so you can re-grab and ratchet.

   Whole-arm mode (mode = "arm", WebXR body tracking): while a grip is held,
   that robot arm takes the SHAPE of your arm instead: its upper arm points
   where yours does (shoulder -> elbow) and its forearm where yours does
   (elbow -> wrist), measured in your torso frame, so body size doesn't matter.
   The wrist roll still comes from the controller's twist.

   Frames: WebXR is +X right, +Y up, -Z forward. The robot is +X forward,
   +Y left, +Z up. "Forward" is the operator's horizontal facing, captured at
   session start and on recenter, so stand behind the robot facing the same way. */
"use strict";

const TeleopMath = (function () {
  // kin.js globals are top-level consts (not window properties), so name them.
  const kin = (typeof module === "object" && module.exports) ? require("./kin.js")
    : { ARM_CMD_IDX, ARM_SIDE, ARM12_MIN, ARM12_MAX, computeFK, forearmDir, solveArmIK, solveArmDirIK, checkMove };

  // --- quaternions [x, y, z, w] and vectors ---
  const qMul = (a, b) => [
    a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
    a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
    a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
    a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2]];
  const qConj = (a) => [-a[0], -a[1], -a[2], a[3]];
  function qRot(q, v) {
    const p = qMul(qMul(q, [v[0], v[1], v[2], 0]), qConj(q));
    return [p[0], p[1], p[2]];
  }
  const sub = (a, b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
  const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  const norm = (a) => { const n = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0]/n, a[1]/n, a[2]/n]; };
  const wrapPi = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  // Rows of M = robot forward, left, up expressed in XR coordinates.
  function operatorFrame(headQuat) {
    let f = qRot(headQuat, [0, 0, -1]); f = [f[0], 0, f[2]];
    if (Math.hypot(f[0], f[2]) < 0.2) f = [0, 0, -1];      // looking straight up/down
    f = norm(f);
    const u = [0, 1, 0], l = cross(u, f);
    return [f, l, u];
  }
  const toRobot = (M, v) => [dot(M[0], v), dot(M[1], v), dot(M[2], v)];
  const fromRobot = (M, r) => [0, 1, 2].map((k) => M[0][k]*r[0] + M[1][k]*r[1] + M[2][k]*r[2]);
  // Twist of a local rotation about the controller's pointing axis (-Z).
  function twistAboutForward(qLocal) {
    const s = qLocal[3] < 0 ? -1 : 1;
    return wrapPi(-2 * Math.atan2(s * qLocal[2], s * qLocal[3]));
  }
  // Torso frame from tracked joints (XR coords), rows = forward, left, up (like M):
  // left = right shoulder -> left shoulder, up = hips -> chest made square to it.
  function bodyFrame(b) {
    const y = norm(sub(b.lsh, b.rsh));
    let z = b.hips && b.chest ? norm(sub(b.chest, b.hips)) : [0, 1, 0];
    const k = dot(z, y);
    z = norm([z[0] - k * y[0], z[1] - k * y[1], z[2] - k * y[2]]);
    return [cross(y, z), y, z];
  }
  // Head pitch from the headset: looking down = positive (URDF axis +Y).
  function headPitchFrom(headQuat) {
    const f = qRot(headQuat, [0, 0, -1]);
    return Math.asin(clamp(-f[1], -1, 1));
  }

  class TeleopCore {
    constructor(opt) {
      this.scale = 0.7;          // robot metres per controller metre
      this.headPitch = false;    // follow headset pitch with the robot head
      this.arms = "both";        // which arms the controllers may drive: "both" | "l" | "r"
      this.mode = "hand";        // "hand": hand tip follows the controller; "arm": whole-arm shape (body tracking)
      Object.assign(this, opt || {});
      this.M = operatorFrame([0, 0, 0, 1]);
      this.arm = { l: null, r: null };   // clutch state per side, null = released
      this.out = null;                   // last 12-vector we produced
      this.blocked = { l: false, r: false };
      this.tgt = { l: null, r: null };   // hand-tip target (robot frame) while clutched, for display
      this.view = { l: null, r: null };  // arm mode: 4 joint angles your arm maps to now (display)
      this.armSeed = { l: null, r: null }; // arm mode: last solution, seeds the next frame
    }
    recenter(headQuat) {
      this.M = operatorFrame(headQuat);
      this.arm.l = this.arm.r = null;    // re-grab after recenter
    }
    release() { this.arm.l = this.arm.r = null; this.out = null; this.tgt = { l: null, r: null }; }

    // inp: { q26: measured full state q, qcmd: bridge's 12 commanded q (or null),
    //        head: quat|null, l/r: { pos, quat, grip } | null,
    //        body: { lsh, rsh, hips, chest, l: { sh, el, wr }, r: {...} } | null  (XR positions) }
    // Returns { q: 12 targets, mask: [left, right, head], warn: [...] }.
    update(inp) {
      const ref = (inp.qcmd && inp.qcmd.length === 12) ? inp.qcmd
        : kin.ARM_CMD_IDX.map((i) => inp.q26[i]);
      if (!this.out) this.out = ref.slice();
      // Base pose: measured body with our current arm targets.
      const base = inp.q26.slice();
      kin.ARM_CMD_IDX.forEach((i, k) => (base[i] = this.out[k]));
      const mask = [0, 0, 0], warn = [];
      const B = this.mode === "arm" && inp.body ? bodyFrame(inp.body) : null;
      this.view = { l: null, r: null };

      for (const side of ["l", "r"]) {
        const c = inp[side], S = kin.ARM_SIDE[side], k0 = S.k0;
        const allowed = this.arms === "both" || this.arms === side;
        // arm mode: solve your arm's shape every frame (shown as a preview), apply it only while gripping
        let armSol = null;
        if (this.mode === "arm" && allowed && B && inp.body[side]) {
          const j = inp.body[side], toR = (v) => [dot(B[0], v), dot(B[1], v), dot(B[2], v)];
          const seed = base.slice();
          if (this.armSeed[side]) S.pos.forEach((i, n) => (seed[i] = this.armSeed[side][n]));
          else kin.ARM_CMD_IDX.slice(k0, k0 + 4).forEach((i, k) => (seed[i] = ref[k0 + k]));
          armSol = kin.solveArmDirIK(seed, side, norm(toR(sub(j.el, j.sh))), norm(toR(sub(j.wr, j.el))));
          this.armSeed[side] = S.pos.map((i) => armSol.q[i]);
          this.view[side] = this.armSeed[side].slice();
        }
        if (!c || !c.grip || !allowed) {           // released (or arm not selected): follow the bridge
          this.arm[side] = null; this.blocked[side] = false; this.tgt[side] = null;
          for (let k = 0; k < 5; k++) this.out[k0 + k] = ref[k0 + k];
          kin.ARM_CMD_IDX.slice(k0, k0 + 5).forEach((i, k) => (base[i] = ref[k0 + k]));
          continue;
        }
        if (this.mode === "arm") {
          this.tgt[side] = null;
          if (!armSol) { warn.push("no body tracking"); continue; }   // hold where it is
          if (!this.arm[side]) this.arm[side] = { Q0: c.quat.slice(), wr0: ref[k0 + 4] };
          const a = this.arm[side];
          const cand = base.slice();
          S.pos.forEach((i) => (cand[i] = armSol.q[i]));
          cand[S.wr] = clamp(a.wr0 + twistAboutForward(qMul(qConj(a.Q0), c.quat)),
                             kin.ARM12_MIN[k0 + 4], kin.ARM12_MAX[k0 + 4]);
          const chk = kin.checkMove(base, cand, side);
          if (chk.ok) {
            kin.ARM_CMD_IDX.slice(k0, k0 + 5).forEach((i, k) => { this.out[k0 + k] = cand[i]; base[i] = cand[i]; });
            this.blocked[side] = false;
          } else {
            this.blocked[side] = true;
            warn.push(...chk.warn);
          }
          mask[side === "l" ? 0 : 1] = 1;
          continue;
        }
        if (!this.arm[side]) {                     // grip just pressed: anchor
          const qa = base.slice();
          kin.ARM_CMD_IDX.slice(k0, k0 + 5).forEach((i, k) => (qa[i] = ref[k0 + k]));
          const F = kin.computeFK(qa);
          this.arm[side] = { P0: c.pos.slice(), Q0: c.quat.slice(), tip0: F[S.tip].p.slice(),
                             dir0: kin.forearmDir(F, side), wr0: qa[S.wr], seed: qa };
        }
        const a = this.arm[side];
        const d = toRobot(this.M, sub(c.pos, a.P0));
        const tgt = [0, 1, 2].map((k) => a.tip0[k] + this.scale * d[k]);
        this.tgt[side] = tgt;
        // world-frame relative rotation, carried into the robot frame
        const rel = qMul(c.quat, qConj(a.Q0));
        const dir = norm(toRobot(this.M, qRot(rel, fromRobot(this.M, a.dir0))));
        const wr = clamp(a.wr0 + twistAboutForward(qMul(qConj(a.Q0), c.quat)),
                         kin.ARM12_MIN[k0 + 4], kin.ARM12_MAX[k0 + 4]);

        const seed = base.slice();
        S.pos.forEach((i) => (seed[i] = a.seed[i]));
        const sol = kin.solveArmIK(seed, side, tgt, dir);
        const cand = base.slice();
        S.pos.forEach((i) => (cand[i] = sol.q[i]));
        cand[S.wr] = wr;
        const chk = kin.checkMove(base, cand, side);   // judge only this arm, vs where it is now
        if (chk.ok) {
          S.pos.forEach((i) => (a.seed[i] = sol.q[i]));
          kin.ARM_CMD_IDX.slice(k0, k0 + 5).forEach((i, k) => {
            this.out[k0 + k] = cand[i]; base[i] = cand[i];
          });
          this.blocked[side] = false;
        } else {                                   // keep the last safe target
          this.blocked[side] = true;
          warn.push(...chk.warn);
        }
        mask[side === "l" ? 0 : 1] = 1;
      }

      if (this.headPitch && inp.head) {
        this.out[10] = clamp(headPitchFrom(inp.head), kin.ARM12_MIN[10], kin.ARM12_MAX[10]);
        mask[2] = 1;
      } else {
        this.out[10] = ref[10];
      }
      this.out[11] = ref[11];                      // head yaw: never commanded
      return { q: this.out.slice(), mask, warn: [...new Set(warn)] };
    }
    // What to draw as the green target: the commanded pose, plus (arm mode) the
    // pose your arm maps to for arms you are not gripping yet.
    displayQ(q12) {
      const q = q12.slice();
      for (const side of ["l", "r"]) {
        const v = this.view[side], k0 = kin.ARM_SIDE[side].k0;
        if (v && !this.arm[side]) v.forEach((x, n) => (q[k0 + n] = x));
      }
      return q;
    }
  }

  return { TeleopCore, operatorFrame, bodyFrame, toRobot, fromRobot, twistAboutForward, headPitchFrom, qMul, qConj, qRot };
})();

if (typeof module === "object" && module.exports) module.exports = TeleopMath;
