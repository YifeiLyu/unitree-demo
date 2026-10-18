/* Magikid R1-A — shared kinematics (dashboard + Quest teleop) ------------- */
/* Classic script: everything below is a global, so load it before app.js /
   quest.js. Also loadable from Node (tests/) through module.exports. */
"use strict";

/* State-array indices the low-level arm control drives (12: arms + head, no
   waist), matching the bridge's ARM_IDL / r1A_wrist_swing_example. */
const ARM_CMD_IDX = [14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25];

// short index lookups
const IDX = {
  L_SP: 14, L_SR: 15, L_SY: 16, L_EL: 17, L_WR: 18,
  R_SP: 19, R_SR: 20, R_SY: 21, R_EL: 22, R_WR: 23,
  WAIST_YAW: 13, HEAD_PITCH: 24, HEAD_YAW: 25,
};

/* ---------------------------------------------------------------------- */
/* 3D forward-kinematics skeleton (from the r1_a5 URDF)                    */
/* ---------------------------------------------------------------------- */
// Kinematic chain: parent, origin translation (xyz, metres), fixed rotation
// (rpy), joint axis, and the state-array index that drives it. Values are the
// exact URDF origins/axes for the R1-A upper body (r1_a5_description).
const KIN = [
  { n: "waist", par: null,    xyz: [0, 0, 0],                        rpy: [0, 0, 0],        axis: [0, 0, 1], q: IDX.WAIST_YAW },
  { n: "l_sp",  par: "waist", xyz: [0, 0.085688, 0.19749],           rpy: [0.26187, 0, 0],  axis: [0, 1, 0], q: IDX.L_SP },
  { n: "l_sr",  par: "l_sp",  xyz: [0.03445, 0.047132, -0.025693],   rpy: [-0.26187, 0, 0], axis: [1, 0, 0], q: IDX.L_SR },
  { n: "l_sy",  par: "l_sr",  xyz: [-0.03445, 0.0043, -0.10835],     rpy: [0, 0, 0],        axis: [0, 0, 1], q: IDX.L_SY },
  { n: "l_el",  par: "l_sy",  xyz: [0.016191, 0.026461, -0.082858],  rpy: [0, 0, 0],        axis: [0, 1, 0], q: IDX.L_EL },
  { n: "l_wr",  par: "l_el",  xyz: [0.11218, -0.03002, -0.011702],   rpy: [0, 0, 0],        axis: [1, 0, 0], q: IDX.L_WR },
  { n: "r_sp",  par: "waist", xyz: [0, -0.085688, 0.19749],          rpy: [-0.26187, 0, 0], axis: [0, 1, 0], q: IDX.R_SP },
  { n: "r_sr",  par: "r_sp",  xyz: [0.03445, -0.047132, -0.025693],  rpy: [0.26187, 0, 0],  axis: [1, 0, 0], q: IDX.R_SR },
  { n: "r_sy",  par: "r_sr",  xyz: [-0.03445, -0.0043, -0.10835],    rpy: [0, 0, 0],        axis: [0, 0, 1], q: IDX.R_SY },
  { n: "r_el",  par: "r_sy",  xyz: [0.016191, -0.026461, -0.082858], rpy: [0, 0, 0],        axis: [0, 1, 0], q: IDX.R_EL },
  { n: "r_wr",  par: "r_el",  xyz: [0.11218, 0.03002, -0.011702],    rpy: [0, 0, 0],        axis: [1, 0, 0], q: IDX.R_WR },
  { n: "h_p",   par: "waist", xyz: [-0.006, 0.03155, 0.255],         rpy: [0, 0, 0],        axis: [0, 1, 0], q: IDX.HEAD_PITCH },
  { n: "h_y",   par: "h_p",   xyz: [0.0010126, -0.03155, 0.1165],    rpy: [0, 0, 0],        axis: [0, 0, 1], q: IDX.HEAD_YAW },
];

// --- tiny 3x3 matrix / vec3 helpers ---
function mMul(a, b) { const c = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i=0;i<3;i++) for (let j=0;j<3;j++){ let s=0; for (let k=0;k<3;k++) s+=a[i][k]*b[k][j]; c[i][j]=s; } return c; }
function mVec(a, v) { return [a[0][0]*v[0]+a[0][1]*v[1]+a[0][2]*v[2],
                              a[1][0]*v[0]+a[1][1]*v[1]+a[1][2]*v[2],
                              a[2][0]*v[0]+a[2][1]*v[1]+a[2][2]*v[2]]; }
function rpyR(r, p, y) { // URDF fixed-axis rpy: Rz(y)·Ry(p)·Rx(r)
  const cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
  const Rx=[[1,0,0],[0,cr,-sr],[0,sr,cr]];
  const Ry=[[cp,0,sp],[0,1,0],[-sp,0,cp]];
  const Rz=[[cy,-sy,0],[sy,cy,0],[0,0,1]];
  return mMul(Rz, mMul(Ry, Rx));
}
function axisR(ax, q) { // Rodrigues rotation about unit axis by q
  const [x,y,z]=ax, c=Math.cos(q), s=Math.sin(q), t=1-c;
  return [[t*x*x+c,   t*x*y-s*z, t*x*z+s*y],
          [t*x*y+s*z, t*y*y+c,   t*y*z-s*x],
          [t*x*z-s*y, t*y*z+s*x, t*z*z+c]];
}

// Forward kinematics -> world position of every joint frame + a few tips.
function computeFK(q) {
  const I = [[1,0,0],[0,1,0],[0,0,1]];
  const F = { root: { R: I, p: [0,0,0] } };
  for (const j of KIN) {
    const parent = F[j.par || "root"];
    const localR = mMul(rpyR(j.rpy[0], j.rpy[1], j.rpy[2]), axisR(j.axis, q[j.q]));
    const R = mMul(parent.R, localR);
    const dp = mVec(parent.R, j.xyz);
    F[j.n] = { R, p: [parent.p[0]+dp[0], parent.p[1]+dp[1], parent.p[2]+dp[2]] };
  }
  F.l_tip = { p: addLocal(F.l_wr, [0.10, 0, 0]) };  // hand
  F.r_tip = { p: addLocal(F.r_wr, [0.10, 0, 0]) };
  F.h_top = { p: addLocal(F.h_y,  [0.03, 0, 0.09]) }; // gaze / crown
  return F;
}
function addLocal(f, off) { const d = mVec(f.R, off); return [f.p[0]+d[0], f.p[1]+d[1], f.p[2]+d[2]]; }

/* ---------------------------------------------------------------------- */
/* Arm IK + self-collision check (used by Quest teleop)                    */
/* ---------------------------------------------------------------------- */
// URDF limits for the 12 commanded joints, same order as ARM_CMD_IDX and the
// bridge's ARM_MIN/ARM_MAX.
const ARM12_MIN = [-3.1416, -0.22689, -1.9199, -0.97564, -1.9199,
                   -3.1416, -2.47849, -1.9199, -0.97564, -1.9199, -0.62832, -2.0071];
const ARM12_MAX = [ 2.0944,  2.4784,   1.9199,  2.1852,   1.9199,
                    2.0944,  0.2268,   1.9199,  2.1852,   1.9199,  0.62832,  2.0071];
// Per side: the 4 position joints IK solves (state indices), the wrist joint,
// and the FK frame names.
const ARM_SIDE = {
  l: { pos: [IDX.L_SP, IDX.L_SR, IDX.L_SY, IDX.L_EL], wr: IDX.L_WR, el: "l_el", wrf: "l_wr", tip: "l_tip", k0: 0 },
  r: { pos: [IDX.R_SP, IDX.R_SR, IDX.R_SY, IDX.R_EL], wr: IDX.R_WR, el: "r_el", wrf: "r_wr", tip: "r_tip", k0: 5 },
};
function limit26(i, v) {
  const k = i - ARM_CMD_IDX[0];
  return Math.max(ARM12_MIN[k], Math.min(ARM12_MAX[k], v));
}
// Forearm direction = wrist frame +X (the wrist-roll axis).
function forearmDir(F, side) { const R = F[ARM_SIDE[side].wrf].R; return [R[0][0], R[1][0], R[2][0]]; }
// Upper-arm direction (shoulder -> elbow) = -Z of the shoulder-yaw frame (the yaw
// joint turns about the upper arm's long axis). At q = 0 it points straight down.
function upperArmDir(F, side) { const R = F[side + "_sy"].R; return [-R[0][2], -R[1][2], -R[2][2]]; }

// Damped least squares on the 4 position joints: move the hand tip to `tgt`
// and (weighted by dirW) point the forearm along unit vector `dir`.
// q26 is the seed (not modified). Returns { q: new 26-vector, err: metres }.
function solveArmIK(q26, side, tgt, dir, opt) {
  const o = Object.assign({ iters: 12, dirW: 0.08, lambda: 0.03, maxStep: 0.15 }, opt || {});
  const S = ARM_SIDE[side], q = q26.slice(), h = 1e-4;
  const resid = (F) => {
    const p = F[S.tip].p, x = forearmDir(F, side), e = [tgt[0]-p[0], tgt[1]-p[1], tgt[2]-p[2]];
    if (dir) for (let k = 0; k < 3; k++) e.push(o.dirW * (dir[k] - x[k]));
    return e;
  };
  let F = computeFK(q), e = resid(F);
  for (let it = 0; it < o.iters; it++) {
    if (Math.hypot(e[0], e[1], e[2]) < 2e-4) break;
    const m = e.length, J = [];            // m x 4, built column by column
    for (const i of S.pos) {
      const qq = q.slice(); qq[i] += h;
      const e2 = resid(computeFK(qq));
      J.push(e.map((v, r) => (v - e2[r]) / h));   // d(model)/dq = -d(resid)/dq
    }
    // (J^T J + lambda^2 I) dq = J^T e
    const A = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]], b = [0,0,0,0];
    for (let a = 0; a < 4; a++) {
      for (let c = 0; c < 4; c++) { let s = 0; for (let r = 0; r < m; r++) s += J[a][r] * J[c][r]; A[a][c] = s; }
      A[a][a] += o.lambda * o.lambda;
      let s = 0; for (let r = 0; r < m; r++) s += J[a][r] * e[r]; b[a] = s;
    }
    const dq = solve4(A, b);
    S.pos.forEach((i, a) => {
      const d = Math.max(-o.maxStep, Math.min(o.maxStep, dq[a]));
      q[i] = limit26(i, q[i] + d);
    });
    F = computeFK(q); e = resid(F);
  }
  return { q, err: Math.hypot(e[0], e[1], e[2]) };
}
// Whole-arm retargeting: point the robot's upper arm along unit vector `up` and
// its forearm along unit vector `fore` (both robot frame). 4 joints, 2 directions.
// Damped least squares from the seed q26 (not modified). Returns { q, err } where
// err is the summed direction error (|du| + |df|).
function solveArmDirIK(q26, side, up, fore, opt) {
  const o = Object.assign({ iters: 15, lambda: 0.05, maxStep: 0.2 }, opt || {});
  const S = ARM_SIDE[side], q = q26.slice(), h = 1e-4;
  const resid = (F) => {
    const u = upperArmDir(F, side), f = forearmDir(F, side);
    return [up[0]-u[0], up[1]-u[1], up[2]-u[2], fore[0]-f[0], fore[1]-f[1], fore[2]-f[2]];
  };
  const errOf = (e) => Math.hypot(e[0], e[1], e[2]) + Math.hypot(e[3], e[4], e[5]);
  let e = resid(computeFK(q));
  for (let it = 0; it < o.iters && errOf(e) > 1e-4; it++) {
    const J = S.pos.map((i) => {
      const qq = q.slice(); qq[i] += h;
      const e2 = resid(computeFK(qq));
      return e.map((v, r) => (v - e2[r]) / h);
    });
    const A = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]], b = [0,0,0,0];
    for (let a = 0; a < 4; a++) {
      for (let c = 0; c < 4; c++) { let t = 0; for (let r = 0; r < 6; r++) t += J[a][r] * J[c][r]; A[a][c] = t; }
      A[a][a] += o.lambda * o.lambda;
      let t = 0; for (let r = 0; r < 6; r++) t += J[a][r] * e[r]; b[a] = t;
    }
    const dq = solve4(A, b);
    S.pos.forEach((i, a) => { q[i] = limit26(i, q[i] + Math.max(-o.maxStep, Math.min(o.maxStep, dq[a]))); });
    e = resid(computeFK(q));
  }
  return { q, err: errOf(e) };
}
function solve4(A, b) {               // Gaussian elimination, partial pivoting
  const M = A.map((row, i) => row.concat([b[i]]));
  for (let c = 0; c < 4; c++) {
    let p = c; for (let r = c + 1; r < 4; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    if (Math.abs(M[c][c]) < 1e-12) continue;
    for (let r = 0; r < 4; r++) if (r !== c) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k < 5; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((row, i) => (Math.abs(row[i]) < 1e-12 ? 0 : row[4] / row[i]));
}

// Collision geometry from the r1_a5 STL bounding boxes (robot root frame).
// The checked points lie on the forearm AXIS; the forearm mesh is ~3.6 cm in
// radius, so every limit below includes that radius plus a margin.
const ARM_R = 0.036;
const BODY = {
  floorZ: -0.026 + ARM_R + 0.02,  // pelvis_link.STL bottom (mounting surface); home clears it by ~2 cm
  torso: { min: [-0.098 - ARM_R - 0.01, -0.100 - ARM_R - 0.01, 0.0],   // waist_yaw_link.STL box
           max: [ 0.071 + ARM_R + 0.01,  0.100 + ARM_R + 0.01, 0.294] },
  handsMin: 0.10,                 // forearm axis to forearm axis
  headMin: 0.15,                  // forearm to head centre (same as checkMotion)
};
// Points along each forearm: wrist origin, middle, end of the wrist mesh (14 cm).
function forearmPts(F, side) {
  const f = F[ARM_SIDE[side].wrf];
  return [f.p, addLocal(f, [0.07, 0, 0]), addLocal(f, [0.14, 0, 0])];
}
function inBox(p, b) { return p[0] > b.min[0] && p[0] < b.max[0] && p[1] > b.min[1] && p[1] < b.max[1] && p[2] > b.min[2] && p[2] < b.max[2]; }
// Clearance numbers of one pose: forearm-to-forearm distance, and per arm the
// lowest point, distance to the head centre, and whether a forearm is in the torso.
function poseMetrics(q26) {
  const F = computeFK(q26);
  const L = forearmPts(F, "l"), R = forearmPts(F, "r");
  const headC = F.h_y.p.map((v, k) => (v + F.h_top.p[k]) / 2);
  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  let hands = Infinity; for (const a of L) for (const b of R) hands = Math.min(hands, dist(a, b));
  const arm = (side, pts) => ({
    minZ: Math.min(...pts.concat([F[side + "_el"].p]).map((p) => p[2])),
    head: Math.min(...pts.map((p) => dist(p, headC))),
    torso: pts.some((p) => inBox(p, BODY.torso)),
  });
  return { hands, l: arm("l", L), r: arm("r", R) };
}
// Is moving `side` from pose `before` to `after` acceptable? Only that arm is
// judged, and a limit it already breaks only stops it from getting WORSE: the
// robot's own resting pose sits below floorZ, and the other, untouched arm must
// never block this one. Returns { ok, warn[] }.
function checkMove(before26, after26, side) {
  const b = poseMetrics(before26), a = poseMetrics(after26), eps = 1e-4;
  const name = side === "l" ? "left" : "right", A = a[side], B = b[side], warn = [];
  if (a.hands < BODY.handsMin && a.hands < b.hands - eps) warn.push("hands " + (a.hands * 100).toFixed(0) + " cm apart");
  if (A.head < BODY.headMin && A.head < B.head - eps) warn.push(name + " hand near head");
  if (A.torso && !B.torso) warn.push(name + " forearm into torso");
  if (A.minZ < BODY.floorZ && A.minZ < B.minZ - eps) warn.push(name + " arm too low");
  return { ok: !warn.length, warn };
}
// Self-collision / workspace check of one full pose. Returns { ok, warn[] }.
function checkPose(q26) {
  const F = computeFK(q26);
  const L = forearmPts(F, "l"), R = forearmPts(F, "r");
  const headC = F.h_y.p.map((v, k) => (v + F.h_top.p[k]) / 2);
  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  const warn = [];
  let hands = Infinity; for (const a of L) for (const b of R) hands = Math.min(hands, dist(a, b));
  if (hands < BODY.handsMin) warn.push("hands " + (hands * 100).toFixed(0) + " cm apart");
  for (const [side, pts] of [["left", L], ["right", R]]) {
    // the elbow can legitimately sit next to the torso (shoulder roll limit), so it is
    // only checked against the floor
    const low = pts.concat([F[side[0] + "_el"].p]);
    if (Math.min(...pts.map((p) => dist(p, headC))) < BODY.headMin) warn.push(side + " hand near head");
    if (pts.some((p) => inBox(p, BODY.torso))) warn.push(side + " forearm into torso");
    if (Math.min(...low.map((p) => p[2])) < BODY.floorZ) warn.push(side + " arm too low");
  }
  return { ok: !warn.length, warn, hands };
}

if (typeof module === "object" && module.exports) {
  module.exports = { IDX, ARM_CMD_IDX, KIN, ARM12_MIN, ARM12_MAX, ARM_SIDE, BODY,
    computeFK, addLocal, mVec, forearmDir, upperArmDir, forearmPts, solveArmIK, solveArmDirIK,
    checkPose, checkMove, poseMetrics, limit26 };
}
