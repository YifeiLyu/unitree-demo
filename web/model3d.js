/* Magikid R1-A — WebGL URDF mesh viewer (three.js r128, classic globals).
   Loads the r1_a5 STL meshes into a kinematic tree and drives the 13 upper-body
   joints from live telemetry. If three.js or the meshes are unavailable it stays
   inert (Arm3D.ready === false) and app.js keeps using the SVG stick figure. */
window.Arm3D = (function () {
  "use strict";
  const MESH_BASE = "model/meshes/";
  const MAT = {           // URDF rgba -> hex
    silvery:   0xcad1ee,
    dark_gray: 0x6a6d74,
  };
  // Link table. Root (pelvis) has no joint; every other link carries the fixed
  // origin (xyz + rpy, from the URDF) and the driven joint (axis + state index).
  const LINKS = [
    { name: "pelvis_link",              mesh: "pelvis_link.STL",              mat: "silvery",   parent: null },
    { name: "waist_yaw_link",           mesh: "waist_yaw_link.STL",           mat: "silvery",   parent: "pelvis_link",           xyz: [0,0,0],                        rpy: [0,0,0],        axis: [0,0,1], qi: 13 },
    { name: "left_shoulder_pitch_link", mesh: "left_shoulder_pitch_link.STL", mat: "dark_gray", parent: "waist_yaw_link",        xyz: [0,0.085688,0.19749],           rpy: [0.26187,0,0],  axis: [0,1,0], qi: 14 },
    { name: "left_shoulder_roll_link",  mesh: "left_shoulder_roll_link.STL",  mat: "silvery",   parent: "left_shoulder_pitch_link", xyz: [0.03445,0.047132,-0.025693], rpy: [-0.26187,0,0], axis: [1,0,0], qi: 15 },
    { name: "left_shoulder_yaw_link",   mesh: "left_shoulder_yaw_link.STL",   mat: "silvery",   parent: "left_shoulder_roll_link",  xyz: [-0.03445,0.0043,-0.10835],   rpy: [0,0,0],        axis: [0,0,1], qi: 16 },
    { name: "left_elbow_link",          mesh: "left_elbow_link.STL",          mat: "silvery",   parent: "left_shoulder_yaw_link",   xyz: [0.016191,0.026461,-0.082858],rpy: [0,0,0],        axis: [0,1,0], qi: 17 },
    { name: "left_wrist_roll_link",     mesh: "left_wrist_roll_link.STL",     mat: "silvery",   parent: "left_elbow_link",          xyz: [0.11218,-0.03002,-0.011702], rpy: [0,0,0],        axis: [1,0,0], qi: 18 },
    { name: "right_shoulder_pitch_link",mesh: "right_shoulder_pitch_link.STL",mat: "dark_gray", parent: "waist_yaw_link",        xyz: [0,-0.085688,0.19749],          rpy: [-0.26187,0,0], axis: [0,1,0], qi: 19 },
    { name: "right_shoulder_roll_link", mesh: "right_shoulder_roll_link.STL", mat: "silvery",   parent: "right_shoulder_pitch_link",xyz: [0.03445,-0.047132,-0.025693],rpy: [0.26187,0,0],  axis: [1,0,0], qi: 20 },
    { name: "right_shoulder_yaw_link",  mesh: "right_shoulder_yaw_link.STL",  mat: "silvery",   parent: "right_shoulder_roll_link", xyz: [-0.03445,-0.0043,-0.10835],  rpy: [0,0,0],        axis: [0,0,1], qi: 21 },
    { name: "right_elbow_link",         mesh: "right_elbow_link.STL",         mat: "silvery",   parent: "right_shoulder_yaw_link",  xyz: [0.016191,-0.026461,-0.082858],rpy:[0,0,0],        axis: [0,1,0], qi: 22 },
    { name: "right_wrist_roll_link",    mesh: "right_wrist_roll_link.STL",    mat: "silvery",   parent: "right_elbow_link",         xyz: [0.11218,0.03002,-0.011702],  rpy: [0,0,0],        axis: [1,0,0], qi: 23 },
    { name: "head_pitch_link",          mesh: "head_pitch_link.STL",          mat: "silvery",   parent: "waist_yaw_link",        xyz: [-0.006,0.03155,0.255],         rpy: [0,0,0],        axis: [0,1,0], qi: 24 },
    { name: "head_yaw_link",            mesh: "head_yaw_link.STL",            mat: "silvery",   parent: "head_pitch_link",          xyz: [0.0010126,-0.03155,0.1165],  rpy: [0,0,0],        axis: [0,0,1], qi: 25 },
  ];
  // camera positions (x,y,z); robot +X = forward/face, +Y = left, +Z = up.
  const VIEWS = {
    front: [1.45, 0.0, 0.22],    // in front of the robot, looking at its face
    side:  [0.0, 1.45, 0.22],    // robot's left side
    iso:   [1.15, -0.85, 0.42],  // front-right 3/4 (face + right arm)
    top:   [0.02, 0.0, 1.55],    // directly above
  };
  const TARGET = [0, 0, 0.2];

  let renderer, scene, camera, controls, THREEref;
  const jointNodes = [];   // { node: Group, axis: Vector3, qi }
  let ready = false, started = false, lastQ = null;

  // URDF fixed-axis rpy (Rz*Ry*Rx) -> THREE.Quaternion (matches the FK helper).
  function rpyQuat(r, p, y) {
    const cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
    const m = new THREEref.Matrix4();
    m.set(
      cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, 0,
      sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, 0,
      -sp,   cp*sr,            cp*cr,            0,
      0,0,0,1);
    return new THREEref.Quaternion().setFromRotationMatrix(m);
  }

  function init(container) {
    THREEref = window.THREE;
    if (!THREEref || !THREEref.STLLoader || !container) return false;
    const W = container.clientWidth || 320, H = container.clientHeight || 360;

    scene = new THREEref.Scene();
    scene.background = new THREEref.Color(0x12151d);

    camera = new THREEref.PerspectiveCamera(45, W / H, 0.05, 50);
    camera.up.set(0, 0, 1);                 // URDF is Z-up

    renderer = new THREEref.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    renderer.setSize(W, H);
    container.appendChild(renderer.domElement);

    scene.add(new THREEref.AmbientLight(0xffffff, 0.65));
    const key = new THREEref.DirectionalLight(0xffffff, 0.9); key.position.set(1, -1.5, 2); scene.add(key);
    const fill = new THREEref.DirectionalLight(0x88aaff, 0.35); fill.position.set(-1.5, 1, 0.5); scene.add(fill);

    if (THREEref.OrbitControls) {
      controls = new THREEref.OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true; controls.dampingFactor = 0.1;
      controls.target.set(TARGET[0], TARGET[1], TARGET[2]);
    }
    setView("front");

    // Build the kinematic tree.
    const groups = {};
    const robot = new THREEref.Group();
    scene.add(robot);

    let pending = LINKS.length;
    const loader = new THREEref.STLLoader();

    LINKS.forEach((L) => {
      // frame the mesh attaches to (post joint-rotation for children)
      let attach;
      if (L.parent === null) {
        attach = robot;
        groups[L.name] = attach;
      } else {
        const parent = groups[L.parent];
        const fixed = new THREEref.Group();               // fixed origin (xyz+rpy)
        fixed.position.set(L.xyz[0], L.xyz[1], L.xyz[2]);
        fixed.quaternion.copy(rpyQuat(L.rpy[0], L.rpy[1], L.rpy[2]));
        parent.add(fixed);
        const jr = new THREEref.Group();                  // driven joint rotation
        fixed.add(jr);
        groups[L.name] = jr;
        jointNodes.push({ node: jr, axis: new THREEref.Vector3(L.axis[0], L.axis[1], L.axis[2]).normalize(), qi: L.qi });
      }
      const color = MAT[L.mat] || 0x9aa3b5;
      const material = new THREEref.MeshStandardMaterial({ color, metalness: 0.55, roughness: 0.45 });
      loader.load(MESH_BASE + L.mesh, (geom) => {
        const mesh = new THREEref.Mesh(geom, material);
        groups[L.name].add(mesh);
        if (--pending === 0) { ready = true; Arm3D.ready = true; }
      }, undefined, (err) => {
        console.warn("[Arm3D] mesh load failed", L.mesh, err);
        if (--pending === 0 && jointNodes.length) { ready = true; Arm3D.ready = true; }
      });
    });

    window.addEventListener("resize", onResize);
    animate();
    return true;
  }

  function onResize() {
    const c = renderer && renderer.domElement.parentElement;
    if (!c) return;
    const W = c.clientWidth, H = c.clientHeight;
    camera.aspect = W / H; camera.updateProjectionMatrix(); renderer.setSize(W, H);
  }

  function setView(name) {
    const v = VIEWS[name]; if (!v || !camera) return;
    camera.position.set(v[0], v[1], v[2]);
    if (controls) controls.target.set(TARGET[0], TARGET[1], TARGET[2]);
    camera.lookAt(TARGET[0], TARGET[1], TARGET[2]);
    if (controls) controls.update();
  }

  function setJoints(q) {
    lastQ = q;
    if (!ready) return;
    for (const j of jointNodes) j.node.quaternion.setFromAxisAngle(j.axis, q[j.qi] || 0);
  }

  function animate() {
    started = true;
    requestAnimationFrame(animate);
    if (controls) controls.update();
    if (renderer && scene && camera) renderer.render(scene, camera);
  }

  return { init, setJoints, setView, get isReady() { return ready; }, ready: false };
})();
