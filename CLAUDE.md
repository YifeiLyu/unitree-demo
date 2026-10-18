# CLAUDE.md

Guidance for Claude Code (and humans) working in this repo: the **Magikid web
teleop / telemetry dashboard** for a **Unitree R1-A** (URDF model `r1_a5`).

## What the robot is

- R1-A is **upper body only**: two 5-DoF arms (shoulder pitch/roll/yaw, elbow,
  wrist roll) and a 2-DoF head (pitch, yaw). It has no legs, so it has no
  locomotion stack.
- The compute unit is an **Orin Nano** (Ubuntu 20.04, JetPack 5, Python 3.8),
  reached over direct ethernet at **`192.168.123.164`**, SSH user `unitree`.
  `192.168.123.161` answers ping but drops every TCP port. That is normal. Don't
  debug it.
- The robot has **no internet**. Download dependencies on the dev machine and
  copy them over. Everything the browser loads must be self-hosted in `web/`,
  because the browser can only reach the robot. CDN links will break.
- On-board toolchain: gcc 9, cmake, cv2 4.1.1, tornado, rsync, git, and a prebuilt
  `unitree_sdk2` (`/usr/local/lib/libunitree_sdk2.a`, source in
  `~/unitree_sdk2-main`). **Not available:** pycyclonedds / unitree_sdk2py,
  pyrealsense2, node. Write DDS code in C++, not Python.

### Dev-machine network

The robot runs no DHCP. Give the ethernet interface a static IP such as
`192.168.123.222/24` with the **router/gateway field left blank**. A gateway of
`192.168.123.1` on that interface steals the default route and kills internet
on the dev machine while the cable is plugged in.

## Where the code runs

This repo is the source. The deployed copy lives **on the robot** at
`~/magikid_teleop/`. Nothing here runs on the dev machine except syntax checks.

```bash
# deploy (from repo root): never sync bridge/build (built on the robot) or the
# robot's own certs/ + teleop_key (generated there on first start)
rsync -av --exclude .git --exclude bridge/build --exclude __pycache__ \
  --exclude certs --exclude teleop_key \
  ./ unitree@192.168.123.164:~/magikid_teleop/

# run on the robot
ssh unitree@192.168.123.164
cd ~/magikid_teleop && ./start.sh      # backgrounds run.sh, safe to rerun, log in /tmp/magikid_server.log
# or: ./run.sh [iface=eth10] [port=8080]   (foreground; builds the bridge if missing)
# open http://192.168.123.164:8080/
```

The dashboard does **not** start automatically on boot.

### Building the bridge (on the robot)

```bash
cd ~/magikid_teleop/bridge && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make clean && make -j4
strings magikid_bridge | grep <a-new-symbol>    # confirm the new code is in
```

The robot's clock is wrong (it reads 2024), so after a copy `make` can decide a
new `.cpp` is up to date and skip it. **Always `make clean` after deploying
bridge changes**, then check the binary with `strings`.

### Checks you can run locally (no robot needed)

```bash
for f in web/*.js; do node --check "$f"; done && python3 -m py_compile server.py
node tests/test_teleop.js        # IK, collision check, Quest mapping
node tests/test_quest_page.js    # quest.js against a mock WebXR session
python3 tests/test_server.py     # server.py + tests/fake_bridge.py: listeners, key, whitelists
```

The C++ bridge needs the robot's SDK headers. To syntax-check it without
writing anything on the robot, pipe it to the robot's compiler:
`ssh unitree@192.168.123.164 'g++ -std=c++17 -fsyntax-only -Wall -x c++ -I/usr/local/include -I/usr/local/include/ddscxx -I/usr/local/include/iceoryx/v2.0.2 -' < bridge/magikid_bridge.cpp`

## Architecture

```
browser ──HTTP / WS──► server.py (tornado :8080) ──stdin/stdout JSON lines──► bridge/build/magikid_bridge eth10 50
   ▲                       │  + HTTPS :8443 (Quest)                              │ unitree_sdk2 / CycloneDDS on eth10
   └──── MJPEG /cam/N ◄────┘ cv2 + V4L2                                          ├─ sub rt/lowstate       (hg LowState_)
                                                                                 ├─ sub rt/secondary_imu  (torso IMU)
Quest 3S ──Wi-Fi──► Mac :8443 ──ssh -L──► robot :8443                            └─ pub rt/lowcmd 200 Hz  (hg LowCmd_, only while engaged)
```

| Path | Role |
|---|---|
| `server.py` | tornado app. Routes: `/` (index), `/ws` (state broadcast + commands), `/cam/N` (MJPEG), `/api/meta` (camera list), static `web/`. Spawns and owns the bridge process. `scan_cameras()` auto-detects capture nodes through `/dev/v4l/by-id/*index0` and labels them head (RealSense) or gripper (JR0001/JR0002). Also runs a second, HTTPS listener for the Quest (see below). |
| `bridge/magikid_bridge.cpp` | The only DDS process. Streams `{"type":"state",...}` at the given rate (50 Hz) and `{"type":"log","msg":...}` lines on stdout. Reads commands from stdin. Owns arm control and trajectory playback. |
| `web/index.html`, `style.css` | Single-page dashboard: Arm Pose, Cameras, Body IMU, Joints, Arm Command (with the Motions box). |
| `web/kin.js` | Shared kinematics, loaded before `app.js` and `quest.js`: `IDX`, `ARM_CMD_IDX`, `KIN` (URDF chain), `computeFK`, plus `solveArmIK` (damped least squares on the 4 position joints) and `checkPose` (self-collision limits in `BODY`). Plain globals, also `require`-able from Node. |
| `web/app.js` | Dashboard logic: WS client, joint tables, command sliders, SVG skeleton fallback, scripted motions (`buildMotion`, `checkMotion`, preview). |
| `web/quest.html`, `quest.js`, `teleop.js` | Meta Quest teleop page. `teleop.js` holds the pure mapping math (`TeleopCore`), `quest.js` the WebXR / three.js / WebSocket glue. |
| `scripts/quest_tunnel.sh` | Run on the Mac: relays the robot's :8443 onto the Mac's Wi-Fi and prints the Quest URL with its key. |
| `tests/` | Node and Python tests that run without the robot. `fake_bridge.py` speaks the bridge's stdin/stdout protocol. |
| `web/model3d.js` | `window.Arm3D`: three.js r128 viewer that loads the 14 STL meshes into the URDF kinematic tree. If three.js or the meshes fail to load, `app.js` keeps the SVG stick figure. |
| `web/vendor/` | three.js r128 + `STLLoader` + `OrbitControls` (classic globals, self-hosted). |
| `web/model/meshes/` | r1_a5 STL meshes (from `unitreerobotics/unitree_ros`, `robots/r1_a5_description`). |
| `r1_a5.urdf` | The source for joint origins, axes and limits. `KIN` in app.js, `LINKS` in model3d.js and `ARM_MIN/ARM_MAX` in the bridge all come from it. Read it instead of trusting memory. |

### WebSocket / bridge command protocol

`server.py` forwards a browser message **only if its `type` is on that
listener's whitelist**: `DASHBOARD_CMDS` for :8080, `QUEST_CMDS` for :8443
(top of `server.py`). If you add a bridge command, add it there too, or it is
silently dropped.

| `type` | Payload | Effect |
|---|---|---|
| `arm_engage` | none | Releases the motion service, seeds targets from the measured pose, starts publishing `rt/lowcmd`. Needs a lowstate first. |
| `arm_release` / `estop` | none | Stops any motion, stops publishing, `SelectMode` back to the saved motion service. |
| `arm_speed` (alias `arm_weight`) | `value` rad/s | Manual move velocity limit, clamped to 0.02–1.5. Default 0.3 in the bridge; the UI sends 0.2. |
| `arm_set` | `q`: 12 floats | Manual targets, clamped to URDF limits. Cancels a running motion. |
| `traj` | `name`, `dur`: [s…], `q`: 12×N floats | Scripted motion. Segments are cosine-eased from the current `q_cmd`. Max 64 segments, 0.2–30 s each, 1.0 rad/s hard cap. Rejected unless engaged. |
| `traj_stop` | none | Stops the motion and holds the current pose. |
| `teleop` | `q`: 12 floats, `mask`: [left, right, head], `vel` rad/s | Quest stream (~30 Hz). Applies only the masked groups, never head yaw, clamped to URDF limits. Velocity cap is `vel`, clamped to 0.05–1.0. An all-zero mask is a heartbeat. Ignored unless engaged, and while a `traj` plays. If no frame arrives for 0.3 s while live, the bridge holds the current pose. |
| `teleop_stop` | none | Ends teleop and holds the current pose (the headset's FREEZE). |
| `loco` | — | **Legacy, ignored by the bridge.** Still on the whitelist and in `wireModeButtons()` (app.js), which is never called. Safe to delete. |

State JSON fields: `have_state`, `crc_ok`, `seq`, `mode_machine`,
`arm_engaged`, `arm_speed`, `traj_active`, `traj_name`, `traj_prog`,
`teleop_live`, `q_cmd` (the 12 joint angles actually being commanded),
`head_yaw_enabled`, and 26-wide `q`/`dq`/`tau`/`temp`/`vol`/`mstate`, plus
`imu_rpy`/`imu_gyro`/`imu_acc`/`imu_quat`, `have_torso`, `torso_rpy`.

## Joint indexing (three different index spaces)

| Joint | 12-vector (`arm_set`, `traj`, `ARM_*`) | State index (`q[]`, 0–25) | IDL motor slot (`motor_cmd`/`motor_state`) | Kp / Kd |
|---|---|---|---|---|
| L shoulder pitch / roll / yaw, elbow, wrist roll | 0–4 | 14–18 | 15–19 | 100,100,100,100,50 / 2 |
| R shoulder pitch / roll / yaw, elbow, wrist roll | 5–9 | 19–23 | 22–26 | 100,100,100,100,50 / 2 |
| Head pitch | 10 | 24 | 29 | 50 / 2 |
| Head yaw | 11 | 25 | 30 | 10 / 0.1 (**disabled**, see below) |

State indices 0–11 (legs) and 12–13 (waist roll/yaw) exist in the message but
aren't actuated on R1-A. Never put a nonzero `kp` on a slot you don't control.
Leave it at mode 0 with zero gains.

Sign conventions (from the URDF FK): at q = 0 the forearm points straight
forward. Shoulder pitch < 0 moves the arm forward/up. Elbow > 0 moves the
forearm down. The default home pose is `[-0.8, 0, 0, 0.8, 0]` per arm. Each
viewer can override it in the browser (`localStorage` key `magikid.home.v1`).

## Control model: read before touching control code

These look right but **do not work** on R1-A:
- `rt/arm_sdk` weight blending: there's no high-level controller to blend onto.
- `LocoClient` / FSM ids (Damp, StandUp, Start = 811): `GetFsmId` times out. There's no loco service.
- `mode_machine` in LowState is the robot **type** id (R1-A reports 3), not an
  operating mode. Echo it back in every LowCmd.

What works (verified on hardware 2026-10-02), modeled on the official
`unitree_sdk2/example/r1/low_level/r1A_wrist_swing_example.cpp`:
1. `MotionSwitcherClient` (`unitree/robot/b2/motion_switcher/`): `CheckMode`, then
   `ReleaseMode` until the name is empty. Remember the name.
2. Seed `q_target = q_cmd =` the measured `q`, so the first frame holds position.
3. Publish `hg LowCmd_` on `rt/lowcmd` every 5 ms: per joint `mode=1, q, dq=0, kp, kd, tau=0`;
   `mode_pr=0`, echo `mode_machine`, and set `crc = Crc32Core(..., (sizeof>>2)-1)`.
   **A bad CRC makes the robot ignore the frame silently.**
4. Velocity-limit every step toward the target and clamp it to URDF limits.
5. On release/e-stop: stop publishing, then `SelectMode(saved_name)`.

Only one process may publish `rt/lowcmd`, because two publishers fight. Add
new behaviors (poses, sequences, recording) as new bridge stdin commands. Don't
write a second publisher.

**Head yaw is disabled** (`HEAD_YAW_ENABLED = false` in the bridge). Its encoder
reads about 1.7–2.0 rad while the head faces forward, and the offset changes
between power-ons, so commanding it could swing the head hard to one side. Slot 30
is never powered. The UI disables that slider and `headForward()` always draws
the head facing forward. Scripted motions never command the head. The real fix
is a motor-zero recalibration by Unitree.

## Meta Quest teleop

The robot has no Wi-Fi, so the Quest reaches it through the Mac: the Quest and
the Mac share a Wi-Fi network, and `scripts/quest_tunnel.sh` relays the Mac's
:8443 to the robot over SSH (`ssh -g -L`). WebXR only runs in a secure
context, so the robot serves `quest.html` over HTTPS with a self-signed
certificate (`certs/`, made with openssl on first start). The Quest Browser
asks once to accept it.

Run order: on the robot `./start.sh`. On the dashboard, ARM CONTROL → Engage.
On the Mac `scripts/quest_tunnel.sh`. On the Quest, open the printed URL and
tap Enter passthrough.

Safety layers, outermost first:
1. **Key.** The :8443 WebSocket needs `?key=` from `~/magikid_teleop/teleop_key`
   (8 random characters, made on first start). Without it the upgrade gets a 403.
   The relay puts this port on the Mac's Wi-Fi, so the key matters.
2. **Whitelist.** :8443 may only send `teleop`, `teleop_stop` and `estop`
   (stopping is always allowed). Engaging and motions are dashboard-only, so
   someone at the Mac has to engage first.
3. **Page.** `TeleopCore` runs `checkMove(before, after, side)` on every IK
   result and keeps the last safe target instead. The limits: a forearm within
   10 cm of the other forearm, within 15 cm of the head centre, into the torso
   box, or below `floorZ`. They are in `BODY` in `kin.js`, taken from the STL
   bounding boxes with the 3.6 cm forearm radius allowed for. Two rules matter:
   - **Only the moving arm is judged.** The untouched arm never blocks it.
   - **A limit already broken only stops it getting worse.** The robot's own
     resting pose (arms hanging, lowest point z ≈ 0.006) is below `floorZ`
     (0.030), so an arm starting there may move up or sideways but not lower.
   `checkPose` (whole pose, absolute) is still used by the tests.
4. **Bridge.** URDF clamping, a velocity cap of at most 1.0 rad/s, a 0.3 s
   watchdog, and head yaw is never commanded.

**In-headset view.** A dark "stage" (front-left, so the middle stays clear to
see the real robot) shows **YOU**, a figure built from the headset and
controller poses (shoulders and elbows estimated, `drawHuman`), next to
**ROBOT**: gray = measured pose, green = target, and a ball where your hand
is asking the robot hand to go (`TeleopCore.tgt`). A white line from the hand
to the ball means the robot can't reach it, and a red ball means the pose was
refused. Bones are cylinders, because WebGL lines are 1 px. The status panel
sits front-right. Both can be dragged with the trigger.

**Whole-arm mode** (setting "Tracking → Whole arm"). This uses WebXR body
tracking in the Quest Browser: optional feature `body-tracking`, read from
`frame.body.get("left-arm-upper" | "left-arm-lower" | "left-hand-wrist" | "chest" | "hips" | …)`
via `frame.getPose`. The Quest provides `frame.body` even though it doesn't
list the feature in `enabledFeatures`. `bodyFrame()` builds your torso frame
from the shoulders and hips→chest, so turning your body doesn't move the robot.
`solveArmDirIK` (kin.js) then points the robot's upper arm (−Z of the
shoulder-yaw frame) and forearm (+X of the wrist frame) along yours. That is
4 joints for 2 directions, and body size doesn't matter. Wrist roll still
comes from the controller twist. It's an absolute mapping, so each grab starts
at ≤ 0.25 rad/s for 1.5 s (`SOFT_MS`/`SOFT_VEL` in quest.js). Before you grab,
the green model previews the pose your arm maps to (`TeleopCore.view` /
`displayQ`). With no body data, a gripped arm just holds ("no body tracking").
Quest 3/3S can track elbows with the cameras (IOBT), but whether the browser
uses that or only infers elbows from head + hands is untested. To check: keep
the hand still, move the elbow, and see whether YOU's elbow follows.

**Preview.** While the arms are not engaged, the grips still drive the green
target, but nothing is sent ("PREVIEW" on the panel, yellow cone). When
the arms become engaged, `core.release()` throws the preview pose away, so the
robot never jumps to it.

Mapping (`teleop.js`): holding a grip makes that hand move the robot hand tip
relative to where the grip started, times the hand scale (0.7 by default).
Controller rotation turns the forearm direction (low weight in the IK), and the
twist about the pointing axis drives wrist roll. "Forward" is the operator's
horizontal facing when they enter or click a thumbstick, and the operator is
assumed to face the same way as the robot. Buttons (xr-standard indices):
squeeze `[1]` = clutch, trigger `[0]` = drag the status panel or the stage (by its backdrop)
(ray hit-test; thumbstick axis `[3]` pushes it farther or pulls it closer while
dragging), A/X `[4]` = cycle arms (both / left only / right only, also in the
page settings), B/Y `[5]` tap = freeze (`teleop_stop`, hold pose), B/Y held
1 s = `estop` (release, same as the dashboard E-STOP), thumbstick press `[3]` = recentre. Page
settings are remembered in the headset's localStorage (`magikid.quest.v1`).
Head pitch follows the headset only when that option is ticked.

The robot's reach is small: home already has the hand near full forward
reach, so expect the hand to stop at the workspace edge. Verified so far: unit
tests, the mock-XR test and the server integration test. **Not yet run on the
headset or the robot.**

## Safety rules

- **Claude must never run motion itself.** That means no engaging, no `traj`, and
  no official motion examples. Someone has to stand by the robot. Claude builds,
  compiles and checks syntax, then hands the user the exact steps and describes
  the motion to expect.
- Fine to run without asking: SSH, reading files, `ps`, camera checks,
  compiling, starting the server (it stays read-only until a browser engages), and reading `rt/lowstate`.
- New motion code needs four things: a hold-on-engage seed, a velocity limit,
  URDF clamping, and a release/e-stop path that restores the motion service.
  Start with low speed and small amplitude.
- Before restarting the server or bridge, check that `arm_engaged` is false.
  Killing the bridge while engaged stops the frames, and the arms can drop.
- Over SSH, `pkill -f "<pattern>"` can match the SSH shell's own command line
  and kill your session. Use `pgrep -f` or `ps` to get PIDs, then `kill <pid>`.

## Hardware notes

- Cameras on the Orin: two UVC wrist/gripper cams. JR0001 is `/dev/video2` and
  JR0002 is `/dev/video0`. `video1` and `video3` are metadata nodes.
- The head camera is an Intel RealSense D435. It isn't attached to the Orin as of
  2026-10. If it gets plugged in, `scan_cameras()` picks up its color stream
  through V4L2 and adds a "Head (RealSense)" tile. pyrealsense2 isn't installed,
  so there is no depth stream.
- Troubleshooting:
  - Dashboard loads but shows no state: check that the interface is `eth10` and
    that no other DDS process is running. The bridge's stderr is in the server
    log.
  - Arms don't move and there are no errors: the motion service wasn't
    released, or the CRC or `mode_machine` is wrong, or the code published on
    `rt/arm_sdk`.
