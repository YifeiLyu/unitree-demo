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
# deploy (from repo root): never sync bridge/build, it is built on the robot
rsync -av --exclude .git --exclude bridge/build --exclude __pycache__ \
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

### Checks you can run locally

```bash
node --check web/app.js && node --check web/model3d.js
python3 -m py_compile server.py
```

## Architecture

```
browser ──HTTP / WS──► server.py (tornado :8080) ──stdin/stdout JSON lines──► bridge/build/magikid_bridge eth10 50
   ▲                       │                                                     │ unitree_sdk2 / CycloneDDS on eth10
   └──── MJPEG /cam/N ◄────┘ cv2 + V4L2                                          ├─ sub rt/lowstate       (hg LowState_)
                                                                                 ├─ sub rt/secondary_imu  (torso IMU)
                                                                                 └─ pub rt/lowcmd 200 Hz  (hg LowCmd_, only while engaged)
```

| Path | Role |
|---|---|
| `server.py` | tornado app. Routes: `/` (index), `/ws` (state broadcast + commands), `/cam/N` (MJPEG), `/api/meta` (camera list), static `web/`. Spawns and owns the bridge process. `scan_cameras()` auto-detects capture nodes through `/dev/v4l/by-id/*index0` and labels them head (RealSense) or gripper (JR0001/JR0002). |
| `bridge/magikid_bridge.cpp` | The only DDS process. Streams `{"type":"state",...}` at the given rate (50 Hz) and `{"type":"log","msg":...}` lines on stdout. Reads commands from stdin. Owns arm control and trajectory playback. |
| `web/index.html`, `style.css` | Single-page dashboard: Arm Pose, Cameras, Body IMU, Joints, Arm Command (with the Motions box). |
| `web/app.js` | All client logic: WS client, joint tables, command sliders, SVG FK fallback (`KIN`/`computeFK`), scripted motions (`buildMotion`, `checkMotion`, preview). |
| `web/model3d.js` | `window.Arm3D`: three.js r128 viewer that loads the 14 STL meshes into the URDF kinematic tree. If three.js or the meshes fail to load, `app.js` keeps the SVG stick figure. |
| `web/vendor/` | three.js r128 + `STLLoader` + `OrbitControls` (classic globals, self-hosted). |
| `web/model/meshes/` | r1_a5 STL meshes (from `unitreerobotics/unitree_ros`, `robots/r1_a5_description`). |
| `r1_a5.urdf` | The source for joint origins, axes and limits. `KIN` in app.js, `LINKS` in model3d.js and `ARM_MIN/ARM_MAX` in the bridge all come from it. Read it instead of trusting memory. |

### WebSocket / bridge command protocol

`server.py` forwards a browser message **only if its `type` is on the whitelist
in `StateWSHandler.on_message`**. If you add a bridge command, add it there too,
or it is silently dropped.

| `type` | Payload | Effect |
|---|---|---|
| `arm_engage` | none | Releases the motion service, seeds targets from the measured pose, starts publishing `rt/lowcmd`. Needs a lowstate first. |
| `arm_release` / `estop` | none | Stops any motion, stops publishing, `SelectMode` back to the saved motion service. |
| `arm_speed` (alias `arm_weight`) | `value` rad/s | Manual move velocity limit, clamped to 0.02–1.5. Default 0.3 in the bridge; the UI sends 0.2. |
| `arm_set` | `q`: 12 floats | Manual targets, clamped to URDF limits. Cancels a running motion. |
| `traj` | `name`, `dur`: [s…], `q`: 12×N floats | Scripted motion. Segments are cosine-eased from the current `q_cmd`. Max 64 segments, 0.2–30 s each, 1.0 rad/s hard cap. Rejected unless engaged. |
| `traj_stop` | none | Stops the motion and holds the current pose. |
| `loco` | — | **Legacy, ignored by the bridge.** Still on the whitelist and in `wireModeButtons()` (app.js), which is never called. Safe to delete. |

State JSON fields: `have_state`, `crc_ok`, `seq`, `mode_machine`,
`arm_engaged`, `arm_speed`, `traj_active`, `traj_name`, `traj_prog`,
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
