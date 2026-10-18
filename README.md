# Magikid · Unitree R1-A Web Console

A browser dashboard for the Magikid lab's **Unitree R1-A** (upper-body
humanoid, URDF `r1_a5`). It shows the live arm pose on a 3D model, telemetry
for every joint, the body IMU and the onboard cameras. It also has a
**safety-gated** panel for commanding both arms over low-level `rt/lowcmd`,
and scripted motions (go home, wave).

It runs entirely **on the robot's Orin** (Ubuntu 20.04). It needs no internet
and no extra `pip`/`apt` packages: it uses the pre-installed `unitree_sdk2`,
`python3`, `tornado` and `cv2`.

```
 browser ──HTTP/WS──► server.py (tornado) ──stdin/stdout(JSON)──► magikid_bridge (C++)
                          │                                            │  DDS (CycloneDDS, eth10)
                          └── MJPEG ◄── cv2 ── /dev/video*             ▼
                                                              rt/lowstate      (read)
                                                              rt/secondary_imu (read)
                                                              rt/lowcmd        (write, only while engaged)
```

## Deploy and run

Connect the robot over ethernet and give your machine a static IP on
`192.168.123.0/24` (e.g. `192.168.123.222`), **leaving the router field blank**.

```bash
# copy to the robot (the bridge is built there)
rsync -av --exclude .git --exclude bridge/build ./ unitree@192.168.123.164:~/magikid_teleop/

# on the robot
ssh unitree@192.168.123.164
cd ~/magikid_teleop
./start.sh            # background; or ./run.sh [iface=eth10] [port=8080] in the foreground
```

Then open **http://192.168.123.164:8080/**.

`run.sh` builds the bridge the first time. After changing
`bridge/magikid_bridge.cpp`, rebuild with
`cd bridge/build && make clean && make`. The robot's clock is wrong, so a plain
`make` may skip the new file.

## Panels

- **Arm Pose:** a WebGL model built from the real r1_a5 STL meshes, posed by
  live joint angles. Drag to orbit, or use the front, side, 3/4 and top
  buttons. If WebGL or the meshes aren't available, it shows a stick figure
  instead.
- **Cameras:** the wrist/gripper UVC cameras, streamed as MJPEG and detected
  automatically. A RealSense head camera gets its own tile when it's plugged in.
- **Body IMU:** artificial horizon (roll/pitch), yaw, gyro, accel and torso RPY.
- **Joints:** angle, velocity and torque for the arms and head, or for all 26
  slots. A row turns red when its motor reports an error.
- **Arm Command:** 12 target sliders, a move-speed limit, presets, E-STOP, and
  **Motions** (Go home / Wave left / Wave right). Each motion is previewed on
  the 3D model with a clearance check before it runs.

## Safety model

1. The bridge is **read-only** until it receives an explicit `arm_engage`.
2. The **ARM CONTROL** master switch is off by default and asks for
   confirmation before it unlocks the panel.
3. **Engage** releases the robot's built-in motion service and then **holds the
   current pose**. Targets start at the measured angles, so nothing jumps.
4. Every move is velocity-limited (default 0.2 rad/s from the UI) and clamped
   to the URDF joint limits. Scripted motions are hard-capped at 1.0 rad/s.
5. **E-STOP / release** stops publishing and hands control back to the robot's
   motion service. Be ready to support the arms when you release.
6. **Head yaw is disabled.** Its sensor offset varies between power-ons, so it
   is never powered or commanded.

Verified on hardware: arm control on 2026-10-02, motions on 2026-10-04. Always
keep the workspace clear and have someone at the E-STOP.

## Files

```
├── run.sh / start.sh       launcher (builds bridge if needed) / background starter
├── server.py               tornado: static + WebSocket + MJPEG + bridge process
├── r1_a5.urdf              joint origins, axes, limits (source of truth)
├── bridge/
│   ├── magikid_bridge.cpp  DDS <-> JSON bridge and arm control (unitree_sdk2)
│   └── CMakeLists.txt
└── web/
    ├── index.html, style.css
    ├── app.js              dashboard logic, FK, motions
    ├── model3d.js          three.js URDF mesh viewer
    ├── vendor/             three.js r128 + STLLoader + OrbitControls
    └── model/meshes/       r1_a5 STL meshes
```

See [CLAUDE.md](CLAUDE.md) for the control model, joint index maps, the
bridge command protocol and development notes.

## Third-party assets

- `web/vendor/`: [three.js](https://github.com/mrdoob/three.js) r128 (MIT).
- `r1_a5.urdf` and `web/model/meshes/`: from
  [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)
  (`robots/r1_a5_description`). That repository's license applies to them.
