# Magikid · Unitree R1-A Web Console

A browser dashboard for the Magikid lab's **Unitree R1-A** (upper-body
humanoid, URDF `r1_a5`). It shows the live arm pose on a 3D model, telemetry
for every joint, the body IMU and the onboard cameras. It also has a
**safety-gated** panel for commanding both arms over low-level `rt/lowcmd`,
scripted motions (go home, wave), and **Meta Quest 3S teleop** over WebXR.

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
rsync -av --exclude .git --exclude bridge/build --exclude certs --exclude teleop_key \
  ./ unitree@192.168.123.164:~/magikid_teleop/

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

## Meta Quest 3S teleop

Drive both arms by hand from a Quest, in passthrough so you can see the robot.

1. Put the Quest on the **same Wi-Fi as the Mac** that is wired to the robot.
2. Start the dashboard on the robot (`./start.sh`). On the dashboard, switch
   ARM CONTROL on and press **Engage**. The headset can't engage the arms by itself.
3. On the Mac, run `scripts/quest_tunnel.sh`. It prints a URL like
   `https://192.168.1.218:8443/?key=…`.
4. In the Quest Browser, open that URL. On the first visit, accept the
   self-signed certificate (Advanced → Proceed). Pick the hand scale and speed,
   then tap **Enter passthrough**.

| Control | Action |
|---|---|
| Hold **grip** | That robot arm follows your hand, relative to where you grabbed. Twist to roll the wrist. Let go and the arm holds. |
| Hold **trigger** on the status panel or the dark YOU / ROBOT board | Drag it somewhere else. Push the thumbstick forward or back to move it farther or closer. |
| **A / X** | Switch arms: both → left only → right only. You can also pick this in the page settings. |
| **B / Y** (tap) | FREEZE both arms in place. Release both grips and grab again to resume. |
| Hold **B / Y** for 1 s | **E-STOP**, same as the dashboard: the robot lets go of the arms, so be ready to support them. Re-engage on the dashboard. |
| Thumbstick **click** | Re-centre "forward" to where you're looking. |

A dark board on your front-left shows **YOU**, a figure that copies your head
and hands, next to **ROBOT**: gray is where the robot is, green is where it
will move, and a ball marks where your hand is asking it to go. A white line
means the robot can't reach that far, and a red ball means the pose is
blocked. Before you engage, the grips only move the green target ("PREVIEW"
on the panel), so you can practise without the robot moving. Stand behind
the robot, facing the same way it faces. The page refuses
poses that would hit the robot's body, its head, the other arm or the base,
and the controller buzzes when that happens. If the headset or Wi-Fi drops,
the bridge holds the arms after 0.3 s. The dashboard E-STOP still works.

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
7. **Quest teleop** uses its own HTTPS port. That port needs a key and can
   only stream arm targets, freeze, or E-STOP. Engaging and motions stay on
   the dashboard.

Verified on hardware: arm control on 2026-10-02, motions on 2026-10-04. Always
keep the workspace clear and have someone at the E-STOP.

## Files

```
├── run.sh / start.sh       launcher (builds bridge if needed) / background starter
├── server.py               tornado: static + WebSocket + MJPEG + bridge process (+ HTTPS for the Quest)
├── scripts/quest_tunnel.sh Mac-side Wi-Fi relay for the Quest
├── tests/                  Node + Python tests that run without the robot
├── r1_a5.urdf              joint origins, axes, limits (source of truth)
├── bridge/
│   ├── magikid_bridge.cpp  DDS <-> JSON bridge and arm control (unitree_sdk2)
│   └── CMakeLists.txt
└── web/
    ├── index.html, style.css
    ├── kin.js              shared URDF kinematics, arm IK, self-collision check
    ├── app.js              dashboard logic, motions
    ├── quest.html, quest.js, teleop.js   Meta Quest WebXR teleop page
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
