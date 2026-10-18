#!/usr/bin/env python3
"""Stand-in for bridge/build/magikid_bridge used by test_server.py.

Speaks the same stdin/stdout JSON-lines protocol (no DDS, no robot): streams a
state object at 20 Hz and logs every command it receives as "got <type>", so a
test can see exactly which commands server.py forwarded.
"""
import json
import sys
import threading
import time

HOME = [-0.8, 0, 0, 0.8, 0, -0.8, 0, 0, 0.8, 0, 0, 0]
state = {"engaged": False, "teleop": False, "q_cmd": HOME[:]}
lock = threading.Lock()


def out(obj):
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


def reader():
    for line in sys.stdin:
        try:
            cmd = json.loads(line)
        except ValueError:
            continue
        t = cmd.get("type")
        with lock:
            if t == "arm_engage":
                state["engaged"] = True
            elif t in ("arm_release", "estop"):
                state["engaged"] = state["teleop"] = False
            elif t == "teleop" and state["engaged"] and any(cmd.get("mask", [])):
                state["teleop"] = True
                state["q_cmd"] = cmd["q"]
            elif t == "teleop_stop":
                state["teleop"] = False
        out({"type": "log", "msg": "got " + str(t)})


threading.Thread(target=reader, daemon=True).start()
out({"type": "log", "msg": "fake bridge up"})
while True:
    q = [0.0] * 26
    with lock:
        for k, i in enumerate(range(14, 26)):
            q[i] = state["q_cmd"][k]
        s = {"type": "state", "have_state": True, "crc_ok": True, "seq": 0, "mode_machine": 3,
             "arm_engaged": state["engaged"], "arm_speed": 0.3, "traj_active": False,
             "traj_name": "", "traj_prog": 0, "teleop_live": state["teleop"],
             "q_cmd": state["q_cmd"], "head_yaw_enabled": False,
             "q": q, "dq": [0] * 26, "tau": [0] * 26, "temp": [30] * 26, "vol": [0] * 26,
             "mstate": [0] * 26, "imu_rpy": [0, 0, 0], "imu_gyro": [0, 0, 0],
             "imu_acc": [0, 0, 9.8], "imu_quat": [1, 0, 0, 0], "have_torso": False,
             "torso_rpy": [0, 0, 0]}
    out(s)
    time.sleep(0.05)
