#!/usr/bin/env python3
"""
Magikid Lab - Unitree R1-A web teleop / telemetry server.

Ties together three things and serves them on one HTTP port:

  1. The C++ ``magikid_bridge`` subprocess (DDS <-> JSON over stdin/stdout).
     Its state stream is broadcast to every connected browser over a WebSocket,
     and browser commands are forwarded to its stdin.
  2. Two onboard cameras, streamed as MJPEG via OpenCV (cv2 + V4L2).
  3. The static single-page dashboard in ``web/``.
  4. Optionally a second, HTTPS listener (default :8443) for the Meta Quest
     teleop page ``web/quest.html``. WebXR needs a secure context, so it uses a
     self-signed certificate generated on first start. That listener requires a
     key (``?key=`` from the ``teleop_key`` file) and may only send streamed
     teleop targets: engaging, motions and release stay on the dashboard.

Runs on the robot's Orin (Ubuntu 20.04, Python 3.8, tornado, cv2). No internet
or extra pip packages required.

SAFETY: this server never engages the arms on its own. Arm commands only flow
when the browser explicitly sends them, and the bridge stays inert (weight 0)
until an explicit engage. See magikid_bridge.cpp.
"""

import hmac
import json
import os
import secrets
import ssl
import subprocess
import threading
import time
import argparse

import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.gen
import tornado.iostream

try:
    import cv2
    HAVE_CV2 = True
except Exception as e:  # pragma: no cover
    HAVE_CV2 = False
    print("[warn] cv2 unavailable, cameras disabled:", e)

HERE = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(HERE, "web")

# Commands each listener may forward to the bridge.
DASHBOARD_CMDS = ("arm_engage", "arm_release", "estop", "arm_weight", "arm_speed",
                  "arm_set", "loco", "traj", "traj_stop", "teleop_stop")
QUEST_CMDS = ("teleop", "teleop_stop")


# ---------------------------------------------------------------------------
# Bridge process manager
# ---------------------------------------------------------------------------
class Bridge:
    """Owns the magikid_bridge subprocess and fans its output out to clients."""

    def __init__(self, binary, iface, rate):
        self.binary = binary
        self.iface = iface
        self.rate = rate
        self.proc = None
        self.latest_state = None
        self.ioloop = None
        self.clients = set()          # set of WSHandler
        self._stdout_thread = None
        self._lock = threading.Lock()

    def start(self, ioloop):
        self.ioloop = ioloop
        cmd = [self.binary, self.iface, str(self.rate)]
        print("[bridge] launching:", " ".join(cmd))
        self.proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, bufsize=1, universal_newlines=True)
        self._stdout_thread = threading.Thread(target=self._read_stdout, daemon=True)
        self._stdout_thread.start()

    def _read_stdout(self):
        for line in self.proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except ValueError:
                # bridge diagnostics that aren't JSON - surface on the console
                print("[bridge]", line)
                continue
            if obj.get("type") == "state":
                self.latest_state = obj
            # broadcast on the IOLoop thread (thread-safe)
            self.ioloop.add_callback(self.broadcast, line)
        print("[bridge] stdout closed (process exited)")

    def broadcast(self, line):
        dead = []
        for c in list(self.clients):
            try:
                c.write_message(line)
            except Exception:
                dead.append(c)
        for c in dead:
            self.clients.discard(c)

    def send_command(self, obj):
        """Forward a command dict to the bridge stdin as one JSON line."""
        if not self.proc or self.proc.poll() is not None:
            return False
        try:
            self.proc.stdin.write(json.dumps(obj) + "\n")
            self.proc.stdin.flush()
            return True
        except Exception as e:
            print("[bridge] command write failed:", e)
            return False

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.terminate()
            except Exception:
                pass


BRIDGE = None  # set in main()


# ---------------------------------------------------------------------------
# Camera capture
# ---------------------------------------------------------------------------
class CameraStream:
    """Background V4L2 capture loop that keeps the latest JPEG in memory."""

    def __init__(self, index, dev, label="Camera", kind="cam", width=640, height=480, fps=15, quality=70):
        self.index = index
        self.dev = dev
        self.label = label
        self.kind = kind          # "head" | "gripper" | "cam"
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.jpeg = None
        self.frame_id = 0
        self.ok = False
        self._lock = threading.Lock()
        self._run = True

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _placeholder(self, text):
        import numpy as np
        img = np.zeros((self.height, self.width, 3), dtype="uint8")
        img[:] = (28, 30, 38)
        cv2.putText(img, text, (24, self.height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 130, 150), 2)
        ok, buf = cv2.imencode(".jpg", img)
        return buf.tobytes() if ok else None

    def _loop(self):
        if not HAVE_CV2:
            return
        cap = None
        backoff = 1.0
        enc = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        while self._run:
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    cap.set(cv2.CAP_PROP_FPS, self.fps)
                    self.ok = True
                    backoff = 1.0
                else:
                    self.ok = False
                    with self._lock:
                        self.jpeg = self._placeholder("camera %d: no signal" % self.index)
                        self.frame_id += 1
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 5.0)
                    continue
            grabbed, frame = cap.read()
            if not grabbed:
                self.ok = False
                cap.release()
                cap = None
                time.sleep(0.3)
                continue
            ok, buf = cv2.imencode(".jpg", frame, enc)
            if ok:
                with self._lock:
                    self.jpeg = buf.tobytes()
                    self.frame_id += 1
            time.sleep(1.0 / max(1, self.fps))

    def get(self):
        with self._lock:
            return self.jpeg, self.frame_id


CAMERAS = {}  # index -> CameraStream


def classify(name):
    """Map a v4l2 by-id name to (label, kind)."""
    low = name.lower()
    if "realsense" in low or "intel" in low:
        return "Head (RealSense)", "head"
    if "jr0001" in low:
        return "Gripper A (JR0001)", "gripper"
    if "jr0002" in low:
        return "Gripper B (JR0002)", "gripper"
    if "jr0" in low:
        return "Gripper (%s)" % name.split("_")[0][-6:], "gripper"
    base = name.split("-video")[0].replace("usb-", "")[:18]
    return base or "Camera", "cam"


def scan_cameras():
    """Auto-detect capture-capable video devices, newest topology first.

    Prefers the /dev/v4l/by-id/*-video-index0 symlinks (the actual capture
    nodes; index1 is metadata), which also carry the camera's identity, so a
    RealSense head camera is picked up and labelled the moment it is plugged in.
    Falls back to probing raw /dev/video* with OpenCV.
    """
    found = []  # (dev, label, kind)
    byid = "/dev/v4l/by-id"
    if os.path.isdir(byid):
        for name in sorted(os.listdir(byid)):
            if "index0" not in name:      # capture node only
                continue
            dev = os.path.realpath(os.path.join(byid, name))
            label, kind = classify(name)
            found.append((dev, label, kind))
    if not found and HAVE_CV2:            # fallback probe
        for n in range(0, 8):
            dev = "/dev/video%d" % n
            if not os.path.exists(dev):
                continue
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            ok = False
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                ok = cap.read()[0]
            cap.release()
            if ok:
                found.append((dev, "Cam %d" % n, "cam"))
    order = {"head": 0, "gripper": 1, "cam": 2}
    found.sort(key=lambda c: order.get(c[2], 3))
    return found


# ---------------------------------------------------------------------------
# HTTP / WS handlers
# ---------------------------------------------------------------------------
class StateWSHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True  # LAN-only tool on a direct link

    def prepare(self):
        # The Quest listener is reachable from Wi-Fi through the Mac's tunnel,
        # so it needs the teleop key before the WebSocket upgrade.
        key = self.settings.get("teleop_key")
        if key and not hmac.compare_digest(self.get_argument("key", ""), key):
            raise tornado.web.HTTPError(403)

    def open(self):
        BRIDGE.clients.add(self)
        if BRIDGE.latest_state is not None:
            try:
                self.write_message(json.dumps(BRIDGE.latest_state))
            except Exception:
                pass

    def on_message(self, message):
        try:
            obj = json.loads(message)
        except ValueError:
            return
        # Only forward known command types to the bridge.
        if obj.get("type") in self.settings.get("commands", DASHBOARD_CMDS):
            BRIDGE.send_command(obj)

    def on_close(self):
        BRIDGE.clients.discard(self)


class MjpegHandler(tornado.web.RequestHandler):
    async def get(self, index):
        index = int(index)
        cam = CAMERAS.get(index)
        if cam is None:
            self.set_status(404)
            self.finish("no such camera")
            return
        self.set_header("Content-Type",
                        "multipart/x-mixed-replace; boundary=frame")
        self.set_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.set_header("Pragma", "no-cache")
        last_id = -1
        try:
            while True:
                jpeg, fid = cam.get()
                if jpeg is not None and fid != last_id:
                    last_id = fid
                    self.write(b"--frame\r\n")
                    self.write(b"Content-Type: image/jpeg\r\n")
                    self.write(("Content-Length: %d\r\n\r\n" % len(jpeg)).encode())
                    self.write(jpeg)
                    self.write(b"\r\n")
                    await self.flush()
                await tornado.gen.sleep(0.03)
        except (tornado.iostream.StreamClosedError, tornado.web.Finish):
            return
        except Exception:
            return


class MetaHandler(tornado.web.RequestHandler):
    """Static metadata (joint layout, camera list) for the frontend."""
    def get(self):
        self.set_header("Content-Type", "application/json")
        self.write({
            "robot": "Unitree R1-A",
            "lab": "Magikid",
            "cameras": [{"index": i, "label": c.label, "kind": c.kind, "dev": c.dev}
                        for i, c in sorted(CAMERAS.items())],
        })


class IndexHandler(tornado.web.RequestHandler):
    def initialize(self, page="index.html"):
        self.page = page

    def get(self):
        self.render_from_file(self.page)

    def render_from_file(self, name):
        with open(os.path.join(WEB_DIR, name), "rb") as f:
            self.set_header("Content-Type", "text/html; charset=utf-8")
            self.write(f.read())


def make_app(page="index.html", cameras=True, **settings):
    routes = [(r"/", IndexHandler, {"page": page}), (r"/ws", StateWSHandler)]
    if cameras:  # the Quest listener is on Wi-Fi with no key on these: leave them off
        routes += [(r"/cam/(\d+)", MjpegHandler), (r"/api/meta", MetaHandler)]
    routes.append((r"/(.*)", tornado.web.StaticFileHandler, {"path": WEB_DIR}))
    return tornado.web.Application(routes, **settings)


def ensure_cert(cert_dir):
    """Self-signed TLS cert for the Quest listener (WebXR needs HTTPS)."""
    cert = os.path.join(cert_dir, "cert.pem")
    key = os.path.join(cert_dir, "key.pem")
    if not (os.path.exists(cert) and os.path.exists(key)):
        os.makedirs(cert_dir, exist_ok=True)
        print("[https] generating self-signed certificate in", cert_dir)
        subprocess.check_call(
            ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
             "-days", "3650", "-subj", "/CN=magikid-r1a",
             "-keyout", key, "-out", cert],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return cert, key


def load_teleop_key(path):
    """Read the Quest teleop key, creating a random one on first start."""
    if os.path.exists(path):
        with open(path) as f:
            key = f.read().strip()
        if key:
            return key
    key = "".join(secrets.choice("abcdefghjkmnpqrstuvwxyz23456789") for _ in range(8))
    with open(path, "w") as f:
        f.write(key + "\n")
    os.chmod(path, 0o600)
    return key


def main():
    global BRIDGE
    ap = argparse.ArgumentParser(description="Magikid R1-A teleop server")
    ap.add_argument("--iface", default="eth10", help="DDS network interface")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--rate", type=int, default=50, help="state stream Hz")
    ap.add_argument("--bridge", default=os.path.join(HERE, "bridge/build/magikid_bridge"))
    ap.add_argument("--cameras", default="auto",
                    help="'auto' to detect, or a comma list of index:/dev/videoN[:label]")
    ap.add_argument("--no-cameras", action="store_true")
    ap.add_argument("--https-port", type=int, default=8443,
                    help="Quest teleop HTTPS port (0 disables)")
    ap.add_argument("--https-address", default="0.0.0.0")
    ap.add_argument("--cert-dir", default=os.path.join(HERE, "certs"))
    ap.add_argument("--teleop-key-file", default=os.path.join(HERE, "teleop_key"))
    args = ap.parse_args()

    ioloop = tornado.ioloop.IOLoop.current()

    BRIDGE = Bridge(args.bridge, args.iface, args.rate)
    BRIDGE.start(ioloop)

    if not args.no_cameras and HAVE_CV2:
        specs = []  # (dev, label, kind)
        if args.cameras == "auto":
            specs = scan_cameras()
        else:
            for spec in args.cameras.split(","):
                spec = spec.strip()
                if not spec:
                    continue
                parts = spec.split(":")
                dev = parts[1] if len(parts) > 1 else parts[0]
                label = parts[2] if len(parts) > 2 else os.path.basename(dev)
                specs.append((dev, label, "cam"))
        for idx, (dev, label, kind) in enumerate(specs):
            cam = CameraStream(idx, dev, label=label, kind=kind)
            cam.start()
            CAMERAS[idx] = cam
            print("[cam] %d: %-22s %-8s %s" % (idx, label, kind, dev))
        if not specs:
            print("[cam] no capture devices detected")

    app = make_app()
    app.listen(args.port, address="0.0.0.0")
    quest_url = None
    if args.https_port:
        try:
            cert, key = ensure_cert(args.cert_dir)
            ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
            ctx.load_cert_chain(cert, key)
            teleop_key = load_teleop_key(args.teleop_key_file)
            quest = make_app(page="quest.html", cameras=False,
                             teleop_key=teleop_key, commands=QUEST_CMDS)
            quest.listen(args.https_port, address=args.https_address, ssl_options=ctx)
            quest_url = "https://<host>:%d/?key=%s" % (args.https_port, teleop_key)
        except Exception as e:
            print("[https] Quest listener disabled:", e)
    print("=" * 60)
    print(" Magikid R1-A dashboard:  http://<robot-ip>:%d/" % args.port)
    if quest_url:
        print(" Quest teleop page:       %s" % quest_url)
        print("   (from the Mac: scripts/quest_tunnel.sh prints the exact URL)")
    print(" DDS interface: %s   state rate: %d Hz" % (args.iface, args.rate))
    print("=" * 60)
    try:
        ioloop.start()
    except KeyboardInterrupt:
        print("\nshutting down...")
    finally:
        BRIDGE.stop()


if __name__ == "__main__":
    main()
