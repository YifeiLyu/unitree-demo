#!/usr/bin/env python3
"""Integration test for server.py with tests/fake_bridge.py (runs anywhere,
no robot needed): dashboard + Quest HTTPS listeners, teleop key, and which
commands each listener is allowed to forward.

    python3 tests/test_server.py
"""
import json
import os
import shutil
import ssl
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

from tornado import gen, httpclient, ioloop, websocket

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HTTP, HTTPS = 18080, 18443


async def ws_connect(url):
    req = httpclient.HTTPRequest(url, validate_cert=False)
    conn = await websocket.websocket_connect(req)
    conn.inbox = []

    async def pump():                     # one reader per connection, forever
        while True:
            m = await conn.read_message()
            if m is None:
                return
            conn.inbox.append(json.loads(m))
    ioloop.IOLoop.current().spawn_callback(pump)
    return conn


async def collect(conn, secs):
    """Messages received during the next `secs` seconds."""
    del conn.inbox[:]
    await gen.sleep(secs)
    msgs = conn.inbox[:]
    del conn.inbox[:]
    return msgs


def logs(msgs):
    return [m["msg"] for m in msgs if m.get("type") == "log"]


async def main(tmp):
    key = open(os.path.join(tmp, "teleop_key")).read().strip()
    dash = await ws_connect("ws://127.0.0.1:%d/ws" % HTTP)
    first = await collect(dash, 0.5)
    assert any(m.get("type") == "state" for m in first), "dashboard gets state"
    print("ok  dashboard WebSocket streams state")

    for bad in ("", "?key=wrong"):
        try:
            await ws_connect("wss://127.0.0.1:%d/ws%s" % (HTTPS, bad))
            raise AssertionError("Quest WS accepted key %r" % bad)
        except httpclient.HTTPClientError as e:
            assert e.code == 403, e.code
    print("ok  Quest WebSocket refuses a missing / wrong key (403)")

    quest = await ws_connect("wss://127.0.0.1:%d/ws?key=%s" % (HTTPS, key))
    await collect(quest, 0.3)
    quest.write_message(json.dumps({"type": "arm_engage"}))
    quest.write_message(json.dumps({"type": "traj", "name": "x", "dur": [1], "q": [0] * 12}))
    got = logs(await collect(dash, 0.6))
    assert "got arm_engage" not in got and "got traj" not in got, got
    print("ok  Quest listener cannot engage or start motions")

    dash.write_message(json.dumps({"type": "arm_engage"}))
    got = logs(await collect(dash, 0.6))
    assert "got arm_engage" in got, got
    print("ok  dashboard can engage")

    q = [-0.6, 0.3, 0, -0.2, 0, -0.8, 0, 0, 0.8, 0, 0, 0]
    quest.write_message(json.dumps({"type": "teleop", "q": q, "mask": [1, 0, 0], "vel": 0.7}))
    msgs = await collect(quest, 0.6)
    assert "got teleop" in logs(msgs), logs(msgs)
    st = [m for m in msgs if m.get("type") == "state"][-1]
    assert st["teleop_live"] and st["q_cmd"] == q, st
    print("ok  Quest teleop frames reach the bridge; state shows teleop_live + q_cmd")

    quest.write_message(json.dumps({"type": "teleop_stop"}))
    assert "got teleop_stop" in logs(await collect(dash, 0.6))
    print("ok  teleop_stop forwarded")

    ctx = ssl.create_default_context()
    ctx.check_hostname, ctx.verify_mode = False, ssl.CERT_NONE
    page = urllib.request.urlopen("https://127.0.0.1:%d/" % HTTPS, context=ctx).read().decode()
    assert "quest.js" in page
    for f in ("kin.js", "teleop.js", "quest.js", "vendor/three.min.js"):
        assert urllib.request.urlopen("https://127.0.0.1:%d/%s" % (HTTPS, f), context=ctx).status == 200
    for f in ("cam/0", "api/meta"):
        try:
            urllib.request.urlopen("https://127.0.0.1:%d/%s" % (HTTPS, f), context=ctx)
            raise AssertionError("Quest listener serves /" + f)
        except urllib.error.HTTPError as e:
            assert e.code == 404, e.code
    dash_page = urllib.request.urlopen("http://127.0.0.1:%d/" % HTTP).read().decode()
    assert "kin.js" in dash_page and "app.js" in dash_page
    print("ok  HTTPS / serves quest.html and its scripts, not the cameras; HTTP / serves the dashboard")


if __name__ == "__main__":
    tmp = tempfile.mkdtemp()
    srv = subprocess.Popen(
        [sys.executable, os.path.join(ROOT, "server.py"), "--port", str(HTTP),
         "--https-port", str(HTTPS), "--no-cameras",
         "--bridge", os.path.join(ROOT, "tests", "fake_bridge.py"),
         "--cert-dir", os.path.join(tmp, "certs"),
         "--teleop-key-file", os.path.join(tmp, "teleop_key")],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    try:
        for _ in range(100):
            if os.path.exists(os.path.join(tmp, "teleop_key")):
                break
            time.sleep(0.1)
        time.sleep(1.0)
        ioloop.IOLoop.current().run_sync(lambda: main(tmp), timeout=30)
        print("all server tests passed")
    finally:
        srv.terminate()
        log = srv.communicate(timeout=5)[0]
        if os.environ.get("VERBOSE"):
            print(log)
        shutil.rmtree(tmp, ignore_errors=True)
