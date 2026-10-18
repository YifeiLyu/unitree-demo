#!/usr/bin/env bash
# Start the Magikid dashboard in the background (safe to run more than once).
cd "$(dirname "$0")"
if pgrep -f "^python3 server.py --iface" >/dev/null; then
  echo "Dashboard is already running: http://192.168.123.164:8080"
  exit 0
fi
nohup ./run.sh "$@" > /tmp/magikid_server.log 2>&1 &
sleep 5
if pgrep -f "^python3 server.py --iface" >/dev/null; then
  echo "Dashboard started: http://192.168.123.164:8080"
else
  echo "Dashboard failed to start. Log:"; tail -20 /tmp/magikid_server.log; exit 1
fi
