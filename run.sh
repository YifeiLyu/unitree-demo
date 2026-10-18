#!/usr/bin/env bash
# Magikid R1-A dashboard launcher.
#   ./run.sh [iface] [port]
# Defaults: iface=eth10 (the robot's onboard DDS interface), port=8080.
set -e
cd "$(dirname "$0")"

IFACE="${1:-eth10}"
PORT="${2:-8080}"

BRIDGE=bridge/build/magikid_bridge
if [ ! -x "$BRIDGE" ]; then
  echo "[build] compiling bridge..."
  ( cd bridge && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null && make -j4 )
fi

echo "[run] iface=$IFACE port=$PORT"
exec python3 server.py --iface "$IFACE" --port "$PORT"
