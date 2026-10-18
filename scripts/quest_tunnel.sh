#!/usr/bin/env bash
# Run on the Mac that is wired to the robot (the robot has no Wi-Fi).
# Relays the robot's Quest teleop port (HTTPS 8443) onto this Mac's Wi-Fi, so a
# Meta Quest on the same Wi-Fi network can open it, and prints the URL to type
# into the Quest Browser. Ctrl-C stops the relay.
#   scripts/quest_tunnel.sh [port]
set -e
ROBOT="${ROBOT:-unitree@192.168.123.164}"
PORT="${1:-8443}"

IFACE=$(route -n get default 2>/dev/null | awk '/interface:/{print $2}')
IP=$(ipconfig getifaddr "$IFACE" 2>/dev/null || true)
KEY=$(ssh -o ConnectTimeout=5 "$ROBOT" 'cat ~/magikid_teleop/teleop_key 2>/dev/null' || true)
if [ -z "$KEY" ]; then
  echo "No teleop key on the robot yet. Start the dashboard there first (./start.sh)." >&2
  exit 1
fi
if [ -z "$IP" ]; then
  echo "Could not find this Mac's Wi-Fi address (default route: ${IFACE:-none})." >&2
  exit 1
fi
echo "Quest Browser URL:   https://$IP:$PORT/?key=$KEY"
echo "First visit: the certificate is self-signed, so tap Advanced -> Proceed."
echo "Relaying $IP:$PORT -> robot 127.0.0.1:$PORT  (Ctrl-C to stop)"
exec ssh -N -g -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 \
  -L "$PORT:127.0.0.1:$PORT" "$ROBOT"
