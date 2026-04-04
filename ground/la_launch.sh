#!/bin/bash
# ══════════════════════════════════════════════════════════════
# Luna-Aegis Short Hopper — Ground Control Launcher
#
# Starts the WebSocket-to-UDP bridge and HTTP file server.
# Open http://<LAN_IP>:8080/index.html on any LAN device.
#
# Usage:
#   ./la_launch.sh                    # Default ports
#   ./la_launch.sh --ws-port 9765     # Custom WS port
#   ./la_launch.sh --http-port 9090   # Custom HTTP port
# ══════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONSOLE_DIR="${SCRIPT_DIR}/console"
WS_PORT=8765
HTTP_PORT=8080

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws-port)   WS_PORT="$2"; shift 2 ;;
    --http-port) HTTP_PORT="$2"; shift 2 ;;
    --help|-h)
      echo "Usage: $0 [--ws-port PORT] [--http-port PORT]"
      exit 0
      ;;
    *) shift ;;
  esac
done

# Auto-detect LAN IP
LAN_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
if [ -z "$LAN_IP" ]; then
    LAN_IP=$(ipconfig getifaddr en0 2>/dev/null || echo "127.0.0.1")
fi

echo ""
echo -e "  \033[0;36m══════════════════════════════════════════════\033[0m"
echo -e "  \033[0;36m  LUNA-AEGIS · SHORT HOPPER\033[0m"
echo -e "  \033[0;36m  Ground Control Launcher\033[0m"
echo -e "  \033[0;36m══════════════════════════════════════════════\033[0m"
echo ""
echo "  Bridge  : ws://${LAN_IP}:${WS_PORT}"
echo "  Console : http://${LAN_IP}:${HTTP_PORT}/index.html"
echo "  ──────────────────────────────────────────────"
echo ""

# Check for websockets
python3 -c "import websockets" 2>/dev/null || {
    echo "  Installing websockets..."
    pip3 install websockets --break-system-packages 2>/dev/null || \
    pip3 install websockets 2>/dev/null || {
        echo "  ERROR: Could not install websockets. Run: pip3 install websockets"
        exit 1
    }
}

# Inject the WS URL into the console HTML so it connects to the bridge
# instead of running standalone sim mode
CONSOLE_FILE="${CONSOLE_DIR}/index.html"
if [[ -f "$CONSOLE_FILE" ]]; then
    # Check if we need to patch the console for WS mode
    if ! grep -q "LA_BRIDGE_WS_URL" "$CONSOLE_FILE"; then
        echo "  Note: Console running in standalone sim mode"
        echo "  (WS bridge telemetry integration is a future enhancement)"
    fi
fi

# Kill any stale processes on our ports from a previous run
for port in "$WS_PORT" "$HTTP_PORT"; do
  pids=$(lsof -ti:"$port" 2>/dev/null || true)
  if [[ -n "$pids" ]]; then
    echo "  Killing stale process on port $port (PID $pids)"
    kill $pids 2>/dev/null || true
  fi
done
sleep 0.5

# Start WebSocket bridge in background
cd "$SCRIPT_DIR"
python3 la_bridge.py --ws-port "$WS_PORT" &
BRIDGE_PID=$!
echo "  [+] Bridge started (PID $BRIDGE_PID)"

# Give bridge a moment to bind
sleep 1

# Start HTTP file server serving the console directory
# Bind to 0.0.0.0 so it's accessible on LAN
python3 -m http.server "$HTTP_PORT" --directory "$CONSOLE_DIR" --bind 0.0.0.0 &
HTTP_PID=$!
echo "  [+] HTTP server started (PID $HTTP_PID)"

echo ""
echo "  Ready. Open on any LAN device:"
echo "  http://${LAN_IP}:${HTTP_PORT}/index.html"
echo ""
echo "  Press Ctrl+C to shut down everything."
echo ""

# On Ctrl+C, kill both
cleanup() {
    echo ""
    echo "  Shutting down..."
    kill $BRIDGE_PID $HTTP_PID 2>/dev/null || true
    wait $BRIDGE_PID $HTTP_PID 2>/dev/null || true
    echo "  Ground control terminated."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Wait for both
wait
