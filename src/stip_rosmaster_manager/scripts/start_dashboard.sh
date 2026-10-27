#!/usr/bin/env bash
# STIP ROSMASTER Dashboard: HTTP server independent from ROS process health.
set -uo pipefail
WS="${STIP_WS:-$HOME/Documents/stip_rosmaster_ws}"
HOST="${STIP_DASHBOARD_BIND:-0.0.0.0}"
PORT="${STIP_DASHBOARD_PORT:-8765}"
# Dashboard listens on all IPv4 interfaces by default.
# Optional authentication: export STIP_DASHBOARD_TOKEN before launching.
# Use only on a trusted Wi-Fi/hotspot; unauthenticated clients can operate the robot.
ENGINE="${STIP_NAV_BACKEND:-stip}"
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash || echo 'WARNING: ROS environment setup failed' >&2
fi
if [ -f "$WS/devel/setup.bash" ]; then
    source "$WS/devel/setup.bash" || echo 'WARNING: STIP devel setup failed; web interface continues' >&2
else
    echo 'WARNING: Catkin devel/setup.bash not found; web interface runs with ROS offline' >&2
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DASH_DIR="$SCRIPT_DIR/../dashboard"
if [ ! -f "$DASH_DIR/server.py" ]; then
    echo "ERROR: dashboard/server.py missing: $DASH_DIR" >&2
    exit 1
fi
# Restart HTTP daemon on unexpected failure without restarting the ROS robot.
# Ctrl+C or SIGTERM exits the supervisor and closes owned ROS processes.
stop_requested=0
server_pid=""
manager_pid=""
# ROS is optional for HTTP availability. Auto-start the existing headless
# stage manager once; never launch a duplicate if an external manager owns it.
if [ "${STIP_AUTO_START_SYSTEM:-1}" = "1" ]; then
    if command -v rosnode >/dev/null 2>&1 && timeout 4 rosnode list 2>/dev/null | grep -qx '/stip_stage_manager'; then
        echo 'STIP stage manager already running; dashboard attaching.'
    elif command -v roslaunch >/dev/null 2>&1; then
        mkdir -p "${HOME}/.ros/stip_dashboard"
        roslaunch stip_rosmaster_manager dashboard_system.launch \
          >>"${HOME}/.ros/stip_dashboard/stage_manager.log" 2>&1 &
        manager_pid=$!
        echo "STIP managed ROS bringup requested (pid ${manager_pid}); web remains independent."
    else
        echo 'WARNING: ROS launch unavailable; dashboard HTTP server will continue.' >&2
    fi
fi
stop_dashboard() {
    stop_requested=1
    if [ -n "$server_pid" ]; then
        kill -TERM "$server_pid" 2>/dev/null || true
    fi
    # Stop only the stage manager started by THIS invocation.
    if [ -n "$manager_pid" ]; then
        kill -INT "$manager_pid" 2>/dev/null || true
    fi
}
trap stop_dashboard INT TERM
while [ "$stop_requested" -eq 0 ]; do
    /usr/bin/python3 -u "$DASH_DIR/server.py" --host "$HOST" --port "$PORT" --engine "$ENGINE" &
    server_pid=$!
    wait "$server_pid" || rc=$?
    server_pid=""
    if [ "$stop_requested" -ne 0 ]; then break; fi
    echo "WARNING: STIP Dashboard server exited unexpectedly (code ${rc:-0}); retrying in 3s" >&2
    rc=0
    sleep 3 || true
done

