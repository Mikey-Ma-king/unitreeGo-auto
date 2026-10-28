#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-foxy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ROS setup not found: $ROS_SETUP" >&2
    echo "Set ROS_DISTRO to the installed ROS 2 distribution." >&2
    exit 1
fi
source "$ROS_SETUP"

if [[ -f "$ROOT_DIR/cyclonedds_ws/install/setup.bash" ]]; then
    source "$ROOT_DIR/cyclonedds_ws/install/setup.bash"
elif [[ -f "$HOME/cyclonedds_ws/install/setup.bash" ]]; then
    source "$HOME/cyclonedds_ws/install/setup.bash"
else
    echo "Unitree ROS 2 workspace is not built. Build cyclonedds_ws first." >&2
    exit 1
fi

if [[ -f "$ROOT_DIR/.env/bin/activate" ]]; then
    source "$ROOT_DIR/.env/bin/activate"
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    if [[ -f "$HOME/cyclonedds_ws/cyclonedds.xml" ]]; then
        export CYCLONEDDS_URI="$HOME/cyclonedds_ws/cyclonedds.xml"
    elif [[ -f "$ROOT_DIR/cyclonedds_ws/src/cyclonedds.xml" ]]; then
        export CYCLONEDDS_URI="$ROOT_DIR/cyclonedds_ws/src/cyclonedds.xml"
    fi
fi

cd "$ROOT_DIR"
python tools/udp_controller_client.py &
UDP_CLIENT_PID=$!
cleanup() {
    kill "$UDP_CLIENT_PID" 2>/dev/null || true
    wait "$UDP_CLIENT_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 1
python "$ROOT_DIR/dp_s_route.py"
