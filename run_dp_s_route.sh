#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROUTE_NAME="s_route"
ROS_DISTRO="${ROS_DISTRO:-foxy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

log() {
    printf '[%s] [INFO] %s\n' "$(date '+%Y-%m-%d %H:%M:%S%z')" "$*"
}

warn() {
    printf '[%s] [WARN] %s\n' "$(date '+%Y-%m-%d %H:%M:%S%z')" "$*" >&2
}

fail() {
    printf '[%s] [ERROR] %s\n' "$(date '+%Y-%m-%d %H:%M:%S%z')" "$*" >&2
    exit 1
}

log "Starting ${ROUTE_NAME} route launcher"
log "Repository: ${ROOT_DIR}"
log "ROS distribution: ${ROS_DISTRO}"
if [[ ! -f "$ROS_SETUP" ]]; then
    fail "ROS setup not found: ${ROS_SETUP}. Install that ROS 2 distribution or set ROS_DISTRO."
fi
if ! source "$ROS_SETUP"; then
    fail "Failed to load ROS environment: ${ROS_SETUP}"
fi
log "Loaded ROS environment from ${ROS_SETUP}"

if [[ -f "$ROOT_DIR/cyclonedds_ws/install/setup.bash" ]]; then
    if ! source "$ROOT_DIR/cyclonedds_ws/install/setup.bash"; then
        fail "Failed to load Unitree workspace: ${ROOT_DIR}/cyclonedds_ws/install/setup.bash"
    fi
    log "Loaded Unitree workspace: ${ROOT_DIR}/cyclonedds_ws/install/setup.bash"
elif [[ -f "$HOME/cyclonedds_ws/install/setup.bash" ]]; then
    if ! source "$HOME/cyclonedds_ws/install/setup.bash"; then
        fail "Failed to load Unitree workspace: ${HOME}/cyclonedds_ws/install/setup.bash"
    fi
    log "Loaded Unitree workspace: ${HOME}/cyclonedds_ws/install/setup.bash"
else
    fail "Unitree ROS 2 workspace setup.bash not found. Build cyclonedds_ws first."
fi

if [[ -f "$ROOT_DIR/.env/bin/activate" ]]; then
    if ! source "$ROOT_DIR/.env/bin/activate"; then
        fail "Failed to activate project virtual environment: ${ROOT_DIR}/.env"
    fi
    log "Activated project virtual environment: ${ROOT_DIR}/.env"
else
    warn "Project virtual environment not found at ${ROOT_DIR}/.env; using the current Python environment."
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    if [[ -f "$HOME/cyclonedds_ws/cyclonedds.xml" ]]; then
        export CYCLONEDDS_URI="$HOME/cyclonedds_ws/cyclonedds.xml"
    elif [[ -f "$ROOT_DIR/cyclonedds_ws/src/cyclonedds.xml" ]]; then
        export CYCLONEDDS_URI="$ROOT_DIR/cyclonedds_ws/src/cyclonedds.xml"
    fi
fi

log "ROS middleware: ${RMW_IMPLEMENTATION}"
if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    log "CycloneDDS configuration is set."
else
    warn "CYCLONEDDS_URI is not set; DDS will use its default interface selection."
fi

for required_command in python ros2 timeout grep; do
    command -v "$required_command" >/dev/null 2>&1 \
        || fail "Required command '${required_command}' was not found on PATH."
done
log "Python executable: $(command -v python)"
log "ROS 2 CLI executable: $(command -v ros2)"

MODEL_PATH="$ROOT_DIR/diffphysics_inference/nav_large_v_20d_smooth_lag.onnx"
for required_file in \
    "$ROOT_DIR/tools/udp_controller_client.py" \
    "$ROOT_DIR/dp_s_route.py" \
    "$ROOT_DIR/dp_route_navigation.py" \
    "$ROOT_DIR/sport_client.py" \
    "$MODEL_PATH"; do
    [[ -s "$required_file" ]] || fail "Required file is missing or empty: ${required_file}"
    log "Found required file: ${required_file#$ROOT_DIR/}"
done

log "Checking Python imports required by ROS, the camera, and ONNX inference..."
if ! python -c 'import numpy, onnxruntime, pyrealsense2, rclpy; from unitree_api.msg import Request; from unitree_go.msg import LowState, SportModeState, WirelessController; print("Python dependencies and Unitree ROS messages imported successfully.")'; then
    fail "Python dependency check failed. Verify the active interpreter, ROS environment, and installed packages."
fi
log "Checking RealSense device discovery..."
if ! python -c 'import pyrealsense2 as rs; devices=rs.context().query_devices(); print("RealSense devices: " + (", ".join(d.get_info(rs.camera_info.name) for d in devices) if len(devices) else "none")); raise SystemExit(0 if len(devices) else 1)'; then
    fail "No RealSense device was discovered. Check the camera USB connection, permissions, and whether another process is using it."
fi

log "Running a zero-input ONNX smoke inference before starting the remote-controller bridge..."
if ! ROUTE_MODEL_PATH="$MODEL_PATH" python -c 'import os, numpy as np, onnxruntime as ort; session=ort.InferenceSession(os.environ["ROUTE_MODEL_PATH"]); outputs=session.run(None, {"img": np.zeros((1, 1, 36, 48), dtype=np.float32), "state": np.zeros((1, 2), dtype=np.float32), "h": np.zeros((1, 192), dtype=np.float32)}); print("ONNX smoke inference passed; providers=" + ", ".join(session.get_providers()) + "; output_shapes=" + repr([getattr(value, "shape", None) for value in outputs]))'; then
    fail "ONNX model load or smoke inference failed. Check the model file, ONNX Runtime installation, and execution provider."
fi

DISCOVERY_TIMEOUT="${ROUTE_TOPIC_DISCOVERY_TIMEOUT:-20}"
[[ "$DISCOVERY_TIMEOUT" =~ ^[0-9]+$ ]] \
    || fail "ROUTE_TOPIC_DISCOVERY_TIMEOUT must be a positive integer (seconds)."
(( DISCOVERY_TIMEOUT > 0 )) \
    || fail "ROUTE_TOPIC_DISCOVERY_TIMEOUT must be greater than zero."
log "Waiting up to ${DISCOVERY_TIMEOUT}s for robot state topics to appear in the ROS graph..."
deadline=$((SECONDS + DISCOVERY_TIMEOUT))
missing_topics="/lf/lowstate /lf/sportmodestate"
while (( SECONDS < deadline )); do
    topic_output="$(timeout 4s ros2 topic list 2>&1)" || {
        warn "Could not query the ROS topic graph yet: ${topic_output//$'\n'/; }"
        sleep 1
        continue
    }
    missing_topics=""
    for required_topic in /lf/lowstate /lf/sportmodestate; do
        if ! grep -Fxq "$required_topic" <<<"$topic_output"; then
            missing_topics="${missing_topics} ${required_topic}"
        fi
    done
    if [[ -z "$missing_topics" ]]; then
        log "Robot state topics discovered: /lf/lowstate and /lf/sportmodestate"
        break
    fi
    warn "Still waiting for robot state topics:${missing_topics}. Check robot power, network, DDS interface, and ROS domain."
    sleep 1
done
[[ -z "$missing_topics" ]] \
    || fail "Robot state topics were not discovered within ${DISCOVERY_TIMEOUT}s. No controller was started."

cd "$ROOT_DIR"
log "Starting UDP remote-controller bridge. Its initialization sends one StandDown request."
python tools/udp_controller_client.py &
UDP_CLIENT_PID=$!
cleanup_started=0
cleanup() {
    local exit_status=$?
    if (( cleanup_started )); then
        return
    fi
    cleanup_started=1
    trap - EXIT INT TERM
    log "Stopping UDP controller bridge..."
    kill "$UDP_CLIENT_PID" 2>/dev/null || true
    wait "$UDP_CLIENT_PID" 2>/dev/null || true
    log "Launcher cleanup finished (exit status ${exit_status})."
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

sleep 1
if ! kill -0 "$UDP_CLIENT_PID" 2>/dev/null; then
    wait "$UDP_CLIENT_PID" || true
    fail "UDP controller bridge exited during startup. Check UDP port 37021 and the remote-controller network."
fi
log "UDP controller bridge is running (PID ${UDP_CLIENT_PID}); waiting for the route node."

log "Launching Python route node: dp_s_route.py"
if python "$ROOT_DIR/dp_s_route.py"; then
    log "Python route node exited normally."
else
    route_exit_status=$?
    warn "Python route node exited with status ${route_exit_status}."
    exit "$route_exit_status"
fi
