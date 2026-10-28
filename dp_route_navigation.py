import math
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort
import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from unitree_api.msg import Request
from unitree_go.msg import LowState, SportModeState, WirelessController

from sport_client import SportClient


MODEL_PATH = Path(__file__).resolve().parent / "diffphysics_inference" / "nav_large_v_20d_smooth_lag.onnx"
TARGET_SPEED = 0.5
MAX_FORWARD_SPEED = 0.5
MAX_SIDE_SPEED = 0.3
MAX_YAW_SPEED = 0.7
LOOKAHEAD_POINTS = 3
WAYPOINT_REACHED_DISTANCE = 0.55
STATE_TIMEOUT = 1.0
S_ROUTE_LENGTH = 20.0
S_ROUTE_SPACING = 0.4
S_BEND_RADIUS = 1.0
S_TURN_RADIUS = 1.0
S_BEND_PAIRS = 10
CIRCLE_RADIUS = 10.0
CIRCLE_SPACING = 0.4


class Keys:
    R1 = 1 << 0
    L1 = 1 << 1
    R2 = 1 << 4
    L2 = 1 << 5


def quaternion_to_matrix(q):
    q = np.asarray(q, dtype=np.float32)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("Invalid IMU quaternion")
    norm = np.linalg.norm(q)
    if norm < 1e-6:
        raise ValueError("IMU quaternion has zero magnitude")
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def horizontal_basis(q):
    rotation = quaternion_to_matrix(q)
    forward = rotation[:, 0].copy()
    forward[2] = 0
    norm = np.linalg.norm(forward)
    if norm < 1e-6:
        raise ValueError("Robot forward direction is vertical")
    forward /= norm
    up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    left = np.cross(up, forward)
    return forward[:2], left[:2]


def make_circle_route(position, q):
    start = np.asarray(position[:2], dtype=np.float32)
    forward, left = horizontal_basis(q)
    point_count = max(16, math.ceil(2 * math.pi * CIRCLE_RADIUS / CIRCLE_SPACING))
    angles = 2 * math.pi * np.arange(1, point_count, dtype=np.float32) / point_count
    return np.asarray(
        [
            start
            + CIRCLE_RADIUS * math.sin(angle) * forward
            + CIRCLE_RADIUS * (1 - math.cos(angle)) * left
            for angle in angles
        ],
        dtype=np.float32,
    )


def make_s_route(position, q):
    start = np.asarray(position[:2], dtype=np.float32)
    forward, left = horizontal_basis(q)
    if not math.isclose(S_ROUTE_LENGTH, 2 * S_BEND_RADIUS * S_BEND_PAIRS):
        raise ValueError("S-route length must equal two bend radii per bend pair")

    forward_points = [np.zeros(2, dtype=np.float32)]
    local_position = np.zeros(2, dtype=np.float32)
    heading = 0.0

    def append_arc(direction, sweep, radius):
        nonlocal local_position, heading
        left_normal = np.array([-math.sin(heading), math.cos(heading)])
        center = local_position + direction * radius * left_normal
        radial = local_position - center
        start_angle = math.atan2(float(radial[1]), float(radial[0]))
        steps = max(1, math.ceil(radius * sweep / S_ROUTE_SPACING))
        angles = np.linspace(0, sweep, steps + 1, dtype=np.float32)[1:]
        points = np.asarray(
            [
                center
                + radius
                * np.array(
                    [
                        math.cos(start_angle + direction * angle),
                        math.sin(start_angle + direction * angle),
                    ],
                    dtype=np.float32,
                )
                for angle in angles
            ],
            dtype=np.float32,
        )
        forward_points.extend(points)
        local_position = points[-1].copy()
        heading += direction * sweep

    for pair_index in range(S_BEND_PAIRS):
        first_direction = 1 if pair_index % 2 == 0 else -1
        append_arc(first_direction, math.pi / 2, S_BEND_RADIUS)
        append_arc(-first_direction, math.pi / 2, S_BEND_RADIUS)

    forward_points = np.asarray(forward_points, dtype=np.float32)

    def semicircle(initial_position, initial_heading, direction):
        left_normal = np.array(
            [-math.sin(initial_heading), math.cos(initial_heading)],
            dtype=np.float32,
        )
        center = initial_position + direction * S_TURN_RADIUS * left_normal
        radial = initial_position - center
        start_angle = math.atan2(float(radial[1]), float(radial[0]))
        steps = max(1, math.ceil(math.pi * S_TURN_RADIUS / S_ROUTE_SPACING))
        angles = np.linspace(0, math.pi, steps + 1, dtype=np.float32)[1:]
        return np.asarray(
            [
                center
                + S_TURN_RADIUS
                * np.array(
                    [
                        math.cos(start_angle + direction * angle),
                        math.sin(start_angle + direction * angle),
                    ],
                    dtype=np.float32,
                )
                for angle in angles
            ],
            dtype=np.float32,
        )

    return_offset = np.array([0, 2 * S_TURN_RADIUS], dtype=np.float32)
    far_turn = semicircle(local_position, heading, direction=1)
    return_points = forward_points[::-1] + return_offset
    near_turn = semicircle(return_points[-1], heading + math.pi, direction=1)
    route_local = np.concatenate(
        (forward_points[1:], far_turn, return_points[1:], near_turn),
        axis=0,
    )
    return np.asarray(
        start[None, :]
        + route_local[:, 0, None] * forward[None, :]
        + route_local[:, 1, None] * left[None, :],
        dtype=np.float32,
    )


class RouteNavigation(Node):
    def __init__(self, route_name):
        super().__init__(f"dp_{route_name}_navigation")
        if route_name not in ("circle", "s_route"):
            raise ValueError(f"Unsupported route: {route_name}")
        if not MODEL_PATH.is_file():
            raise FileNotFoundError(f"Navigation model not found: {MODEL_PATH}")

        self.route_name = route_name
        self.model = ort.InferenceSession(str(MODEL_PATH))
        self.hidden = np.zeros((1, 192), dtype=np.float32)
        self.model.run(
            None,
            {
                "img": np.zeros((1, 1, 36, 48), dtype=np.float32),
                "state": np.zeros((1, 2), dtype=np.float32),
                "h": self.hidden,
            },
        )

        self.state_sub = self.create_subscription(
            SportModeState, "/lf/sportmodestate", self.sport_state_callback, 10
        )
        self.low_state_sub = self.create_subscription(
            LowState, "/lf/lowstate", self.low_state_callback, 10
        )
        self.js_sub = self.create_subscription(
            WirelessController, "/udp_controller", self.joystick_callback, 10
        )
        self.sport_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.v_cmd_pub = self.create_publisher(Vector3, "/planner/v_cmd", 10)
        self.report_timer = self.create_timer(2.0, self.report)
        self.watchdog_timer = self.create_timer(0.1, self.check_state_watchdog)

        self.position = None
        self.quaternion = None
        self.mode = None
        self.battery_soc = 0
        self.low_state_time = 0.0
        self.sport_state_time = 0.0
        self.depth_time = 0.0
        self.active = False
        self.route = None
        self.route_index = 0
        self.previous_keys = 0

    def sport_state_callback(self, msg):
        position = np.asarray(msg.position, dtype=np.float32)
        quaternion = np.asarray(msg.imu_state.quaternion, dtype=np.float32)
        if position.shape != (3,) or not np.isfinite(position).all():
            self.get_logger().error("Ignoring invalid sport position")
            return
        try:
            quaternion_to_matrix(quaternion)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.position = position
        self.quaternion = quaternion
        self.mode = int(msg.mode)
        self.sport_state_time = time.monotonic()

    def low_state_callback(self, msg):
        self.battery_soc = int(msg.bms_state.soc)
        self.low_state_time = time.monotonic()

    def joystick_callback(self, msg):
        keys = int(msg.keys)
        pressed = keys & (keys ^ self.previous_keys)
        self.previous_keys = keys

        if pressed & Keys.L1:
            self.stop("L1 stop")
        elif pressed & Keys.L2:
            if self.active:
                self.get_logger().warning("Route is already running")
            elif self.mode == 1:
                self.start_route()
            else:
                self.get_logger().warning(
                    "Stand the robot in balanceStand mode before pressing L2"
                )
        elif pressed & Keys.R1:
            if self.active:
                self.stop("recovery requested")
            req = Request()
            SportClient.RecoveryStand(req)
            self.sport_pub.publish(req)
        elif pressed & Keys.R2:
            if self.active:
                self.stop("stand-down requested")
            req = Request()
            SportClient.StandDown(req)
            self.sport_pub.publish(req)

    def start_route(self):
        if not self.state_is_fresh():
            self.get_logger().error("Cannot start: waiting for fresh robot state")
            return
        route_builder = make_circle_route if self.route_name == "circle" else make_s_route
        try:
            self.route = route_builder(self.position, self.quaternion)
        except ValueError as exc:
            self.get_logger().error(f"Cannot create route: {exc}")
            return
        if len(self.route) == 0 or not np.isfinite(self.route).all():
            self.get_logger().error("Cannot start: generated route is invalid")
            self.route = None
            return

        self.route_index = 0
        self.hidden.fill(0)
        self.active = True
        self.get_logger().info(
            f"Started {self.route_name} route at {self.position[:2].tolist()}; "
            "press L1 to stop"
        )

    def stop(self, reason):
        if self.active:
            self.get_logger().warning(f"Stopping route: {reason}")
        self.active = False
        req = Request()
        SportClient.StopMove(req)
        self.sport_pub.publish(req)

    def state_is_fresh(self):
        now = time.monotonic()
        return (
            self.position is not None
            and self.quaternion is not None
            and self.mode is not None
            and now - self.sport_state_time <= STATE_TIMEOUT
            and now - self.low_state_time <= STATE_TIMEOUT
        )

    def check_state_watchdog(self):
        if not self.active:
            return
        if not self.state_is_fresh():
            self.stop("robot state timed out")
        elif self.depth_time and time.monotonic() - self.depth_time > STATE_TIMEOUT:
            self.stop("depth camera timed out")

    def report(self):
        route_state = "running" if self.active else "stopped"
        self.get_logger().info(
            f"Route: {self.route_name} ({route_state}); "
            f"robot mode: {self.mode}; battery: {self.battery_soc}%"
        )

    def advance_route(self):
        for _ in range(len(self.route)):
            target = self.route[self.route_index]
            if np.linalg.norm(target - self.position[:2]) >= WAYPOINT_REACHED_DISTANCE:
                break
            self.route_index = (self.route_index + 1) % len(self.route)

        lookahead_index = (self.route_index + LOOKAHEAD_POINTS) % len(self.route)
        return self.route[lookahead_index]

    def send_move(self, vx, vy, vyaw):
        req = Request()
        SportClient.Move(req, vx, vy, vyaw)
        self.sport_pub.publish(req)
        self.v_cmd_pub.publish(Vector3(x=vx, y=vy, z=vyaw))

    def process_depth(self, depth_image):
        self.depth_time = time.monotonic()
        if not self.active:
            return
        if not self.state_is_fresh():
            self.stop("robot state timed out")
            return
        if depth_image.ndim != 2 or depth_image.size == 0:
            self.stop("invalid depth image")
            return

        target = self.advance_route()
        world_delta = np.zeros(3, dtype=np.float32)
        world_delta[:2] = target - self.position[:2]
        distance = float(np.linalg.norm(world_delta))
        if distance < 1e-6:
            self.stop("route target is too close")
            return
        world_delta *= min(distance, TARGET_SPEED) / distance

        rotation = quaternion_to_matrix(self.quaternion)
        forward = rotation[:, 0].copy()
        forward[2] = 0
        forward /= np.linalg.norm(forward)
        up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        left = np.cross(up, forward)
        body_delta = np.array(
            [np.dot(world_delta, forward), np.dot(world_delta, left)],
            dtype=np.float32,
        )
        depth = depth_image[None, None, 2::5, 43:283:5].astype(np.float32)
        depth /= np.float32(1000)

        action, self.hidden = self.model.run(
            None,
            {
                "img": depth,
                "state": body_delta[None],
                "h": self.hidden,
            },
        )
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.size < 3 or not np.isfinite(action[:3]).all():
            self.stop("navigation model returned an invalid action")
            return

        min_forward_speed = 0.0 if self.route_name == "s_route" else -MAX_FORWARD_SPEED
        vx = float(np.clip(action[0], min_forward_speed, MAX_FORWARD_SPEED))
        vy = float(np.clip(action[1] * 0.3, -MAX_SIDE_SPEED, MAX_SIDE_SPEED))
        vyaw = float(np.clip(math.tanh(float(action[2])), -MAX_YAW_SPEED, MAX_YAW_SPEED))
        self.send_move(vx, vy, vyaw)


def run_navigation(route_name):
    rclpy.init()
    node = RouteNavigation(route_name)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
    started = False

    try:
        pipeline.start(config)
        started = True
        hole_filling = rs.hole_filling_filter(1)
        decimation = rs.decimation_filter()
        node.get_logger().info(
            "Ready. Use L2 to start when the robot is in balanceStand; "
            "use L1 to stop, R1 to recover, R2 to stand down."
        )

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if frames.frame_number % 2:
                continue
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                if node.active:
                    node.stop("depth frame unavailable")
                continue
            depth_frame = decimation.process(depth_frame)
            depth_frame = hole_filling.process(depth_frame)
            node.process_depth(np.asanyarray(depth_frame.get_data()))
    finally:
        if node.active:
            node.stop("navigation process exiting")
        if started:
            pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()
