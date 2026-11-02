import argparse
import json
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
S_ROUTE_SPACING = 0.4
S_BEND_RADIUS = 1.0
S_TURN_RADIUS = 1.0
S_ROUTE_LENGTH = 20.0
CIRCLE_RADIUS = 10.0
CIRCLE_SPACING = 0.4
CONFIG_DIR = Path(__file__).resolve().parent / "route_configs"


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


def make_circle_route(position, q, radius=CIRCLE_RADIUS, direction="counterclockwise"):
    start = np.asarray(position[:2], dtype=np.float32)
    forward, left = horizontal_basis(q)
    direction_sign = 1 if direction == "counterclockwise" else -1
    point_count = max(16, math.ceil(2 * math.pi * radius / CIRCLE_SPACING))
    angles = 2 * math.pi * np.arange(1, point_count, dtype=np.float32) / point_count
    return np.asarray(
        [
            start + radius * math.sin(angle) * forward
            + direction_sign * radius * (1 - math.cos(angle)) * left
            for angle in angles
        ],
        dtype=np.float32,
    )


def make_s_route(
    position,
    q,
    first_turn_direction="left",
    bend_radius=S_BEND_RADIUS,
    turn_radius=S_TURN_RADIUS,
    outbound_distance=S_ROUTE_LENGTH,
):
    start = np.asarray(position[:2], dtype=np.float32)
    forward, left = horizontal_basis(q)
    first_direction = 1 if first_turn_direction == "left" else -1
    pair_distance = 2 * bend_radius
    full_pairs = int(outbound_distance // pair_distance)
    remaining_distance = outbound_distance - full_pairs * pair_distance
    if math.isclose(remaining_distance, pair_distance, rel_tol=1e-9, abs_tol=1e-9):
        full_pairs += 1
        remaining_distance = 0.0

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

    for pair_index in range(full_pairs):
        pair_direction = first_direction if pair_index % 2 == 0 else -first_direction
        append_arc(pair_direction, math.pi / 2, bend_radius)
        append_arc(-pair_direction, math.pi / 2, bend_radius)

    if remaining_distance > 1e-9:
        pair_direction = first_direction if full_pairs % 2 == 0 else -first_direction
        sweep = math.asin(remaining_distance / pair_distance)
        append_arc(pair_direction, sweep, bend_radius)
        append_arc(-pair_direction, sweep, bend_radius)

    forward_points = np.asarray(forward_points, dtype=np.float32)

    def semicircle(initial_position, initial_heading, direction):
        left_normal = np.array(
            [-math.sin(initial_heading), math.cos(initial_heading)],
            dtype=np.float32,
        )
        center = initial_position + direction * turn_radius * left_normal
        radial = initial_position - center
        start_angle = math.atan2(float(radial[1]), float(radial[0]))
        steps = max(1, math.ceil(math.pi * turn_radius / S_ROUTE_SPACING))
        angles = np.linspace(0, math.pi, steps + 1, dtype=np.float32)[1:]
        return np.asarray(
            [
                center
                + turn_radius
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

    turn_direction = first_direction
    return_offset = np.array([0, 2 * turn_direction * turn_radius], dtype=np.float32)
    far_turn = semicircle(local_position, heading, direction=turn_direction)
    return_points = forward_points[::-1] + return_offset
    near_turn = semicircle(
        return_points[-1], heading + math.pi, direction=turn_direction
    )
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


def build_argument_parser(route_name):
    parser = argparse.ArgumentParser(
        description=f"Run the configured {route_name} route."
    )
    default_config = CONFIG_DIR / f"{route_name}.json"
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config,
        help=f"JSON route configuration (default: {default_config})",
    )
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="Validate and print the effective route configuration, then exit.",
    )
    if route_name == "circle":
        parser.add_argument(
            "--direction",
            choices=("counterclockwise", "clockwise"),
            help="Circle direction, relative to the robot's initial heading.",
        )
        parser.add_argument("--radius", type=float, help="Circle radius in meters.")
        expected_keys = {"direction", "radius_m"}
    else:
        parser.add_argument(
            "--first-turn",
            choices=("left", "right"),
            help="Direction of the first S bend, relative to the initial heading.",
        )
        parser.add_argument(
            "--bend-radius", type=float, help="Radius of each S-bend arc in meters."
        )
        parser.add_argument(
            "--turnaround-radius",
            type=float,
            help="Radius of the two end turnaround semicircles in meters.",
        )
        parser.add_argument(
            "--outbound-distance",
            type=float,
            help="S-route outbound longitudinal distance in meters.",
        )
        expected_keys = {
            "first_turn_direction",
            "bend_radius_m",
            "turnaround_radius_m",
            "outbound_distance_m",
        }
    return parser, expected_keys


def load_route_config(route_name, argv=None):
    parser, expected_keys = build_argument_parser(route_name)
    args = parser.parse_args(argv)
    try:
        with args.config.open(encoding="utf-8") as config_file:
            config = json.load(config_file)
    except OSError as exc:
        parser.error(f"cannot read config '{args.config}': {exc}")
    except json.JSONDecodeError as exc:
        parser.error(
            f"invalid JSON in '{args.config}' at line {exc.lineno}, "
            f"column {exc.colno}: {exc.msg}"
        )

    if not isinstance(config, dict):
        parser.error(f"config '{args.config}' must contain a JSON object")
    unknown_keys = set(config) - expected_keys
    missing_keys = expected_keys - set(config)
    if unknown_keys:
        parser.error(f"unknown config key(s): {', '.join(sorted(unknown_keys))}")
    if missing_keys:
        parser.error(f"missing config key(s): {', '.join(sorted(missing_keys))}")

    if route_name == "circle":
        effective = {
            "direction": config["direction"],
            "radius_m": config["radius_m"],
        }
        if args.direction is not None:
            effective["direction"] = args.direction
        if args.radius is not None:
            effective["radius_m"] = args.radius
        if effective["direction"] not in ("counterclockwise", "clockwise"):
            parser.error("config 'direction' must be 'counterclockwise' or 'clockwise'")
        radius = effective["radius_m"]
        if isinstance(radius, bool) or not isinstance(radius, (int, float)):
            parser.error("circle radius_m must be a number")
        if not math.isfinite(radius) or radius <= 0:
            parser.error("circle radius_m must be a finite number greater than zero")
    else:
        effective = {
            "first_turn_direction": config["first_turn_direction"],
            "bend_radius_m": config["bend_radius_m"],
            "turnaround_radius_m": config["turnaround_radius_m"],
            "outbound_distance_m": config["outbound_distance_m"],
        }
        if args.first_turn is not None:
            effective["first_turn_direction"] = args.first_turn
        if args.bend_radius is not None:
            effective["bend_radius_m"] = args.bend_radius
        if args.turnaround_radius is not None:
            effective["turnaround_radius_m"] = args.turnaround_radius
        if args.outbound_distance is not None:
            effective["outbound_distance_m"] = args.outbound_distance
        if effective["first_turn_direction"] not in ("left", "right"):
            parser.error("config 'first_turn_direction' must be 'left' or 'right'")
        for key in ("bend_radius_m", "turnaround_radius_m", "outbound_distance_m"):
            radius = effective[key]
            if isinstance(radius, bool) or not isinstance(radius, (int, float)):
                parser.error(f"config '{key}' must be a number")
            if not math.isfinite(radius) or radius <= 0:
                parser.error(f"config '{key}' must be a finite number greater than zero")

    return args, effective


class RouteNavigation(Node):
    def __init__(self, route_name, route_config):
        super().__init__(f"dp_{route_name}_navigation")
        if route_name not in ("circle", "s_route"):
            raise ValueError(f"Unsupported route: {route_name}")
        if not MODEL_PATH.is_file():
            raise FileNotFoundError(f"Navigation model not found: {MODEL_PATH}")

        self.route_name = route_name
        self.route_config = route_config
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
        try:
            if self.route_name == "circle":
                self.route = make_circle_route(
                    self.position,
                    self.quaternion,
                    radius=self.route_config["radius_m"],
                    direction=self.route_config["direction"],
                )
            else:
                self.route = make_s_route(
                    self.position,
                    self.quaternion,
                    first_turn_direction=self.route_config["first_turn_direction"],
                    bend_radius=self.route_config["bend_radius_m"],
                    turn_radius=self.route_config["turnaround_radius_m"],
                    outbound_distance=self.route_config["outbound_distance_m"],
                )
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
            f"Started {self.route_name} route with config {self.route_config} "
            f"at {self.position[:2].tolist()}; "
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


def run_navigation(route_name, argv=None):
    args, route_config = load_route_config(route_name, argv)
    if args.check_config:
        print(
            f"Validated {route_name} route config from {args.config}: "
            f"{json.dumps(route_config, ensure_ascii=False, sort_keys=True)}"
        )
        return

    rclpy.init()
    node = RouteNavigation(route_name, route_config)
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
