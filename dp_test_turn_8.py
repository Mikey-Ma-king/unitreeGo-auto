import math
import time
import os
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, SportModeState, WirelessController
from unitree_api.msg import Request
from geometry_msgs.msg import Vector3
import cv2
import threading

def imwrite_thread(filename, image):
    threading.Thread(target=cv2.imwrite, args=(filename, image, [cv2.IMWRITE_JPEG_QUALITY, 95]), daemon=True).start()

from sport_client import SportClient

SINGLE_WAYPOINT_DURATION = 12
SINGLE_WAYPOINT_DURATION_MINER = 10
TURN_TIME = 3
TURN_ON_APPROACHED = False
TARGET_SPEED = 1
FWD_DISTANCE = 5

class Keys:
    R1 = 1 << 0
    L1 = 1 << 1
    start = 1 << 2
    select = 1 << 3
    R2 = 1 << 4
    L2 = 1 << 5
    F1 = 1 << 6
    F2 = 1 << 7
    A = 1 << 8
    B = 1 << 9
    X = 1 << 10
    Y = 1 << 11
    up = 1 << 12
    right = 1 << 13
    down = 1 << 14
    left = 1 << 15

MODES = [
    "idle, default stand",
    "balanceStand",
    "pose",
    "locomotion",
    "reserve",
    "lieDown",
    "jointLock",
    "damping",
    "recoveryStand",
    "reserve",
    "sit",
    "frontFlip",
    "frontJump",
    "frontPounc",
]

def quaternion_to_matrix(q):
    return np.array([[1-2*(q[2]**2+q[3]**2), 2*(q[1]*q[2]-q[3]*q[0]), 2*(q[1]*q[3]+q[2]*q[0])],
              [2*(q[1]*q[2]+q[3]*q[0]), 1-2*(q[1]**2+q[3]**2), 2*(q[2]*q[3]-q[1]*q[0])],
              [2*(q[1]*q[3]-q[2]*q[0]), 2*(q[2]*q[3]+q[1]*q[0]), 1-2*(q[1]**2+q[2]**2)]], dtype=np.float32)


class DpNavigation(Node):

    def __init__(self):
        super().__init__('dp_navigation')
        self.extra = np.array([0.3], np.float32)

        self.model = ort.InferenceSession('./diffphysics_inference/nav_large_v_20d_smooth_lag.onnx')
        sr = 3
        in_features = 2
        self.model.run(None, {
            'img': np.zeros((1, 1, 12*sr, 16*sr), dtype=np.float32),
            'state': np.zeros((1, in_features), dtype=np.float32),
            'h': np.zeros((1, 192), dtype=np.float32),
        })

        self.state_sub = self.create_subscription(LowState, '/lf/lowstate', self.lowstate_callback, 10)
        self.sport_sub = self.create_subscription(SportModeState, '/lf/sportmodestate', self.sportmodestate_callback, 10)
        #self.js_sub = self.create_subscription(WirelessController, '/wirelesscontroller', self.js_callback, 10)
        self.js_sub = self.create_subscription(WirelessController, '/udp_controller', self.js_callback, 10)
        self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.v_cmd_pub = self.create_publisher(Vector3, '/planner/v_cmd', 10)
        self.report_timer = self.create_timer(1, self.report)

        self.mode = 0
        self.power_v = 0
        self.power_a = 0
        self.soc = 0
        self.position = np.zeros((3,), dtype=np.float32)
        self.velocity = np.zeros((3,), dtype=np.float32)

        self.plan_until_t = 0
        self.record_dir = None
        self.record_idx = 0
        self.record_csv_f = None
        self.h = np.zeros((1, 192), dtype=np.float32)
        self.ready = False
        self.robot_id = os.environ.get("ROBOT_ID", "-1")
        self.p_target_list = None

    def report(self):
        self.get_logger().info(f"Robot_id:{self.robot_id}; Mode: {MODES[self.mode]}; Batttery: {self.power_v/8:.2f}V ({self.soc:d}%) {self.power_a:.2f}A")
        self.get_logger().info(f"Robot_id:{self.robot_id}; pos: {self.position}; vel: {self.velocity}")

    def lowstate_callback(self, msg: LowState):
        self.power_v = msg.power_v
        self.power_a = msg.power_a
        self.soc = msg.bms_state.soc

    def sportmodestate_callback(self, msg: SportModeState):
        self.mode = msg.mode
        self.position = msg.position
        self.yaw_rate = msg.yaw_speed
        self.q = msg.imu_state.quaternion
        self.velocity = msg.velocity

    def js_callback(self, msg:WirelessController):
        if msg.keys & Keys.L1:
            self.stop()
        elif msg.keys & Keys.L2:
            if MODES[self.mode] == "balanceStand":
                self.trigger()
        elif msg.keys & Keys.A:
            if MODES[self.mode] == "balanceStand":
                self.trigger(back_to_start_position=True)
        elif msg.keys & Keys.R1:
            req = Request()
            SportClient.RecoveryStand(req)
            self.sport_pub.publish(req)
        elif msg.keys & Keys.R2:
            req = Request()
            SportClient.StandDown(req)
            self.sport_pub.publish(req)

    def trigger(self, back_to_start_position=False):
        if self.plan_until_t > 0:
            self.get_logger().info(f"Robot_id:{self.robot_id}, already triggered")
            return
        self.get_logger().info(f"Robot_id:{self.robot_id}, trigger")

        # req = Request()
        # SportClient.BodyHeight(req, -0.03)
        # self.sport_pub.publish(req)
        # req = Request()
        # SportClient.SpeedLevel(req, 1)
        # self.sport_pub.publish(req)
        # req = Request()
        # SportClient.SwitchGait(req, 1)
        # self.sport_pub.publish(req)

        if back_to_start_position:
            if not self.p_target_list:
                self.get_logger().info(f"Robot_id:{self.robot_id}, bad p_target_list, faile to trigger")
                return
            self.plan_until_t = time.monotonic() + SINGLE_WAYPOINT_DURATION_MINER + TURN_TIME * 2
            self.p_target_list_idx = len(self.p_target_list) - 1
            self.this_turn_start_t = time.monotonic() + TURN_TIME
            self.approaching_destination = False
            self.h = np.zeros((1, 192), dtype=np.float32)
            return

        # horizontal R
        env_R = quaternion_to_matrix(self.q)
        fwd = env_R[:, 0].copy()
        fwd[2] = 0
        fwd = fwd / np.linalg.norm(fwd)
        left = env_R[:, 1].copy()
        left[2] = 0
        left = left / np.linalg.norm(left)
        self.p_target_list = [
            self.position + fwd * FWD_DISTANCE,
            #self.position + fwd * FWD_DISTANCE / 2 - left * FWD_DISTANCE / 2,
            #self.position + fwd * FWD_DISTANCE / 2 + left * FWD_DISTANCE / 2,
            self.position,
        ]
        self.duration_list = [
            SINGLE_WAYPOINT_DURATION,
            #SINGLE_WAYPOINT_DURATION_MINER,
            SINGLE_WAYPOINT_DURATION,
            #SINGLE_WAYPOINT_DURATION_MINER,
        ]
        self.p_target_list_idx = 0
        self.this_turn_start_t = time.monotonic()

        self.plan_until_t = time.monotonic() + 120
        self.record_dir = "record/" + time.strftime("%Y%m%d_%H%M%S")
        os.makedirs(self.record_dir, exist_ok=True)
        self.record_csv_f = open(f"{self.record_dir}/v_cmd.csv", "w")
        self.record_csv_f.write(f"cmd_vx,cmd_vy,cmd_raw_rate,yaw_rate,x,y,z,w,i,j,k,vx,vy,vz\n")
        self.record_idx = 0
        self.h = np.zeros((1, 192), dtype=np.float32)
        self.approaching_destination = False

        with open(f"{self.record_dir}/p_target.csv", "w") as record_target_f:
            for p_target in self.p_target_list:
                x, y, z = p_target
                record_target_f.write(f"{x},{y},{z}\n")

    def stop(self):
        if self.plan_until_t == 0:
            return
        self.get_logger().info("Stop, switching to balanced stand")
        self.plan_until_t = 0
        req = Request()
        SportClient.StopMove(req)
        self.sport_pub.publish(req)
        if self.record_csv_f is not None:
            self.record_csv_f.close()
            self.record_csv_f = None

    def is_planning(self):
        if self.plan_until_t:
            if time.monotonic() < self.plan_until_t:
                return True
            self.stop()
        return False

    def read_color(self, color):
        if self.is_planning() and self.record_csv_f is not None:
            color = np.asanyarray(color.get_data())
            color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
            save_path = os.path.join(f"{self.record_dir}/color_{self.record_idx:06d}.jpg")
            self.record_idx += 1
            imwrite_thread(save_path, color)

    def read_depth(self, depth):
        # new_req = Request()
        # SportClient.Euler(new_req,0,-0.10,0)
        # self.sport_pub.publish(new_req)
        if not self.ready:
            self.get_logger().info("Ready")
            self.ready = True
            # self.beep_pub.publish(True)
        if self.is_planning():
            vx, vy, yaw_rate = self.process(depth[None, None, 2::5,43:283:5] / np.float32(1000))
            if self.record_csv_f is not None:
                save_path = os.path.join(f"{self.record_dir}/depth_{self.record_idx:06d}.jpg")
                depth_viz = np.uint8(np.float32(51000) / np.maximum(depth, 200))
                imwrite_thread(save_path, depth_viz)
                self.record_csv_f.write(f"{vx},{vy},{yaw_rate},{self.yaw_rate}")
                self.record_csv_f.write((",{}"*10).format(*self.position, *self.q, *self.velocity) + "\n")

    def process(self, depth):
        p = np.array(self.position, dtype=np.float32)
        v = np.array(self.velocity, dtype=np.float32)
        env_R = quaternion_to_matrix(self.q)

        # horizontal R
        # R = env_R
        fwd = env_R[:, 0].copy()
        up = np.zeros_like(fwd)
        fwd[2] = 0
        up[2] = 1
        fwd = fwd / np.linalg.norm(fwd)
        R = np.stack([fwd, np.cross(up, fwd), up], -1)

        p_target = self.p_target_list[self.p_target_list_idx]
        target_v = p_target - p
        target_v_norm = np.linalg.norm(target_v)
        target_v = target_v / target_v_norm * min(target_v_norm, TARGET_SPEED)

        state = [target_v[None] @ R, env_R[None, 2], self.extra[None]]
        state = np.concatenate(state, -1)
        state = state[:,:2]

        # state
        time_now = time.monotonic()
        turn_timeout = time_now - self.this_turn_start_t > self.duration_list[self.p_target_list_idx]
        turn_on_approached = TURN_ON_APPROACHED and np.linalg.norm(p - p_target) < 0.5
        if turn_timeout or turn_on_approached:
            # trigger turn
            self.p_target_list_idx = (self.p_target_list_idx + 1) % len(self.p_target_list)
            self.this_turn_start_t = time_now + TURN_TIME
            self.h.fill(0)
            self.approaching_destination = False
        if time_now - self.this_turn_start_t < 0:
            # turning
            target_v = p_target - p
            yaw_rate = math.atan2(state[0, 1].item(), state[0, 0].item())
            yaw_rate = max(-2, min(2, yaw_rate))
            req = Request()
            SportClient.Move(req, 0, 0, yaw_rate)
            self.sport_pub.publish(req)
            self.h.fill(0)
            return 0, 0, yaw_rate

        if not self.approaching_destination:
            angle = np.dot(state[0, :2], v[:2]) / np.linalg.norm(state[0, :2]) / np.linalg.norm(v[:2])
            self.approaching_destination = np.linalg.norm(p - p_target) < 1.5 and angle < 0.5
            self.approaching_destination |= np.linalg.norm(p - p_target) < 1.0
        if self.approaching_destination:
            x, y = (state * 0.5).flatten().tolist()
            if math.sqrt(x ** 2 + y ** 2) < 0.05:
                x, y = 0, 0
            req = Request()
            SportClient.Move(req, x, y, 0)
            self.sport_pub.publish(req)
            return x, y, 0

        # state.insert(0, v[None])
        # state = (state - states_mean) / states_std
        act, self.h = self.model.run(None, {'img': depth, 'state': state, 'h': self.h})
        # act = act * action_std + action_mean
        v_setpoint, = act.reshape(3, -1).T
        v_setpoint = v_setpoint.tolist()
        # v_setpoint, v_est = (R @ act.reshape(3, -1)).T
        # SportClient.Move(v_setpoint[0], v_setpoint[1])
        v_setpoint[1] *= 0.3
        # v_setpoint[1] = max(-0.3, min(0.3, v_setpoint[1]))
        req = Request()
        SportClient.Move(req, v_setpoint[0], v_setpoint[1], math.tanh(v_setpoint[2]))
        self.sport_pub.publish(req)
        self.v_cmd_pub.publish(Vector3(x=v_setpoint[0], y=v_setpoint[1]))
        #self.get_logger().info('Send cmd')
        return v_setpoint[0], v_setpoint[1], math.tanh(v_setpoint[2])


def main(args=None):
    rclpy.init(args=args)
    planner = DpNavigation()
    # rclpy.spin(planner)


    import pyrealsense2 as rs
    # Set up RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

    # filter
    hole_filling = rs.hole_filling_filter(1)
    decimation = rs.decimation_filter()

    # Start streaming
    pipeline.start(config)
    #from tqdm import tqdm
    #pbar = tqdm()
    try:
        to_skip_frame_number = -1
        while True:
            rclpy.spin_once(planner)
            frames = pipeline.wait_for_frames()
            planner.read_color(frames.get_color_frame())

            # downscale to 15 hz
            if frames.frame_number == to_skip_frame_number:
                continue
            to_skip_frame_number = frames.frame_number + 1
            depth_frame = frames.get_depth_frame()
            depth_frame = decimation.process(depth_frame)
            depth_frame = hole_filling.process(depth_frame)
            depth_image = np.asanyarray(depth_frame.get_data())
            planner.read_depth(depth_image)
            #pbar.update()

    finally:
        pipeline.stop()
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
