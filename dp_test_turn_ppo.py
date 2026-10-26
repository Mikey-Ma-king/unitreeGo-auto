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

        self.model = ort.InferenceSession('./diffphysics_inference/nav_large_v_20d_smooth_lag_ppo.onnx')
        sr = 3
        in_features = 8
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
        self.p_target = np.zeros((3,), dtype=np.float32)
        self.target_speed = 0.5

        self.plan_until_t = 0
        self.record_dir = None
        self.record_idx = 0
        self.record_csv_f = None
        self.h = np.zeros((1, 192), dtype=np.float32)
        self.ready = False
        self.robot_id = os.environ.get("ROBOT_ID", "-1")

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
        if msg.keys & (1 << 1):
            self.stop()
        elif msg.keys & (1 << 5):
            if MODES[self.mode] == "balanceStand":
                self.trigger()
        elif msg.keys & (1 << 0):  #R1
            req = Request()
            SportClient.RecoveryStand(req)
            self.sport_pub.publish(req)
            #req = Request()
            #SportClient.StopMove(req)
            #self.sport_pub.publish(req)
        elif msg.keys & (1 << 4):  #R2
            req = Request()
            SportClient.StandDown(req)
            self.sport_pub.publish(req)

    def trigger(self):
        self.get_logger().info(f"Robot_id:{self.robot_id},trigger")
        #if self.plan_until_t > 0:
        #    return

        # req = Request()
        # SportClient.BodyHeight(req, -0.03)
        # self.sport_pub.publish(req)
        # req = Request()
        # SportClient.SpeedLevel(req, 1)
        # self.sport_pub.publish(req)
        # req = Request()
        # SportClient.SwitchGait(req, 1)
        # self.sport_pub.publish(req)
        self.get_logger().info("Trigger")
        # horizontal R
        env_R = quaternion_to_matrix(self.q)
        fwd = env_R[:, 0].copy()
        fwd = fwd / np.linalg.norm(fwd)
        self.p_target = self.position + fwd * 1
        self.p_init = self.position - fwd * 1
        self.this_turn_start_t = time.monotonic()

        self.plan_until_t = time.monotonic() + 30
        self.record_dir = "record/" + time.strftime("%Y%m%d_%H%M%S")
        os.makedirs(self.record_dir, exist_ok=True)
        self.record_csv_f = open(f"{self.record_dir}/v_cmd.csv", "w")
        self.record_csv_f.write(f"cmd_vx,cmd_vy,cmd_raw_rate,yaw_rate,x,y,z,w,i,j,k,vx,vy,vz\n")
        self.record_idx = 0
        self.h = np.zeros((1, 192), dtype=np.float32)

    def stop(self):
        if self.plan_until_t == 0:
            return
        self.get_logger().info("Stop, switing to balanced stand")
        self.plan_until_t = 0
        req = Request()
        SportClient.StopMove(req)
        self.sport_pub.publish(req)
        self.record_csv_f.close()

    def is_planning(self):
        if self.plan_until_t:
            if time.monotonic() < self.plan_until_t:
                return True
            self.stop()
        return False

    def read_color(self, color):
        if self.is_planning():
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
            vx, vy, raw_rate = self.process(depth[None, None, 2::5,43:283:5] / np.float32(1000))

            save_path = os.path.join(f"{self.record_dir}/depth_{self.record_idx:06d}.jpg")
            depth_viz = np.uint8(np.float32(51000) / np.maximum(depth, 200))
            imwrite_thread(save_path, depth_viz)
            self.record_csv_f.write(f"{vx},{vy},{raw_rate},{self.yaw_rate}")
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

        target_v = self.p_target - p
        target_v_norm = np.linalg.norm(target_v)
        target_v = target_v / target_v_norm * min(target_v_norm, self.target_speed)

        state = [(target_v[None] @ R)[:, :2], v[None, :], v[None, :]]
        state = np.concatenate(state, -1)
        # state = state[:,:2]
        self.get_logger().info(f"target_v: {state[0]}")

        # state
        time_now = time.monotonic()
        if time_now - self.this_turn_start_t > 2:
            # trigger turn
            self.get_logger().info("trigger turn")
            self.p_target, self.p_init = self.p_init, self.p_target
            self.this_turn_start_t = time_now + 5
            self.h = np.zeros_like(self.h)
        if time_now - self.this_turn_start_t < 0:
            # turning
            target_v = self.p_target - p
            yaw_rate = math.atan2(state[0, 1].item(), state[0, 0].item())
            req = Request()
            SportClient.Move(req, 0, 0, -yaw_rate)
            self.sport_pub.publish(req)
            self.get_logger().info(f"turning with yaw_rate {yaw_rate}")
            return 0, 0, yaw_rate
        # if np.linalg.norm(target_v) < 1.5:
        #     self.get_logger().info("stand")
        #     req = Request()
        #     SportClient.Move(req, 0, 0, 0)
        #     self.sport_pub.publish(req)
        #     return 0, 0, 0

        # state.insert(0, v[None])
        # state = (state - states_mean) / states_std
        act, self.h = self.model.run(None, {'img': depth, 'state': state, 'h': self.h})
        # act = act * action_std + action_mean
        v_setpoint, = act.reshape(3, -1).T
        v_setpoint = v_setpoint.tolist()
        # v_setpoint, v_est = (R @ act.reshape(3, -1)).T
        # SportClient.Move(v_setpoint[0], v_setpoint[1])
        v_setpoint[1] *= 0.3
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
