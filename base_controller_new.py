from enum import Enum
import json
import os
import math
import time
import numpy as np
import onnxruntime as ort

import rclpy
import rclpy.serialization
from rclpy.node import Node
from unitree_go.msg import LowState, SportModeState, WirelessController
from unitree_go.msg import LowCmd, MotorCmd
from unitree_api.msg import Request
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from go2_cpp_utils import LowControl

class States(Enum):
    STAND = 1
    DAMPING = 2
    ALGO = 3

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


class BaseController(Node):
    dry_run = False

    def __init__(self) -> None:
        super().__init__('base_controller')
        self.low_state = LowState()
        self.odom = Odometry()
        self.joystick = WirelessController()
        self.pressed = 0
        self.pd_ratio = 0
        self.state = States.DAMPING
        self.low_control = LowControl()
        self.traj_q = np.loadtxt('traj_q.txt')
        self.max_traj_len = len(self.traj_q)
        self.prev_state = None
        self.traj_ptr = 0

        # low command
        self.default_dof_pos = np.array([
            0.05, 0.8, -1.5,
            -0.05, 0.8, -1.5,
            0.05, 1.0, -1.5,
            -0.05, 1.0, -1.5,
        ], dtype=np.float32)
        self.idle_pos = np.array((
            0.05, 0.8, -1.5,
            -0.05, 0.8, -1.5,
            0.05, 1.0, -1.5,
            -0.05, 1.0, -1.5,
        ), dtype=np.float32)
        
        self.act_v = Vector3(x=0.,y=0.,z=0.)

        self.req_pub = self.create_publisher(Request, '/api/robot_state/request', 10)
        self.state_sub = self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)
        self.js_sub = self.create_subscription(WirelessController, '/udp_controller', self.js_callback, 10)
        self.v_cmd_pub = self.create_subscription(Vector3, '/planner/v_cmd',self.v_callback, 10)
        self.ctrl_loop = self.create_timer(1 / 100, self.control_step)
        self.log_loop = self.create_timer(2, self.log)
        self.v_flag = False
        self.auto = True
        self.v_cmd_sub_timeout = 1.0  # 设置超时时间（单位：秒）
        self.last_v_cmd_time = self.get_clock().now()  # 初始化最后一次接收消息的时间
        # 设置定时器来检查是否超时
        self.timeout_timer = self.create_timer(1 / 100, self.check_timeout)
        self.model = ort.InferenceSession('./diffphysics_inference/new_two_ckpt_mix.onnx')
        self.yaw_rate = 0
        self.vel_target = (0, 0)
        self.reset()
        self.algo()

        # def sport_mode_off(self):
        time.sleep(0.2)
        req = Request()
        req.header.identity.id = 4001001
        req.header.identity.api_id = 1001
        req.parameter = """{"name":"sport_mode","switch":0}"""
        self.req_pub.publish(req)
        self.kp = np.array([30,30,30,30,30,30,30,30,30,30,30,30], dtype=np.float32)
        self.dog_control = 0  # 0->all, 1-4 ->
        
        self.robot_id = os.environ.get("ROBOT_ID", "-1")
        self.operator = True
        
        
    def v_callback(self,msg: Vector3):
        self.act_v = msg
        self.v_flag = True  # 每次接收到消息时，设置标志为True
        self.last_v_cmd_time = self.get_clock().now()  # 更新最后一次接收消息的时间
        
    def check_timeout(self):
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.last_v_cmd_time
        if elapsed_time.nanoseconds / 1e9 > self.v_cmd_sub_timeout and self.v_flag:
            self.v_flag = False  # 如果超过了设定的超时时间，设置标志为False
            print("Timeout: No /planner/v_cmd message received for over", self.v_cmd_sub_timeout, "seconds.")

    def lowstate_callback(self, msg: LowState):
        self.low_state = msg

    def get_on_press(self):
        on_press = self.joystick.keys & (self.joystick.keys ^ self.pressed)
        self.pressed = self.joystick.keys
        return on_press

    def js_callback(self, msg: WirelessController):
        self.joystick = msg
        self.yaw_rate = -msg.rx
        self.vel_target = (msg.ly, -msg.lx)

    def log(self):
        self.get_logger().info(f"Robot {self.robot_id} {self.state.name},Traj q: {self.traj_ptr}, pd_ratio: {self.pd_ratio} ({self.low_state.bms_state.soc:d}%) {self.low_state.power_a:.2f}A, v_flag: {self.v_flag}, auto: {self.auto}, operator: {self.dog_control}")

    def control_step(self):
        on_press = self.get_on_press()
        if on_press & Keys.L1:
            self.dog_control = 0
            self.auto = True
            
        if on_press & Keys.select:
            self.dog_control = (self.dog_control + 1) % 5
            
        if self.dog_control != 0 and self.dog_control != int(self.robot_id):
            #此时应该保持为dampling
            if self.state == States.ALGO:
                self.pd_ratio = 1
                self.traj_ptr = self.max_traj_len - 1
                self.state = States.STAND
                self.prev_state = States.ALGO
            if self.state == States.STAND and self.pd_ratio == 1:
                self.prev_state = States.ALGO
        else:
            if on_press & Keys.R2:
                if self.state == States.DAMPING:
                    self.pd_ratio = 0
                    self.traj_ptr = 0
                    self.state = States.STAND
                    self.prev_state = States.DAMPING
                    
            elif on_press & Keys.R1:
                if self.state == States.ALGO:
                    self.pd_ratio = 1
                    self.traj_ptr = self.max_traj_len - 1
                    self.state = States.STAND
                    self.prev_state = States.ALGO
                if self.state == States.STAND and self.pd_ratio == 1:
                    self.prev_state = States.ALGO
                    
            elif self.joystick.keys & Keys.L2 and on_press & Keys.B:
                self.state = States.DAMPING
                self.prev_state = None
                self.pd_ratio = 0
                self.traj_ptr = 0
            
        if on_press & Keys.L2:
            self.auto = not self.auto
            
        if on_press & Keys.A:
            if self.state == States.STAND and self.pd_ratio > 0.5:
                self.state = States.ALGO
                self.reset()
        
        if self.state == States.STAND:
            if self.prev_state == States.DAMPING:
                self.traj_ptr = min(self.max_traj_len - 1, self.traj_ptr + 5)
            if self.prev_state == States.ALGO:
                self.traj_ptr = max(0, self.traj_ptr - 3)

            self.low_control.publish(
                    self.traj_q[self.traj_ptr],
                    np.array([70 for i in range(12)]),
                    0.5
                )
            if self.pd_ratio == 0 and self.traj_ptr == self.max_traj_len - 1:
                self.pd_ratio = 1
            if self.pd_ratio == 1 and self.traj_ptr == 0:
                self.pd_ratio = 0
                self.state = States.DAMPING

        elif self.state == States.DAMPING:
            self.low_control.publish(self.traj_q[0], np.full_like(self.kp, 3.), 2)

        elif self.state == States.ALGO:
                self.low_control.publish(
                    self.algo(),
                    self.kp,
                    0.5
                )

    def reset(self):
        self._current_phase = 0
        self._freq = 1.8
        self._start_time = time.monotonic()

    def get_obs(self):
        phase = (time.monotonic() - self._start_time) * 2 * math.pi * self._freq
        gyro = self.low_state.imu_state.gyroscope
        rpy = self.low_state.imu_state.rpy
        motor_positions = [
            s.q - d
            for s, d in zip(self.low_state.motor_state, self.default_dof_pos)
        ]
        motor_qd = [s.dq * 0.1 for s in self.low_state.motor_state[:12]]
        
        if self.v_flag and self.auto:
            obs = np.array([
                math.sin(phase),
                math.cos(phase),
                self.act_v.x,self.act_v.y, 0.,  # ref vel body
                self.act_v.z,  # yaw diff
                0, 0, 0,
                *gyro.tolist(),
                *rpy[:2].tolist(),
                *motor_positions,
                *motor_qd
            ], dtype=np.float32)
        else:
            obs = np.array([
                math.sin(phase),
                math.cos(phase),
                *self.vel_target, 0.,  # ref vel body
                self.yaw_rate,  # yaw diff
                0, 0, 0,
                *gyro.tolist(),
                *rpy[:2].tolist(),
                *motor_positions,
                *motor_qd
            ], dtype=np.float32)
        return obs

    def algo(self):
        obs = self.get_obs()[None]
        act, = self.model.run(None, {'observation': obs})
        return act[0] + self.default_dof_pos


def main(args=None):
    rclpy.init(args=args)
    planner = BaseController()
    rclpy.spin(planner)


if __name__ == '__main__':
    main()
























































