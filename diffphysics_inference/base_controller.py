from enum import Enum
import json
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

        self.state_sub = self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)
        self.js_sub = self.create_subscription(WirelessController, '/wirelesscontroller', self.js_callback, 10)
        self.ctrl_loop = self.create_timer(1 / 60, self.control_step)
        self.log_loop = self.create_timer(1/5, self.log)

        self.model = ort.InferenceSession('policy_240602_1535.onnx')
        self.yaw_rate = 0
        self.vel_target = (0, 0)
        self.reset()
        self.algo()

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
        print(f"State: {self.state.name}, pd_ratio: {self.pd_ratio} ({self.low_state.bms_state.soc:d}%) {self.low_state.power_a:.2f}A")

    def control_step(self):
        on_press = self.get_on_press()
        if on_press & Keys.R2:
            if self.state == States.DAMPING:
                self.pd_ratio = 0
                self.state = States.STAND
            if self.state == States.ALGO:
                self.pd_ratio = 0.5
                self.state = States.STAND
        elif self.joystick.keys & Keys.L2 and on_press & Keys.B:
            self.state = States.DAMPING
            self.pd_ratio = 0
        if on_press & Keys.A:
            if self.state == States.STAND and self.pd_ratio > 0.5:
                self.state = States.ALGO
                self.reset()

        if self.state == States.STAND:
            if self.pressed & Keys.R2:
                self.pd_ratio = min(1., self.pd_ratio + 0.01)
            if self.pressed & Keys.R1:
                self.pd_ratio = max(0., self.pd_ratio - 0.005)
            self.low_control.publish(
                self.idle_pos,
                100 * self.pd_ratio,
                1 * self.pd_ratio
            )
        elif self.state == States.DAMPING:
            self.low_control.publish(self.idle_pos, 0, 2)
        elif self.state == States.ALGO:
                self.low_control.publish(
                    self.algo(),
                    30,
                    0.5
                )

    def reset(self):
        self._current_phase = 0
        self._freq = 2.0
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
