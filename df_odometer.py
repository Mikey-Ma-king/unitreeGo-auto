import math
import time
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, SportModeState, WirelessController
from unitree_api.msg import Request
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point

from sport_client import SportClient
import pdb
from cv_bridge import CvBridge
import cv2

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
        
        self.bridge = CvBridge()

        self.model = ort.InferenceSession('./diffphysics_inference/nav_large_v.onnx')
        # sr = 3
        # in_features = 7
        # self.model.run(None, {
        #     'img': np.zeros((1, 1, 12*sr, 16*sr), dtype=np.float32),
        #     'state': np.zeros((1, in_features), dtype=np.float32),
        #     'h': np.zeros((1, 192), dtype=np.float32),
        # })

        self.state_sub = self.create_subscription(LowState, '/lf/lowstate', self.lowstate_callback, 10)
        # self.sport_sub = self.create_subscription(SportModeState, '/lf/sportmodestate', self.sportmodestate_callback, 10)
        self.js_sub = self.create_subscription(WirelessController, '/udp_controller', self.js_callback, 10)
        # self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.v_cmd_pub = self.create_publisher(Vector3, '/planner/v_cmd', 10)
        # self.report_timer = self.create_timer(1, self.report)
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/infra1/image_rect_raw',
            self.image_callback,
            10
        )
        
        self.pos_sub = self.create_subscription(PoseStamped,
            '/svo/pose_cam_0', 
            self.pose_callback,
            10)

        self.mode = 0
        self.power_v = 0
        self.power_a = 0
        self.soc = 0
        self.position = np.zeros((3,), dtype=np.float32)
        self.velocity = np.zeros((3,), dtype=np.float32)
        self.p_target = np.zeros((3,), dtype=np.float32)
        self.target_speed = 5

        self.yaw_rate = 0
        self.vel_target = (0, 0)
        self.plan_until_t = 0
        self.h = np.zeros((1, 192), dtype=np.float32)
        self.ready = False
        
        self.first_q = True
        self.current_pos = Point()
        self.delta_xy = (0.6,0.2)
        
    
    def pose_callback(self,msg: PoseStamped):
        self.current_pos = msg.pose.position
        
    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = cv2.resize(cv_image, (320, 180), interpolation=cv2.INTER_AREA)
        self.read_depth(cv_image)
        
            
    def lowstate_callback(self, msg: LowState):
        self.power_v = msg.power_v
        self.power_a = msg.power_a
        self.soc = msg.bms_state.soc
        self.q = msg.imu_state.quaternion
        
        if self.first_q:
            self.target_pos = (self.current_pos.z + self.delta_xy[0],self.current_pos.x + self.delta_xy[1])
            self.first_q = False

    def js_callback(self, msg:WirelessController):
        if msg.keys == 256:
            #A
            self.trigger()
            
        self.yaw_rate = -msg.rx
        self.vel_target = (msg.ly, -msg.lx)
        
    def trigger(self):
        self.get_logger().info("Trigger")

        self.h = np.zeros((1, 192), dtype=np.float32)
        self.first_q = True

    def read_depth(self, depth):
        if not self.ready:
            self.get_logger().info("Ready")
            self.ready = True
            # self.beep_pub.publish(True)
        self.process(depth[None, None, 2::5,43:283:5] / np.float32(1000))

    def process(self, depth):
        env_R = quaternion_to_matrix(self.q)

        # horizontal R
        # R = env_R
        fwd = env_R[:, 0].copy()
        up = np.zeros_like(fwd)
        fwd[2] = 0
        up[2] = 1
        fwd = fwd / np.linalg.norm(fwd)
        R = np.stack([fwd, np.cross(up, fwd), up], -1)

        target_v = np.array([self.target_pos[0]-self.current_pos.z,self.target_pos[1]-self.current_pos.x,0]) 
        target_v = target_v * 1.375 / np.linalg.norm(target_v)
        v_body = target_v[None] @ R
        # v_body = target_v.reshape(1,-1)
        
        act, self.h = self.model.run(None, {'img': depth, 'state': v_body[:,:2].astype('float32'), 'h': self.h})
        # act = act * action_std + action_mean
        v_setpoint = np.concatenate([act[:,:2],np.zeros_like(act[:,2:])],-1)
        # ref_v = (R @ v_setpoint.reshape(3, -1)).reshape(-1)
        # ref_v = ref_v.tolist()
        ref_v = v_setpoint.reshape(-1).tolist()
        
        d_yaw = np.tanh(act[:,2])
        self.v_cmd_pub.publish(Vector3(x=ref_v[0], y=ref_v[1],z=d_yaw[0].item()))


def main(args=None):
    rclpy.init(args=args)
    planner = DpNavigation()
    rclpy.spin(planner)

if __name__ == '__main__':
    main()
