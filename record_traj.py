import numpy as np
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, WirelessController

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
class TrajRecorder(Node):
    def __init__(self,):
        super().__init__('traj_recorder_node')
        self.joystick = WirelessController()
        self.pressed = 0


        self.sub = self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)
        self.js_sub = self.create_subscription(WirelessController, '/wirelesscontroller', self.js_callback, 10)

        self.trigger_loop = self.create_timer(1 / 100, self.trigger_func)

        self.traj_q = []
        self.traj_dq = []
        self.trigger = False

    def js_callback(self, msg: WirelessController):
        self.joystick = msg
        self.yaw_rate = -msg.rx
        self.vel_target = (msg.ly, -msg.lx)

    def get_on_press(self):
        on_press = self.joystick.keys & (self.joystick.keys ^ self.pressed)
        self.pressed = self.joystick.keys
        return on_press
        
    def lowstate_callback(self, msg: LowState):
        q = [s.q for s in msg.motor_state[:12]]
        dq = [s.dq for s in msg.motor_state[:12]]
        if self.trigger:  
            self.traj_q.append(q)
            self.traj_dq.append(dq)
        
        
    def trigger_func(self,):
        on_press = self.get_on_press()
        if self.joystick.keys & Keys.L2 and on_press & Keys.Y:
            self.trigger = not self.trigger
            print(f"Trigger state: {self.trigger}")
        
        if (not self.trigger) and (len(self.traj_q) > 10):
            np.savetxt('traj_q.txt', np.c_[np.array(self.traj_q)], delimiter='\t')
            np.savetxt('traj_dq.txt', np.c_[np.array(self.traj_dq)], delimiter='\t')
            self.get_logger().info('writing into txt...')

            #
            self.destroy_node()
            rclpy.shutdown()
            exit()
        
def main(args=None):
    rclpy.init(args=args)
    rec = TrajRecorder()
    rclpy.spin(rec)

if __name__ == '__main__':
    main()