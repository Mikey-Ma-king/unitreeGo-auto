import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState  # 需要确保你已经安装并且能够访问 unitree_go 消息类型

INFO_FOOT_STATE = 0  # 设置为 1 时输出脚部状态（脚部位置和速度）
HIGH_FREQ = 0        # 设置为 1 时订阅运动状态的高频数据（500Hz）
count = 0

class MotionStateSuber(Node):

    def __init__(self):
        super().__init__('motion_state_suber')
        
        # 设置话题名称，如果需要高频率订阅，则使用 sportmodestate，否则使用 lf/sportmodestate
        topic_name = 'lf/sportmodestate'
        if HIGH_FREQ:
            topic_name = 'sportmodestate'
        
        
        # 创建订阅者
        self.suber = self.create_subscription(
            SportModeState,
            topic_name,
            self.topic_callback,
            10
        )

    def topic_callback(self, data):
        # 输出运动状态信息
        global count
        count += 1
        if count <= 7:
            return
        # self.get_logger().info(f"Gait state -- gait type: {data.gait_type}; raise height: {data.foot_raise_height}")
        self.get_logger().info(f"x: {data.position[0]:.{3}}; y: {data.position[1]:.{3}}; z: {data.position[2]:.{3}}; body height: {data.body_height:.{3}}")
        self.get_logger().info(f"vx: {data.velocity[0]:.{3}}; vy: {data.velocity[1]:.{3}}; vz: {data.velocity[2]:.{3}}; yaw: {data.yaw_speed:.{3}}")
        count = 0
        
        if INFO_FOOT_STATE:
            # 输出脚部状态信息（脚部位置和速度）
            foot_pos = data.foot_position_body
            foot_vel = data.foot_speed_body
            for i in range(4):
                self.get_logger().info(f"Foot position and velocity relative to body -- num: {i}; "
                                        f"x: {foot_pos[i*3]}; y: {foot_pos[i*3+1]}; z: {foot_pos[i*3+2]}; "
                                        f"vx: {foot_vel[i*3]}; vy: {foot_vel[i*3+1]}; vz: {foot_vel[i*3+2]}")
                
def main(args=None):
    rclpy.init(args=args)
    
    motion_state_suber_node = MotionStateSuber()
    
    # 运行节点
    rclpy.spin(motion_state_suber_node)
    
    # 关闭节点
    motion_state_suber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()