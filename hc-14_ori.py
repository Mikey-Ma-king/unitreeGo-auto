# import rclpy
# from rclpy.node import Node
# from unitree_go.msg import SportModeState
# import serial
# import time
# import struct
# import math
# import threading

# INFO_FOOT_STATE = 0  # 设置为 1 时输出脚部状态（脚部位置和速度）
# HIGH_FREQ = 0        # 设置为 1 时订阅运动状态的高频数据（500Hz）
# count = 0
# timee = time.time()
# class MotionStateSuber(Node):

#     def __init__(self):
#         super().__init__('motion_state_suber')

#         # 设置话题名称
#         topic_name = 'lf/sportmodestate'
#         if HIGH_FREQ:
#             topic_name = '/sportmodestate'

#         # 创建订阅者
#         self.suber = self.create_subscription(
#             SportModeState,
#             topic_name,
#             self.topic_callback,
#             10
#         )

#         # 设置串口连接
#         self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)  # 9600 为波特率，根据需要调整
        
#         # 创建一个线程用于监听串口
#         # self.serial_thread = threading.Thread(target=self.serial_listener, daemon=True)
#         # self.serial_thread.start()

#     def topic_callback(self, data):
#         global count
#         global timee
#         count +=1
#         # if count <= 20:
#         #     return
#         count = 0
#         # 提取位置 x, y, z
#         x = data.position[0]
#         y = data.position[1]
#         z = data.position[2]
        
#         vx = data.velocity[0]
#         vy = data.velocity[1]
#         vz = data.velocity[2]
        
#         q_w = data.imu_state.quaternion[0]
#         q_x = data.imu_state.quaternion[1]
#         q_y = data.imu_state.quaternion[2]
#         q_z = data.imu_state.quaternion[3] 
        
#         yaw = math.atan2(2.0*(q_y*q_x + q_w*q_z),1 - 2*(q_y*q_y + q_z*q_z))

#         x = int(x * 100)
#         y = int(y * 100)
#         z = int(z * 100)
#         vx = int(vx * 100)
#         vy = int(vy * 100)
#         yaw = int(yaw * 100)
#         # print("x:",x)

#         # 格式化数据为 x:(value), y:(value), z:(value)
#         formatted_data = struct.pack("6h", x, y, z, vx, vy, yaw)
#         unpacked_data = struct.unpack("6h", formatted_data)
#         # print(unpacked_data)

#         # 发送数据到串口
#         if self.ser.is_open:
#             self.ser.write(formatted_data + "@".encode())  # 发送字节数据
#             # self.ser.write()
#             # self.get_logger().info(f"Send:{formatted_data}")
#         else:
#             self.get_logger().error("串口未打开")

# # 创建一个线程用于监听串口
#         # self.serial_thread = threading.Thread(target=self.serial_listener, daemon=True)
#         # self.serial_thread.start()
        
#     def serial_listener(self):
#         """ 串口监听线程，接收数据并打印 """
#         # self.get_logger().info("串口监听线程启动...")

#         while True:
#             if self.ser.in_waiting:  # 检查是否有数据可读
#                 received_data = self.ser.read(1).decode('utf-8', errors='ignore')  # 读取 1 字节数据
#                 if received_data in ['+', '-']:
#                     print(f"收到串口数据: {received_data}")
#                 else:
#                     print(f"收到未知数据: {received_data}")
#                     time.sleep(0.01)
        
# def main(args=None):
#     rclpy.init(args=args)
    
#     motion_state_suber_node = MotionStateSuber()
    
#     # 运行节点
#     rclpy.spin(motion_state_suber_node)
    
#     # 关闭节点并关闭串口
#     motion_state_suber_node.ser.close()  # 关闭串口
#     motion_state_suber_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState
import serial
import time
import struct
import math
import threading
import numpy as np
from busservo import STservo
from nav_msgs.msg import Odometry

# port = '/dev/ttyACM0'

INFO_FOOT_STATE = 0  # 设置为 1 时输出脚部状态（脚部位置和速度）
HIGH_FREQ = 0        # 设置为 1 时订阅运动状态的高频数据（500Hz）
count = 0
timee = time.time()
class MotionStateSuber(Node):

    def __init__(self):
        super().__init__('motion_state_suber')

        # 设置话题名称
        topic_name = 'lf/sportmodestate'
        if HIGH_FREQ:
            topic_name = '/sportmodestate'

        # self.STservo = STservo(port)

        # 创建订阅者
        self.suber = self.create_subscription(
            SportModeState,
            topic_name,
            self.topic_callback,
            10
        )

        # 设置串口连接
        self.ser = serial.Serial('/dev/USB_state', 115200, timeout=0.1)  # 9600 为波特率，根据需要调整
        
        # 用于控制舵机模块
        # self.bus_ser = serial.Serial('/dev/USB_servo', 115200, timeout=0.1)
        
        # 创建一个线程用于监听串口
        # self.serial_thread = threading.Thread(target=self.serial_listener, daemon=True)
        # self.serial_thread.start()
        
        self.buffer = ""


    def topic_callback(self, data):
        global count
        global timee
        count +=1
        # if count <= 20:
        #     return
        count = 0
        # 提取位置 x, y, z
        x = data.position[0]
        y = data.position[1]
        z = data.position[2]
        
        vx = data.velocity[0]
        vy = data.velocity[1]
        vz = data.velocity[2]
        
        q_w = data.imu_state.quaternion[0]
        q_x = data.imu_state.quaternion[1]
        q_y = data.imu_state.quaternion[2]
        q_z = data.imu_state.quaternion[3] 
        
        yaw = math.atan2(2.0*(q_y*q_x + q_w*q_z),1 - 2*(q_y*q_y + q_z*q_z))

        x = int(x * 100)
        y = int(y * 100)
        z = int(z * 100)
        vx = int(vx * 100)
        vy = int(vy * 100)
        yaw = int(yaw * 100)
        
        self.target_angle = yaw
        # print("x:",x)

        # 格式化数据为 x:(value), y:(value), z:(value)
        formatted_data = struct.pack("6h", x, y, z, vx, vy, yaw)
        unpacked_data = struct.unpack("6h", formatted_data)
        # print(unpacked_data)

        # 发送数据到串口
        if self.ser.is_open:
            self.ser.write(formatted_data + "@".encode())  # 发送字节数据
            # print(formatted_data)
            
            # self.ser.write()
            # self.get_logger().info(f"Send:{formatted_data}")
        else:
            self.get_logger().error("串口未打开")

# 创建一个线程用于监听串口
        # self.serial_thread = threading.Thread(target=self.serial_listener, daemon=True)
        # self.serial_thread.start()
        
    def serial_listener(self):
        """ 串口监听线程，接收数据并打印 """
        # self.get_logger().info("串口监听线程启动...")

        while True:

            if self.bus_ser.in_waiting > 0:  # 检查是否有数据可读
                
                self.buffer += self.bus_ser.read(self.bus_ser.in_waiting).decode('utf-8')
                
                
            pos = self.buffer.find('@')
            
            if pos != -1:
                data = int(self.buffer[0:pos])
                self.buffer = self.buffer[pos+1:]
                
                try:   
                    print(f"收到串口数据: {data}")
                    # self.STservo.RotateServo(data)

                except Exception as e:
                    print(f"舵机控制错误: {e}")
                
            if self.STservo is None:
                self.get_logger().warn("舵机未初始化，跳过控制")
                return


        
def main(args=None):
    rclpy.init(args=args)
    
    motion_state_suber_node = MotionStateSuber()
    
    # 运行节点
    rclpy.spin(motion_state_suber_node)
    
    # 关闭节点并关闭串口
    motion_state_suber_node.ser.close()  # 关闭串口
    motion_state_suber_node.bus_ser.close()  # 关闭串口
    motion_state_suber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    