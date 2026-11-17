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



INFO_FOOT_STATE = 0  # 设置为 1 时输出脚部状态（脚部位置和速度）
HIGH_FREQ = 0        # 设置为 1 时订阅运动状态的高频数据（500Hz）
count = 0
timee = time.time()


import serial

def send_to_plc(data, port='/dev/ttyUSB0', baudrate=9600):
    """
    发送数据到PLC的最简函数
    参数:
      data: 要发送的数据，支持字符串或十六进制字符串
      port: 串口号，默认COM3
      baudrate: 波特率，默认9600
    """
    try:
        # 打开串口
        with serial.Serial(port, baudrate, timeout=1) as ser:
            # 数据转换
            if isinstance(data, str):
                if all(c in '0123456789abcdefABCDEF ' for c in data):
                    # 十六进制
                    data = bytes.fromhex(data.replace(' ', ''))
                else:
                    # 普通字符串
                    data = data.encode('ascii')
            
            # 发送
            ser.write(data)
            print(f"已发送: {data.hex().upper() if isinstance(data, bytes) else data}")
            return True
            
    except Exception as e:
        print(f"发送失败: {e}")
        return False


class MotionStateSuber(Node):

    def __init__(self):
        super().__init__('motion_state_suber')

        # 设置话题名称
        topic_name = 'lf/sportmodestate'
        if HIGH_FREQ:
            topic_name = '/sportmodestate'
            
        try:
            self.STservo = STservo('/dev/USB_servo')
        except:
            self.get_logger().warn("舵机初始化失败")
            self.STservo = None
            
        # 创建订阅者
        self.suber = self.create_subscription(
            SportModeState,
            topic_name,
            self.topic_callback,
            10
        )

        # 设置串口连接 - 先清理再初始化
        self.init_serial_ports()
        
        # 创建串口监听线程
        self.serial_thread = threading.Thread(target=self.serial_listener, daemon=True)
        self.serial_thread.start()
        
        self.buffer = ""

        # 线程停止事件，用于优雅退出串口监听线程
        self.stop_event = threading.Event()
        
        # 存储charge flag
        self.charge_flag = None

        # 初始化标志位
        self.initialization_complete = True
        
    def init_serial_ports(self):
        """初始化串口连接，先清理再重新初始化"""
        import subprocess
        
        # # 清理串口设备
        # try:
        #     # 重置串口设备
        #     subprocess.run(['sudo', 'usbreset'], capture_output=True)
        #     self.get_logger().info("重置USB设备")
        # except Exception as e:
        #     self.get_logger().warn(f"USB重置失败: {e}")
        
        # 等待设备稳定
        time.sleep(0.5)
        
        # 初始化数据发送串口
        try:
            self.send_ser = serial.Serial('/dev/USB_send', 115200, timeout=0.1)
            self.receive_ser = serial.Serial('/dev/USB_receive', 115200, timeout=0.1)
            self.get_logger().info("数据发送串口初始化成功")
        except Exception as e:
            self.get_logger().error(f"数据发送/接收串口初始化失败: {e}")
            self.send_ser = None
            self.receive_ser = None
        

    def topic_callback(self, data):
        # 检查初始化是否完成
        if not hasattr(self, 'initialization_complete') or not self.initialization_complete:
            return
        
        global count
        global timee

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

        # 发送数据到串口
        if self.send_ser.is_open:
            self.send_ser.write(formatted_data + "@".encode())  # 发送字节数据
        else:
            self.get_logger().error("串口未打开")
        
    def serial_listener(self):
        """ 串口监听线程，接收数据并打印 """

        while True:
            # 检查初始化是否完成
            if not hasattr(self, 'initialization_complete') or not self.initialization_complete:
                time.sleep(0.1)  # 等待初始化完成
                continue

            if self.receive_ser and self.receive_ser.in_waiting > 0:  # 检查是否有数据可读
                
                self.buffer += self.receive_ser.read(self.receive_ser.in_waiting).decode('utf-8')
                
                
            pos = self.buffer.find('@')
            
            if pos != -1:
                # 解析数据：格式为 "角度;charge_flag"
                data_str = self.buffer[0:pos]
                self.buffer = self.buffer[pos+1:]
                
                    # 用分号分割数据
                parts = data_str.split(';')
                if len(parts) == 2:
                    servo_angle = int(parts[0])  # 舵机角度
                    self.charge_flag = int(parts[1])   # charge flag
                else:
                    self.get_logger().warn(f"数据格式错误，期望 '角度;charge_flag'，收到: {data_str}")
                    
                try:
                    # 控制舵机
                    if not hasattr(self, 'STservo') or self.STservo is None:
                        self.get_logger().warn("舵机未初始化，跳过控制")
                        continue
                    
                    print(f"收到串口数据: 角度={servo_angle}, charge_flag={self.charge_flag}")
                    self.STservo.RotateServo(servo_angle)
                except ValueError as e:
                    self.get_logger().error(f"数据解析错误: {e}, 数据: {data_str}")
                except Exception as e:
                    print(f"舵机控制错误: {e}")
                    
                try:
                    if self.charge_flag == 1:
                        send_to_plc("0001")
                    elif self.charge_flag == 2:
                        send_to_plc("0002")
                except ValueError as e:
                    print(f"起降平台错误: {e}")
                         
                
            if self.stop_event.is_set():
                break
            time.sleep(0.05)

        
def main(args=None):
    rclpy.init(args=args)
    
    motion_state_suber_node = MotionStateSuber()
    
    try:
        # 运行节点
        rclpy.spin(motion_state_suber_node)
    except KeyboardInterrupt:
        motion_state_suber_node.get_logger().info("程序被用户中断")
    except Exception as e:
        motion_state_suber_node.get_logger().error(f"程序运行错误: {e}")
    finally:
        # 通知串口监听线程退出并等待其结束
        try:
            if hasattr(motion_state_suber_node, 'stop_event'):
                motion_state_suber_node.stop_event.set()
            if hasattr(motion_state_suber_node, 'serial_thread') and motion_state_suber_node.serial_thread.is_alive():
                motion_state_suber_node.serial_thread.join(timeout=1.0)
        except Exception as e:
            motion_state_suber_node.get_logger().error(f"停止串口监听线程时出错: {e}")

        # # 清理串口资源
        # motion_state_suber_node.get_logger().info("正在清理串口资源...")
        
        # # 清理串口缓冲区并关闭连接
        # try:
        #     if motion_state_suber_node.ser and motion_state_suber_node.ser.is_open:
        #         # 读取所有剩余字符
        #         remaining_data = motion_state_suber_node.ser.read_all()
        #         if remaining_data:
        #             motion_state_suber_node.get_logger().info(f"数据发送串口剩余数据: {len(remaining_data)} 字节")
        #         motion_state_suber_node.ser.close()
        #         motion_state_suber_node.get_logger().info("数据发送串口已关闭")
        # except Exception as e:
        #     motion_state_suber_node.get_logger().error(f"关闭数据发送串口时出错: {e}")
            
        # try:
        #     if motion_state_suber_node.ser and motion_state_suber_node.ser.is_open:
        #         # 读取所有剩余字符
        #         remaining_data = motion_state_suber_node.ser.read_all()
        #         if remaining_data:
        #             motion_state_suber_node.get_logger().info(f"舵机控制串口剩余数据: {len(remaining_data)} 字节")
        #         motion_state_suber_node.ser.close()
        #         motion_state_suber_node.get_logger().info("舵机控制串口已关闭")
        # except Exception as e:
        #     motion_state_suber_node.get_logger().error(f"关闭舵机控制串口时出错: {e}")
        
        # # 销毁节点
        motion_state_suber_node.destroy_node()
        rclpy.shutdown()
        motion_state_suber_node.get_logger().info("程序已安全退出")

if __name__ == '__main__':
    main()
    