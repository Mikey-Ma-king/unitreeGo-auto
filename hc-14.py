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
import signal
import sys
import atexit

INFO_FOOT_STATE = 0
HIGH_FREQ = 0
count = 0
timee = time.time()

def send_to_plc(data, port='/dev/ttyUSB0', baudrate=9600):
    """发送数据到PLC的最简函数"""
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            if isinstance(data, str):
                if all(c in '0123456789abcdefABCDEF ' for c in data):
                    data = bytes.fromhex(data.replace(' ', ''))
                else:
                    data = data.encode('ascii')
            
            ser.write(data)
            print(f"已发送: {data.hex().upper() if isinstance(data, bytes) else data}")
            return True
    except Exception as e:
        print(f"发送失败: {e}")
        return False


class MotionStateSuber(Node):

    def __init__(self):
        super().__init__('motion_state_suber')

        # 初始化控制标志和事件
        self.stop_event = threading.Event()
        self.initialization_complete = False
        self.charge_flag = None
        self.last_charge_flag = None
        self.layback_timer = None
        self.first_stage_timer = None
        self.layback_in_progress = False
        self.charge_to_first_delay = 3.5  # 收到充电标志到发送第一阶段命令的延迟，单位秒
        self.send_ser = None
        self.receive_ser = None
        self.cleaned_up = False  # 防止重复清理
        self.serial_thread = None
        
        # 设置信号处理
        self.setup_signal_handlers()
        
        # 注册退出处理
        atexit.register(self.cleanup)
        
        # 初始化舵机
        try:
            self.STservo = STservo('/dev/USB_servo')
            self.get_logger().info("舵机初始化成功")
        except Exception as e:
            self.get_logger().warn(f"舵机初始化失败: {e}")
            self.STservo = None
            
        # 创建订阅者
        topic_name = 'lf/sportmodestate'
        if HIGH_FREQ:
            topic_name = '/sportmodestate'
            
        self.suber = self.create_subscription(
            SportModeState,
            topic_name,
            self.topic_callback,
            10
        )

        # 初始化串口连接
        self.init_serial_ports()
        
        # 创建串口监听线程
        self.serial_thread = threading.Thread(target=self.serial_listener, daemon=True)
        self.serial_thread.start()
        
        self.buffer = ""
        
        # 初始化完成标志
        self.initialization_complete = True
        
    def setup_signal_handlers(self):
        """设置信号处理器，确保优雅退出"""
        def signal_handler(sig, frame):
            self.get_logger().info(f"收到信号 {sig}，正在清理资源...")
            if not self.cleaned_up:
                self.cleanup()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
    def init_serial_ports(self):
        """初始化串口连接"""
        # 先关闭可能存在的旧连接
        self.close_serial_ports()
        
        # 等待系统释放串口
        time.sleep(0.5)
        
        # 初始化数据发送串口
        try:
            self.send_ser = serial.Serial(
                port='/dev/USB_send',
                baudrate=115200,
                timeout=0.1,
                write_timeout=1.0
            )
            # 清空缓冲区
            if self.send_ser.is_open:
                self.send_ser.reset_input_buffer()
                self.send_ser.reset_output_buffer()
                self.get_logger().info("数据发送串口初始化成功")
            else:
                self.get_logger().error("数据发送串口打开失败")
                self.send_ser = None
        except Exception as e:
            self.get_logger().error(f"数据发送串口初始化失败: {e}")
            self.send_ser = None
            
        # 初始化数据接收串口
        try:
            self.receive_ser = serial.Serial(
                port='/dev/USB_receive',
                baudrate=115200,
                timeout=0.1
            )
            if self.receive_ser.is_open:
                self.receive_ser.reset_input_buffer()
                self.receive_ser.reset_output_buffer()
                self.get_logger().info("数据接收串口初始化成功")
            else:
                self.get_logger().error("数据接收串口打开失败")
                self.receive_ser = None
        except Exception as e:
            self.get_logger().error(f"数据接收串口初始化失败: {e}")
            self.receive_ser = None
    
    def close_serial_ports(self):
        """安全关闭串口"""
        # 关闭发送串口
        if self.send_ser is not None:
            try:
                if self.send_ser.is_open:
                    self.send_ser.close()
                    self.get_logger().info("发送串口已关闭")
            except Exception as e:
                pass  # 忽略关闭时的错误
            finally:
                self.send_ser = None
        
        # 关闭接收串口
        if self.receive_ser is not None:
            try:
                if self.receive_ser.is_open:
                    self.receive_ser.close()
                    self.get_logger().info("接收串口已关闭")
            except Exception as e:
                pass  # 忽略关闭时的错误
            finally:
                self.receive_ser = None

    def topic_callback(self, data):
        if not self.initialization_complete or self.cleaned_up:
            return
        
        global count, timee
        
        try:
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
            
            yaw = math.atan2(2.0*(q_y*q_x + q_w*q_z), 1 - 2*(q_y*q_y + q_z*q_z))

            x = int(x * 100)
            y = int(y * 100)
            z = int(z * 100)
            vx = int(vx * 100)
            vy = int(vy * 100)
            yaw = int(yaw * 100)
            
            self.target_angle = yaw

            formatted_data = struct.pack("6h", x, y, z, vx, vy, yaw)
            
            # 发送数据前检查串口状态
            if self.send_ser is not None and hasattr(self.send_ser, 'is_open') and self.send_ser.is_open:
                try:
                    self.send_ser.write(formatted_data + "@".encode())
                except (serial.SerialException, OSError) as e:
                    self.get_logger().error(f"串口写入错误: {e}")
                    # 尝试重新连接
                    self.reconnect_serial()
                except Exception as e:
                    self.get_logger().error(f"发送数据失败: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"处理运动状态数据失败: {e}")
    
    def reconnect_serial(self):
        """重新连接串口"""
        if self.cleaned_up:
            return
            
        self.get_logger().info("尝试重新连接串口...")
        self.init_serial_ports()
        
    def serial_listener(self):
        """串口监听线程"""
        self.get_logger().info("串口监听线程启动...")
        
        # 添加串口健康检查
        last_heartbeat = time.time()
        reconnect_count = 0
        max_reconnect = 5

        while not self.stop_event.is_set() and not self.cleaned_up:
            if not self.initialization_complete:
                time.sleep(0.1)
                continue
            
            # 定期检查串口健康状态
            current_time = time.time()
            if current_time - last_heartbeat > 10:  # 每10秒检查一次
                last_heartbeat = current_time
                if self.receive_ser is None or not self.receive_ser.is_open:
                    self.get_logger().warn("接收串口未打开或已关闭")
                    if reconnect_count < max_reconnect:
                        self.get_logger().info("尝试重新连接接收串口...")
                        self.init_serial_ports()
                        reconnect_count += 1
                    else:
                        self.get_logger().error("重连次数过多，停止重连")
                        break

            # 读取串口数据
            if self.receive_ser is not None and hasattr(self.receive_ser, 'in_waiting'):
                try:
                    if self.receive_ser.in_waiting > 0:
                        data = self.receive_ser.read(self.receive_ser.in_waiting)
                        self.buffer += data.decode('utf-8', errors='ignore')
                        
                        # 解析数据包
                        pos = self.buffer.find('@')
                        if pos != -1:
                            data_str = self.buffer[0:pos]
                            self.buffer = self.buffer[pos+1:]
                            
                            try:
                                current_charge_flag = int(data_str)
                                self.charge_flag = current_charge_flag
                                self.get_logger().info(f"收到充电标志: {self.charge_flag}")

                                if current_charge_flag == 1:
                                    # 在完整流程结束前，任何 1 都视为本轮流程中的重复触发
                                    if self.layback_in_progress or self.first_stage_timer is not None:
                                        self.get_logger().warn("当前充电流程未结束，忽略重复触发")
                                        continue

                                    # 仅在上一轮流程执行完后才允许重新触发
                                    if self.last_charge_flag == 1 and not self.layback_in_progress:
                                        self.get_logger().warn("上一轮流程已结束，可重新触发新一轮充电")

                                    self.last_charge_flag = 1
                                    self.get_logger().info(
                                        f"收到充电标志，{self.charge_to_first_delay}秒后发送第一阶段命令"
                                    )
                                    self.first_stage_timer = self.create_timer(
                                        self.charge_to_first_delay,
                                        self.layback_to_plc
                                    )
                                elif current_charge_flag == 0:
                                    self.last_charge_flag = 0

                            except ValueError as e:
                                self.get_logger().error(f"数据解析错误: {e}, 数据: {data_str}")
                                
                except (serial.SerialException, OSError, AttributeError) as e:
                    if not self.cleaned_up:  # 只在未清理时记录错误
                        self.get_logger().error(f"串口读取错误: {e}")
                        time.sleep(0.5)
                except Exception as e:
                    if not self.cleaned_up:
                        self.get_logger().error(f"串口读取未知错误: {e}")
                        
            time.sleep(0.01)
        
        self.get_logger().info("串口监听线程退出")
    
    def send_second_layback_command(self):
        """发送第二阶段降落命令"""
        if self.cleaned_up:
            return

        try:
            if send_to_plc("0002"):
                self.get_logger().info("发送降落第二阶段命令: 0002 成功")
            else:
                self.get_logger().error("发送降落第二阶段命令失败")
        except Exception as e:
            self.get_logger().error(f"发送第二阶段命令异常: {e}")
        finally:
            self.layback_in_progress = False
            self.first_stage_timer = None
            if self.layback_timer:
                self.layback_timer.cancel()
                self.layback_timer = None

    def layback_to_plc(self):
        """发送降落序列命令"""
        if self.cleaned_up:
            return

        if self.first_stage_timer is not None:
            self.first_stage_timer.cancel()
            self.first_stage_timer = None

        if self.layback_in_progress:
            self.get_logger().warn("降落流程已在进行中，忽略重复请求")
            return

        self.layback_in_progress = True

        try:
            if send_to_plc("0001"):
                self.get_logger().info("发送降落第一阶段命令: 0001 成功")
                self.layback_timer = self.create_timer(5.0, self.send_second_layback_command)
                self.get_logger().info("已设置5秒后发送第二阶段命令")
            else:
                self.get_logger().error("发送降落第一阶段命令失败")
                self.layback_in_progress = False
        except Exception as e:
            self.get_logger().error(f"发送第一阶段命令异常: {e}")
            self.layback_in_progress = False
    
    def cleanup(self):
        """彻底清理资源，防止重复调用"""
        if self.cleaned_up:
            return
            
        self.cleaned_up = True
        self.initialization_complete = False
        
        self.get_logger().info("开始清理资源...")
        
        # 停止串口监听线程
        self.stop_event.set()
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=2.0)
            self.get_logger().info("串口监听线程已停止")
        
        # 取消计时器
        if self.first_stage_timer:
            try:
                self.first_stage_timer.cancel()
            except:
                pass
            self.first_stage_timer = None

        if self.layback_timer:
            try:
                self.layback_timer.cancel()
            except:
                pass
            self.layback_timer = None
        
        # 关闭串口
        self.close_serial_ports()
        
        # 额外等待确保资源释放
        time.sleep(0.1)
        self.get_logger().info("资源清理完成")


def main(args=None):
    rclpy.init(args=args)
    
    motion_state_suber_node = None
    
    try:
        motion_state_suber_node = MotionStateSuber()
        rclpy.spin(motion_state_suber_node)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行错误: {e}")
    finally:
        if motion_state_suber_node:
            motion_state_suber_node.cleanup()
            motion_state_suber_node.destroy_node()
        rclpy.shutdown()
        print("程序已安全退出")

if __name__ == '__main__':
    main()
