import sys
import os
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from STservo_sdk import *                 # Uses STServo SDK library


class STservo():
    def __init__(self,port):
        super().__init__()

        # Default setting
        self.STS_ID                      = 1                 # STServo ID : 1
        self.BAUDRATE                    = 1000000           # STServo default baudrate : 1000000
        self.DEVICENAME                  = port    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        self.STS_MOVING_SPEED            = 1000              # STServo moving speed (0-3000)
        self.STS_MOVING_ACC              = 50                # STServo moving acc (0-255)
        self.STS_MODE                    = 0x21
        self.STS_MIN_SPEED               = 0x09
        self.STS_MAX_SPEED               = 0x0B
        self.STS_STEP                    = 0x2A


        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Get methods and members of Protocol
        self.packetHandler = sts(self.portHandler)

        self.pre_angle = -1

        self.init_Servo()


    def init_Servo(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        sts_comm_result, sts_error = self.packetHandler.WriteSpec(self.STS_ID, 0, self.STS_MOVING_ACC)
        if sts_comm_result != COMM_SUCCESS:
            print("设置速度失败: %s" % self.packetHandler.getTxRxResult(sts_comm_result))
        elif sts_error != 0:
            print("舵机错误: %s" % self.packetHandler.getRxPacketError(sts_error))
        else:
            print("舵机已设置为位置模式")

        ## 设置工作模式
        self.packetHandler.write2ByteTxRx(self.STS_ID, self.STS_MIN_SPEED, 0)   # 最小角度=0
        self.packetHandler.write2ByteTxRx(self.STS_ID, self.STS_MAX_SPEED, 0)   # 最大角度=0
        self.packetHandler.write1ByteTxRx(self.STS_ID, self.STS_MODE, 3)   # 模式
        time.sleep(0.1)



    def AngleTransform(self, pre_angle, target_angle):

        # target_position = int((angle / 360.0) * 4095)
        # sts_present_position = int((pre_angle / 360.0) * 4095)

        if pre_angle > target_angle:
            anticlkwise = pre_angle - target_angle
            clkwise = 360 + target_angle - pre_angle

        else :
            clkwise = target_angle - pre_angle
            anticlkwise = 360 + pre_angle - target_angle
        
        if clkwise <= anticlkwise:
            return clkwise, 1
        else:
            return anticlkwise, 0

    def RotateServo(self,target_angle):

        try:
            if self.pre_angle == -1:
                self.pre_angle = 0
            else:
                self.pre_angle = target_angle
            target_angle = float(target_angle)
            ang_value, flag = self.AngleTransform(self.pre_angle,target_angle)
            print(f'pre angle is {self.pre_angle}')
            print(f'present angle is {target_angle}')
            print(f'step value is {ang_value}')
            print(flag)

            step_value = int(ang_value  * 4095 / 360)
            
            ## clkwise
            if flag == 1:
                step_value = step_value & 0x7FFF
                self.packetHandler.write2ByteTxRx(self.STS_ID, self.STS_STEP, step_value)
            else:
                step_value = step_value | 0x8000
                self.packetHandler.write2ByteTxRx(self.STS_ID, self.STS_STEP, step_value)

        except ValueError:
            print("舵机控制出现错误！")


if __name__ == "__main__":

    STservo = STservo(port="COM5")
    for angle in [20,70,150,200,70,350,50]:
        time.sleep(2)
        STservo.RotateServo(angle)
        

    STservo.portHandler.closePort()
    print("程序已退出")