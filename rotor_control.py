import serial

def main():
    # 打开串口
    ser = serial.Serial('/dev/ttyUSB3', 115200, timeout=0.1)  # 根据实际情况修改端口
    print("等待接收数据...")

    while True:
        if ser.in_waiting:  # 检查串口缓冲区是否有数据
            received_data = ser.read(1).decode('utf-8', errors='ignore')  # 读取一个字节并解码
            if received_data in ['+', '-']:
                print(f"收到: {received_data}")
            else:
                print(f"收到未知数据: {received_data}")

if __name__ == "__main__":
    main()
