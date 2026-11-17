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

# 使用示例
send_to_plc("0002")           # 发送文本
