#!/bin/bash
sudo chmod 777 /dev/ttyUSB1
# 设置串口路径
PORT="/dev/ttyUSB1"

# 配置串口参数
# 例如设置波特率为 9600, 数据位为 8, 停止位为 1, 无校验位
stty -F $PORT 9600 cs8 -cstopb -parenb

# 发送二进制数据
# 注意：这里使用 echo 的 -e 选项来允许解释特殊字符
echo -e '\x00\x5b\x00\x00\x70\x00' > "$PORT"

# 可选：读取串口返回的数据
# 如果你的设备在接收到命令后会返回数据，可以用以下命令读取
cat $PORT
