#!/bin/bash

# 定义所有机器的 IP 地址
MACHINES=("192.168.1.125" "192.168.1.141" "localhost" "192.168.1.142") # 假设 localhost 代表本机

# 定义清理函数
for IP in "${MACHINES[@]}"
do
    echo "Sending stop signal to $IP..."
    ssh unitree@$IP 'pkill -f python'
done
