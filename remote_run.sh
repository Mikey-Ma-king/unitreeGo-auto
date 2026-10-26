#!/bin/bash

# 定义所有机器的 IP 地址
# MACHINES=("localhost" "192.168.1.125" "192.168.1.149" "192.168.1.142") # 假设 localhost 代表本机
MACHINES=("192.168.1.175" "192.168.1.125" "192.168.1.149" "192.168.1.142") # 假设 localhost 代表本机
#MACHINES=("192.168.200.236" "192.168.200.32" "192.168.200.174" "192.168.200.34") # 假设 localhost 代表本机

# 定义清理函数
cleanup() {
    for IP in "${MACHINES[@]}"
    do
        echo "Sending stop signal to $IP..."
        if [ "$IP" == "localhost" ]; then
            pkill -f python
        else
            ssh unitree@$IP 'pkill -f python'
        fi
    done
    exit
}

# 设置信号处理
trap cleanup SIGINT

# 推送代码到远程仓库
echo "Pushing latest changes to the master branch..."
cd /home/unitree/unitree_ros2
git add .
git commit -m 'Auto-commit: update changes before running scripts'
git push origin master

# 远程执行脚本
python tools/udp_controller_server.py &
for IP in "${MACHINES[@]}"
do
    if [ "$IP" == "localhost" ]; then
        echo "Running scripts on localhost..."
        bash /home/unitree/unitree_ros2/run.sh &
    else
        echo "Attempting to run scripts on $IP..."
        ssh unitree@$IP 'bash --init-file ~/.bashrc /home/unitree/unitree_ros2/run.sh' &
    fi
    if [ $? -ne 0 ]; then
        echo "Failed to connect or execute on $IP"
    fi
done

wait
