#!/bin/bash

# 定义一个函数来处理中断信号
cleanup() {
    echo "Terminating all running processes..."
    pkill -P $$
    exit 0
}

# 捕获 SIGINT (Ctrl+C) 信号并调用 cleanup 函数
trap cleanup SIGINT

# 第一个命令块
source /opt/ros/noetic/setup.bash
roslaunch realsense2_camera rs_camera.launch &

# 等待几秒钟以确保第一个命令块启动
sleep 5

# 第二个命令块
source /opt/ros/noetic/setup.bash
cd /unitree/module/Odometer_service/
source devel/setup.bash
cd /unitree/module/Odometer_service/src/rpg_svo_pro_open/svo_ros/launch
rosparam load bridge.yaml
roslaunch rs_camera.launch &

# 再次等待几秒钟以确保第二个命令块启动
sleep 5

# 第三个命令块
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge parameter_bridge &

# 等待所有后台进程完成
wait
