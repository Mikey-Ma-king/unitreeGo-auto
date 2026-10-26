#!/bin/bash
source  /opt/ros/foxy/setup.bash;
source ~/cyclonedds_ws/install/setup.bash;
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml;
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# <<< fishros initialize <<<
export PATH=/usr/local/cuda-11.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH

source /home/unitree/unitree_ros2/cyclonedds_ws/install/setup.bash

# 切换工作目录
cd /home/unitree/unitree_ros2

# 从远程仓库更新代码
git reset --hard
git pull origin master

# 激活Python虚拟环境
. .env/bin/activate

# 同时执行两个Python脚本，并且能够通过一个ctrl + C同时结束这两个脚本
function cleanup() {
    echo "Terminating Python scripts..."
    pkill -P $$
}
trap cleanup SIGINT

python tools/udp_controller_client.py &
sleep 2
# python df_new_one_dir.py &
ROBOT_ID=$(cat /home/unitree/robot_id) python dp_test_turn_8.py &

wait
