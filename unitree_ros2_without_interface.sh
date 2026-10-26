#!/bin/bash
echo "Setup unitree ros2 simulation environment"
source /opt/ros/foxy/setup.bash
source cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
