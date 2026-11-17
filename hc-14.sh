#!/bin/bash
source /home/unitree/unitree_ros2/cyclonedds_ws/install/setup.bash
cd /home/unitree/unitree_ros2
sudo chmod 777 /dev/USB_state
sudo chmod 777 /dev/USB_servo
sudo chmod 777 /dev/ttyUSB0
python hc-14.py