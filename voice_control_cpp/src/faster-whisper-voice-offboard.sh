#!/bin/bash

# Paths to your executables and model
SERVER_IP="0.0.0.0"  # Change to server IP if needed
SERVER_PORT="12345"  # Match the port from your server
ROS_WORKSPACE="/home/ubuntu/Documents/ws_ros2"

export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages
source /opt/ros/humble/setup.bash  # Change "humble" to your ROS2 distribution

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/Documents/ws_ros2/src/porcupine/lib/linux/x86_64:/home/ubuntu/Documents/ws_ros2/src/whisper.cpp/build/src

colcon build --packages-select voice_control_cpp
# Ensure the workspace is built
cd $ROS_WORKSPACE
source install/setup.bash


python3 -u /home/ubuntu/Documents/ws_ros2/src/faster-whisper/tests/laptop_server.py --host 0.0.0.0 --port 12345 --model-path /home/ubuntu/Downloads/drone_model --device cpu --compute int8 | ros2 run voice_control_cpp faster_offboard_node
