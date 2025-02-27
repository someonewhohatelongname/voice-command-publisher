#!/bin/bash

# Paths to your executables and model
WHISPER_EXEC="stdbuf -oL -eL /home/ubuntu/Documents/ws_ros2/src/whisper.cpp/build/bin/whisper-command"
WHISPER_MODEL="/home/ubuntu/Documents/ws_ros2/src/whisper.cpp/models/ggml-model_test1.bin"
ROS_WORKSPACE="/home/ubuntu/Documents/ws_ros2"

# Ensure the workspace is built
cd $ROS_WORKSPACE
source install/setup.bash

# Start Whisper.cpp and pipe its output to the voice_offboard_node
$WHISPER_EXEC -m $WHISPER_MODEL -t 8 -c 0 | ros2 run voice_control_cpp whisper_offboard_node

