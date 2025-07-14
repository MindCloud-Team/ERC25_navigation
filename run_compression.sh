#!/bin/bash

echo "Starting image compression nodes with UNIQUE NAMES..."

# Source your ROS 2 workspace
# IMPORTANT: Make sure this path is correct for your system!
source ~/ros2_ws/install/setup.bash

# --- NOTE THE ADDITION OF --remap __node:=... TO EACH COMMAND ---

# Command for front camera
ros2 run image_transport republish raw compressed --ros-args --remap __node:=compress_front_cam \
-r in:=/front_cam/zed_node/rgb/image_rect_color \
-r out/compressed:=/front_cam/zed_node/rgb/image_rect_color/compressed &

# Command for back camera
ros2 run image_transport republish raw compressed --ros-args --remap __node:=compress_back_cam \
-r in:=/back_cam/zed_node/rgb/image_rect_color \
-r out/compressed:=/back_cam/zed_node/rgb/image_rect_color/compressed &

# Command for left camera
ros2 run image_transport republish raw compressed --ros-args --remap __node:=compress_left_cam \
-r in:=/left_cam/zed_node/rgb/image_rect_color \
-r out/compressed:=/left_cam/zed_node/rgb/image_rect_color/compressed &

# Command for right camera
ros2 run image_transport republish raw compressed --ros-args --remap __node:=compress_right_cam \
-r in:=/right_cam/zed_node/rgb/image_rect_color \
-r out/compressed:=/right_cam/zed_node/rgb/image_rect_color/compressed &

echo "All compression nodes launched. You can verify with 'ros2 node list'."
echo "Press Ctrl+C in this terminal to stop them all."

# Wait for all background processes to finish (e.g., when you press Ctrl+C)
wait
