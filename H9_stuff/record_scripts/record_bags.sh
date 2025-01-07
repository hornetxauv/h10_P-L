#!/bin/bash

# Make bag directory if not present
mkdir -p pooltest_bags

# Define filename template
filename_template="pooltest_bags/%d.bag"

# Get the current highest numbered file
latest_file=$(ls -v pooltest_bags/ | tail -n 1)

# Extract the number from the latest file
latest_number=$(basename "$latest_file" .bag)

# Increment the number
next_number=$((latest_number + 1))

# Format the filename with the next number
filename=$(printf "$filename_template" "$next_number")

# Record the published topics
# ros2 bag record -o "$filename" /left/image_raw /right/image_raw /bottom/image_raw /poolLines /left/yolo/box # INSERT TOPIC FOR GATE BEARING , DISTANCE, AND TILT ANGLE

# ros2 bag record -o "$filename" /debug/bgr/compressed
# ros2 bag record -o "$filename" /bottom/image_raw
# ros2 bag record -o "$filename" /bottom/rect/image
# `ros2 bag record -o "$filename" /bottom/rect/image orange_flare/bgr/compressed
# ros2 bag record -o "$filename" /bottom/rect/detections_output /sensors/imu/corrected /sensors/depth
# ros2 bag record -o "$filename" /bgr/compressed
ros2 bag record -o "$filename" /left/image_raw /perc/debug_img_4 /perc/quali_gate
# ros2 bag record -o "$filename" /left/image_raw /right/image_raw # INSERT TOPIC FOR GATE BEARING , DISTANCE, AND TILT ANGLE
# ros2 bag record -o "$filename" /bottom/image_raw /sensors/imu # INSERT TOPIC FOR GATE BEARING , DISTANCE, AND TILT ANGLE
# ros2 bag record -o "$filename" /left/image_raw /left/image_rect_color /left/yolo/box
# ros2 bag record -o "$filename" /sensors/imu /left/camera_info /left/image_raw/compressed /left/image_rect_color /right/camera_info /right/image_raw/compressed /right/image_rect_color /bottom/camera_info /bottom/image_raw/compressed
 
