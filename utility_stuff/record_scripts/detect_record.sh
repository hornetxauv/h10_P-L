#!/bin/bash

# Define filename template
filename_template="pooltest_bags/%d.bag"

# Get the current highest numbered file
latest_file=$(ls -v pooltest_bags/*.bag | tail -n 1)

# Extract the number from the latest file
latest_number=$(basename "$latest_file" .bag)

# Increment the number
next_number=$((latest_number + 1))

# Format the filename with the next number
filename=$(printf "$filename_template" "$next_number")

# Record the published topics

# Topics:
# /left/camera_info
# /right/camera_info
# /bottom/camera_info
# /left/image_raw
# /right/image_raw
# /bottom/image_raw
# /left/image_rect_color
# /right/image_rect/color
# /left/gray_world/compressed
# /gate/finalmask/compressed
# /object/gate/distance
# /object/gate/bearing

ros2 bag record -o $filename /left/camera_info /right/camera_info /bottom/camera_info /left/image_raw /right/image_raw /bottom/image_raw