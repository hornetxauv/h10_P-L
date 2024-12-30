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
# /sensors/imu
# /left/image_raw/compressed
# /right/image_raw/compressed
# /bottom/image_raw/compressed

ros2 bag record -o $filename /sensors/imu /left/image_raw/compressed /right/image_raw/compressed /bottom/image_raw/compressed