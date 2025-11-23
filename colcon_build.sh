#!/bin/bash

colcon build --symlink-install --packages-up-to realsense_splitter
colcon build --symlink-install --packages-skip-regex "nvblox*"

echo "Source the workspace with: source install/setup.bash"