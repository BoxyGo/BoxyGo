#!/usr/bin/env bash
set -e

DEST_DIR="$HOME/isaac_ros"

mkdir -p "$DEST_DIR"
cd "$DEST_DIR"

git clone --branch release-3.2 --depth 1 \
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
