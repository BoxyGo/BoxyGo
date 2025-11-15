#!/bin/bash
set -e

VERSION=${1:-latest}
IMAGE_NAME="docker.io/ligas10105/boxygo:${VERSION}"

WS=~/workspaces/BoxyGo
ISAAC_COMMON=~/isaac_ros/isaac_ros_common/scripts

echo ">>> Running docker_deploy.sh..."
$ISAAC_COMMON/docker_deploy.sh \
   --base_image_key "aarch64.ros2_humble" \
   --ros_ws $WS \
   --launch_package "boxygo" \
   --launch_file "moteus_launch.py" \
   --name "$IMAGE_NAME"

echo " >>> Done! "
