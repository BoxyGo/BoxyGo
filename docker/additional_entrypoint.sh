#!/bin/bash

WS=/workspaces/isaac_ros-dev
REPOS_FILE="$WS/boxygo.repos"

cd $WS

usermod -aG dialout admin || true
usermod -aG plocate admin || true

git config --global --add safe.directory "$WS" || true
git config --global --add safe.directory "$WS/src/moteus_ros2" || true
git config --global --add safe.directory /workspaces/isaac_ros-dev/src/ydlidar_ros2_driver-humble || true
git config --global --add safe.directory /workspaces/isaac_ros-dev/src/YDLidar-SDK || true

vcs import --input "$REPOS_FILE" src

chown -R admin:admin "$WS/src"

cd $WS/src/YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
cd ..
pip install .

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/isaac_ros-dev/install/boxygo_description/share/boxygo_description

cd $WS

echo "Environment setup for BoxyGo complete."
echo "Run 'source ~/.bashrc' to load new aliases."
