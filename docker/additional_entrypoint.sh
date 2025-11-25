#!/bin/bash

WS=/workspaces/isaac_ros-dev
REPOS_FILE="$WS/boxygo.repos"

cd $WS

usermod -aG dialout admin || true
usermod -aG plocate admin || true

git config --global --add safe.directory "$WS" || true
git config --global --add safe.directory "$WS/src/moteus_ros2" || true
git config --global --add safe.directory /workspaces/isaac_ros-dev/src/yydlidar_ros2_driver-humble || true
git config --global --add safe.directory /workspaces/isaac_ros-dev/src/YDLidar-SDK || true
git config --global --add safe.directory /workspaces/isaac_ros-dev/src/isaac_ros_nvblox || true

vcs import --input "$REPOS_FILE" src

cd ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/nvblox_examples/realsense_splitter && \
    git update-index --assume-unchanged COLCON_IGNORE && \
    rm COLCON_IGNORE

chown -R admin:admin "$WS/src"

cd $WS/src/YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
cd ..
pip install .

if ! grep -q "# BoxyGo aliases" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# BoxyGo aliases" >> ~/.bashrc

    echo "alias b='cd $WS && colcon build --symlink-install && source install/setup.bash'" >> ~/.bashrc

    echo "alias k=\"ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped\"" >> ~/.bashrc
fi

cd $WS

echo "Environment setup for BoxyGo complete."
echo "Run 'source ~/.bashrc' to load new aliases."

