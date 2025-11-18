#!/usr/bin/env bash
set -e

echo "[boxygo] EntryPoint hook start"

WS=/workspaces/isaac_ros-dev
REPOS_FILE="$WS/boxygo.repos"

cd $WS

# Import repo
vcs import --input "$REPOS_FILE" src

# Add aliases once (if not already added)
if ! grep -q "# BoxyGo aliases" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# BoxyGo aliases" >> ~/.bashrc

    # Alias: build + source
    echo "alias b='cd $WS && colcon build --symlink-install && source install/setup.bash'" >> ~/.bashrc

    # Alias: run teleop
    echo "alias k=\"ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped\"" >> ~/.bashrc
fi

echo "Environment setup for BoxyGo complete."
echo "Run 'source ~/.bashrc' to load new aliases."
