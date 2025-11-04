#!/usr/bin/env bash
set -e

ISAAC_ROOT="$HOME/isaac_ros"
ISAAC_COMMON_DIR="$ISAAC_ROOT/isaac_ros_common"
REPO_URL="https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git"
BRANCH="release-3.2"

echo ">>> Creating directory: $ISAAC_ROOT"
mkdir -p "$ISAAC_ROOT"

# 1. Clone isaac_ros_common (if not exists)
if [ ! -d "$ISAAC_COMMON_DIR/.git" ]; then
  echo ">>> Cloning $REPO_URL into $ISAAC_COMMON_DIR"
  git clone "$REPO_URL" "$ISAAC_COMMON_DIR"
else
  echo ">>> Repository already exists in $ISAAC_COMMON_DIR, skipping clone"
fi

# 2. Switch to the correct branch and update submodules
cd "$ISAAC_COMMON_DIR"
echo ">>> Fetching origin and checking out branch $BRANCH"
git fetch origin
git checkout "$BRANCH"
git submodule update --init --recursive

# 3. Add environment variables and alias to ~/.bashrc

# PATH_TO_ISAAC_ROS_COMMON
if ! grep -q "PATH_TO_ISAAC_ROS_COMMON" "$HOME/.bashrc"; then
  echo ">>> Adding PATH_TO_ISAAC_ROS_COMMON to ~/.bashrc"
  cat >> "$HOME/.bashrc" << 'EOF'

# Isaac ROS common path
export PATH_TO_ISAAC_ROS_COMMON="$HOME/isaac_ros/isaac_ros_common"
EOF
else
  echo ">>> PATH_TO_ISAAC_ROS_COMMON already exists in ~/.bashrc, skipping"
fi

# ISAAC_ROS_WS
if ! grep -q "ISAAC_ROS_WS" "$HOME/.bashrc"; then
  echo ">>> Adding ISAAC_ROS_WS to ~/.bashrc"
  cat >> "$HOME/.bashrc" << 'EOF'

# Isaac ROS workspace path
export ISAAC_ROS_WS="$HOME/workspaces/BoxyGo"
EOF
else
  echo ">>> ISAAC_ROS_WS already exists in ~/.bashrc, skipping"
fi

# alias k – teleop to control robot in Gazebo
if ! grep -q "alias k=" "$HOME/.bashrc"; then
  echo ">>> Adding alias k to ~/.bashrc"
  cat >> "$HOME/.bashrc" << 'EOF'

# Quick teleop alias to control the robot in Gazebo
alias k='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped'
EOF
else
  echo ">>> Alias k already exists in ~/.bashrc, skipping"
fi

# 4. Set variables and alias in current session
export PATH_TO_ISAAC_ROS_COMMON="$HOME/isaac_ros/isaac_ros_common"
export ISAAC_ROS_WS="$HOME/workspaces/BoxyGo"
alias k='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped'

echo ">>> PATH_TO_ISAAC_ROS_COMMON set to: $PATH_TO_ISAAC_ROS_COMMON"
echo ">>> ISAAC_ROS_WS set to: $ISAAC_ROS_WS"
echo ">>> Alias k available in current session"

# 5. Create ~/.isaac_ros_common-config
echo ">>> Creating ~/.isaac_ros_common-config"
cat > "$HOME/.isaac_ros_common-config" << 'EOF'
CONFIG_IMAGE_KEY="ros2_humble.boxygo.dev"
CONFIG_DOCKER_SEARCH_DIRS=(~/workspaces/BoxyGo/docker)
CONFIG_WORKSPACE_DIR=~/workspaces/BoxyGo
EOF

# 6. Source updated ~/.bashrc
echo ">>> Sourcing updated ~/.bashrc"
# shellcheck source=/dev/null
source "$HOME/.bashrc"

echo ">>> Done! You can now use:"
echo "    alias r - to run the Isaac ROS developer container with BoxyGo"
echo "    alias k - to control the robot in Gazebo using the keyboard"
