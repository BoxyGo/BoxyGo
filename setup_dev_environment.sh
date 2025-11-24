#!/usr/bin/env bash
set -euo pipefail

echo "=== Installing Docker (if needed) ==="
if ! command -v docker >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y docker.io
  sudo systemctl enable --now docker
else
  echo "Docker already installed, skipping."
fi

echo "=== Adding user to docker group ==="
if ! groups "$USER" | grep -q docker; then
  sudo usermod -aG docker "$USER"
  echo "User added to docker group. Log out and log in again for permissions to apply."
else
  echo "User already in docker group, skipping."
fi

echo "=== Installing Git LFS (if needed) ==="
if ! command -v git-lfs >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y git-lfs
  git lfs install
else
  echo "Git LFS already installed, skipping."
fi

echo "=== Creating workspaces directory ==="
mkdir -p "$HOME/workspaces"

echo "=== Cloning BoxyGo repository ==="
BOXYGO_DIR="$HOME/workspaces/BoxyGo"
if [ ! -d "$BOXYGO_DIR/.git" ]; then
  git clone https://github.com/BoxyGo/BoxyGo.git "$BOXYGO_DIR"
else
  echo "BoxyGo repo already exists in $BOXYGO_DIR, skipping."
fi

echo "=== Cloning Isaac ROS Common ==="
ISAAC_ROOT="$HOME/isaac_ros"
ISAAC_COMMON_DIR="$ISAAC_ROOT/isaac_ros_common"
mkdir -p "$ISAAC_ROOT"

if [ ! -d "$ISAAC_COMMON_DIR/.git" ]; then
  git clone --branch release-3.2 --depth 1 \
    https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git \
    "$ISAAC_COMMON_DIR"
else
  echo "Isaac ROS Common already exists in $ISAAC_COMMON_DIR, skipping."
fi

CONFIG_PATH="$HOME/.isaac_ros_common-config"

ARCH="$(uname -m || echo unknown)"

case "$ARCH" in
  x86_64)
    CONFIG_IMAGE_KEY="ros2_humble.realsense.boxygo.dev"
    ;;
  aarch64|arm64)
    CONFIG_IMAGE_KEY="ros2_humble.realsense.boxygo.robot"
    ;;
  *)
    echo "Warning: unsupported architecture '$ARCH'. Defaulting to x86_64 image."
    CONFIG_IMAGE_KEY="ros2_humble.boxygo.dev"
    ;;
esac

echo "Writing config file to: $CONFIG_PATH"

cat > "$CONFIG_PATH" << EOF
CONFIG_IMAGE_KEY="$CONFIG_IMAGE_KEY"
CONFIG_DOCKER_SEARCH_DIRS=(~/workspaces/BoxyGo/docker)

ADDITIONAL_RUN_ARGS=(
  "--privileged"
  "--device=/dev/ttyUSB0:/dev/ttyUSB0"
)
EOF

echo
echo "=== Setup complete ==="
echo "If this is the first time adding yourself to the docker group, log out and log in again."
echo
echo "Next steps:"
echo "  cd \$HOME/workspaces/BoxyGo"
echo "  ./run_dev_container.sh"
echo "  ./configure_dev_container.sh"
