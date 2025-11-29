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

RULES_FILE="/etc/udev/rules.d/99-robot-usb.rules"

echo "=== [AUTOSETUP] Check udev rules ==="

NEED_SETUP=false

if [ ! -e /dev/ydlidar ]; then
  echo "[AUTOSETUP] Brak /dev/ydlidar"
  NEED_SETUP=true
fi

if [ ! -e /dev/can_usb ]; then
  echo "[AUTOSETUP] Brak /dev/can_usb"
  NEED_SETUP=true
fi

if [ "${NEED_SETUP}" = true ]; then
  echo "[AUTOSETUP] Create ${RULES_FILE}"

  sudo bash -c "cat > ${RULES_FILE}" << 'EOF'
# LIDAR: vendor 0483, product 5740, serial 00000000001A
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="00000000001A", SYMLINK+="ydlidar"

# CAN: vendor 0483, product 5740, serial 12BB92BE
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="12BB92BE", SYMLINK+="can_usb"
EOF

  echo "[AUTOSETUP] Realoading udev rules"
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  sleep 1
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
EOF

DOCKERARGS_FILE="$HOME/.isaac_ros_dev-dockerargs"

echo "Writing docker args file to: $DOCKERARGS_FILE"

cat > "$DOCKERARGS_FILE" << EOF
--cap-add=SYS_NICE
--cap-add=IPC_LOCK
--ulimit rtprio=99
--ulimit memlock=-1
--privileged
-v /dev:/dev
EOF

echo "Config written to $DOCKERARGS_FILE"

echo "Next steps:"
echo "  cd \$HOME/workspaces/BoxyGo"
echo "  ./run_dev_container.sh"
