# Boxygo

**Boxygo** is an innovative solution for residential complexes – an autonomous parcel delivery robot that is part of an integrated package delivery system. It enables contactless delivery of parcels directly to the residents' doorsteps.

# Configure environment

1. Install docker:
```
sudo apt-get update
sudo apt-get install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
newgrp docker
```
2. Install git lfs: `sudo apt-get install -y git-lfs`
3. Create workspaces directory: `mkdir -p ~/workspaces`
4. Clone repo into this directory: `git clone https://github.com/BoxyGo/BoxyGo.git ~/workspaces`
5. Clone issac ros common: `git clone --branch release-3.2 --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git ~/isaac_ros/isaac_ros_common`
6. Create config file: .issac_common-config:

For x86_64:

```
cat << 'EOF' > isaac_common-config
CONFIG_IMAGE_KEY="ros2_humble.boxygo.dev"
CONFIG_DOCKER_SEARCH_DIRS=(~/workspaces/BoxyGo/docker)

ADDITIONAL_RUN_ARGS=(
  "--privileged"
  "--device=/dev:/dev"
)
EOF
```
For aarch64 on Jetson:
```
cat << 'EOF' > isaac_common-config
CONFIG_IMAGE_KEY="ros2_humble.boxygo.robot"
CONFIG_DOCKER_SEARCH_DIRS=(~/workspaces/BoxyGo/docker)

ADDITIONAL_RUN_ARGS=(
  "--privileged"
  "--device=/dev:/dev"
)
EOF
```
7. Go to workspace: `cd ~/workspaces/BoxyGo`
8. Run script: `./run_dev_container.sh`
9. If container started without problems, run script: `./configure_dev_container.sh`

# Moteus configuration:

**Wheels config:**
`conf set aux1.spi.mode 1`      
`conf set aux1.pins.0.mode 6`  
`conf set aux1.pins.0.pull 1`  
`conf set aux1.pins.2.mode 6`   
`conf set aux1.pins.2.pull 1`   
`conf set aux1.pins.3.mode 6` 
`conf set aux1.pins.3.pull 1`  
`conf set aux1.hall.enabled 1` 
`conf set motor_position.sources.0.type 4`

**Wheels calibration command:**
`python3 -m moteus.moteus_tool --target 1 --calibrate --cal-hall --call-motor-poles 30`

**Write config to wheel command:**
`python3 -m moteus.moteus_tool --target 1 --write-config FILE`




