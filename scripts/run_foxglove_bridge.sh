#!/bin/bash

source /opt/ros/humble/setup.bash

source ~/workspaces/BoxyGo/install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
