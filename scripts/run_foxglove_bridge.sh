#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/workspaces/BoxyGo/install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
BRIDGE_PID=$!

sleep 2

xdg-open "https://studio.foxglove.dev" >/dev/null 2>&1 &

wait $BRIDGE_PID