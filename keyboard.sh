#!/bin/bash

echo "Select the topic to publish to:"
echo "1) /cmd_vel"
echo "2) /cmd_vel_unstamped"
read -p "Your choice (1/2): " CHOICE

case $CHOICE in
  1)
    TOPIC="/cmd_vel"
    ;;
  2)
    TOPIC="/cmd_vel_unstamped"
    ;;
  *)
    echo "Invalid choice"
    exit 1
    ;;
esac

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=${TOPIC}
