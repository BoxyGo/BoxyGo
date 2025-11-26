#!/bin/bash

echo "Select the topic to publish to:"
echo "1) /diff_cont/cmd_vel"
echo "2) /diff_cont/cmd_vel_unstamped"
echo "3) /diff_cont/cmd_vel_out"
read -p "Your choice (1/2): " CHOICE

case $CHOICE in
  1)
    TOPIC="/diff_cont/cmd_vel"
    ;;
  2)
    TOPIC="/diff_cont/cmd_vel_unstamped"
    ;;
  3)
    TOPIC="/diff_cont/cmd_vel_out"
    ;;
  *)
    echo "Invalid choice"
    exit 1
    ;;
esac

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=${TOPIC}
