
# This script builds the BoxyGo project using colcon
cd ~/BoxyGo

colcon build --symlink-install
source ~/BoxyGo/install/setup.bash
