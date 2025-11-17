#!/usr/bin/env bash
set -e

echo "[boxygo] EntryPoint hook start"

WS=/workspaces/isaac_ros-dev
REPOS_FILE="$WS/boxygo.repos"

cd $WS

vcs import --input "$REPOS_FILE" src

colcon build --symlink-install

echo "Environement setup for BoxyGo complete."
