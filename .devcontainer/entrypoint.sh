#!/usr/bin/env bash
set -euo pipefail

SENTINEL="/home/ws/.devcontainer/.setup_done"

# First-run only (equivalent to postCreateCommand):
# fix volume ownership, clear stale git locks, install rosdep deps
if [ ! -f "$SENTINEL" ]; then
    echo "[entrypoint] First-run setup..."
    sudo chown -R "$(whoami)" /home/ws
    sudo rm -f /home/ws/.git/*.lock /home/ws/.git/refs/heads/*.lock
    sudo rosdep update
    sudo rosdep install --from-paths src --ignore-src -y
    touch "$SENTINEL"
fi

# Every start (equivalent to postStartCommand): build the workspace
echo "[entrypoint] Building ROS 2 workspace..."
source "/opt/ros/$ROS_DISTRO/setup.bash"
colcon build --symlink-install

exec "$@"
