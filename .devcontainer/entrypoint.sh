#!/usr/bin/env bash
set -eo pipefail

SENTINEL="/home/ws/.devcontainer/.setup_done"

# First-run only: fix volume ownership, clear stale git locks, install rosdep deps
if [ ! -f "$SENTINEL" ]; then
    echo "[entrypoint] First-run setup..."
    sudo chown -R "$(whoami)" /home/ws
    sudo rm -f /home/ws/.git/*.lock /home/ws/.git/refs/heads/*.lock
    sudo rosdep update
    sudo rosdep install --from-paths src --ignore-src -y
    mkdir -p "$(dirname "$SENTINEL")"
    touch "$SENTINEL"
fi

# Source ROS so subsequent steps (colcon, exec'd shell) can use it.
# ROS setup scripts reference unset vars internally, so -u must be off here.
set +u
# shellcheck disable=SC1091
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

# Build the workspace on every start (no-op if nothing changed).
echo "[entrypoint] Building ROS 2 workspace..."
colcon build --symlink-install || echo "[entrypoint] colcon build failed (non-fatal) -- continuing"

exec "$@"
