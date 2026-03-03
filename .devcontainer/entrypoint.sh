#!/usr/bin/env bash

# The container MUST reach `exec "$@"` so the IDE can attach.
# Keep every step best-effort where possible.

SENTINEL="/home/ws/.devcontainer/.setup_done"

# Source ROS early so ROS_DISTRO and other vars are available for rosdep.
source "/opt/ros/${ROS_DISTRO:-kilted}/setup.bash" 2>/dev/null || true

if [ ! -f "$SENTINEL" ]; then
    echo "[entrypoint] First-run setup..."
    sudo chown -R "$(whoami)" /home/ws                                  2>&1 || true
    sudo rm -f /home/ws/.git/*.lock /home/ws/.git/refs/heads/*.lock          || true
    rosdep update                                                       2>&1 || true
    sudo -E rosdep install --from-paths src --ignore-src -y             2>&1 || true
    mkdir -p "$(dirname "$SENTINEL")" && touch "$SENTINEL"                   || true
fi

echo "[entrypoint] Building ROS 2 workspace..."
colcon build --symlink-install 2>&1 || echo "[entrypoint] colcon build failed (non-fatal)"

exec "$@"
