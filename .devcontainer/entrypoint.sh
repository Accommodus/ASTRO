#!/usr/bin/env bash

# All setup is best-effort. The container MUST reach `exec "$@"` (sleep infinity)
# so VS Code / Cursor can attach. No step here should be able to kill the container.

SENTINEL="/home/ws/.devcontainer/.setup_done"

if [ ! -f "$SENTINEL" ]; then
    echo "[entrypoint] First-run setup..."
    sudo chown -R "$(whoami)" /home/ws                             2>&1 || true
    sudo rm -f /home/ws/.git/*.lock /home/ws/.git/refs/heads/*.lock     || true
    sudo rosdep update                                             2>&1 || true
    sudo rosdep install --from-paths src --ignore-src -y           2>&1 || true
    mkdir -p "$(dirname "$SENTINEL")" && touch "$SENTINEL"              || true
fi

# ROS setup scripts reference uninitialized vars; source without strict mode
source "/opt/ros/${ROS_DISTRO:-kilted}/setup.bash" 2>/dev/null || true

echo "[entrypoint] Building ROS 2 workspace..."
colcon build --symlink-install 2>&1 || echo "[entrypoint] colcon build failed (non-fatal)"

exec "$@"
