#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERRIDE_FILE="$ROOT_DIR/docker-compose.override.yml"

UNAME_S="$(uname -s || echo unknown)"

echo "Generating docker-compose.override.yml for host OS: $UNAME_S"

case "$UNAME_S" in
  Linux*)
    DISPLAY_VALUE="${DISPLAY:-:0}"
    cat > "$OVERRIDE_FILE" << YAML
version: "3.8"

services:
  ros2:
    network_mode: "host"
    pid: "host"
    ipc: "host"
    environment:
      - DISPLAY=$DISPLAY_VALUE
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - /dev/dri:/dev/dri:rw
YAML
    ;;
  Darwin*)
    cat > "$OVERRIDE_FILE" << 'YAML'
services:
  ros2: {}
YAML
    ;;
  MINGW*|MSYS*|CYGWIN*)
    cat > "$OVERRIDE_FILE" << 'YAML'
version: "3.8"

services:
  ros2:
    network_mode: "host"
    pid: "host"
    ipc: "host"
    environment:
      - DISPLAY=host.docker.internal:0
YAML
    ;;
  *)
    echo "Warning: unknown host OS '$UNAME_S', generating minimal override."
    cat > "$OVERRIDE_FILE" << 'YAML'
version: "3.8"

services:
  ros2: {}
YAML
    ;;

esac

echo "Wrote $OVERRIDE_FILE"
