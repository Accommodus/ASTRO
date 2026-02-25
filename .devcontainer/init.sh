#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="$ROOT_DIR/docker-compose.override.yml"

detect_host_os() {
  if grep -qi microsoft /proc/version 2>/dev/null; then
    echo "windows"
  else
    case "$(uname -s)" in
      Linux*)                 echo "linux"   ;;
      Darwin*)                echo "darwin"  ;;
      MINGW*|MSYS*|CYGWIN*)  echo "windows" ;;
      *)                      echo "darwin"  ;;
    esac
  fi
}

HOST_OS="$(detect_host_os)"

echo "[init] Detected host OS: $HOST_OS"
cp "$ROOT_DIR/docker-compose.$HOST_OS.yml" "$TARGET"
echo "[init] Wrote $TARGET"
