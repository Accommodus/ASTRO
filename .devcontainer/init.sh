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
TEMPLATE="$ROOT_DIR/docker-compose.$HOST_OS.yml"

echo "[init] Detected host OS: $HOST_OS"
echo "[init] Using template: $TEMPLATE"

# Write to a temp file we own, then atomically move it into place.
# mv (rename) in a world-writable, non-sticky directory succeeds for any user,
# regardless of who owns the existing target file.
TMPFILE="$(mktemp)"
trap 'rm -f "$TMPFILE"' EXIT

cat "$TEMPLATE" > "$TMPFILE"
mv -f "$TMPFILE" "$TARGET"

echo "[init] Wrote $TARGET"
