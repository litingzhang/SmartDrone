#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./upload.sh [--restart]

Environment variables:
  TARGET_HOST        SSH host, default: ltz@192.168.0.105
  REMOTE_DIR         Remote deploy directory, default: /home/ltz
  REMOTE_SERVICE     systemd service name, default: smart_drone
  RESTART_SERVICE    1 to restart service after deploy, default: 0
EOF
}

RESTART_SERVICE="${RESTART_SERVICE:-0}"

while [ $# -gt 0 ]; do
    case "$1" in
        --restart)
            RESTART_SERVICE=1
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage
            exit 1
            ;;
    esac
    shift
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TARGET_HOST="${TARGET_HOST:-ltz@192.168.0.105}"
REMOTE_DIR="${REMOTE_DIR:-/home/ltz}"
REMOTE_SERVICE="${REMOTE_SERVICE:-smart_drone}"

SMART_DRONE_BIN="$SCRIPT_DIR/build/cmake/smart_drone"
ORB_SO="$SCRIPT_DIR/ORB_SLAM3/lib/libORB_SLAM3.so"
DBOW2_SO="$SCRIPT_DIR/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so"
G2O_SO="$SCRIPT_DIR/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so"
CALIB_YAML="$SCRIPT_DIR/src/smart_drone/stereo_inertial.yaml"

require_file() {
    if [ ! -f "$1" ]; then
        echo "Required file not found: $1" >&2
        exit 1
    fi
}

upload_atomic() {
    local local_path="$1"
    local remote_name="$2"
    local remote_tmp="$REMOTE_DIR/$remote_name.new"
    local remote_dst="$REMOTE_DIR/$remote_name"

    echo "upload $remote_name"
    scp "$local_path" "$TARGET_HOST:$remote_tmp"
    ssh "$TARGET_HOST" "mv '$remote_tmp' '$remote_dst'"
}

require_file "$SMART_DRONE_BIN"
require_file "$ORB_SO"
require_file "$DBOW2_SO"
require_file "$G2O_SO"
require_file "$CALIB_YAML"

upload_atomic "$SMART_DRONE_BIN" "smart_drone"
upload_atomic "$ORB_SO" "libORB_SLAM3.so"
upload_atomic "$DBOW2_SO" "libDBoW2.so"
upload_atomic "$G2O_SO" "libg2o.so"
upload_atomic "$CALIB_YAML" "stereo_inertial.yaml"

if [ "$RESTART_SERVICE" = "1" ]; then
    echo "restart service $REMOTE_SERVICE"
    ssh "$TARGET_HOST" "sudo systemctl restart '$REMOTE_SERVICE'"
fi

echo "deploy complete: $TARGET_HOST:$REMOTE_DIR"
