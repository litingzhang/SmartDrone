#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/upload.sh [--restart] [--platform <name>] [--adb-ip <ip> --adb-port <port>] [--apk <path>] [--adb-only]

Environment variables:
  TARGET_HOST        SSH host, default: ltz@192.168.0.105
  REMOTE_DIR         Remote deploy directory, default: /home/ltz
  REMOTE_SERVICE     systemd service name, default: smart_drone
  RESTART_SERVICE    1 to restart service after deploy, default: 0
  DEPLOY_PLATFORM    Artifact platform name under output/artifacts, default: cm5
EOF
}

RESTART_SERVICE="${RESTART_SERVICE:-0}"
ADB_IP="${ADB_IP:-}"
ADB_PORT="${ADB_PORT:-}"
ADB_ONLY="${ADB_ONLY:-0}"
DEPLOY_PLATFORM="${DEPLOY_PLATFORM:-cm5}"

while [ $# -gt 0 ]; do
    case "$1" in
        --restart)
            RESTART_SERVICE=1
            ;;
        --platform)
            if [ $# -lt 2 ]; then
                echo "Missing value for --platform" >&2
                usage
                exit 1
            fi
            DEPLOY_PLATFORM="$2"
            shift
            ;;
        --adb-ip)
            if [ $# -lt 2 ]; then
                echo "Missing value for --adb-ip" >&2
                usage
                exit 1
            fi
            ADB_IP="$2"
            shift
            ;;
        --adb-port)
            if [ $# -lt 2 ]; then
                echo "Missing value for --adb-port" >&2
                usage
                exit 1
            fi
            ADB_PORT="$2"
            shift
            ;;
        --apk)
            if [ $# -lt 2 ]; then
                echo "Missing value for --apk" >&2
                usage
                exit 1
            fi
            APK_PATH="$2"
            shift
            ;;
        --adb-only)
            ADB_ONLY=1
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
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TARGET_HOST="${TARGET_HOST:-ltz@192.168.0.105}"
REMOTE_DIR="${REMOTE_DIR:-/home/ltz}"
REMOTE_SERVICE="${REMOTE_SERVICE:-smart_drone}"
ARTIFACT_ROOT="$REPO_ROOT/output/artifacts/$DEPLOY_PLATFORM"
if [ -z "${APK_PATH:-}" ]; then
    if [ -f "$REPO_ROOT/output/artifacts/android/latest.apk" ]; then
        APK_PATH="$REPO_ROOT/output/artifacts/android/latest.apk"
    else
        APK_PATH="$REPO_ROOT/src/android/app/build/outputs/apk/debug/app-debug.apk"
    fi
fi

SMART_DRONE_BIN="$ARTIFACT_ROOT/bin/smart_drone"
ORB_SO="$ARTIFACT_ROOT/lib/libORB_SLAM3.so"
DBOW2_SO="$ARTIFACT_ROOT/lib/libDBoW2.so"
G2O_SO="$ARTIFACT_ROOT/lib/libg2o.so"
CALIB_YAML="$ARTIFACT_ROOT/config/stereo_inertial.yaml"
STEREO_YAML="$ARTIFACT_ROOT/config/stereo.yaml"
MONO_YAML="$ARTIFACT_ROOT/config/mono_right.yaml"
MONO_IMU_YAML="$ARTIFACT_ROOT/config/mono_inertial_right.yaml"

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

ensure_remote_dirs() {
    ssh "$TARGET_HOST" "mkdir -p '$REMOTE_DIR/config'"
}

if [ "$ADB_ONLY" != "1" ]; then
    require_file "$SMART_DRONE_BIN"
    require_file "$ORB_SO"
    require_file "$DBOW2_SO"
    require_file "$G2O_SO"
    require_file "$CALIB_YAML"
    require_file "$STEREO_YAML"
    require_file "$MONO_YAML"
    require_file "$MONO_IMU_YAML"

    ensure_remote_dirs

    upload_atomic "$SMART_DRONE_BIN" "smart_drone"
    upload_atomic "$ORB_SO" "libORB_SLAM3.so"
    upload_atomic "$DBOW2_SO" "libDBoW2.so"
    upload_atomic "$G2O_SO" "libg2o.so"
    upload_atomic "$CALIB_YAML" "config/stereo_inertial.yaml"
    upload_atomic "$STEREO_YAML" "config/stereo.yaml"
    upload_atomic "$MONO_YAML" "config/mono_right.yaml"
    upload_atomic "$MONO_IMU_YAML" "config/mono_inertial_right.yaml"

    if [ "$RESTART_SERVICE" = "1" ]; then
        echo "restart service $REMOTE_SERVICE"
        ssh "$TARGET_HOST" "sudo systemctl restart '$REMOTE_SERVICE'"
    fi
fi

if [ -n "$ADB_IP" ] || [ -n "$ADB_PORT" ]; then
    if [ -z "$ADB_IP" ] || [ -z "$ADB_PORT" ]; then
        echo "Both --adb-ip and --adb-port are required when enabling adb deploy." >&2
        exit 1
    fi
    require_file "$APK_PATH"
    if ! command -v adb >/dev/null 2>&1; then
        echo "adb command not found in PATH." >&2
        exit 1
    fi
    ADB_TARGET="$ADB_IP:$ADB_PORT"
    echo "adb connect $ADB_TARGET"
    adb connect "$ADB_TARGET"
    echo "adb install -r $APK_PATH"
    adb install -r "$APK_PATH"
fi

if [ "$ADB_ONLY" = "1" ]; then
    echo "deploy complete: adb-only"
else
    echo "deploy complete: $TARGET_HOST:$REMOTE_DIR"
fi
