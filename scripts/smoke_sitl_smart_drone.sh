#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/smoke_sitl_smart_drone.sh [--bind PORT] [--timeout SECONDS]

Options:
  --bind PORT         Local UDP port SmartDrone should listen on. Defaults to 14540.
  --timeout SECONDS   Runtime smoke duration. Defaults to 5.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SMART_DRONE_BIND_PORT="${SMART_DRONE_SITL_BIND_PORT:-14540}"
SMOKE_TIMEOUT=5
LOG_FILE=""

while [ "$#" -gt 0 ]; do
    case "$1" in
        --bind)
            if [ "$#" -lt 2 ]; then
                echo "--bind requires a value" >&2
                usage
                exit 1
            fi
            SMART_DRONE_BIND_PORT="$2"
            shift
            ;;
        --bind=*)
            SMART_DRONE_BIND_PORT="${1#--bind=}"
            ;;
        --timeout)
            if [ "$#" -lt 2 ]; then
                echo "--timeout requires a value" >&2
                usage
                exit 1
            fi
            SMOKE_TIMEOUT="$2"
            shift
            ;;
        --timeout=*)
            SMOKE_TIMEOUT="${1#--timeout=}"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
    esac
    shift
done

LOG_FILE="$(mktemp "${TMPDIR:-/tmp}/smart_drone_sitl_smoke.XXXXXX.log")"
trap 'rm -f "$LOG_FILE"' EXIT

echo "build host smart_drone"
"$REPO_ROOT/scripts/build.sh" host-smart_drone --camera-provider uvc_stereo_opencv

set +e
timeout "$SMOKE_TIMEOUT"s "$REPO_ROOT/scripts/run_sitl_smart_drone.sh" \
    --bind "$SMART_DRONE_BIND_PORT" -- --auto-mode idle >"$LOG_FILE" 2>&1
status=$?
set -e

if [ "$status" -ne 124 ]; then
    cat "$LOG_FILE"
    echo "SITL SmartDrone smoke failed: runtime exited with status $status" >&2
    exit 1
fi

if ! grep -q "\\[discovery\\] broadcasting" "$LOG_FILE"; then
    cat "$LOG_FILE"
    echo "SITL SmartDrone smoke failed: discovery did not start" >&2
    exit 1
fi

cat "$LOG_FILE"
echo
echo "SITL SmartDrone smoke passed."
