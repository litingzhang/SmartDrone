#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/run_sitl_smart_drone.sh [--bind PORT] [--dev ENDPOINT] [--] [smart_drone args...]

Options:
  --bind PORT     Use udp-listen://PORT as the MAVLink endpoint. Defaults to 14540.
  --dev ENDPOINT  Full SMART_DRONE_MAVLINK_DEV value. Overrides --bind.

The host smart_drone binary is expected at:
  output/artifacts/host/bin/smart_drone
EOF
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SMART_DRONE_BIN="$REPO_ROOT/output/artifacts/host/bin/smart_drone"
MAVLINK_DEV="${SMART_DRONE_MAVLINK_DEV:-}"
MAVLINK_BIND_PORT="${SMART_DRONE_SITL_BIND_PORT:-14540}"
ARGS=()

while [ "$#" -gt 0 ]; do
    case "$1" in
        --bind)
            if [ "$#" -lt 2 ]; then
                echo "--bind requires a value" >&2
                usage
                exit 1
            fi
            MAVLINK_BIND_PORT="$2"
            shift
            ;;
        --bind=*)
            MAVLINK_BIND_PORT="${1#--bind=}"
            ;;
        --dev)
            if [ "$#" -lt 2 ]; then
                echo "--dev requires a value" >&2
                usage
                exit 1
            fi
            MAVLINK_DEV="$2"
            shift
            ;;
        --dev=*)
            MAVLINK_DEV="${1#--dev=}"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            ARGS+=("$@")
            break
            ;;
        *)
            ARGS+=("$1")
            ;;
    esac
    shift
done

if [ ! -x "$SMART_DRONE_BIN" ]; then
    echo "Host smart_drone binary not found: $SMART_DRONE_BIN" >&2
    echo "Build it first with: ./scripts/build.sh host-smart_drone --camera-provider uvc_stereo_opencv" >&2
    exit 1
fi

if [ -z "$MAVLINK_DEV" ]; then
    MAVLINK_DEV="udp-listen://$MAVLINK_BIND_PORT"
fi

export SMART_DRONE_MAVLINK_DEV="$MAVLINK_DEV"
export SMART_DRONE_MAVLINK_BAUD="${SMART_DRONE_MAVLINK_BAUD:-921600}"

echo "SMART_DRONE_MAVLINK_DEV:$SMART_DRONE_MAVLINK_DEV"
echo "SMART_DRONE_MAVLINK_BAUD:$SMART_DRONE_MAVLINK_BAUD"
echo "SMART_DRONE_BIN:$SMART_DRONE_BIN"

exec "$SMART_DRONE_BIN" "${ARGS[@]}"
