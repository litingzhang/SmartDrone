#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/run_px4_gz_sitl.sh [--px4-dir PATH] [--model NAME] [--headless]
                               [--standalone] [--world NAME] [--bind PORT]

Options:
  --px4-dir PATH   PX4-Autopilot checkout. Defaults to PX4_AUTOPILOT_DIR,
                   ../PX4-Autopilot, or ~/PX4-Autopilot.
  --model NAME     PX4 Gazebo model. Defaults to x500.
  --headless       Start Gazebo without the GUI.
  --standalone     Use PX4_GZ_STANDALONE=1 for an already-running Gazebo server.
  --world NAME     PX4_GZ_WORLD value passed to PX4.
  --bind PORT      Local UDP port SmartDrone should listen on. Defaults to 14540.
EOF
}

find_px4_dir() {
    local candidate
    for candidate in "${PX4_AUTOPILOT_DIR:-}" \
        "$REPO_ROOT/../PX4-Autopilot" \
        "$HOME/PX4-Autopilot"; do
        if [ -n "$candidate" ] && [ -f "$candidate/Makefile" ]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
    return 1
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_DIR=""
PX4_MODEL="${PX4_GZ_MODEL:-x500}"
HEADLESS="${HEADLESS:-0}"
STANDALONE="${PX4_GZ_STANDALONE:-0}"
PX4_WORLD="${PX4_GZ_WORLD:-}"
SMART_DRONE_BIND_PORT="${SMART_DRONE_SITL_BIND_PORT:-14540}"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --px4-dir)
            if [ "$#" -lt 2 ]; then
                echo "--px4-dir requires a value" >&2
                usage
                exit 1
            fi
            PX4_DIR="$2"
            shift
            ;;
        --px4-dir=*)
            PX4_DIR="${1#--px4-dir=}"
            ;;
        --model)
            if [ "$#" -lt 2 ]; then
                echo "--model requires a value" >&2
                usage
                exit 1
            fi
            PX4_MODEL="$2"
            shift
            ;;
        --model=*)
            PX4_MODEL="${1#--model=}"
            ;;
        --headless)
            HEADLESS=1
            ;;
        --standalone)
            STANDALONE=1
            ;;
        --world)
            if [ "$#" -lt 2 ]; then
                echo "--world requires a value" >&2
                usage
                exit 1
            fi
            PX4_WORLD="$2"
            shift
            ;;
        --world=*)
            PX4_WORLD="${1#--world=}"
            ;;
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

if [ -z "$PX4_DIR" ]; then
    PX4_DIR="$(find_px4_dir || true)"
fi

if [ -z "$PX4_DIR" ] || [ ! -f "$PX4_DIR/Makefile" ]; then
    echo "PX4-Autopilot checkout not found." >&2
    echo "Set PX4_AUTOPILOT_DIR or pass --px4-dir /path/to/PX4-Autopilot." >&2
    exit 1
fi

echo "PX4_DIR:$PX4_DIR"
echo "PX4_GZ_MODEL:$PX4_MODEL"
echo "HEADLESS:$HEADLESS"
echo "PX4_GZ_STANDALONE:$STANDALONE"
if [ -n "$PX4_WORLD" ]; then
    echo "PX4_GZ_WORLD:$PX4_WORLD"
fi
echo
echo "SmartDrone MAVLink endpoint:"
echo "  export SMART_DRONE_MAVLINK_DEV=udp-listen://$SMART_DRONE_BIND_PORT"
echo "  export SMART_DRONE_MAVLINK_BAUD=921600"
echo

(
    cd "$PX4_DIR"
    export HEADLESS
    export PX4_GZ_MODEL="$PX4_MODEL"
    export PX4_GZ_STANDALONE="$STANDALONE"
    if [ -n "$PX4_WORLD" ]; then
        export PX4_GZ_WORLD="$PX4_WORLD"
    fi
    make px4_sitl "gz_$PX4_MODEL"
)
