#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/check_sitl_env.sh [--px4-dir PATH] [--bind PORT]

Options:
  --px4-dir PATH   PX4-Autopilot checkout. Defaults to PX4_AUTOPILOT_DIR,
                   ../PX4-Autopilot, or ~/PX4-Autopilot.
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

check_tool() {
    local name="$1"
    if command -v "$name" >/dev/null 2>&1; then
        echo "OK  $name:$(command -v "$name")"
        return 0
    fi
    echo "ERR $name:not found"
    return 1
}

check_udp_port_free() {
    local port="$1"
    if command -v ss >/dev/null 2>&1 &&
        ss -H -lun "sport = :$port" | grep -q .; then
        echo "ERR udp/$port:already in use"
        return 1
    fi
    echo "OK  udp/$port:available"
    return 0
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_DIR=""
SMART_DRONE_BIND_PORT="${SMART_DRONE_SITL_BIND_PORT:-14540}"
SMART_DRONE_BIN="$REPO_ROOT/output/artifacts/host/bin/smart_drone"
missing=0

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

if [ -n "$PX4_DIR" ] && [ -f "$PX4_DIR/Makefile" ]; then
    echo "OK  PX4_DIR:$PX4_DIR"
else
    echo "ERR PX4_DIR:not found"
    missing=1
fi

for tool in make gz; do
    if ! check_tool "$tool"; then
        missing=1
    fi
done

if [ -x "$SMART_DRONE_BIN" ]; then
    echo "OK  smart_drone:$SMART_DRONE_BIN"
else
    echo "ERR smart_drone:not built"
    missing=1
fi

if ! check_udp_port_free "$SMART_DRONE_BIND_PORT"; then
    missing=1
fi

if [ "$missing" -ne 0 ]; then
    echo
    echo "SITL environment is not ready."
    echo "Build SmartDrone with:"
    echo "  ./scripts/build.sh host-smart_drone --camera-provider uvc_stereo_opencv"
    echo "Install PX4/Gazebo with:"
    echo "  git clone https://github.com/PX4/PX4-Autopilot.git ../PX4-Autopilot"
    echo "  cd ../PX4-Autopilot && bash ./Tools/setup/ubuntu.sh"
    exit 1
fi

echo
echo "SITL environment is ready."
