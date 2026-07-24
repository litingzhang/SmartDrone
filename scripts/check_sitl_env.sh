#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/check_sitl_env.sh [options]

Options:
  --px4-dir PATH       PX4 v1.17.0 checkout. PX4_AUTOPILOT_DIR has priority.
  --bind PORT          SmartDrone receive port. Defaults to 14540.
  --px4-port PORT      PX4 local onboard port. Defaults to 14580.
  --profile NAME       control, truth, or vision. Defaults to control.
  --skip-smart-drone   Do not require the host SmartDrone binary.
  -h, --help           Show this help.
EOF
}

find_px4_dir() {
    local candidate
    for candidate in "${PX4_AUTOPILOT_DIR:-}" \
        "$REPO_ROOT/../px4/PX4-Autopilot-v1.17.0" \
        "$REPO_ROOT/../PX4-Autopilot-v1.17.0" \
        "$HOME/PX4-Autopilot-v1.17.0"; do
        if [ -n "$candidate" ] && [ -f "$candidate/Makefile" ]; then
            realpath "$candidate"
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

valid_port() {
    [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -ge 1 ] && [ "$1" -le 65535 ]
}

check_udp_port_free() {
    local label="$1"
    local port="$2"
    if command -v ss >/dev/null 2>&1 && \
        ss -H -lun "sport = :$port" | grep -q .; then
        echo "ERR $label udp/$port:already in use"
        return 1
    fi
    echo "OK  $label udp/$port:available"
    return 0
}

check_px4_revision() {
    local expected actual
    expected="$(git -C "$PX4_DIR" rev-list -n 1 v1.17.0 2>/dev/null || true)"
    actual="$(git -C "$PX4_DIR" rev-parse HEAD 2>/dev/null || true)"
    if [ -n "$expected" ] && [ "$actual" = "$expected" ]; then
        echo "OK  PX4 revision:v1.17.0 ($actual)"
        return 0
    fi
    echo "ERR PX4 revision:expected v1.17.0, got ${actual:-unknown}"
    return 1
}

check_gazebo_version() {
    local version
    version="$(gz sim --versions 2>/dev/null | head -n 1 | tr -d ' ')"
    if [[ "$version" =~ ^8([.]|$) ]]; then
        echo "OK  Gazebo:Harmonic gz-sim $version"
        return 0
    fi
    echo "ERR Gazebo:expected Harmonic gz-sim 8, got ${version:-unknown}"
    return 1
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_DIR=""
SMART_DRONE_BIND_PORT="${SMART_DRONE_SITL_BIND_PORT:-14540}"
PX4_LOCAL_PORT="${SMART_DRONE_PX4_LOCAL_PORT:-14580}"
PROFILE="control"
SKIP_SMART_DRONE=0
SMART_DRONE_BIN="$REPO_ROOT/output/artifacts/host/bin/smart_drone"
SIM_ROOT="$REPO_ROOT/sim/px4_gz"
missing=0

while [ "$#" -gt 0 ]; do
    case "$1" in
        --px4-dir)
            [ "$#" -ge 2 ] || { echo "--px4-dir requires a value" >&2; exit 1; }
            PX4_DIR="$2"
            shift
            ;;
        --px4-dir=*) PX4_DIR="${1#--px4-dir=}" ;;
        --bind)
            [ "$#" -ge 2 ] || { echo "--bind requires a value" >&2; exit 1; }
            SMART_DRONE_BIND_PORT="$2"
            shift
            ;;
        --bind=*) SMART_DRONE_BIND_PORT="${1#--bind=}" ;;
        --px4-port)
            [ "$#" -ge 2 ] || { echo "--px4-port requires a value" >&2; exit 1; }
            PX4_LOCAL_PORT="$2"
            shift
            ;;
        --px4-port=*) PX4_LOCAL_PORT="${1#--px4-port=}" ;;
        --profile)
            [ "$#" -ge 2 ] || { echo "--profile requires a value" >&2; exit 1; }
            PROFILE="$2"
            shift
            ;;
        --profile=*) PROFILE="${1#--profile=}" ;;
        --skip-smart-drone) SKIP_SMART_DRONE=1 ;;
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

case "$PROFILE" in
    control|truth|vision) ;;
    *) echo "ERR profile:must be control, truth, or vision"; missing=1 ;;
esac

valid_port "$SMART_DRONE_BIND_PORT" || { echo "ERR bind port:invalid"; missing=1; }
valid_port "$PX4_LOCAL_PORT" || { echo "ERR PX4 port:invalid"; missing=1; }
if [ "$SMART_DRONE_BIND_PORT" = "$PX4_LOCAL_PORT" ]; then
    echo "ERR UDP ports:SmartDrone and PX4 local ports must differ"
    missing=1
fi

if [ -z "$PX4_DIR" ]; then
    PX4_DIR="$(find_px4_dir || true)"
fi

if [ -n "$PX4_DIR" ] && [ -f "$PX4_DIR/Makefile" ]; then
    PX4_DIR="$(realpath "$PX4_DIR")"
    echo "OK  PX4_DIR:$PX4_DIR"
    check_px4_revision || missing=1
    if [ -f "$PX4_DIR/Tools/simulation/gz/models/x500/model.sdf" ]; then
        echo "OK  PX4 Gazebo models:initialized"
    else
        echo "ERR PX4 Gazebo models:submodule not initialized"
        missing=1
    fi
else
    echo "ERR PX4_DIR:v1.17.0 checkout not found"
    missing=1
fi

for tool in git make gz setsid tail timeout; do
    check_tool "$tool" || missing=1
done
if command -v gz >/dev/null 2>&1; then
    check_gazebo_version || missing=1
fi

for resource in \
    "$SIM_ROOT/models/smartdrone_indoor_assets/model.config" \
    "$SIM_ROOT/models/smartdrone_indoor_assets/model.sdf" \
    "$SIM_ROOT/models/smartdrone_indoor_assets/materials/textures/floor_features.png" \
    "$SIM_ROOT/models/smartdrone_indoor_assets/materials/textures/wall_features.png" \
    "$SIM_ROOT/models/smartdrone_x500_stereo/model.sdf" \
    "$SIM_ROOT/worlds/smartdrone_indoor.sdf" \
    "$SIM_ROOT/config/smartdrone_sim.yaml" \
    "$SIM_ROOT/config/stereo_rectified.yaml" \
    "$SIM_ROOT/px4/startup.sh" \
    "$SIM_ROOT/px4/airframes/4001_gz_x500" \
    "$SIM_ROOT/px4/profiles/$PROFILE.sh"; do
    if [ -f "$resource" ]; then
        echo "OK  resource:$resource"
    else
        echo "ERR resource:missing $resource"
        missing=1
    fi
done

if [ "$SKIP_SMART_DRONE" -eq 0 ]; then
    if [ -x "$SMART_DRONE_BIN" ]; then
        echo "OK  smart_drone:$SMART_DRONE_BIN"
    else
        echo "ERR smart_drone:not built"
        missing=1
    fi
fi

if valid_port "$SMART_DRONE_BIND_PORT"; then
    check_udp_port_free "SmartDrone" "$SMART_DRONE_BIND_PORT" || missing=1
fi
if valid_port "$PX4_LOCAL_PORT"; then
    check_udp_port_free "PX4" "$PX4_LOCAL_PORT" || missing=1
fi

if [ "$missing" -ne 0 ]; then
    echo
    echo "SITL environment is not ready."
    echo "Install the pinned environment with:"
    echo "  ./scripts/setup_px4_gz_sitl.sh"
    echo "Build SmartDrone with:"
    echo "  ./scripts/build.sh host-smart_drone --camera-provider gz_stereo"
    exit 1
fi

echo
echo "SITL environment is ready for profile '$PROFILE'."
