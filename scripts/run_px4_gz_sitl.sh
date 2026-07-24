#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/run_px4_gz_sitl.sh [options]

Options:
  --px4-dir PATH       Dedicated PX4 v1.17.0 checkout.
  --profile NAME       control, truth, or vision. Defaults to control.
  --model NAME         Gazebo model resource. Defaults to smartdrone_x500_stereo.
  --model-name NAME    Attach PX4 to an entity already present in the world.
  --spawn              Let PX4 spawn --model instead of attaching.
  --world NAME|PATH    World resource or SDF path. Defaults to smartdrone_indoor.
  --headless           Run server-side rendering without the Gazebo GUI.
  --gui                Start the Gazebo GUI (default).
  --standalone         Attach to an already-running Gazebo world.
  --bind PORT          SmartDrone receive port. Defaults to 14540.
  --px4-port PORT      PX4 local onboard port. Defaults to 14580.
  --partition NAME     Gazebo Transport partition. Defaults to smartdrone_sitl.
  --log-dir PATH       Run output directory. Defaults under output/sitl/.
  --skip-build         Require an existing px4_sitl_default build.
  -h, --help           Show this help.
EOF
}

fail() {
    echo "ERR $*" >&2
    exit 1
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

resolve_world() {
    local requested="$1"
    local candidate
    for candidate in "$requested" "$SIM_ROOT/worlds/${requested%.sdf}.sdf" \
        "$PX4_DIR/Tools/simulation/gz/worlds/${requested%.sdf}.sdf"; do
        if [ -f "$candidate" ]; then
            realpath "$candidate"
            return 0
        fi
    done
    return 1
}

read_world_name() {
    sed -n "s/.*<world[[:space:]]\+name=['\"]\([^'\"]*\)['\"].*/\1/p" "$1" | head -n 1
}

stop_process_group() {
    local process_id="$1"
    [ -n "$process_id" ] || return 0
    kill -0 "$process_id" >/dev/null 2>&1 || return 0
    kill -TERM -- "-$process_id" >/dev/null 2>&1 || kill -TERM "$process_id" >/dev/null 2>&1 || true
    local attempt
    for attempt in {1..30}; do
        kill -0 "$process_id" >/dev/null 2>&1 || return 0
        sleep 0.1
    done
    kill -KILL -- "-$process_id" >/dev/null 2>&1 || kill -KILL "$process_id" >/dev/null 2>&1 || true
}

cleanup() {
    trap - EXIT INT TERM
    stop_process_group "$PX4_PID"
    stop_process_group "$GZ_GUI_PID"
    if [ "$STANDALONE" -eq 0 ]; then
        stop_process_group "$GZ_SERVER_PID"
    fi
}

wait_for_world() {
    local service="/world/$WORLD_NAME/scene/info"
    local attempt
    for attempt in {1..60}; do
        if gz service -l 2>/dev/null | grep -Fxq "$service"; then
            return 0
        fi
        if [ -n "$GZ_SERVER_PID" ] && ! kill -0 "$GZ_SERVER_PID" >/dev/null 2>&1; then
            return 1
        fi
        sleep 0.5
    done
    return 1
}

write_run_environment() {
    local auto_mode pose_output require_vision
    case "$PROFILE" in
        control) auto_mode="idle"; pose_output="none"; require_vision=0 ;;
        truth) auto_mode="idle"; pose_output="position_velocity"; require_vision=0 ;;
        vision) auto_mode="slam"; pose_output="position_velocity"; require_vision=1 ;;
    esac
    {
        printf 'export GZ_PARTITION=%q\n' "$GZ_PARTITION"
        printf 'export GZ_IP=127.0.0.1\n'
        printf 'export SMART_DRONE_GZ_PARTITION=%q\n' "$GZ_PARTITION"
        printf 'export GZ_SIM_RESOURCE_PATH=%q\n' "$GZ_SIM_RESOURCE_PATH"
        printf 'export SMART_DRONE_MAVLINK_DEV=%q\n' "udp-listen://$SMART_DRONE_BIND_PORT"
        printf 'export SMART_DRONE_MAVLINK_BAUD=921600\n'
        printf 'export SMART_DRONE_SIM_PROFILE=%q\n' "$PROFILE"
        printf 'export SMART_DRONE_SIM_SEED=%q\n' "$SIM_SEED"
        printf 'export SMART_DRONE_SIM_CONFIG=%q\n' "$SIM_ROOT/config/smartdrone_sim.yaml"
        printf 'export SMART_DRONE_SIM_WORLD=%q\n' "$WORLD_NAME"
        printf 'export SMART_DRONE_SIM_MODEL=%q\n' "${MODEL_ENTITY_NAME:-${PX4_MODEL}_0}"
        printf 'export SMART_DRONE_SIM_LOG_DIR=%q\n' "$LOG_DIR"
        if [ -n "$SITL_RUN_ID" ]; then
            printf 'export SMART_DRONE_SITL_RUN_ID=%q\n' "$SITL_RUN_ID"
        fi
        printf 'export SMART_DRONE_AUTO_MODE=%q\n' "$auto_mode"
        printf 'export SMART_DRONE_PX4_POSE_OUTPUT_MODE=%q\n' "$pose_output"
        printf 'export SMART_DRONE_OFFBOARD_REQUIRES_VISION=%q\n' "$require_vision"
        printf 'export SMART_DRONE_VISUAL_POSE_MAX_AGE_MS=100\n'
        printf 'export SMART_DRONE_VISUAL_LOSS_LAND_MS=500\n'
        printf 'export SMART_DRONE_SIM_WATCHDOG_SCALE=%q\n' "${SMART_DRONE_SIM_WATCHDOG_SCALE:-1}"
    } >"$LOG_DIR/run.env"
}

write_px4_attestation() {
    if [ -z "$SITL_RUN_ID" ] && [ -z "$PX4_ATTESTATION_FILE" ]; then
        return 0
    fi
    [ -n "$SITL_RUN_ID" ] || fail "SMART_DRONE_SITL_RUN_ID is required for attestation"
    [ -n "$PX4_ATTESTATION_FILE" ] || \
        fail "SMART_DRONE_PX4_ATTESTATION_FILE is required for attestation"
    python3 "$SCRIPT_DIR/sitl_hover/provenance.py" attest \
        --output "$PX4_ATTESTATION_FILE" \
        --role px4 \
        --run-id "$SITL_RUN_ID" \
        --profile "$PROFILE" \
        --seed "$SIM_SEED" \
        --detail "world=$WORLD_NAME" \
        --detail "world_file=$WORLD_FILE" \
        --detail "model=${MODEL_ENTITY_NAME:-${PX4_MODEL}_0}" \
        --detail "px4_binary=$PX4_BIN" \
        --detail "partition=$GZ_PARTITION" \
        --detail "standalone=$STANDALONE" \
        --detail "headless=$HEADLESS" \
        --detail "log_dir=$LOG_DIR"
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SIM_ROOT="$REPO_ROOT/sim/px4_gz"
PX4_DIR=""
PX4_MODEL="smartdrone_x500_stereo"
MODEL_ENTITY_NAME=""
MODEL_NAME_SET=0
PX4_WORLD="smartdrone_indoor"
PROFILE="control"
HEADLESS="${HEADLESS:-0}"
STANDALONE="${PX4_GZ_STANDALONE:-0}"
SMART_DRONE_BIND_PORT="${SMART_DRONE_SITL_BIND_PORT:-14540}"
PX4_LOCAL_PORT="${SMART_DRONE_PX4_LOCAL_PORT:-14580}"
GZ_PARTITION="${SMART_DRONE_GZ_PARTITION:-${GZ_PARTITION:-smartdrone_sitl}}"
SIM_SEED="${SMART_DRONE_SIM_SEED:-1}"
LOG_DIR="${SMART_DRONE_PX4_LOG_DIR:-}"
SITL_RUN_ID="${SMART_DRONE_SITL_RUN_ID:-}"
PX4_ATTESTATION_FILE="${SMART_DRONE_PX4_ATTESTATION_FILE:-}"
SKIP_BUILD=0
PX4_PID=""
GZ_SERVER_PID=""
GZ_GUI_PID=""

while [ "$#" -gt 0 ]; do
    case "$1" in
        --px4-dir)
            [ "$#" -ge 2 ] || fail "--px4-dir requires a value"
            PX4_DIR="$2"
            shift
            ;;
        --px4-dir=*) PX4_DIR="${1#--px4-dir=}" ;;
        --profile)
            [ "$#" -ge 2 ] || fail "--profile requires a value"
            PROFILE="$2"
            shift
            ;;
        --profile=*) PROFILE="${1#--profile=}" ;;
        --model)
            [ "$#" -ge 2 ] || fail "--model requires a value"
            PX4_MODEL="$2"
            shift
            ;;
        --model=*) PX4_MODEL="${1#--model=}" ;;
        --model-name)
            [ "$#" -ge 2 ] || fail "--model-name requires a value"
            MODEL_ENTITY_NAME="$2"
            MODEL_NAME_SET=1
            shift
            ;;
        --model-name=*) MODEL_ENTITY_NAME="${1#--model-name=}"; MODEL_NAME_SET=1 ;;
        --spawn) MODEL_ENTITY_NAME=""; MODEL_NAME_SET=1 ;;
        --world)
            [ "$#" -ge 2 ] || fail "--world requires a value"
            PX4_WORLD="$2"
            shift
            ;;
        --world=*) PX4_WORLD="${1#--world=}" ;;
        --headless) HEADLESS=1 ;;
        --gui) HEADLESS=0 ;;
        --standalone) STANDALONE=1 ;;
        --bind)
            [ "$#" -ge 2 ] || fail "--bind requires a value"
            SMART_DRONE_BIND_PORT="$2"
            shift
            ;;
        --bind=*) SMART_DRONE_BIND_PORT="${1#--bind=}" ;;
        --px4-port)
            [ "$#" -ge 2 ] || fail "--px4-port requires a value"
            PX4_LOCAL_PORT="$2"
            shift
            ;;
        --px4-port=*) PX4_LOCAL_PORT="${1#--px4-port=}" ;;
        --partition)
            [ "$#" -ge 2 ] || fail "--partition requires a value"
            GZ_PARTITION="$2"
            shift
            ;;
        --partition=*) GZ_PARTITION="${1#--partition=}" ;;
        --log-dir)
            [ "$#" -ge 2 ] || fail "--log-dir requires a value"
            LOG_DIR="$2"
            shift
            ;;
        --log-dir=*) LOG_DIR="${1#--log-dir=}" ;;
        --skip-build) SKIP_BUILD=1 ;;
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
    *) fail "--profile must be control, truth, or vision" ;;
esac
[[ "$SIM_SEED" =~ ^[1-9][0-9]*$ ]] || \
    fail "SMART_DRONE_SIM_SEED must be a positive integer"

if [ -z "$PX4_DIR" ]; then
    PX4_DIR="$(find_px4_dir || true)"
fi
[ -n "$PX4_DIR" ] || fail "PX4 v1.17.0 checkout not found; run scripts/setup_px4_gz_sitl.sh"
PX4_DIR="$(realpath "$PX4_DIR")"

WORLD_FILE="$(resolve_world "$PX4_WORLD" || true)"
[ -n "$WORLD_FILE" ] || fail "Gazebo world not found: $PX4_WORLD"
WORLD_NAME="$(read_world_name "$WORLD_FILE")"
[ -n "$WORLD_NAME" ] || fail "unable to read <world name> from $WORLD_FILE"

if [ "$MODEL_NAME_SET" -eq 0 ] && [ "$PX4_MODEL" = "smartdrone_x500_stereo" ] && \
    [ "$WORLD_NAME" = "smartdrone_hover" ]; then
    MODEL_ENTITY_NAME="smartdrone_x500"
fi

"$SCRIPT_DIR/check_sitl_env.sh" --px4-dir "$PX4_DIR" --profile "$PROFILE" \
    --bind "$SMART_DRONE_BIND_PORT" --px4-port "$PX4_LOCAL_PORT" \
    --skip-smart-drone

PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_BIN="$PX4_BUILD_DIR/bin/px4"
if [ "$SKIP_BUILD" -eq 0 ]; then
    make -C "$PX4_DIR" px4_sitl_default
fi
[ -x "$PX4_BIN" ] || fail "PX4 SITL binary not found: $PX4_BIN"
[ -d "$PX4_BUILD_DIR/etc" ] || fail "PX4 generated etc directory not found"

if [ -z "$LOG_DIR" ]; then
    LOG_DIR="$REPO_ROOT/output/sitl/$(date +%Y%m%d_%H%M%S)_${PROFILE}_$$"
else
    LOG_DIR="$(realpath -m "$LOG_DIR")"
fi
[ ! -e "$LOG_DIR" ] || fail "log directory already exists: $LOG_DIR"
mkdir -p "$LOG_DIR/px4-rootfs"
cp -a "$PX4_BUILD_DIR/etc" "$LOG_DIR/px4-rootfs/etc"
cp "$SIM_ROOT/px4/airframes/4001_gz_x500" \
    "$LOG_DIR/px4-rootfs/etc/init.d-posix/airframes/4001_gz_x500"

export GZ_PARTITION
export GZ_IP="127.0.0.1"
export GZ_SIM_RESOURCE_PATH="$SIM_ROOT/models:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
if [ "$STANDALONE" -eq 0 ] && \
    timeout 2 gz service -l 2>/dev/null | grep -Eq '^/world/.*/scene/info$'; then
    fail "Gazebo partition '$GZ_PARTITION' already has a world; choose --partition or use --standalone"
fi
write_run_environment
trap cleanup EXIT
trap 'exit 130' INT TERM

if [ "$STANDALONE" -eq 0 ]; then
    gazebo_args=(sim --verbose=2 -r -s --seed "$SIM_SEED")
    if [ "$HEADLESS" -eq 1 ]; then
        gazebo_args+=(--headless-rendering)
    fi
    gazebo_args+=("$WORLD_FILE")
    setsid gz "${gazebo_args[@]}" >"$LOG_DIR/gazebo-server.log" 2>&1 &
    GZ_SERVER_PID=$!
fi

if ! wait_for_world; then
    [ -f "$LOG_DIR/gazebo-server.log" ] && tail -n 80 "$LOG_DIR/gazebo-server.log" >&2
    fail "Gazebo world '$WORLD_NAME' did not become ready"
fi

write_px4_attestation

if [ -n "${SMART_DRONE_PX4_READY_FILE:-}" ]; then
    printf 'ready\n' >"$SMART_DRONE_PX4_READY_FILE"
fi

if [ "$HEADLESS" -eq 0 ] && [ "$STANDALONE" -eq 0 ]; then
    setsid gz sim -g >"$LOG_DIR/gazebo-gui.log" 2>&1 &
    GZ_GUI_PID=$!
fi

PX4_PROFILE_FILE="$SIM_ROOT/px4/profiles/$PROFILE.sh"
PX4_RUN_ROOT="$LOG_DIR/px4-rootfs"
(
    cd "$PX4_RUN_ROOT"
    exec setsid env \
        PX4_SIM_MODEL="gz_$PX4_MODEL" \
        PX4_SYS_AUTOSTART=4001 \
        PX4_GZ_WORLD="$WORLD_NAME" \
        PX4_GZ_STANDALONE=1 \
        PX4_GZ_MODEL_NAME="$MODEL_ENTITY_NAME" \
        SMARTDRONE_PX4_BASE_AIRFRAME="$PX4_BUILD_DIR/etc/init.d-posix/airframes/4001_gz_x500" \
        SMARTDRONE_PX4_PROFILE_FILE="$PX4_PROFILE_FILE" \
        SMARTDRONE_MAVLINK_HOST=127.0.0.1 \
        SMARTDRONE_MAVLINK_REMOTE_PORT="$SMART_DRONE_BIND_PORT" \
        SMARTDRONE_MAVLINK_LOCAL_PORT="$PX4_LOCAL_PORT" \
        "$PX4_BIN" -d -s "$SIM_ROOT/px4/startup.sh" "$PX4_RUN_ROOT/etc"
) >"$LOG_DIR/px4.log" 2>&1 &
PX4_PID=$!

echo
echo "PX4_DIR:$PX4_DIR"
echo "PROFILE:$PROFILE"
echo "WORLD:$WORLD_NAME"
echo "MODEL:${MODEL_ENTITY_NAME:-$PX4_MODEL (spawn)}"
echo "GZ_PARTITION:$GZ_PARTITION"
echo "SIM_SEED:$SIM_SEED"
echo "LOG_DIR:$LOG_DIR"
echo "STEREO_LEFT_TOPIC:/smartdrone/stereo/left/image"
echo "STEREO_RIGHT_TOPIC:/smartdrone/stereo/right/image"
echo "GROUND_TRUTH_TOPIC:/world/$WORLD_NAME/dynamic_pose/info"
echo "GROUND_TRUTH_MODEL:${MODEL_ENTITY_NAME:-${PX4_MODEL}_0}"
echo
echo "SmartDrone environment: source '$LOG_DIR/run.env'"
echo 'SmartDrone args: --auto-mode "$SMART_DRONE_AUTO_MODE" --px4-pose-output-mode "$SMART_DRONE_PX4_POSE_OUTPUT_MODE"'
echo

tail -n +1 --pid="$PX4_PID" -F "$LOG_DIR/px4.log" &
TAIL_PID=$!
set +e
wait "$PX4_PID"
status=$?
set -e
wait "$TAIL_PID" >/dev/null 2>&1 || true
exit "$status"
