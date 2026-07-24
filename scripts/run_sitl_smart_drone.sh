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
SITL_RUN_ID="${SMART_DRONE_SITL_RUN_ID:-}"
SMART_DRONE_ATTESTATION_FILE="${SMART_DRONE_ATTESTATION_FILE:-}"

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
    echo "Build it first with: ./scripts/build.sh host-smart_drone --camera-provider gz_stereo" >&2
    exit 1
fi

if [ -z "$MAVLINK_DEV" ]; then
    MAVLINK_DEV="udp-listen://$MAVLINK_BIND_PORT"
fi

export SMART_DRONE_MAVLINK_DEV="$MAVLINK_DEV"
export SMART_DRONE_MAVLINK_BAUD="${SMART_DRONE_MAVLINK_BAUD:-921600}"

AUTO_MODE="${SMART_DRONE_AUTO_MODE:-idle}"
POSE_OUTPUT_MODE="${SMART_DRONE_PX4_POSE_OUTPUT_MODE:-position_velocity}"
for ((index = 0; index < ${#ARGS[@]}; index++)); do
    case "${ARGS[$index]}" in
        --auto-mode)
            if ((index + 1 < ${#ARGS[@]})); then
                AUTO_MODE="${ARGS[$((index + 1))]}"
            fi
            ;;
        --auto-mode=*) AUTO_MODE="${ARGS[$index]#--auto-mode=}" ;;
        --px4-pose-output-mode)
            if ((index + 1 < ${#ARGS[@]})); then
                POSE_OUTPUT_MODE="${ARGS[$((index + 1))]}"
            fi
            ;;
        --px4-pose-output-mode=*)
            POSE_OUTPUT_MODE="${ARGS[$index]#--px4-pose-output-mode=}"
            ;;
    esac
done

if [ -n "$SITL_RUN_ID" ] || [ -n "$SMART_DRONE_ATTESTATION_FILE" ]; then
    [ -n "$SITL_RUN_ID" ] || {
        echo "SMART_DRONE_SITL_RUN_ID is required for attestation" >&2
        exit 1
    }
    [ -n "$SMART_DRONE_ATTESTATION_FILE" ] || {
        echo "SMART_DRONE_ATTESTATION_FILE is required for attestation" >&2
        exit 1
    }
    python3 "$SCRIPT_DIR/sitl_hover/provenance.py" attest \
        --output "$SMART_DRONE_ATTESTATION_FILE" \
        --role smart_drone \
        --run-id "$SITL_RUN_ID" \
        --profile "${SMART_DRONE_SIM_PROFILE:-}" \
        --seed "${SMART_DRONE_SIM_SEED:-0}" \
        --detail "binary=$SMART_DRONE_BIN" \
        --detail "mavlink_dev=$SMART_DRONE_MAVLINK_DEV" \
        --detail "auto_mode=$AUTO_MODE" \
        --detail "pose_output_mode=$POSE_OUTPUT_MODE" \
        --detail "requires_vision=${SMART_DRONE_OFFBOARD_REQUIRES_VISION:-}" \
        --detail "sim_config=${SMART_DRONE_SIM_CONFIG:-}"
fi

echo "SMART_DRONE_MAVLINK_DEV:$SMART_DRONE_MAVLINK_DEV"
echo "SMART_DRONE_MAVLINK_BAUD:$SMART_DRONE_MAVLINK_BAUD"
echo "SMART_DRONE_BIN:$SMART_DRONE_BIN"

exec "$SMART_DRONE_BIN" "${ARGS[@]}"
