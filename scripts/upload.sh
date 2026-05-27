#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/upload.sh [--restart] [--platform <name>|--jetson-orin-nx] [--adb-ip <ip> --adb-port <port>] [--apk <path>] [--adb-only]

Environment variables:
  TARGET_HOST        SSH host, default: ltz@192.168.0.105
  REMOTE_DIR         Remote deploy directory, default: /home/ltz
  REMOTE_SERVICE     systemd service name, default: smart_drone
  SYSTEMD_UNIT_FILE  Optional local systemd unit file to install remotely.
                     Default: <repo>/smart_drone.service
  EUROC_EVAL_ROOT    Jetson EuRoC eval deploy directory.
                     Default: <REMOTE_DIR>/euroc_eval
  CLEAN_LEGACY_DIRS  1 to remove legacy Jetson project directories after
                     deployment. Default: 0
  RESTART_SERVICE    1 to restart service after deploy, default: 0
  DEPLOY_PLATFORM    Artifact platform name under output/artifacts, default: cm5
  UPLOAD_LAYOUT      flat or artifact-root. Defaults to artifact-root for
                     jetson-orin-nx and flat for other platforms.
  SSH_PASSWORD       Optional. When set, upload.sh uses sshpass for ssh/scp.
EOF
}

RESTART_SERVICE="${RESTART_SERVICE:-0}"
ADB_IP="${ADB_IP:-}"
ADB_PORT="${ADB_PORT:-}"
ADB_ONLY="${ADB_ONLY:-0}"
DEPLOY_PLATFORM="${DEPLOY_PLATFORM:-cm5}"
UPLOAD_LAYOUT="${UPLOAD_LAYOUT:-}"
CLEAN_LEGACY_DIRS="${CLEAN_LEGACY_DIRS:-0}"
JETSON_ORIN_NX=0

while [ $# -gt 0 ]; do
    case "$1" in
        --restart)
            RESTART_SERVICE=1
            ;;
        --jetson-orin-nx)
            JETSON_ORIN_NX=1
            DEPLOY_PLATFORM="jetson-orin-nx"
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
if [ "$JETSON_ORIN_NX" = "1" ] || [ "$DEPLOY_PLATFORM" = "jetson-orin-nx" ]; then
    TARGET_HOST="${TARGET_HOST:-nvidia@192.168.0.103}"
    REMOTE_DIR="${REMOTE_DIR:-/home/nvidia}"
else
    TARGET_HOST="${TARGET_HOST:-ltz@192.168.0.105}"
    REMOTE_DIR="${REMOTE_DIR:-/home/ltz}"
fi
REMOTE_SERVICE="${REMOTE_SERVICE:-smart_drone}"
SYSTEMD_UNIT_FILE="${SYSTEMD_UNIT_FILE:-$REPO_ROOT/smart_drone.service}"
ARTIFACT_ROOT="$REPO_ROOT/output/artifacts/$DEPLOY_PLATFORM"
if [ -z "$UPLOAD_LAYOUT" ]; then
    if [ "$DEPLOY_PLATFORM" = "jetson-orin-nx" ]; then
        UPLOAD_LAYOUT="flat"
    else
        UPLOAD_LAYOUT="flat"
    fi
fi
if [ -z "${APK_PATH:-}" ]; then
    APK_ARTIFACT="$REPO_ROOT/output/artifacts/android/latest.apk"
    APK_GRADLE="$REPO_ROOT/src/android/app/build/outputs/apk/debug/app-debug.apk"
    if [ -f "$APK_ARTIFACT" ] && [ -f "$APK_GRADLE" ]; then
        if [ "$APK_GRADLE" -nt "$APK_ARTIFACT" ]; then
            APK_PATH="$APK_GRADLE"
        else
            APK_PATH="$APK_ARTIFACT"
        fi
    elif [ -f "$APK_ARTIFACT" ]; then
        APK_PATH="$APK_ARTIFACT"
    else
        APK_PATH="$APK_GRADLE"
    fi
fi

SMART_DRONE_BIN="$ARTIFACT_ROOT/bin/smart_drone"
# Legacy ORB-SLAM3 libraries are uploaded only when an ORB-enabled artifact bundle contains them.
ORB_SO="$ARTIFACT_ROOT/lib/libORB_SLAM3.so"
DBOW2_SO="$ARTIFACT_ROOT/lib/libDBoW2.so"
G2O_SO="$ARTIFACT_ROOT/lib/libg2o.so"
CALIB_YAML="$ARTIFACT_ROOT/config/stereo_inertial.yaml"
STEREO_YAML="$ARTIFACT_ROOT/config/stereo.yaml"
MONO_YAML="$ARTIFACT_ROOT/config/mono_right.yaml"
MONO_IMU_YAML="$ARTIFACT_ROOT/config/mono_inertial_right.yaml"
RUNTIME_GRAPH_CONFIG_DIR="$ARTIFACT_ROOT/config/runtime_graph"
EPG_CONFIG_DIR="$ARTIFACT_ROOT/config/epg"
REPLAY_ARTIFACT_DIR="$ARTIFACT_ROOT/offline-replay"
EUROC_EVAL_ROOT="${EUROC_EVAL_ROOT:-$REMOTE_DIR/euroc_eval}"
SSH_PASSWORD="${SSH_PASSWORD:-}"
SSH_CMD=(ssh)
SCP_CMD=(scp)

if [ -n "$SSH_PASSWORD" ]; then
    if ! command -v sshpass >/dev/null 2>&1; then
        echo "SSH_PASSWORD is set but sshpass is not installed." >&2
        exit 1
    fi
    SSH_CMD=(sshpass -p "$SSH_PASSWORD" ssh)
    SCP_CMD=(sshpass -p "$SSH_PASSWORD" scp)
fi

require_file() {
    if [ ! -f "$1" ]; then
        echo "Required file not found: $1" >&2
        exit 1
    fi
}

remote_service_unit_name() {
    if [[ "$REMOTE_SERVICE" == *.service ]]; then
        printf '%s\n' "$REMOTE_SERVICE"
    else
        printf '%s.service\n' "$REMOTE_SERVICE"
    fi
}

run_remote_sudo() {
    local remote_cmd="$1"
    if [ -n "$SSH_PASSWORD" ]; then
        "${SSH_CMD[@]}" "$TARGET_HOST" "printf '%s\n' '$SSH_PASSWORD' | sudo -S bash -lc \"$remote_cmd\""
    else
        "${SSH_CMD[@]}" "$TARGET_HOST" "sudo bash -lc \"$remote_cmd\""
    fi
}

upload_atomic() {
    local local_path="$1"
    local remote_name="$2"
    local remote_tmp="$REMOTE_DIR/$remote_name.new"
    local remote_dst="$REMOTE_DIR/$remote_name"

    echo "upload $remote_name"
    "${SCP_CMD[@]}" "$local_path" "$TARGET_HOST:$remote_tmp"
    "${SSH_CMD[@]}" "$TARGET_HOST" "mv '$remote_tmp' '$remote_dst'"
}

ensure_remote_dirs() {
    "${SSH_CMD[@]}" "$TARGET_HOST" "mkdir -p '$REMOTE_DIR/config' '$REMOTE_DIR/scripts'"
}

upload_dir_atomic() {
    local local_dir="$1"
    local remote_name="$2"
    local remote_tmp="$REMOTE_DIR/.${remote_name}.new"
    local remote_dst="$REMOTE_DIR/$remote_name"

    echo "upload $remote_name/"
    tar -C "$local_dir" -cf - . | "${SSH_CMD[@]}" "$TARGET_HOST" \
        "rm -rf '$remote_tmp' && mkdir -p '$remote_tmp' && tar -xf - -C '$remote_tmp' && rm -rf '$remote_dst' && mv '$remote_tmp' '$remote_dst'"
}

upload_absolute_atomic() {
    local local_path="$1"
    local remote_dst="$2"
    local remote_dir
    local remote_tmp

    remote_dir="$(dirname "$remote_dst")"
    remote_tmp="$remote_dst.new"

    echo "upload $remote_dst"
    "${SSH_CMD[@]}" "$TARGET_HOST" "mkdir -p '$remote_dir'"
    "${SCP_CMD[@]}" "$local_path" "$TARGET_HOST:$remote_tmp"
    "${SSH_CMD[@]}" "$TARGET_HOST" "mv '$remote_tmp' '$remote_dst'"
}

upload_artifact_root() {
    local remote_parent
    local remote_name
    local remote_tmp

    remote_parent="$(dirname "$REMOTE_DIR")"
    remote_name="$(basename "$REMOTE_DIR")"
    remote_tmp="$remote_parent/.${remote_name}.new"

    echo "upload artifact-root -> $TARGET_HOST:$REMOTE_DIR"
    tar -C "$ARTIFACT_ROOT" -cf - . | "${SSH_CMD[@]}" "$TARGET_HOST" \
        "rm -rf '$remote_tmp' && mkdir -p '$remote_tmp' && tar -xf - -C '$remote_tmp' && rm -rf '$REMOTE_DIR' && mv '$remote_tmp' '$REMOTE_DIR'"
}

install_systemd_unit() {
    local unit_name
    local remote_tmp

    unit_name="$(remote_service_unit_name)"
    remote_tmp="/tmp/${unit_name}.new"

    if [ ! -f "$SYSTEMD_UNIT_FILE" ]; then
        return 0
    fi

    echo "install systemd unit $unit_name"
    "${SCP_CMD[@]}" "$SYSTEMD_UNIT_FILE" "$TARGET_HOST:$remote_tmp"
    run_remote_sudo "mv '$remote_tmp' '/etc/systemd/system/$unit_name' && chmod 0644 '/etc/systemd/system/$unit_name' && systemctl daemon-reload"
}

cleanup_legacy_dirs() {
    if [ "$CLEAN_LEGACY_DIRS" != "1" ]; then
        return 0
    fi
    if [ "$DEPLOY_PLATFORM" != "jetson-orin-nx" ]; then
        return 0
    fi

    echo "remove legacy Jetson project directories"
    run_remote_sudo "rm -rf /home/nvidia/SmartDrone_cross /home/nvidia/SmartDrone_codex /home/nvidia/smart_drone.service.codex"
}

upload_jetson_euroc_eval_bundle() {
    local replay_bin="$REPLAY_ARTIFACT_DIR/smart_drone_offline_replay"
    local replay_epg="$REPLAY_ARTIFACT_DIR/config/epg/epg_topology.dot"

    if [ "$DEPLOY_PLATFORM" != "jetson-orin-nx" ]; then
        return 0
    fi
    if [ ! -f "$replay_bin" ]; then
        return 0
    fi

    echo "upload Jetson EuRoC eval bundle -> $TARGET_HOST:$EUROC_EVAL_ROOT"
    upload_absolute_atomic "$replay_bin" \
        "$EUROC_EVAL_ROOT/bin/smart_drone_offline_replay"
    upload_absolute_atomic "$SCRIPT_DIR/run_jetson_mh04_openvins_accuracy_perf.sh" \
        "$EUROC_EVAL_ROOT/scripts/run_jetson_mh04_openvins_accuracy_perf.sh"
    upload_absolute_atomic "$REPO_ROOT/tests/euroc/evaluate_euroc_regression.py" \
        "$EUROC_EVAL_ROOT/tests/euroc/evaluate_euroc_regression.py"
    if [ -f "$replay_epg" ]; then
        upload_absolute_atomic "$replay_epg" \
            "$EUROC_EVAL_ROOT/config/epg/epg_topology.dot"
    fi
    if [ -d "$REPO_ROOT/config/openvins" ]; then
        upload_dir_atomic "$REPO_ROOT/config/openvins" \
            "euroc_eval/config/openvins"
    fi
    "${SSH_CMD[@]}" "$TARGET_HOST" \
        "chmod +x '$EUROC_EVAL_ROOT/bin/smart_drone_offline_replay' '$EUROC_EVAL_ROOT/scripts/run_jetson_mh04_openvins_accuracy_perf.sh' '$EUROC_EVAL_ROOT/tests/euroc/evaluate_euroc_regression.py'"
}

if [ "$ADB_ONLY" != "1" ]; then
    require_file "$SMART_DRONE_BIN"
    require_file "$CALIB_YAML"
    require_file "$STEREO_YAML"
    require_file "$MONO_YAML"
    require_file "$MONO_IMU_YAML"

    if [ "$UPLOAD_LAYOUT" = "artifact-root" ]; then
        upload_artifact_root
    else
        ensure_remote_dirs

        upload_atomic "$SMART_DRONE_BIN" "smart_drone"
        if [ -f "$ORB_SO" ]; then
            upload_atomic "$ORB_SO" "libORB_SLAM3.so"
        fi
        if [ -f "$DBOW2_SO" ]; then
            upload_atomic "$DBOW2_SO" "libDBoW2.so"
        fi
        if [ -f "$G2O_SO" ]; then
            upload_atomic "$G2O_SO" "libg2o.so"
        fi
        upload_atomic "$CALIB_YAML" "config/stereo_inertial.yaml"
        upload_atomic "$STEREO_YAML" "config/stereo.yaml"
        upload_atomic "$MONO_YAML" "config/mono_right.yaml"
        upload_atomic "$MONO_IMU_YAML" "config/mono_inertial_right.yaml"
        if [ -d "$RUNTIME_GRAPH_CONFIG_DIR" ]; then
            upload_dir_atomic "$RUNTIME_GRAPH_CONFIG_DIR" "config/runtime_graph"
        fi
        if [ -d "$EPG_CONFIG_DIR" ]; then
            upload_dir_atomic "$EPG_CONFIG_DIR" "config/epg"
        fi
    fi

    upload_jetson_euroc_eval_bundle

    install_systemd_unit
    cleanup_legacy_dirs

    if [ "$RESTART_SERVICE" = "1" ]; then
        echo "restart service $REMOTE_SERVICE"
        run_remote_sudo "systemctl restart '$(remote_service_unit_name)'"
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
