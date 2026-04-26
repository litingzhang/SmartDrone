#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/sync_jetson_runtime_libs.sh [--jetson HOST] [--sysroot PATH] [--setup-key] [--with-tensorrt]

Copies Jetson runtime libraries required by TensorRT XFeat into the cross-build
sysroot, preserving their absolute directory layout.

Options:
  --jetson HOST   SSH host for Jetson; defaults to JETSON_SSH_HOST or jetson
  --sysroot PATH  Target sysroot; defaults to JETSON_SYSROOT or ../sysroots/jetson-orin-nx
  --setup-key     Generate/install a dedicated SSH key before syncing
  --with-tensorrt Also sync TensorRT runtime libraries and headers if installed
EOF
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
JETSON_HOST="${JETSON_SSH_HOST:-jetson}"
SYSROOT="${JETSON_SYSROOT:-$REPO_ROOT/../sysroots/jetson-orin-nx}"
SETUP_KEY=0
WITH_TENSORRT=0
SSH_KEY="${SMART_DRONE_JETSON_SSH_KEY:-$HOME/.ssh/smartdrone_jetson_ed25519}"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --jetson)
            if [ "$#" -lt 2 ]; then
                echo "--jetson requires a value" >&2
                usage
                exit 1
            fi
            JETSON_HOST="$2"
            shift
            ;;
        --jetson=*)
            JETSON_HOST="${1#--jetson=}"
            ;;
        --sysroot)
            if [ "$#" -lt 2 ]; then
                echo "--sysroot requires a value" >&2
                usage
                exit 1
            fi
            SYSROOT="$2"
            shift
            ;;
        --sysroot=*)
            SYSROOT="${1#--sysroot=}"
            ;;
        --setup-key)
            SETUP_KEY=1
            ;;
        --with-tensorrt)
            WITH_TENSORRT=1
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

mkdir -p "$SYSROOT"

SSH_ARGS=()
RSYNC_SSH="ssh"

if [ "$SETUP_KEY" -eq 1 ]; then
    mkdir -p "$HOME/.ssh"
    chmod 700 "$HOME/.ssh"
    if [ ! -f "$SSH_KEY" ]; then
        ssh-keygen -t ed25519 -N "" -f "$SSH_KEY" -C "smartdrone-jetson"
    fi
    echo "Installing SSH key on $JETSON_HOST. Enter the Jetson password once if prompted."
    pub_key="$(cat "$SSH_KEY.pub")"
    ssh "$JETSON_HOST" "mkdir -p ~/.ssh && chmod 700 ~/.ssh && touch ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && grep -qxF '$pub_key' ~/.ssh/authorized_keys || echo '$pub_key' >> ~/.ssh/authorized_keys"
fi

if [ -f "$SSH_KEY" ]; then
    SSH_ARGS=(-i "$SSH_KEY")
    RSYNC_SSH="ssh -i $SSH_KEY"
fi

required_libs=(
    /usr/lib/aarch64-linux-gnu/libcudnn.so.8
    /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudart.so.11.0
    /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcublas.so.11
    /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcublasLt.so.11
)

if [ "$WITH_TENSORRT" -eq 1 ]; then
    tensorrt_candidates=(
        /usr/lib/aarch64-linux-gnu/libnvinfer.so.8
        /usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so.8
        /usr/lib/aarch64-linux-gnu/libnvonnxparser.so.8
        /usr/lib/aarch64-linux-gnu/tegra/libnvdla_compiler.so
        /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1
        /usr/include/NvInfer.h
        /usr/include/NvInferPlugin.h
        /usr/include/NvInferRuntime.h
        /usr/include/NvInferRuntimeCommon.h
        /usr/include/NvInferVersion.h
        /usr/include/aarch64-linux-gnu/NvInfer.h
        /usr/include/aarch64-linux-gnu/NvInferPlugin.h
        /usr/include/aarch64-linux-gnu/NvInferRuntime.h
        /usr/include/aarch64-linux-gnu/NvInferRuntimeCommon.h
        /usr/include/aarch64-linux-gnu/NvInferVersion.h
        /usr/include/aarch64-linux-gnu/NvUtils.h
    )
    for path in "${tensorrt_candidates[@]}"; do
        if ssh "${SSH_ARGS[@]}" "$JETSON_HOST" "test -e '$path'"; then
            required_libs+=("$path")
        fi
    done
fi

echo "JETSON_HOST:$JETSON_HOST"
echo "SYSROOT:$SYSROOT"
if [ -f "$SSH_KEY" ]; then
    echo "SSH_KEY:$SSH_KEY"
fi

check_script="missing=0"
for lib in "${required_libs[@]}"; do
    check_script="$check_script; test -e '$lib' || { echo '$lib'; missing=1; }"
done
check_script="$check_script; exit \$missing"

if ! missing_output="$(ssh "${SSH_ARGS[@]}" "$JETSON_HOST" "$check_script")"; then
    echo "Missing on Jetson:" >&2
    printf '%s\n' "$missing_output" | sed 's/^/  /' >&2
    exit 1
fi

tmp_files="$(mktemp)"
trap 'rm -f "$tmp_files"' EXIT
printf '%s\n' "${required_libs[@]}" > "$tmp_files"

rsync -avRL --copy-links --files-from="$tmp_files" -e "$RSYNC_SSH" "$JETSON_HOST:/" "$SYSROOT/"

if [ "$WITH_TENSORRT" -eq 1 ]; then
    if ssh "${SSH_ARGS[@]}" "$JETSON_HOST" "test -d /usr/local/cuda-11.4/targets/aarch64-linux/include"; then
        rsync -avRL --copy-links -e "$RSYNC_SSH" \
            "$JETSON_HOST:/usr/local/cuda-11.4/targets/aarch64-linux/include/" "$SYSROOT/"
    fi
fi

echo "Jetson runtime libraries synced."
