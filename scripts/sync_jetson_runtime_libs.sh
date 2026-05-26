#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/sync_jetson_runtime_libs.sh [--jetson HOST] [--sysroot PATH] [--setup-key] [--with-tensorrt]
                                       [--extra-manifest FILE] [--with-openvins]

Copies Jetson runtime libraries required by TensorRT SuperPoint into the cross-build
sysroot, preserving their absolute directory layout.

Options:
  --jetson HOST   SSH host for Jetson; defaults to JETSON_SSH_HOST or jetson
  --sysroot PATH  Target sysroot; defaults to JETSON_SYSROOT or ../sysroots/jetson-orin-nx
  --setup-key     Generate/install a dedicated SSH key before syncing
  --with-tensorrt Also sync TensorRT runtime libraries and headers if installed
  --with-openvins Also sync Jetson-side OpenVINS dependencies listed in toolchain/jetson_openvins_extra_libs.txt
  --extra-manifest FILE
                  Additional absolute paths on the Jetson to sync into the sysroot
EOF
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
JETSON_HOST="${JETSON_SSH_HOST:-jetson}"
SYSROOT="${JETSON_SYSROOT:-$REPO_ROOT/../sysroots/jetson-orin-nx}"
SETUP_KEY=0
WITH_TENSORRT=0
WITH_OPENVINS=0
EXTRA_MANIFEST=""
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
        --with-openvins)
            WITH_OPENVINS=1
            ;;
        --extra-manifest)
            if [ "$#" -lt 2 ]; then
                echo "--extra-manifest requires a value" >&2
                usage
                exit 1
            fi
            EXTRA_MANIFEST="$2"
            shift
            ;;
        --extra-manifest=*)
            EXTRA_MANIFEST="${1#--extra-manifest=}"
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

if [ -n "$EXTRA_MANIFEST" ]; then
    if [ ! -f "$EXTRA_MANIFEST" ]; then
        echo "extra manifest not found: $EXTRA_MANIFEST" >&2
        exit 1
    fi
    while IFS= read -r path; do
        if [ -n "$path" ]; then
            required_libs+=("$path")
        fi
    done < "$EXTRA_MANIFEST"
fi

if [ "$WITH_OPENVINS" -eq 1 ]; then
    OPENVINS_MANIFEST="$REPO_ROOT/toolchain/jetson_openvins_extra_libs.txt"
    if [ ! -f "$OPENVINS_MANIFEST" ]; then
        echo "OpenVINS manifest not found: $OPENVINS_MANIFEST" >&2
        exit 1
    fi
    while IFS= read -r path; do
        if [ -n "$path" ]; then
            required_libs+=("$path")
        fi
    done < "$OPENVINS_MANIFEST"
    required_libs+=(
        /home/nvidia/openvins_deps/prefix/usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
        /home/nvidia/openvins_deps/prefix/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
    )
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

if [ "$WITH_OPENVINS" -eq 1 ]; then
    for lib in boost_system boost_filesystem; do
        for base in \
            "$SYSROOT/usr/lib/aarch64-linux-gnu" \
            "$SYSROOT/lib/aarch64-linux-gnu" \
            "$SYSROOT/home/nvidia/openvins_deps/prefix/usr/lib/aarch64-linux-gnu"; do
            versioned="$(find "$base" -maxdepth 1 -type f -name "lib${lib}.so.*" | sort | head -n 1)"
            if [ -n "$versioned" ]; then
                ln -sf "$(basename "$versioned")" "$base/lib${lib}.so"
            fi
        done
    done

    for remote_dir in \
        /home/nvidia/openvins_deps/prefix/usr/include/ceres \
        /home/nvidia/openvins_deps/prefix/usr/include/glog \
        /home/nvidia/openvins_deps/prefix/usr/include/gflags \
        /home/nvidia/openvins_deps/prefix/usr/lib/cmake/Ceres; do
        if ssh "${SSH_ARGS[@]}" "$JETSON_HOST" "test -d '$remote_dir'"; then
            mkdir -p "$SYSROOT${remote_dir}"
            ssh "${SSH_ARGS[@]}" "$JETSON_HOST" \
                "tar -C '$remote_dir' -cf - ." | tar -C "$SYSROOT${remote_dir}" -xf -
        fi
    done
fi

echo "Jetson runtime libraries synced."
