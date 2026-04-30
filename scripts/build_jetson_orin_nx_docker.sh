#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKERFILE="$REPO_ROOT/toolchain/docker/jetson-orin-nx-focal-cross.Dockerfile"
IMAGE_TAG="${JETSON_DOCKER_IMAGE_TAG:-smartdrone-jetson-orin-nx-focal-cross:local}"
BASE_IMAGE="${JETSON_DOCKER_BASE_IMAGE:-ubuntu:20.04}"
SYSROOT="${JETSON_SYSROOT:-$REPO_ROOT/../sysroots/jetson-orin-nx}"
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"
CAMERA_PROVIDER="${SMART_DRONE_CAMERA_PROVIDER:-uvc_stereo_opencv}"
CCACHE_DIR="${JETSON_DOCKER_CCACHE_DIR:-$REPO_ROOT/.ccache/jetson-orin-nx-focal-cross}"
CONTAINER_REPO="/workspace/repo"
CONTAINER_SYSROOT="/workspace/sysroots/jetson-orin-nx"

usage() {
    cat <<EOF
Usage:
  ./scripts/build_jetson_orin_nx_docker.sh [--clean] [--reconfigure]

Environment:
  JETSON_SYSROOT              Path to the Jetson Orin NX sysroot
  BUILD_JOBS                  Parallel build jobs inside the container
  SMART_DRONE_CAMERA_PROVIDER Camera provider to compile, default: uvc_stereo_opencv
  JETSON_DOCKER_IMAGE_TAG     Docker image tag, default: ${IMAGE_TAG}
  JETSON_DOCKER_BASE_IMAGE    Docker base image, default: ${BASE_IMAGE}
  JETSON_DOCKER_CCACHE_DIR    ccache directory on the host
EOF
}

CLEAN_BUILD=0
FORCE_RECONFIGURE=0

while [ "$#" -gt 0 ]; do
    case "$1" in
        --clean)
            CLEAN_BUILD=1
            ;;
        --reconfigure)
            FORCE_RECONFIGURE=1
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

if [ ! -d "$SYSROOT" ]; then
    echo "Jetson sysroot not found: $SYSROOT" >&2
    echo "Set JETSON_SYSROOT=/path/to/sysroot and retry." >&2
    exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "docker is required but was not found in PATH." >&2
    exit 1
fi

mkdir -p "$CCACHE_DIR"

if ! docker image inspect "$IMAGE_TAG" >/dev/null 2>&1; then
    docker build --build-arg "BASE_IMAGE=$BASE_IMAGE" -t "$IMAGE_TAG" -f "$DOCKERFILE" "$REPO_ROOT"
fi

CONTAINER_SCRIPT="$(cat <<'EOF'
set -euo pipefail
export CCACHE_DIR=/ccache
export CCACHE_BASEDIR=/workspace/repo
export CCACHE_COMPILERCHECK=content
export JETSON_SYSROOT=/workspace/sysroots/jetson-orin-nx

cd /workspace/repo

ORB_BUILD_DIR="output/build/jetson-orin-nx/orbslam3"
APP_BUILD_DIR="output/build/jetson-orin-nx/smart_drone"
ARTIFACT_DIR="output/artifacts/jetson-orin-nx"

if [ "$CLEAN_BUILD" = "1" ]; then
    rm -rf "$ORB_BUILD_DIR" "$APP_BUILD_DIR"
fi

mkdir -p "$ORB_BUILD_DIR" "$APP_BUILD_DIR" "$ARTIFACT_DIR/lib" "$ARTIFACT_DIR/bin" \
         "$ARTIFACT_DIR/config" "$ARTIFACT_DIR/scripts"

if [ "$FORCE_RECONFIGURE" = "1" ] || [ ! -f "$ORB_BUILD_DIR/CMakeCache.txt" ]; then
    cmake -S ORB_SLAM3 -B "$ORB_BUILD_DIR" -G Ninja \
        -DSYSROOT="$JETSON_SYSROOT" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="$PWD/toolchain/toolchain-jetson-orin-nx-aarch64.cmake" \
        -DORB_SLAM3_OUTPUT_DIR="$PWD/$ARTIFACT_DIR/lib" \
        -DDBOW2_OUTPUT_DIR="$PWD/$ARTIFACT_DIR/lib" \
        -DG2O_OUTPUT_DIR="$PWD/$ARTIFACT_DIR/lib" \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DJETSON_TOOLCHAIN_PREFIX=aarch64-linux-gnu
fi

cmake --build "$ORB_BUILD_DIR" -j"$BUILD_JOBS"

if [ "$FORCE_RECONFIGURE" = "1" ] || [ ! -f "$APP_BUILD_DIR/CMakeCache.txt" ]; then
    cmake -S . -B "$APP_BUILD_DIR" -G Ninja \
        -DSYSROOT="$JETSON_SYSROOT" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="$PWD/toolchain/toolchain-jetson-orin-nx-aarch64.cmake" \
        -DBUILD_SMART_DRONE=ON \
        -DSMART_DRONE_CAMERA_PROVIDER="$CAMERA_PROVIDER" \
        -DORB_LIB_DIR="$PWD/$ARTIFACT_DIR/lib" \
        -DDBOW2_LIB_DIR="$PWD/$ARTIFACT_DIR/lib" \
        -DG2O_LIB_DIR="$PWD/$ARTIFACT_DIR/lib" \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DJETSON_TOOLCHAIN_PREFIX=aarch64-linux-gnu
fi

cmake --build "$APP_BUILD_DIR" --target smart_drone -j"$BUILD_JOBS"

cp -f "$APP_BUILD_DIR/src/native/smart_drone" "$ARTIFACT_DIR/bin/smart_drone"
cp -f config/stereo.yaml "$ARTIFACT_DIR/config/stereo.yaml"
cp -f config/stereo_inertial.yaml "$ARTIFACT_DIR/config/stereo_inertial.yaml"
cp -f config/mono_right.yaml "$ARTIFACT_DIR/config/mono_right.yaml"
cp -f config/mono_inertial_right.yaml "$ARTIFACT_DIR/config/mono_inertial_right.yaml"

file "$ARTIFACT_DIR/bin/smart_drone"
ccache -s || true
EOF
)"

docker run --rm \
    --user "$(id -u):$(id -g)" \
    -e BUILD_JOBS="$BUILD_JOBS" \
    -e CAMERA_PROVIDER="$CAMERA_PROVIDER" \
    -e CLEAN_BUILD="$CLEAN_BUILD" \
    -e FORCE_RECONFIGURE="$FORCE_RECONFIGURE" \
    -v "$REPO_ROOT:$CONTAINER_REPO" \
    -v "$SYSROOT:$CONTAINER_SYSROOT:ro" \
    -v "$CCACHE_DIR:/ccache" \
    -w "$CONTAINER_REPO" \
    "$IMAGE_TAG" \
    bash -lc "$CONTAINER_SCRIPT"
