#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./build.sh [calib_recorder|smart_drone|both|all]

Modes:
  calib_recorder  Build only the calibration recorder target
  smart_drone     Build only the VIO target
  both            Build calib_recorder and smart_drone targets
  all             Build ORB-SLAM3 first, then build calib_recorder and smart_drone
EOF
}

MODE="${1:-both}"
BUILD_ORB=0
BUILD_CALIB_RECORDER=OFF
BUILD_SMART_DRONE=OFF

case "$MODE" in
    calib_recorder)
        BUILD_CALIB_RECORDER=ON
        ;;
    smart_drone)
        BUILD_SMART_DRONE=ON
        ;;
    both)
        BUILD_CALIB_RECORDER=ON
        BUILD_SMART_DRONE=ON
        ;;
    all)
        BUILD_ORB=1
        BUILD_CALIB_RECORDER=ON
        BUILD_SMART_DRONE=ON
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        usage
        exit 1
        ;;
esac

SYSROOT="$(cd ../sysroots/cm5 && pwd)"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/temp"

echo "SYSROOT:$SYSROOT"
echo "MODE:$MODE"

if [ "$BUILD_ORB" -eq 1 ]; then
    echo "build ORB-SLAM3"
    cd ../ORB_SLAM3-master
    rm -rf temp
    cmake -S . -B temp \
        -DSYSROOT="$SYSROOT" \
        -DCMAKE_TOOLCHAIN_FILE="$SCRIPT_DIR/toolchain/toolchain-cm5-aarch64.cmake"
    cmake --build temp -j16
    cd - >/dev/null
fi

rm -rf "$BUILD_DIR"
cmake -S . -B "$BUILD_DIR" \
    -DSYSROOT="$SYSROOT" \
    -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config \
    -DBUILD_CALIB_RECORDER="$BUILD_CALIB_RECORDER" \
    -DBUILD_SMART_DRONE="$BUILD_SMART_DRONE"

if [ "$MODE" = "calib_recorder" ]; then
    cmake --build "$BUILD_DIR" --target calib_recorder -j16
elif [ "$MODE" = "smart_drone" ]; then
    cmake --build "$BUILD_DIR" --target smart_drone -j16
else
    cmake --build "$BUILD_DIR" -j16
fi
