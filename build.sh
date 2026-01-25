#!/usr/bin/env bash
set -e

SYSROOT="$(cd ../sysroots/cm5 && pwd)"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "SYSROOT:$SYSROOT"
if [ "$1" = "all" ]; then
    echo "build ORB-SLAM"
    cd ../ORB_SLAM3-master
    rm -rf temp
    cmake -S . -B temp -DSYSROOT=$SYSROOT -DCMAKE_TOOLCHAIN_FILE=$SCRIPT_DIR/toolchain/toolchain-cm5-aarch64.cmake
    cmake --build temp -j16
    cd -
fi

rm -rf temp
cmake -S . -B temp -DSYSROOT="$SYSROOT" -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config
cmake --build temp -j16
