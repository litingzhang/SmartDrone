#!/usr/bin/env bash
set -e

SYSROOT="$(cd ../sysroots/cm5 && pwd)"
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v "$SYSROOT/usr/bin" | paste -sd ':' -)"
if [ "$1" = "all" ]; then
    echo "build ORB-SLAM"
    cd ../ORB_SLAM3-master
    rm -rf build
    cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=$SYSROOT/toolchain-cm5-aarch64.cmake
    cmake --build build -j16
    cd -
fi

rm -rf build

cmake -S . -B build \
  -DSYSROOT="$SYSROOT" \
  -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config

cmake --build build -j16
