#!/usr/bin/env bash
set -e

if [ "$1" = "all" ]; then
    echo "build ORB-SLAM"
    cd ../ORB_SLAM3-master
    rm -rf build
    cmake -S . -B build
    cmake --build build -j16
fi

SYSROOT="$(cd ../sysroots/cm5 && pwd)"
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v "$SYSROOT/usr/bin" | paste -sd ':' -)"

rm -rf build

cmake -S . -B build \
  -DSYSROOT="$SYSROOT" \
  -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config

cmake --build build -j16
