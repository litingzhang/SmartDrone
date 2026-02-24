#!/usr/bin/env bash
set -e

SYSROOT=/home/ltz/workspace/sysroots/cm5

rm -rf temp
cmake -S . -B temp -DSYSROOT="$SYSROOT" -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config
cmake --build temp -j16
