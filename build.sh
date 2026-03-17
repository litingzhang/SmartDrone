#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./build.sh [calib_recorder|smart_drone|both|android|all]

Modes:
  calib_recorder  Compatibility alias, builds unified smart_drone target
  smart_drone     Build the unified runtime target
  both            Compatibility alias, builds unified smart_drone target
  android         Build the Android app (:app:assembleDebug)
  all             Build ORB-SLAM3 first, then build unified smart_drone target and Android app
EOF
}

MODE="${1:-both}"
BUILD_ORB=0
BUILD_CALIB_RECORDER=OFF
BUILD_SMART_DRONE=OFF
BUILD_ANDROID=0

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
    android)
        BUILD_ANDROID=1
        ;;
    all)
        BUILD_ORB=1
        BUILD_CALIB_RECORDER=ON
        BUILD_SMART_DRONE=ON
        BUILD_ANDROID=1
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

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SYSROOT="$(cd "$SCRIPT_DIR/../sysroots/cm5" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build/cmake"
ANDROID_DIR="$SCRIPT_DIR/src/android"
ANDROID_APP_DIR="$ANDROID_DIR/app"
ANDROID_GRADLE_TASK="${ANDROID_GRADLE_TASK:-assembleDebug}"

build_android_app() {
    if [ ! -d "$ANDROID_DIR" ]; then
        echo "Android project not found: $ANDROID_DIR" >&2
        exit 1
    fi

    # The Android Gradle plugin caches absolute native build paths in app/.cxx.
    # Clearing it avoids stale CMake paths after the repo is moved or renamed.
    rm -rf "$ANDROID_APP_DIR/.cxx"

    echo "build Android app (:app:$ANDROID_GRADLE_TASK)"

    if [ -x "$ANDROID_DIR/gradlew" ]; then
        (
            cd "$ANDROID_DIR"
            ./gradlew ":app:$ANDROID_GRADLE_TASK"
        )
        return
    fi

    if [ -f "$ANDROID_DIR/gradlew" ]; then
        (
            cd "$ANDROID_DIR"
            bash ./gradlew ":app:$ANDROID_GRADLE_TASK"
        )
        return
    fi

    if [ -f "$ANDROID_DIR/gradlew.bat" ]; then
        (
            cd "$ANDROID_DIR"
            cmd.exe /c gradlew.bat ":app:$ANDROID_GRADLE_TASK"
        )
        return
    fi

    if command -v gradle >/dev/null 2>&1; then
        (
            cd "$ANDROID_DIR"
            gradle ":app:$ANDROID_GRADLE_TASK"
        )
        return
    fi

    echo "No Gradle wrapper or gradle executable found in $ANDROID_DIR" >&2
    exit 1
}

echo "SYSROOT:$SYSROOT"
echo "MODE:$MODE"

if [ "$BUILD_ORB" -eq 1 ]; then
    echo "build ORB-SLAM3"
    cd "$SCRIPT_DIR/ORB_SLAM3"
    rm -rf build
    cmake -S . -B build \
        -DSYSROOT="$SYSROOT" \
        -DCMAKE_TOOLCHAIN_FILE="$SCRIPT_DIR/toolchain/toolchain-cm5-aarch64.cmake"
    cmake --build build -j16
    cd - >/dev/null
fi

if [ "$BUILD_CALIB_RECORDER" = "ON" ] || [ "$BUILD_SMART_DRONE" = "ON" ]; then
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
    cmake -S . -B "$BUILD_DIR" \
        -DSYSROOT="$SYSROOT" \
        -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config \
        -DBUILD_CALIB_RECORDER="$BUILD_CALIB_RECORDER" \
        -DBUILD_SMART_DRONE="$BUILD_SMART_DRONE"

    if [ "$MODE" = "calib_recorder" ]; then
        cmake --build "$BUILD_DIR" --target smart_drone -j16
    elif [ "$MODE" = "smart_drone" ]; then
        cmake --build "$BUILD_DIR" --target smart_drone -j16
    else
        cmake --build "$BUILD_DIR" -j16
    fi
fi

if [ "$BUILD_ANDROID" -eq 1 ]; then
    build_android_app
fi
