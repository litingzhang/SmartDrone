#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./build.sh [smart_drone|android|all|test|replay] [--clean] [--reconfigure]

Modes:
  smart_drone     Build the unified runtime target
  android         Build the Android app (:app:assembleDebug)
  all             Build ORB-SLAM3 first, then build smart_drone and Android app
  test            Build and run host-side unit tests with GoogleTest
  replay          Build the host-side offline replay tool

Options:
  --clean         Remove existing build directories before building
  --reconfigure   Re-run CMake configure for native builds even if build dir already exists
EOF
}

MODE="${1:-smart_drone}"
shift $(( $# > 0 ? 1 : 0 ))
BUILD_ORB=0
BUILD_SMART_DRONE=OFF
BUILD_ANDROID=0
BUILD_TESTS=0
BUILD_REPLAY=0
CLEAN_BUILD=0
FORCE_RECONFIGURE=0

case "$MODE" in
    smart_drone)
        BUILD_SMART_DRONE=ON
        ;;
    android)
        BUILD_ANDROID=1
        ;;
    all)
        BUILD_ORB=1
        BUILD_SMART_DRONE=ON
        BUILD_ANDROID=1
        ;;
    test)
        BUILD_TESTS=1
        ;;
    replay)
        BUILD_REPLAY=1
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
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
    esac
    shift
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SYSROOT="$(cd "$SCRIPT_DIR/../sysroots/cm5" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build/cmake"
TEST_BUILD_DIR="$SCRIPT_DIR/build/unit-test"
REPLAY_BUILD_DIR="$SCRIPT_DIR/build/offline-replay"
ANDROID_DIR="$SCRIPT_DIR/src/android"
ANDROID_APP_DIR="$ANDROID_DIR/app"
ANDROID_GRADLE_TASK="${ANDROID_GRADLE_TASK:-assembleDebug}"
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"

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
echo "CLEAN_BUILD:$CLEAN_BUILD"
echo "FORCE_RECONFIGURE:$FORCE_RECONFIGURE"
echo "BUILD_JOBS:$BUILD_JOBS"

if [ "$BUILD_ORB" -eq 1 ]; then
    echo "build ORB-SLAM3"
    cd "$SCRIPT_DIR/ORB_SLAM3"
    if [ "$CLEAN_BUILD" -eq 1 ]; then
        rm -rf build
    fi
    if [ "$FORCE_RECONFIGURE" -eq 1 ] || [ ! -d build ]; then
        cmake -S . -B build \
            -DSYSROOT="$SYSROOT" \
            -DCMAKE_TOOLCHAIN_FILE="$SCRIPT_DIR/toolchain/toolchain-cm5-aarch64.cmake"
    fi
    cmake --build build -j"$BUILD_JOBS"
    cd - >/dev/null
fi

if [ "$BUILD_SMART_DRONE" = "ON" ]; then
    if [ "$CLEAN_BUILD" -eq 1 ]; then
        rm -rf "$BUILD_DIR"
    fi
    mkdir -p "$BUILD_DIR"
    if [ "$FORCE_RECONFIGURE" -eq 1 ] || [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
        cmake -S . -B "$BUILD_DIR" \
            -DSYSROOT="$SYSROOT" \
            -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config \
            -DBUILD_SMART_DRONE="$BUILD_SMART_DRONE"
    fi

    if [ "$MODE" = "smart_drone" ]; then
        cmake --build "$BUILD_DIR" --target smart_drone -j"$BUILD_JOBS"
    else
        cmake --build "$BUILD_DIR" -j"$BUILD_JOBS"
    fi
fi

if [ "$BUILD_TESTS" -eq 1 ]; then
    if [ "$CLEAN_BUILD" -eq 1 ]; then
        rm -rf "$TEST_BUILD_DIR"
    fi
    mkdir -p "$TEST_BUILD_DIR"
    if [ "$FORCE_RECONFIGURE" -eq 1 ] || [ ! -f "$TEST_BUILD_DIR/CMakeCache.txt" ]; then
        cmake -S . -B "$TEST_BUILD_DIR" \
            -DCROSS_AARCH64=OFF \
            -DBUILD_SMART_DRONE=OFF \
            -DENABLE_UNIT_TESTS=ON
    fi
    cmake --build "$TEST_BUILD_DIR" -j"$BUILD_JOBS"
    ctest --test-dir "$TEST_BUILD_DIR" --output-on-failure
fi

if [ "$BUILD_REPLAY" -eq 1 ]; then
    if [ "$CLEAN_BUILD" -eq 1 ]; then
        rm -rf "$REPLAY_BUILD_DIR"
    fi
    mkdir -p "$REPLAY_BUILD_DIR"
    if [ "$FORCE_RECONFIGURE" -eq 1 ] || [ ! -f "$REPLAY_BUILD_DIR/CMakeCache.txt" ]; then
        cmake -S . -B "$REPLAY_BUILD_DIR" \
            -DCROSS_AARCH64=OFF \
            -DBUILD_SMART_DRONE=OFF \
            -DENABLE_OFFLINE_REPLAY=ON
    fi
    cmake --build "$REPLAY_BUILD_DIR" --target smart_drone_offline_replay -j"$BUILD_JOBS"
    echo "offline replay tool built:"
    echo "  $REPLAY_BUILD_DIR/tests/smart_drone_offline_replay"
fi

if [ "$BUILD_ANDROID" -eq 1 ]; then
    build_android_app
fi
