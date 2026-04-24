#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/build.sh [smart_drone|orb|android|all|test|replay] [--clean] [--reconfigure] [--jetson-orin-nx]

Modes:
  smart_drone     Build the unified runtime target
  orb             Build ORB-SLAM3 and its native third-party shared libs
  android         Build the Android app (:app:assembleDebug)
  all             Build ORB-SLAM3 first, then build smart_drone and Android app
  test            Build and run host-side unit tests with GoogleTest
  replay          Build the host-side offline replay tool

Options:
  --clean         Remove existing build directories before building
  --reconfigure   Re-run CMake configure for native builds even if build dir already exists
  --jetson-orin-nx
                  Cross-build native targets for Jetson Orin NX instead of the default CM5 profile
EOF
}

find_first_existing_dir() {
    local path
    for path in "$@"; do
        if [ -n "$path" ] && [ -d "$path" ]; then
            printf '%s\n' "$path"
            return 0
        fi
    done
    return 1
}

find_first_executable() {
    local path
    for path in "$@"; do
        if [ -n "$path" ] && [ -x "$path" ]; then
            printf '%s\n' "$path"
            return 0
        fi
    done
    return 1
}

find_first_toolchain_prefix() {
    local prefix
    for prefix in "$@"; do
        if [ -n "$prefix" ] && [ -x "${prefix}-g++" ]; then
            printf '%s\n' "$prefix"
            return 0
        fi
    done
    return 1
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
JETSON_ORIN_NX=0
NATIVE_RECONFIGURE_REQUIRED=0

case "$MODE" in
    smart_drone)
        BUILD_SMART_DRONE=ON
        ;;
    orb)
        BUILD_ORB=1
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
        --jetson-orin-nx)
            JETSON_ORIN_NX=1
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
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OUTPUT_ROOT="$REPO_ROOT/output"
BUILD_DIR="$OUTPUT_ROOT/build/cm5/smart_drone"
TEST_BUILD_DIR="$OUTPUT_ROOT/build/host/unit-test"
REPLAY_BUILD_DIR="$OUTPUT_ROOT/build/host/offline-replay"
ANDROID_DIR="$REPO_ROOT/src/android"
ANDROID_APP_DIR="$ANDROID_DIR/app"
ANDROID_GRADLE_TASK="${ANDROID_GRADLE_TASK:-assembleDebug}"
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"
PLATFORM_NAME="cm5"
SYSROOT_DEFAULT="$REPO_ROOT/../sysroots/cm5"
SYSROOT_ENV_NAME="SYSROOT"
TOOLCHAIN_FILE="$REPO_ROOT/toolchain/toolchain-cm5-aarch64.cmake"
TOOLCHAIN_PREFIX=""
ORB_BUILD_DIR="$OUTPUT_ROOT/build/cm5/orbslam3"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
NATIVE_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/$PLATFORM_NAME"
HOST_TEST_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/host/unit-test"
HOST_REPLAY_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/host/offline-replay"
REPLAY_ARTIFACTS_DIR="$HOST_REPLAY_ARTIFACTS_DIR"
ANDROID_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/android"

if [ "$JETSON_ORIN_NX" -eq 1 ]; then
    PLATFORM_NAME="jetson-orin-nx"
    SYSROOT_DEFAULT="$REPO_ROOT/../sysroots/jetson-orin-nx"
    SYSROOT_ENV_NAME="JETSON_SYSROOT"
    TOOLCHAIN_FILE="$REPO_ROOT/toolchain/toolchain-jetson-orin-nx-aarch64.cmake"
    BUILD_DIR="$OUTPUT_ROOT/build/jetson-orin-nx/smart_drone"
    ORB_BUILD_DIR="$OUTPUT_ROOT/build/jetson-orin-nx/orbslam3"
    REPLAY_BUILD_DIR="$OUTPUT_ROOT/build/jetson-orin-nx/offline-replay"
fi

NATIVE_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/$PLATFORM_NAME"
if [ "$JETSON_ORIN_NX" -eq 1 ]; then
    REPLAY_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/$PLATFORM_NAME/offline-replay"
fi

SYSROOT="${SYSROOT:-$SYSROOT_DEFAULT}"
if [ "$JETSON_ORIN_NX" -eq 1 ]; then
    SYSROOT="$(find_first_existing_dir \
        "${JETSON_SYSROOT:-}" \
        "$SYSROOT" \
        "$REPO_ROOT/../sysroots/jetson-orin-nx" \
        "$HOME/workspace/sysroots/jetson-orin-nx" \
        "$HOME/sysroots/jetson-orin-nx" || true)"
    TOOLCHAIN_PREFIX="$(find_first_toolchain_prefix \
        "${JETSON_TOOLCHAIN_PREFIX:-}" \
        "$HOME/toolchains/jetson-focal-cross/usr/bin/aarch64-linux-gnu" \
        "/usr/bin/aarch64-linux-gnu" || true)"
    JETSON_TOOLCHAIN_HOST_LIBDIR_DEFAULT="$(find_first_existing_dir \
        "${JETSON_TOOLCHAIN_HOST_LIBDIR:-}" \
        "$HOME/toolchains/jetson-focal-cross/usr/lib/x86_64-linux-gnu" || true)"
    if [ -n "$JETSON_TOOLCHAIN_HOST_LIBDIR_DEFAULT" ]; then
        export LD_LIBRARY_PATH="$JETSON_TOOLCHAIN_HOST_LIBDIR_DEFAULT:${LD_LIBRARY_PATH:-}"
    fi
else
    TOOLCHAIN_PREFIX="${TOOLCHAIN_PREFIX:-}"
fi

if [ "$BUILD_SMART_DRONE" = "ON" ] || [ "$BUILD_ORB" -eq 1 ]; then
    if [ ! -d "$SYSROOT" ]; then
        echo "Sysroot not found: $SYSROOT" >&2
        echo "Set $SYSROOT_ENV_NAME=/path/to/sysroot and retry." >&2
        exit 1
    fi
    if [ "$JETSON_ORIN_NX" -eq 1 ] && [ -z "$TOOLCHAIN_PREFIX" ]; then
        echo "Jetson cross compiler prefix not found." >&2
        echo "Set JETSON_TOOLCHAIN_PREFIX=/path/to/aarch64-linux-gnu and retry." >&2
        exit 1
    fi
fi

if [ "$BUILD_REPLAY" -eq 1 ] && [ "$JETSON_ORIN_NX" -eq 1 ]; then
    if [ ! -d "$SYSROOT" ]; then
        echo "Sysroot not found: $SYSROOT" >&2
        echo "Set $SYSROOT_ENV_NAME=/path/to/sysroot and retry." >&2
        exit 1
    fi
    if [ -z "$TOOLCHAIN_PREFIX" ]; then
        echo "Jetson cross compiler prefix not found." >&2
        echo "Set JETSON_TOOLCHAIN_PREFIX=/path/to/aarch64-linux-gnu and retry." >&2
        exit 1
    fi
fi

configure_native_args=(
    -DSYSROOT="$SYSROOT"
    -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
)

if [ -n "$TOOLCHAIN_FILE" ]; then
    configure_native_args+=(-DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE")
fi
if [ -n "$TOOLCHAIN_PREFIX" ]; then
    configure_native_args+=(-DJETSON_TOOLCHAIN_PREFIX="$TOOLCHAIN_PREFIX")
fi
if [ -n "${SMART_DRONE_CAMERA_PROVIDER:-}" ]; then
    configure_native_args+=(-DSMART_DRONE_CAMERA_PROVIDER="$SMART_DRONE_CAMERA_PROVIDER")
fi

copy_artifact() {
    local src="$1"
    local dst="$2"
    mkdir -p "$(dirname "$dst")"
    cp -f "$src" "$dst"
}

sync_native_artifacts() {
    local native_bin="$BUILD_DIR/src/native/smart_drone"
    if [ -f "$native_bin" ]; then
        copy_artifact "$native_bin" "$NATIVE_ARTIFACTS_DIR/bin/smart_drone"
    fi

    local lib_dir="$NATIVE_ARTIFACTS_DIR/lib"
    mkdir -p "$lib_dir"
    for lib_name in libORB_SLAM3.so libDBoW2.so libg2o.so; do
        if [ -f "$lib_dir/$lib_name" ]; then
            :
        fi
    done

    local config_dir="$NATIVE_ARTIFACTS_DIR/config"
    mkdir -p "$config_dir"
    for cfg in stereo.yaml stereo_inertial.yaml mono_right.yaml mono_inertial_right.yaml; do
        if [ -f "$REPO_ROOT/config/$cfg" ]; then
            copy_artifact "$REPO_ROOT/config/$cfg" "$config_dir/$cfg"
        fi
    done

    local scripts_dir="$NATIVE_ARTIFACTS_DIR/scripts"
    mkdir -p "$scripts_dir"
    if [ -f "$REPO_ROOT/scripts/xfeat_keypoint_worker.py" ]; then
        copy_artifact "$REPO_ROOT/scripts/xfeat_keypoint_worker.py" \
            "$scripts_dir/xfeat_keypoint_worker.py"
    fi
    if [ -f "$REPO_ROOT/ORBvoc.txt" ]; then
        copy_artifact "$REPO_ROOT/ORBvoc.txt" "$NATIVE_ARTIFACTS_DIR/ORBvoc.txt"
    elif [ -f "$REPO_ROOT/ORB_SLAM3/Vocabulary/ORBvoc.txt" ]; then
        copy_artifact "$REPO_ROOT/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
            "$NATIVE_ARTIFACTS_DIR/ORBvoc.txt"
    fi

    if [ -d "$REPO_ROOT/accelerated_features" ]; then
        rm -rf "$NATIVE_ARTIFACTS_DIR/accelerated_features"
        cp -a "$REPO_ROOT/accelerated_features" "$NATIVE_ARTIFACTS_DIR/accelerated_features"
    fi
}

sync_android_artifact() {
    local apk_src
    apk_src="$(find "$ANDROID_APP_DIR/build/outputs/apk" -type f -name '*.apk' | sort | tail -n 1)"
    if [ -z "$apk_src" ]; then
        return
    fi
    copy_artifact "$apk_src" "$ANDROID_ARTIFACTS_DIR/latest.apk"
}

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

echo "PLATFORM:$PLATFORM_NAME"
echo "SYSROOT:$SYSROOT"
if [ -n "$TOOLCHAIN_PREFIX" ]; then
    echo "TOOLCHAIN_PREFIX:$TOOLCHAIN_PREFIX"
fi
if [ "$JETSON_ORIN_NX" -eq 1 ] && [ -n "${JETSON_TOOLCHAIN_HOST_LIBDIR_DEFAULT:-}" ]; then
    echo "TOOLCHAIN_HOST_LIBDIR:$JETSON_TOOLCHAIN_HOST_LIBDIR_DEFAULT"
fi
echo "MODE:$MODE"
echo "CLEAN_BUILD:$CLEAN_BUILD"
echo "FORCE_RECONFIGURE:$FORCE_RECONFIGURE"
echo "BUILD_JOBS:$BUILD_JOBS"
echo "CMAKE_BUILD_TYPE:$CMAKE_BUILD_TYPE"
echo "OUTPUT_ROOT:$OUTPUT_ROOT"

if [ "$BUILD_ORB" -eq 1 ]; then
    echo "build ORB-SLAM3"
    if [ "$CLEAN_BUILD" -eq 1 ]; then
        rm -rf "$ORB_BUILD_DIR"
    fi
    if [ "$FORCE_RECONFIGURE" -eq 1 ] || [ ! -f "$ORB_BUILD_DIR/CMakeCache.txt" ]; then
        cmake -S "$REPO_ROOT/ORB_SLAM3" -B "$ORB_BUILD_DIR" \
            "${configure_native_args[@]}" \
            -DORB_SLAM3_OUTPUT_DIR="$NATIVE_ARTIFACTS_DIR/lib" \
            -DDBOW2_OUTPUT_DIR="$NATIVE_ARTIFACTS_DIR/lib" \
            -DG2O_OUTPUT_DIR="$NATIVE_ARTIFACTS_DIR/lib"
    fi
    cmake --build "$ORB_BUILD_DIR" -j"$BUILD_JOBS"
    NATIVE_RECONFIGURE_REQUIRED=1
fi

if [ "$BUILD_SMART_DRONE" = "ON" ]; then
    if [ "$CLEAN_BUILD" -eq 1 ]; then
        rm -rf "$BUILD_DIR"
    fi
    mkdir -p "$BUILD_DIR"
    if [ "$FORCE_RECONFIGURE" -eq 1 ] || [ "$NATIVE_RECONFIGURE_REQUIRED" -eq 1 ] || [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
        cmake -S "$REPO_ROOT" -B "$BUILD_DIR" \
            "${configure_native_args[@]}" \
            -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config \
            -DBUILD_SMART_DRONE="$BUILD_SMART_DRONE" \
            -DORB_LIB_DIR="$NATIVE_ARTIFACTS_DIR/lib" \
            -DDBOW2_LIB_DIR="$NATIVE_ARTIFACTS_DIR/lib" \
            -DG2O_LIB_DIR="$NATIVE_ARTIFACTS_DIR/lib"
    fi

    if [ "$MODE" = "smart_drone" ]; then
        cmake --build "$BUILD_DIR" --target smart_drone -j"$BUILD_JOBS"
    else
        cmake --build "$BUILD_DIR" -j"$BUILD_JOBS"
    fi
    sync_native_artifacts
fi

if [ "$BUILD_TESTS" -eq 1 ]; then
    if [ "$CLEAN_BUILD" -eq 1 ]; then
        rm -rf "$TEST_BUILD_DIR"
    fi
    mkdir -p "$TEST_BUILD_DIR"
    if [ "$FORCE_RECONFIGURE" -eq 1 ] || [ ! -f "$TEST_BUILD_DIR/CMakeCache.txt" ]; then
        cmake -S "$REPO_ROOT" -B "$TEST_BUILD_DIR" \
            -DCROSS_AARCH64=OFF \
            -DBUILD_SMART_DRONE=OFF \
            -DENABLE_UNIT_TESTS=ON
    fi
    cmake --build "$TEST_BUILD_DIR" -j"$BUILD_JOBS"
    if [ -f "$TEST_BUILD_DIR/tests/smart_drone_unit_tests" ]; then
        copy_artifact "$TEST_BUILD_DIR/tests/smart_drone_unit_tests" \
            "$HOST_TEST_ARTIFACTS_DIR/smart_drone_unit_tests"
    fi
    ctest --test-dir "$TEST_BUILD_DIR" --output-on-failure
fi

if [ "$BUILD_REPLAY" -eq 1 ]; then
    if [ "$CLEAN_BUILD" -eq 1 ]; then
        rm -rf "$REPLAY_BUILD_DIR"
    fi
    mkdir -p "$REPLAY_BUILD_DIR"
    if [ "$FORCE_RECONFIGURE" -eq 1 ] || [ ! -f "$REPLAY_BUILD_DIR/CMakeCache.txt" ]; then
        replay_configure_args=(
            -DBUILD_SMART_DRONE=OFF
            -DENABLE_OFFLINE_REPLAY=ON
        )
        if [ "$JETSON_ORIN_NX" -eq 1 ]; then
            replay_configure_args+=(
                "${configure_native_args[@]}"
                -DCROSS_AARCH64=ON
            )
        else
            replay_configure_args+=(-DCROSS_AARCH64=OFF)
        fi
        cmake -S "$REPO_ROOT" -B "$REPLAY_BUILD_DIR" "${replay_configure_args[@]}"
    fi
    cmake --build "$REPLAY_BUILD_DIR" --target smart_drone_offline_replay -j"$BUILD_JOBS"
    if [ -f "$REPLAY_BUILD_DIR/tests/smart_drone_offline_replay" ]; then
        copy_artifact "$REPLAY_BUILD_DIR/tests/smart_drone_offline_replay" \
            "$REPLAY_ARTIFACTS_DIR/smart_drone_offline_replay"
    fi
    echo "offline replay tool built:"
    echo "  $REPLAY_ARTIFACTS_DIR/smart_drone_offline_replay"
fi

if [ "$BUILD_ANDROID" -eq 1 ]; then
    build_android_app
    sync_android_artifact
fi
