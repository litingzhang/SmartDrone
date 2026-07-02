#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/build.sh [smart_drone|host-smart_drone|android|all|test|replay] [--clean] [--reconfigure] [--jetson-orin-nx]
                   [--jobs N] [--camera-provider NAME] [--enable-orb-slam3] [--enable-openvins]

Modes:
  smart_drone     Build the unified runtime target
  host-smart_drone
                  Build the unified runtime target for the Linux host
  android         Build the Android app (:app:assembleDebug)
  all             Build smart_drone and Android app
  test            Build and run host-side unit tests with GoogleTest
  replay          Build the host-side offline replay tool

Options:
  --clean         Remove existing build directories before building
  --reconfigure   Re-run CMake configure for native builds even if build dir already exists
  --jetson-orin-nx
                  Cross-build native targets for Jetson Orin NX instead of the default CM5 profile
  --opencv-cuda-orb
                  Enable the experimental OpenCV CUDA ORB extractor path when the sysroot provides cudafeatures2d
  --opencv-cuda-orb-root PATH
                  OpenCV install prefix that provides cudafeatures2d for --opencv-cuda-orb
  --enable-orb-slam3
                  Build and link the optional ORB-SLAM3 backend from src/native/adapters/slam/orb/orb_slam3
  --enable-openvins
                  Build and link the optional in-tree OpenVINS backend under src/native
  --jobs N        Build parallelism; defaults to BUILD_JOBS or nproc
  --camera-provider NAME
                  Native camera provider, e.g. libcamera_stereo_ov9281 or uvc_stereo_opencv
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

cache_value_matches() {
    local cache_file="$1"
    local key="$2"
    local expected="$3"
    if [ ! -f "$cache_file" ]; then
        return 1
    fi
    grep -Eq "^${key}:[^=]*=${expected}$" "$cache_file"
}

cmake_cache_needs_reconfigure() {
    local cache_file="$1"
    if [ ! -f "$cache_file" ]; then
        return 0
    fi
    if ! cache_value_matches "$cache_file" "SMART_DRONE_ENABLE_ORB_SLAM3" "$ENABLE_ORB_SLAM3"; then
        return 0
    fi
    if ! cache_value_matches "$cache_file" "SMART_DRONE_ENABLE_OPENCV_CUDA_ORB" "$ENABLE_OPENCV_CUDA_ORB"; then
        return 0
    fi
    if ! cache_value_matches "$cache_file" "SMART_DRONE_ENABLE_OPENVINS" "$ENABLE_OPENVINS"; then
        return 0
    fi
    return 1
}

MODE="${1:-smart_drone}"
shift $(( $# > 0 ? 1 : 0 ))
BUILD_SMART_DRONE=OFF
BUILD_ANDROID=0
BUILD_TESTS=0
BUILD_REPLAY=0
HOST_SMART_DRONE=0
CLEAN_BUILD=0
FORCE_RECONFIGURE=0
JETSON_ORIN_NX=0
NATIVE_RECONFIGURE_REQUIRED=0
ENABLE_ORB_SLAM3="${SMART_DRONE_ENABLE_ORB_SLAM3:-OFF}"
ENABLE_OPENVINS="${SMART_DRONE_ENABLE_OPENVINS:-OFF}"
BUILD_JOBS_OVERRIDE=""
CAMERA_PROVIDER_OVERRIDE=""
ENABLE_OPENCV_CUDA_ORB=OFF
OPENCV_CUDA_ORB_ROOT="${OPENCV_CUDA_ORB_ROOT:-}"
OPENVINS_ROOT=""
CERES_DIR_OVERRIDE="${CERES_DIR:-}"

case "$MODE" in
    smart_drone)
        BUILD_SMART_DRONE=ON
        ;;
    host-smart_drone)
        BUILD_SMART_DRONE=ON
        HOST_SMART_DRONE=1
        ;;
    android)
        BUILD_ANDROID=1
        ;;
    all)
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
        --opencv-cuda-orb)
            ENABLE_OPENCV_CUDA_ORB=ON
            ;;
        --enable-orb-slam3)
            ENABLE_ORB_SLAM3=ON
            ;;
        --enable-openvins)
            ENABLE_OPENVINS=ON
            ;;
        --openvins-root|--openvins-root=*)
            echo "--openvins-root is no longer supported; OpenVINS is built from src/native." >&2
            exit 1
            ;;
        --opencv-cuda-orb-root)
            if [ "$#" -lt 2 ]; then
                echo "--opencv-cuda-orb-root requires a value" >&2
                usage
                exit 1
            fi
            OPENCV_CUDA_ORB_ROOT="$2"
            shift
            ;;
        --opencv-cuda-orb-root=*)
            OPENCV_CUDA_ORB_ROOT="${1#--opencv-cuda-orb-root=}"
            ;;
        --jobs)
            if [ "$#" -lt 2 ]; then
                echo "--jobs requires a value" >&2
                usage
                exit 1
            fi
            BUILD_JOBS_OVERRIDE="$2"
            shift
            ;;
        --jobs=*)
            BUILD_JOBS_OVERRIDE="${1#--jobs=}"
            ;;
        --camera-provider)
            if [ "$#" -lt 2 ]; then
                echo "--camera-provider requires a value" >&2
                usage
                exit 1
            fi
            CAMERA_PROVIDER_OVERRIDE="$2"
            shift
            ;;
        --camera-provider=*)
            CAMERA_PROVIDER_OVERRIDE="${1#--camera-provider=}"
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
HOST_SMART_DRONE_BUILD_DIR="$OUTPUT_ROOT/build/host/smart_drone"
TEST_BUILD_DIR="$OUTPUT_ROOT/build/host/unit-test"
REPLAY_BUILD_DIR="$OUTPUT_ROOT/build/host/offline-replay"
ANDROID_DIR="$REPO_ROOT/src/android"
ANDROID_APP_DIR="$ANDROID_DIR/app"
ANDROID_GRADLE_TASK="${ANDROID_GRADLE_TASK:-assembleDebug}"
BUILD_JOBS="${BUILD_JOBS_OVERRIDE:-${BUILD_JOBS:-$(nproc)}}"
SMART_DRONE_CAMERA_PROVIDER="${CAMERA_PROVIDER_OVERRIDE:-${SMART_DRONE_CAMERA_PROVIDER:-}}"
PLATFORM_NAME="cm5"
SYSROOT_DEFAULT="$REPO_ROOT/../sysroots/cm5"
SYSROOT_ENV_NAME="SYSROOT"
TOOLCHAIN_FILE="$REPO_ROOT/toolchain/toolchain-cm5-aarch64.cmake"
TOOLCHAIN_PREFIX=""
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
NATIVE_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/$PLATFORM_NAME"
HOST_SMART_DRONE_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/host"
HOST_TEST_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/host/unit-test"
HOST_REPLAY_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/host/offline-replay"
REPLAY_ARTIFACTS_DIR="$HOST_REPLAY_ARTIFACTS_DIR"
ANDROID_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/android"
ORB_VOCAB_DIR="$REPO_ROOT/src/native/adapters/slam/orb/orb_slam3/place_recognition/vocabulary"
ORB_VOCAB_TXT="$ORB_VOCAB_DIR/ORBvoc.txt"
ORB_VOCAB_ARCHIVE="$ORB_VOCAB_DIR/ORBvoc.txt.tar.gz"

if [ "$JETSON_ORIN_NX" -eq 1 ]; then
    PLATFORM_NAME="jetson-orin-nx"
    SYSROOT_DEFAULT="$REPO_ROOT/../sysroots/jetson-orin-nx"
    SYSROOT_ENV_NAME="JETSON_SYSROOT"
    TOOLCHAIN_FILE="$REPO_ROOT/toolchain/toolchain-jetson-orin-nx-aarch64.cmake"
    BUILD_DIR="$OUTPUT_ROOT/build/jetson-orin-nx/smart_drone"
    REPLAY_BUILD_DIR="$OUTPUT_ROOT/build/jetson-orin-nx/offline-replay"
fi

if [ "$HOST_SMART_DRONE" -eq 1 ]; then
    PLATFORM_NAME="host"
    BUILD_DIR="$HOST_SMART_DRONE_BUILD_DIR"
    TOOLCHAIN_FILE=""
    SYSROOT=""
fi

NATIVE_ARTIFACTS_DIR="$OUTPUT_ROOT/artifacts/$PLATFORM_NAME"
if [ "$HOST_SMART_DRONE" -eq 1 ]; then
    NATIVE_ARTIFACTS_DIR="$HOST_SMART_DRONE_ARTIFACTS_DIR"
fi
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

if [ "$BUILD_SMART_DRONE" = "ON" ] && [ "$HOST_SMART_DRONE" -eq 0 ]; then
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
    -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
)

if [ "$HOST_SMART_DRONE" -eq 1 ]; then
    configure_native_args+=(
        -DCROSS_AARCH64=OFF
        -DBUILD_SMART_DRONE=ON
        -DSMART_DRONE_ENABLE_BOARD_IO=OFF
    )
else
    configure_native_args+=(-DSYSROOT="$SYSROOT")
fi

if [ -n "$TOOLCHAIN_FILE" ]; then
    configure_native_args+=(-DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE")
fi
if [ -n "$TOOLCHAIN_PREFIX" ]; then
    configure_native_args+=(-DJETSON_TOOLCHAIN_PREFIX="$TOOLCHAIN_PREFIX")
fi
if [ -n "${SMART_DRONE_CAMERA_PROVIDER:-}" ]; then
    configure_native_args+=(-DSMART_DRONE_CAMERA_PROVIDER="$SMART_DRONE_CAMERA_PROVIDER")
fi
configure_native_args+=(-DSMART_DRONE_ENABLE_OPENCV_CUDA_ORB="$ENABLE_OPENCV_CUDA_ORB")
configure_native_args+=(-DSMART_DRONE_ENABLE_ORB_SLAM3="$ENABLE_ORB_SLAM3")
if [ "$ENABLE_OPENVINS" = "ON" ] && [ -z "$CERES_DIR_OVERRIDE" ]; then
    if [ "$JETSON_ORIN_NX" -eq 1 ]; then
        CERES_DIR_OVERRIDE="$(find_first_existing_dir \
            "$SYSROOT/home/nvidia/openvins_deps/prefix/usr/lib/cmake/Ceres" \
            "$SYSROOT/usr/lib/cmake/Ceres" \
            "$SYSROOT/usr/lib/aarch64-linux-gnu/cmake/Ceres" || true)"
    elif [ "$BUILD_SMART_DRONE" != "ON" ]; then
        CERES_DIR_OVERRIDE="$(find_first_existing_dir \
            "$REPO_ROOT/output/third_party/ceres/lib/cmake/Ceres" \
            "$REPO_ROOT/../output/third_party/ceres/lib/cmake/Ceres" \
            "$HOME/workspace/SmartDrone/output/third_party/ceres/lib/cmake/Ceres" || true)"
    fi
fi
if [ "$ENABLE_OPENVINS" = "ON" ] && [ "$JETSON_ORIN_NX" -eq 1 ]; then
    OPENVINS_JETSON_PREFIX="$(find_first_existing_dir \
        "$SYSROOT/home/nvidia/openvins_deps/prefix/usr" \
        "$SYSROOT/usr" || true)"
    if [ -n "$OPENVINS_JETSON_PREFIX" ]; then
        configure_native_args+=(-DBoost_ROOT="$OPENVINS_JETSON_PREFIX")
        configure_native_args+=(-DBOOST_ROOT="$OPENVINS_JETSON_PREFIX")
        configure_native_args+=(-DBOOST_INCLUDEDIR="$OPENVINS_JETSON_PREFIX/include")
        configure_native_args+=(-DBOOST_LIBRARYDIR="$OPENVINS_JETSON_PREFIX/lib/aarch64-linux-gnu")
    fi
fi
configure_native_args+=(-DSMART_DRONE_ENABLE_OPENVINS="$ENABLE_OPENVINS")
if [ -n "$CERES_DIR_OVERRIDE" ]; then
    configure_native_args+=(-DCeres_DIR="$CERES_DIR_OVERRIDE")
fi
if [ "$ENABLE_OPENCV_CUDA_ORB" = "ON" ]; then
    if [ -z "$OPENCV_CUDA_ORB_ROOT" ]; then
        OPENCV_CUDA_ORB_ROOT="$(find_first_existing_dir \
            "$REPO_ROOT/../third_party/opencv_cuda_orb" \
            "$REPO_ROOT/output/opencv_cuda_orb" \
            "$HOME/workspace/third_party/opencv_cuda_orb" \
            "$HOME/opencv_cuda_orb" || true)"
    fi
    if [ -z "$OPENCV_CUDA_ORB_ROOT" ] || [ ! -f "$OPENCV_CUDA_ORB_ROOT/lib/cmake/opencv4/OpenCVConfig.cmake" ]; then
        echo "OpenCV CUDA ORB root not found or missing OpenCVConfig.cmake." >&2
        echo "Set OPENCV_CUDA_ORB_ROOT or pass --opencv-cuda-orb-root /path/to/opencv_cuda_orb." >&2
        exit 1
    fi
    configure_native_args+=(-DSMART_DRONE_OPENCV_CUDA_ORB_ROOT="$OPENCV_CUDA_ORB_ROOT")
    if [ -z "${CUDA_TOOLKIT_ROOT_DIR:-}" ]; then
        _opencv_cuda_fake_root="$(find_first_existing_dir \
            "$REPO_ROOT/../third_party/fake_cuda_11_4" \
            "$HOME/workspace/third_party/fake_cuda_11_4" || true)"
        if [ -n "$_opencv_cuda_fake_root" ]; then
            configure_native_args+=(-DCUDA_TOOLKIT_ROOT_DIR="$_opencv_cuda_fake_root")
        elif [ "$JETSON_ORIN_NX" -eq 1 ] && [ -d "$SYSROOT/usr/local/cuda-11.4" ]; then
            configure_native_args+=(-DCUDA_TOOLKIT_ROOT_DIR="$SYSROOT/usr/local/cuda-11.4")
        fi
    fi
fi
if [ "$JETSON_ORIN_NX" -eq 1 ] && [ -d "$REPO_ROOT/output/vpi2_jetson" ]; then
    configure_native_args+=(
        -DSMART_DRONE_ENABLE_VPI=ON
        -DSMART_DRONE_VPI_ROOT="$REPO_ROOT/output/vpi2_jetson"
    )
fi

copy_artifact() {
    local src="$1"
    local dst="$2"
    mkdir -p "$(dirname "$dst")"
    cp -f "$src" "$dst"
}

copy_openvins_configs() {
    local dst_dir="$1"
    if [ ! -d "$REPO_ROOT/config/openvins" ]; then
        return
    fi
    rm -rf "$dst_dir"
    mkdir -p "$dst_dir"
    cp -f "$REPO_ROOT"/config/openvins/*.yaml "$dst_dir/" 2>/dev/null || true
}

ensure_orb_vocabulary() {
    if [ -f "$ORB_VOCAB_TXT" ]; then
        return 0
    fi
    if [ -f "$ORB_VOCAB_ARCHIVE" ]; then
        tar -xzf "$ORB_VOCAB_ARCHIVE" -C "$ORB_VOCAB_DIR"
        return 0
    fi
    return 1
}

sync_native_artifacts() {
    local native_bin=""
    for candidate in "$BUILD_DIR/smart_drone" "$BUILD_DIR/src/native/smart_drone"; do
        if [ -f "$candidate" ]; then
            native_bin="$candidate"
            break
        fi
    done
    if [ -n "$native_bin" ]; then
        copy_artifact "$native_bin" "$NATIVE_ARTIFACTS_DIR/bin/smart_drone"
    fi

    local lib_dir="$NATIVE_ARTIFACTS_DIR/lib"
    mkdir -p "$lib_dir"

    local config_dir="$NATIVE_ARTIFACTS_DIR/config"
    mkdir -p "$config_dir"
    for cfg in stereo.yaml stereo_inertial.yaml mono_right.yaml mono_inertial_right.yaml; do
        if [ -f "$REPO_ROOT/config/$cfg" ]; then
            copy_artifact "$REPO_ROOT/config/$cfg" "$config_dir/$cfg"
        fi
    done
    copy_openvins_configs "$config_dir/openvins"
    if [ -d "$REPO_ROOT/config/runtime_graph" ]; then
        rm -rf "$config_dir/runtime_graph"
        mkdir -p "$config_dir/runtime_graph"
        cp -f "$REPO_ROOT"/config/runtime_graph/*.md "$config_dir/runtime_graph/" 2>/dev/null || true
    fi
    if [ -d "$REPO_ROOT/config/epg" ]; then
        rm -rf "$config_dir/epg"
        mkdir -p "$config_dir/epg"
        cp -f "$REPO_ROOT"/config/epg/*.dot "$config_dir/epg/" 2>/dev/null || true
    fi

    local scripts_dir="$NATIVE_ARTIFACTS_DIR/scripts"
    mkdir -p "$scripts_dir"
    if [ "$ENABLE_ORB_SLAM3" = "ON" ] && [ -d "$BUILD_DIR/orb_slam3/lib" ]; then
        local orb_lib
        for orb_lib in libORB_SLAM3.so libDBoW2.so libg2o.so; do
            if [ -f "$BUILD_DIR/orb_slam3/lib/$orb_lib" ]; then
                copy_artifact "$BUILD_DIR/orb_slam3/lib/$orb_lib" "$lib_dir/$orb_lib"
            fi
        done
    fi
    if [ "$ENABLE_ORB_SLAM3" = "ON" ]; then
        if ensure_orb_vocabulary; then
            copy_artifact "$ORB_VOCAB_TXT" "$NATIVE_ARTIFACTS_DIR/ORBvoc.txt"
        else
            echo "ORB-SLAM3 vocabulary not found: $ORB_VOCAB_TXT or $ORB_VOCAB_ARCHIVE" >&2
        fi
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
echo "OPENCV_CUDA_ORB:$ENABLE_OPENCV_CUDA_ORB"
echo "ORB_SLAM3:$ENABLE_ORB_SLAM3"
echo "OPENVINS:$ENABLE_OPENVINS"
if [ -n "$CERES_DIR_OVERRIDE" ]; then
    echo "CERES_DIR:$CERES_DIR_OVERRIDE"
fi
if [ -n "$OPENCV_CUDA_ORB_ROOT" ]; then
    echo "OPENCV_CUDA_ORB_ROOT:$OPENCV_CUDA_ORB_ROOT"
fi
echo "OUTPUT_ROOT:$OUTPUT_ROOT"

if [ "$BUILD_SMART_DRONE" = "ON" ]; then
    if [ "$CLEAN_BUILD" -eq 1 ]; then
        rm -rf "$BUILD_DIR"
    fi
    mkdir -p "$BUILD_DIR"
    if [ "$FORCE_RECONFIGURE" -eq 1 ] || [ "$NATIVE_RECONFIGURE_REQUIRED" -eq 1 ] || cmake_cache_needs_reconfigure "$BUILD_DIR/CMakeCache.txt"; then
        smart_drone_configure_args=(
            "${configure_native_args[@]}"
            -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config
            -DBUILD_SMART_DRONE="$BUILD_SMART_DRONE"
        )
        cmake -S "$REPO_ROOT" -B "$BUILD_DIR" "${smart_drone_configure_args[@]}"
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
    if [ "$FORCE_RECONFIGURE" -eq 1 ] || cmake_cache_needs_reconfigure "$REPLAY_BUILD_DIR/CMakeCache.txt"; then
        replay_configure_args=(
            -DBUILD_SMART_DRONE=OFF
            -DENABLE_OFFLINE_REPLAY=ON
            -DSMART_DRONE_ENABLE_ORB_SLAM3="$ENABLE_ORB_SLAM3"
            -DSMART_DRONE_ENABLE_OPENVINS="$ENABLE_OPENVINS"
        )
        if [ -n "$CERES_DIR_OVERRIDE" ]; then
            replay_configure_args+=(-DCeres_DIR="$CERES_DIR_OVERRIDE")
        fi
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
    if [ -f "$REPO_ROOT/config/epg/epg_topology.dot" ]; then
        copy_artifact "$REPO_ROOT/config/epg/epg_topology.dot" \
            "$REPLAY_ARTIFACTS_DIR/config/epg/epg_topology.dot"
    fi
    copy_openvins_configs "$REPLAY_ARTIFACTS_DIR/config/openvins"
    if [ "$ENABLE_ORB_SLAM3" = "ON" ] && [ "$JETSON_ORIN_NX" -eq 1 ] && [ -f "$REPLAY_BUILD_DIR/orb_slam3/lib/libORB_SLAM3.so" ]; then
        copy_artifact "$REPLAY_BUILD_DIR/orb_slam3/lib/libORB_SLAM3.so" \
            "$NATIVE_ARTIFACTS_DIR/lib/libORB_SLAM3.so"
    fi
    echo "offline replay tool built:"
    echo "  $REPLAY_ARTIFACTS_DIR/smart_drone_offline_replay"
fi

if [ "$BUILD_ANDROID" -eq 1 ]; then
    build_android_app
    sync_android_artifact
fi
