#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/build_jetson_orin_nx_cross.sh [--clean] [--jobs N]

This wrapper now forwards to:
  ./scripts/build.sh orb --jetson-orin-nx ...
  ./scripts/build.sh smart_drone --jetson-orin-nx ...

Jetson sysroot/toolchain can be auto-detected by scripts/build.sh. You may still
override them with:
  JETSON_SYSROOT
  JETSON_TOOLCHAIN_PREFIX
  JETSON_TOOLCHAIN_HOST_LIBDIR
  SMART_DRONE_CAMERA_PROVIDER
EOF
}

CLEAN_BUILD=0
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --clean)
            CLEAN_BUILD=1
            ;;
        --jobs)
            shift
            BUILD_JOBS="${1:?missing value for --jobs}"
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
ARTIFACTS_DIR="$REPO_ROOT/output/artifacts/jetson-orin-nx"
export BUILD_JOBS

build_args=(--jetson-orin-nx --reconfigure)
if [ "$CLEAN_BUILD" -eq 1 ]; then
    build_args+=(--clean)
fi

"$REPO_ROOT/scripts/build.sh" orb "${build_args[@]}"
"$REPO_ROOT/scripts/build.sh" smart_drone "${build_args[@]}"

printf 'Cross build complete.\n'
printf 'Artifacts: %s\n' "$ARTIFACTS_DIR"
printf 'Binary: %s\n' "$ARTIFACTS_DIR/bin/smart_drone"
