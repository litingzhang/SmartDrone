#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/setup_px4_gz_sitl.sh [options]

Options:
  --px4-dir PATH          Dedicated PX4 v1.17.0 checkout location.
  --check-only            Only report readiness; do not install or clone.
  --non-interactive       Fail instead of prompting for sudo credentials.
  --skip-dependencies     Do not run the upstream PX4 Ubuntu dependency setup.
  --force-dependencies    Run dependency setup even when local checks pass.
  -h, --help              Show this help.

The default checkout is ../px4/PX4-Autopilot-v1.17.0 relative to this repo.
The legacy ../px4/PX4-Autopilot checkout is deliberately never modified.
EOF
}

fail() {
    echo "ERR $*" >&2
    exit 1
}

is_ubuntu_2404() {
    [ -r /etc/os-release ] || return 1
    # shellcheck disable=SC1091
    . /etc/os-release
    [ "${ID:-}" = "ubuntu" ] && [ "${VERSION_ID:-}" = "24.04" ]
}

gazebo_harmonic_ready() {
    command -v gz >/dev/null 2>&1 || return 1
    local version
    version="$(gz sim --versions 2>/dev/null | head -n 1 | tr -d ' ')"
    [[ "$version" =~ ^8([.]|$) ]]
}

dependencies_ready() {
    local tool
    for tool in cmake git make ninja python3 rsync; do
        command -v "$tool" >/dev/null 2>&1 || return 1
    done
    gazebo_harmonic_ready || return 1
    python3 -c 'import jinja2, jsonschema, kconfiglib, numpy, yaml' \
        >/dev/null 2>&1
}

check_sudo() {
    if [ "$NON_INTERACTIVE" -eq 0 ] || [ "$(id -u)" -eq 0 ]; then
        return 0
    fi
    command -v sudo >/dev/null 2>&1 || fail "sudo is required to install dependencies"
    sudo -n true >/dev/null 2>&1 || \
        fail "non-interactive sudo is unavailable; install dependencies manually or rerun interactively"
}

verify_px4_checkout() {
    [ -f "$PX4_DIR/Makefile" ] || fail "PX4 checkout is incomplete: $PX4_DIR"
    git -C "$PX4_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1 || \
        fail "not a git checkout: $PX4_DIR"

    local expected actual
    expected="$(git -C "$PX4_DIR" rev-list -n 1 "$PX4_TAG" 2>/dev/null || true)"
    actual="$(git -C "$PX4_DIR" rev-parse HEAD)"
    [ -n "$expected" ] || fail "$PX4_TAG is not available in $PX4_DIR"
    [ "$actual" = "$expected" ] || \
        fail "$PX4_DIR is not pinned to $PX4_TAG; refusing to reset an existing checkout"
}

clone_px4_checkout() {
    [ ! -e "$PX4_DIR" ] || fail "path exists but is not a valid PX4 checkout: $PX4_DIR"
    mkdir -p "$(dirname "$PX4_DIR")"
    git clone --branch "$PX4_TAG" --depth 1 --recurse-submodules \
        --shallow-submodules https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
}

install_dependencies() {
    check_sudo
    export DEBIAN_FRONTEND="${DEBIAN_FRONTEND:-noninteractive}"
    "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_TAG="v1.17.0"
PX4_DIR="$REPO_ROOT/../px4/PX4-Autopilot-v1.17.0"
LEGACY_PX4_DIR="$REPO_ROOT/../px4/PX4-Autopilot"
CHECK_ONLY=0
NON_INTERACTIVE=0
SKIP_DEPENDENCIES=0
FORCE_DEPENDENCIES=0

while [ "$#" -gt 0 ]; do
    case "$1" in
        --px4-dir)
            [ "$#" -ge 2 ] || fail "--px4-dir requires a value"
            PX4_DIR="$2"
            shift
            ;;
        --px4-dir=*) PX4_DIR="${1#--px4-dir=}" ;;
        --check-only) CHECK_ONLY=1 ;;
        --non-interactive) NON_INTERACTIVE=1 ;;
        --skip-dependencies) SKIP_DEPENDENCIES=1 ;;
        --force-dependencies) FORCE_DEPENDENCIES=1 ;;
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

PX4_DIR="$(realpath -m "$PX4_DIR")"
LEGACY_PX4_DIR="$(realpath -m "$LEGACY_PX4_DIR")"
[ "$PX4_DIR" != "$LEGACY_PX4_DIR" ] || \
    fail "refusing to modify the legacy PX4 checkout: $LEGACY_PX4_DIR"

if [ "$CHECK_ONLY" -eq 1 ]; then
    exec "$SCRIPT_DIR/check_sitl_env.sh" --px4-dir "$PX4_DIR" \
        --skip-smart-drone
fi

is_ubuntu_2404 || fail "this setup script supports Ubuntu 24.04 only"

if [ ! -e "$PX4_DIR" ]; then
    clone_px4_checkout
fi
verify_px4_checkout

if [ -n "$(git -C "$PX4_DIR" status --porcelain --untracked-files=no --ignore-submodules=all)" ]; then
    fail "dedicated PX4 checkout has local changes; refusing to update submodules"
fi
git -C "$PX4_DIR" submodule update --init --recursive

if [ "$SKIP_DEPENDENCIES" -eq 0 ]; then
    if [ "$FORCE_DEPENDENCIES" -eq 1 ] || ! dependencies_ready; then
        install_dependencies
        dependencies_ready || fail "dependency setup completed but PX4/Gazebo checks still fail"
    else
        echo "OK  dependencies:already installed"
    fi
fi

[ -f "$PX4_DIR/Tools/simulation/gz/models/x500/model.sdf" ] || \
    fail "PX4 Gazebo model submodule was not initialized"

echo
echo "PX4/Gazebo source environment is installed."
echo "PX4_AUTOPILOT_DIR=$PX4_DIR"
echo "Build and validate it with:"
echo "  ./scripts/run_px4_gz_sitl.sh --profile control --headless"
