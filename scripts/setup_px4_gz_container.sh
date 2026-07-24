#!/usr/bin/env bash
set -euo pipefail

Usage()
{
    cat <<'EOF'
Usage: scripts/setup_px4_gz_container.sh [options]

Build the sudo-free Ubuntu 24.04 / Gazebo Harmonic development image.

Options:
  --px4-dir PATH       Dedicated PX4 v1.17.0 checkout.
  --image NAME         Container image tag.
  --check-only         Verify the checkout and existing image only.
  -h, --help           Show this help.

PX4_GZ_BASE_IMAGE can override the Ubuntu 24.04 base image.
EOF
}

Fail()
{
    echo "ERR $*" >&2
    exit 1
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_DIR="$REPO_ROOT/../px4/PX4-Autopilot-v1.17.0"
IMAGE="${SMART_DRONE_PX4_GZ_IMAGE:-smartdrone-px4-gz:v1.17.0}"
BASE_IMAGE="${PX4_GZ_BASE_IMAGE:-swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/library/ubuntu:24.04}"
CHECK_ONLY=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --px4-dir) [[ $# -ge 2 ]] || Fail "--px4-dir requires a value"; PX4_DIR="$2"; shift 2 ;;
        --px4-dir=*) PX4_DIR="${1#--px4-dir=}"; shift ;;
        --image) [[ $# -ge 2 ]] || Fail "--image requires a value"; IMAGE="$2"; shift 2 ;;
        --image=*) IMAGE="${1#--image=}"; shift ;;
        --check-only) CHECK_ONLY=1; shift ;;
        -h|--help) Usage; exit 0 ;;
        *) Fail "unknown option: $1" ;;
    esac
done

command -v docker >/dev/null 2>&1 || Fail "docker is not installed"
PX4_DIR="$(realpath -m "$PX4_DIR")"
"$SCRIPT_DIR/setup_px4_gz_sitl.sh" --px4-dir "$PX4_DIR" \
    --skip-dependencies --non-interactive

if ((CHECK_ONLY == 0)); then
    docker build \
        --build-arg "BASE_IMAGE=$BASE_IMAGE" \
        --file "$REPO_ROOT/sim/px4_gz/docker/Dockerfile" \
        --tag "$IMAGE" \
        "$PX4_DIR/Tools/setup"
fi

docker image inspect "$IMAGE" >/dev/null 2>&1 || \
    Fail "container image is not available: $IMAGE"
docker run --rm "$IMAGE" bash -lc '
    version="$(gz sim --versions | head -n 1 | tr -d " ")"
    case "$version" in
        8|8.*) ;;
        *) echo "expected gz-sim 8, got ${version:-unknown}" >&2; exit 1 ;;
    esac
'

echo "OK  PX4:$PX4_DIR"
echo "OK  image:$IMAGE"
echo "Run commands with:"
echo "  ./scripts/run_in_px4_gz_container.sh -- <command>"
echo "Use --software-rendering for deterministic headless camera simulation."
