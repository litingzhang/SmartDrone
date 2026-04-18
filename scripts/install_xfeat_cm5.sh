#!/usr/bin/env bash
[ -n "${BASH_VERSION:-}" ] || exec bash "$0" "$@"
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/verlab/accelerated_features.git}"
REPO_DIR="${REPO_DIR:-$HOME/third_party/accelerated_features}"
VENV_DIR="${VENV_DIR:-$HOME/.venvs/xfeat}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
INSTALL_DEMO_DEPS="${INSTALL_DEMO_DEPS:-1}"
APT_GET="${APT_GET:-sudo apt-get}"
PIP_RETRIES="${PIP_RETRIES:-5}"
PIP_TIMEOUT="${PIP_TIMEOUT:-120}"

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Install XFeat into a Python virtualenv on Raspberry Pi CM5 / Linux aarch64.

Options:
  --repo-dir PATH      Clone or update XFeat repo here. Default: $REPO_DIR
  --venv-dir PATH      Python virtualenv path. Default: $VENV_DIR
  --python BIN         Python executable to use. Default: $PYTHON_BIN
  --no-demo-deps       Skip OpenCV/tqdm demo dependencies
  -h, --help           Show this help

Environment overrides:
  REPO_URL             XFeat Git repository URL
  APT_GET              apt-get command prefix, default: "sudo apt-get"
  PIP_RETRIES          pip retry count for flaky networks, default: $PIP_RETRIES
  PIP_TIMEOUT          pip timeout in seconds, default: $PIP_TIMEOUT
EOF
}

pip_install() {
    python -m pip install \
        --retries "$PIP_RETRIES" \
        --timeout "$PIP_TIMEOUT" \
        "$@"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --repo-dir)
            REPO_DIR="$2"
            shift 2
            ;;
        --venv-dir)
            VENV_DIR="$2"
            shift 2
            ;;
        --python)
            PYTHON_BIN="$2"
            shift 2
            ;;
        --no-demo-deps)
            INSTALL_DEMO_DEPS=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

ARCH="$(uname -m)"
if [[ "$ARCH" != "aarch64" ]]; then
    echo "Warning: this script is tuned for Linux aarch64 / CM5, current arch is: $ARCH" >&2
fi

echo "[xfeat] installing base packages"
$APT_GET update
$APT_GET install -y --no-install-recommends \
    git \
    "$PYTHON_BIN" \
    python3-pip \
    python3-venv \
    libopenblas-dev \
    libjpeg-dev \
    libpng-dev \
    pkg-config

mkdir -p "$(dirname "$VENV_DIR")"
if [[ ! -d "$VENV_DIR" ]]; then
    echo "[xfeat] creating virtualenv: $VENV_DIR"
    "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

source "$VENV_DIR/bin/activate"

echo "[xfeat] upgrading pip tooling"
pip_install --upgrade pip setuptools wheel

echo "[xfeat] installing torch cpu wheels"
pip_install --upgrade \
    torch \
    torchvision \
    --index-url https://download.pytorch.org/whl/cpu

if [[ "$INSTALL_DEMO_DEPS" == "1" ]]; then
    echo "[xfeat] installing demo dependencies (OpenCV download is large; retries enabled)"
    if ! pip_install --upgrade tqdm opencv-contrib-python; then
        echo "[xfeat] demo dependency install failed, likely due to a network reset while downloading OpenCV." >&2
        echo "[xfeat] core XFeat is already installed. Retry later with:" >&2
        echo "  source \"$VENV_DIR/bin/activate\" && python -m pip install --retries \"$PIP_RETRIES\" --timeout \"$PIP_TIMEOUT\" --upgrade opencv-contrib-python tqdm" >&2
        echo "[xfeat] or rerun this installer with --no-demo-deps if you only need inference/smoke-test support." >&2
        exit 1
    fi
else
    echo "[xfeat] installing core utility dependency: tqdm"
    pip_install --upgrade tqdm
fi

mkdir -p "$(dirname "$REPO_DIR")"
if [[ -d "$REPO_DIR/.git" ]]; then
    echo "[xfeat] updating repo: $REPO_DIR"
    git -C "$REPO_DIR" pull --ff-only
else
    echo "[xfeat] cloning repo to: $REPO_DIR"
    git clone "$REPO_URL" "$REPO_DIR"
fi

echo "[xfeat] installation complete"
echo "[xfeat] activate with: source \"$VENV_DIR/bin/activate\""
echo "[xfeat] quick test:"
echo "  python \"$REPO_DIR/minimal_example.py\""
echo "  python \"$(cd "$(dirname "$0")" && pwd)/xfeat_smoke_test.py\" --repo \"$REPO_DIR\""
