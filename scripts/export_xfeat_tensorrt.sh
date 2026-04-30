#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./scripts/export_xfeat_tensorrt.sh [--repo PATH] [--weights PATH] [--onnx PATH] [--engine PATH] [--fp32]
                                      [--min-shape SHAPE] [--opt-shape SHAPE] [--max-shape SHAPE]

Exports accelerated_features XFeat to ONNX and builds a TensorRT engine with trtexec.
Run this on Jetson where TensorRT and the accelerated_features Python repo are installed.

Options:
  --repo PATH     accelerated_features repo; default: /home/nvidia/accelerated_features
  --weights PATH  xfeat.pt state_dict; default: <repo>/weights/xfeat.pt
  --onnx PATH     output ONNX path; default: <repo>/weights/xfeat.onnx
  --engine PATH   output engine path; default: <repo>/weights/xfeat_trt_fp16.engine
  --min-shape S   min TensorRT input shape; default: 1x1x480x640
  --opt-shape S   opt TensorRT input shape and ONNX export shape; default: 2x1x480x640
  --max-shape S   max TensorRT input shape; default: 2x1x480x640
  --fp32          build FP32 engine instead of FP16
EOF
}

REPO="/home/nvidia/accelerated_features"
WEIGHTS=""
ONNX=""
ENGINE=""
FP16=1
MIN_SHAPE="1x1x480x640"
OPT_SHAPE="2x1x480x640"
MAX_SHAPE="2x1x480x640"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --repo)
            REPO="$2"
            shift
            ;;
        --repo=*)
            REPO="${1#--repo=}"
            ;;
        --weights)
            WEIGHTS="$2"
            shift
            ;;
        --weights=*)
            WEIGHTS="${1#--weights=}"
            ;;
        --onnx)
            ONNX="$2"
            shift
            ;;
        --onnx=*)
            ONNX="${1#--onnx=}"
            ;;
        --engine)
            ENGINE="$2"
            shift
            ;;
        --engine=*)
            ENGINE="${1#--engine=}"
            ;;
        --min-shape)
            MIN_SHAPE="$2"
            shift
            ;;
        --min-shape=*)
            MIN_SHAPE="${1#--min-shape=}"
            ;;
        --opt-shape)
            OPT_SHAPE="$2"
            shift
            ;;
        --opt-shape=*)
            OPT_SHAPE="${1#--opt-shape=}"
            ;;
        --max-shape)
            MAX_SHAPE="$2"
            shift
            ;;
        --max-shape=*)
            MAX_SHAPE="${1#--max-shape=}"
            ;;
        --fp32)
            FP16=0
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

WEIGHTS="${WEIGHTS:-$REPO/weights/xfeat.pt}"
ONNX="${ONNX:-$REPO/weights/xfeat.onnx}"
if [ -z "$ENGINE" ]; then
    if [ "$FP16" -eq 1 ]; then
        ENGINE="$REPO/weights/xfeat_trt_fp16.engine"
    else
        ENGINE="$REPO/weights/xfeat_trt_fp32.engine"
    fi
fi

TRTEXEC="${TRTEXEC:-}"
if [ -z "$TRTEXEC" ]; then
    if command -v trtexec >/dev/null 2>&1; then
        TRTEXEC="$(command -v trtexec)"
    elif [ -x /usr/src/tensorrt/bin/trtexec ]; then
        TRTEXEC=/usr/src/tensorrt/bin/trtexec
    else
        echo "trtexec not found" >&2
        exit 1
    fi
fi

mkdir -p "$(dirname "$ONNX")" "$(dirname "$ENGINE")"

python3 - "$REPO" "$WEIGHTS" "$ONNX" "$OPT_SHAPE" <<'PY'
import pathlib
import sys

import torch

repo = pathlib.Path(sys.argv[1]).resolve()
weights = pathlib.Path(sys.argv[2]).resolve()
onnx = pathlib.Path(sys.argv[3]).resolve()
shape = [int(part) for part in sys.argv[4].split("x")]
if len(shape) != 4:
    raise ValueError(f"invalid shape: {sys.argv[4]}")

sys.path.insert(0, str(repo))
from modules.model import XFeatModel

device = torch.device("cpu")
model = XFeatModel().to(device).eval()
state = torch.load(str(weights), map_location=device)
model.load_state_dict(state)

example = torch.zeros(*shape, device=device)
with torch.no_grad():
    torch.onnx.export(
        model,
        example,
        str(onnx),
        export_params=True,
        opset_version=17,
        do_constant_folding=True,
        input_names=["images"],
        output_names=["dense_features", "keypoint_logits", "reliability"],
        dynamic_axes={
            "images": {0: "batch"},
            "dense_features": {0: "batch"},
            "keypoint_logits": {0: "batch"},
            "reliability": {0: "batch"},
        },
    )
print(f"exported {onnx}")
PY

trt_args=(
    "--onnx=$ONNX"
    "--saveEngine=$ENGINE"
    "--minShapes=images:$MIN_SHAPE"
    "--optShapes=images:$OPT_SHAPE"
    "--maxShapes=images:$MAX_SHAPE"
    "--workspace=1024"
)
if [ "$FP16" -eq 1 ]; then
    trt_args+=("--fp16")
fi

"$TRTEXEC" "${trt_args[@]}"
echo "exported $ENGINE"
