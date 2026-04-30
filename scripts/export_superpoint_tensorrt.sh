#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/export_superpoint_tensorrt.sh [--repo PATH] [--onnx PATH] [--engine PATH]
                                          [--width N] [--height N] [--fp32]

Exports the SuperPoint dense convolutional backbone from LightGlue to ONNX and
builds a TensorRT engine. The engine outputs dense detector logits and dense
descriptor maps; NMS/top-k/descriptor sampling are intentionally kept outside
the engine so the worker can preserve the existing keypoint format.
USAGE
}

REPO="${SUPERPOINT_LIGHTGLUE_REPO:-/home/nvidia/LightGlue}"
WIDTH="${SUPERPOINT_TRT_WIDTH:-640}"
HEIGHT="${SUPERPOINT_TRT_HEIGHT:-480}"
FP32=0
ONNX=""
ENGINE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo)
      REPO="$2"
      shift 2
      ;;
    --onnx)
      ONNX="$2"
      shift 2
      ;;
    --engine)
      ENGINE="$2"
      shift 2
      ;;
    --width)
      WIDTH="$2"
      shift 2
      ;;
    --height)
      HEIGHT="$2"
      shift 2
      ;;
    --fp32)
      FP32=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$ONNX" ]]; then
  ONNX="$REPO/weights/superpoint_dense_${WIDTH}x${HEIGHT}.onnx"
fi
if [[ -z "$ENGINE" ]]; then
  if [[ "$FP32" -eq 1 ]]; then
    ENGINE="$REPO/weights/superpoint_dense_${WIDTH}x${HEIGHT}_fp32.engine"
  else
    ENGINE="$REPO/weights/superpoint_dense_${WIDTH}x${HEIGHT}_fp16.engine"
  fi
fi

mkdir -p "$(dirname "$ONNX")" "$(dirname "$ENGINE")"

python3 - "$REPO" "$ONNX" "$WIDTH" "$HEIGHT" <<'PY'
import os
import sys
import types

import torch
from torch import nn

repo, onnx_path, width, height = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
package_dir = os.path.join(repo, "lightglue")
if os.path.isdir(package_dir) and "lightglue" not in sys.modules:
    package = types.ModuleType("lightglue")
    package.__path__ = [package_dir]
    sys.modules["lightglue"] = package
if repo not in sys.path:
    sys.path.insert(0, repo)

from lightglue.superpoint import SuperPoint


class SuperPointDense(nn.Module):
    def __init__(self, model: SuperPoint):
        super().__init__()
        self.model = model

    def forward(self, image):
        m = self.model
        x = m.relu(m.conv1a(image))
        x = m.relu(m.conv1b(x))
        x = m.pool(x)
        x = m.relu(m.conv2a(x))
        x = m.relu(m.conv2b(x))
        x = m.pool(x)
        x = m.relu(m.conv3a(x))
        x = m.relu(m.conv3b(x))
        x = m.pool(x)
        x = m.relu(m.conv4a(x))
        x = m.relu(m.conv4b(x))
        detector = m.convPb(m.relu(m.convPa(x)))
        descriptors = m.convDb(m.relu(m.convDa(x)))
        return detector, descriptors


torch.backends.cudnn.benchmark = True
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = SuperPoint(max_num_keypoints=None).eval().to(device)
dense = SuperPointDense(model).eval().to(device)
dummy = torch.zeros((1, 1, height, width), dtype=torch.float32, device=device)
torch.onnx.export(
    dense,
    dummy,
    onnx_path,
    input_names=["image"],
    output_names=["detector_logits", "dense_descriptors"],
    opset_version=13,
    do_constant_folding=True,
)
print(f"exported {onnx_path}")
PY

TRTEXEC="${TRTEXEC:-}"
if [[ -z "$TRTEXEC" ]]; then
  if command -v trtexec >/dev/null 2>&1; then
    TRTEXEC="$(command -v trtexec)"
  elif [[ -x /usr/src/tensorrt/bin/trtexec ]]; then
    TRTEXEC="/usr/src/tensorrt/bin/trtexec"
  else
    echo "trtexec not found; ONNX exported but engine was not built: $ONNX" >&2
    exit 3
  fi
fi

TRT_ARGS=(--onnx="$ONNX" --saveEngine="$ENGINE" --workspace=2048)
if [[ "$FP32" -eq 0 ]]; then
  TRT_ARGS+=(--fp16)
fi

"$TRTEXEC" "${TRT_ARGS[@]}"
echo "engine: $ENGINE"
