#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/export_dpvo_tensorrt.sh [--repo PATH] [--weights PATH]
                                   [--width N] [--height N] [--max-edges N]
                                   [--patch-onnx PATH] [--patch-engine PATH]
                                   [--update-onnx PATH] [--update-engine PATH]
                                   [--update-preagg-onnx PATH] [--update-preagg-engine PATH]
                                   [--update-postagg-onnx PATH] [--update-postagg-engine PATH]
                                   [--fp32] [--skip-download]

Exports the TensorRT-ready neural pieces used by the native DPVO backend:

  1. dpvo_patchifier_fp16.engine
     Runs the DPVO fnet/inet image encoders. Native C++/CUDA still owns patch
     centroid selection and patch sampling.

  2. dpvo_update_fp16.engine
     Runs the TensorRT-friendly update core. Native C++/CUDA still owns dynamic
     graph neighbor lookup, scatter aggregation, correlation, BA, and SE3 state.

  3. dpvo_update_preagg_fp16.engine / dpvo_update_postagg_fp16.engine
     Splits the official update operator around SoftAgg so native C++ can perform
     the runtime scatter-softmax/scatter-sum reductions without dropping the
     learned agg_kk/agg_ij weights.

Python/PyTorch are used only for offline ONNX export. The smart_drone runtime
loads only TensorRT engines and does not start Python.
USAGE
}

REPO="${DPVO_REPO:-/home/nvidia/DPVO}"
WEIGHTS="${DPVO_WEIGHTS:-}"
WIDTH="${DPVO_TRT_WIDTH:-640}"
HEIGHT="${DPVO_TRT_HEIGHT:-400}"
MAX_EDGES="${DPVO_TRT_MAX_EDGES:-4096}"
FP32=0
SKIP_DOWNLOAD=0
PATCH_ONNX=""
PATCH_ENGINE=""
UPDATE_ONNX=""
UPDATE_ENGINE=""
UPDATE_PREAGG_ONNX=""
UPDATE_PREAGG_ENGINE=""
UPDATE_POSTAGG_ONNX=""
UPDATE_POSTAGG_ENGINE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo)
      REPO="$2"
      shift 2
      ;;
    --weights)
      WEIGHTS="$2"
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
    --max-edges)
      MAX_EDGES="$2"
      shift 2
      ;;
    --patch-onnx)
      PATCH_ONNX="$2"
      shift 2
      ;;
    --patch-engine)
      PATCH_ENGINE="$2"
      shift 2
      ;;
    --update-onnx)
      UPDATE_ONNX="$2"
      shift 2
      ;;
    --update-engine)
      UPDATE_ENGINE="$2"
      shift 2
      ;;
    --update-preagg-onnx)
      UPDATE_PREAGG_ONNX="$2"
      shift 2
      ;;
    --update-preagg-engine)
      UPDATE_PREAGG_ENGINE="$2"
      shift 2
      ;;
    --update-postagg-onnx)
      UPDATE_POSTAGG_ONNX="$2"
      shift 2
      ;;
    --update-postagg-engine)
      UPDATE_POSTAGG_ENGINE="$2"
      shift 2
      ;;
    --fp32)
      FP32=1
      shift
      ;;
    --skip-download)
      SKIP_DOWNLOAD=1
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

if [[ -z "$WEIGHTS" ]]; then
  WEIGHTS="$REPO/dpvo.pth"
fi
if [[ -z "$PATCH_ONNX" ]]; then
  PATCH_ONNX="$REPO/weights/dpvo_patchifier_${WIDTH}x${HEIGHT}.onnx"
fi
if [[ -z "$UPDATE_ONNX" ]]; then
  UPDATE_ONNX="$REPO/weights/dpvo_update_core_edges${MAX_EDGES}.onnx"
fi
if [[ -z "$UPDATE_PREAGG_ONNX" ]]; then
  UPDATE_PREAGG_ONNX="$REPO/weights/dpvo_update_preagg_edges${MAX_EDGES}.onnx"
fi
if [[ -z "$UPDATE_POSTAGG_ONNX" ]]; then
  UPDATE_POSTAGG_ONNX="$REPO/weights/dpvo_update_postagg_edges${MAX_EDGES}.onnx"
fi
if [[ -z "$PATCH_ENGINE" ]]; then
  if [[ "$FP32" -eq 1 ]]; then
    PATCH_ENGINE="$REPO/weights/dpvo_patchifier_fp32.engine"
  else
    PATCH_ENGINE="$REPO/weights/dpvo_patchifier_fp16.engine"
  fi
fi
if [[ -z "$UPDATE_ENGINE" ]]; then
  if [[ "$FP32" -eq 1 ]]; then
    UPDATE_ENGINE="$REPO/weights/dpvo_update_fp32.engine"
  else
    UPDATE_ENGINE="$REPO/weights/dpvo_update_fp16.engine"
  fi
fi
if [[ -z "$UPDATE_PREAGG_ENGINE" ]]; then
  if [[ "$FP32" -eq 1 ]]; then
    UPDATE_PREAGG_ENGINE="$REPO/weights/dpvo_update_preagg_fp32.engine"
  else
    UPDATE_PREAGG_ENGINE="$REPO/weights/dpvo_update_preagg_fp16.engine"
  fi
fi
if [[ -z "$UPDATE_POSTAGG_ENGINE" ]]; then
  if [[ "$FP32" -eq 1 ]]; then
    UPDATE_POSTAGG_ENGINE="$REPO/weights/dpvo_update_postagg_fp32.engine"
  else
    UPDATE_POSTAGG_ENGINE="$REPO/weights/dpvo_update_postagg_fp16.engine"
  fi
fi
if (( MAX_EDGES < 1 )); then
  echo "--max-edges must be at least 1" >&2
  exit 2
fi
OPT_EDGES="$MAX_EDGES"
if (( OPT_EDGES > 1024 )); then
  OPT_EDGES=1024
fi

if [[ ! -d "$REPO/.git" && ! -f "$REPO/dpvo/net.py" ]]; then
  if [[ "$SKIP_DOWNLOAD" -eq 1 ]]; then
    echo "DPVO repo is missing and --skip-download was set: $REPO" >&2
    exit 2
  fi
  if [[ -e "$REPO" && -n "$(find "$REPO" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]]; then
    echo "DPVO repo path exists but is not a DPVO checkout: $REPO" >&2
    exit 2
  fi
  mkdir -p "$(dirname "$REPO")"
  rm -rf "$REPO"
  git clone --depth 1 https://github.com/princeton-vl/DPVO.git "$REPO"
fi

mkdir -p "$(dirname "$PATCH_ONNX")" "$(dirname "$PATCH_ENGINE")" \
  "$(dirname "$UPDATE_ONNX")" "$(dirname "$UPDATE_ENGINE")" \
  "$(dirname "$UPDATE_PREAGG_ONNX")" "$(dirname "$UPDATE_PREAGG_ENGINE")" \
  "$(dirname "$UPDATE_POSTAGG_ONNX")" "$(dirname "$UPDATE_POSTAGG_ENGINE")"

if [[ ! -f "$WEIGHTS" ]]; then
  if [[ "$SKIP_DOWNLOAD" -eq 1 ]]; then
    echo "DPVO weights are missing and --skip-download was set: $WEIGHTS" >&2
    exit 2
  fi
  tmp_zip="$REPO/models.zip"
  echo "downloading DPVO models.zip to $tmp_zip"
  if ! wget -O "$tmp_zip" "https://drive.google.com/uc?export=download&id=1dRqftpImtHbbIPNBIseCv9EvrlHEnjhX"; then
    echo "Google Drive download failed; trying Dropbox mirror" >&2
    wget -O "$tmp_zip" "https://www.dropbox.com/s/nap0u8zslspdwm4/models.zip?dl=1"
  fi
  python3 - "$tmp_zip" "$REPO" "$WEIGHTS" <<'PY'
import sys
import zipfile
from pathlib import Path

zip_path = Path(sys.argv[1])
repo = Path(sys.argv[2])
weights = Path(sys.argv[3])

with zipfile.ZipFile(zip_path) as zf:
    names = zf.namelist()
    candidates = [n for n in names if n.endswith("dpvo.pth")]
    if not candidates:
        raise SystemExit("models.zip did not contain dpvo.pth")
    source = candidates[0]
    weights.parent.mkdir(parents=True, exist_ok=True)
    with zf.open(source) as src, open(weights, "wb") as dst:
        dst.write(src.read())
    print(f"extracted {source} -> {weights}")
PY
fi

python3 - "$REPO" "$WEIGHTS" "$PATCH_ONNX" "$UPDATE_ONNX" "$UPDATE_PREAGG_ONNX" "$UPDATE_POSTAGG_ONNX" "$WIDTH" "$HEIGHT" "$MAX_EDGES" <<'PY'
import os
import sys
from collections import OrderedDict

import torch
from torch import nn

repo, weights_path, patch_onnx, update_onnx, update_preagg_onnx, update_postagg_onnx = (
    sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6]
)
width, height, max_edges = int(sys.argv[7]), int(sys.argv[8]), int(sys.argv[9])

if repo not in sys.path:
    sys.path.insert(0, repo)

from dpvo.extractor import BasicEncoder4

DIM = 384
CORR_DIM = 2 * 49 * 3 * 3


def load_state(path):
    state = torch.load(path, map_location="cpu")
    out = OrderedDict()
    for k, v in state.items():
        key = k.replace("module.", "")
        if "update.lmbda" not in key:
            out[key] = v
    return out


def sub_state(state, prefix):
    result = OrderedDict()
    for k, v in state.items():
        if k.startswith(prefix):
            result[k[len(prefix):]] = v
    return result


class DpvoPatchifierEncoders(nn.Module):
    def __init__(self, fnet, inet):
        super().__init__()
        self.fnet = fnet
        self.inet = inet

    def forward(self, image):
        images = 2.0 * (image.unsqueeze(1) / 255.0) - 0.5
        fmap = self.fnet(images) / 4.0
        imap = self.inet(images) / 4.0
        return fmap[:, 0], imap[:, 0]


class OnnxLayerNorm1D(nn.Module):
    def __init__(self, dim, eps=1e-3):
        super().__init__()
        self.eps = eps
        self.weight = nn.Parameter(torch.ones(dim))
        self.bias = nn.Parameter(torch.zeros(dim))

    def forward(self, x):
        mean = x.mean(dim=-1, keepdim=True)
        centered = x - mean
        var = (centered * centered).mean(dim=-1, keepdim=True)
        return centered * torch.rsqrt(var + self.eps) * self.weight + self.bias


class GatedResidual(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.gate = nn.Sequential(nn.Linear(dim, dim), nn.Sigmoid())
        self.res = nn.Sequential(nn.Linear(dim, dim), nn.ReLU(inplace=True), nn.Linear(dim, dim))

    def forward(self, x):
        return x + self.gate(x) * self.res(x)


class DpvoUpdateCore(nn.Module):
    def __init__(self):
        super().__init__()
        self.c1 = nn.Sequential(nn.Linear(DIM, DIM), nn.ReLU(inplace=True), nn.Linear(DIM, DIM))
        self.c2 = nn.Sequential(nn.Linear(DIM, DIM), nn.ReLU(inplace=True), nn.Linear(DIM, DIM))
        self.norm = OnnxLayerNorm1D(DIM, eps=1e-3)
        self.gru = nn.Sequential(
            OnnxLayerNorm1D(DIM, eps=1e-3),
            GatedResidual(DIM),
            OnnxLayerNorm1D(DIM, eps=1e-3),
            GatedResidual(DIM),
        )
        self.corr = nn.Sequential(
            nn.Linear(CORR_DIM, DIM),
            nn.ReLU(inplace=True),
            nn.Linear(DIM, DIM),
            OnnxLayerNorm1D(DIM, eps=1e-3),
            nn.ReLU(inplace=True),
            nn.Linear(DIM, DIM),
        )
        self.d0 = nn.Linear(DIM, 2)
        self.w0 = nn.Linear(DIM, 2)

    def forward(self, net, inp, corr, prev_net, next_net, prev_mask, next_mask):
        x = net + inp + self.corr(corr)
        x = self.norm(x)
        x = x + self.c1(prev_mask * prev_net)
        x = x + self.c2(next_mask * next_net)
        x = self.gru(x)
        delta = self.d0(torch.relu(x))
        weight = torch.sigmoid(self.w0(torch.relu(x)))
        return x, delta, weight


class DpvoUpdatePreAgg(nn.Module):
    def __init__(self):
        super().__init__()
        self.c1 = nn.Sequential(nn.Linear(DIM, DIM), nn.ReLU(inplace=True), nn.Linear(DIM, DIM))
        self.c2 = nn.Sequential(nn.Linear(DIM, DIM), nn.ReLU(inplace=True), nn.Linear(DIM, DIM))
        self.norm = OnnxLayerNorm1D(DIM, eps=1e-3)
        self.corr = nn.Sequential(
            nn.Linear(CORR_DIM, DIM),
            nn.ReLU(inplace=True),
            nn.Linear(DIM, DIM),
            OnnxLayerNorm1D(DIM, eps=1e-3),
            nn.ReLU(inplace=True),
            nn.Linear(DIM, DIM),
        )
        self.agg_kk_f = nn.Linear(DIM, DIM)
        self.agg_kk_g = nn.Linear(DIM, DIM)
        self.agg_ij_f = nn.Linear(DIM, DIM)
        self.agg_ij_g = nn.Linear(DIM, DIM)

    def forward(self, net, inp, corr, prev_net, next_net, prev_mask, next_mask):
        x = net + inp + self.corr(corr)
        x = self.norm(x)
        x = x + self.c1(prev_mask * prev_net)
        x = x + self.c2(next_mask * next_net)
        return x, self.agg_kk_f(x), self.agg_kk_g(x), self.agg_ij_f(x), self.agg_ij_g(x)


class DpvoUpdatePostAgg(nn.Module):
    def __init__(self):
        super().__init__()
        self.agg_kk_h = nn.Linear(DIM, DIM)
        self.agg_ij_h = nn.Linear(DIM, DIM)
        self.gru = nn.Sequential(
            OnnxLayerNorm1D(DIM, eps=1e-3),
            GatedResidual(DIM),
            OnnxLayerNorm1D(DIM, eps=1e-3),
            GatedResidual(DIM),
        )
        self.d0 = nn.Linear(DIM, 2)
        self.w0 = nn.Linear(DIM, 2)

    def forward(self, base, agg_kk_y, agg_ij_y):
        x = base + self.agg_kk_h(agg_kk_y) + self.agg_ij_h(agg_ij_y)
        x = self.gru(x)
        delta = self.d0(torch.relu(x))
        weight = torch.sigmoid(self.w0(torch.relu(x)))
        return x, delta, weight


def adapt_update_core_state(update_state):
    adapted = OrderedDict()
    for k, v in update_state.items():
        if k.startswith("d.1."):
            adapted["d0." + k[len("d.1."):]] = v
        elif k.startswith("w.1."):
            adapted["w0." + k[len("w.1."):]] = v
        elif k.startswith("agg_"):
            continue
        else:
            adapted[k] = v
    return adapted


def adapt_preagg_state(update_state):
    adapted = OrderedDict()
    for k, v in update_state.items():
        if k.startswith("agg_kk.f."):
            adapted["agg_kk_f." + k[len("agg_kk.f."):]] = v
        elif k.startswith("agg_kk.g."):
            adapted["agg_kk_g." + k[len("agg_kk.g."):]] = v
        elif k.startswith("agg_ij.f."):
            adapted["agg_ij_f." + k[len("agg_ij.f."):]] = v
        elif k.startswith("agg_ij.g."):
            adapted["agg_ij_g." + k[len("agg_ij.g."):]] = v
        elif k.startswith("agg_") or k.startswith("gru.") or k.startswith("d.") or k.startswith("w."):
            continue
        else:
            adapted[k] = v
    return adapted


def adapt_postagg_state(update_state):
    adapted = OrderedDict()
    for k, v in update_state.items():
        if k.startswith("agg_kk.h."):
            adapted["agg_kk_h." + k[len("agg_kk.h."):]] = v
        elif k.startswith("agg_ij.h."):
            adapted["agg_ij_h." + k[len("agg_ij.h."):]] = v
        elif k.startswith("d.1."):
            adapted["d0." + k[len("d.1."):]] = v
        elif k.startswith("w.1."):
            adapted["w0." + k[len("w.1."):]] = v
        elif k.startswith("gru."):
            adapted[k] = v
    return adapted


state = load_state(weights_path)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
torch.backends.cudnn.benchmark = True

fnet = BasicEncoder4(output_dim=128, norm_fn="instance").eval()
inet = BasicEncoder4(output_dim=DIM, norm_fn="none").eval()
fnet.load_state_dict(sub_state(state, "patchify.fnet."), strict=True)
inet.load_state_dict(sub_state(state, "patchify.inet."), strict=True)
patchifier = DpvoPatchifierEncoders(fnet, inet).eval().to(device)
dummy_image = torch.zeros((1, 3, height, width), dtype=torch.float32, device=device)
torch.onnx.export(
    patchifier,
    dummy_image,
    patch_onnx,
    input_names=["image"],
    output_names=["fmap", "imap"],
    dynamic_axes={
        "image": {0: "batch"},
        "fmap": {0: "batch"},
        "imap": {0: "batch"},
    },
    opset_version=17,
    do_constant_folding=True,
)
print(f"exported {patch_onnx}")

update = DpvoUpdateCore().eval()
update_state = sub_state(state, "update.")
missing, unexpected = update.load_state_dict(adapt_update_core_state(update_state), strict=False)
allowed_missing = set()
if unexpected:
    raise RuntimeError(f"unexpected update weights: {unexpected}")
if any(k not in allowed_missing for k in missing):
    raise RuntimeError(f"missing update weights: {missing}")
update = update.to(device)

preagg = DpvoUpdatePreAgg().eval()
missing, unexpected = preagg.load_state_dict(adapt_preagg_state(update_state), strict=True)
if missing or unexpected:
    raise RuntimeError(f"preagg state mismatch missing={missing} unexpected={unexpected}")
preagg = preagg.to(device)

postagg = DpvoUpdatePostAgg().eval()
missing, unexpected = postagg.load_state_dict(adapt_postagg_state(update_state), strict=True)
if missing or unexpected:
    raise RuntimeError(f"postagg state mismatch missing={missing} unexpected={unexpected}")
postagg = postagg.to(device)

edge_count = max(1, min(max_edges, 1024))
dummy = torch.zeros((1, edge_count, DIM), dtype=torch.float32, device=device)
dummy_corr = torch.zeros((1, edge_count, CORR_DIM), dtype=torch.float32, device=device)
dummy_mask = torch.zeros((1, edge_count, 1), dtype=torch.float32, device=device)
torch.onnx.export(
    update,
    (dummy, dummy, dummy_corr, dummy, dummy, dummy_mask, dummy_mask),
    update_onnx,
    input_names=["net", "inp", "corr", "prev_net", "next_net", "prev_mask", "next_mask"],
    output_names=["updated_net", "delta", "weight"],
    dynamic_axes={
        "net": {1: "edges"},
        "inp": {1: "edges"},
        "corr": {1: "edges"},
        "prev_net": {1: "edges"},
        "next_net": {1: "edges"},
        "prev_mask": {1: "edges"},
        "next_mask": {1: "edges"},
        "updated_net": {1: "edges"},
        "delta": {1: "edges"},
        "weight": {1: "edges"},
    },
    opset_version=17,
    do_constant_folding=True,
)
print(f"exported {update_onnx}")

torch.onnx.export(
    preagg,
    (dummy, dummy, dummy_corr, dummy, dummy, dummy_mask, dummy_mask),
    update_preagg_onnx,
    input_names=["net", "inp", "corr", "prev_net", "next_net", "prev_mask", "next_mask"],
    output_names=["base_net", "agg_kk_f", "agg_kk_g", "agg_ij_f", "agg_ij_g"],
    dynamic_axes={
        "net": {1: "edges"},
        "inp": {1: "edges"},
        "corr": {1: "edges"},
        "prev_net": {1: "edges"},
        "next_net": {1: "edges"},
        "prev_mask": {1: "edges"},
        "next_mask": {1: "edges"},
        "base_net": {1: "edges"},
        "agg_kk_f": {1: "edges"},
        "agg_kk_g": {1: "edges"},
        "agg_ij_f": {1: "edges"},
        "agg_ij_g": {1: "edges"},
    },
    opset_version=17,
    do_constant_folding=True,
)
print(f"exported {update_preagg_onnx}")

torch.onnx.export(
    postagg,
    (dummy, dummy, dummy),
    update_postagg_onnx,
    input_names=["base_net", "agg_kk_y", "agg_ij_y"],
    output_names=["updated_net", "delta", "weight"],
    dynamic_axes={
        "base_net": {1: "edges"},
        "agg_kk_y": {1: "edges"},
        "agg_ij_y": {1: "edges"},
        "updated_net": {1: "edges"},
        "delta": {1: "edges"},
        "weight": {1: "edges"},
    },
    opset_version=17,
    do_constant_folding=True,
)
print(f"exported {update_postagg_onnx}")
PY

TRTEXEC="${TRTEXEC:-}"
if [[ -z "$TRTEXEC" ]]; then
  if command -v trtexec >/dev/null 2>&1; then
    TRTEXEC="$(command -v trtexec)"
  elif [[ -x /usr/src/tensorrt/bin/trtexec ]]; then
    TRTEXEC="/usr/src/tensorrt/bin/trtexec"
  else
    echo "trtexec not found; ONNX exported but engines were not built" >&2
    exit 3
  fi
fi

PATCH_TRT_ARGS=(
  --onnx="$PATCH_ONNX"
  --saveEngine="$PATCH_ENGINE"
  --workspace=2048
  --minShapes="image:1x3x${HEIGHT}x${WIDTH}"
  --optShapes="image:1x3x${HEIGHT}x${WIDTH}"
  --maxShapes="image:1x3x${HEIGHT}x${WIDTH}"
)
UPDATE_TRT_ARGS=(
  --onnx="$UPDATE_ONNX"
  --saveEngine="$UPDATE_ENGINE"
  --workspace=2048
  --minShapes="net:1x1x384,inp:1x1x384,corr:1x1x882,prev_net:1x1x384,next_net:1x1x384,prev_mask:1x1x1,next_mask:1x1x1"
  --optShapes="net:1x${OPT_EDGES}x384,inp:1x${OPT_EDGES}x384,corr:1x${OPT_EDGES}x882,prev_net:1x${OPT_EDGES}x384,next_net:1x${OPT_EDGES}x384,prev_mask:1x${OPT_EDGES}x1,next_mask:1x${OPT_EDGES}x1"
  --maxShapes="net:1x${MAX_EDGES}x384,inp:1x${MAX_EDGES}x384,corr:1x${MAX_EDGES}x882,prev_net:1x${MAX_EDGES}x384,next_net:1x${MAX_EDGES}x384,prev_mask:1x${MAX_EDGES}x1,next_mask:1x${MAX_EDGES}x1"
)
UPDATE_PREAGG_TRT_ARGS=(
  --onnx="$UPDATE_PREAGG_ONNX"
  --saveEngine="$UPDATE_PREAGG_ENGINE"
  --workspace=2048
  --minShapes="net:1x1x384,inp:1x1x384,corr:1x1x882,prev_net:1x1x384,next_net:1x1x384,prev_mask:1x1x1,next_mask:1x1x1"
  --optShapes="net:1x${OPT_EDGES}x384,inp:1x${OPT_EDGES}x384,corr:1x${OPT_EDGES}x882,prev_net:1x${OPT_EDGES}x384,next_net:1x${OPT_EDGES}x384,prev_mask:1x${OPT_EDGES}x1,next_mask:1x${OPT_EDGES}x1"
  --maxShapes="net:1x${MAX_EDGES}x384,inp:1x${MAX_EDGES}x384,corr:1x${MAX_EDGES}x882,prev_net:1x${MAX_EDGES}x384,next_net:1x${MAX_EDGES}x384,prev_mask:1x${MAX_EDGES}x1,next_mask:1x${MAX_EDGES}x1"
)
UPDATE_POSTAGG_TRT_ARGS=(
  --onnx="$UPDATE_POSTAGG_ONNX"
  --saveEngine="$UPDATE_POSTAGG_ENGINE"
  --workspace=2048
  --minShapes="base_net:1x1x384,agg_kk_y:1x1x384,agg_ij_y:1x1x384"
  --optShapes="base_net:1x${OPT_EDGES}x384,agg_kk_y:1x${OPT_EDGES}x384,agg_ij_y:1x${OPT_EDGES}x384"
  --maxShapes="base_net:1x${MAX_EDGES}x384,agg_kk_y:1x${MAX_EDGES}x384,agg_ij_y:1x${MAX_EDGES}x384"
)
if [[ "$FP32" -eq 0 ]]; then
  PATCH_TRT_ARGS+=(--fp16)
  UPDATE_TRT_ARGS+=(--fp16)
  UPDATE_PREAGG_TRT_ARGS+=(--fp16)
  UPDATE_POSTAGG_TRT_ARGS+=(--fp16)
fi

"$TRTEXEC" "${PATCH_TRT_ARGS[@]}"
"$TRTEXEC" "${UPDATE_TRT_ARGS[@]}"
"$TRTEXEC" "${UPDATE_PREAGG_TRT_ARGS[@]}"
"$TRTEXEC" "${UPDATE_POSTAGG_TRT_ARGS[@]}"

echo "patchifier_engine: $PATCH_ENGINE"
echo "update_engine: $UPDATE_ENGINE"
echo "update_preagg_engine: $UPDATE_PREAGG_ENGINE"
echo "update_postagg_engine: $UPDATE_POSTAGG_ENGINE"
