#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/export_lightglue_tensorrt.sh [--repo PATH] [--onnx PATH] [--engine PATH]
                                         [--points N] [--layers N] [--fp32]

Exports LightGlue(superpoint) matching to ONNX and builds a TensorRT engine.
Runtime inference remains C++ TensorRT; Python is used only for this offline export.
USAGE
}

REPO="${SUPERPOINT_LIGHTGLUE_REPO:-/home/nvidia/LightGlue}"
POINTS="${SMART_DRONE_LIGHTGLUE_POINTS:-512}"
LAYERS="${SMART_DRONE_LIGHTGLUE_LAYERS:-6}"
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
    --points)
      POINTS="$2"
      shift 2
      ;;
    --layers)
      LAYERS="$2"
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
  ONNX="$REPO/weights/lightglue_superpoint_${POINTS}.onnx"
fi
if [[ -z "$ENGINE" ]]; then
  if [[ "$FP32" -eq 1 ]]; then
    ENGINE="$REPO/weights/lightglue_superpoint_${POINTS}_fp32.engine"
  else
    ENGINE="$REPO/weights/lightglue_superpoint_${POINTS}_fp16.engine"
  fi
fi

mkdir -p "$(dirname "$ONNX")" "$(dirname "$ENGINE")"

python3 - "$REPO" "$ONNX" "$POINTS" "$LAYERS" <<'PY'
import os
import sys
import types

import torch
import torch.nn.functional as F
from torch import nn

repo, onnx_path, points, layers = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
package_dir = os.path.join(repo, "lightglue")
if os.path.isdir(package_dir) and "lightglue" not in sys.modules:
    package = types.ModuleType("lightglue")
    package.__path__ = [package_dir]
    sys.modules["lightglue"] = package
if repo not in sys.path:
    sys.path.insert(0, repo)

import lightglue.lightglue as lg
from lightglue.lightglue import LightGlue


def rotate_half_onnx(x: torch.Tensor) -> torch.Tensor:
    shape = x.shape
    x = x.reshape(*shape[:-1], shape[-1] // 2, 2)
    x1 = x[..., 0]
    x2 = x[..., 1]
    return torch.stack((-x2, x1), dim=-1).reshape(*shape)


def self_block_forward_onnx(self, x, encoding, mask=None):
    qkv = self.Wqkv(x)
    b, n, _ = qkv.shape
    qkv = qkv.reshape(b, n, self.num_heads, self.head_dim, 3).permute(0, 2, 1, 3, 4)
    q, k, v = qkv[..., 0], qkv[..., 1], qkv[..., 2]
    q = (q * encoding[0]) + (rotate_half_onnx(q) * encoding[1])
    k = (k * encoding[0]) + (rotate_half_onnx(k) * encoding[1])
    context = self.inner_attn(q, k, v, mask=mask)
    message = self.out_proj(context.permute(0, 2, 1, 3).reshape(b, n, self.embed_dim))
    return x + self.ffn(torch.cat([x, message], dim=2))


def cross_block_forward_onnx(self, x0, x1, mask=None):
    qk0, qk1 = self.map_(self.to_qk, x0, x1)
    v0, v1 = self.map_(self.to_v, x0, x1)
    b, n0, _ = qk0.shape
    _, n1, _ = qk1.shape
    head_dim = qk0.shape[-1] // self.heads
    qk0 = qk0.reshape(b, n0, self.heads, head_dim).permute(0, 2, 1, 3)
    qk1 = qk1.reshape(b, n1, self.heads, head_dim).permute(0, 2, 1, 3)
    v0 = v0.reshape(b, n0, self.heads, head_dim).permute(0, 2, 1, 3)
    v1 = v1.reshape(b, n1, self.heads, head_dim).permute(0, 2, 1, 3)
    qk0, qk1 = qk0 * self.scale**0.5, qk1 * self.scale**0.5
    sim = torch.matmul(qk0, qk1.permute(0, 1, 3, 2))
    if mask is not None:
        sim = sim.masked_fill(~mask, -float("inf"))
    attn01 = torch.softmax(sim, dim=3)
    attn10 = torch.softmax(sim.permute(0, 1, 3, 2).contiguous(), dim=3)
    m0 = torch.matmul(attn01, v1)
    m1 = torch.matmul(attn10, v0)
    inner_dim = self.heads * head_dim
    m0 = self.to_out(m0.permute(0, 2, 1, 3).reshape(b, n0, inner_dim))
    m1 = self.to_out(m1.permute(0, 2, 1, 3).reshape(b, n1, inner_dim))
    x0 = x0 + self.ffn(torch.cat([x0, m0], dim=2))
    x1 = x1 + self.ffn(torch.cat([x1, m1], dim=2))
    return x0, x1


def attention_forward_onnx(self, q, k, v, mask=None):
    scale = q.shape[-1] ** -0.5
    sim = torch.matmul(q, k.permute(0, 1, 3, 2)) * scale
    if mask is not None:
        sim = sim.masked_fill(~mask, -float("inf"))
    attn = torch.softmax(sim, dim=3)
    return torch.matmul(attn, v)


lg.rotate_half = rotate_half_onnx
lg.Attention.forward = attention_forward_onnx
lg.SelfBlock.forward = self_block_forward_onnx
lg.CrossBlock.forward = cross_block_forward_onnx


class OnnxLayerNorm(nn.Module):
    def __init__(self, source: nn.LayerNorm):
        super().__init__()
        self.eps = float(source.eps)
        self.weight = nn.Parameter(source.weight.detach().clone())
        self.bias = nn.Parameter(source.bias.detach().clone())

    def forward(self, x):
        mean = x.mean(dim=2, keepdim=True)
        centered = x - mean
        var = (centered * centered).mean(dim=2, keepdim=True)
        return centered * torch.rsqrt(var + self.eps) * self.weight + self.bias


def replace_layer_norm(module: nn.Module):
    for name, child in list(module.named_children()):
        if isinstance(child, nn.LayerNorm):
            setattr(module, name, OnnxLayerNorm(child))
        else:
            replace_layer_norm(child)


class LightGlueStatic(nn.Module):
    def __init__(self, point_count: int, n_layers: int):
        super().__init__()
        self.point_count = point_count
        self.matcher = LightGlue(
            features="superpoint",
            n_layers=n_layers,
            flash=False,
            mp=False,
            depth_confidence=-1,
            width_confidence=-1,
            filter_threshold=0.1,
        ).eval()

    def forward(self, keypoints0, keypoints1, descriptors0, descriptors1, image_size0, image_size1):
        m = self.matcher
        kpts0 = lg.normalize_keypoints(keypoints0, image_size0).clone()
        kpts1 = lg.normalize_keypoints(keypoints1, image_size1).clone()
        desc0 = m.input_proj(descriptors0.detach().contiguous())
        desc1 = m.input_proj(descriptors1.detach().contiguous())
        encoding0 = m.posenc(kpts0)
        encoding1 = m.posenc(kpts1)
        for layer in m.transformers:
            desc0, desc1 = layer(desc0, desc1, encoding0, encoding1)
        assignment = m.log_assignment[-1]
        mdesc0 = assignment.final_proj(desc0)
        mdesc1 = assignment.final_proj(desc1)
        scale = float(mdesc0.shape[-1]) ** 0.25
        mdesc0 = mdesc0 / scale
        mdesc1 = mdesc1 / scale
        sim = torch.matmul(mdesc0, mdesc1.permute(0, 2, 1))
        z0 = assignment.matchability(desc0)
        z1 = assignment.matchability(desc1)
        scores0 = torch.log_softmax(sim, dim=2)
        scores1 = torch.log_softmax(sim.permute(0, 2, 1).contiguous(), dim=2).permute(0, 2, 1)
        certainties = F.logsigmoid(z0) + F.logsigmoid(z1).permute(0, 2, 1)
        return (scores0 + scores1 + certainties).exp()


torch.backends.cudnn.benchmark = True
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = LightGlueStatic(points, layers).eval().to(device)
replace_layer_norm(model)
dummy_kpts = torch.zeros((1, points, 2), dtype=torch.float32, device=device)
dummy_desc = torch.zeros((1, points, 256), dtype=torch.float32, device=device)
dummy_size = torch.tensor([[640.0, 480.0]], dtype=torch.float32, device=device)

torch.onnx.export(
    model,
    (dummy_kpts, dummy_kpts, dummy_desc, dummy_desc, dummy_size, dummy_size),
    onnx_path,
    input_names=[
        "keypoints0",
        "keypoints1",
        "descriptors0",
        "descriptors1",
        "image_size0",
        "image_size1",
    ],
    output_names=["assignment_scores"],
    opset_version=17,
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

TRT_ARGS=(--onnx="$ONNX" --saveEngine="$ENGINE" --workspace=4096)
if [[ "$FP32" -eq 0 ]]; then
  TRT_ARGS+=(--fp16)
fi

"$TRTEXEC" "${TRT_ARGS[@]}"
echo "engine: $ENGINE"
