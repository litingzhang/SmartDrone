#!/usr/bin/env python3

import argparse
import contextlib
import importlib.util
import io
import os
import struct
import sys

import numpy as np
import torch


REQUEST_HEADER = struct.Struct("<IIII")
RESPONSE_HEADER = struct.Struct("<III")
READY_MAGIC = b"XFWKRDY1"


def _write_stderr(message: str) -> None:
    sys.stderr.write(message + "\n")
    sys.stderr.flush()


def _read_exact(stream, size: int) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < size:
        block = stream.read(size - len(chunks))
        if not block:
            return None
        chunks.extend(block)
    return bytes(chunks)


def _load_xfeat_class(repo_path: str):
    repo_path = os.path.abspath(repo_path)
    modules_dir = os.path.join(repo_path, "modules")
    xfeat_path = os.path.join(modules_dir, "xfeat.py")
    if not os.path.isfile(xfeat_path):
        raise FileNotFoundError(f"xfeat.py not found under repo: {xfeat_path}")
    if repo_path not in sys.path:
        sys.path.insert(0, repo_path)

    spec = importlib.util.spec_from_file_location("smartdrone_xfeat_module", xfeat_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load spec for {xfeat_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, "XFeat"):
        raise RuntimeError("XFeat class not found in xfeat.py")
    return module.XFeat


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True)
    parser.add_argument("--top-k", type=int, default=1024)
    parser.add_argument("--max-points", type=int, default=160)
    args = parser.parse_args()

    try:
        xfeat_cls = _load_xfeat_class(args.repo)
        with contextlib.redirect_stdout(io.StringIO()):
            model = xfeat_cls(top_k=max(1, args.top_k))
        sys.stdout.buffer.write(READY_MAGIC)
        sys.stdout.buffer.flush()
    except Exception as exc:
        _write_stderr(f"[xfeat_worker] startup failed: {exc}")
        return 2

    stdin = sys.stdin.buffer
    stdout = sys.stdout.buffer

    while True:
        header_bytes = _read_exact(stdin, REQUEST_HEADER.size)
        if header_bytes is None:
            return 0
        seq, rows, cols, payload_bytes = REQUEST_HEADER.unpack(header_bytes)
        if rows == 0 or cols == 0 or payload_bytes != rows * cols:
            _write_stderr(
                f"[xfeat_worker] invalid request seq={seq} rows={rows} cols={cols} payload={payload_bytes}"
            )
            return 3

        image_bytes = _read_exact(stdin, payload_bytes)
        if image_bytes is None:
            return 0

        try:
            gray = np.frombuffer(image_bytes, dtype=np.uint8).reshape((rows, cols))
            tensor = torch.from_numpy(gray.copy()).unsqueeze(0).unsqueeze(0).float() / 255.0
            result = model.detectAndCompute(tensor, top_k=max(1, args.top_k))[0]
            keypoints = result["keypoints"]
            descriptors = result["descriptors"]
            if keypoints.ndim != 2 or keypoints.shape[1] != 2:
                keypoints = torch.zeros((0, 2), dtype=torch.float32)
            if descriptors.ndim != 2:
                descriptors = torch.zeros((0, 64), dtype=torch.float32)
            limit = max(1, args.max_points)
            keypoints = keypoints[:limit].detach().to(torch.float32).cpu().contiguous()
            descriptors = descriptors[:limit].detach().to(torch.float32).cpu().contiguous()
            descriptor_dim = int(descriptors.shape[1]) if descriptors.ndim == 2 and descriptors.shape[0] > 0 else 64
            response = RESPONSE_HEADER.pack(seq, int(keypoints.shape[0]), descriptor_dim)
            stdout.write(response)
            if keypoints.numel() > 0:
                stdout.write(keypoints.numpy().tobytes(order="C"))
                stdout.write(descriptors.numpy().tobytes(order="C"))
            stdout.flush()
        except Exception as exc:
            _write_stderr(f"[xfeat_worker] inference failed seq={seq}: {exc}")
            try:
                stdout.write(RESPONSE_HEADER.pack(seq, 0, 64))
                stdout.flush()
            except Exception:
                return 4


if __name__ == "__main__":
    raise SystemExit(main())
