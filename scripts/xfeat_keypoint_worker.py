#!/usr/bin/env python3

import argparse
import contextlib
import importlib.util
import io
import os
import struct
import sys
from typing import Optional, Tuple

import numpy as np
import torch


REQUEST_HEADER = struct.Struct("<II")
IMAGE_HEADER = struct.Struct("<III")
RESPONSE_HEADER = struct.Struct("<II")
FEATURE_HEADER = struct.Struct("<II")
READY_MAGIC = b"XFWKRDY1"
_PERF_LOG_INTERVAL = 120


def _write_stderr(message: str) -> None:
    sys.stderr.write(message + "\n")
    sys.stderr.flush()


def _read_exact(stream, size: int) -> Optional[bytes]:
    chunks = bytearray()
    while len(chunks) < size:
        block = stream.read(size - len(chunks))
        if not block:
            return None
        chunks.extend(block)
    return bytes(chunks)


def _empty_result() -> Tuple[torch.Tensor, torch.Tensor]:
    return (
        torch.zeros((0, 2), dtype=torch.float32),
        torch.zeros((0, 64), dtype=torch.float32),
    )


def _normalize_result(result: dict, max_points: int) -> Tuple[torch.Tensor, torch.Tensor]:
    keypoints = result["keypoints"]
    descriptors = result["descriptors"]
    if keypoints.ndim != 2 or keypoints.shape[1] != 2:
        keypoints, descriptors = _empty_result()
    elif descriptors.ndim != 2:
        _, descriptors = _empty_result()
    limit = max(1, max_points)
    keypoints = keypoints[:limit].detach().to(torch.float32).cpu().contiguous()
    descriptors = descriptors[:limit].detach().to(torch.float32).cpu().contiguous()
    return keypoints, descriptors


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


def _resolve_device(device_arg: str) -> torch.device:
    normalized = (device_arg or "auto").strip().lower()
    if normalized == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if normalized == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError("cuda requested but torch.cuda.is_available() is false")
        return torch.device("cuda")
    if normalized == "cpu":
        return torch.device("cpu")
    raise RuntimeError(f"unsupported xfeat device: {device_arg}")


def _build_batch(images) -> torch.Tensor:
    image_count = len(images)
    rows, cols = images[0].shape
    if image_count == 1:
        return torch.from_numpy(images[0]).unsqueeze(0).unsqueeze(0)

    batch = np.empty((image_count, rows, cols), dtype=np.uint8)
    for index, gray in enumerate(images):
        batch[index, :, :] = gray
    return torch.from_numpy(batch).unsqueeze(1)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--top-k", type=int, default=1024)
    parser.add_argument("--max-points", type=int, default=160)
    args = parser.parse_args()

    try:
        xfeat_cls = _load_xfeat_class(args.repo)
        device = _resolve_device(args.device)
        use_cuda = device.type == "cuda"
        infer_dtype = torch.float16 if use_cuda else torch.float32
        autocast_enabled = use_cuda
        if use_cuda:
            torch.backends.cudnn.benchmark = True
        with contextlib.redirect_stdout(io.StringIO()):
            model = xfeat_cls(top_k=max(1, args.top_k))
            to_fn = getattr(model, "to", None)
            if callable(to_fn):
                to_fn(device)
            eval_fn = getattr(model, "eval", None)
            if callable(eval_fn):
                eval_fn()
        sys.stdout.buffer.write(READY_MAGIC)
        sys.stdout.buffer.flush()
        _write_stderr(
            f"[xfeat_worker] ready device={device.type} top_k={max(1, args.top_k)} max_points={max(1, args.max_points)} "
            f"dtype={infer_dtype}"
        )
    except Exception as exc:
        _write_stderr(f"[xfeat_worker] startup failed: {exc}")
        return 2

    stdin = sys.stdin.buffer
    stdout = sys.stdout.buffer
    request_count = 0

    while True:
        header_bytes = _read_exact(stdin, REQUEST_HEADER.size)
        if header_bytes is None:
            return 0
        seq, image_count = REQUEST_HEADER.unpack(header_bytes)
        if image_count == 0:
            _write_stderr(f"[xfeat_worker] invalid request seq={seq} image_count={image_count}")
            return 3

        images = []
        for _ in range(image_count):
            image_header_bytes = _read_exact(stdin, IMAGE_HEADER.size)
            if image_header_bytes is None:
                return 0
            rows, cols, payload_bytes = IMAGE_HEADER.unpack(image_header_bytes)
            if rows == 0 or cols == 0 or payload_bytes != rows * cols:
                _write_stderr(
                    f"[xfeat_worker] invalid image header seq={seq} rows={rows} cols={cols} payload={payload_bytes}"
                )
                return 3
            image_bytes = _read_exact(stdin, payload_bytes)
            if image_bytes is None:
                return 0
            gray = np.frombuffer(image_bytes, dtype=np.uint8).reshape((rows, cols))
            images.append(gray)

        try:
            batch_cpu = _build_batch(images)
            tensor = batch_cpu.to(device=device, dtype=infer_dtype, non_blocking=False)
            tensor = tensor.div_(255.0)
            with torch.inference_mode():
                with torch.autocast(device_type=device.type, dtype=infer_dtype, enabled=autocast_enabled):
                    results = model.detectAndCompute(tensor, top_k=max(1, args.top_k))
            normalized_results = [_normalize_result(result, args.max_points) for result in results]
            response = RESPONSE_HEADER.pack(seq, len(normalized_results))
            stdout.write(response)
            for keypoints, descriptors in normalized_results:
                descriptor_dim = (
                    int(descriptors.shape[1]) if descriptors.ndim == 2 and descriptors.shape[0] > 0 else 64
                )
                stdout.write(FEATURE_HEADER.pack(int(keypoints.shape[0]), descriptor_dim))
                if keypoints.numel() > 0:
                    stdout.write(keypoints.numpy().tobytes(order="C"))
                    stdout.write(descriptors.numpy().tobytes(order="C"))
            stdout.flush()
            request_count += 1
            if request_count % _PERF_LOG_INTERVAL == 0:
                total_points = sum(int(keypoints.shape[0]) for keypoints, _ in normalized_results)
                _write_stderr(
                    f"[xfeat_worker] perf req={request_count} batch={len(images)} shape={images[0].shape[1]}x{images[0].shape[0]} "
                    f"points={total_points}"
                )
        except Exception as exc:
            _write_stderr(f"[xfeat_worker] inference failed seq={seq}: {exc}")
            try:
                stdout.write(RESPONSE_HEADER.pack(seq, image_count))
                for _ in range(image_count):
                    stdout.write(FEATURE_HEADER.pack(0, 64))
                stdout.flush()
            except Exception:
                return 4


if __name__ == "__main__":
    raise SystemExit(main())
