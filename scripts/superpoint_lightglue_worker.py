#!/usr/bin/env python3

import argparse
import contextlib
import importlib
import io
import os
import struct
import sys
import types
from typing import List, Optional, Tuple

import numpy as np
import torch
import torch.nn.functional as F


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
    raise RuntimeError(f"unsupported device: {device_arg}")


def _image_to_tensor(gray: np.ndarray, device: torch.device) -> torch.Tensor:
    tensor = torch.from_numpy(gray).to(device=device, dtype=torch.float32)
    return tensor.unsqueeze(0).unsqueeze(0).div_(255.0)


def _load_lightglue(repo_path: str):
    if repo_path:
        abs_repo = os.path.abspath(repo_path)
        if os.path.isdir(abs_repo) and abs_repo not in sys.path:
            sys.path.insert(0, abs_repo)
        package_dir = os.path.join(abs_repo, "lightglue")
        if os.path.isdir(package_dir) and "lightglue" not in sys.modules:
            package = types.ModuleType("lightglue")
            package.__path__ = [package_dir]
            sys.modules["lightglue"] = package
    from lightglue.lightglue import LightGlue
    from lightglue.superpoint import SuperPoint
    from lightglue.utils import rbd

    return SuperPoint, LightGlue, rbd


def _normalize_feature_dict(features: dict, rbd_fn, max_points: int,
                            sort_by_score: bool = True) -> Tuple[torch.Tensor, torch.Tensor]:
    try:
        features = rbd_fn(features)
    except Exception:
        pass
    keypoints = features.get("keypoints")
    descriptors = features.get("descriptors")
    scores = features.get("keypoint_scores", features.get("scores"))
    if keypoints is None or descriptors is None:
        return torch.zeros((0, 2), dtype=torch.float32), torch.zeros((0, 256), dtype=torch.float32)
    if keypoints.ndim == 3:
        keypoints = keypoints[0]
    if descriptors.ndim == 3:
        descriptors = descriptors[0]
    if descriptors.ndim == 2 and keypoints.ndim == 2 and descriptors.shape[0] != keypoints.shape[0] and descriptors.shape[1] == keypoints.shape[0]:
        descriptors = descriptors.t()
    if keypoints.ndim != 2 or keypoints.shape[-1] != 2 or descriptors.ndim != 2:
        return torch.zeros((0, 2), dtype=torch.float32), torch.zeros((0, 256), dtype=torch.float32)
    count = min(int(keypoints.shape[0]), int(descriptors.shape[0]), max(1, max_points))
    if scores is not None and sort_by_score:
        if scores.ndim == 2:
            scores = scores[0]
        order = torch.argsort(scores[: keypoints.shape[0]], descending=True)
        order = order[:count]
        keypoints = keypoints.index_select(0, order)
        descriptors = descriptors.index_select(0, order)
    else:
        keypoints = keypoints[:count]
        descriptors = descriptors[:count]
    descriptors = F.normalize(descriptors.to(torch.float32), p=2, dim=1)
    return keypoints.detach().to(torch.float32).cpu().contiguous(), descriptors.detach().cpu().contiguous()


def _extract_raw(extractor, image: np.ndarray, device: torch.device, rbd_fn, max_points: int) -> Tuple[torch.Tensor, torch.Tensor]:
    with torch.inference_mode():
        features = extractor.extract(_image_to_tensor(image, device))
    return _normalize_feature_dict(features, rbd_fn, max_points)


def _extract_pair(extractor, matcher, images: List[np.ndarray], device: torch.device, rbd_fn,
                  max_points: int) -> List[Tuple[torch.Tensor, torch.Tensor]]:
    image0 = _image_to_tensor(images[0], device)
    image1 = _image_to_tensor(images[1], device)
    with torch.inference_mode():
        feats0 = extractor.extract(image0)
        feats1 = extractor.extract(image1)
        match_result = matcher({"image0": feats0, "image1": feats1})
    # Keep SuperPoint's original feature order here: LightGlue match indices refer to that order.
    kpts0, desc0 = _normalize_feature_dict(feats0, rbd_fn, max_points * 2, sort_by_score=False)
    kpts1, desc1 = _normalize_feature_dict(feats1, rbd_fn, max_points * 2, sort_by_score=False)
    try:
        match_result = rbd_fn(match_result)
    except Exception:
        pass
    matches = match_result.get("matches")
    if matches is None:
        matches0 = match_result.get("matches0")
        if matches0 is not None:
            if matches0.ndim == 2:
                matches0 = matches0[0]
            idx0 = torch.nonzero(matches0 >= 0, as_tuple=False).flatten()
            matches = torch.stack([idx0, matches0[idx0].to(idx0.device)], dim=1)
    if matches is None:
        return [(kpts0[:max_points], desc0[:max_points]), (kpts1[:max_points], desc1[:max_points])]
    if matches.ndim == 3:
        matches = matches[0]
    matches = matches.detach().cpu().to(torch.long)
    valid = []
    for i0, i1 in matches.tolist():
        if 0 <= i0 < kpts0.shape[0] and 0 <= i1 < kpts1.shape[0]:
            valid.append((i0, i1))
        if len(valid) >= max(1, max_points):
            break
    if not valid:
        return [(kpts0[:max_points], desc0[:max_points]), (kpts1[:max_points], desc1[:max_points])]
    idx0 = torch.tensor([pair[0] for pair in valid], dtype=torch.long)
    idx1 = torch.tensor([pair[1] for pair in valid], dtype=torch.long)
    return [(kpts0.index_select(0, idx0), desc0.index_select(0, idx0)),
            (kpts1.index_select(0, idx1), desc1.index_select(0, idx1))]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default="")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--top-k", type=int, default=1024)
    parser.add_argument("--max-points", type=int, default=320)
    args = parser.parse_args()

    try:
        SuperPoint, LightGlue, rbd_fn = _load_lightglue(args.repo)
        device = _resolve_device(args.device)
        if device.type == "cuda":
            torch.backends.cudnn.benchmark = True
        with contextlib.redirect_stdout(io.StringIO()):
            extractor = SuperPoint(max_num_keypoints=max(1, args.top_k)).eval().to(device)
            matcher = LightGlue(features="superpoint").eval().to(device)
        sys.stdout.buffer.write(READY_MAGIC)
        sys.stdout.buffer.flush()
        _write_stderr(
            f"[sp_lightglue_worker] ready device={device.type} top_k={max(1, args.top_k)} "
            f"max_points={max(1, args.max_points)}"
        )
    except Exception as exc:
        _write_stderr(f"[sp_lightglue_worker] startup failed: {exc}")
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
            _write_stderr(f"[sp_lightglue_worker] invalid request seq={seq} image_count={image_count}")
            return 3
        images = []
        for _ in range(image_count):
            image_header_bytes = _read_exact(stdin, IMAGE_HEADER.size)
            if image_header_bytes is None:
                return 0
            rows, cols, payload_bytes = IMAGE_HEADER.unpack(image_header_bytes)
            if rows == 0 or cols == 0 or payload_bytes != rows * cols:
                _write_stderr(f"[sp_lightglue_worker] invalid image header seq={seq}")
                return 3
            image_bytes = _read_exact(stdin, payload_bytes)
            if image_bytes is None:
                return 0
            images.append(np.frombuffer(image_bytes, dtype=np.uint8).reshape((rows, cols)))
        try:
            if len(images) == 2:
                normalized_results = _extract_pair(extractor, matcher, images, device, rbd_fn, max(1, args.max_points))
            else:
                normalized_results = [_extract_raw(extractor, image, device, rbd_fn, max(1, args.max_points))
                                      for image in images]
            stdout.write(RESPONSE_HEADER.pack(seq, len(normalized_results)))
            for keypoints, descriptors in normalized_results:
                descriptor_dim = int(descriptors.shape[1]) if descriptors.ndim == 2 and descriptors.shape[0] > 0 else 256
                stdout.write(FEATURE_HEADER.pack(int(keypoints.shape[0]), descriptor_dim))
                if keypoints.numel() > 0:
                    stdout.write(keypoints.numpy().tobytes(order="C"))
                    stdout.write(descriptors.numpy().tobytes(order="C"))
            stdout.flush()
            request_count += 1
            if request_count % _PERF_LOG_INTERVAL == 0:
                total_points = sum(int(keypoints.shape[0]) for keypoints, _ in normalized_results)
                _write_stderr(f"[sp_lightglue_worker] perf req={request_count} batch={len(images)} points={total_points}")
        except Exception as exc:
            _write_stderr(f"[sp_lightglue_worker] inference failed seq={seq}: {exc}")
            stdout.write(RESPONSE_HEADER.pack(seq, image_count))
            for _ in range(image_count):
                stdout.write(FEATURE_HEADER.pack(0, 256))
            stdout.flush()


if __name__ == "__main__":
    raise SystemExit(main())
