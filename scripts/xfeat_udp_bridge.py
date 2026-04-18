#!/usr/bin/env python3
"""Send stereo previews + XFeat keypoints to the Android app over the existing UDP protocol."""

from __future__ import annotations

import argparse
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import torch

try:
    import cv2
except Exception as exc:  # pragma: no cover - import diagnostic path
    print(f"[xfeat_bridge] failed to import cv2: {exc}", file=sys.stderr)
    print("[xfeat_bridge] install OpenCV first, for example: python -m pip install opencv-contrib-python", file=sys.stderr)
    raise SystemExit(2)


VIDEO_MAGIC = 0x5643494D
VIDEO_VERSION = 1
VIDEO_FLAG_FEATURE_POINTS = 0x01
VIDEO_HEADER = struct.Struct("<I H B B I d I H H I I")
DEFAULT_PHONE_PORT = 5000
DEFAULT_MAX_PAYLOAD = 1200


class FrameSource:
    def read(self) -> tuple[bool, np.ndarray | None]:
        raise NotImplementedError

    def release(self) -> None:
        pass


class CvCaptureSource(FrameSource):
    def __init__(self, source: int | str, width: int, height: int) -> None:
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise RuntimeError(f"failed to open source: {source}")
        if width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        if height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        self._cap = cap

    def read(self) -> tuple[bool, np.ndarray | None]:
        return self._cap.read()

    def release(self) -> None:
        self._cap.release()


class RpicamRawSource(FrameSource):
    def __init__(self, camera_index: int, width: int, height: int, fps: float) -> None:
        if width <= 0 or height <= 0:
            raise RuntimeError("rpicam source requires positive width/height")
        frame_rate = fps if fps > 0 else 30.0
        self._width = width
        self._height = height
        self._y_plane_bytes = width * height
        self._frame_bytes = self._y_plane_bytes + (self._y_plane_bytes // 2)
        cmd = [
            "rpicam-vid",
            "--camera",
            str(camera_index),
            "--nopreview",
            "--timeout",
            "0",
            "--framerate",
            f"{frame_rate:.2f}",
            "--width",
            str(width),
            "--height",
            str(height),
            "--codec",
            "yuv420",
            "--flush",
            "1",
            "-o",
            "-",
        ]
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except FileNotFoundError as exc:
            raise RuntimeError("rpicam-vid not found in PATH") from exc
        if self._proc.stdout is None:
            raise RuntimeError("failed to open rpicam stdout")

    def _read_exact(self, size: int) -> bytes | None:
        chunks: list[bytes] = []
        remaining = size
        while remaining > 0:
            chunk = self._proc.stdout.read(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def read(self) -> tuple[bool, np.ndarray | None]:
        payload = self._read_exact(self._frame_bytes)
        if payload is None:
            return False, None
        y_plane = payload[: self._y_plane_bytes]
        frame = np.frombuffer(y_plane, dtype=np.uint8).reshape(self._height, self._width).copy()
        return True, frame

    def release(self) -> None:
        if hasattr(self, "_proc") and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait(timeout=2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read stereo video, run XFeat, and stream previews/features to app.")
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path.home() / "third_party" / "accelerated_features",
        help="Path to cloned verlab/accelerated_features repository.",
    )
    parser.add_argument("--phone-ip", required=True, help="Android phone IP address.")
    parser.add_argument("--phone-port", type=int, default=DEFAULT_PHONE_PORT, help="Android video UDP port.")
    parser.add_argument(
        "--left-source",
        default="0",
        help="Left camera source, e.g. 0, /dev/video0, file path, or rpicam:0.",
    )
    parser.add_argument(
        "--right-source",
        default="1",
        help="Right camera source, e.g. 1, /dev/video1, file path, or rpicam:1.",
    )
    parser.add_argument("--width", type=int, default=640, help="Requested capture width.")
    parser.add_argument("--height", type=int, default=480, help="Requested capture height.")
    parser.add_argument("--fps", type=float, default=6.0, help="Max processing FPS.")
    parser.add_argument("--top-k", type=int, default=1024, help="XFeat top_k.")
    parser.add_argument("--max-points", type=int, default=160, help="Max keypoints sent per camera.")
    parser.add_argument("--jpeg-quality", type=int, default=80, help="Preview JPEG quality.")
    parser.add_argument("--max-payload", type=int, default=DEFAULT_MAX_PAYLOAD, help="UDP payload bytes per image chunk.")
    parser.add_argument("--status-every", type=int, default=30, help="Print one status line every N stereo pairs.")
    parser.add_argument("--no-image", action="store_true", help="Only send feature packets, no JPEG previews.")
    return parser.parse_args()


def source_from_text(text: str) -> int | str:
    stripped = text.strip()
    if stripped.isdigit():
        return int(stripped)
    return stripped


def open_capture(source_text: str, width: int, height: int, fps: float) -> FrameSource:
    stripped = source_text.strip()
    if stripped.startswith("rpicam:"):
        camera_index = int(stripped.split(":", 1)[1])
        return RpicamRawSource(camera_index, width, height, fps)
    return CvCaptureSource(source_from_text(stripped), width, height)


def load_xfeat(repo_root: Path, top_k: int) -> tuple[object, torch.device]:
    repo_root = repo_root.resolve()
    if not repo_root.exists():
        raise FileNotFoundError(f"repo not found: {repo_root}")
    sys.path.insert(0, str(repo_root))
    from modules.xfeat import XFeat  # type: ignore

    torch.set_grad_enabled(False)
    device = torch.device("cpu")
    model = XFeat(top_k=top_k).to(device).eval()
    return model, device


def to_gray(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return frame
    if frame.ndim == 3 and frame.shape[2] == 1:
        return frame[:, :, 0]
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def to_xfeat_tensor(frame: np.ndarray, device: torch.device) -> torch.Tensor:
    if frame.ndim == 2:
        rgb = np.repeat(frame[:, :, None], 3, axis=2)
    elif frame.ndim == 3 and frame.shape[2] == 3:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    elif frame.ndim == 3 and frame.shape[2] == 1:
        rgb = np.repeat(frame, 3, axis=2)
    else:
        raise ValueError(f"unsupported frame shape: {frame.shape}")

    tensor = torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1).unsqueeze(0).to(device=device)
    return tensor.float() / 255.0


def sample_grid(points: np.ndarray, width: int, height: int, max_count: int) -> np.ndarray:
    if points.size == 0 or width <= 0 or height <= 0 or max_count <= 0:
        return np.empty((0, 2), dtype=np.int32)

    grid_cols = 8
    grid_rows = 6
    per_cell_cap = 4
    cells: list[list[int]] = [[] for _ in range(grid_cols * grid_rows)]

    for idx, pt in enumerate(points):
        xi = int(np.clip(np.rint(pt[0]), 0, width - 1))
        yi = int(np.clip(np.rint(pt[1]), 0, height - 1))
        gx = min(grid_cols - 1, (xi * grid_cols) // width)
        gy = min(grid_rows - 1, (yi * grid_rows) // height)
        cells[gy * grid_cols + gx].append(idx)

    selected: list[np.ndarray] = []
    cursor = [0] * len(cells)
    used = [0] * len(cells)
    made_progress = True
    while len(selected) < max_count and made_progress:
        made_progress = False
        for cell_idx, bucket in enumerate(cells):
            if len(selected) >= max_count:
                break
            if not bucket or used[cell_idx] >= per_cell_cap or cursor[cell_idx] >= len(bucket):
                continue
            selected.append(points[bucket[cursor[cell_idx]]])
            cursor[cell_idx] += 1
            used[cell_idx] += 1
            made_progress = True

    if not selected:
        return np.empty((0, 2), dtype=np.int32)
    out = np.rint(np.vstack(selected)).astype(np.int32, copy=False)
    out[:, 0] = np.clip(out[:, 0], 0, width - 1)
    out[:, 1] = np.clip(out[:, 1], 0, height - 1)
    return out


def pack_header(cam_index: int, flags: int, seq: int, frame_time_sec: float, frame_id: int,
                chunk_idx: int, chunk_cnt: int, total_size: int, chunk_size: int) -> bytes:
    return VIDEO_HEADER.pack(
        VIDEO_MAGIC,
        VIDEO_VERSION,
        cam_index,
        flags,
        seq,
        frame_time_sec,
        frame_id & 0xFFFFFFFF,
        chunk_idx,
        chunk_cnt,
        total_size,
        chunk_size,
    )


def send_feature_packet(sock: socket.socket, dst: tuple[str, int], cam_index: int, seq: int, frame_time_sec: float,
                        frame_id: int, width: int, height: int, points_xy: np.ndarray) -> None:
    count = int(points_xy.shape[0])
    payload = bytearray(6 + count * 4)
    struct.pack_into("<H", payload, 0, min(width, 0xFFFF))
    struct.pack_into("<H", payload, 2, min(height, 0xFFFF))
    struct.pack_into("<H", payload, 4, count)
    offset = 6
    for x, y in points_xy:
        struct.pack_into("<H", payload, offset, int(x))
        struct.pack_into("<H", payload, offset + 2, int(y))
        offset += 4
    header = pack_header(cam_index, VIDEO_FLAG_FEATURE_POINTS, seq, frame_time_sec, frame_id, 0, 1, len(payload), len(payload))
    sock.sendto(header + payload, dst)


def send_image_packets(sock: socket.socket, dst: tuple[str, int], cam_index: int, seq: int, frame_time_sec: float,
                       frame_id: int, gray: np.ndarray, jpeg_quality: int, max_payload: int) -> None:
    ok, encoded = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    if not ok or encoded is None or encoded.size == 0:
        raise RuntimeError("jpeg encode failed")
    data = encoded.tobytes()
    chunk_cnt = max(1, (len(data) + max_payload - 1) // max_payload)
    for chunk_idx in range(chunk_cnt):
        start = chunk_idx * max_payload
        end = min(len(data), start + max_payload)
        chunk = data[start:end]
        header = pack_header(cam_index, 0, seq, frame_time_sec, frame_id, chunk_idx, chunk_cnt, len(data), len(chunk))
        sock.sendto(header + chunk, dst)


def extract_points(model: object, device: torch.device, frame: np.ndarray, top_k: int, max_points: int) -> np.ndarray:
    tensor = to_xfeat_tensor(frame, device)
    with torch.inference_mode():
        output = model.detectAndCompute(tensor, top_k=top_k)[0]
    keypoints = output["keypoints"]
    if isinstance(keypoints, torch.Tensor):
        pts = keypoints.detach().cpu().numpy()
    else:
        pts = np.asarray(keypoints)
    if pts.ndim != 2 or pts.shape[1] < 2:
        return np.empty((0, 2), dtype=np.int32)
    return sample_grid(pts[:, :2], frame.shape[1], frame.shape[0], max_points)


def main() -> int:
    args = parse_args()

    try:
        model, device = load_xfeat(args.repo, args.top_k)
        left_cap = open_capture(args.left_source, args.width, args.height, args.fps)
        right_cap = open_capture(args.right_source, args.width, args.height, args.fps)
    except Exception as exc:
        print(f"[xfeat_bridge] setup failed: {exc}", file=sys.stderr)
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (args.phone_ip, args.phone_port)
    frame_interval = 1.0 / args.fps if args.fps > 0 else 0.0
    pair_index = 0
    last_loop_start = 0.0
    started_at = time.monotonic()

    print(f"[xfeat_bridge] streaming to udp://{args.phone_ip}:{args.phone_port}")
    print(f"[xfeat_bridge] left={args.left_source} right={args.right_source} size={args.width}x{args.height} top_k={args.top_k}")

    try:
        while True:
            now = time.monotonic()
            if frame_interval > 0.0 and last_loop_start > 0.0:
                remaining = frame_interval - (now - last_loop_start)
                if remaining > 0.0:
                    time.sleep(remaining)
            last_loop_start = time.monotonic()

            ok_l, frame_l = left_cap.read()
            ok_r, frame_r = right_cap.read()
            if not ok_l or frame_l is None:
                print("[xfeat_bridge] failed to read left frame", file=sys.stderr)
                return 3
            if not ok_r or frame_r is None:
                print("[xfeat_bridge] failed to read right frame", file=sys.stderr)
                return 4

            frame_time_sec = time.monotonic()
            gray_l = to_gray(frame_l)
            gray_r = to_gray(frame_r)
            pts_l = extract_points(model, device, frame_l, args.top_k, args.max_points)
            pts_r = extract_points(model, device, frame_r, args.top_k, args.max_points)

            if not args.no_image:
                send_image_packets(sock, dst, 0, pair_index, frame_time_sec, pair_index, gray_l, args.jpeg_quality, args.max_payload)
                send_image_packets(sock, dst, 1, pair_index, frame_time_sec, pair_index, gray_r, args.jpeg_quality, args.max_payload)

            send_feature_packet(sock, dst, 0, pair_index, frame_time_sec, pair_index, gray_l.shape[1], gray_l.shape[0], pts_l)
            send_feature_packet(sock, dst, 1, pair_index, frame_time_sec, pair_index, gray_r.shape[1], gray_r.shape[0], pts_r)

            pair_index += 1
            if args.status_every > 0 and (pair_index % args.status_every) == 0:
                elapsed = max(time.monotonic() - started_at, 1e-6)
                print(
                    f"[xfeat_bridge] pairs={pair_index} left_pts={pts_l.shape[0]} right_pts={pts_r.shape[0]} "
                    f"avg_fps={pair_index / elapsed:.2f}"
                )
    except KeyboardInterrupt:
        print("\n[xfeat_bridge] stopped by user")
        return 0
    finally:
        left_cap.release()
        right_cap.release()
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
