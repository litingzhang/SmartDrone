#!/usr/bin/env python3
"""Generate deterministic high-contrast textures for Gazebo visual odometry."""

from __future__ import annotations

import argparse
import binascii
import struct
import zlib
from pathlib import Path


WIDTH = 1024
HEIGHT = 1024


class XorShift32:
    def __init__(self, seed: int) -> None:
        self._state = seed & 0xFFFFFFFF or 1

    def next(self) -> int:
        value = self._state
        value ^= (value << 13) & 0xFFFFFFFF
        value ^= value >> 17
        value ^= (value << 5) & 0xFFFFFFFF
        self._state = value & 0xFFFFFFFF
        return self._state

    def bounded(self, limit: int) -> int:
        return self.next() % limit


def _set_pixel(image: bytearray, x: int, y: int, value: tuple[int, int, int]) -> None:
    if not (0 <= x < WIDTH and 0 <= y < HEIGHT):
        return
    offset = (y * WIDTH + x) * 3
    image[offset : offset + 3] = bytes(value)


def _rectangle(
    image: bytearray,
    x0: int,
    y0: int,
    width: int,
    height: int,
    value: tuple[int, int, int],
) -> None:
    x1 = min(WIDTH, x0 + width)
    y1 = min(HEIGHT, y0 + height)
    row = bytes(value) * max(0, x1 - x0)
    for y in range(max(0, y0), y1):
        offset = (y * WIDTH + max(0, x0)) * 3
        image[offset : offset + len(row)] = row


def _line(
    image: bytearray,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    value: tuple[int, int, int],
    thickness: int,
) -> None:
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        _rectangle(image, x0 - thickness // 2, y0 - thickness // 2, thickness, thickness, value)
        if x0 == x1 and y0 == y1:
            return
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x0 += sx
        if twice_error <= dx:
            error += dx
            y0 += sy


def _marker(image: bytearray, rng: XorShift32, x0: int, y0: int, cell: int) -> None:
    dark = (18, 20, 22)
    light = (236, 239, 232)
    _rectangle(image, x0, y0, 7 * cell, 7 * cell, light)
    for row in range(7):
        for column in range(7):
            border = row in (0, 6) or column in (0, 6)
            bit = border or bool(rng.next() & 1)
            if bit:
                _rectangle(image, x0 + column * cell, y0 + row * cell, cell, cell, dark)


def _texture(seed: int, background: tuple[int, int, int]) -> bytearray:
    rng = XorShift32(seed)
    image = bytearray(WIDTH * HEIGHT * 3)
    for y in range(HEIGHT):
        for x in range(WIDTH):
            noise = rng.bounded(17) - 8
            value = tuple(max(0, min(255, channel + noise)) for channel in background)
            _set_pixel(image, x, y, value)

    palette = ((20, 24, 28), (232, 235, 226), (67, 91, 105), (178, 154, 92))
    for _ in range(260):
        width = 5 + rng.bounded(70)
        height = 5 + rng.bounded(70)
        _rectangle(
            image,
            rng.bounded(WIDTH - width),
            rng.bounded(HEIGHT - height),
            width,
            height,
            palette[rng.bounded(len(palette))],
        )
    for _ in range(90):
        _line(
            image,
            rng.bounded(WIDTH),
            rng.bounded(HEIGHT),
            rng.bounded(WIDTH),
            rng.bounded(HEIGHT),
            palette[rng.bounded(len(palette))],
            2 + rng.bounded(5),
        )
    for row in range(3):
        for column in range(4):
            cell = 7 + rng.bounded(5)
            x0 = 35 + column * 245 + rng.bounded(70)
            y0 = 45 + row * 315 + rng.bounded(90)
            _marker(image, rng, x0, y0, cell)
    return image


def _chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = binascii.crc32(kind + payload) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)


def _write_png(path: Path, image: bytearray) -> None:
    scanlines = bytearray()
    stride = WIDTH * 3
    for y in range(HEIGHT):
        scanlines.append(0)
        scanlines.extend(image[y * stride : (y + 1) * stride])
    header = struct.pack(">IIBBBBB", WIDTH, HEIGHT, 8, 2, 0, 0, 0)
    payload = b"\x89PNG\r\n\x1a\n"
    payload += _chunk(b"IHDR", header)
    payload += _chunk(b"IDAT", zlib.compress(bytes(scanlines), level=9))
    payload += _chunk(b"IEND", b"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    _write_png(args.output_dir / "wall_features.png", _texture(0x51A7D00D, (145, 151, 153)))
    _write_png(args.output_dir / "floor_features.png", _texture(0xF100A55E, (118, 120, 113)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
