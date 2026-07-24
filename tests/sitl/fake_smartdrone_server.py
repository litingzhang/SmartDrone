#!/usr/bin/env python3
"""Deterministic TLV peer used by the hover runner integration test."""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from sitl_hover.protocol import (  # noqa: E402
    ACK_PAYLOAD,
    CMD_ACK,
    CMD_ARM,
    CMD_HEARTBEAT,
    CMD_LAND,
    CMD_MOVE,
    CMD_OFFBOARD,
    CMD_RUNTIME_MODE,
    CMD_STATE,
    STATE_FLAG_PX4_FLIGHT_STATE_VALID,
    STATE_PAYLOAD,
    TlvFrame,
    decode_datagram,
    encode_frame,
    monotonic_ms32,
)
from sitl_hover.provenance import write_launch_attestation  # noqa: E402


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--tracking-loss-after-move-s", type=float)
    parser.add_argument("--auto-land-after-move-s", type=float)
    parser.add_argument("--disarm-after-move-s", type=float)
    parser.add_argument("--flight-state-invalid-after-move-s", type=float)
    parser.add_argument("--stop-state-after-move-s", type=float)
    parser.add_argument("--arm-after-count", type=int, default=1)
    parser.add_argument("--reject-move-count", type=int, default=0)
    parser.add_argument("--reject-land", action="store_true")
    parser.add_argument("--environment-output", type=Path)
    return parser.parse_args()


def _ack(
    sock: socket.socket,
    peer: tuple[str, int],
    request: TlvFrame,
    status: int = 0,
) -> None:
    payload = ACK_PAYLOAD.pack(request.command, request.sequence, status, 0)
    sock.sendto(encode_frame(TlvFrame(CMD_ACK, 0, request.sequence, request.timestamp_ms, payload)), peer)


def _state_payload(
    armed: bool,
    main_mode: int,
    sub_mode: int,
    tracking_state: int,
    z: float,
) -> bytes:
    return STATE_PAYLOAD.pack(
        1, 3, tracking_state, int(armed), 0, 0,
        0.0, 0.0, z, 1.0, 0.0, 0.0, 0.0,
        main_mode, sub_mode,
    )


def _write_attestation_from_environment() -> None:
    run_id = os.environ.get("SMART_DRONE_SITL_RUN_ID")
    path = os.environ.get("SMART_DRONE_ATTESTATION_FILE")
    if run_id is None and path is None:
        return
    if run_id is None or path is None:
        raise RuntimeError("incomplete SmartDrone attestation environment")
    write_launch_attestation(
        Path(path), role="smart_drone", run_id=run_id,
        profile=os.environ["SMART_DRONE_SIM_PROFILE"],
        seed=int(os.environ["SMART_DRONE_SIM_SEED"]),
        details={"launcher": "fake_smartdrone_server.py"},
    )


def main() -> int:
    args = _arguments()
    _write_attestation_from_environment()
    if args.environment_output is not None:
        names = (
            "GZ_PARTITION", "GZ_IP", "SMART_DRONE_GZ_PARTITION",
            "SMART_DRONE_SIM_PROFILE",
            "SMART_DRONE_SIM_SEED", "SMART_DRONE_SIM_WORLD", "SMART_DRONE_SIM_MODEL",
            "SMART_DRONE_PX4_LOG_DIR",
            "SMART_DRONE_PX4_READY_FILE",
            "SMART_DRONE_AUTO_MODE", "SMART_DRONE_PX4_POSE_OUTPUT_MODE",
            "SMART_DRONE_OFFBOARD_REQUIRES_VISION", "SMART_DRONE_VISUAL_LOSS_LAND_MS",
            "SMART_DRONE_JSON_DIAGNOSTICS", "SMART_DRONE_SIM_CONFIG",
            "SMART_DRONE_SIM_FAULT_FILE",
            "SMART_DRONE_SITL_RUN_ID", "SMART_DRONE_ATTESTATION_FILE",
        )
        values = {name: os.environ.get(name) for name in names}
        args.environment_output.write_text(json.dumps(values, sort_keys=True), encoding="utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(0.02)
    peer: tuple[str, int] | None = None
    armed, main_mode, sub_mode, tracking_state = False, 1, 0, 5
    z, sequence = 0.0, 1
    move_time_s: float | None = None
    arm_requests, rejected_moves = 0, 0
    next_state_s = time.monotonic()
    while True:
        try:
            data, sender = sock.recvfrom(65535)
            request = decode_datagram(data)
            peer = sender
            if request.command != CMD_HEARTBEAT:
                status = 0
                if request.command == CMD_MOVE and rejected_moves < args.reject_move_count:
                    rejected_moves += 1
                    status = -4
                if request.command == CMD_LAND and args.reject_land:
                    status = -4
                _ack(sock, sender, request, status)
                if status != 0:
                    continue
            if request.command == CMD_OFFBOARD:
                main_mode = 6
            elif request.command == CMD_ARM:
                arm_requests += 1
                armed = arm_requests >= args.arm_after_count
            elif request.command == CMD_MOVE and len(request.payload) == 21:
                z = struct.unpack_from("<f", request.payload, 9)[0]
                move_time_s = time.monotonic()
            elif request.command == CMD_LAND:
                armed, main_mode, sub_mode, z = False, 4, 6, 0.0
            elif request.command == CMD_RUNTIME_MODE:
                pass
        except socket.timeout:
            pass
        if move_time_s is not None:
            move_elapsed_s = time.monotonic() - move_time_s
            if args.tracking_loss_after_move_s is not None and move_elapsed_s >= args.tracking_loss_after_move_s:
                tracking_state = 4
            if args.auto_land_after_move_s is not None and move_elapsed_s >= args.auto_land_after_move_s:
                main_mode, sub_mode = 4, 6
            if args.disarm_after_move_s is not None and move_elapsed_s >= args.disarm_after_move_s:
                armed = False
        state_enabled = (
            move_time_s is None or args.stop_state_after_move_s is None
            or move_elapsed_s < args.stop_state_after_move_s
        )
        if peer is not None and state_enabled and time.monotonic() >= next_state_s:
            payload = _state_payload(armed, main_mode, sub_mode, tracking_state, z)
            flight_state_valid = (
                move_time_s is None or args.flight_state_invalid_after_move_s is None
                or move_elapsed_s < args.flight_state_invalid_after_move_s
            )
            flags = STATE_FLAG_PX4_FLIGHT_STATE_VALID if flight_state_valid else 0
            frame = TlvFrame(CMD_STATE, flags, sequence, monotonic_ms32(), payload)
            sock.sendto(encode_frame(frame), peer)
            sequence += 1
            next_state_s = time.monotonic() + 0.02


if __name__ == "__main__":
    raise SystemExit(main())
