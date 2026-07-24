"""SmartDrone UDP TLV codec and non-blocking command client."""

from __future__ import annotations

import select
import socket
import struct
import time
from dataclasses import asdict, dataclass
from typing import Callable, Iterable


SYNC = b"\xAA\x55"
VERSION = 1
HEADER = struct.Struct("<2sBBB H I I")
CRC = struct.Struct("<H")
ACK_PAYLOAD = struct.Struct("<BIhH")
STATE_PAYLOAD = struct.Struct("<BBBBHH7fBB")
MOVE_PAYLOAD = struct.Struct("<Bfffff")

CMD_ARM = 0x10
CMD_OFFBOARD = 0x12
CMD_LAND = 0x14
CMD_MOVE = 0x20
CMD_RUNTIME_MODE = 0x30
CMD_ACK = 0xF0
CMD_STATE = 0xF1
CMD_HEARTBEAT = 0xF5
ACK_E_BAD_STATE = -4

FRAME_NED = 2
RUNTIME_MODE_SLAM = 1
STATE_FLAG_PX4_FLIGHT_STATE_VALID = 0x01
PX4_MAIN_MODE_OFFBOARD = 6
PX4_MAIN_MODE_AUTO = 4
PX4_SUB_MODE_AUTO_LAND = 6
TRACKING_STATES_USABLE = frozenset((2, 5))


class ProtocolError(ValueError):
    """Raised when a TLV datagram is malformed."""


class CommandError(RuntimeError):
    """Raised when SmartDrone rejects or does not acknowledge a command."""

    def __init__(
        self,
        message: str,
        *,
        command: int | None = None,
        status: int | None = None,
    ) -> None:
        super().__init__(message)
        self.command = command
        self.status = status


@dataclass(frozen=True)
class TlvFrame:
    command: int
    flags: int
    sequence: int
    timestamp_ms: int
    payload: bytes
    version: int = VERSION


@dataclass(frozen=True)
class CommandAck:
    command: int
    sequence: int
    status: int


@dataclass(frozen=True)
class VehicleState:
    runtime_mode: int
    slam_mode: int
    tracking_state: int
    armed: bool
    reset_counter: int
    reset_map_count: int
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float
    px4_flight_state_valid: bool
    px4_main_mode: int
    px4_sub_mode: int
    sequence: int
    received_monotonic_s: float

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def monotonic_ms32() -> int:
    return int(time.monotonic() * 1000.0) & 0xFFFFFFFF


def encode_frame(frame: TlvFrame) -> bytes:
    header = HEADER.pack(
        SYNC,
        frame.version,
        frame.command,
        frame.flags,
        len(frame.payload),
        frame.sequence & 0xFFFFFFFF,
        frame.timestamp_ms & 0xFFFFFFFF,
    )
    content = header + frame.payload
    return content + CRC.pack(crc16_ccitt_false(content[2:]))


def decode_datagram(data: bytes) -> TlvFrame:
    if len(data) < HEADER.size + CRC.size:
        raise ProtocolError("TLV datagram is shorter than the frame overhead")
    sync, version, command, flags, length, sequence, timestamp_ms = HEADER.unpack_from(data)
    expected_size = HEADER.size + length + CRC.size
    if sync != SYNC or version != VERSION or len(data) != expected_size:
        raise ProtocolError("TLV sync or payload length is invalid")
    expected_crc = CRC.unpack_from(data, expected_size - CRC.size)[0]
    actual_crc = crc16_ccitt_false(data[2:-CRC.size])
    if expected_crc != actual_crc:
        raise ProtocolError("TLV CRC mismatch")
    payload = data[HEADER.size : HEADER.size + length]
    return TlvFrame(command, flags, sequence, timestamp_ms, payload, version)


def parse_ack(frame: TlvFrame) -> CommandAck:
    if frame.command != CMD_ACK or len(frame.payload) != ACK_PAYLOAD.size:
        raise ProtocolError("not a current 9-byte CMD_ACK frame")
    command, sequence, status, _ = ACK_PAYLOAD.unpack(frame.payload)
    return CommandAck(command, sequence, status)


def parse_state(frame: TlvFrame, received_s: float | None = None) -> VehicleState:
    if frame.command != CMD_STATE or len(frame.payload) != STATE_PAYLOAD.size:
        raise ProtocolError("not a current 38-byte CMD_STATE frame")
    values = STATE_PAYLOAD.unpack(frame.payload)
    return VehicleState(
        runtime_mode=values[0], slam_mode=values[1], tracking_state=values[2],
        armed=bool(values[3]), reset_counter=values[4], reset_map_count=values[5],
        x=values[6], y=values[7], z=values[8], qw=values[9], qx=values[10],
        qy=values[11], qz=values[12],
        px4_flight_state_valid=bool(frame.flags & STATE_FLAG_PX4_FLIGHT_STATE_VALID),
        px4_main_mode=values[13],
        px4_sub_mode=values[14], sequence=frame.sequence,
        received_monotonic_s=time.monotonic() if received_s is None else received_s,
    )


def build_move_payload(x: float, y: float, z: float, yaw: float, max_velocity: float) -> bytes:
    return MOVE_PAYLOAD.pack(FRAME_NED, x, y, z, yaw, max_velocity)


StateHandler = Callable[[VehicleState], None]


class TlvClient:
    """Maintains the control heartbeat while waiting for state and ACK frames."""

    def __init__(
        self,
        host: str,
        port: int,
        bind_host: str = "0.0.0.0",
        bind_port: int = 0,
        heartbeat_period_s: float = 0.25,
        sequence_seed: int = 1,
        state_handler: StateHandler | None = None,
    ) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((bind_host, bind_port))
        self._socket.connect((host, port))
        self._socket.setblocking(False)
        self._heartbeat_period_s = heartbeat_period_s
        self._next_heartbeat_s = 0.0
        self._next_sequence = max(1, sequence_seed & 0xFFFFFFFF)
        self._acks: dict[int, CommandAck] = {}
        self._latest_state: VehicleState | None = None
        self._state_handler = state_handler
        self.protocol_errors = 0

    @property
    def latest_state(self) -> VehicleState | None:
        return self._latest_state

    @property
    def local_address(self) -> tuple[str, int]:
        host, port = self._socket.getsockname()
        return str(host), int(port)

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "TlvClient":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _allocate_sequence(self) -> int:
        sequence = self._next_sequence
        self._next_sequence = (self._next_sequence + 1) & 0xFFFFFFFF
        if self._next_sequence == 0:
            self._next_sequence = 1
        return sequence

    def _send(self, command: int, payload: bytes = b"", flags: int = 0, sequence: int = 0) -> None:
        frame = TlvFrame(command, flags, sequence, monotonic_ms32(), payload)
        self._socket.send(encode_frame(frame))

    def send_heartbeat_if_due(self, now_s: float | None = None) -> None:
        now_s = time.monotonic() if now_s is None else now_s
        if now_s < self._next_heartbeat_s:
            return
        self._send(CMD_HEARTBEAT)
        self._next_heartbeat_s = now_s + self._heartbeat_period_s

    def _receive_available(self) -> None:
        while True:
            try:
                data = self._socket.recv(65535)
            except BlockingIOError:
                return
            try:
                frame = decode_datagram(data)
                self._handle_frame(frame)
            except ProtocolError:
                self.protocol_errors += 1

    def _handle_frame(self, frame: TlvFrame) -> None:
        if frame.command == CMD_ACK:
            ack = parse_ack(frame)
            self._acks[ack.sequence] = ack
        elif frame.command == CMD_STATE:
            state = parse_state(frame)
            self._latest_state = state
            if self._state_handler is not None:
                self._state_handler(state)

    def poll(self, duration_s: float = 0.05) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while True:
            now_s = time.monotonic()
            self.send_heartbeat_if_due(now_s)
            wait_s = min(0.05, max(0.0, deadline - now_s))
            readable, _, _ = select.select((self._socket,), (), (), wait_s)
            if readable:
                self._receive_available()
            if time.monotonic() >= deadline:
                return

    def command(self, command: int, payload: bytes = b"", flags: int = 0, timeout_s: float = 3.0) -> CommandAck:
        sequence = self._allocate_sequence()
        self._send(command, payload, flags, sequence)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll(min(0.05, deadline - time.monotonic()))
            ack = self._acks.pop(sequence, None)
            if ack is None:
                continue
            if ack.command != command or ack.status != 0:
                raise CommandError(
                    f"command 0x{command:02X} rejected: ack_cmd=0x{ack.command:02X} status={ack.status}",
                    command=command,
                    status=ack.status,
                )
            return ack
        raise CommandError(
            f"command 0x{command:02X} ACK timeout after {timeout_s:.1f}s",
            command=command,
        )

    def wait_for_state(
        self,
        predicate: Callable[[VehicleState], bool],
        timeout_s: float,
        stable_s: float = 0.0,
        max_state_gap_s: float = 1.5,
    ) -> VehicleState:
        deadline = time.monotonic() + timeout_s
        condition_since: float | None = None
        last_valid_s: float | None = None
        last_seen_s: float | None = None
        while time.monotonic() < deadline:
            self.poll(min(0.05, deadline - time.monotonic()))
            state = self._latest_state
            if state is None or state.received_monotonic_s == last_seen_s:
                continue
            last_seen_s = state.received_monotonic_s
            if predicate(state):
                if last_valid_s is None or state.received_monotonic_s - last_valid_s > max_state_gap_s:
                    condition_since = state.received_monotonic_s
                last_valid_s = state.received_monotonic_s
                if condition_since is not None and last_valid_s - condition_since >= stable_s:
                    return state
            else:
                condition_since = None
                last_valid_s = None
        raise TimeoutError(f"state condition did not hold for {stable_s:.1f}s within {timeout_s:.1f}s")


def usable_tracking(state: VehicleState) -> bool:
    return state.runtime_mode == RUNTIME_MODE_SLAM and state.tracking_state in TRACKING_STATES_USABLE


def states_to_rows(states: Iterable[VehicleState], origin_s: float) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for state in states:
        row = state.to_dict()
        row["elapsed_s"] = state.received_monotonic_s - origin_s
        rows.append(row)
    return rows
