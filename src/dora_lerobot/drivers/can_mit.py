"""CAN/MIT packet primitives shared by RS and DM drivers."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Protocol


@dataclass(frozen=True, slots=True)
class CanFrame:
    arbitration_id: int
    data: bytes

    def __post_init__(self) -> None:
        if not 0 <= self.arbitration_id <= 0x1FFFFFFF:
            raise ValueError("invalid CAN arbitration id")
        if len(self.data) > 8:
            raise ValueError("classic CAN data must not exceed 8 bytes")


class CanTransport(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def send(self, frame: CanFrame) -> None: ...
    def receive(self, timeout_s: float) -> CanFrame | None: ...


def float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
    if not isfinite(value) or minimum >= maximum or bits <= 0:
        raise ValueError("invalid MIT conversion parameters")
    value = min(max(value, minimum), maximum)
    return round((value - minimum) * ((1 << bits) - 1) / (maximum - minimum))


def uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
    if not 0 <= value <= (1 << bits) - 1:
        raise ValueError("MIT encoded value is out of range")
    return value * (maximum - minimum) / ((1 << bits) - 1) + minimum


def encode_mit_position(position_rad: float, velocity_rad_s: float = 0.0) -> bytes:
    """Encode position/velocity into the standard 8-byte MIT control layout."""
    position = float_to_uint(position_rad, -12.5, 12.5, 16)
    velocity = float_to_uint(velocity_rad_s, -45.0, 45.0, 12)
    return bytes((position >> 8, position & 0xFF, velocity >> 4, (velocity & 0xF) << 4, 0, 0, 0, 0))


def decode_mit_position(data: bytes) -> float:
    if len(data) < 3:
        raise ValueError("MIT feedback frame is too short")
    return uint_to_float((data[1] << 8) | data[2], -12.5, 12.5, 16)
