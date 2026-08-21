"""Minimal hardware-only driver abstractions."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from math import isfinite
from typing import Protocol, runtime_checkable


@dataclass(frozen=True, slots=True)
class JointLimit:
    minimum_rad: float
    maximum_rad: float
    max_velocity_rad_s: float = 3.14

    def __post_init__(self) -> None:
        if not all(
            isfinite(value)
            for value in (self.minimum_rad, self.maximum_rad, self.max_velocity_rad_s)
        ):
            raise ValueError("joint limits must be finite")
        if self.minimum_rad >= self.maximum_rad or self.max_velocity_rad_s <= 0:
            raise ValueError("invalid joint limit")

    def clamp(self, position_rad: float) -> float:
        return min(max(position_rad, self.minimum_rad), self.maximum_rad)


@runtime_checkable
class JointDriver(Protocol):
    """Protocol-only boundary for a set of named joints."""

    @property
    def joint_names(self) -> tuple[str, ...]: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def enable_torque(self, enabled: bool) -> None: ...

    def read_positions_rad(self) -> Mapping[str, float]: ...

    def write_positions_rad(self, positions_rad: Mapping[str, float]) -> None: ...


class MemoryJointDriver:
    """Deterministic in-memory driver used by tests and dry runs."""

    def __init__(self, joint_names: tuple[str, ...]) -> None:
        self._joint_names = joint_names
        self.positions = {name: 0.0 for name in joint_names}
        self.connected = False
        self.torque_enabled = False
        self.last_written_positions_rad: dict[str, float] | None = None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._joint_names

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def enable_torque(self, enabled: bool) -> None:
        if not self.connected:
            raise RuntimeError("driver is disconnected")
        self.torque_enabled = enabled

    def read_positions_rad(self) -> Mapping[str, float]:
        if not self.connected:
            raise RuntimeError("driver is disconnected")
        return dict(self.positions)

    def write_positions_rad(self, positions_rad: Mapping[str, float]) -> None:
        if not self.connected or not self.torque_enabled:
            raise RuntimeError("driver torque is disabled")
        if set(positions_rad) != set(self._joint_names):
            raise ValueError("driver action joint set mismatch")
        self.positions.update(positions_rad)
        self.last_written_positions_rad = dict(positions_rad)
