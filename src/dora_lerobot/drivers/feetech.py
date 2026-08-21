"""Feetech serial driver kept below the robot adapter boundary."""

from __future__ import annotations

from collections.abc import Mapping
from math import pi
from typing import Protocol


class FeetechBus(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def torque(self, enabled: bool) -> None: ...
    def read_positions(self) -> Mapping[str, int]: ...
    def write_positions(self, positions: Mapping[str, int]) -> None: ...


class FeetechDriver:
    """Converts Feetech ticks to radians; SDK construction stays outside this class."""

    def __init__(
        self, bus: FeetechBus, joint_names: tuple[str, ...], ticks_per_turn: int = 4096
    ) -> None:
        if ticks_per_turn <= 0:
            raise ValueError("ticks_per_turn must be positive")
        self.bus = bus
        self._joint_names = joint_names
        self.ticks_per_turn = ticks_per_turn

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._joint_names

    def connect(self) -> None:
        self.bus.connect()

    def disconnect(self) -> None:
        self.bus.disconnect()

    def enable_torque(self, enabled: bool) -> None:
        self.bus.torque(enabled)

    def read_positions_rad(self) -> Mapping[str, float]:
        positions = self.bus.read_positions()
        return {name: ticks * (2 * pi / self.ticks_per_turn) for name, ticks in positions.items()}

    def write_positions_rad(self, positions_rad: Mapping[str, float]) -> None:
        if set(positions_rad) != set(self._joint_names):
            raise ValueError("driver action joint set mismatch")
        positions = {
            name: round(value * self.ticks_per_turn / (2 * pi))
            for name, value in positions_rad.items()
        }
        self.bus.write_positions(positions)
