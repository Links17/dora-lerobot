"""Feetech serial driver kept below the robot adapter boundary."""

from __future__ import annotations

from collections.abc import Mapping
from math import degrees, pi, radians
from typing import Any, Protocol


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


class LeRobotSOFollowerDriver:
    """Joint driver backed by LeRobot's official SO follower runtime.

    LeRobot does the Feetech transport, motor calibration and position encoding.
    This wrapper only translates its named ``*.pos`` robot interface into this
    repository's explicit joint interface and keeps torque off until requested.
    """

    def __init__(
        self,
        runtime: Any,
        joint_names: tuple[str, ...],
        *,
        use_degrees: bool = False,
        normalized_joint_ranges: Mapping[str, tuple[float, float]] | None = None,
    ) -> None:
        self.runtime = runtime
        self._joint_names = joint_names
        self._use_degrees = use_degrees
        self._normalized_joint_ranges = dict(normalized_joint_ranges or {})
        if not set(self._normalized_joint_ranges).issubset(self._joint_names):
            raise ValueError("normalized joint range has an unknown joint")
        self._connected = False
        self._torque_enabled = False

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._joint_names

    def connect(self) -> None:
        if self._connected:
            return
        self.runtime.connect(calibrate=False)
        self._connected = True
        self.enable_torque(False)

    def disconnect(self) -> None:
        if not self._connected:
            return
        try:
            self.enable_torque(False)
        finally:
            self.runtime.disconnect()
            self._connected = False

    def enable_torque(self, enabled: bool) -> None:
        if not self._connected:
            raise RuntimeError("driver is disconnected")
        bus = getattr(self.runtime, "bus", None)
        if bus is None or not hasattr(bus, "sync_write"):
            raise RuntimeError("LeRobot SO runtime does not expose a Feetech motor bus")
        bus.sync_write("Torque_Enable", {name: int(enabled) for name in self._joint_names})
        self._torque_enabled = enabled

    def read_positions_rad(self) -> Mapping[str, float]:
        if not self._connected:
            raise RuntimeError("driver is disconnected")
        observation = self.runtime.get_observation()
        positions = {
            name: self._from_lerobot(name, float(observation[f"{name}.pos"]))
            for name in self._joint_names
        }
        return positions

    def write_positions_rad(self, positions_rad: Mapping[str, float]) -> None:
        if not self._connected:
            raise RuntimeError("driver is disconnected")
        if not self._torque_enabled:
            raise RuntimeError("driver torque is disabled")
        if set(positions_rad) != set(self._joint_names):
            raise ValueError("driver action joint set mismatch")
        self.runtime.send_action(
            {
                f"{name}.pos": self._to_lerobot(name, positions_rad[name])
                for name in self._joint_names
            }
        )

    def _from_lerobot(self, name: str, value: float) -> float:
        if name in self._normalized_joint_ranges:
            minimum, maximum = self._normalized_joint_ranges[name]
            return minimum + (value / 100.0) * (maximum - minimum)
        return radians(value) if self._use_degrees else value

    def _to_lerobot(self, name: str, value: float) -> float:
        if name in self._normalized_joint_ranges:
            minimum, maximum = self._normalized_joint_ranges[name]
            if maximum <= minimum:
                raise ValueError(f"invalid normalized range for {name}")
            return 100.0 * (value - minimum) / (maximum - minimum)
        return degrees(value) if self._use_degrees else value


class LeRobotSOLeader:
    """Read-only official LeRobot SO leader wrapper for teleoperation."""

    def __init__(
        self,
        runtime: Any,
        joint_names: tuple[str, ...],
        *,
        use_degrees: bool = False,
        normalized_joint_ranges: Mapping[str, tuple[float, float]] | None = None,
    ) -> None:
        self.runtime = runtime
        self.joint_names = joint_names
        self._use_degrees = use_degrees
        self._normalized_joint_ranges = dict(normalized_joint_ranges or {})
        self._connected = False

    def connect(self) -> None:
        if not self._connected:
            self.runtime.connect(calibrate=False)
            self._connected = True

    def disconnect(self) -> None:
        if self._connected:
            self.runtime.disconnect()
            self._connected = False

    def read_positions_rad(self) -> Mapping[str, float]:
        if not self._connected:
            raise RuntimeError("leader is disconnected")
        action = self.runtime.get_action()
        return {
            name: self._from_lerobot(name, float(action[f"{name}.pos"]))
            for name in self.joint_names
        }

    def _from_lerobot(self, name: str, value: float) -> float:
        if name in self._normalized_joint_ranges:
            minimum, maximum = self._normalized_joint_ranges[name]
            return minimum + (value / 100.0) * (maximum - minimum)
        return radians(value) if self._use_degrees else value
