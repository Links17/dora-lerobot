"""Versioned robot data contracts.

All joint positions are radians and all timestamps are UTC epoch nanoseconds.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import StrEnum
from math import isfinite
from types import MappingProxyType

SCHEMA_VERSION = "v1"


class ControlMode(StrEnum):
    POSITION = "position"
    VELOCITY = "velocity"
    TORQUE = "torque"


class LifecycleState(StrEnum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    CALIBRATED = "calibrated"
    DISABLED = "disabled"
    ENABLED = "enabled"
    FAULTED = "faulted"


def _immutable_finite_mapping(values: Mapping[str, float], field_name: str) -> Mapping[str, float]:
    normalized = dict(values)
    if not normalized:
        raise ValueError(f"{field_name} must not be empty")
    if any(not name or not isinstance(name, str) for name in normalized):
        raise ValueError(f"{field_name} contains an invalid joint name")
    if any(not isfinite(value) for value in normalized.values()):
        raise ValueError(f"{field_name} contains a non-finite value")
    return MappingProxyType(normalized)


@dataclass(frozen=True, slots=True)
class RobotCapabilities:
    joint_names: tuple[str, ...]
    control_modes: frozenset[ControlMode] = field(
        default_factory=lambda: frozenset({ControlMode.POSITION})
    )
    has_gripper: bool = False
    camera_names: tuple[str, ...] = ()
    has_force_sensor: bool = False
    max_control_hz: float = 30.0

    def __post_init__(self) -> None:
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be a non-empty unique tuple")
        if any(not name for name in self.joint_names):
            raise ValueError("joint_names must not contain empty names")
        if not self.control_modes:
            raise ValueError("control_modes must not be empty")
        if not isfinite(self.max_control_hz) or self.max_control_hz <= 0:
            raise ValueError("max_control_hz must be positive")


@dataclass(frozen=True, slots=True)
class Observation:
    timestamp_ns: int
    joints_rad: Mapping[str, float]
    schema_version: str = SCHEMA_VERSION
    images: Mapping[str, object] = field(default_factory=dict)
    fault: str | None = None

    def __post_init__(self) -> None:
        if self.timestamp_ns <= 0:
            raise ValueError("timestamp_ns must be positive")
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError(f"unsupported schema_version: {self.schema_version}")
        object.__setattr__(
            self, "joints_rad", _immutable_finite_mapping(self.joints_rad, "joints_rad")
        )
        object.__setattr__(self, "images", MappingProxyType(dict(self.images)))


@dataclass(frozen=True, slots=True)
class Action:
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    timestamp_ns: int
    control_mode: ControlMode = ControlMode.POSITION
    schema_version: str = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if not self.joint_names or len(self.joint_names) != len(self.positions_rad):
            raise ValueError("joint_names and positions_rad must have the same non-zero length")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be unique")
        if any(not isfinite(value) for value in self.positions_rad):
            raise ValueError("positions_rad contains a non-finite value")
        if self.timestamp_ns <= 0:
            raise ValueError("timestamp_ns must be positive")
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError(f"unsupported schema_version: {self.schema_version}")

    def as_mapping(self) -> Mapping[str, float]:
        return MappingProxyType(dict(zip(self.joint_names, self.positions_rad, strict=True)))
