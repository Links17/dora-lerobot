"""Persistent, versioned references to device-owned calibration data."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from time import time_ns
from typing import Literal

CALIBRATION_PROFILE_VERSION = "v1"
CalibrationRole = Literal["leader", "follower"]


@dataclass(frozen=True, slots=True)
class CalibrationProfile:
    """Identity and location of an authoritative LeRobot calibration bundle.

    The profile intentionally does not duplicate servo offsets. LeRobot and its
    device runtime remain the owners of those calibration files.
    """

    robot_id: str
    role: CalibrationRole
    device_id: str
    calibration_dir: str
    created_at_ns: int
    schema_version: str = CALIBRATION_PROFILE_VERSION

    @classmethod
    def create(
        cls, *, robot_id: str, role: CalibrationRole, calibration_dir: Path, device_id: str
    ) -> CalibrationProfile:
        return cls(
            robot_id=robot_id,
            role=role,
            device_id=device_id,
            calibration_dir=str(calibration_dir),
            created_at_ns=time_ns(),
        )

    def __post_init__(self) -> None:
        if not self.robot_id or not self.device_id:
            raise ValueError("robot_id and device_id must be non-empty")
        if self.role not in ("leader", "follower"):
            raise ValueError("role must be leader or follower")
        if not self.calibration_dir or self.created_at_ns <= 0:
            raise ValueError("calibration_dir and created_at_ns are required")
        if self.schema_version != CALIBRATION_PROFILE_VERSION:
            raise ValueError(f"unsupported calibration profile: {self.schema_version}")

    def save(self, path: str | Path) -> None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(asdict(self), indent=2, sort_keys=True) + "\n")

    @classmethod
    def load(cls, path: str | Path, *, robot_id: str, role: CalibrationRole) -> CalibrationProfile:
        payload = json.loads(Path(path).read_text())
        profile = cls(**payload)
        if profile.robot_id != robot_id or profile.role != role:
            raise ValueError("calibration profile does not match this device")
        return profile
