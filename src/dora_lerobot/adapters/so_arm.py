"""SO-ARM adapter and hardware-safe smoke entrypoint."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
from time import time_ns
from typing import Any

import yaml

from dora_lerobot.adapters.base import RobotConfiguration, SafeRobotAdapter
from dora_lerobot.contracts.models import Action
from dora_lerobot.drivers.base import JointLimit


class SoArmAdapter(SafeRobotAdapter):
    """SO-ARM robot semantic adapter; Feetech specifics stay in the driver."""


@dataclass(frozen=True, slots=True)
class SoArmTeleopMapper:
    """Explicit SO-ARM leader-to-follower position mapping.

    SO leader/follower share named joints. Per-joint gains support a mechanical
    inversion without letting serial details leak into a workflow.
    """

    leader_joint_names: tuple[str, ...]
    follower_joint_names: tuple[str, ...]
    gains: dict[str, float] = field(default_factory=dict)
    offsets_rad: dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if set(self.leader_joint_names) != set(self.follower_joint_names):
            raise ValueError("leader and follower joint names must match")
        unknown = (set(self.gains) | set(self.offsets_rad)) - set(self.follower_joint_names)
        if unknown:
            raise ValueError(f"teleop mapping has unknown joints: {sorted(unknown)}")

    def map_positions(
        self, leader_positions_rad: dict[str, float], timestamp_ns: int | None = None
    ) -> Action:
        if set(leader_positions_rad) != set(self.leader_joint_names):
            raise ValueError("leader positions joint set mismatch")
        return Action(
            joint_names=self.follower_joint_names,
            positions_rad=tuple(
                leader_positions_rad[name] * self.gains.get(name, 1.0)
                + self.offsets_rad.get(name, 0.0)
                for name in self.follower_joint_names
            ),
            timestamp_ns=timestamp_ns or time_ns(),
        )


def configuration_from_mapping(payload: dict[str, Any]) -> RobotConfiguration:
    joints = payload["joints"]
    joint_names = tuple(item["name"] for item in joints)
    limits = {
        item["name"]: JointLimit(
            minimum_rad=float(item["minimum_rad"]),
            maximum_rad=float(item["maximum_rad"]),
            max_velocity_rad_s=float(item.get("max_velocity_rad_s", 3.14)),
        )
        for item in joints
    }
    return RobotConfiguration(
        robot_id=str(payload["robot_id"]),
        joint_names=joint_names,
        joint_limits=limits,
        max_control_hz=float(payload.get("max_control_hz", 30)),
        camera_names=tuple(payload.get("camera_names", ())),
        has_gripper=bool(payload.get("has_gripper", False)),
    )


def load_configuration(path: str | Path) -> RobotConfiguration:
    with Path(path).open() as stream:
        payload = yaml.safe_load(stream)
    if not isinstance(payload, dict):
        raise TypeError("robot configuration must be a mapping")
    return configuration_from_mapping(payload)


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-ARM lifecycle smoke check")
    parser.add_argument("--config", required=True)
    parser.add_argument("--smoke-check", action="store_true")
    args = parser.parse_args()
    configuration = load_configuration(args.config)
    print(
        {
            "robot_id": configuration.robot_id,
            "state": "dry-run",
            "detail": "Use the SO-ARM runtime launcher with explicit serial configuration.",
        }
    )
