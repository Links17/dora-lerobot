"""SO-ARM adapter and hardware-safe smoke entrypoint."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml

from dora_lerobot.adapters.base import RobotConfiguration, SafeRobotAdapter
from dora_lerobot.drivers.base import JointLimit, MemoryJointDriver


class SoArmAdapter(SafeRobotAdapter):
    """SO-ARM robot semantic adapter; Feetech specifics stay in the driver."""


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
    # A production launcher injects a FeetechDriver. This CLI is deliberately dry
    # until an operator provides a hardware-specific launcher outside the graph.
    adapter = SoArmAdapter(MemoryJointDriver(configuration.joint_names), configuration)
    adapter.connect()
    adapter.calibrate()
    observation = adapter.read_observation()
    print(
        {
            "robot_id": configuration.robot_id,
            "state": adapter.lifecycle.value,
            "observation": observation.joints_rad,
        }
    )
    adapter.safe_stop()
    adapter.disconnect()
