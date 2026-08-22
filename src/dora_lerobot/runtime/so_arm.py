"""Explicit, hardware-safe SO-ARM runtime composition."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import yaml

from dora_lerobot.adapters.base import RobotConfiguration
from dora_lerobot.adapters.calibration import CalibrationProfile
from dora_lerobot.adapters.so_arm import SoArmAdapter, configuration_from_mapping
from dora_lerobot.drivers.feetech import LeRobotSOFollowerDriver, LeRobotSOLeader


@dataclass(frozen=True, slots=True)
class SoArmHardwareConfiguration:
    robot: RobotConfiguration
    follower_id: str
    follower_port: str
    calibration_dir: Path
    calibration_profile: Path
    leader_id: str | None = None
    leader_port: str | None = None
    leader_calibration_dir: Path | None = None
    leader_calibration_profile: Path | None = None


def load_hardware_configuration(path: str | Path) -> SoArmHardwareConfiguration:
    source = Path(path)
    payload = yaml.safe_load(source.read_text())
    if not isinstance(payload, dict):
        raise TypeError("SO-ARM hardware configuration must be a mapping")
    robot_payload = payload.get("robot")
    follower = payload.get("follower")
    leader = payload.get("leader")
    if not isinstance(robot_payload, dict) or not isinstance(follower, dict):
        raise TypeError("SO-ARM configuration requires robot and follower mappings")
    leader_fields: dict[str, str | Path | None] = {}
    if leader is not None:
        if not isinstance(leader, dict):
            raise TypeError("leader must be a mapping")
        leader_fields = {
            "leader_id": str(leader["device_id"]),
            "leader_port": str(leader["port"]),
            "leader_calibration_dir": Path(leader["calibration_dir"]).expanduser(),
            "leader_calibration_profile": Path(leader["calibration_profile"]).expanduser(),
        }
    return SoArmHardwareConfiguration(
        robot=configuration_from_mapping(robot_payload),
        follower_id=str(follower["device_id"]),
        follower_port=str(follower["port"]),
        calibration_dir=Path(follower["calibration_dir"]).expanduser(),
        calibration_profile=Path(follower["calibration_profile"]).expanduser(),
        **leader_fields,
    )


def create_adapter(configuration: SoArmHardwareConfiguration) -> SoArmAdapter:
    """Create but do not connect or enable a physical SO-ARM."""
    CalibrationProfile.load(
        configuration.calibration_profile,
        robot_id=configuration.robot.robot_id,
        role="follower",
    )
    from lerobot.robots.so_follower.config_so_follower import SO100FollowerConfig
    from lerobot.robots.utils import make_robot_from_config

    runtime = make_robot_from_config(
        SO100FollowerConfig(
            id=configuration.follower_id,
            port=configuration.follower_port,
            calibration_dir=configuration.calibration_dir,
            cameras={},
            use_degrees=True,
        )
    )
    gripper_range = (
        {
            "gripper": (
                configuration.robot.joint_limits["gripper"].minimum_rad,
                configuration.robot.joint_limits["gripper"].maximum_rad,
            )
        }
        if "gripper" in configuration.robot.joint_limits
        else {}
    )
    driver = LeRobotSOFollowerDriver(
        runtime,
        configuration.robot.joint_names,
        use_degrees=True,
        normalized_joint_ranges=gripper_range,
    )
    return SoArmAdapter(driver, configuration.robot)


def create_leader(configuration: SoArmHardwareConfiguration) -> LeRobotSOLeader:
    """Create the optional read-only SO-ARM leader without connecting it."""
    required = (
        configuration.leader_id,
        configuration.leader_port,
        configuration.leader_calibration_dir,
        configuration.leader_calibration_profile,
    )
    if any(value is None for value in required):
        raise ValueError("SO-ARM leader configuration is required for teleoperation")
    CalibrationProfile.load(
        configuration.leader_calibration_profile,
        robot_id=configuration.robot.robot_id,
        role="leader",
    )
    from lerobot.teleoperators.so_leader.config_so_leader import SO100LeaderConfig
    from lerobot.teleoperators.utils import make_teleoperator_from_config

    runtime = make_teleoperator_from_config(
        SO100LeaderConfig(
            id=configuration.leader_id,
            port=configuration.leader_port,
            calibration_dir=configuration.leader_calibration_dir,
            use_degrees=True,
        )
    )
    gripper_range = (
        {
            "gripper": (
                configuration.robot.joint_limits["gripper"].minimum_rad,
                configuration.robot.joint_limits["gripper"].maximum_rad,
            )
        }
        if "gripper" in configuration.robot.joint_limits
        else {}
    )
    return LeRobotSOLeader(
        runtime,
        configuration.robot.joint_names,
        use_degrees=True,
        normalized_joint_ranges=gripper_range,
    )


def calibrate_device(configuration: SoArmHardwareConfiguration, role: str) -> None:
    """Run LeRobot's explicit interactive calibration, then persist its identity profile."""
    if role == "follower":
        from lerobot.robots.so_follower.config_so_follower import SO100FollowerConfig
        from lerobot.robots.utils import make_robot_from_config

        runtime = make_robot_from_config(
            SO100FollowerConfig(
                id=configuration.follower_id,
                port=configuration.follower_port,
                calibration_dir=configuration.calibration_dir,
                cameras={},
                use_degrees=True,
            )
        )
        profile_path = configuration.calibration_profile
        device_id = configuration.follower_id
        calibration_dir = configuration.calibration_dir
    elif role == "leader":
        if (
            configuration.leader_id is None
            or configuration.leader_port is None
            or configuration.leader_calibration_dir is None
            or configuration.leader_calibration_profile is None
        ):
            raise ValueError("SO-ARM leader configuration is required for calibration")
        from lerobot.teleoperators.so_leader.config_so_leader import SO100LeaderConfig
        from lerobot.teleoperators.utils import make_teleoperator_from_config

        runtime = make_teleoperator_from_config(
            SO100LeaderConfig(
                id=configuration.leader_id,
                port=configuration.leader_port,
                calibration_dir=configuration.leader_calibration_dir,
                use_degrees=True,
            )
        )
        profile_path = configuration.leader_calibration_profile
        device_id = configuration.leader_id
        calibration_dir = configuration.leader_calibration_dir
    else:
        raise ValueError("role must be leader or follower")

    try:
        runtime.connect(calibrate=True)
    finally:
        runtime.disconnect()
    CalibrationProfile.create(
        robot_id=configuration.robot.robot_id,
        role=role,  # type: ignore[arg-type]
        device_id=device_id,
        calibration_dir=calibration_dir,
    ).save(profile_path)


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-ARM real hardware safety check")
    parser.add_argument("--hardware-config", required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--connect", action="store_true", help="explicitly open the serial device")
    mode.add_argument(
        "--calibrate",
        choices=("leader", "follower"),
        help="run LeRobot's interactive calibration for exactly one device",
    )
    args = parser.parse_args()
    configuration = load_hardware_configuration(args.hardware_config)
    if args.calibrate:
        calibrate_device(configuration, args.calibrate)
        print(
            {
                "robot_id": configuration.robot.robot_id,
                "state": "calibrated",
                "role": args.calibrate,
            }
        )
        return
    if args.connect:
        adapter = create_adapter(configuration)
        adapter.connect()
        try:
            adapter.calibrate()
            print({"robot_id": configuration.robot.robot_id, "state": adapter.lifecycle.value})
        finally:
            adapter.safe_stop()
            adapter.disconnect()
        return
    print({"robot_id": configuration.robot.robot_id, "state": "dry-run"})
