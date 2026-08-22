"""Explicit B601 DM/RS real-hardware composition and calibration commands."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from math import isfinite
from pathlib import Path
from typing import Literal

import yaml

from dora_lerobot.adapters.base import RobotConfiguration, SafeRobotAdapter
from dora_lerobot.adapters.calibration import CalibrationProfile
from dora_lerobot.adapters.dm import DmRobotAdapter
from dora_lerobot.adapters.rs import RsRobotAdapter
from dora_lerobot.adapters.so_arm import configuration_from_mapping
from dora_lerobot.drivers.b601 import LeRobotB601Driver

B601_ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)
B601_JOINTS = (*B601_ARM_JOINTS, "gripper")
B601Kind = Literal["dm", "rs"]


@dataclass(frozen=True, slots=True)
class RsControlParameters:
    """RS-only vendor MIT configuration, deliberately outside Dora graphs."""

    mit_kp: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 50.0,
            "shoulder_lift": 150.0,
            "elbow_flex": 150.0,
            "wrist_flex": 50.0,
            "wrist_yaw": 50.0,
            "wrist_roll": 50.0,
        }
    )
    mit_kd: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 3.0,
            "shoulder_lift": 10.0,
            "elbow_flex": 10.0,
            "wrist_flex": 5.0,
            "wrist_yaw": 4.0,
            "wrist_roll": 4.0,
        }
    )
    gripper_mit_kp: float = 24.0
    gripper_mit_torque_limit: float = 10.0
    gripper_mit_hold_torque_limit: float = 10.0
    max_relative_target_deg: float = 10.0
    gravity_compensation: bool = True
    gravity_urdf_path: str | None = None

    def __post_init__(self) -> None:
        for name, values in (("mit_kp", self.mit_kp), ("mit_kd", self.mit_kd)):
            if set(values) != set(B601_ARM_JOINTS):
                raise ValueError(f"{name} must contain exactly the six B601 arm joints")
            if any(not isfinite(value) or not 0 <= value <= 500 for value in values.values()):
                raise ValueError(f"{name} values must be finite and within 0..500")
        for value in (
            self.gripper_mit_kp,
            self.gripper_mit_torque_limit,
            self.gripper_mit_hold_torque_limit,
            self.max_relative_target_deg,
        ):
            if not isfinite(value) or value < 0:
                raise ValueError("RS control values must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class B601HardwareConfiguration:
    kind: B601Kind
    robot: RobotConfiguration
    device_id: str
    channel: str
    calibration_dir: Path
    calibration_profile: Path
    dm_serial_baud: int = 921600
    rs_control: RsControlParameters | None = None


def load_hardware_configuration(path: str | Path, *, kind: B601Kind) -> B601HardwareConfiguration:
    payload = yaml.safe_load(Path(path).read_text())
    if not isinstance(payload, dict) or not isinstance(payload.get("robot"), dict):
        raise TypeError("B601 hardware configuration requires a robot mapping")
    device = payload.get("device")
    if not isinstance(device, dict):
        raise TypeError("B601 hardware configuration requires a device mapping")
    rs_control = None
    if kind == "rs":
        control = payload.get("rs_control", {})
        if not isinstance(control, dict):
            raise TypeError("rs_control must be a mapping")
        rs_control = RsControlParameters(**control)
    robot = configuration_from_mapping(payload["robot"])
    if robot.joint_names != B601_JOINTS:
        raise ValueError(
            "B601 requires the fixed seven joint layout including wrist_yaw and gripper"
        )
    return B601HardwareConfiguration(
        kind=kind,
        robot=robot,
        device_id=str(device["device_id"]),
        channel=str(device["channel"]),
        calibration_dir=Path(device["calibration_dir"]).expanduser(),
        calibration_profile=Path(device["calibration_profile"]).expanduser(),
        dm_serial_baud=int(device.get("dm_serial_baud", 921600)),
        rs_control=rs_control,
    )


def create_adapter(configuration: B601HardwareConfiguration) -> SafeRobotAdapter:
    """Create, but never connect or enable, the vendor B601 runtime."""
    CalibrationProfile.load(
        configuration.calibration_profile,
        robot_id=configuration.robot.robot_id,
        role="follower",
    )
    runtime = _make_runtime(configuration)
    driver = LeRobotB601Driver(runtime, configuration.robot.joint_names)
    if configuration.kind == "dm":
        return DmRobotAdapter(driver, configuration.robot)
    return RsRobotAdapter(driver, configuration.robot)


def calibrate_device(configuration: B601HardwareConfiguration) -> None:
    """Run the explicit B601 mechanical-zero prompt and persist its profile."""
    runtime = _make_runtime(configuration)
    try:
        runtime.connect(calibrate=True)
    finally:
        runtime.disconnect()
    CalibrationProfile.create(
        robot_id=configuration.robot.robot_id,
        role="follower",
        device_id=configuration.device_id,
        calibration_dir=configuration.calibration_dir,
    ).save(configuration.calibration_profile)


def _make_runtime(configuration: B601HardwareConfiguration):
    try:
        import lerobot_robot_seeed_b601  # noqa: F401
        from lerobot.robots.utils import make_robot_from_config
    except ImportError as error:
        raise RuntimeError(
            "B601 runtime is unavailable; install the pinned lerobot_robot_seeed_b601 dependency"
        ) from error
    if configuration.kind == "dm":
        from lerobot_robot_seeed_b601 import SeeedB601DMFollowerConfig

        config = SeeedB601DMFollowerConfig(
            id=configuration.device_id,
            port=configuration.channel,
            calibration_dir=configuration.calibration_dir,
            cameras={},
            can_adapter="damiao",
            dm_serial_baud=configuration.dm_serial_baud,
        )
    else:
        from lerobot_robot_seeed_b601 import SeeedB601RSFollowerConfig

        control = configuration.rs_control or RsControlParameters()
        config = SeeedB601RSFollowerConfig(
            id=configuration.device_id,
            port=configuration.channel,
            calibration_dir=configuration.calibration_dir,
            cameras={},
            can_adapter="socketcan",
            mit_kp=control.mit_kp,
            mit_kd=control.mit_kd,
            gripper_mit_kp=control.gripper_mit_kp,
            gripper_mit_torque_limit=control.gripper_mit_torque_limit,
            gripper_mit_hold_torque_limit=control.gripper_mit_hold_torque_limit,
            max_relative_target=control.max_relative_target_deg,
            gravity_compensation=control.gravity_compensation,
            gravity_urdf_path=control.gravity_urdf_path,
        )
    return make_robot_from_config(config)


def _main(kind: B601Kind) -> None:
    parser = argparse.ArgumentParser(description=f"B601 {kind.upper()} real hardware safety check")
    parser.add_argument("--hardware-config", required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--connect", action="store_true", help="explicitly open the CAN device")
    mode.add_argument(
        "--calibrate", action="store_true", help="run explicit mechanical-zero calibration"
    )
    args = parser.parse_args()
    configuration = load_hardware_configuration(args.hardware_config, kind=kind)
    if args.calibrate:
        calibrate_device(configuration)
        print({"robot_id": configuration.robot.robot_id, "state": "calibrated"})
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


def dm_main() -> None:
    _main("dm")


def rs_main() -> None:
    _main("rs")
