from __future__ import annotations

from math import pi
from pathlib import Path
from time import time_ns

import pytest

from dora_lerobot.adapters.so_arm import SoArmTeleopMapper
from dora_lerobot.contracts import Action
from dora_lerobot.drivers.feetech import LeRobotSOFollowerDriver, LeRobotSOLeader
from dora_lerobot.runtime.so_arm import load_hardware_configuration


class FakeFollowerRuntime:
    def __init__(self) -> None:
        self.connected = False
        self.disconnected = False
        self.torque: list[bool] = []
        self.sent: list[dict[str, float]] = []

    def connect(self, *, calibrate: bool = True) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.disconnected = True

    def get_observation(self) -> dict[str, float]:
        return {"shoulder_pan.pos": 1.0, "elbow_flex.pos": -0.5}

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        self.sent.append(action)
        return action


class FakeBus:
    def __init__(self) -> None:
        self.writes: list[tuple[str, dict[str, int]]] = []

    def sync_write(self, register: str, values: dict[str, int]) -> None:
        self.writes.append((register, values))


class FakeLeaderRuntime:
    def connect(self, *, calibrate: bool = True) -> None: ...

    def disconnect(self) -> None: ...

    def get_action(self) -> dict[str, float]:
        return {"shoulder_pan.pos": 1.0, "elbow_flex.pos": -0.5}


def test_lerobot_follower_driver_is_safe_until_explicitly_enabled() -> None:
    runtime = FakeFollowerRuntime()
    runtime.bus = FakeBus()
    driver = LeRobotSOFollowerDriver(runtime, ("shoulder_pan", "elbow_flex"))

    driver.connect()
    assert runtime.connected
    assert runtime.bus.writes == [("Torque_Enable", {"shoulder_pan": 0, "elbow_flex": 0})]

    assert driver.read_positions_rad() == {"shoulder_pan": 1.0, "elbow_flex": -0.5}
    with pytest.raises(RuntimeError, match="disabled"):
        driver.write_positions_rad({"shoulder_pan": 0.0, "elbow_flex": 0.0})

    driver.enable_torque(True)
    driver.write_positions_rad({"shoulder_pan": 0.2, "elbow_flex": -0.2})
    assert runtime.sent == [{"shoulder_pan.pos": 0.2, "elbow_flex.pos": -0.2}]


def test_leader_reads_action_without_control_access() -> None:
    leader = LeRobotSOLeader(FakeLeaderRuntime(), ("shoulder_pan", "elbow_flex"))
    leader.connect()
    assert leader.read_positions_rad() == {"shoulder_pan": 1.0, "elbow_flex": -0.5}


def test_lerobot_degree_runtime_is_converted_to_the_radian_contract() -> None:
    runtime = FakeFollowerRuntime()
    runtime.bus = FakeBus()
    driver = LeRobotSOFollowerDriver(runtime, ("shoulder_pan", "elbow_flex"), use_degrees=True)
    driver.connect()
    assert driver.read_positions_rad()["shoulder_pan"] == pytest.approx(pi / 180)
    driver.enable_torque(True)
    driver.write_positions_rad({"shoulder_pan": pi, "elbow_flex": -pi / 2})
    assert runtime.sent == [{"shoulder_pan.pos": 180.0, "elbow_flex.pos": -90.0}]


def test_so_arm_teleop_mapper_validates_order_and_applies_gain() -> None:
    mapper = SoArmTeleopMapper(
        leader_joint_names=("shoulder_pan", "elbow_flex"),
        follower_joint_names=("shoulder_pan", "elbow_flex"),
        gains={"elbow_flex": -2.0},
    )
    action = mapper.map_positions({"shoulder_pan": 1.0, "elbow_flex": -0.5}, time_ns())
    assert action == Action(("shoulder_pan", "elbow_flex"), (1.0, 1.0), action.timestamp_ns)

    with pytest.raises(ValueError, match="joint names"):
        SoArmTeleopMapper(("leader",), ("follower",))


def test_hardware_configuration_expands_calibration_paths_and_accepts_leader(tmp_path) -> None:
    config = tmp_path / "so-arm.yaml"
    config.write_text(
        """robot:
  robot_id: so-arm-a
  joints:
    - {name: shoulder_pan, minimum_rad: -1, maximum_rad: 1}
follower:
  device_id: follower-a
  port: /dev/follower
  calibration_dir: ~/calibration/follower
  calibration_profile: ~/profiles/follower.json
leader:
  device_id: leader-a
  port: /dev/leader
  calibration_dir: ~/calibration/leader
  calibration_profile: ~/profiles/leader.json
"""
    )
    hardware = load_hardware_configuration(config)
    assert hardware.leader_id == "leader-a"
    assert str(hardware.calibration_dir).startswith(str(Path.home()))
