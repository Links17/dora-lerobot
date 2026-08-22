from __future__ import annotations

from math import pi

import pytest

from dora_lerobot.drivers.b601 import LeRobotB601Driver
from dora_lerobot.runtime.b601 import B601_JOINTS, RsControlParameters, load_hardware_configuration


class FakeB601Runtime:
    def __init__(self) -> None:
        self.connected = False
        self.disabled = 0
        self.configured = 0
        self.disconnected = False
        self.sent: list[dict[str, float]] = []

    def connect(self, *, calibrate: bool = True) -> None:
        self.connected = True

    def disable_torque(self) -> None:
        self.disabled += 1

    def configure(self) -> None:
        self.configured += 1

    def get_observation(self) -> dict[str, float]:
        return {"shoulder_pan.pos": 180.0, "wrist_yaw.pos": -90.0}

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        self.sent.append(action)
        return action

    def disconnect(self) -> None:
        self.disconnected = True


def test_b601_driver_disables_after_connect_and_avoids_vendor_safe_zero_on_disconnect() -> None:
    runtime = FakeB601Runtime()
    driver = LeRobotB601Driver(runtime, ("shoulder_pan", "wrist_yaw"))

    driver.connect()
    assert runtime.disabled == 1
    assert driver.read_positions_rad() == pytest.approx({"shoulder_pan": pi, "wrist_yaw": -pi / 2})

    with pytest.raises(RuntimeError, match="disabled"):
        driver.write_positions_rad({"shoulder_pan": 0.0, "wrist_yaw": 0.0})
    driver.enable_torque(True)
    driver.write_positions_rad({"shoulder_pan": pi, "wrist_yaw": -pi / 2})
    assert runtime.sent == [{"shoulder_pan.pos": 180.0, "wrist_yaw.pos": -90.0}]

    driver.disconnect()
    assert runtime.disabled == 2
    assert runtime.disconnected


def test_rs_control_parameters_reject_incomplete_kp_map() -> None:
    with pytest.raises(ValueError, match="exactly"):
        RsControlParameters(mit_kp={"shoulder_pan": 1.0})


def test_b601_hardware_configuration_requires_the_vendor_seven_joint_layout(tmp_path) -> None:
    config = tmp_path / "dm.yaml"
    config.write_text(
        """robot:
  robot_id: dm-a
  joints:
    - {name: shoulder_pan, minimum_rad: -1, maximum_rad: 1}
device:
  device_id: dm-a
  channel: /dev/tty.usbmodem1
  calibration_dir: ~/calibration/dm
  calibration_profile: ~/profiles/dm.json
"""
    )
    with pytest.raises(ValueError, match="seven joint"):
        load_hardware_configuration(config, kind="dm")

    assert len(B601_JOINTS) == 7
