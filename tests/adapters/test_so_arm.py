from time import time_ns

import pytest

from dora_lerobot.adapters.base import RobotConfiguration
from dora_lerobot.adapters.so_arm import SoArmAdapter
from dora_lerobot.contracts import Action, LifecycleState
from dora_lerobot.drivers import JointLimit, MemoryJointDriver


@pytest.fixture
def so_arm():
    joints = ("shoulder", "elbow")
    driver = MemoryJointDriver(joints)
    configuration = RobotConfiguration(
        robot_id="test",
        joint_names=joints,
        joint_limits={name: JointLimit(-1.0, 1.0) for name in joints},
    )
    adapter = SoArmAdapter(driver, configuration)
    adapter.connect()
    adapter.calibrate()
    return adapter, driver


def test_adapter_clamps_action_before_driver_write(so_arm):
    adapter, driver = so_arm
    adapter.enable()
    safe_action = adapter.apply_action(Action(("shoulder", "elbow"), (9.0, -9.0), time_ns()))
    assert safe_action.positions_rad == (1.0, -1.0)
    assert driver.last_written_positions_rad == {"shoulder": 1.0, "elbow": -1.0}


def test_adapter_rejects_actions_while_disabled(so_arm):
    adapter, _ = so_arm
    with pytest.raises(RuntimeError, match="calibrated"):
        adapter.apply_action(Action(("shoulder", "elbow"), (0.0, 0.0), time_ns()))


def test_safe_stop_disables_torque(so_arm):
    adapter, driver = so_arm
    adapter.enable()
    adapter.safe_stop()
    assert adapter.lifecycle is LifecycleState.DISABLED
    assert not driver.torque_enabled
