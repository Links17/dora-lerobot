from time import time_ns

from dora_lerobot.adapters.base import RobotConfiguration
from dora_lerobot.adapters.so_arm import SoArmAdapter
from dora_lerobot.bridges import CloudPolicyBridge, LeRobotPolicyBridge
from dora_lerobot.contracts import Action, Observation
from dora_lerobot.drivers import JointLimit, MemoryJointDriver


def make_adapter():
    driver = MemoryJointDriver(("joint_1",))
    adapter = SoArmAdapter(
        driver, RobotConfiguration("test", ("joint_1",), {"joint_1": JointLimit(-1, 1)})
    )
    adapter.connect()
    adapter.calibrate()
    adapter.enable()
    return adapter, driver


def test_local_policy_action_is_filtered_before_driver_write():
    adapter, driver = make_adapter()
    bridge = LeRobotPolicyBridge(adapter, lambda _: (99.0,), "local-test")
    result = bridge.step(Observation(time_ns(), {"joint_1": 0.0}))
    assert result.positions_rad == (1.0,)
    assert driver.last_written_positions_rad == {"joint_1": 1.0}


def test_cloud_policy_uses_safe_local_fallback(monkeypatch):
    adapter, _ = make_adapter()
    fallback = lambda _: Action(("joint_1",), (0.25,), time_ns())
    bridge = CloudPolicyBridge(adapter, "https://policy.example", "remote-v1", 0.01, fallback)
    monkeypatch.setattr(bridge, "_request", lambda _: (_ for _ in ()).throw(TimeoutError()))
    result = bridge.step(Observation(time_ns(), {"joint_1": 0.0}))
    assert bridge.last_source == "fallback"
    assert result.positions_rad == (0.25,)
