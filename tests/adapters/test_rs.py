from dora_lerobot.adapters.base import RobotConfiguration
from dora_lerobot.adapters.rs import RsRobotAdapter
from dora_lerobot.drivers import JointLimit, MemoryJointDriver


def test_rs_contract_with_driver_compatible_fake():
    driver = MemoryJointDriver(("joint_1",))
    adapter = RsRobotAdapter(
        driver, RobotConfiguration("rs", ("joint_1",), {"joint_1": JointLimit(-1, 1)})
    )
    adapter.connect()
    adapter.calibrate()
    assert adapter.read_observation().schema_version == "v1"
    adapter.safe_stop()
