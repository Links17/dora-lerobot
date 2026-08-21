from dora_lerobot.adapters.base import RobotConfiguration
from dora_lerobot.adapters.dm import DmRobotAdapter
from dora_lerobot.drivers import JointLimit, MemoryJointDriver


def test_dm_contract_with_driver_compatible_fake():
    driver = MemoryJointDriver(("joint_1",))
    adapter = DmRobotAdapter(
        driver, RobotConfiguration("dm", ("joint_1",), {"joint_1": JointLimit(-1, 1)})
    )
    adapter.connect()
    adapter.calibrate()
    assert adapter.read_observation().schema_version == "v1"
    adapter.safe_stop()
