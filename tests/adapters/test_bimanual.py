from time import time_ns

from dora_lerobot.adapters.base import RobotConfiguration
from dora_lerobot.adapters.bimanual import BimanualRobotAdapter
from dora_lerobot.adapters.so_arm import SoArmAdapter
from dora_lerobot.contracts import Action
from dora_lerobot.drivers import JointLimit, MemoryJointDriver


def make_arm(robot_id: str):
    joints = ("joint_1",)
    return SoArmAdapter(
        MemoryJointDriver(joints),
        RobotConfiguration(robot_id, joints, {"joint_1": JointLimit(-1.0, 1.0)}),
    )


def test_bimanual_preserves_left_right_namespaces():
    bimanual = BimanualRobotAdapter(make_arm("left"), make_arm("right"))
    bimanual.connect()
    bimanual.calibrate()
    bimanual.enable()
    assert bimanual.capabilities.joint_names == ("left.joint_1", "right.joint_1")
    result = bimanual.apply_action(
        Action(bimanual.capabilities.joint_names, (0.2, -0.2), time_ns())
    )
    assert result.positions_rad == (0.2, -0.2)
    bimanual.disconnect()
