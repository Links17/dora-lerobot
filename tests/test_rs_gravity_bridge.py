from dora_lerobot.bridges.rs_gravity import RsGravityBridge
from dora_lerobot.contracts.models import Action, ControlMode, Observation


def test_rs_gravity_bridge_attaches_arm_torque_and_keeps_gripper_passive():
    bridge = RsGravityBridge(lambda q: [1.0, -2.0, 3.0, -4.0, 5.0, -6.0])
    joints = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper")
    observation = Observation(10, {name: 0.1 for name in joints})
    action = Action(joints, (0.0,) * 7, 11, ControlMode.POSITION)

    assert bridge.augment(action, observation) == [1.0, -2.0, 3.0, -4.0, 5.0, -6.0, 0.0]


def test_rs_gravity_bridge_rejects_invalid_dynamics_output():
    bridge = RsGravityBridge(lambda _q: [float("nan")] * 6)
    joints = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper")
    observation = Observation(10, {name: 0.1 for name in joints})
    action = Action(joints, (0.0,) * 7, 11, ControlMode.POSITION)

    try:
        bridge.augment(action, observation)
    except ValueError as error:
        assert "finite" in str(error)
    else:
        raise AssertionError("invalid gravity output must be rejected")
