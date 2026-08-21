from time import time_ns

import pytest

from dora_lerobot.contracts import Action, Observation, RobotCapabilities


def test_action_rejects_mismatched_joint_order_and_values():
    with pytest.raises(ValueError, match="joint_names"):
        Action(("joint_1",), (0.0, 1.0), time_ns())


def test_observation_is_versioned_timestamped_and_immutable():
    observation = Observation(timestamp_ns=time_ns(), joints_rad={"joint_1": 0.0})
    assert observation.schema_version == "v1"
    with pytest.raises(TypeError):
        observation.joints_rad["joint_1"] = 1.0  # type: ignore[index]


def test_capabilities_require_unique_joints():
    with pytest.raises(ValueError, match="unique"):
        RobotCapabilities(joint_names=("joint_1", "joint_1"))
