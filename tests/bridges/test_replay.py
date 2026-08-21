from time import time_ns

import pytest

from dora_lerobot.bridges import LeRobotRecorder, LeRobotReplay
from dora_lerobot.contracts import Action, Observation


def test_replay_preserves_recorded_action_order(tmp_path):
    recorder = LeRobotRecorder.create(
        root=tmp_path,
        repo_id="local/so_arm",
        robot_type="so_arm",
        joint_names=("joint_1", "joint_2"),
        fps=30,
    )
    recorder.append(
        Observation(time_ns(), {"joint_1": 0.0, "joint_2": 0.0}),
        Action(("joint_1", "joint_2"), (0.1, -0.1), time_ns()),
        task="move",
    )
    recorder.save_episode()
    recorder.finalize()
    actions = list(
        LeRobotReplay.open(
            root=str(recorder.root), repo_id="local/so_arm", joint_names=("joint_1", "joint_2")
        ).actions()
    )
    assert actions[0].positions_rad == pytest.approx((0.1, -0.1))
