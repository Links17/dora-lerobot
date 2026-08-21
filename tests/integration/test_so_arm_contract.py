from time import time_ns

from dora_lerobot.bridges import LeRobotRecorder, LeRobotReplay
from dora_lerobot.contracts import Action, Observation


def test_recorded_episode_is_readable_and_replayable(tmp_path):
    recorder = LeRobotRecorder.create(
        root=tmp_path,
        repo_id="local/so_arm",
        robot_type="so_arm",
        joint_names=("joint_1",),
        fps=30,
    )
    recorder.append(
        Observation(time_ns(), {"joint_1": 0.0}),
        Action(("joint_1",), (0.1,), time_ns()),
        task="acceptance fixture",
    )
    recorder.save_episode()
    recorder.finalize()
    replay = LeRobotReplay.open(
        root=str(recorder.root), repo_id="local/so_arm", joint_names=("joint_1",)
    )
    assert next(replay.actions()).timestamp_ns > 0
