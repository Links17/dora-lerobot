from time import time_ns

from dora_lerobot.bridges import LeRobotRecorder
from dora_lerobot.contracts import Action, Observation


def test_recorder_writes_a_dataset_v3_episode(tmp_path):
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
        task="pick up block",
    )
    recorder.save_episode()
    recorder.finalize()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(repo_id="local/so_arm", root=recorder.root)
    assert dataset.num_episodes == 1
    assert len(dataset) == 1
