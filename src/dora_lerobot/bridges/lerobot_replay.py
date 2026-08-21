"""Read LeRobot episodes without exposing their storage format to workflows."""

from __future__ import annotations

from collections.abc import Iterator
from time import time_ns

import numpy as np

from dora_lerobot.contracts.models import Action


class LeRobotReplay:
    def __init__(self, dataset: object, joint_names: tuple[str, ...]) -> None:
        self.dataset = dataset
        self.joint_names = joint_names

    @classmethod
    def open(cls, *, root: str, repo_id: str, joint_names: tuple[str, ...]) -> LeRobotReplay:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        return cls(LeRobotDataset(repo_id=repo_id, root=root), joint_names)

    def actions(self) -> Iterator[Action]:
        for index in range(len(self.dataset)):
            frame = self.dataset[index]
            raw_action = frame["action"]
            if hasattr(raw_action, "detach"):
                raw_action = raw_action.detach().cpu().numpy()
            values = tuple(float(value) for value in np.asarray(raw_action).reshape(-1))
            if len(values) != len(self.joint_names):
                raise ValueError("dataset action shape does not match configured joint_names")
            timestamp = frame.get("observation.timestamp_ns", frame.get("timestamp", time_ns()))
            if hasattr(timestamp, "item"):
                timestamp = timestamp.item()
            yield Action(self.joint_names, values, int(timestamp))
