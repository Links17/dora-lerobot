"""Write synchronized workflow frames as LeRobot dataset v3 episodes."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from dora_lerobot.contracts.models import Action, Observation


@dataclass(slots=True)
class LeRobotRecorder:
    """Owns only LeRobot dataset writing, never hardware control."""

    dataset: object
    joint_names: tuple[str, ...]

    @property
    def root(self) -> Path:
        return Path(self.dataset.root)

    @classmethod
    def create(
        cls,
        *,
        root: str | Path,
        repo_id: str,
        robot_type: str,
        joint_names: tuple[str, ...],
        fps: int,
        image_shapes: Mapping[str, tuple[int, int, int]] | None = None,
    ) -> LeRobotRecorder:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        features: dict[str, dict] = {
            "observation.state": {
                "dtype": "float32",
                "shape": (len(joint_names),),
                "names": list(joint_names),
            },
            "action": {
                "dtype": "float32",
                "shape": (len(joint_names),),
                "names": list(joint_names),
            },
            "observation.timestamp_ns": {"dtype": "int64", "shape": (1,)},
        }
        for name, shape in (image_shapes or {}).items():
            features[f"observation.images.{name}"] = {
                "dtype": "image",
                "shape": shape,
                "names": ["height", "width", "channels"],
            }
        target_root = Path(root)
        if target_root.exists():
            target_root = target_root / repo_id.replace("/", "_")
        dataset = LeRobotDataset.create(
            repo_id=repo_id,
            root=target_root,
            fps=fps,
            features=features,
            robot_type=robot_type,
            use_videos=bool(image_shapes),
        )
        return cls(dataset=dataset, joint_names=joint_names)

    def append(self, observation: Observation, action: Action, *, task: str) -> None:
        if not task.strip():
            raise ValueError("task must not be empty")
        if (
            tuple(observation.joints_rad) != self.joint_names
            or action.joint_names != self.joint_names
        ):
            raise ValueError("observation/action joint order must match recorder joint_names")
        frame: dict[str, object] = {
            "observation.state": np.asarray(
                [observation.joints_rad[name] for name in self.joint_names], dtype=np.float32
            ),
            "action": np.asarray(action.positions_rad, dtype=np.float32),
            "observation.timestamp_ns": np.asarray([observation.timestamp_ns], dtype=np.int64),
            "task": task,
        }
        for name, image in observation.images.items():
            frame[f"observation.images.{name}"] = image
        self.dataset.add_frame(frame)

    def save_episode(self) -> None:
        self.dataset.save_episode()

    def discard_episode(self) -> None:
        self.dataset.clear_episode_buffer(delete_images=True)

    def finalize(self) -> None:
        self.dataset.finalize()
