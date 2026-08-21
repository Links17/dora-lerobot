"""Dora node helper for a LeRobot recorder bridge."""

from __future__ import annotations

from dora_lerobot.bridges.lerobot_recorder import LeRobotRecorder
from dora_lerobot.nodes.codec import message_to_action, message_to_observation


class RecorderNode:
    def __init__(self, recorder: LeRobotRecorder, task: str) -> None:
        self.recorder = recorder
        self.task = task
        self._observation = None

    def accept_observation(self, value: object) -> None:
        self._observation = message_to_observation(value)

    def accept_action(self, value: object) -> None:
        if self._observation is None:
            raise RuntimeError("cannot record an action without an observation")
        self.recorder.append(self._observation, message_to_action(value), task=self.task)

    def save_episode(self) -> None:
        self.recorder.save_episode()
