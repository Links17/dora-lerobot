"""Local policy bridge that always returns through the robot safety boundary."""

from __future__ import annotations

from collections.abc import Callable
from time import time_ns
from typing import Any

from dora_lerobot.contracts.models import Action, Observation
from dora_lerobot.contracts.protocols import RobotAdapter

PolicyCallable = Callable[[Observation], Action | tuple[float, ...] | list[float]]


class LeRobotPolicyBridge:
    def __init__(self, adapter: RobotAdapter, policy: PolicyCallable, model_id: str) -> None:
        if not model_id:
            raise ValueError("model_id is required")
        self.adapter = adapter
        self.policy = policy
        self.model_id = model_id

    def step(self, observation: Observation) -> Action:
        result = self.policy(observation)
        if isinstance(result, Action):
            action = result
        else:
            action = Action(
                joint_names=self.adapter.capabilities.joint_names,
                positions_rad=tuple(float(value) for value in result),
                timestamp_ns=time_ns(),
            )
        if action.schema_version != observation.schema_version:
            raise ValueError("policy action schema version does not match observation")
        return self.adapter.apply_action(action)

    @classmethod
    def from_lerobot_policy(
        cls, adapter: RobotAdapter, policy: Any, model_id: str
    ) -> LeRobotPolicyBridge:
        """Wrap a loaded LeRobot policy using its conventional `select_action` API."""

        def invoke(observation: Observation) -> tuple[float, ...]:
            import torch

            state = torch.tensor(
                [[observation.joints_rad[name] for name in adapter.capabilities.joint_names]],
                dtype=torch.float32,
            )
            output = policy.select_action({"observation.state": state})
            return tuple(float(value) for value in output.squeeze(0).detach().cpu().tolist())

        return cls(adapter, invoke, model_id)
