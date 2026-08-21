"""Protocols that preserve the workflow-to-hardware boundary."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from .models import Action, LifecycleState, Observation, RobotCapabilities


@runtime_checkable
class RobotAdapter(Protocol):
    """Robot semantic boundary consumed by Dora and LeRobot bridges."""

    @property
    def capabilities(self) -> RobotCapabilities: ...

    @property
    def lifecycle(self) -> LifecycleState: ...

    def connect(self) -> None: ...

    def calibrate(self) -> None: ...

    def enable(self) -> None: ...

    def disable(self) -> None: ...

    def read_observation(self) -> Observation: ...

    def apply_action(self, action: Action) -> Action: ...

    def safe_stop(self) -> None: ...

    def disconnect(self) -> None: ...
