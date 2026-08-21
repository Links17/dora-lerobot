"""Composable bimanual adapter."""

from __future__ import annotations

from time import time_ns

from dora_lerobot.contracts.models import Action, LifecycleState, Observation, RobotCapabilities
from dora_lerobot.contracts.protocols import RobotAdapter


class BimanualRobotAdapter:
    """Namespaced composition of two independently safe adapters."""

    def __init__(self, left: RobotAdapter, right: RobotAdapter) -> None:
        self.left = left
        self.right = right

    @property
    def capabilities(self) -> RobotCapabilities:
        return RobotCapabilities(
            joint_names=tuple(f"left.{name}" for name in self.left.capabilities.joint_names)
            + tuple(f"right.{name}" for name in self.right.capabilities.joint_names),
            control_modes=self.left.capabilities.control_modes
            & self.right.capabilities.control_modes,
            has_gripper=self.left.capabilities.has_gripper or self.right.capabilities.has_gripper,
            camera_names=tuple(f"left.{name}" for name in self.left.capabilities.camera_names)
            + tuple(f"right.{name}" for name in self.right.capabilities.camera_names),
            has_force_sensor=self.left.capabilities.has_force_sensor
            or self.right.capabilities.has_force_sensor,
            max_control_hz=min(
                self.left.capabilities.max_control_hz, self.right.capabilities.max_control_hz
            ),
        )

    @property
    def lifecycle(self) -> LifecycleState:
        if (
            self.left.lifecycle is LifecycleState.FAULTED
            or self.right.lifecycle is LifecycleState.FAULTED
        ):
            return LifecycleState.FAULTED
        if (
            self.left.lifecycle is LifecycleState.ENABLED
            and self.right.lifecycle is LifecycleState.ENABLED
        ):
            return LifecycleState.ENABLED
        return LifecycleState.DISABLED

    def connect(self) -> None:
        self.left.connect()
        try:
            self.right.connect()
        except Exception:
            self.left.disconnect()
            raise

    def calibrate(self) -> None:
        self.left.calibrate()
        self.right.calibrate()

    def enable(self) -> None:
        self.left.enable()
        try:
            self.right.enable()
        except Exception:
            self.left.safe_stop()
            raise

    def disable(self) -> None:
        self.left.disable()
        self.right.disable()

    def read_observation(self) -> Observation:
        left = self.left.read_observation()
        right = self.right.read_observation()
        joints = {f"left.{name}": value for name, value in left.joints_rad.items()}
        joints.update({f"right.{name}": value for name, value in right.joints_rad.items()})
        return Observation(
            timestamp_ns=max(left.timestamp_ns, right.timestamp_ns, time_ns()), joints_rad=joints
        )

    def apply_action(self, action: Action) -> Action:
        expected = self.capabilities.joint_names
        if action.joint_names != expected:
            raise ValueError("bimanual action order does not match namespaced capabilities")
        values = action.as_mapping()
        left_action = Action(
            self.left.capabilities.joint_names,
            tuple(values[f"left.{name}"] for name in self.left.capabilities.joint_names),
            action.timestamp_ns,
            action.control_mode,
        )
        right_action = Action(
            self.right.capabilities.joint_names,
            tuple(values[f"right.{name}"] for name in self.right.capabilities.joint_names),
            action.timestamp_ns,
            action.control_mode,
        )
        safe_left = self.left.apply_action(left_action)
        try:
            safe_right = self.right.apply_action(right_action)
        except Exception:
            self.safe_stop()
            raise
        safe = {f"left.{name}": value for name, value in safe_left.as_mapping().items()}
        safe.update({f"right.{name}": value for name, value in safe_right.as_mapping().items()})
        return Action(
            expected,
            tuple(safe[name] for name in expected),
            action.timestamp_ns,
            action.control_mode,
        )

    def safe_stop(self) -> None:
        self.left.safe_stop()
        self.right.safe_stop()

    def disconnect(self) -> None:
        try:
            self.safe_stop()
        finally:
            self.left.disconnect()
            self.right.disconnect()
