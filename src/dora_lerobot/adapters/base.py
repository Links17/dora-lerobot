"""Safe robot adapter implementation independent of device protocol."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from time import time_ns

from dora_lerobot.contracts.models import (
    Action,
    ControlMode,
    LifecycleState,
    Observation,
    RobotCapabilities,
)
from dora_lerobot.drivers.base import JointDriver, JointLimit


@dataclass(frozen=True, slots=True)
class RobotConfiguration:
    robot_id: str
    joint_names: tuple[str, ...]
    joint_limits: Mapping[str, JointLimit]
    max_control_hz: float = 30.0
    camera_names: tuple[str, ...] = ()
    has_gripper: bool = False

    def __post_init__(self) -> None:
        if set(self.joint_names) != set(self.joint_limits):
            raise ValueError("joint_limits must exactly match joint_names")


class SafeRobotAdapter:
    """Enforces lifecycle and local action limits before calling a driver."""

    def __init__(self, driver: JointDriver, configuration: RobotConfiguration) -> None:
        if tuple(driver.joint_names) != configuration.joint_names:
            raise ValueError("driver joint_names must match configuration order")
        self.driver = driver
        self.configuration = configuration
        self._lifecycle = LifecycleState.DISCONNECTED
        self._last_fault: str | None = None

    @property
    def capabilities(self) -> RobotCapabilities:
        return RobotCapabilities(
            joint_names=self.configuration.joint_names,
            control_modes=frozenset({ControlMode.POSITION}),
            has_gripper=self.configuration.has_gripper,
            camera_names=self.configuration.camera_names,
            max_control_hz=self.configuration.max_control_hz,
        )

    @property
    def lifecycle(self) -> LifecycleState:
        return self._lifecycle

    def max_position(self, joint_name: str) -> float:
        return self.configuration.joint_limits[joint_name].maximum_rad

    def connect(self) -> None:
        if self._lifecycle is not LifecycleState.DISCONNECTED:
            return
        self.driver.connect()
        self._lifecycle = LifecycleState.CONNECTED

    def calibrate(self) -> None:
        self._require(LifecycleState.CONNECTED, LifecycleState.DISABLED, LifecycleState.CALIBRATED)
        self.driver.read_positions_rad()
        self._lifecycle = LifecycleState.CALIBRATED

    def enable(self) -> None:
        self._require(LifecycleState.CALIBRATED, LifecycleState.DISABLED)
        self.driver.enable_torque(True)
        self._lifecycle = LifecycleState.ENABLED

    def disable(self) -> None:
        if self._lifecycle is LifecycleState.DISCONNECTED:
            return
        self.driver.enable_torque(False)
        self._lifecycle = LifecycleState.DISABLED

    def read_observation(self) -> Observation:
        self._require(
            LifecycleState.CONNECTED,
            LifecycleState.CALIBRATED,
            LifecycleState.DISABLED,
            LifecycleState.ENABLED,
        )
        try:
            positions = self.driver.read_positions_rad()
            if set(positions) != set(self.configuration.joint_names):
                raise RuntimeError("driver returned an unexpected joint set")
            return Observation(timestamp_ns=time_ns(), joints_rad=positions, fault=self._last_fault)
        except Exception as error:
            self._fault(error)
            raise

    def apply_action(self, action: Action) -> Action:
        self._require(LifecycleState.ENABLED)
        if action.control_mode is not ControlMode.POSITION:
            raise ValueError("this adapter only supports position actions")
        if action.joint_names != self.configuration.joint_names:
            raise ValueError("action joint_names must match adapter joint order")
        safe = {
            joint: self.configuration.joint_limits[joint].clamp(position)
            for joint, position in action.as_mapping().items()
        }
        safe_action = Action(
            joint_names=self.configuration.joint_names,
            positions_rad=tuple(safe[name] for name in self.configuration.joint_names),
            timestamp_ns=action.timestamp_ns,
            control_mode=action.control_mode,
        )
        try:
            self.driver.write_positions_rad(safe_action.as_mapping())
        except Exception as error:
            self._fault(error)
            raise
        return safe_action

    def safe_stop(self) -> None:
        if self._lifecycle is not LifecycleState.DISCONNECTED:
            try:
                self.driver.enable_torque(False)
            finally:
                self._lifecycle = LifecycleState.DISABLED

    def disconnect(self) -> None:
        if self._lifecycle is LifecycleState.DISCONNECTED:
            return
        try:
            self.safe_stop()
        finally:
            self.driver.disconnect()
            self._lifecycle = LifecycleState.DISCONNECTED

    def _require(self, *states: LifecycleState) -> None:
        if self._lifecycle not in states:
            accepted = ", ".join(state.value for state in states)
            raise RuntimeError(f"adapter is {self._lifecycle.value}; expected one of: {accepted}")

    def _fault(self, error: Exception) -> None:
        self._last_fault = str(error)
        self._lifecycle = LifecycleState.FAULTED
        try:
            self.driver.enable_torque(False)
        except Exception:
            pass
