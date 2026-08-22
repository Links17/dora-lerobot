"""LeRobot B601 runtime wrapper for DM and RS follower arms."""

from __future__ import annotations

from collections.abc import Mapping
from math import degrees, radians
from typing import Any


class LeRobotB601Driver:
    """Preserves the radian contract around the vendor B601 runtime.

    The vendor runtime owns DM POS_VEL/FORCE_POS and RS MIT/gravity control.
    It exposes degrees, while this driver's public surface is radians.
    """

    def __init__(self, runtime: Any, joint_names: tuple[str, ...]) -> None:
        self.runtime = runtime
        self._joint_names = joint_names
        self._connected = False
        self._torque_enabled = False

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._joint_names

    def connect(self) -> None:
        if self._connected:
            return
        self.runtime.connect(calibrate=False)
        self._connected = True
        self.enable_torque(False)

    def disconnect(self) -> None:
        if not self._connected:
            return
        try:
            self.enable_torque(False)
            # The vendor disconnect performs an optional motion-to-zero routine.
            # Local safe stop must only disable torque, never introduce movement.
            self.runtime._emergency_disable_requested = True
        finally:
            self.runtime.disconnect()
            self._connected = False

    def enable_torque(self, enabled: bool) -> None:
        if not self._connected:
            raise RuntimeError("driver is disconnected")
        if enabled:
            self.runtime.configure()
        else:
            self.runtime.disable_torque()
        self._torque_enabled = enabled

    def read_positions_rad(self) -> Mapping[str, float]:
        if not self._connected:
            raise RuntimeError("driver is disconnected")
        observation = self.runtime.get_observation()
        return {name: radians(float(observation[f"{name}.pos"])) for name in self._joint_names}

    def write_positions_rad(self, positions_rad: Mapping[str, float]) -> None:
        if not self._connected:
            raise RuntimeError("driver is disconnected")
        if not self._torque_enabled:
            raise RuntimeError("driver torque is disabled")
        if set(positions_rad) != set(self._joint_names):
            raise ValueError("driver action joint set mismatch")
        self.runtime.send_action(
            {f"{name}.pos": degrees(positions_rad[name]) for name in self._joint_names}
        )
