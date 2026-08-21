"""Seeed RS robot adapter."""

from __future__ import annotations

from dora_lerobot.adapters.base import RobotConfiguration, SafeRobotAdapter
from dora_lerobot.drivers.robstride import RobStrideMitDriver


class RsRobotAdapter(SafeRobotAdapter):
    def __init__(self, driver: RobStrideMitDriver, configuration: RobotConfiguration) -> None:
        super().__init__(driver, configuration)
