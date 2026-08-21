"""Seeed DM robot adapter."""

from __future__ import annotations

from dora_lerobot.adapters.base import RobotConfiguration, SafeRobotAdapter
from dora_lerobot.drivers.damiao import DamiaoMitDriver


class DmRobotAdapter(SafeRobotAdapter):
    def __init__(self, driver: DamiaoMitDriver, configuration: RobotConfiguration) -> None:
        super().__init__(driver, configuration)
