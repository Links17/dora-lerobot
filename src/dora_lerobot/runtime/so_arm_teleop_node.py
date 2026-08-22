"""Runtime entry point for the real read-only SO-ARM leader Dora node."""

from __future__ import annotations

import os

from dora_lerobot.adapters.so_arm import SoArmTeleopMapper
from dora_lerobot.nodes.so_arm_teleop_node import main as node_main
from dora_lerobot.runtime.so_arm import create_leader, load_hardware_configuration


def factory():
    config_path = os.environ.get("DORA_LEROBOT_SO_ARM_CONFIG")
    if not config_path:
        raise RuntimeError("DORA_LEROBOT_SO_ARM_CONFIG must name an operator hardware config")
    configuration = load_hardware_configuration(config_path)
    return (
        create_leader(configuration),
        SoArmTeleopMapper(configuration.robot.joint_names, configuration.robot.joint_names),
    )


if __name__ == "__main__":
    node_main(factory)
