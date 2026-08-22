"""Runtime entry point that injects physical SO-ARM composition into a Dora node."""

from __future__ import annotations

import os

from dora_lerobot.nodes.so_arm_node import main as node_main
from dora_lerobot.runtime.so_arm import create_adapter, load_hardware_configuration


def factory():
    config_path = os.environ.get("DORA_LEROBOT_SO_ARM_CONFIG")
    if not config_path:
        raise RuntimeError("DORA_LEROBOT_SO_ARM_CONFIG must name an operator hardware config")
    return create_adapter(load_hardware_configuration(config_path))


if __name__ == "__main__":
    node_main(factory)
