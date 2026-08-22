"""Inject a configured B601 DM or RS adapter into the generic Dora robot node."""

from __future__ import annotations

import os

from dora_lerobot.nodes.so_arm_node import main as node_main
from dora_lerobot.runtime.b601 import create_adapter, load_hardware_configuration


def factory():
    config_path = os.environ.get("DORA_LEROBOT_B601_CONFIG")
    kind = os.environ.get("DORA_LEROBOT_B601_KIND")
    if not config_path or kind not in {"dm", "rs"}:
        raise RuntimeError("DORA_LEROBOT_B601_CONFIG and DORA_LEROBOT_B601_KIND=dm|rs are required")
    return create_adapter(load_hardware_configuration(config_path, kind=kind))


if __name__ == "__main__":
    node_main(factory)
