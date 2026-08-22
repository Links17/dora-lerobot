"""Dora node that turns a read-only leader sample into a common robot action."""

from __future__ import annotations

from collections.abc import Callable

from dora_lerobot.adapters.so_arm import SoArmTeleopMapper
from dora_lerobot.drivers.feetech import LeRobotSOLeader
from dora_lerobot.nodes.codec import action_to_message


def run(leader: LeRobotSOLeader, mapper: SoArmTeleopMapper, *, node_id: str | None = None) -> None:
    from dora import Node

    node = Node(node_id)
    leader.connect()
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] == "ERROR":
                raise RuntimeError(event["error"])
            if event["type"] == "INPUT" and event["id"] == "tick":
                action = mapper.map_positions(dict(leader.read_positions_rad()))
                node.send_output("action", action_to_message(action), event.get("metadata"))
    finally:
        leader.disconnect()


def main(
    factory: Callable[[], tuple[LeRobotSOLeader, SoArmTeleopMapper]] | None = None,
) -> None:
    if factory is None:
        raise RuntimeError("A hardware launcher must provide a SO-ARM leader factory.")
    leader, mapper = factory()
    run(leader, mapper)
