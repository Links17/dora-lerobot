"""Dora 1.0 dynamic node for a robot adapter.

The factory is injected by a launcher so this node never imports a hardware SDK
or reads serial/CAN settings from a graph.
"""

from __future__ import annotations

import os
from collections.abc import Callable

from dora_lerobot.contracts.protocols import RobotAdapter
from dora_lerobot.nodes.codec import action_to_message, message_to_action, observation_to_message


def run(adapter: RobotAdapter, *, node_id: str | None = None) -> None:
    from dora import Node

    node = Node(node_id)
    adapter.connect()
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] == "ERROR":
                raise RuntimeError(event["error"])
            if event["type"] != "INPUT":
                continue
            event_id = event["id"]
            metadata = event.get("metadata")
            if event_id == "calibrate":
                adapter.calibrate()
            elif event_id == "enable":
                adapter.enable()
            elif event_id == "disable":
                adapter.disable()
            elif event_id == "tick":
                node.send_output(
                    "observation", observation_to_message(adapter.read_observation()), metadata
                )
            elif event_id == "action":
                safe_action = adapter.apply_action(message_to_action(event["value"]))
                node.send_output("safe_action", action_to_message(safe_action), metadata)
    finally:
        adapter.safe_stop()
        adapter.disconnect()


def main(factory: Callable[[], RobotAdapter] | None = None) -> None:
    if factory is None:
        raise RuntimeError(
            "A hardware launcher must provide an adapter factory; graph configuration never contains device protocol details."
        )
    run(factory(), node_id=os.getenv("DORA_NODE_ID"))
