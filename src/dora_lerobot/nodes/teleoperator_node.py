"""Teleoperator workflow boundary."""

from __future__ import annotations

from dora_lerobot.nodes.codec import action_to_message, message_to_action


def run(*, node_id: str | None = None) -> None:
    from dora import Node

    node = Node(node_id)
    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] == "ERROR":
            raise RuntimeError(event["error"])
        if event["type"] == "INPUT" and event["id"] == "leader_action":
            action = message_to_action(event["value"])
            node.send_output("action", action_to_message(action), event.get("metadata"))
