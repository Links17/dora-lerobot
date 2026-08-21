"""Dora runtime helper for a local LeRobot policy bridge."""

from __future__ import annotations

from dora_lerobot.bridges.lerobot_policy import LeRobotPolicyBridge
from dora_lerobot.nodes.codec import action_to_message, message_to_observation


def run(bridge: LeRobotPolicyBridge, *, node_id: str | None = None) -> None:
    from dora import Node

    node = Node(node_id)
    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] == "ERROR":
            raise RuntimeError(event["error"])
        if event["type"] == "INPUT" and event["id"] == "observation":
            safe_action = bridge.step(message_to_observation(event["value"]))
            node.send_output("action", action_to_message(safe_action), event.get("metadata"))
