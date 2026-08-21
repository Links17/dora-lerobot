"""Dora replay-node runtime helper."""

from __future__ import annotations

from collections.abc import Iterator

from dora_lerobot.bridges.lerobot_replay import LeRobotReplay
from dora_lerobot.contracts.models import Action
from dora_lerobot.nodes.codec import action_to_message


def next_action(actions: Iterator[Action]) -> Action | None:
    try:
        return next(actions)
    except StopIteration:
        return None


def run(replay: LeRobotReplay, *, node_id: str | None = None) -> None:
    from dora import Node

    node = Node(node_id)
    actions = replay.actions()
    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] == "ERROR":
            raise RuntimeError(event["error"])
        if event["type"] == "INPUT" and event["id"] == "tick":
            action = next_action(actions)
            if action is None:
                break
            node.send_output("action", action_to_message(action), event.get("metadata"))
