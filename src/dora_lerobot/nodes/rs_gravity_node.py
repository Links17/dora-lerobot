"""RS gravity feed-forward node; emits a normal action plus bounded torque metadata."""
from __future__ import annotations

import os

from dora_lerobot.bridges.rs_gravity import pure_python_rs_gravity
from dora_lerobot.nodes.codec import decode_message, encode_message


def run(*, node_id: str | None = None) -> None:
    from dora import Node

    urdf = os.environ.get("DORA_LEROBOT_RS_GRAVITY_URDF", "").strip()
    if not urdf:
        raise RuntimeError("DORA_LEROBOT_RS_GRAVITY_URDF is required for RS gravity node")
    bridge = pure_python_rs_gravity(urdf)
    node = Node(node_id)
    latest_observation = None
    for event in node:
        if event["type"] in {"STOP", "ERROR"}:
            break
        if event["type"] != "INPUT":
            continue
        if event["id"] == "observation":
            latest_observation = decode_message(event["value"])
        elif event["id"] == "action" and latest_observation is not None:
            from dora_lerobot.contracts.models import Action, Observation

            observation = Observation(int(latest_observation["timestamp_ns"]), latest_observation["joints_rad"])
            payload = decode_message(event["value"])
            action = Action(tuple(payload["joint_names"]), tuple(payload["positions_rad"]), int(payload["timestamp_ns"]), payload.get("control_mode", "position"))
            payload["torque_nm"] = bridge.augment(action, observation)
            node.send_output("action", encode_message(payload), event.get("metadata"))
