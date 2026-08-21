"""Arrow-safe wire codec for versioned workflow messages."""

from __future__ import annotations

import json
from typing import Any

import pyarrow as pa

from dora_lerobot.contracts.models import Action, Observation


def encode_message(payload: dict[str, Any]) -> pa.Array:
    return pa.array([json.dumps(payload, separators=(",", ":"))])


def decode_message(value: Any) -> dict[str, Any]:
    scalar = value[0] if hasattr(value, "__getitem__") else value
    raw = scalar.as_py() if hasattr(scalar, "as_py") else scalar
    if not isinstance(raw, str):
        raise TypeError("Dora message payload must be a JSON string")
    parsed = json.loads(raw)
    if not isinstance(parsed, dict):
        raise TypeError("Dora message payload must decode to an object")
    return parsed


def observation_to_message(observation: Observation) -> pa.Array:
    return encode_message(
        {
            "schema_version": observation.schema_version,
            "timestamp_ns": observation.timestamp_ns,
            "joints_rad": dict(observation.joints_rad),
            "fault": observation.fault,
        }
    )


def message_to_observation(value: Any) -> Observation:
    payload = decode_message(value)
    return Observation(
        schema_version=payload["schema_version"],
        timestamp_ns=int(payload["timestamp_ns"]),
        joints_rad=payload["joints_rad"],
        fault=payload.get("fault"),
    )


def action_to_message(action: Action) -> pa.Array:
    return encode_message(
        {
            "schema_version": action.schema_version,
            "timestamp_ns": action.timestamp_ns,
            "joint_names": list(action.joint_names),
            "positions_rad": list(action.positions_rad),
            "control_mode": action.control_mode.value,
        }
    )


def message_to_action(value: Any) -> Action:
    from dora_lerobot.contracts.models import ControlMode

    payload = decode_message(value)
    return Action(
        schema_version=payload["schema_version"],
        timestamp_ns=int(payload["timestamp_ns"]),
        joint_names=tuple(payload["joint_names"]),
        positions_rad=tuple(float(item) for item in payload["positions_rad"]),
        control_mode=ControlMode(payload.get("control_mode", "position")),
    )
