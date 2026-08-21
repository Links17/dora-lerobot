"""Optional deadline-bound cloud inference with deterministic local fallback."""

from __future__ import annotations

import json
from collections.abc import Callable
from time import time_ns
from urllib.error import URLError
from urllib.request import Request, urlopen

from dora_lerobot.contracts.models import Action, Observation
from dora_lerobot.contracts.protocols import RobotAdapter


class CloudPolicyBridge:
    def __init__(
        self,
        adapter: RobotAdapter,
        endpoint: str,
        model_id: str,
        deadline_s: float,
        fallback: Callable[[Observation], Action],
    ) -> None:
        if not endpoint.startswith(("https://", "http://")):
            raise ValueError("endpoint must use HTTP(S)")
        if deadline_s <= 0:
            raise ValueError("deadline_s must be positive")
        self.adapter = adapter
        self.endpoint = endpoint
        self.model_id = model_id
        self.deadline_s = deadline_s
        self.fallback = fallback
        self.last_source = "uninitialized"

    def step(self, observation: Observation) -> Action:
        try:
            action = self._request(observation)
            self.last_source = "cloud"
        except (TimeoutError, URLError, ValueError, json.JSONDecodeError):
            action = self.fallback(observation)
            self.last_source = "fallback"
        return self.adapter.apply_action(action)

    def _request(self, observation: Observation) -> Action:
        payload = {
            "model_id": self.model_id,
            "schema_version": observation.schema_version,
            "timestamp_ns": observation.timestamp_ns,
            "joints_rad": dict(observation.joints_rad),
        }
        request = Request(
            self.endpoint,
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request, timeout=self.deadline_s) as response:
            result = json.loads(response.read())
        if (
            result.get("model_id") != self.model_id
            or result.get("schema_version") != observation.schema_version
        ):
            raise ValueError("cloud policy response identity mismatch")
        return Action(
            joint_names=tuple(result["joint_names"]),
            positions_rad=tuple(float(value) for value in result["positions_rad"]),
            timestamp_ns=int(result.get("timestamp_ns", time_ns())),
        )
