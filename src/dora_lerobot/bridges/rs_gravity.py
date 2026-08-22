"""Pure software bridge from observations to bounded RS torque feed-forward."""

from collections.abc import Callable, Sequence
from math import isfinite

from dora_lerobot.contracts.models import Action, Observation

RS_ARM_JOINTS = (
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"
)


class RsGravityBridge:
    def __init__(self, compute: Callable[[Sequence[float]], Sequence[float]], *, torque_limits: Sequence[float] | None = None) -> None:
        self._compute = compute
        self._limits = tuple(torque_limits or (14.0, 14.0, 14.0, 14.0, 14.0, 14.0))
        if len(self._limits) != 6 or any(not isfinite(v) or v <= 0 for v in self._limits):
            raise ValueError("torque_limits must contain six positive finite values")

    def augment(self, action: Action, observation: Observation) -> list[float]:
        q = [float(observation.joints_rad[name]) for name in RS_ARM_JOINTS]
        raw = list(self._compute(q))
        if len(raw) != 6 or any(not isfinite(value) for value in raw):
            raise ValueError("gravity computation must return six finite torques")
        return [max(-limit, min(limit, value)) for value, limit in zip(raw, self._limits, strict=True)] + [0.0]


def pure_python_rs_gravity(urdf_path: str) -> RsGravityBridge:
    from lerobot_robot_seeed_b601.gravity import compute_generalized_gravity, load_dynamics_model

    model = load_dynamics_model(urdf_path)
    return RsGravityBridge(lambda q: compute_generalized_gravity(model=model, q=q)[:6])
